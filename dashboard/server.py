#!/usr/bin/env python3
"""Localhost dashboard server for SerBot control and map viewing."""

import argparse
import ast
import base64
import collections
import json
import math
import mimetypes
import os
import pathlib
import re
import shlex
import subprocess
import tempfile
import threading
import time
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


APP_ROOT = pathlib.Path(__file__).resolve().parent
STATIC_DIR = APP_ROOT / "static"
MAX_LOG_ENTRIES = 40
MAX_DASHBOARD_SPEED = 10
DEFAULT_CAMERA_STREAM_PATH = "/home/soda/ros_ws/tools/camera_stream_server.py"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def generate_demo_map(width=72, height=48, resolution=0.08):
    data = [0] * (width * height)

    def fill_rect(x0, y0, x1, y1, value):
        for y in range(y0, y1):
            for x in range(x0, x1):
                if 0 <= x < width and 0 <= y < height:
                    data[y * width + x] = value

    fill_rect(0, 0, width, 2, 100)
    fill_rect(0, height - 2, width, height, 100)
    fill_rect(0, 0, 2, height, 100)
    fill_rect(width - 2, 0, width, height, 100)

    fill_rect(12, 8, 18, 26, 100)
    fill_rect(24, 16, 30, 44, 100)
    fill_rect(38, 4, 44, 28, 100)
    fill_rect(48, 20, 64, 26, 100)
    fill_rect(54, 30, 60, 44, 100)

    fill_rect(4, 32, 14, 36, -1)
    fill_rect(44, 34, 50, 40, -1)

    return {
        "name": "데모 창고 루프",
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": [0.0, 0.0, 0.0],
        "frame_id": "map",
        "data": data,
    }


class SshRuntimeBridge:
    def __init__(
        self,
        mode="mock",
        target="",
        password="",
        runtime_path="",
        camera_stream_url="",
        camera_stream_path=DEFAULT_CAMERA_STREAM_PATH,
    ):
        self.mode = mode
        self.target = target
        self.password = password
        self.runtime_path = runtime_path
        self.camera_stream_url = camera_stream_url
        self.camera_stream_path = camera_stream_path
        self.last_error = None
        self.last_camera_error = None
        self.last_camera_at = None
        self.last_ultra_error = None
        self.last_ultra_at = None

    def as_dict(self):
        return {
            "mode": self.mode,
            "target": self.target,
            "runtime_path": self.runtime_path,
            "camera_stream_url": self._effective_camera_stream_url(),
            "camera_stream_path": self.camera_stream_path,
            "configured": bool(self.target and self.runtime_path),
            "last_error": self.last_error,
            "last_camera_error": self.last_camera_error,
            "last_camera_at": self.last_camera_at,
            "last_ultra_error": self.last_ultra_error,
            "last_ultra_at": self.last_ultra_at,
            "password_set": bool(self.password),
        }

    def update(self, payload):
        self.mode = payload.get("mode", self.mode)
        self.target = payload.get("target", self.target)
        self.password = payload.get("password", self.password)
        self.runtime_path = payload.get("runtime_path", self.runtime_path)
        self.camera_stream_url = payload.get("camera_stream_url", self.camera_stream_url)
        self.camera_stream_path = payload.get("camera_stream_path", self.camera_stream_path)
        return self.as_dict()

    def _effective_camera_stream_url(self):
        if self.camera_stream_url:
            return self.camera_stream_url
        if not self.target:
            return ""
        host = self.target.rsplit("@", 1)[-1]
        if ":" in host and not host.startswith("["):
            host = host.split(":", 1)[0]
        return "http://%s:8766/stream.mjpg" % host.strip("[]")

    def _ssh_base(self):
        base = [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "ConnectTimeout=4",
            self.target,
        ]
        if self.password:
            base = ["sshpass", "-p", self.password] + base
        return base

    @staticmethod
    def _remote_python_command(script):
        encoded = base64.b64encode(script.encode("utf-8")).decode("ascii")
        return [
            "python3",
            "-c",
            "import base64; exec(base64.b64decode(%r).decode('utf-8'))" % encoded,
        ]

    def _run_remote_text(self, remote_parts, timeout=10.0):
        cmd = self._ssh_base() + [" ".join(shlex.quote(part) for part in remote_parts)]
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    def _run_remote_binary(self, remote_parts, timeout=12.0):
        cmd = self._ssh_base() + [" ".join(shlex.quote(part) for part in remote_parts)]
        return subprocess.run(cmd, capture_output=True, timeout=timeout)

    def _scp_to(self, local_path, remote_path, timeout=20.0):
        if not self.target:
            raise RuntimeError("SSH target is missing")
        base = [
            "scp",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "ConnectTimeout=4",
            str(local_path),
            "%s:%s" % (self.target, remote_path),
        ]
        if self.password:
            base = ["sshpass", "-p", self.password] + base
        result = subprocess.run(base, capture_output=True, text=True, timeout=timeout)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "scp failed")

    def deploy_camera_stream_server(self):
        if self.mode != "ssh":
            return {"mode": "mock", "deployed": False}
        if not self.target or not self.camera_stream_path:
            raise RuntimeError("SSH target or camera stream path is missing")
        local = APP_ROOT.parent / "tools" / "camera_stream_server.py"
        if not local.exists():
            raise RuntimeError("local camera stream server is missing: %s" % local)
        remote_dir = str(pathlib.PurePosixPath(self.camera_stream_path).parent)
        self._run_remote_text(["mkdir", "-p", remote_dir], timeout=8.0)
        self._scp_to(local, self.camera_stream_path, timeout=25.0)
        result = self._run_remote_text(["chmod", "+x", self.camera_stream_path], timeout=8.0)
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout).strip() or "chmod failed")
        return {"mode": "ssh", "deployed": True, "remote": self.camera_stream_path}

    def camera_stream_url_for(self, port=8766):
        if self.camera_stream_url:
            return self.camera_stream_url
        if not self.target:
            return ""
        host = self.target.rsplit("@", 1)[-1]
        if ":" in host and not host.startswith("["):
            host = host.split(":", 1)[0]
        return "http://%s:%s/stream.mjpg" % (host.strip("[]"), int(port))

    def stop_camera_stream(self, port=8766):
        if self.mode != "ssh":
            return {"mode": "mock", "running": False, "port": int(port)}
        if not self.target:
            raise RuntimeError("SSH target is missing")
        pid_file = "/tmp/serbot_camera_stream_%s.pid" % int(port)
        script_path = self.camera_stream_path
        script = """
import os
import signal
import subprocess
import time

pid_file = %r
script_path = %r
current = os.getpid()
parent = os.getppid()
pids = set()

if os.path.exists(pid_file):
    try:
        text = open(pid_file).read().strip()
        if text:
            pids.add(int(text))
    except Exception:
        pass

try:
    output = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, _, args = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid in (current, parent):
            continue
        if script_path in args and "camera_stream_server.py" in args:
            pids.add(pid)
except Exception:
    pass

for pid in sorted(pids):
    try:
        os.kill(pid, signal.SIGTERM)
    except Exception:
        pass
time.sleep(0.35)
for pid in sorted(pids):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        continue
    except Exception:
        continue
    try:
        os.kill(pid, signal.SIGKILL)
    except Exception:
        pass
try:
    os.remove(pid_file)
except FileNotFoundError:
    pass
print("stopped", sorted(pids))
""" % (pid_file, script_path)
        result = self._run_remote_text(self._remote_python_command(script), timeout=8.0)
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout).strip() or "camera stream stop failed")
        return {"mode": "ssh", "running": False, "port": int(port), "stdout": result.stdout.strip()}

    def camera_stream_status(self, port=8766):
        if self.mode != "ssh":
            return {"mode": "mock", "running": False, "port": int(port), "url": self.camera_stream_url_for(port)}
        pid_file = "/tmp/serbot_camera_stream_%s.pid" % int(port)
        log_file = "/tmp/serbot_camera_stream_%s.log" % int(port)
        script = """
import json
import os
import subprocess

pid_file = %r
log_file = %r
pid = None
running = False
if os.path.exists(pid_file):
    pid = open(pid_file).read().strip() or None
    if pid:
        running = subprocess.call(["kill", "-0", pid], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
tail = ""
if os.path.exists(log_file):
    with open(log_file, "r", errors="ignore") as handle:
        tail = "".join(handle.readlines()[-8:])
print(json.dumps({"running": running, "pid": pid, "log_tail": tail}, ensure_ascii=False))
""" % (pid_file, log_file)
        result = self._run_remote_text(self._remote_python_command(script), timeout=8.0)
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout).strip() or "camera stream status failed")
        payload = json.loads(result.stdout.strip().splitlines()[-1])
        payload.update({"mode": "ssh", "port": int(port), "url": self.camera_stream_url_for(port)})
        return payload

    def start_camera_stream(self, port=8766, width=640, height=360, fps=10.0, quality=70, deploy=True):
        if self.mode != "ssh":
            return {"mode": "mock", "running": False, "port": int(port), "url": self.camera_stream_url_for(port)}
        if not self.target or not self.camera_stream_path:
            raise RuntimeError("SSH target or camera stream path is missing")
        if deploy:
            self.deploy_camera_stream_server()
        self.stop_camera_stream(port=port)
        pid_file = "/tmp/serbot_camera_stream_%s.pid" % int(port)
        log_file = "/tmp/serbot_camera_stream_%s.log" % int(port)
        remote = (
            "nohup python3 {script} --host 0.0.0.0 --port {port} --device /dev/video0 "
            "--width {width} --height {height} --fps {fps} --quality {quality} "
            ">{log} 2>&1 & echo $! > {pid}"
        ).format(
            script=shlex.quote(self.camera_stream_path),
            port=int(port),
            width=int(width),
            height=int(height),
            fps=float(fps),
            quality=int(quality),
            log=shlex.quote(log_file),
            pid=shlex.quote(pid_file),
        )
        result = self._run_remote_text(["bash", "-lc", remote], timeout=10.0)
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout).strip() or "camera stream start failed")
        time.sleep(0.35)
        status = self.camera_stream_status(port=port)
        self.camera_stream_url = status["url"]
        return status

    def send_command(self, direction, speed, duration):
        if self.mode != "ssh":
            return {"mode": "mock", "stdout": "", "stderr": ""}
        if not self.target or not self.runtime_path:
            raise RuntimeError("SSH bridge is missing target or runtime path")

        if direction == "stop":
            remote = ["python3", self.runtime_path, "stop"]
        else:
            remote = [
                "python3",
                self.runtime_path,
                "--ultra-backend",
                "pop",
                "move",
                direction,
                "--speed",
                str(speed),
                "--seconds",
                str(duration),
            ]

        result = self._run_remote_text(remote, timeout=max(8.0, duration + 6.0))
        if result.returncode != 0:
            self.last_error = (result.stderr or result.stdout).strip() or "bridge failure"
            raise RuntimeError(self.last_error)
        self.last_error = None
        return {
            "mode": "ssh",
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }

    def capture_camera_frame(self):
        if self.mode != "ssh":
            return None, "mock mode does not pull a remote camera frame"
        if not self.target:
            return None, "SSH target is missing"

        script = """
import sys
try:
    import cv2
except Exception as exc:
    sys.stderr.write("opencv import failed: %s" % exc)
    sys.exit(2)

candidates = [
    ('/dev/video0', getattr(cv2, 'CAP_V4L2', 200)),
    (0, getattr(cv2, 'CAP_V4L2', 200)),
    ('/dev/video1', getattr(cv2, 'CAP_V4L2', 200)),
]
frame = None
errors = []
for candidate, backend in candidates:
    cap = cv2.VideoCapture(candidate, backend)
    if cap is None or not cap.isOpened():
        errors.append("%s:open" % candidate)
        if cap is not None:
            cap.release()
        continue
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    ok, image = cap.read()
    cap.release()
    if ok and image is not None:
        frame = image
        break
    errors.append("%s:read" % candidate)
if frame is None:
    sys.stderr.write("camera unavailable: " + ", ".join(errors))
    sys.exit(3)
ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
if not ok:
    sys.stderr.write("jpeg encode failed")
    sys.exit(4)
sys.stdout.buffer.write(encoded.tobytes())
"""
        try:
            result = self._run_remote_binary(
                self._remote_python_command(script),
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            error = "camera capture timed out"
            self.last_camera_error = error
            return None, error
        if result.returncode != 0 or not result.stdout:
            error = (result.stderr or b"").decode("utf-8", "ignore").strip() or "camera frame capture failed"
            self.last_camera_error = error
            return None, error

        self.last_camera_error = None
        self.last_camera_at = round(time.time(), 3)
        return result.stdout, None

    @staticmethod
    def _extract_runtime_value(text, key):
        match = re.search(r"%s=([^=]+?)(?= [a-z_]+=|$)" % re.escape(key), text)
        if not match:
            return None
        return match.group(1).strip()

    @classmethod
    def _parse_runtime_sample(cls, stdout_text):
        line = stdout_text.strip().splitlines()[-1]
        raw_text = cls._extract_runtime_value(line, "raw_cm")
        backend = cls._extract_runtime_value(line, "backend") or "unknown"
        front_min = cls._extract_runtime_value(line, "front_min")
        rear_min = cls._extract_runtime_value(line, "rear_min")
        raw_values = ast.literal_eval(raw_text) if raw_text else []
        return {
            "backend": backend,
            "raw_cm": list(raw_values),
            "front_min_cm": float(front_min) if front_min is not None else None,
            "rear_min_cm": float(rear_min) if rear_min is not None else None,
            "timestamp": round(time.time(), 3),
        }

    def read_ultrasonic(self):
        if self.mode != "ssh":
            return None, "mock mode does not pull remote ultrasonic data"
        if not self.target or not self.runtime_path:
            return None, "SSH bridge is missing target or runtime path"

        remote = [
            "python3",
            self.runtime_path,
            "--ultra-backend",
            "pop",
            "ultra",
            "--count",
            "1",
            "--timeout",
            "0.45",
            "--interval",
            "0.0",
        ]
        try:
            result = self._run_remote_text(remote, timeout=8.0)
        except subprocess.TimeoutExpired:
            error = "ultrasonic read timed out"
            self.last_ultra_error = error
            return None, error
        if result.returncode != 0 or not result.stdout.strip():
            error = (result.stderr or result.stdout).strip() or "ultrasonic read failed"
            self.last_ultra_error = error
            return None, error

        try:
            sample = self._parse_runtime_sample(result.stdout)
        except Exception as exc:
            error = "failed to parse ultrasonic sample: %s" % exc
            self.last_ultra_error = error
            return None, error

        self.last_ultra_error = None
        self.last_ultra_at = round(time.time(), 3)
        return sample, None


class DashboardState:
    def __init__(self, bridge):
        self.lock = threading.Lock()
        self.bridge = bridge
        self.start_time = time.time()
        self.map_data = generate_demo_map()
        self.path = collections.deque(maxlen=320)
        self.command_log = collections.deque(maxlen=MAX_LOG_ENTRIES)
        self.connection_status = "mock-ready"
        self.last_bridge_error = None
        self.speed = 6
        self.duration = 0.8
        self.last_telemetry_refresh = 0.0
        self._ultra_poll_interval = 8.0
        self._camera_poll_interval = 2.2
        self.camera_frame = None
        self.camera_content_type = "image/jpeg"
        self.camera_error = "카메라 연결 대기 중"
        self.pose = self._spawn_pose()
        self.path.append(self._pose_point())
        self.telemetry = {}
        self.refresh_telemetry()
        self._start_ultra_worker()
        self._start_camera_worker()
        self._log("system", "%s 모드로 대시보드 준비 완료" % bridge.mode)

    def update_bridge(self, payload):
        updated = self.bridge.update(payload)
        self._log("bridge", "브리지 갱신: %s -> %s" % (updated["mode"], updated["target"] or "미설정"))
        return updated

    def _spawn_pose(self):
        res = self.map_data["resolution"]
        return {"x": 5.5 * res, "y": 6.0 * res, "heading": 0.0}

    def _pose_point(self):
        return {"x": round(self.pose["x"], 4), "y": round(self.pose["y"], 4)}

    def _log(self, kind, message):
        self.command_log.appendleft(
            {
                "timestamp": round(time.time(), 3),
                "kind": kind,
                "message": message,
            }
        )

    def _map_dimensions(self):
        return self.map_data["width"], self.map_data["height"], self.map_data["resolution"]

    def _world_to_cell(self, x_m, y_m):
        width, height, res = self._map_dimensions()
        x = int(x_m / res)
        y = int(y_m / res)
        return clamp(x, 0, width - 1), clamp(y, 0, height - 1)

    def _cell_occupied(self, x_m, y_m):
        width, height, _ = self._map_dimensions()
        cx, cy = self._world_to_cell(x_m, y_m)
        value = self.map_data["data"][cy * width + cx]
        return value >= 65

    def _raycast_cm(self, angle, max_range_m=2.2, step_m=0.04):
        width, height, _ = self._map_dimensions()
        x = self.pose["x"]
        y = self.pose["y"]
        dist = step_m
        while dist <= max_range_m:
            px = x + math.cos(angle) * dist
            py = y + math.sin(angle) * dist
            cx, cy = self._world_to_cell(px, py)
            if cx <= 0 or cy <= 0 or cx >= width - 1 or cy >= height - 1:
                return round(dist * 100.0, 1)
            if self._cell_occupied(px, py):
                return round(dist * 100.0, 1)
            dist += step_m
        return round(max_range_m * 100.0, 1)

    def _start_ultra_worker(self):
        worker = threading.Thread(target=self._ultra_worker, daemon=True)
        worker.start()

    def _start_camera_worker(self):
        worker = threading.Thread(target=self._camera_worker, daemon=True)
        worker.start()

    def _apply_remote_sample(self, sample):
        raw = sample.get("raw_cm", [])

        def pick(index, fallback):
            if index < len(raw):
                try:
                    return float(raw[index])
                except (TypeError, ValueError):
                    return fallback
            return fallback

        battery = self.telemetry.get("battery_percent", 84.0)
        self.telemetry = {
            "front_left_cm": pick(0, sample.get("front_min_cm", 0.0) or 0.0),
            "front_right_cm": pick(1, sample.get("front_min_cm", 0.0) or 0.0),
            "left_cm": pick(2, 0.0),
            "right_cm": pick(3, 0.0),
            "rear_left_cm": pick(4, sample.get("rear_min_cm", 0.0) or 0.0),
            "rear_right_cm": pick(5, sample.get("rear_min_cm", 0.0) or 0.0),
            "front_min_cm": sample.get("front_min_cm") or 0.0,
            "rear_min_cm": sample.get("rear_min_cm") or 0.0,
            "battery_percent": round(float(battery), 1),
            "mode": "ssh-live",
            "updated_at": sample.get("timestamp", round(time.time(), 3)),
        }
        self.connection_status = "ssh-live"
        self.last_bridge_error = None
        self.last_telemetry_refresh = time.time()

    def _ultra_worker(self):
        while True:
            time.sleep(self._ultra_poll_interval)
            if not (self.bridge.mode == "ssh" and self.bridge.target and self.bridge.runtime_path):
                continue
            sample, error = self.bridge.read_ultrasonic()
            with self.lock:
                if sample:
                    self._apply_remote_sample(sample)

    def _camera_worker(self):
        while True:
            time.sleep(self._camera_poll_interval)
            if not (self.bridge.mode == "ssh" and self.bridge.target):
                with self.lock:
                    self.camera_error = "카메라 브리지가 아직 설정되지 않았습니다"
                continue
            frame, error = self.bridge.capture_camera_frame()
            with self.lock:
                if frame:
                    self.camera_frame = frame
                    self.camera_error = None
                elif error:
                    self.camera_error = error

    def refresh_telemetry(self, force=False):
        if not force and (time.time() - self.last_telemetry_refresh) < 0.7:
            return

        heading = self.pose["heading"]
        front_left = self._raycast_cm(heading + math.radians(18))
        front_right = self._raycast_cm(heading - math.radians(18))
        left_side = self._raycast_cm(heading + math.radians(90))
        right_side = self._raycast_cm(heading - math.radians(90))
        rear_left = self._raycast_cm(heading + math.radians(180 + 18))
        rear_right = self._raycast_cm(heading + math.radians(180 - 18))
        battery = 84.0 - min(12.0, (time.time() - self.start_time) / 300.0)
        self.telemetry = {
            "front_left_cm": front_left,
            "front_right_cm": front_right,
            "left_cm": left_side,
            "right_cm": right_side,
            "rear_left_cm": rear_left,
            "rear_right_cm": rear_right,
            "front_min_cm": min(front_left, front_right),
            "rear_min_cm": min(rear_left, rear_right),
            "battery_percent": round(battery, 1),
            "mode": self.bridge.mode,
            "updated_at": round(time.time(), 3),
        }
        self.last_telemetry_refresh = time.time()

    def _simulate_motion(self, direction, speed, duration):
        dt = 0.05
        steps = max(1, int(duration / dt))
        linear_speed = 0.42 * (speed / float(MAX_DASHBOARD_SPEED))
        angular_speed = 1.45 * (speed / float(MAX_DASHBOARD_SPEED))
        stopped_by_collision = False

        for _ in range(steps):
            if direction == "forward":
                dx = math.cos(self.pose["heading"]) * linear_speed * dt
                dy = math.sin(self.pose["heading"]) * linear_speed * dt
                nx = self.pose["x"] + dx
                ny = self.pose["y"] + dy
                if self._cell_occupied(nx, ny):
                    stopped_by_collision = True
                    break
                self.pose["x"] = nx
                self.pose["y"] = ny
            elif direction == "backward":
                dx = math.cos(self.pose["heading"]) * linear_speed * dt
                dy = math.sin(self.pose["heading"]) * linear_speed * dt
                nx = self.pose["x"] - dx
                ny = self.pose["y"] - dy
                if self._cell_occupied(nx, ny):
                    stopped_by_collision = True
                    break
                self.pose["x"] = nx
                self.pose["y"] = ny
            elif direction == "spin-left":
                self.pose["heading"] += angular_speed * dt
            elif direction == "spin-right":
                self.pose["heading"] -= angular_speed * dt

            self.pose["heading"] = (self.pose["heading"] + math.pi * 2.0) % (math.pi * 2.0)
            self.path.append(self._pose_point())

        return stopped_by_collision

    def apply_control(self, direction, speed, duration):
        speed = int(clamp(speed, 0, MAX_DASHBOARD_SPEED))
        duration = float(clamp(duration, 0.1, 12.0))
        self.speed = speed
        self.duration = duration

        bridge_result = None
        bridge_error = None
        if direction == "stop":
            self.connection_status = "stop-requested"
        else:
            self.connection_status = "command-sent"

        try:
            bridge_result = self.bridge.send_command(direction, speed, duration)
        except Exception as exc:
            bridge_error = str(exc)
            self.last_bridge_error = bridge_error
            self.connection_status = "bridge-error"

        collision = False
        if direction != "stop":
            collision = self._simulate_motion(direction, speed, duration)
        self.refresh_telemetry()

        label = "%s speed=%s duration=%.1fs" % (direction, speed, duration)
        if collision:
            label += " blocked-by-map"
        self._log("control", label)
        if bridge_error:
            self._log("bridge", bridge_error)
        elif bridge_result and bridge_result["mode"] == "ssh":
            self._log("bridge", "SSH 브리지를 통해 원격 명령 실행")

        return {
            "ok": bridge_error is None,
            "direction": direction,
            "speed": speed,
            "duration": duration,
            "collision": collision,
            "bridge": bridge_result or self.bridge.as_dict(),
            "bridge_error": bridge_error,
            "status": self.connection_status,
        }

    def import_map(self, payload):
        if "info" in payload and "data" in payload:
            info = payload["info"]
            width = int(info["width"])
            height = int(info["height"])
            resolution = float(info["resolution"])
            origin = info.get("origin", {})
            position = origin.get("position", {})
            yaw = float(origin.get("yaw", 0.0))
            grid = {
                "name": payload.get("name", "불러온 OccupancyGrid"),
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": [
                    float(position.get("x", 0.0)),
                    float(position.get("y", 0.0)),
                    yaw,
                ],
                "frame_id": payload.get("header", {}).get("frame_id", "map"),
                "data": payload["data"],
            }
        else:
            grid = {
                "name": payload.get("name", "불러온 맵"),
                "width": int(payload["width"]),
                "height": int(payload["height"]),
                "resolution": float(payload["resolution"]),
                "origin": payload.get("origin", [0.0, 0.0, 0.0]),
                "frame_id": payload.get("frame_id", "map"),
                "data": payload["data"],
            }

        total = grid["width"] * grid["height"]
        if len(grid["data"]) != total:
            raise ValueError("map data length does not match width*height")

        normalized = []
        for value in grid["data"]:
            if value is None:
                normalized.append(-1)
                continue
            ivalue = int(value)
            if ivalue < -1 or ivalue > 100:
                raise ValueError("occupancy values must be between -1 and 100")
            normalized.append(ivalue)
        grid["data"] = normalized

        self.map_data = grid
        self.pose = self._spawn_pose()
        self.path.clear()
        self.path.append(self._pose_point())
        self.refresh_telemetry()
        self._log("map", "맵 불러오기: %s" % grid["name"])
        return {"ok": True, "name": grid["name"]}

    def reset_demo(self):
        self.map_data = generate_demo_map()
        self.pose = self._spawn_pose()
        self.path.clear()
        self.path.append(self._pose_point())
        self.refresh_telemetry()
        self._log("map", "데모 맵으로 초기화")
        return {"ok": True, "name": self.map_data["name"]}

    def _code_env_snapshot(self):
        env = os.environ.copy()
        workspace = str(APP_ROOT.parent)
        env["PYTHONPATH"] = workspace + os.pathsep + env.get("PYTHONPATH", "")
        env["SERBOT_TARGET"] = self.bridge.target or env.get("SERBOT_TARGET", "soda@threnody.local")
        env["SERBOT_PASSWORD"] = self.bridge.password or env.get("SERBOT_PASSWORD", "soda")
        env["SERBOT_RUNTIME"] = self.bridge.runtime_path or env.get("SERBOT_RUNTIME", "/home/soda/ros_ws/tools/serbot_runtime.py")
        env["SERBOT_CAMERA_STREAM"] = self.bridge.camera_stream_path or env.get("SERBOT_CAMERA_STREAM", DEFAULT_CAMERA_STREAM_PATH)
        env["SERBOT_ROS_DISTRO"] = env.get("SERBOT_ROS_DISTRO", "foxy")
        env["SERBOT_DASHBOARD_SAFE_MAX_SPEED"] = str(MAX_DASHBOARD_SPEED)
        return env

    def run_code(self, code, timeout=20.0):
        if not isinstance(code, str) or not code.strip():
            raise ValueError("실행할 코드가 비어 있습니다")
        if len(code.encode("utf-8")) > 64_000:
            raise ValueError("코드가 너무 깁니다")
        timeout = float(clamp(float(timeout or 20.0), 1.0, 30.0))
        artifacts = APP_ROOT.parent / "artifacts" / "code"
        artifacts.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            suffix=".py",
            prefix="dashboard-",
            dir=artifacts,
            delete=False,
        ) as handle:
            script_path = pathlib.Path(handle.name)
            handle.write(code)
            if not code.endswith("\n"):
                handle.write("\n")

        started = time.time()
        try:
            result = subprocess.run(
                ["python3", str(script_path)],
                cwd=str(APP_ROOT.parent),
                env=self._code_env_snapshot(),
                text=True,
                capture_output=True,
                timeout=timeout,
            )
            elapsed_ms = int((time.time() - started) * 1000)
            payload = {
                "ok": result.returncode == 0,
                "returncode": result.returncode,
                "stdout": result.stdout,
                "stderr": result.stderr,
                "elapsed_ms": elapsed_ms,
                "script": str(script_path),
                "timeout": False,
            }
        except subprocess.TimeoutExpired as exc:
            elapsed_ms = int((time.time() - started) * 1000)
            stdout = exc.stdout or ""
            stderr = exc.stderr or ""
            if isinstance(stdout, bytes):
                stdout = stdout.decode("utf-8", "ignore")
            if isinstance(stderr, bytes):
                stderr = stderr.decode("utf-8", "ignore")
            payload = {
                "ok": False,
                "returncode": -9,
                "stdout": stdout,
                "stderr": stderr + ("\n" if stderr else "") + "실행 시간이 초과되어 중단했습니다.",
                "elapsed_ms": elapsed_ms,
                "script": str(script_path),
                "timeout": True,
            }
        self._log("code", "코드 실행 %s · %sms" % ("성공" if payload["ok"] else "실패", payload["elapsed_ms"]))
        return payload

    def start_camera_stream(self, payload):
        width = int(clamp(int(payload.get("width", 640)), 160, 1280))
        height = int(clamp(int(payload.get("height", 360)), 120, 720))
        fps = float(clamp(float(payload.get("fps", 10.0)), 1.0, 30.0))
        quality = int(clamp(int(payload.get("quality", 70)), 35, 92))
        port = int(clamp(int(payload.get("port", 8766)), 1024, 65535))
        deploy = bool(payload.get("deploy", True))
        status = self.bridge.start_camera_stream(
            port=port,
            width=width,
            height=height,
            fps=fps,
            quality=quality,
            deploy=deploy,
        )
        self._log("camera", "MJPEG 스트림 시작: %sx%s %.0ffps q%s" % (width, height, fps, quality))
        return status

    def stop_camera_stream(self, payload):
        port = int(clamp(int(payload.get("port", 8766)), 1024, 65535))
        status = self.bridge.stop_camera_stream(port=port)
        self._log("camera", "MJPEG 스트림 중지")
        return status

    def camera_stream_status(self, port=8766):
        return self.bridge.camera_stream_status(port=port)

    def status_snapshot(self):
        self.refresh_telemetry()
        return {
            "connection": {
                "state": self.connection_status,
                "bridge": self.bridge.as_dict(),
                "last_bridge_error": self.last_bridge_error,
            },
            "control": {
                "speed": self.speed,
                "duration": self.duration,
            },
            "robot": {
                "pose": {
                    "x": round(self.pose["x"], 3),
                    "y": round(self.pose["y"], 3),
                    "heading": round(self.pose["heading"], 3),
                },
                "telemetry": self.telemetry,
            },
            "map": {
                "name": self.map_data["name"],
                "frame_id": self.map_data["frame_id"],
                "width": self.map_data["width"],
                "height": self.map_data["height"],
                "resolution": self.map_data["resolution"],
            },
            "logs": list(self.command_log),
            "uptime_sec": round(time.time() - self.start_time, 1),
        }

    def map_snapshot(self):
        self.refresh_telemetry()
        return {
            "map": self.map_data,
            "pose": {
                "x": round(self.pose["x"], 3),
                "y": round(self.pose["y"], 3),
                "heading": round(self.pose["heading"], 3),
            },
            "path": list(self.path),
            "telemetry": self.telemetry,
        }


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = "SerBotDashboard/1.0"

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/status":
            with self.server.state.lock:
                payload = self.server.state.status_snapshot()
            return self._write_json(payload)
        if parsed.path == "/api/map":
            with self.server.state.lock:
                payload = self.server.state.map_snapshot()
            return self._write_json(payload)
        if parsed.path == "/api/camera/frame":
            return self._serve_camera_frame()
        if parsed.path == "/api/camera/stream":
            query = urllib.parse.parse_qs(parsed.query)
            port = int(query.get("port", ["8766"])[0])
            with self.server.state.lock:
                payload = self.server.state.camera_stream_status(port=port)
            return self._write_json(payload)
        return self._serve_static(parsed.path)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/control":
            payload = self._read_json()
            with self.server.state.lock:
                response = self.server.state.apply_control(
                    payload.get("direction", "stop"),
                    payload.get("speed", self.server.state.speed),
                    payload.get("duration", self.server.state.duration),
                )
            status = HTTPStatus.OK if response["ok"] else HTTPStatus.BAD_GATEWAY
            return self._write_json(response, status=status)

        if parsed.path == "/api/map/import":
            payload = self._read_json()
            with self.server.state.lock:
                response = self.server.state.import_map(payload)
            return self._write_json(response)

        if parsed.path == "/api/reset":
            with self.server.state.lock:
                response = self.server.state.reset_demo()
            return self._write_json(response)

        if parsed.path == "/api/bridge":
            payload = self._read_json()
            with self.server.state.lock:
                response = self.server.state.update_bridge(payload)
            return self._write_json(response)

        if parsed.path == "/api/camera/stream/start":
            payload = self._read_json()
            with self.server.state.lock:
                response = self.server.state.start_camera_stream(payload)
            return self._write_json(response)

        if parsed.path == "/api/camera/stream/stop":
            payload = self._read_json()
            with self.server.state.lock:
                response = self.server.state.stop_camera_stream(payload)
            return self._write_json(response)

        if parsed.path == "/api/code/run":
            payload = self._read_json()
            response = self.server.state.run_code(
                payload.get("code", ""),
                timeout=payload.get("timeout", 20.0),
            )
            status = HTTPStatus.OK if response["ok"] else HTTPStatus.BAD_REQUEST
            return self._write_json(response, status=status)

        self.send_error(HTTPStatus.NOT_FOUND, "Unsupported endpoint")

    def _serve_static(self, request_path):
        relative = request_path.lstrip("/") or "index.html"
        target = (STATIC_DIR / relative).resolve()
        if not str(target).startswith(str(STATIC_DIR.resolve())):
            self.send_error(HTTPStatus.FORBIDDEN, "Invalid path")
            return
        if target.is_dir():
            target = target / "index.html"
        if not target.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "Missing asset")
            return

        content_type, _ = mimetypes.guess_type(str(target))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(target.read_bytes())

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length) if length else b"{}"
        try:
            return json.loads(body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError("invalid JSON body") from exc

    def _write_json(self, payload, status=HTTPStatus.OK):
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _serve_camera_frame(self):
        with self.server.state.lock:
            frame = self.server.state.camera_frame
            error = self.server.state.camera_error or self.server.state.bridge.last_camera_error
        if frame:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", self.server.state.camera_content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return

        placeholder = self._camera_placeholder_svg(error or "카메라를 사용할 수 없습니다")
        encoded = placeholder.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    @staticmethod
    def _camera_placeholder_svg(message):
        safe = (
            message.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
        )
        return """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">
<defs>
  <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
    <stop offset="0%%" stop-color="#10181a"/>
    <stop offset="100%%" stop-color="#212b2e"/>
  </linearGradient>
</defs>
<rect width="960" height="540" fill="url(#bg)"/>
<rect x="36" y="36" width="888" height="468" rx="24" fill="none" stroke="#5f7276" stroke-width="2" stroke-dasharray="10 10"/>
<text x="64" y="108" fill="#f4eee6" font-family="Pretendard Variable, Apple SD Gothic Neo, sans-serif" font-size="44">카메라 영상 대기 중</text>
<text x="64" y="160" fill="#f28f45" font-family="Pretendard Variable, Apple SD Gothic Neo, sans-serif" font-size="22">SSH 브리지가 연결되고 /dev/video0 이 열리면 여기에 실시간 화면이 표시됩니다.</text>
<text x="64" y="228" fill="#d6d0c7" font-family="Avenir Next, Segoe UI, sans-serif" font-size="20">%s</text>
</svg>""" % safe

    def log_message(self, fmt, *args):
        return


class DashboardServer(ThreadingHTTPServer):
    def __init__(self, server_address, state):
        super().__init__(server_address, DashboardRequestHandler)
        self.state = state


def build_parser():
    parser = argparse.ArgumentParser(description="SerBot localhost dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--bridge-mode",
        default="mock",
        choices=["mock", "ssh"],
        help="mock keeps everything local; ssh forwards commands to serbot_runtime.py",
    )
    parser.add_argument("--ssh-target", default="")
    parser.add_argument("--ssh-password", default="")
    parser.add_argument("--runtime-path", default="/home/soda/ros_ws/tools/serbot_runtime.py")
    parser.add_argument("--camera-stream-url", default="")
    parser.add_argument("--camera-stream-path", default=DEFAULT_CAMERA_STREAM_PATH)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    bridge = SshRuntimeBridge(
        mode=args.bridge_mode,
        target=args.ssh_target,
        password=args.ssh_password,
        runtime_path=args.runtime_path,
        camera_stream_url=args.camera_stream_url,
        camera_stream_path=args.camera_stream_path,
    )
    state = DashboardState(bridge)
    server = DashboardServer((args.host, args.port), state)
    url = "http://%s:%s" % (args.host, args.port)
    print("SerBot dashboard listening on %s" % url)
    print("Bridge mode: %s" % bridge.mode)
    server.serve_forever()


if __name__ == "__main__":
    main()
