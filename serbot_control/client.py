"""Local SerBot client.

This module is intentionally local-first: code runs on the Mac, and robot-side
work is executed over SSH on the Jetson. The robot must already have
``/home/soda/ros_ws/tools/serbot_runtime.py`` deployed for motor and ultrasonic
commands.
"""

from __future__ import annotations

import ast
import base64
import json
import math
import os
import re
import shlex
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


DEFAULT_TARGET = "soda@threnody.local"
DEFAULT_PASSWORD = "soda"
DEFAULT_RUNTIME_PATH = "/home/soda/ros_ws/tools/serbot_runtime.py"
DEFAULT_CAMERA_STREAM_PATH = "/home/soda/ros_ws/tools/camera_stream_server.py"
DEFAULT_ROS_DISTRO = "foxy"
DEFAULT_SAFE_MAX_SPEED = 10
DEFAULT_MAX_MOTION_SECONDS = 10.0

VALID_DIRECTIONS = {"forward", "backward", "spin-left", "spin-right"}
DEFAULT_DISCOVERY_CANDIDATES = (
    "threnody.local",
    "revalue.local",
    "murks.local",
    "192.168.0.68",
    "192.168.0.7",
    "192.168.101.101",
)


class SerBotError(RuntimeError):
    """Raised when the local client cannot complete a robot command."""


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return int(value)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return float(value)


@dataclass
class RobotConfig:
    target: str = DEFAULT_TARGET
    password: str | None = DEFAULT_PASSWORD
    runtime_path: str = DEFAULT_RUNTIME_PATH
    camera_stream_path: str = DEFAULT_CAMERA_STREAM_PATH
    ros_distro: str = DEFAULT_ROS_DISTRO
    connect_timeout: int = 8
    safe_max_speed: int = DEFAULT_SAFE_MAX_SPEED
    max_motion_seconds: float = DEFAULT_MAX_MOTION_SECONDS
    allow_unsafe_speed: bool = False

    @classmethod
    def from_env(cls) -> "RobotConfig":
        return cls(
            target=os.environ.get("SERBOT_TARGET", DEFAULT_TARGET),
            password=os.environ.get("SERBOT_PASSWORD", DEFAULT_PASSWORD),
            runtime_path=os.environ.get("SERBOT_RUNTIME", DEFAULT_RUNTIME_PATH),
            camera_stream_path=os.environ.get("SERBOT_CAMERA_STREAM", DEFAULT_CAMERA_STREAM_PATH),
            ros_distro=os.environ.get("SERBOT_ROS_DISTRO", DEFAULT_ROS_DISTRO),
            connect_timeout=_env_int("SERBOT_CONNECT_TIMEOUT", 8),
            safe_max_speed=_env_int("SERBOT_SAFE_MAX_SPEED", DEFAULT_SAFE_MAX_SPEED),
            max_motion_seconds=_env_float("SERBOT_MAX_MOTION_SECONDS", DEFAULT_MAX_MOTION_SECONDS),
            allow_unsafe_speed=_env_bool("SERBOT_ALLOW_UNSAFE_SPEED", False),
        )

    def safe_dict(self) -> dict[str, Any]:
        return {
            "target": self.target,
            "password_set": bool(self.password),
            "runtime_path": self.runtime_path,
            "camera_stream_path": self.camera_stream_path,
            "ros_distro": self.ros_distro,
            "connect_timeout": self.connect_timeout,
            "safe_max_speed": self.safe_max_speed,
            "max_motion_seconds": self.max_motion_seconds,
            "allow_unsafe_speed": self.allow_unsafe_speed,
        }


@dataclass
class MotionStep:
    """One robot-side motion segment for precise sequence control."""

    seconds: float
    left: int | None = None
    right: int | None = None
    linear: float | None = None
    angular: float | None = None
    direction: str | None = None
    speed: int | None = None
    period: float | None = None
    guard_direction: str = "auto"
    guard_cm: float | None = None
    allow_blind: bool = False
    limit: int = 120

    @classmethod
    def wheels(
        cls,
        left: int,
        right: int,
        *,
        seconds: float,
        period: float | None = None,
        guard_direction: str = "auto",
        guard_cm: float | None = None,
        allow_blind: bool = False,
    ) -> "MotionStep":
        return cls(
            seconds=seconds,
            left=left,
            right=right,
            period=period,
            guard_direction=guard_direction,
            guard_cm=guard_cm,
            allow_blind=allow_blind,
        )

    @classmethod
    def vector(
        cls,
        linear: float,
        angular: float,
        *,
        seconds: float,
        period: float | None = None,
        guard_direction: str = "auto",
        guard_cm: float | None = None,
        allow_blind: bool = False,
        limit: int = 120,
    ) -> "MotionStep":
        return cls(
            seconds=seconds,
            linear=linear,
            angular=angular,
            period=period,
            guard_direction=guard_direction,
            guard_cm=guard_cm,
            allow_blind=allow_blind,
            limit=limit,
        )

    @classmethod
    def named(
        cls,
        direction: str,
        speed: int,
        *,
        seconds: float,
        period: float | None = None,
        guard_direction: str = "auto",
        guard_cm: float | None = None,
        allow_blind: bool = False,
    ) -> "MotionStep":
        return cls(
            seconds=seconds,
            direction=direction,
            speed=speed,
            period=period,
            guard_direction=guard_direction,
            guard_cm=guard_cm,
            allow_blind=allow_blind,
        )

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {"seconds": self.seconds}
        for key in (
            "left",
            "right",
            "linear",
            "angular",
            "direction",
            "speed",
            "period",
            "guard_direction",
            "guard_cm",
            "allow_blind",
            "limit",
        ):
            value = getattr(self, key)
            if value is not None:
                data[key] = value
        return data


@dataclass
class CommandResult:
    command: list[str]
    returncode: int
    stdout: str
    stderr: str

    def check(self) -> "CommandResult":
        if self.returncode != 0:
            message = self.stderr.strip() or self.stdout.strip() or "remote command failed"
            raise SerBotError(message)
        return self


class SshTransport:
    def __init__(self, config: RobotConfig):
        self.config = config

    def _base(self) -> list[str]:
        base = [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            f"ConnectTimeout={self.config.connect_timeout}",
            self.config.target,
        ]
        if self.config.password:
            return ["sshpass", "-p", self.config.password] + base
        return base

    def run_text(self, parts: Iterable[str], timeout: float = 12.0) -> CommandResult:
        remote = " ".join(shlex.quote(str(part)) for part in parts)
        cmd = self._base() + [remote]
        try:
            result = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
        except FileNotFoundError as exc:
            raise SerBotError(f"required command is missing: {exc.filename}") from exc
        except subprocess.TimeoutExpired as exc:
            raise SerBotError(f"remote command timed out after {timeout:.1f}s") from exc
        return CommandResult(cmd, result.returncode, result.stdout, result.stderr)

    def run_binary(self, parts: Iterable[str], timeout: float = 12.0) -> bytes:
        remote = " ".join(shlex.quote(str(part)) for part in parts)
        cmd = self._base() + [remote]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=timeout)
        except FileNotFoundError as exc:
            raise SerBotError(f"required command is missing: {exc.filename}") from exc
        except subprocess.TimeoutExpired as exc:
            raise SerBotError(f"remote binary command timed out after {timeout:.1f}s") from exc
        if result.returncode != 0 or not result.stdout:
            message = result.stderr.decode("utf-8", "ignore").strip() or "remote binary command failed"
            raise SerBotError(message)
        return result.stdout

    def run_script(self, script: str, timeout: float = 20.0) -> CommandResult:
        cmd = self._base() + ["bash", "-s"]
        try:
            result = subprocess.run(cmd, input=script, text=True, capture_output=True, timeout=timeout)
        except FileNotFoundError as exc:
            raise SerBotError(f"required command is missing: {exc.filename}") from exc
        except subprocess.TimeoutExpired as exc:
            raise SerBotError(f"remote script timed out after {timeout:.1f}s") from exc
        return CommandResult(cmd, result.returncode, result.stdout, result.stderr)

    def scp_to(self, local_path: Path, remote_path: str, timeout: float = 20.0) -> None:
        base = [
            "scp",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            f"ConnectTimeout={self.config.connect_timeout}",
            str(local_path),
            f"{self.config.target}:{remote_path}",
        ]
        cmd = ["sshpass", "-p", self.config.password] + base if self.config.password else base
        try:
            result = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
        except FileNotFoundError as exc:
            raise SerBotError(f"required command is missing: {exc.filename}") from exc
        except subprocess.TimeoutExpired as exc:
            raise SerBotError(f"scp timed out after {timeout:.1f}s") from exc
        if result.returncode != 0:
            raise SerBotError(result.stderr.strip() or result.stdout.strip() or "scp failed")


def _remote_python_command(script: str) -> list[str]:
    encoded = base64.b64encode(script.encode("utf-8")).decode("ascii")
    return [
        "python3",
        "-c",
        "import base64; exec(base64.b64decode(%r).decode('utf-8'))" % encoded,
    ]


def _extract_runtime_value(text: str, key: str) -> str | None:
    match = re.search(r"%s=([^=]+?)(?= [a-z_]+=|$)" % re.escape(key), text)
    return match.group(1).strip() if match else None


def _parse_runtime_sample(stdout_text: str) -> dict[str, Any]:
    line = stdout_text.strip().splitlines()[-1]
    raw_text = _extract_runtime_value(line, "raw_cm")
    backend = _extract_runtime_value(line, "backend") or "unknown"
    front_text = _extract_runtime_value(line, "front_cm")
    rear_text = _extract_runtime_value(line, "rear_cm")
    front_min = _extract_runtime_value(line, "front_min")
    rear_min = _extract_runtime_value(line, "rear_min")
    return {
        "backend": backend,
        "raw_cm": list(ast.literal_eval(raw_text)) if raw_text else [],
        "front_cm": list(ast.literal_eval(front_text)) if front_text else [],
        "rear_cm": list(ast.literal_eval(rear_text)) if rear_text else [],
        "front_min_cm": float(front_min) if front_min is not None else None,
        "rear_min_cm": float(rear_min) if rear_min is not None else None,
        "timestamp": round(time.time(), 3),
    }


def _scan_summary(scan: dict[str, Any]) -> dict[str, Any]:
    ranges = scan.get("ranges", [])
    finite = [float(v) for v in ranges if v is not None and math.isfinite(float(v))]
    if not finite:
        return {"sample_count": len(ranges), "finite_count": 0}

    def sector_min(center_deg: float, half_width_deg: float = 20.0) -> float | None:
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)
        angle = float(scan["angle_min"])
        best = None
        for value in ranges:
            if value is not None:
                delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
                if abs(delta) <= half:
                    best = float(value) if best is None else min(best, float(value))
            angle += float(scan["angle_increment"])
        return best

    points = []
    angle = float(scan["angle_min"])
    for value in ranges:
        if value is not None:
            distance = float(value)
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
        angle += float(scan["angle_increment"])
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]

    return {
        "sample_count": len(ranges),
        "finite_count": len(finite),
        "min_range_m": round(min(finite), 3),
        "max_range_m": round(max(finite), 3),
        "visible_width_m": round(max(xs) - min(xs), 3) if xs else None,
        "visible_height_m": round(max(ys) - min(ys), 3) if ys else None,
        "front_min_m": round(sector_min(0.0), 3) if sector_min(0.0) is not None else None,
        "left_min_m": round(sector_min(90.0), 3) if sector_min(90.0) is not None else None,
        "rear_min_m": round(sector_min(180.0), 3) if sector_min(180.0) is not None else None,
        "right_min_m": round(sector_min(-90.0), 3) if sector_min(-90.0) is not None else None,
    }


class SerBot:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig.from_env()
        self.transport = SshTransport(self.config)

    @classmethod
    def from_env(cls) -> "SerBot":
        return cls(RobotConfig.from_env())

    @staticmethod
    def discover_targets(
        candidates: Iterable[str] | None = None,
        *,
        port: int = 22,
        timeout: float = 0.8,
    ) -> list[dict[str, Any]]:
        if candidates is None:
            env_candidates = os.environ.get("SERBOT_DISCOVERY_CANDIDATES", "")
            candidates = [item.strip() for item in env_candidates.split(",") if item.strip()] or DEFAULT_DISCOVERY_CANDIDATES

        seen: set[str] = set()
        results: list[dict[str, Any]] = []
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            item: dict[str, Any] = {
                "target": candidate,
                "resolved": [],
                "ssh_open": False,
                "banner": None,
                "error": None,
            }
            try:
                resolved = socket.getaddrinfo(candidate, port, type=socket.SOCK_STREAM)
                addresses = []
                for family, socktype, proto, _, sockaddr in resolved:
                    address = sockaddr[0]
                    if address not in addresses:
                        addresses.append(address)
                item["resolved"] = addresses
            except OSError as exc:
                item["error"] = str(exc)
                results.append(item)
                continue

            for address in item["resolved"]:
                sock = socket.socket(socket.AF_INET if ":" not in address else socket.AF_INET6, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                try:
                    sock.connect((address, port))
                    item["ssh_open"] = True
                    try:
                        item["banner"] = sock.recv(120).decode("utf-8", "ignore").strip()
                    except OSError:
                        item["banner"] = ""
                    break
                except OSError as exc:
                    item["error"] = str(exc)
                finally:
                    sock.close()
            results.append(item)
        return results

    def config_snapshot(self) -> dict[str, Any]:
        return self.config.safe_dict()

    def status(self) -> dict[str, Any]:
        result = self.transport.run_text(
            ["bash", "-lc", "hostname; hostname -I; uname -a; whoami"],
            timeout=10.0,
        ).check()
        lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        return {
            "hostname": lines[0] if len(lines) > 0 else None,
            "addresses": lines[1].split() if len(lines) > 1 else [],
            "kernel": lines[2] if len(lines) > 2 else None,
            "user": lines[3] if len(lines) > 3 else None,
            "target": self.config.target,
        }

    def network_interfaces(self) -> list[dict[str, str]]:
        script = r"""
python3 - <<'PY'
import json
from pathlib import Path

items = []
for address_file in sorted(Path('/sys/class/net').glob('*/address')):
    iface = address_file.parent.name
    try:
        address = address_file.read_text().strip()
    except Exception:
        address = ''
    try:
        state = (address_file.parent / 'operstate').read_text().strip()
    except Exception:
        state = ''
    items.append({'interface': iface, 'mac': address, 'state': state})
print(json.dumps(items, ensure_ascii=False))
PY
"""
        result = self.transport.run_script(script, timeout=10.0).check()
        return json.loads(result.stdout.strip().splitlines()[-1])

    def doctor(self) -> dict[str, Any]:
        report: dict[str, Any] = {
            "config": self.config.safe_dict(),
            "checks": {},
            "ok": False,
        }

        try:
            report["status"] = self.status()
            report["checks"]["ssh"] = "ok"
        except Exception as exc:
            report["checks"]["ssh"] = f"error: {exc}"
            return report

        script = f"""
python3 - <<'PY'
import importlib.util
import json
from pathlib import Path

def exists(path):
    return Path(path).exists()

def text(path):
    try:
        return Path(path).read_text().strip()
    except Exception:
        return None

payload = {{
    "runtime_exists": exists({self.config.runtime_path!r}),
    "camera_stream_exists": exists({self.config.camera_stream_path!r}),
    "can0_state": text("/sys/class/net/can0/operstate"),
    "video_devices": sorted(str(path) for path in Path("/dev").glob("video*")),
    "tty_usb": sorted(str(path) for path in Path("/dev").glob("ttyUSB*")),
    "python_can": importlib.util.find_spec("can") is not None,
    "opencv": importlib.util.find_spec("cv2") is not None,
}}
print(json.dumps(payload, ensure_ascii=False))
PY
"""
        try:
            remote = self.transport.run_script(script, timeout=12.0).check()
            report["remote"] = json.loads(remote.stdout.strip().splitlines()[-1])
        except Exception as exc:
            report["remote_error"] = str(exc)

        try:
            report["network_interfaces"] = self.network_interfaces()
        except Exception as exc:
            report["network_interfaces_error"] = str(exc)

        remote_info = report.get("remote", {})
        report["ok"] = bool(
            report["checks"].get("ssh") == "ok"
            and remote_info.get("runtime_exists")
            and remote_info.get("can0_state") is not None
        )
        return report

    def deploy_all(self) -> dict[str, Any]:
        return {
            "runtime": self.deploy_runtime(),
            "camera_stream": self.deploy_camera_stream_server(),
        }

    def deploy_runtime(self, local_path: str | Path | None = None) -> dict[str, str]:
        if local_path is None:
            local_path = Path(__file__).resolve().parents[1] / "tools" / "serbot_runtime.py"
        local = Path(local_path)
        if not local.exists():
            raise SerBotError(f"runtime file does not exist: {local}")

        remote_dir = str(Path(self.config.runtime_path).parent)
        self.transport.run_text(["mkdir", "-p", remote_dir], timeout=10.0).check()
        self.transport.scp_to(local, self.config.runtime_path, timeout=30.0)
        self.transport.run_text(["chmod", "+x", self.config.runtime_path], timeout=10.0).check()
        return {"local": str(local), "remote": self.config.runtime_path}

    def deploy_camera_stream_server(self, local_path: str | Path | None = None) -> dict[str, str]:
        if local_path is None:
            local_path = Path(__file__).resolve().parents[1] / "tools" / "camera_stream_server.py"
        local = Path(local_path)
        if not local.exists():
            raise SerBotError(f"camera stream server file does not exist: {local}")

        remote_dir = str(Path(self.config.camera_stream_path).parent)
        self.transport.run_text(["mkdir", "-p", remote_dir], timeout=10.0).check()
        self.transport.scp_to(local, self.config.camera_stream_path, timeout=30.0)
        self.transport.run_text(["chmod", "+x", self.config.camera_stream_path], timeout=10.0).check()
        return {"local": str(local), "remote": self.config.camera_stream_path}

    def camera_stream_start(
        self,
        *,
        port: int = 8766,
        device: str = "/dev/video0",
        width: int = 640,
        height: int = 360,
        fps: float = 10.0,
        quality: int = 70,
        deploy: bool = True,
    ) -> dict[str, Any]:
        if deploy:
            self.deploy_camera_stream_server()
        self.camera_stream_stop()
        pid_file = f"/tmp/serbot_camera_stream_{port}.pid"
        log_file = f"/tmp/serbot_camera_stream_{port}.log"
        remote = (
            "nohup python3 {script} --host 0.0.0.0 --port {port} --device {device} "
            "--width {width} --height {height} --fps {fps} --quality {quality} "
            ">{log} 2>&1 & echo $! > {pid}"
        ).format(
            script=shlex.quote(self.config.camera_stream_path),
            port=int(port),
            device=shlex.quote(str(device)),
            width=int(width),
            height=int(height),
            fps=float(fps),
            quality=int(quality),
            log=shlex.quote(log_file),
            pid=shlex.quote(pid_file),
        )
        self.transport.run_text(["bash", "-lc", remote], timeout=10.0).check()
        time.sleep(0.4)
        status = self.camera_stream_status(port=port)
        status["url"] = self.camera_stream_url(port=port)
        return status

    def camera_stream_stop(self, *, port: int = 8766) -> dict[str, Any]:
        pid_file = f"/tmp/serbot_camera_stream_{port}.pid"
        remote = f"""python3 - <<'PY'
import os
import signal
import subprocess
import time

pid_file = {pid_file!r}
script_path = {self.config.camera_stream_path!r}
current = os.getpid()
parent = os.getppid()
pids = set()

if os.path.exists(pid_file):
    try:
        with open(pid_file) as handle:
            text = handle.read().strip()
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
    except ProcessLookupError:
        pass
    except Exception:
        pass

time.sleep(0.4)
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
PY"""
        self.transport.run_text(["bash", "-lc", remote], timeout=8.0).check()
        return {"running": False, "port": int(port)}

    def camera_stream_status(self, *, port: int = 8766) -> dict[str, Any]:
        pid_file = f"/tmp/serbot_camera_stream_{port}.pid"
        log_file = f"/tmp/serbot_camera_stream_{port}.log"
        remote = (
            "python3 - <<'PY'\n"
            "import json, os, subprocess\n"
            f"pid_file={pid_file!r}\n"
            f"log_file={log_file!r}\n"
            "pid=None\n"
            "running=False\n"
            "if os.path.exists(pid_file):\n"
            "    pid=open(pid_file).read().strip()\n"
            "    if pid:\n"
            "        running=subprocess.call(['kill','-0',pid], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)==0\n"
            "tail=''\n"
            "if os.path.exists(log_file):\n"
            "    with open(log_file, 'r', errors='ignore') as f:\n"
            "        tail=''.join(f.readlines()[-10:])\n"
            "print(json.dumps({'running': running, 'pid': pid, 'port': "
            + str(int(port))
            + ", 'log_tail': tail}))\n"
            "PY"
        )
        result = self.transport.run_text(["bash", "-lc", remote], timeout=8.0).check()
        data = json.loads(result.stdout.strip().splitlines()[-1])
        data["url"] = self.camera_stream_url(port=port)
        return data

    def camera_stream_url(self, *, port: int = 8766, path: str = "/stream.mjpg") -> str:
        host = self._target_host()
        return f"http://{host}:{int(port)}{path}"

    def probe(self) -> str:
        return self._runtime(["probe"], timeout=12.0).stdout.strip()

    def stop(self) -> str:
        return self._runtime(["stop"], timeout=8.0).stdout.strip()

    def _check_motion_seconds(self, seconds: float) -> None:
        seconds = float(seconds)
        if seconds <= 0:
            raise SerBotError("motion duration must be greater than 0")
        if not self.config.allow_unsafe_speed and seconds > self.config.max_motion_seconds:
            raise SerBotError(
                "motion duration %.2fs exceeds safe max %.2fs; set SERBOT_MAX_MOTION_SECONDS or "
                "SERBOT_ALLOW_UNSAFE_SPEED=1 if you really need this"
                % (seconds, self.config.max_motion_seconds)
            )

    def _check_speed(self, label: str, value: float) -> None:
        value = float(value)
        if self.config.allow_unsafe_speed:
            return
        limit = float(self.config.safe_max_speed)
        if abs(value) > limit:
            raise SerBotError(
                "%s speed %.2f exceeds safe max %.2f; lower the value or set "
                "SERBOT_ALLOW_UNSAFE_SPEED=1"
                % (label, value, limit)
            )

    def _check_sequence_step(self, step: dict[str, Any], index: int) -> None:
        self._check_motion_seconds(float(step.get("seconds", 0.3)))
        if "speed" in step:
            self._check_speed(f"sequence[{index}].speed", float(step["speed"]))
        for key in ("linear", "angular", "left", "right"):
            if key in step:
                self._check_speed(f"sequence[{index}].{key}", float(step[key]))

    def move(
        self,
        direction: str,
        *,
        speed: int = 10,
        seconds: float = 0.3,
        period: float = 0.05,
        guard: bool = True,
        guard_cm: float = 20.0,
        allow_blind: bool = False,
        ultra_backend: str = "pop",
    ) -> str:
        if direction not in VALID_DIRECTIONS:
            raise ValueError(f"direction must be one of {sorted(VALID_DIRECTIONS)}")
        self._check_speed("move", speed)
        self._check_motion_seconds(seconds)
        parts = [
            "--ultra-backend",
            ultra_backend,
            "move",
            direction,
            "--speed",
            str(speed),
            "--seconds",
            str(seconds),
            "--period",
            str(period),
            "--guard-cm",
            str(guard_cm),
        ]
        if not guard:
            parts.append("--no-guard")
        if allow_blind:
            parts.append("--allow-blind")
        return self._runtime(parts, timeout=max(8.0, float(seconds) + 8.0)).stdout.strip()

    def forward(self, *, speed: int = 10, seconds: float = 0.3, **kwargs: Any) -> str:
        return self.move("forward", speed=speed, seconds=seconds, **kwargs)

    def backward(self, *, speed: int = 10, seconds: float = 0.3, **kwargs: Any) -> str:
        return self.move("backward", speed=speed, seconds=seconds, **kwargs)

    def spin_left(self, *, speed: int = 8, seconds: float = 0.3, **kwargs: Any) -> str:
        return self.move("spin-left", speed=speed, seconds=seconds, **kwargs)

    def spin_right(self, *, speed: int = 8, seconds: float = 0.3, **kwargs: Any) -> str:
        return self.move("spin-right", speed=speed, seconds=seconds, **kwargs)

    def raw(
        self,
        left: int,
        right: int,
        *,
        seconds: float = 0.3,
        period: float = 0.05,
        guard_direction: str = "none",
        guard_cm: float = 20.0,
        allow_blind: bool = False,
    ) -> str:
        self._check_speed("left wheel", left)
        self._check_speed("right wheel", right)
        self._check_motion_seconds(seconds)
        parts = [
            "raw",
            "--left",
            str(left),
            "--right",
            str(right),
            "--seconds",
            str(seconds),
            "--period",
            str(period),
            "--guard-direction",
            guard_direction,
            "--guard-cm",
            str(guard_cm),
        ]
        if allow_blind:
            parts.append("--allow-blind")
        return self._runtime(
            parts,
            timeout=max(8.0, float(seconds) + 8.0),
        ).stdout.strip()

    def wheels(self, left: int, right: int, **kwargs: Any) -> str:
        return self.raw(left, right, **kwargs)

    def drive(
        self,
        *,
        linear: float = 0.0,
        angular: float = 0.0,
        seconds: float = 0.3,
        period: float = 0.05,
        guard_direction: str = "auto",
        guard_cm: float = 20.0,
        allow_blind: bool = False,
        limit: int = 120,
    ) -> str:
        self._check_speed("linear", linear)
        self._check_speed("angular", angular)
        self._check_motion_seconds(seconds)
        parts = [
            "drive",
            "--linear",
            str(linear),
            "--angular",
            str(angular),
            "--limit",
            str(limit),
            "--seconds",
            str(seconds),
            "--period",
            str(period),
            "--guard-direction",
            guard_direction,
            "--guard-cm",
            str(guard_cm),
        ]
        if allow_blind:
            parts.append("--allow-blind")
        return self._runtime(parts, timeout=max(8.0, float(seconds) + 8.0)).stdout.strip()

    def sequence(
        self,
        steps: Iterable[MotionStep | dict[str, Any]],
        *,
        stop_between: bool = False,
    ) -> str:
        payload = [self._step_payload(step) for step in steps]
        if not payload:
            raise ValueError("sequence needs at least one step")
        for index, step in enumerate(payload):
            self._check_sequence_step(step, index)
        parts = ["sequence", "--json", json.dumps(payload, separators=(",", ":"))]
        if stop_between:
            parts.append("--stop-between")
        duration = sum(float(step.get("seconds", 0.3)) for step in payload)
        return self._runtime(parts, timeout=max(10.0, duration + 10.0)).stdout.strip()

    def pulse(
        self,
        left: int,
        right: int,
        *,
        count: int,
        on_seconds: float = 0.15,
        off_seconds: float = 0.1,
        period: float = 0.05,
    ) -> str:
        steps: list[MotionStep] = []
        for _ in range(count):
            steps.append(MotionStep.wheels(left, right, seconds=on_seconds, period=period))
            steps.append(MotionStep.wheels(0, 0, seconds=off_seconds, period=period, guard_direction="none"))
        return self.sequence(steps, stop_between=False)

    def ramp(
        self,
        direction: str,
        speeds: Iterable[int],
        *,
        step_seconds: float = 1.0,
        period: float = 0.05,
        guard: bool = True,
        guard_cm: float = 20.0,
        allow_blind: bool = False,
    ) -> str:
        speed_values = [int(speed) for speed in speeds]
        if not speed_values:
            raise ValueError("ramp needs at least one speed")
        speed_text = ",".join(str(speed) for speed in speed_values)
        for speed in speed_values:
            self._check_speed("ramp", speed)
        self._check_motion_seconds(step_seconds)
        parts = [
            "ramp",
            direction,
            "--speeds",
            speed_text,
            "--step-seconds",
            str(step_seconds),
            "--period",
            str(period),
            "--guard-cm",
            str(guard_cm),
        ]
        if not guard:
            parts.append("--no-guard")
        if allow_blind:
            parts.append("--allow-blind")
        timeout = max(8.0, len(speed_text.split(",")) * float(step_seconds) + 8.0)
        return self._runtime(parts, timeout=timeout).stdout.strip()

    def ultrasonic(self, *, count: int = 1, timeout: float = 0.45, ultra_backend: str = "pop") -> dict[str, Any] | list[dict[str, Any]]:
        result = self._runtime(
            [
                "--ultra-backend",
                ultra_backend,
                "ultra",
                "--count",
                str(count),
                "--timeout",
                str(timeout),
                "--interval",
                "0.0",
            ],
            timeout=max(10.0, count * 2.0 + 8.0),
        )
        samples = [_parse_runtime_sample(line) for line in result.stdout.splitlines() if "raw_cm=" in line]
        if count == 1:
            if not samples:
                raise SerBotError("ultrasonic command returned no sample")
            return samples[-1]
        return samples

    def camera_frame(self, output_path: str | Path | None = None) -> bytes:
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
ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
if not ok:
    sys.stderr.write("jpeg encode failed")
    sys.exit(4)
sys.stdout.buffer.write(encoded.tobytes())
"""
        data = self.transport.run_binary(_remote_python_command(script), timeout=10.0)
        if output_path is not None:
            path = Path(output_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        return data

    def lidar_scan(self, *, include_summary: bool = True) -> dict[str, Any]:
        script = f"""
set -eo pipefail
source /opt/ros/{self.config.ros_distro}/setup.bash >/dev/null 2>&1
source ~/ros_ws/install/setup.bash >/dev/null 2>&1

log_file="/tmp/sllidar_goal_$$.log"
timeout 30s ros2 run sllidar_ros2 sllidar_node --ros-args \\
  -p channel_type:=serial \\
  -p serial_port:=/dev/ttyUSB0 \\
  -p serial_baudrate:=256000 \\
  -p frame_id:=laser \\
  -p inverted:=false \\
  -p angle_compensate:=true \\
  -p scan_mode:=Sensitivity >"$log_file" 2>&1 &
node_pid=$!

cleanup() {{
  kill "$node_pid" >/dev/null 2>&1 || true
  wait "$node_pid" >/dev/null 2>&1 || true
  rm -f "$log_file"
}}
trap cleanup EXIT
sleep 3

python3 - <<'PY'
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanProbe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        self.msg = None
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg):
        self.msg = msg

rclpy.init()
node = ScanProbe()
deadline = time.time() + 30.0
while time.time() < deadline and node.msg is None:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.msg is None:
    print(json.dumps({{"ok": False, "error": "no_scan"}}))
else:
    msg = node.msg
    print(json.dumps({{
        "ok": True,
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "angle_increment": msg.angle_increment,
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "ranges": [None if (not math.isfinite(v) or v <= 0.0) else v for v in msg.ranges],
        "intensities": list(msg.intensities[: len(msg.ranges)]),
    }}))

node.destroy_node()
rclpy.shutdown()
PY
"""
        result = self.transport.run_script(script, timeout=45.0).check()
        lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if not lines:
            raise SerBotError("LiDAR command produced no output")
        payload = json.loads(lines[-1])
        if not payload.get("ok"):
            raise SerBotError(f"LiDAR capture failed: {payload}")
        if include_summary:
            payload["summary"] = _scan_summary(payload)
        return payload

    def write_lidar_scan(self, output_path: str | Path, *, include_summary: bool = True) -> dict[str, Any]:
        scan = self.lidar_scan(include_summary=include_summary)
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(scan, ensure_ascii=False, indent=2), encoding="utf-8")
        return scan

    def goal(self, name: str, **kwargs: Any) -> Any:
        aliases = {
            "left": "spin-left",
            "right": "spin-right",
            "spin_left": "spin-left",
            "spin_right": "spin-right",
            "ultra": "ultrasonic",
            "camera": "camera_frame",
            "lidar": "lidar_scan",
        }
        action = aliases.get(name, name)
        if action in VALID_DIRECTIONS:
            return self.move(action, **kwargs)
        if action == "stop":
            return self.stop()
        if action == "ultrasonic":
            return self.ultrasonic(**kwargs)
        if action == "camera_frame":
            return self.camera_frame(**kwargs)
        if action == "lidar_scan":
            return self.lidar_scan(**kwargs)
        if action == "status":
            return self.status()
        if action == "config":
            return self.config_snapshot()
        if action == "doctor":
            return self.doctor()
        if action == "probe":
            return self.probe()
        if action == "drive":
            return self.drive(**kwargs)
        if action in ("raw", "wheels"):
            return self.raw(**kwargs)
        if action == "sequence":
            return self.sequence(**kwargs)
        raise ValueError(f"unknown goal: {name}")

    @staticmethod
    def _step_payload(step: MotionStep | dict[str, Any]) -> dict[str, Any]:
        if isinstance(step, MotionStep):
            return step.to_dict()
        if isinstance(step, dict):
            return dict(step)
        raise TypeError("sequence steps must be MotionStep or dict objects")

    def _target_host(self) -> str:
        target = self.config.target
        if "@" in target:
            target = target.rsplit("@", 1)[1]
        if ":" in target and not target.startswith("["):
            target = target.split(":", 1)[0]
        return target.strip("[]")

    def _runtime(self, parts: Iterable[str], timeout: float = 12.0) -> CommandResult:
        return self.transport.run_text(["python3", self.config.runtime_path, *parts], timeout=timeout).check()
