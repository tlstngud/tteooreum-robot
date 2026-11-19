import contextlib
import functools
import io
import json
import multiprocessing as mp
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tools import serbot_record as recorder


class OneCycle:
    def __init__(self):
        self.done = False

    def is_set(self):
        return self.done

    def wait(self, _seconds):
        self.done = True


def fake_scan():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=5, nanosec=100), frame_id="laser"),
        angle_min=-1.0, angle_max=1.0, angle_increment=0.5, time_increment=0.01,
        range_min=0.15, range_max=20, scan_time=0.1,
        ranges=[0.3, float("inf"), float("nan")], intensities=[2, 3, float("-inf")],
    )


def synthetic_reader(name, args, folder, stop, stats):
    path = folder / recorder.OUTPUTS[name][-1]
    with path.open("w", encoding="utf-8", buffering=1) as output:
        while not stop.is_set():
            recorder.write_row(output, recorder.timestamp())
            stats["count"] += 1
            stop.wait(0.05)


def broken_reader(args, folder, stop, stats):
    raise RuntimeError("no camera")


def run_continuous_recorder(output_dir):
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    args = recorder.build_parser().parse_args(["--output-dir", output_dir, "--sensors", "ultrasonic"])
    raise SystemExit(recorder.record(args, {"ultrasonic": functools.partial(synthetic_reader, "ultrasonic")}))


def get_recording_folder(output_dir):
    folder, = (entry for entry in Path(output_dir).iterdir() if entry.is_dir() and not entry.is_symlink())
    return folder


class RecorderTests(unittest.TestCase):
    def parse(self, *args):
        return recorder.build_parser().parse_args(args)

    def test_default_sources_and_range(self):
        args = self.parse()
        self.assertEqual(args.sensors, ["camera", "ultrasonic", "lidar"])
        self.assertIsNone(args.seconds)
        for value in ("nan", "inf", "0"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse("--seconds", value)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.parse("--seconds", "30", "--stop")

    def test_scan_record_is_strict_json(self):
        times = {"monotonic_ns": 11, "unix_ns": 12}
        result = recorder.scan_record(fake_scan(), times)
        self.assertEqual(result["header_stamp"], {"sec": 5, "nanosec": 100})
        self.assertEqual(result["ranges"], [0.3, None, None])
        self.assertEqual(result["intensities"], [2, 3, None])
        self.assertEqual(result["monotonic_ns"], 11)
        recorder.write_row(io.StringIO(), result)

    def test_lidar_subscribes_with_sensor_qos_and_saves_scan(self):
        stop = OneCycle()
        node, rclpy, sensor_qos = Mock(), Mock(), object()
        laser_scan = type("LaserScan", (), {})
        rclpy.create_node.return_value = node

        def spin_once(_node, timeout_sec):
            callback = node.create_subscription.call_args.args[2]
            callback(fake_scan())
            stop.wait(timeout_sec)

        rclpy.spin_once.side_effect = spin_once
        modules = {
            "rclpy": rclpy,
            "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=sensor_qos),
            "sensor_msgs.msg": SimpleNamespace(LaserScan=laser_scan),
        }
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, modules), patch.object(recorder.signal, "signal"):
            folder = Path(tmp)
            stats = {"count": 0}
            recorder.capture_lidar(self.parse("--scan-topic", "/test_scan"), folder, stop, stats)
            subscription = node.create_subscription.call_args.args
            self.assertEqual((subscription[0], subscription[1], subscription[3]),
                             (laser_scan, "/test_scan", sensor_qos))
            self.assertEqual(json.loads((folder / "lidar.jsonl").read_text())["ranges"],
                             [0.3, None, None])
            self.assertEqual(stats["count"], 1)
            node.destroy_node.assert_called_once()
            rclpy.shutdown.assert_called_once()

    def test_camera_releases_writer_and_indexes_frame(self):
        cap, writer = Mock(), Mock()
        frame = SimpleNamespace(shape=(360, 640, 3))
        cap.read.return_value = True, frame
        cv2 = Mock(VideoCapture=Mock(return_value=cap), VideoWriter=Mock(return_value=writer))
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {"cv2": cv2}):
            folder = Path(tmp)
            stats = {"count": 0}
            recorder.capture_camera(self.parse("--stream-port", "8766"), folder, OneCycle(), stats)
            timestamps = json.loads((folder / "camera_frames.jsonl").read_text())
            self.assertEqual(timestamps["frame"], 0)
            self.assertIn("monotonic_ns", timestamps)
            self.assertEqual(stats["count"], 1)
            cv2.VideoCapture.assert_called_once_with("http://127.0.0.1:8766/stream.mjpg")
            writer.write.assert_called_once_with(frame)
            writer.release.assert_called_once()
            cap.release.assert_called_once()

    def test_ultrasonic_only_reads_and_shuts_down_can(self):
        can = Mock()
        can.read_ultra_frame.return_value = {
            "backend": "can", "raw_cm": [31, 32, 180, 180, 50, 55],
            "front_cm": [31, 32], "rear_cm": [50, 55],
            "front_min_cm": 31, "rear_min_cm": 50,
        }
        runtime = SimpleNamespace(PopUltraInterface=Mock(), SocketCanInterface=Mock(return_value=can))
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {"serbot_runtime": runtime}):
            folder = Path(tmp)
            stats = {"count": 0}
            recorder.capture_ultrasonic(self.parse("--ultra-backend", "can"), folder, OneCycle(), stats)
            row = json.loads((folder / "ultrasonic.jsonl").read_text())
            self.assertEqual(stats["count"], 1)
            self.assertEqual(row["raw_cm"], [31, 32, 180, 180, 50, 55])
            self.assertEqual(row["rear_min_cm"], 50)
            can.read_ultra_frame.assert_called_once()
            can.send_wheels.assert_not_called()
            can.stop.assert_not_called()
            can.bus.shutdown.assert_called_once()

    def test_parallel_workers_produce_timestamped_logs(self):
        readers = {name: functools.partial(synthetic_reader, name) for name in recorder.OUTPUTS}
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            args = self.parse("--seconds", "1", "--output-dir", tmp)
            self.assertEqual(recorder.record(args, readers), 0)
            folder = get_recording_folder(tmp)
            manifest = json.loads((folder / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "complete")
            for name, source in manifest["sources"].items():
                self.assertGreater(source["count"], 0)
                log = folder / recorder.OUTPUTS[name][-1]
                for line in log.read_text().splitlines():
                    sample = json.loads(line)
                    self.assertGreaterEqual(sample["monotonic_ns"], manifest["started"]["monotonic_ns"])
                    self.assertLessEqual(sample["monotonic_ns"], manifest["finished"]["monotonic_ns"])

    def test_failed_source_keeps_other_log_and_returns_failure(self):
        readers = {"camera": broken_reader, "ultrasonic": functools.partial(synthetic_reader, "ultrasonic")}
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            args = self.parse("--sensors", "camera", "ultrasonic", "--seconds", "1", "--output-dir", tmp)
            self.assertEqual(recorder.record(args, readers), 1)
            folder = get_recording_folder(tmp)
            manifest = json.loads((folder / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "incomplete")
            self.assertEqual(manifest["sources"]["camera"]["error"], "no camera")
            self.assertGreater(manifest["sources"]["ultrasonic"]["count"], 0)

    def test_continuous_recording_survives_hangup_and_stops_from_separate_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            process = mp.get_context("spawn").Process(target=run_continuous_recorder, args=(tmp,))
            process.start()
            try:
                active = Path(tmp) / recorder.ACTIVE_LINK
                log = active / "ultrasonic.jsonl"
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and (not log.exists() or log.stat().st_size == 0):
                    time.sleep(0.05)
                self.assertTrue(log.exists())
                self.assertGreater(log.stat().st_size, 0)
                initial = json.loads((active / "manifest.json").read_text())
                self.assertEqual(initial["status"], "recording")
                self.assertIsNone(initial["requested_seconds"])
                with self.assertRaisesRegex(RuntimeError, "already running"):
                    recorder.record(self.parse("--output-dir", tmp))
                before = log.stat().st_size
                os.kill(process.pid, signal.SIGHUP)
                while time.monotonic() < deadline and log.stat().st_size == before:
                    time.sleep(0.05)
                self.assertGreater(log.stat().st_size, before)
                result = subprocess.run(
                    [sys.executable, recorder.__file__, "--stop", "--output-dir", tmp],
                    capture_output=True, text=True, timeout=15,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("status=complete", result.stdout)
                process.join(5)
                self.assertEqual(process.exitcode, 0)
                self.assertFalse(active.is_symlink())
                folder = get_recording_folder(tmp)
                manifest = json.loads((folder / "manifest.json").read_text())
                self.assertEqual(manifest["stop_reason"], "requested_stop")
                self.assertGreater(manifest["sources"]["ultrasonic"]["count"], 1)
                with self.assertRaisesRegex(RuntimeError, "no active recording"):
                    recorder.request_stop(Path(tmp))
            finally:
                if process.is_alive():
                    process.terminate()
                    process.join(5)
                    if process.is_alive():
                        process.kill()
                        process.join(5)
                process.close()
