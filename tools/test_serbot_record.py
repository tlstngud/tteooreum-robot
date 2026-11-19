import contextlib
import functools
import io
import json
from pathlib import Path
import sys
import tempfile
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
    with path.open("w") as output:
        while not stop.is_set():
            recorder.write_row(output, recorder.timestamp())
            stats["count"] += 1
            stop.wait(0.05)


def broken_reader(args, folder, stop, stats):
    raise RuntimeError("no camera")


class RecorderTests(unittest.TestCase):
    def parse(self, *args):
        return recorder.build_parser().parse_args(args)

    def test_default_sources_and_range(self):
        args = self.parse()
        self.assertEqual(args.sensors, ["camera", "ultrasonic", "lidar"])
        self.assertEqual(args.seconds, 30)
        for value in ("nan", "inf", "0"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse("--seconds", value)

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
            folder, = Path(tmp).iterdir()
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
            folder, = Path(tmp).iterdir()
            manifest = json.loads((folder / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "incomplete")
            self.assertEqual(manifest["sources"]["camera"]["error"], "no camera")
            self.assertGreater(manifest["sources"]["ultrasonic"]["count"], 0)
