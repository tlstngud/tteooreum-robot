#!/usr/bin/env python3
"""Record the SerBot camera and sensors locally on the Jetson (Python 3.8+)."""

import argparse
from datetime import datetime
import fcntl
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import time

DEFAULT_OUTPUT = Path.home() / "serbot-recordings"
MIN_FREE_BYTES = 256 * 1024 * 1024
LOCK_FILE = ".recording.lock"
ACTIVE_LINK = ".active-recording"
STOP_FILE = ".stop-request"
OUTPUTS = {
    "camera": ("camera.avi", "camera_frames.jsonl"),
    "ultrasonic": ("ultrasonic.jsonl",),
    "lidar": ("lidar.jsonl",),
}


def monotonic_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def timestamp():
    return {"monotonic_ns": monotonic_ns(), "unix_ns": time.time_ns()}


def write_row(output, row):
    output.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")


def finite(value):
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def scan_record(scan, times):
    return {
        **times,
        "header_stamp": {
            "sec": scan.header.stamp.sec,
            "nanosec": scan.header.stamp.nanosec,
        },
        "frame_id": scan.header.frame_id,
        "angle_min": finite(scan.angle_min),
        "angle_max": finite(scan.angle_max),
        "angle_increment": finite(scan.angle_increment),
        "range_min": finite(scan.range_min),
        "range_max": finite(scan.range_max),
        "scan_time": finite(scan.scan_time),
        "time_increment": finite(scan.time_increment),
        "ranges": [finite(value) for value in scan.ranges],
        "intensities": [finite(value) for value in scan.intensities],
    }


def capture_camera(args, folder, stop, stats):
    import cv2

    if args.stream_port is None:
        device = int(args.camera_device) if args.camera_device.isdecimal() else args.camera_device
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    else:
        capture = cv2.VideoCapture("http://127.0.0.1:%d/stream.mjpg" % args.stream_port)
    writer = None
    try:
        if not capture.isOpened():
            raise RuntimeError("camera unavailable; check the device or the local camera stream")
        if args.stream_port is None:
            for prop, value in (
                (cv2.CAP_PROP_BUFFERSIZE, 1),
                (cv2.CAP_PROP_FRAME_WIDTH, args.width),
                (cv2.CAP_PROP_FRAME_HEIGHT, args.height),
                (cv2.CAP_PROP_FPS, args.fps),
            ):
                capture.set(prop, value)
        interval = 1.0 / args.fps
        with (folder / "camera_frames.jsonl").open("w", encoding="utf-8", buffering=1) as output:
            while not stop.is_set():
                cycle_start = time.monotonic()
                ok, frame = capture.read()
                times = timestamp()
                if not ok or frame is None:
                    raise RuntimeError("camera stream stopped delivering frames")
                height, width = frame.shape[:2]
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(folder / "camera.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                        args.fps, (width, height),
                    )
                    if not writer.isOpened():
                        raise RuntimeError("OpenCV could not open the MJPG AVI writer")
                    stats.update(width=width, height=height, nominal_fps=args.fps)
                if (width, height) != (stats["width"], stats["height"]):
                    raise RuntimeError("camera resolution changed during recording")
                writer.write(frame)
                write_row(output, {**times, "frame": stats["count"]})
                stats["count"] += 1
                stop.wait(max(0, interval - (time.monotonic() - cycle_start)))
    finally:
        if writer is not None:
            writer.release()
        capture.release()


def capture_ultrasonic(args, folder, stop, stats):
    from serbot_runtime import PopUltraInterface, SocketCanInterface

    can = None
    if args.ultra_backend == "pop":
        read = PopUltraInterface().read
    else:
        can = SocketCanInterface(channel=args.can_channel)

        def read():
            return can.read_ultra_frame(timeout=0.2)

    try:
        with (folder / "ultrasonic.jsonl").open("w", encoding="utf-8", buffering=1) as output:
            while not stop.is_set():
                cycle_start = time.monotonic()
                sample = read()
                times = timestamp()
                if sample and any(value is not None for value in sample["raw_cm"]):
                    row = {**times, "backend": sample["backend"]}
                    for field in ("raw_cm", "front_cm", "rear_cm"):
                        row[field] = [finite(value) for value in sample[field]]
                    for field in ("front_min_cm", "rear_min_cm"):
                        row[field] = finite(sample[field])
                    write_row(output, row)
                    stats["count"] += 1
                else:
                    stats["missed"] = stats.get("missed", 0) + 1
                stop.wait(max(0, 1.0 / args.ultra_hz - (time.monotonic() - cycle_start)))
    finally:
        if can is not None:
            can.bus.shutdown()


def capture_lidar(args, folder, stop, stats):
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        raise RuntimeError("source the robot ROS 2 setup before recording LiDAR") from exc

    rclpy.init(args=[])
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node = None
    try:
        node = rclpy.create_node("serbot_recorder_%d" % os.getpid())
        with (folder / "lidar.jsonl").open("w", encoding="utf-8", buffering=1) as output:
            def receive(scan):
                write_row(output, scan_record(scan, timestamp()))
                stats["count"] += 1

            node.create_subscription(LaserScan, args.scan_topic, receive, qos_profile_sensor_data)
            while not stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


READERS = {
    "camera": capture_camera,
    "ultrasonic": capture_ultrasonic,
    "lidar": capture_lidar,
}


def worker(reader, args, folder, stop, result):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    stats = {"count": 0}
    try:
        reader(args, folder, stop, stats)
    except Exception as exc:
        stats["error"] = str(exc) or type(exc).__name__
    try:
        result.send(stats)
        result.close()
    finally:
        # Some versions of pop leave a CAN reader thread running after capture ends.
        os._exit(1 if "error" in stats else 0)


def save_manifest(folder, data):
    temporary = folder / ".manifest.tmp"
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(folder / "manifest.json")


def record(args, readers=None):
    base = args.output_dir.expanduser().resolve()
    base.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (base / LOCK_FILE).open("a+b") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("a recording is already running in %s" % base) from None
        active = base / ACTIVE_LINK
        active.unlink(missing_ok=True)
        try:
            return capture(args, READERS if readers is None else readers, base, active)
        finally:
            active.unlink(missing_ok=True)


def request_stop(output_dir):
    base = output_dir.expanduser().resolve()
    try:
        with (base / LOCK_FILE).open("r+b") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                target = Path(os.readlink(base / ACTIVE_LINK))
                if target.parent != Path(".") or target.name in ("", ".."):
                    raise RuntimeError("invalid active recording directory")
                folder = (base / target).resolve(strict=True)
                if folder.parent != base or not folder.is_dir():
                    raise RuntimeError("invalid active recording directory")
                fd = os.open(folder / STOP_FILE, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600)
                os.close(fd)
            else:
                raise RuntimeError("no active recording in %s" % base)
    except FileNotFoundError:
        raise RuntimeError("no active recording ready to stop in %s" % base) from None

    # Wait for the recorder to close the video and publish the final status.
    expiration = time.monotonic() + 10
    while time.monotonic() < expiration:
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
        if manifest["status"] != "recording":
            print("recording_dir=%s" % folder)
            print("status=%s" % manifest["status"])
            return 1 if manifest["status"] == "incomplete" else 0
        time.sleep(0.1)
    raise RuntimeError("stop requested; still waiting for final files in %s" % folder)


def capture(args, readers, base, active):
    if shutil.disk_usage(base).free < MIN_FREE_BYTES:
        raise RuntimeError("recording requires at least 256 MiB of free storage")
    folder = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d-%H%M%S-"), dir=base))
    names = list(dict.fromkeys(args.sensors))
    started = timestamp()
    deadline = None if args.seconds is None else started["monotonic_ns"] + int(args.seconds * 1e9)
    manifest = {
        "schema_version": 1, "status": "recording",
        "started": started, "requested_seconds": args.seconds,
        "scan_topic": args.scan_topic if "lidar" in names else None,
        "sources": {name: {"files": list(OUTPUTS[name])} for name in names},
    }
    save_manifest(folder, manifest)

    ctx = mp.get_context("spawn")
    stop = ctx.Event()
    processes = {}
    stop_reason = None

    def interrupt(signum, _frame):
        nonlocal stop_reason
        stop_reason = signal.Signals(signum).name

    signals = [signal.SIGINT, signal.SIGTERM]
    if signal.getsignal(signal.SIGHUP) != signal.SIG_IGN:
        signals.append(signal.SIGHUP)
    previous = {sig: signal.signal(sig, interrupt) for sig in signals}
    try:
        active.symlink_to(folder.name, target_is_directory=True)
        print("recording_dir=%s" % folder, flush=True)
        for name in names:
            if stop_reason is not None:
                break
            receiver, sender = ctx.Pipe(duplex=False)
            process = ctx.Process(target=worker, args=(readers[name], args, folder, stop, sender))
            try:
                process.start()
            except BaseException:
                receiver.close()
                raise
            finally:
                sender.close()
            processes[name] = process, receiver

        while stop_reason is None and (deadline is None or monotonic_ns() < deadline):
            if (folder / STOP_FILE).exists():
                stop_reason = "requested_stop"
                break
            if not any(process.is_alive() for process, _ in processes.values()):
                stop_reason = "sources_exited"
                break
            if shutil.disk_usage(base).free < MIN_FREE_BYTES:
                stop_reason = "low_disk_space"
                break
            time.sleep(0.1)
    except BaseException:
        stop_reason = "recorder_error"
        raise
    finally:
        stop.set()
        try:
            grace_deadline = time.monotonic() + 3
            for name, (process, receiver) in processes.items():
                process.join(timeout=max(0, grace_deadline - time.monotonic()))
                timed_out = process.is_alive()
                if timed_out:
                    process.terminate()
                    process.join(timeout=1)
                    if process.is_alive():
                        process.kill()
                        process.join()
                stats = {"count": 0, "error": "capture process did not report a result"}
                if receiver.poll():
                    try:
                        stats = receiver.recv()
                    except EOFError:
                        pass
                if timed_out:
                    stats["error"] = "capture did not stop cleanly; output may be incomplete"
                receiver.close()
                if stats["count"] == 0 and "error" not in stats:
                    stats["error"] = (
                        "no LiDAR scans; check the ROS 2 publisher and topic"
                        if name == "lidar" else "no samples received"
                    )
                manifest["sources"][name].update(stats)
            for name in set(names) - processes.keys():
                manifest["sources"][name].update(count=0, error="capture did not start")
            failed = [name for name, stats in manifest["sources"].items() if "error" in stats]
            reason = stop_reason or "duration"
            status = "complete" if reason in ("duration", "requested_stop", "SIGINT") else "interrupted"
            if failed or reason in ("sources_exited", "low_disk_space", "recorder_error"):
                status = "incomplete"
            manifest.update(finished=timestamp(), status=status, stop_reason=reason)
            save_manifest(folder, manifest)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)

    for name, stats in manifest["sources"].items():
        print("%s: %d%s" % (name, stats["count"], " (%s)" % stats["error"] if name in failed else ""))
    return 1 if manifest["status"] == "incomplete" else 0


def in_range(converter, minimum, maximum):
    def parse(text):
        value = converter(text)
        if not minimum <= value <= maximum:
            raise argparse.ArgumentTypeError("must be between %s and %s" % (minimum, maximum))
        return value
    return parse


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--seconds", type=in_range(float, 1, 3600),
                      help="optional time limit; omit to record until stopped")
    mode.add_argument("--stop", action="store_true",
                      help="finish the recorder using this output directory")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sensors", nargs="+", choices=tuple(READERS), default=list(READERS))
    camera = parser.add_mutually_exclusive_group()
    camera.add_argument("--camera-device", default="/dev/video0")
    camera.add_argument("--stream-port", type=in_range(int, 1, 65535),
                        help="read an already running MJPEG stream on 127.0.0.1")
    parser.add_argument("--width", type=in_range(int, 16, 1920), default=640)
    parser.add_argument("--height", type=in_range(int, 16, 1080), default=360)
    parser.add_argument("--fps", type=in_range(float, 1, 30), default=10)
    parser.add_argument("--ultra-hz", type=in_range(float, 1, 20), default=5)
    parser.add_argument("--ultra-backend", choices=("pop", "can"), default="pop")
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--scan-topic", default="/scan")
    return parser


def main(argv=None):
    try:
        args = build_parser().parse_args(argv)
        return request_stop(args.output_dir) if args.stop else record(args)
    except (OSError, RuntimeError) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
