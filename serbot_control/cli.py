"""Command line entrypoint for the local SerBot client."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

from .client import (
    DEFAULT_CAMERA_STREAM_PATH,
    DEFAULT_MAX_MOTION_SECONDS,
    DEFAULT_PASSWORD,
    DEFAULT_RUNTIME_PATH,
    DEFAULT_SAFE_MAX_SPEED,
    DEFAULT_TARGET,
    RobotConfig,
    SerBot,
    SerBotError,
)


def print_value(value: Any) -> None:
    if isinstance(value, (dict, list)):
        print(json.dumps(value, ensure_ascii=False, indent=2))
    elif isinstance(value, bytes):
        print(f"bytes={len(value)}")
    elif value is not None:
        print(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="goal",
        description="Control a Hanback AIoT SerBot over SSH with simple goal-style commands.",
    )
    parser.add_argument("--target", default=os.environ.get("SERBOT_TARGET", DEFAULT_TARGET))
    parser.add_argument("--password", default=os.environ.get("SERBOT_PASSWORD", DEFAULT_PASSWORD))
    parser.add_argument("--runtime-path", default=os.environ.get("SERBOT_RUNTIME", DEFAULT_RUNTIME_PATH))
    parser.add_argument("--camera-stream-path", default=os.environ.get("SERBOT_CAMERA_STREAM", DEFAULT_CAMERA_STREAM_PATH))
    parser.add_argument("--ros-distro", default=os.environ.get("SERBOT_ROS_DISTRO", "foxy"))
    parser.add_argument("--connect-timeout", type=int, default=8)
    parser.add_argument("--safe-max-speed", type=int, default=int(os.environ.get("SERBOT_SAFE_MAX_SPEED", DEFAULT_SAFE_MAX_SPEED)))
    parser.add_argument("--max-motion-seconds", type=float, default=float(os.environ.get("SERBOT_MAX_MOTION_SECONDS", DEFAULT_MAX_MOTION_SECONDS)))
    parser.add_argument(
        "--allow-unsafe-speed",
        action="store_true",
        default=os.environ.get("SERBOT_ALLOW_UNSAFE_SPEED", "").lower() in {"1", "true", "yes", "on"},
        help="allow speeds/durations above the configured safety caps",
    )

    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    subparsers.add_parser("config", help="print effective local configuration without exposing the password")
    discover = subparsers.add_parser("discover", help="probe common robot hostnames/IPs for an open SSH port")
    discover.add_argument("--candidates", default="", help="comma-separated hostnames/IPs; defaults to known SerBot candidates")
    discover.add_argument("--timeout", type=float, default=0.8)

    subparsers.add_parser("status", help="check SSH and print robot identity")
    subparsers.add_parser("doctor", help="run non-motion connection/runtime/sensor diagnostics")
    subparsers.add_parser("probe", help="probe runtime, CAN stop, and ultrasonic path")
    subparsers.add_parser("stop", help="send stop frames")

    deploy = subparsers.add_parser("deploy-runtime", help="copy tools/serbot_runtime.py to the robot")
    deploy.add_argument("--local-path", default=None)
    subparsers.add_parser("deploy-all", help="copy runtime and camera stream server to the robot")

    for name in ("forward", "backward", "spin-left", "spin-right"):
        move = subparsers.add_parser(name, help=f"run {name} movement")
        add_motion_args(move)

    move = subparsers.add_parser("move", help="run a movement by direction")
    move.add_argument("direction", choices=["forward", "backward", "spin-left", "spin-right"])
    add_motion_args(move)

    raw = subparsers.add_parser("raw", help="send explicit left/right wheel values")
    raw.add_argument("--left", type=int, required=True)
    raw.add_argument("--right", type=int, required=True)
    raw.add_argument("--seconds", type=float, default=0.3)
    raw.add_argument("--period", type=float, default=0.05)
    raw.add_argument("--guard-direction", choices=["none", "forward", "backward"], default="none")
    raw.add_argument("--guard-cm", type=float, default=20.0)
    raw.add_argument("--allow-blind", action="store_true")

    drive = subparsers.add_parser("drive", help="mix signed linear/angular control values")
    drive.add_argument("--linear", type=float, default=0.0)
    drive.add_argument("--angular", type=float, default=0.0)
    drive.add_argument("--seconds", type=float, default=0.3)
    drive.add_argument("--period", type=float, default=0.05)
    drive.add_argument("--limit", type=int, default=120)
    drive.add_argument("--guard-direction", choices=["auto", "none", "forward", "backward"], default="auto")
    drive.add_argument("--guard-cm", type=float, default=20.0)
    drive.add_argument("--allow-blind", action="store_true")

    ramp = subparsers.add_parser("ramp", help="step through speeds for one direction")
    ramp.add_argument("direction", choices=["forward", "backward", "spin-left", "spin-right"])
    ramp.add_argument("--speeds", required=True, help="comma-separated speeds, for example 6,8,10")
    ramp.add_argument("--step-seconds", type=float, default=1.0)
    ramp.add_argument("--no-guard", action="store_true")
    ramp.add_argument("--allow-blind", action="store_true")

    sequence = subparsers.add_parser("sequence", help="run JSON motion steps inside the robot")
    sequence.add_argument("--json", default=None, help="JSON list or {'steps': [...]}")
    sequence.add_argument("--file", default=None, help="JSON file containing the sequence")
    sequence.add_argument("--stop-between", action="store_true")

    ultra = subparsers.add_parser("ultra", help="read ultrasonic sensor values")
    ultra.add_argument("--count", type=int, default=1)
    ultra.add_argument("--timeout", type=float, default=0.45)
    ultra.add_argument("--backend", default="pop", choices=["auto", "can", "pop"])

    camera = subparsers.add_parser("camera", help="capture one camera JPEG")
    camera.add_argument("--output", default="artifacts/camera/latest.jpg")

    stream = subparsers.add_parser("camera-stream", help="manage direct MJPEG camera streaming on the robot")
    stream.add_argument("action", choices=["deploy", "start", "stop", "status", "url"])
    stream.add_argument("--port", type=int, default=8766)
    stream.add_argument("--device", default="/dev/video0")
    stream.add_argument("--width", type=int, default=640)
    stream.add_argument("--height", type=int, default=360)
    stream.add_argument("--fps", type=float, default=10.0)
    stream.add_argument("--quality", type=int, default=70)
    stream.add_argument("--no-deploy", action="store_true")

    lidar = subparsers.add_parser("lidar", help="capture one LiDAR LaserScan frame")
    lidar.add_argument("--output", default="artifacts/lidar/latest-scan.json")
    lidar.add_argument("--no-summary", action="store_true")

    goal = subparsers.add_parser("goal", help="dispatch a named goal")
    goal.add_argument("name", choices=["config", "status", "doctor", "probe", "stop", "forward", "backward", "spin-left", "spin-right", "drive", "raw", "sequence", "ultra", "camera", "lidar"])
    goal.add_argument("--speed", type=int, default=10)
    goal.add_argument("--seconds", type=float, default=0.3)
    goal.add_argument("--period", type=float, default=0.05)
    goal.add_argument("--guard-cm", type=float, default=20.0)
    goal.add_argument("--count", type=int, default=1)
    goal.add_argument("--output", default=None)
    goal.add_argument("--no-guard", action="store_true")

    return parser


def add_motion_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--seconds", type=float, default=0.3)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--guard-cm", type=float, default=20.0)
    parser.add_argument("--no-guard", action="store_true")
    parser.add_argument("--allow-blind", action="store_true")
    parser.add_argument("--backend", default="pop", choices=["auto", "can", "pop"])


def robot_from_args(args: argparse.Namespace) -> SerBot:
    return SerBot(
        RobotConfig(
            target=args.target,
            password=args.password,
            runtime_path=args.runtime_path,
            camera_stream_path=args.camera_stream_path,
            ros_distro=args.ros_distro,
            connect_timeout=args.connect_timeout,
            safe_max_speed=args.safe_max_speed,
            max_motion_seconds=args.max_motion_seconds,
            allow_unsafe_speed=args.allow_unsafe_speed,
        )
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    robot = robot_from_args(args)

    try:
        if args.command == "config":
            print_value(robot.config_snapshot())
        elif args.command == "discover":
            candidates = [item.strip() for item in args.candidates.split(",") if item.strip()] if args.candidates else None
            print_value(SerBot.discover_targets(candidates, timeout=args.timeout))
        elif args.command == "status":
            print_value(robot.status())
        elif args.command == "doctor":
            print_value(robot.doctor())
        elif args.command == "probe":
            print_value(robot.probe())
        elif args.command == "stop":
            print_value(robot.stop())
        elif args.command == "deploy-runtime":
            print_value(robot.deploy_runtime(args.local_path))
        elif args.command == "deploy-all":
            print_value(robot.deploy_all())
        elif args.command in ("forward", "backward", "spin-left", "spin-right"):
            print_value(
                robot.move(
                    args.command,
                    speed=args.speed,
                    seconds=args.seconds,
                    period=args.period,
                    guard=not args.no_guard,
                    guard_cm=args.guard_cm,
                    allow_blind=args.allow_blind,
                    ultra_backend=args.backend,
                )
            )
        elif args.command == "move":
            print_value(
                robot.move(
                    args.direction,
                    speed=args.speed,
                    seconds=args.seconds,
                    period=args.period,
                    guard=not args.no_guard,
                    guard_cm=args.guard_cm,
                    allow_blind=args.allow_blind,
                    ultra_backend=args.backend,
                )
            )
        elif args.command == "raw":
            print_value(
                robot.raw(
                    args.left,
                    args.right,
                    seconds=args.seconds,
                    period=args.period,
                    guard_direction=args.guard_direction,
                    guard_cm=args.guard_cm,
                    allow_blind=args.allow_blind,
                )
            )
        elif args.command == "drive":
            print_value(
                robot.drive(
                    linear=args.linear,
                    angular=args.angular,
                    seconds=args.seconds,
                    period=args.period,
                    guard_direction=args.guard_direction,
                    guard_cm=args.guard_cm,
                    allow_blind=args.allow_blind,
                    limit=args.limit,
                )
            )
        elif args.command == "ramp":
            speeds = [int(chunk.strip()) for chunk in args.speeds.split(",") if chunk.strip()]
            print_value(
                robot.ramp(
                    args.direction,
                    speeds,
                    step_seconds=args.step_seconds,
                    period=args.period,
                    guard=not args.no_guard,
                    guard_cm=args.guard_cm,
                    allow_blind=args.allow_blind,
                )
            )
        elif args.command == "sequence":
            print_value(robot.sequence(load_sequence_payload(args), stop_between=args.stop_between))
        elif args.command == "ultra":
            print_value(robot.ultrasonic(count=args.count, timeout=args.timeout, ultra_backend=args.backend))
        elif args.command == "camera":
            output = Path(args.output)
            robot.camera_frame(output)
            print(f"camera={output}")
        elif args.command == "camera-stream":
            handle_camera_stream(robot, args)
        elif args.command == "lidar":
            output = Path(args.output)
            scan = robot.write_lidar_scan(output, include_summary=not args.no_summary)
            print(f"lidar={output}")
            if "summary" in scan:
                print_value(scan["summary"])
        elif args.command == "goal":
            run_named_goal(robot, args)
        else:
            parser.error(f"unknown command: {args.command}")
    except (SerBotError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


def run_named_goal(robot: SerBot, args: argparse.Namespace) -> None:
    if args.name in ("forward", "backward", "spin-left", "spin-right"):
        print_value(
            robot.goal(
                args.name,
                speed=args.speed,
                seconds=args.seconds,
                period=args.period,
                guard=not args.no_guard,
                guard_cm=args.guard_cm,
            )
        )
    elif args.name == "drive":
        print_value(robot.goal("drive", linear=args.speed, angular=0.0, seconds=args.seconds, period=args.period))
    elif args.name == "raw":
        print_value(robot.goal("raw", left=args.speed, right=-args.speed, seconds=args.seconds, period=args.period))
    elif args.name == "sequence":
        if not args.output:
            raise ValueError("goal sequence requires --output pointing to a JSON sequence file")
        print_value(robot.sequence(load_sequence_file(Path(args.output))))
    elif args.name == "ultra":
        print_value(robot.goal("ultra", count=args.count))
    elif args.name == "camera":
        output = args.output or "artifacts/camera/latest.jpg"
        robot.goal("camera", output_path=output)
        print(f"camera={output}")
    elif args.name == "lidar":
        output = args.output or "artifacts/lidar/latest-scan.json"
        scan = robot.write_lidar_scan(output)
        print(f"lidar={output}")
        print_value(scan.get("summary", scan))
    else:
        print_value(robot.goal(args.name))


def handle_camera_stream(robot: SerBot, args: argparse.Namespace) -> None:
    if args.action == "deploy":
        print_value(robot.deploy_camera_stream_server())
    elif args.action == "start":
        print_value(
            robot.camera_stream_start(
                port=args.port,
                device=args.device,
                width=args.width,
                height=args.height,
                fps=args.fps,
                quality=args.quality,
                deploy=not args.no_deploy,
            )
        )
    elif args.action == "stop":
        print_value(robot.camera_stream_stop(port=args.port))
    elif args.action == "status":
        print_value(robot.camera_stream_status(port=args.port))
    elif args.action == "url":
        print(robot.camera_stream_url(port=args.port))
    else:
        raise ValueError(f"unknown camera-stream action: {args.action}")


def load_sequence_payload(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.file:
        return load_sequence_file(Path(args.file))
    if args.json:
        return load_sequence_text(args.json)
    raise ValueError("sequence requires --json or --file")


def load_sequence_file(path: Path) -> list[dict[str, Any]]:
    return load_sequence_text(path.read_text(encoding="utf-8"))


def load_sequence_text(text: str) -> list[dict[str, Any]]:
    payload = json.loads(text)
    if isinstance(payload, dict):
        payload = payload.get("steps", [])
    if not isinstance(payload, list) or not payload:
        raise ValueError("sequence must be a non-empty list or {'steps': [...]}")
    for index, step in enumerate(payload):
        if not isinstance(step, dict):
            raise ValueError(f"sequence step {index} must be an object")
    return payload


if __name__ == "__main__":
    raise SystemExit(main())
