#!/usr/bin/env python3
"""Unified low-level motor and ultrasonic tool for Hanback AIoT SerBot."""

import argparse
import collections
import json
import os
import statistics
import sys
import time

try:
    import can
except ImportError:  # pragma: no cover - dependency exists only on the robot
    can = None


MOTOR_ARB_ID = 0x101
ULTRA_ENABLE_ARB_ID = 0x114
ULTRA_DATA_ARB_ID = 0x201
WHEEL_PAIR_MASK = 0x03
NEUTRAL_OFFSET = 120
ULTRA_ENABLE_PAYLOAD = [0b00110011, 0x01]

DEFAULT_SPEED = 10
DEFAULT_SECONDS = 0.3
DEFAULT_PERIOD = 0.05
DEFAULT_GUARD_CM = 20.0
DEFAULT_ULTRA_TIMEOUT = 0.4

FRONT_CAN_INDICES = (0, 1)
REAR_CAN_INDICES = (4, 5)
FRONT_POP_INDICES = (1, 2)
REAR_POP_INDICES = (4, 5)


class RuntimeErrorWithContext(RuntimeError):
    """Raised when hardware access fails with user-actionable context."""


def clamp(value, low, high):
    return max(low, min(high, value))


def signed_speed_to_byte(value):
    value = int(round(value))
    if value < -120 or value > 120:
        raise ValueError("wheel speed must be between -120 and 120")
    encoded = NEUTRAL_OFFSET + value
    if encoded < 0 or encoded > 255:
        raise ValueError("encoded wheel speed is out of byte range")
    return encoded


def pick_values(raw, indices):
    values = []
    for index in indices:
        if index < len(raw):
            values.append(raw[index])
    return values


def valid_min(values):
    usable = [float(v) for v in values if v is not None]
    if not usable:
        return None
    return min(usable)


def parse_speed_list(text):
    speeds = []
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        speeds.append(int(chunk))
    if not speeds:
        raise argparse.ArgumentTypeError("at least one speed is required")
    return speeds


def load_sequence(text):
    try:
        payload = json.loads(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("sequence JSON is invalid: %s" % exc)
    if isinstance(payload, dict):
        payload = payload.get("steps", [])
    if not isinstance(payload, list) or not payload:
        raise argparse.ArgumentTypeError("sequence must be a non-empty list or {'steps': [...]}")
    for index, step in enumerate(payload):
        if not isinstance(step, dict):
            raise argparse.ArgumentTypeError("sequence step %d must be an object" % index)
    return payload


def load_sequence_arg(value):
    if value == "-":
        return load_sequence(sys.stdin.read())
    if value.startswith("@"):
        with open(value[1:], "r") as handle:
            return load_sequence(handle.read())
    return load_sequence(value)


class SocketCanInterface(object):
    def __init__(self, channel="can0", median_window=5):
        if can is None:
            raise RuntimeErrorWithContext(
                "python-can is not installed. Install it on the Jetson first."
            )
        self.channel = channel
        self.bus = self._open_bus(channel)
        self.histories = collections.defaultdict(
            lambda: collections.deque(maxlen=median_window)
        )

    @staticmethod
    def _open_bus(channel):
        attempts = []
        for bustype in ("socketcan_native", "socketcan"):
            try:
                return can.interface.Bus(channel=channel, bustype=bustype)
            except Exception as exc:  # pragma: no cover - depends on robot image
                attempts.append("%s: %s" % (bustype, exc))
        raise RuntimeErrorWithContext(
            "failed to open SocketCAN on %s (%s)" % (channel, "; ".join(attempts))
        )

    def enable_ultrasonic(self):
        msg = can.Message(
            arbitration_id=ULTRA_ENABLE_ARB_ID,
            is_extended_id=False,
            data=ULTRA_ENABLE_PAYLOAD,
        )
        self.bus.send(msg)

    def send_wheels(self, left, right):
        msg = can.Message(
            arbitration_id=MOTOR_ARB_ID,
            is_extended_id=False,
            data=[
                WHEEL_PAIR_MASK,
                signed_speed_to_byte(left),
                signed_speed_to_byte(right),
            ],
        )
        self.bus.send(msg)

    def stop(self, repeat=3, delay=0.02):
        for _ in range(repeat):
            self.send_wheels(0, 0)
            time.sleep(delay)

    def read_ultra_frame(self, timeout=DEFAULT_ULTRA_TIMEOUT, enable=True):
        if enable:
            self.enable_ultrasonic()
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            msg = self.bus.recv(remaining)
            if msg is None:
                continue
            if msg.arbitration_id != ULTRA_DATA_ARB_ID:
                continue
            return self._decode_ultra_frame(msg)
        return None

    def _decode_ultra_frame(self, msg):
        raw = [int(value) for value in msg.data]
        for index, value in enumerate(raw):
            self.histories[index].append(value)
        smoothed = []
        for index in range(len(raw)):
            history = self.histories[index]
            smoothed.append(float(statistics.median(history)) if history else None)
        front = pick_values(smoothed, FRONT_CAN_INDICES)
        rear = pick_values(smoothed, REAR_CAN_INDICES)
        return {
            "backend": "can",
            "raw_cm": raw,
            "smoothed_cm": smoothed,
            "front_cm": front,
            "rear_cm": rear,
            "front_min_cm": valid_min(front),
            "rear_min_cm": valid_min(rear),
            "timestamp": time.time(),
        }


class PopUltraInterface(object):
    def __init__(self):
        try:
            from pop import CAN
        except Exception as exc:  # pragma: no cover - robot only
            raise RuntimeErrorWithContext(
                "pop library is not available for ultrasonic fallback: %s" % exc
            )
        self.device = CAN.OmniWheel_new(0)
        self.device.readStart()
        self.device.ultraEnable()
        time.sleep(0.1)

    def read(self):
        raw = list(self.device.readUltra())
        front = pick_values(raw, FRONT_POP_INDICES)
        rear = pick_values(raw, REAR_POP_INDICES)
        return {
            "backend": "pop",
            "raw_cm": raw,
            "smoothed_cm": raw,
            "front_cm": front,
            "rear_cm": rear,
            "front_min_cm": valid_min(front),
            "rear_min_cm": valid_min(rear),
            "timestamp": time.time(),
        }


class SerbotRuntime(object):
    def __init__(self, channel="can0", ultra_backend="auto"):
        self.can = SocketCanInterface(channel=channel)
        self.ultra_backend = ultra_backend
        self._pop = None

    def _get_pop(self):
        if self._pop is None:
            self._pop = PopUltraInterface()
        return self._pop

    def read_ultra(self, timeout=DEFAULT_ULTRA_TIMEOUT, backend=None):
        backend = backend or self.ultra_backend
        errors = []
        if backend in ("auto", "can"):
            try:
                sample = self.can.read_ultra_frame(timeout=timeout, enable=True)
                if sample is not None:
                    return sample
                errors.append("CAN ultrasonic frame timed out")
            except Exception as exc:
                errors.append("CAN ultrasonic read failed: %s" % exc)
            if backend == "can":
                raise RuntimeErrorWithContext("; ".join(errors))
        if backend in ("auto", "pop"):
            try:
                return self._get_pop().read()
            except Exception as exc:
                errors.append("pop ultrasonic read failed: %s" % exc)
        raise RuntimeErrorWithContext("; ".join(errors))

    def stop(self):
        self.can.stop()

    def run_wheels(
        self,
        left,
        right,
        seconds,
        period,
        guard_direction=None,
        guard_cm=DEFAULT_GUARD_CM,
        require_ultra=True,
        ultra_timeout=DEFAULT_ULTRA_TIMEOUT,
        stop_at_end=True,
    ):
        result = {
            "requested_left": int(left),
            "requested_right": int(right),
            "guard_direction": guard_direction,
            "guard_cm": guard_cm,
            "stopped_by_guard": False,
            "sensor_timeout": False,
            "last_ultra": None,
        }
        deadline = time.time() + seconds
        try:
            while time.time() < deadline:
                self.can.send_wheels(left, right)
                if not guard_direction:
                    time.sleep(period)
                    continue
                sample = self.read_ultra(timeout=min(period, ultra_timeout))
                result["last_ultra"] = sample
                guard_value = self.guard_value_cm(sample, guard_direction)
                if guard_value is None:
                    if require_ultra:
                        result["sensor_timeout"] = True
                        break
                elif guard_value <= guard_cm:
                    result["stopped_by_guard"] = True
                    result["guard_hit_cm"] = guard_value
                    break
                sleep_for = period - 0.005
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            if stop_at_end:
                self.stop()
        return result

    @staticmethod
    def guard_value_cm(sample, direction):
        if direction == "forward":
            return sample.get("front_min_cm")
        if direction == "backward":
            return sample.get("rear_min_cm")
        return None


def direction_to_wheels(direction, speed):
    speed = int(speed)
    if direction == "forward":
        return speed, -speed
    if direction == "backward":
        return -speed, speed
    if direction == "spin-right":
        return speed, speed
    if direction == "spin-left":
        return -speed, -speed
    raise ValueError("unsupported direction: %s" % direction)


def drive_to_wheels(linear, angular, limit=120):
    left = float(linear) - float(angular)
    right = -(float(linear) + float(angular))
    return int(round(clamp(left, -limit, limit))), int(round(clamp(right, -limit, limit)))


def infer_guard_direction(left, right):
    if left > 0 and right < 0:
        return "forward"
    if left < 0 and right > 0:
        return "backward"
    return None


def step_to_wheels(step):
    if "left" in step and "right" in step:
        return int(step["left"]), int(step["right"])
    if "linear" in step or "angular" in step:
        return drive_to_wheels(
            step.get("linear", 0),
            step.get("angular", 0),
            limit=int(step.get("limit", 120)),
        )
    if "direction" in step:
        return direction_to_wheels(step["direction"], int(step.get("speed", DEFAULT_SPEED)))
    raise ValueError("step needs left/right, linear/angular, or direction")


def step_guard_direction(step, left, right):
    if step.get("guard") is False:
        return None
    value = step.get("guard_direction", "auto")
    if value in (None, "none"):
        return None
    if value == "auto":
        if "linear" in step:
            linear = float(step.get("linear", 0))
            if linear > 0:
                return "forward"
            if linear < 0:
                return "backward"
            return None
        if step.get("direction") in ("forward", "backward"):
            return step.get("direction")
        return infer_guard_direction(left, right)
    if value not in ("forward", "backward"):
        raise ValueError("guard_direction must be auto, none, forward, or backward")
    return value


def print_sample(sample):
    print(
        "backend=%s raw_cm=%s front_cm=%s rear_cm=%s front_min=%.1f rear_min=%.1f"
        % (
            sample["backend"],
            sample["raw_cm"],
            sample["front_cm"],
            sample["rear_cm"],
            sample["front_min_cm"] if sample["front_min_cm"] is not None else -1.0,
            sample["rear_min_cm"] if sample["rear_min_cm"] is not None else -1.0,
        )
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Low-level SerBot motor and ultrasonic runtime"
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN channel")
    parser.add_argument(
        "--ultra-backend",
        default="auto",
        choices=["auto", "can", "pop"],
        help="ultrasonic backend preference",
    )

    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    probe = subparsers.add_parser("probe", help="probe CAN and ultrasonic paths")
    probe.add_argument(
        "--timeout", type=float, default=DEFAULT_ULTRA_TIMEOUT, help="read timeout"
    )

    ultra = subparsers.add_parser("ultra", help="stream ultrasonic samples")
    ultra.add_argument("--count", type=int, default=10, help="number of samples")
    ultra.add_argument(
        "--timeout", type=float, default=DEFAULT_ULTRA_TIMEOUT, help="read timeout"
    )
    ultra.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="delay between printed samples",
    )

    stop = subparsers.add_parser("stop", help="send repeated stop frames")
    stop.add_argument("--repeat", type=int, default=3)

    move = subparsers.add_parser("move", help="run a directional movement")
    move.add_argument(
        "direction",
        choices=["forward", "backward", "spin-left", "spin-right"],
    )
    move.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    move.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    move.add_argument("--period", type=float, default=DEFAULT_PERIOD)
    move.add_argument("--guard-cm", type=float, default=DEFAULT_GUARD_CM)
    move.add_argument(
        "--no-guard",
        action="store_true",
        help="disable ultrasonic guard for forward/backward motion",
    )
    move.add_argument(
        "--allow-blind",
        action="store_true",
        help="keep moving even if no ultrasonic sample arrives",
    )

    raw = subparsers.add_parser("raw", help="run explicit left/right wheel values")
    raw.add_argument("--left", type=int, required=True)
    raw.add_argument("--right", type=int, required=True)
    raw.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    raw.add_argument("--period", type=float, default=DEFAULT_PERIOD)
    raw.add_argument(
        "--guard-direction",
        choices=["none", "forward", "backward"],
        default="none",
    )
    raw.add_argument("--guard-cm", type=float, default=DEFAULT_GUARD_CM)
    raw.add_argument("--allow-blind", action="store_true")

    drive = subparsers.add_parser(
        "drive", help="mix signed linear/angular values into wheel commands"
    )
    drive.add_argument("--linear", type=float, default=0.0)
    drive.add_argument("--angular", type=float, default=0.0)
    drive.add_argument("--limit", type=int, default=120)
    drive.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    drive.add_argument("--period", type=float, default=DEFAULT_PERIOD)
    drive.add_argument(
        "--guard-direction",
        choices=["auto", "none", "forward", "backward"],
        default="auto",
    )
    drive.add_argument("--guard-cm", type=float, default=DEFAULT_GUARD_CM)
    drive.add_argument("--allow-blind", action="store_true")

    ramp = subparsers.add_parser(
        "ramp", help="step through multiple speeds using one direction"
    )
    ramp.add_argument(
        "direction", choices=["forward", "backward", "spin-left", "spin-right"]
    )
    ramp.add_argument("--speeds", type=parse_speed_list, required=True)
    ramp.add_argument("--step-seconds", type=float, default=1.0)
    ramp.add_argument("--period", type=float, default=DEFAULT_PERIOD)
    ramp.add_argument("--guard-cm", type=float, default=DEFAULT_GUARD_CM)
    ramp.add_argument("--no-guard", action="store_true")
    ramp.add_argument("--allow-blind", action="store_true")

    sequence = subparsers.add_parser(
        "sequence", help="run a JSON sequence of wheel/vector/directional steps"
    )
    sequence.add_argument(
        "--json",
        type=load_sequence_arg,
        required=True,
        help="JSON list, @file.json, or - for stdin",
    )
    sequence.add_argument(
        "--stop-between",
        action="store_true",
        help="send stop frames between sequence steps",
    )

    return parser


def command_probe(runtime, args):
    runtime.stop()
    print("can_stop=ok")
    sample = runtime.read_ultra(timeout=args.timeout)
    print_sample(sample)


def command_ultra(runtime, args):
    for _ in range(args.count):
        sample = runtime.read_ultra(timeout=args.timeout)
        print_sample(sample)
        time.sleep(args.interval)


def command_stop(runtime, args):
    runtime.can.stop(repeat=args.repeat)
    print("stop=ok repeat=%d" % args.repeat)


def command_move(runtime, args):
    left, right = direction_to_wheels(args.direction, args.speed)
    guard_direction = None
    if args.direction in ("forward", "backward") and not args.no_guard:
        guard_direction = args.direction
    result = runtime.run_wheels(
        left=left,
        right=right,
        seconds=args.seconds,
        period=args.period,
        guard_direction=guard_direction,
        guard_cm=args.guard_cm,
        require_ultra=not args.allow_blind,
    )
    print("move_result=%s" % result)


def command_raw(runtime, args):
    guard_direction = None if args.guard_direction == "none" else args.guard_direction
    result = runtime.run_wheels(
        left=args.left,
        right=args.right,
        seconds=args.seconds,
        period=args.period,
        guard_direction=guard_direction,
        guard_cm=args.guard_cm,
        require_ultra=not args.allow_blind,
    )
    print("raw_result=%s" % result)


def command_drive(runtime, args):
    left, right = drive_to_wheels(args.linear, args.angular, limit=args.limit)
    guard_direction = None
    if args.guard_direction == "auto":
        if args.linear > 0:
            guard_direction = "forward"
        elif args.linear < 0:
            guard_direction = "backward"
    elif args.guard_direction != "none":
        guard_direction = args.guard_direction
    result = runtime.run_wheels(
        left=left,
        right=right,
        seconds=args.seconds,
        period=args.period,
        guard_direction=guard_direction,
        guard_cm=args.guard_cm,
        require_ultra=not args.allow_blind,
    )
    print(
        "drive_result=%s linear=%.3f angular=%.3f left=%d right=%d"
        % (result, args.linear, args.angular, left, right)
    )


def command_ramp(runtime, args):
    for speed in args.speeds:
        left, right = direction_to_wheels(args.direction, speed)
        guard_direction = None
        if args.direction in ("forward", "backward") and not args.no_guard:
            guard_direction = args.direction
        print("ramp_step direction=%s speed=%s" % (args.direction, speed))
        result = runtime.run_wheels(
            left=left,
            right=right,
            seconds=args.step_seconds,
            period=args.period,
            guard_direction=guard_direction,
            guard_cm=args.guard_cm,
            require_ultra=not args.allow_blind,
        )
        print("ramp_result=%s" % result)
        if result.get("stopped_by_guard") or result.get("sensor_timeout"):
            break


def command_sequence(runtime, args):
    results = []
    interrupted = False
    try:
        for index, step in enumerate(args.json):
            left, right = step_to_wheels(step)
            seconds = float(step.get("seconds", DEFAULT_SECONDS))
            period = float(step.get("period", DEFAULT_PERIOD))
            guard_direction = step_guard_direction(step, left, right)
            guard_cm = float(step.get("guard_cm", DEFAULT_GUARD_CM))
            allow_blind = bool(step.get("allow_blind", False))
            stop_at_end = args.stop_between or index == (len(args.json) - 1)
            result = runtime.run_wheels(
                left=left,
                right=right,
                seconds=seconds,
                period=period,
                guard_direction=guard_direction,
                guard_cm=guard_cm,
                require_ultra=not allow_blind,
                stop_at_end=stop_at_end,
            )
            row = {
                "index": index,
                "left": left,
                "right": right,
                "seconds": seconds,
                "guard_direction": guard_direction,
                "result": result,
            }
            results.append(row)
            print("sequence_step=%s" % row)
            if result.get("stopped_by_guard") or result.get("sensor_timeout"):
                interrupted = True
                break
    finally:
        runtime.stop()
    print("sequence_result=%s" % {"steps": results, "interrupted": interrupted})


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    runtime = SerbotRuntime(channel=args.channel, ultra_backend=args.ultra_backend)
    commands = {
        "probe": command_probe,
        "ultra": command_ultra,
        "stop": command_stop,
        "move": command_move,
        "raw": command_raw,
        "drive": command_drive,
        "ramp": command_ramp,
        "sequence": command_sequence,
    }
    try:
        commands[args.command](runtime, args)
    except KeyboardInterrupt:
        runtime.stop()
        print("interrupted=1", file=sys.stderr)
        return 130
    except Exception as exc:
        runtime.stop()
        print("error=%s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
