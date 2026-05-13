#!/usr/bin/env python3
"""Example: precise SerBot control from local Python code.

Run from the workspace root after the robot is reachable:

    python3 examples/precision_control.py
"""

from serbot_control import MotionStep, SerBot


def main() -> None:
    robot = SerBot.from_env()

    print(robot.status())

    # Smooth vector control: linear > 0 means forward, angular > 0 turns left.
    print(robot.drive(linear=8, angular=0, seconds=0.4))
    print(robot.drive(linear=10, angular=2, seconds=0.5))

    # Direct wheel control: values are signed CAN speed units, roughly -120..120.
    print(robot.wheels(left=8, right=-8, seconds=0.3))

    # Multiple sub-second segments run inside the Jetson after one SSH call.
    print(
        robot.sequence(
            [
                MotionStep.vector(6, 0, seconds=0.3),
                MotionStep.vector(9, 2, seconds=0.4),
                MotionStep.vector(9, -2, seconds=0.4),
                MotionStep.wheels(0, 0, seconds=0.1, guard_direction="none"),
            ]
        )
    )

    print(robot.ultrasonic())
    scan = robot.lidar_scan()
    print(scan["summary"])
    robot.camera_frame("artifacts/camera/precision-example.jpg")
    robot.stop()


if __name__ == "__main__":
    main()
