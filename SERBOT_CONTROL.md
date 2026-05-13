# SerBot Control Library

This workspace now has a small local Python library and a `goal` command for
controlling the SerBot over SSH.

The robot does not need to be connected while writing code, but commands that
touch motors or sensors require the robot to be reachable by SSH.

Korean HTML guide:

```text
docs/serbot-control-guide.html
```

## Defaults

- SSH target: `soda@threnody.local`
- SSH password: `soda`
- Robot runtime: `/home/soda/ros_ws/tools/serbot_runtime.py`
- Camera stream server: `/home/soda/ros_ws/tools/camera_stream_server.py`
- ROS distro for LiDAR: `foxy`

Override them per command:

```bash
./goal --target soda@threnody.local status
```

Or with environment variables:

```bash
export SERBOT_TARGET=soda@threnody.local
export SERBOT_PASSWORD=soda
export SERBOT_RUNTIME=/home/soda/ros_ws/tools/serbot_runtime.py
export SERBOT_CAMERA_STREAM=/home/soda/ros_ws/tools/camera_stream_server.py
export SERBOT_ROS_DISTRO=foxy
```

## Command Usage

Check connection:

```bash
./goal status
```

Deploy the robot-side CAN/ultrasonic runtime:

```bash
./goal deploy-runtime
```

Move:

```bash
./goal forward --speed 10 --seconds 1
./goal backward --speed 8 --seconds 1
./goal spin-left --speed 8 --seconds 0.5
./goal spin-right --speed 8 --seconds 0.5
./goal stop
```

Fine control:

```bash
./goal drive --linear 8 --angular 0 --seconds 0.8
./goal drive --linear 10 --angular 3 --seconds 0.8
./goal drive --linear 0 --angular 10 --seconds 0.4 --guard-direction none
./goal raw --left 10 --right -8 --seconds 0.5
./goal raw --left 0 --right 0 --seconds 0.1
```

Control meaning:

- `linear > 0`: forward tendency
- `linear < 0`: backward tendency
- `angular > 0`: left rotation tendency
- `angular < 0`: right rotation tendency
- `raw --left/--right`: direct signed wheel command in the robot's CAN speed units
- wheel range accepted by the runtime: `-120..120`
- practical starting range: `6..10`

Run a full motion script inside the robot with one SSH call:

```bash
./goal sequence --json '[
  {"linear": 8, "angular": 0, "seconds": 0.5},
  {"linear": 10, "angular": 3, "seconds": 0.7},
  {"left": 8, "right": 8, "seconds": 0.25, "guard_direction": "none"},
  {"left": 0, "right": 0, "seconds": 0.1, "guard_direction": "none"}
]'
```

Sequence steps can use any one of these forms:

```json
{"direction": "forward", "speed": 10, "seconds": 1.0}
{"linear": 10, "angular": -3, "seconds": 0.5}
{"left": 10, "right": -10, "seconds": 0.5}
```

Each step also accepts:

```json
{"period": 0.05, "guard_direction": "auto", "guard_cm": 20, "allow_blind": false}
```

Read sensors:

```bash
./goal ultra --count 3
./goal camera --output artifacts/camera/latest.jpg
./goal lidar --output artifacts/lidar/latest-scan.json
```

Low-latency camera streaming:

```bash
./goal camera-stream start --port 8766 --width 640 --height 360 --fps 10 --quality 70
./goal camera-stream url
./goal camera-stream status
./goal camera-stream stop
```

After `start`, open the URL printed by `./goal camera-stream url`, usually:

```text
http://threnody.local:8766/stream.mjpg
```

This avoids per-frame SSH. SSH is used only to deploy/start/stop the stream server; the browser then receives frames directly from the robot over the LAN.

For a 30fps experiment, lower the resolution first:

```bash
./goal camera-stream start --port 8766 --width 424 --height 240 --fps 30 --quality 65
```

The same command also supports an explicit goal dispatcher:

```bash
./goal goal forward --speed 10 --seconds 1
./goal goal ultra --count 3
./goal goal camera --output artifacts/camera/latest.jpg
./goal goal lidar --output artifacts/lidar/latest-scan.json
```

## Python Usage

```python
from serbot_control import MotionStep, SerBot

robot = SerBot.from_env()

print(robot.status())
robot.forward(speed=10, seconds=1)
robot.stop()

robot.drive(linear=10, angular=2, seconds=0.6)
robot.wheels(left=10, right=-8, seconds=0.4)

robot.sequence([
    MotionStep.vector(8, 0, seconds=0.5),
    MotionStep.vector(10, 3, seconds=0.7),
    MotionStep.wheels(0, 0, seconds=0.1, guard_direction="none"),
])

ultra = robot.ultrasonic()
print(ultra["raw_cm"])

robot.camera_frame("artifacts/camera/latest.jpg")
print(robot.camera_stream_start())
print(robot.camera_stream_url())

scan = robot.lidar_scan()
print(scan["summary"])
```

Goal-style Python calls:

```python
robot.goal("forward", speed=10, seconds=1)
robot.goal("drive", linear=10, angular=-2, seconds=0.5)
robot.goal("raw", left=10, right=-10, seconds=0.5)
robot.goal("ultra", count=3)
robot.goal("camera", output_path="artifacts/camera/latest.jpg")
scan = robot.goal("lidar")
```

Runnable example:

```bash
python3 examples/precision_control.py
```

## Safety

- Forward and backward motion uses the robot-side ultrasonic guard by default.
- The runtime default stop threshold is currently `20 cm`.
- Do not use speed `99` unless you explicitly want maximum output.
- If the robot-side runtime is missing, run `./goal deploy-runtime` first.
- For precise tuning, change one variable at a time: `linear`, `angular`, or raw wheel values.
- Keep first tests short, for example `seconds=0.2..0.5`.
