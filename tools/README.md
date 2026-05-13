# SerBot Runtime Tools

`serbot_runtime.py` is a low-level runtime for the Hanback AIoT SerBot that keeps motor control and ultrasonic reads in one place.

Full Korean HTML guide:

```text
docs/serbot-control-guide.html
```

Why this exists:

- `Pilot.SerBot()` has been unreliable across robots because the internal hardware profile detection is inconsistent.
- Direct SocketCAN motor control has already been more predictable for this platform.
- The bundled navigation source also shows that ultrasonic data can be read directly from CAN frame `0x201`.

What the script does:

- sends wheel commands on CAN frame `0x101`
- enables ultrasonic data with CAN frame `0x114`
- reads ultrasonic frames from CAN `0x201`
- falls back to `pop.CAN.OmniWheel_new(0)` for ultrasonic reads when requested
- applies forward and backward ultrasonic stop guards

Wheel sign convention used here:

- forward: `left=+speed`, `right=-speed`
- backward: `left=-speed`, `right=+speed`
- spin right: `left=+speed`, `right=+speed`
- spin left: `left=-speed`, `right=-speed`

Quick usage on the Jetson:

```bash
python3 serbot_runtime.py probe
python3 serbot_runtime.py ultra --count 20
python3 serbot_runtime.py move forward --speed 10 --seconds 2
python3 serbot_runtime.py move backward --speed 8 --seconds 4
python3 serbot_runtime.py raw --left 10 --right -10 --seconds 1.5
python3 serbot_runtime.py ramp forward --speeds 6,8,10 --step-seconds 1
```

Room LiDAR snapshot from this Mac:

```bash
python3 tools/lidar_room_snapshot.py --ssh-target soda@threnody.local
```

That command:

- starts `sllidar_node` on the robot over SSH
- captures one `/scan` frame
- writes raw JSON, a text summary, and an SVG map into `artifacts/lidar/`

Higher-level local control:

```bash
./goal status
./goal forward --speed 10 --seconds 1
./goal ultra --count 3
./goal camera --output artifacts/camera/latest.jpg
./goal lidar --output artifacts/lidar/latest-scan.json
```

For real-time camera video on the same LAN, start the robot-side MJPEG streamer:

```bash
./goal camera-stream start --port 8766 --width 640 --height 360 --fps 10 --quality 70
./goal camera-stream url
```

This uses SSH only to start the server. Video frames then flow directly from the robot at `/stream.mjpg`.

Fine control:

```bash
./goal drive --linear 10 --angular 3 --seconds 0.6
./goal raw --left 10 --right -8 --seconds 0.4
./goal sequence --json '[{"linear":8,"angular":0,"seconds":0.5},{"linear":10,"angular":3,"seconds":0.7}]'
```

Python code can use the same wrapper:

```python
from serbot_control import MotionStep, SerBot

robot = SerBot.from_env()
robot.goal("forward", speed=10, seconds=1)
robot.sequence([
    MotionStep.vector(8, 0, seconds=0.5),
    MotionStep.vector(10, 3, seconds=0.7),
])
print(robot.goal("ultra"))
```

See `SERBOT_CONTROL.md` for the full API.

Safety notes:

- Forward/backward commands use an ultrasonic stop guard by default.
- The default guard threshold is `20 cm`.
- Use `--no-guard` or `--allow-blind` only when you explicitly want to bypass the fail-safe.
- `99` is not a safe default on this platform.

Deploy from this workspace:

```bash
chmod +x tools/deploy_serbot_runtime.sh
SERBOT_PASSWORD=soda ./tools/deploy_serbot_runtime.sh soda@192.168.0.7
```

Then on the robot:

```bash
python3 /home/soda/ros_ws/tools/serbot_runtime.py probe
```
