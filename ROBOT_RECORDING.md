# SerBOT 로봇 내부 녹화 및 센서 기록

이 명령은 **Jetson에 SSH로 접속한 뒤** 실행합니다. 영상과 센서값은 로봇 저장장치에 남으며, 맥이나 GitHub로 전송하지 않습니다. 녹화는 바퀴 제어 명령을 보내지 않습니다.

## 30초 기록

Jetson에서 처음 한 번 저장소를 내려받습니다. 이미 받아 뒀다면 해당 폴더에서 최신 버전을 받으세요.

~~~bash
git clone https://github.com/tlstngud/tteooreum-robot.git
cd tteooreum-robot
~~~

카메라에 접근할 수 있고 LiDAR의 ROS 2 토픽이 발행 중일 때 실행합니다. ROS 2 Foxy가 아닌 경우 첫 줄의 경로를 실제 배포판으로 바꿉니다.

~~~bash
source /opt/ros/foxy/setup.bash
python3 tools/serbot_record.py --seconds 30
~~~

기본 저장 경로는 Jetson의 `~/serbot-recordings/`입니다. 실행마다 고유한 하위 폴더를 만들고 경로를 `recording_dir=`로 출력합니다. 다른 디스크에 저장하려면 `--output-dir /경로`를 사용하세요. `Ctrl+C`로 중단하면 기록한 데이터를 닫아 보존합니다. 요청한 입력 중 하나라도 오류가 나거나 한 건도 기록되지 않으면 종료코드 1과 `manifest.json`의 `incomplete` 상태로 표시합니다.

| 파일 | 내용 |
| --- | --- |
| `camera.avi` | 영상 (MJPG AVI, 기본 640×360 / 10 fps 목표, 오디오 없음) |
| `camera_frames.jsonl` | AVI의 0부터 시작하는 프레임 번호와 수신 시각 |
| `ultrasonic.jsonl` | 원본 배열과 전·후방 센서값 (cm, 기본 초당 5회 요청) |
| `lidar.jsonl` | ROS 2 LaserScan 연속 기록 (거리 m, 각도 rad, 비유한 값은 null) |
| `manifest.json` | 실제 프레임·샘플 수, 시작·종료 시각, 입력별 오류 |

JSONL은 한 줄에 JSON 객체 하나입니다. 세 입력의 `monotonic_ns`는 같은 Jetson 시계를 사용하므로 나중에 가까운 시각의 데이터를 찾을 수 있습니다. `unix_ns`는 수신 당시의 시스템 시각이고, LiDAR의 `header_stamp`는 발행자가 찍은 별도 시각입니다. 하드웨어 동기화는 아니며 고정 FPS인 AVI의 재생 시간 대신 프레임 JSONL로 시각을 맞추세요.

## 센서 하나만 기록하거나 카메라 스트림 재사용

~~~bash
# 초음파만 30초
python3 tools/serbot_record.py --sensors ultrasonic --seconds 30

# ROS 2 환경을 불러온 상태에서 LiDAR만 30초
python3 tools/serbot_record.py --sensors lidar --scan-topic /scan --seconds 30

# 카메라 스트림이 이미 켜져 있으면 장치를 다시 열지 않고 로컬 스트림 사용
python3 tools/serbot_record.py --stream-port 8766 --seconds 30
~~~

- 카메라 직접 접근에는 Jetson의 OpenCV가 필요합니다. 기본 장치가 다르면 `--camera-device /dev/video0` 값을 변경합니다. 이미 대시보드가 카메라를 사용하면 `--stream-port 8766`으로 재사용합니다.
- 초음파는 기본적으로 Jetson에 설치된 `pop`을 사용합니다. 그 이미지에서 SocketCAN으로 읽어야 한다면 `--ultra-backend can --can-channel can0`을 사용합니다. 전·후방 분류는 기존 SerBOT 런타임의 백엔드별 매핑을 따릅니다.
- LiDAR는 실행 중인 드라이버의 `/scan`을 구독합니다. `ros2 topic list`에 토픽이 없다면 먼저 해당 로봇의 LiDAR 드라이버를 시작하세요. 로거가 드라이버를 새로 시작하거나 종료하지 않습니다.
- 카메라가 켜진 공간에서는 촬영 동의를 받고, 기록 파일을 공개 저장소에 올리지 마세요.
