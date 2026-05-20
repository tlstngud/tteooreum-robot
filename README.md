# 떠오름 로봇

Jetson 기반 Hanback AIoT SerBot을 맥/PC에서 쉽게 제어하기 위한 프로젝트입니다.

이 프로젝트는 세 가지를 제공합니다.

- `serbot_control`: Python에서 로봇을 제어하는 라이브러리
- `goal`: 터미널에서 바로 쓰는 명령어
- `dashboard`: 브라우저에서 쓰는 한국어 로봇 관제실, 블록 코딩, 카메라, 센서 화면

기본 안전 원칙은 단순합니다. 처음에는 속도 `10` 이하만 사용합니다.

## 이번 버전에서 강화된 점

- `goal config`: 현재 설정을 비밀번호 없이 확인합니다.
- `goal discover`: 알려진 로봇 이름과 IP의 SSH 포트를 탐색합니다.
- `goal doctor`: 모터를 움직이지 않고 SSH, 런타임, CAN, 카메라, LiDAR 장치를 진단합니다.
- `goal deploy-all`: 모터/초음파 런타임과 카메라 스트림 서버를 한 번에 배포합니다.
- 모든 주행 계열 명령은 기본적으로 속도 `10`을 넘으면 실행 전에 차단합니다.
- 너무 긴 주행 시간도 기본적으로 `10초`를 넘기면 차단합니다.
- Python 라이브러리와 CLI가 같은 안전 정책을 공유합니다.

## 폴더 구조

```text
.
├── serbot_control/            # 맥/PC에서 실행하는 Python 제어 라이브러리
├── goal                       # 터미널 명령어 실행 파일
├── dashboard/                 # localhost 웹 대시보드와 블록 코딩 UI
├── tools/                     # 로봇 Jetson에 올리는 저수준 런타임과 카메라 서버
├── examples/                  # Python 예제
├── docs/                      # HTML 사용설명서
├── SERBOT_CONTROL.md          # CLI/Python API 요약 문서
└── README.md                  # 이 문서
```

## 전체 그림

로봇 제어는 이렇게 흘러갑니다.

```text
내 맥/PC
  ├─ ./goal 명령어
  ├─ Python 코드: from serbot_control import SerBot
  └─ 웹 대시보드: http://127.0.0.1:8765
        ↓ SSH
Jetson 로봇
  ├─ /home/soda/ros_ws/tools/serbot_runtime.py
  ├─ CAN 모터 제어
  ├─ 초음파 센서
  ├─ LiDAR
  └─ MJPEG 카메라 스트림
```

맥/PC에서는 직접 모터 핀을 건드리지 않습니다. 맥/PC가 SSH로 Jetson에 명령을 보내고, Jetson 안의 `serbot_runtime.py`가 실제 모터와 센서를 다룹니다.

## 설치

Python 3.10 이상을 권장합니다.

```bash
cd /Users/sinsuhyeong/Downloads/AIoT-SerBOT
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -U pip
python3 -m pip install -e .
```

설치 후에는 `goal` 명령어를 바로 쓸 수 있습니다.

```bash
goal --help
```

설치하지 않고도 프로젝트 폴더에서는 아래처럼 실행할 수 있습니다.

```bash
./goal --help
python3 -m serbot_control --help
```

## 로봇 연결 설정

기본 대상은 아래처럼 잡혀 있습니다.

```text
SSH target: soda@threnody.local
Robot runtime: /home/soda/ros_ws/tools/serbot_runtime.py
Camera stream: /home/soda/ros_ws/tools/camera_stream_server.py
```

내 로봇 주소가 다르면 환경변수로 바꿉니다.

```bash
export SERBOT_TARGET=soda@threnody.local
export SERBOT_PASSWORD=비밀번호
export SERBOT_RUNTIME=/home/soda/ros_ws/tools/serbot_runtime.py
export SERBOT_CAMERA_STREAM=/home/soda/ros_ws/tools/camera_stream_server.py
export SERBOT_ROS_DISTRO=foxy
```

비밀번호 SSH를 쓰면 로컬에 `sshpass`가 필요할 수 있습니다. 키 기반 SSH를 설정했다면 `SERBOT_PASSWORD`를 비워도 됩니다.

안전 제한도 환경변수로 바꿀 수 있습니다.

```bash
export SERBOT_SAFE_MAX_SPEED=10
export SERBOT_MAX_MOTION_SECONDS=10
```

정말 제한을 풀어야 할 때만 아래 값을 씁니다.

```bash
export SERBOT_ALLOW_UNSAFE_SPEED=1
```

이 값은 수업/실험 기본값으로 권장하지 않습니다.

## 처음 연결할 때

1. 로봇과 맥/PC가 같은 공유기에 있는지 확인합니다.

2. 현재 설정을 확인합니다.

```bash
./goal config
```

3. 로봇을 자동 탐색합니다.

```bash
./goal discover
```

4. SSH 연결을 확인합니다.

```bash
./goal status
```

5. 로봇 쪽 런타임과 카메라 서버를 배포합니다.

```bash
./goal deploy-all
```

6. 모터는 움직이지 않고 전체 상태를 진단합니다.

```bash
./goal doctor
```

7. 모터는 움직이지 않고 CAN 정지와 초음파만 확인합니다.

```bash
./goal probe
```

`can_stop=ok`와 초음파 배열이 보이면 기본 통신은 되는 상태입니다.

## 가장 쉬운 제어: goal 명령어

전진, 후진, 좌회전, 우회전은 한 줄입니다.

```bash
./goal forward --speed 6 --seconds 0.5
./goal backward --speed 6 --seconds 0.5
./goal spin-left --speed 5 --seconds 0.4
./goal spin-right --speed 5 --seconds 0.4
./goal stop
```

속도 `10`을 넘기면 로봇에 SSH 명령을 보내기 전에 막힙니다.

```bash
./goal forward --speed 11 --seconds 0.5
```

위 명령은 기본 설정에서는 실행되지 않습니다. 실수로 과속 명령을 보내는 것을 막기 위한 장치입니다.

초음파, 카메라, LiDAR도 한 줄입니다.

```bash
./goal ultra --count 3
./goal camera --output artifacts/camera/latest.jpg
./goal lidar --output artifacts/lidar/latest-scan.json
```

카메라를 실시간처럼 보고 싶으면 SSH로 프레임을 계속 가져오지 말고, 로봇에서 MJPEG 서버를 띄웁니다.

```bash
./goal camera-stream start --port 8766 --width 640 --height 360 --fps 10 --quality 70
./goal camera-stream url
```

브라우저에서 출력된 URL을 열면 됩니다.

## goal 명령어 전체 요약

| 명령어 | 움직임 여부 | 설명 |
| --- | --- | --- |
| `./goal config` | 안 움직임 | 현재 설정을 비밀번호 없이 출력합니다. |
| `./goal discover` | 안 움직임 | 후보 주소의 SSH 포트를 찾습니다. |
| `./goal status` | 안 움직임 | SSH 접속과 로봇 기본 정보를 확인합니다. |
| `./goal doctor` | 안 움직임 | 런타임, CAN, 카메라, LiDAR 장치를 진단합니다. |
| `./goal deploy-all` | 안 움직임 | Jetson에 필요한 런타임 파일을 배포합니다. |
| `./goal probe` | 바퀴 정지 프레임만 전송 | CAN 정지와 초음파 값을 확인합니다. |
| `./goal forward` | 움직임 | 전진합니다. |
| `./goal backward` | 움직임 | 후진합니다. |
| `./goal spin-left` | 움직임 | 제자리 좌회전합니다. |
| `./goal spin-right` | 움직임 | 제자리 우회전합니다. |
| `./goal drive` | 움직임 | 선속도/회전값으로 정밀 제어합니다. |
| `./goal sequence` | 움직임 가능 | 여러 동작을 Jetson 안에서 이어서 실행합니다. |
| `./goal ultra` | 안 움직임 | 초음파 센서값을 읽습니다. |
| `./goal camera` | 안 움직임 | 카메라 JPEG 한 장을 저장합니다. |
| `./goal camera-stream` | 안 움직임 | MJPEG 스트림 서버를 관리합니다. |
| `./goal lidar` | 안 움직임 | LiDAR 한 프레임을 읽습니다. |

## 웹 대시보드

대시보드는 한국어 UI로 만든 로봇 관제실입니다.

```bash
python3 dashboard/server.py
```

브라우저에서 엽니다.

```text
http://127.0.0.1:8765
```

대시보드에서 할 수 있는 일은 다음과 같습니다.

- 방향 버튼으로 로봇 제어
- 속도와 주행 시간 조절
- 속도 최대값 `10` 제한
- 초음파 센서값 확인
- 카메라 스트림 시작, 중지, 재연결
- 맵 JSON 보기
- 블록 코딩으로 로봇 동작 만들기
- 생성된 Python 코드 확인 및 실행

## 블록 코딩

대시보드의 블록 코딩은 Scratch처럼 완전한 블록 언어는 아니지만, 로봇 수업용으로 바로 쓸 수 있게 만들었습니다.

사용 방법은 간단합니다.

1. 왼쪽 블록 팔레트에서 블록을 누릅니다.
2. 실행 순서에 블록이 추가됩니다.
3. 속도, 방향, 시간 같은 값을 바꿉니다.
4. `블록 실행`을 누릅니다.
5. 아래 Python 코드가 자동으로 만들어지고 실행됩니다.

지원 블록은 다음과 같습니다.

| 블록 | 의미 |
| --- | --- |
| 이동 | 전진, 후진, 좌회전, 우회전을 합니다. |
| 정밀 주행 | `linear`, `angular` 값으로 더 세밀하게 움직입니다. |
| 기다리기 | 지정한 초만큼 쉽니다. |
| 초음파 읽기 | 초음파 센서값을 출력합니다. |
| 사진 저장 | 카메라 사진을 파일로 저장합니다. |
| LiDAR 읽기 | LiDAR 한 프레임을 읽고 요약합니다. |
| 정지 | 모터 정지 명령을 보냅니다. |

블록을 만들면 이런 Python 코드가 자동 생성됩니다.

```python
from serbot_control import SerBot
import time

robot = SerBot.from_env()
print("대상:", robot.config.target)

print(robot.ultrasonic(count=1))
print(robot.goal("forward", speed=6, seconds=0.5))
print(robot.stop())
```

## C를 막 배운 사람을 위한 Python 설명

C를 조금 배웠다면 아래처럼 생각하면 됩니다.

### 1. `import`는 C의 `#include`와 비슷합니다

C에서는 함수를 쓰기 전에 헤더를 포함합니다.

```c
#include <stdio.h>
```

Python에서는 `import`를 씁니다.

```python
from serbot_control import SerBot
```

뜻은 `serbot_control` 라이브러리에서 `SerBot`이라는 로봇 제어 도구를 가져오겠다는 말입니다.

### 2. `robot = SerBot.from_env()`는 로봇 조종기를 만드는 줄입니다

```python
robot = SerBot.from_env()
```

이 줄은 환경변수에 적힌 SSH 주소와 비밀번호를 읽어서 로봇 조종기 객체를 만듭니다.

C식으로 비유하면 구조체 포인터를 하나 준비하는 느낌입니다.

```c
Robot robot = robot_init_from_env();
```

### 3. `robot.forward(...)`는 함수 호출입니다

```python
robot.forward(speed=6, seconds=0.5)
```

이 코드는 로봇에게 `0.5초 동안 속도 6으로 전진하라`고 명령합니다.

C식으로 쓰면 이런 느낌입니다.

```c
robot_forward(&robot, 6, 0.5);
```

Python은 인자 이름을 같이 쓸 수 있어서 더 읽기 쉽습니다.

```python
speed=6       # 속도
seconds=0.5   # 시간
```

### 4. `print(...)`는 C의 `printf(...)`와 비슷합니다

```python
print(robot.ultrasonic(count=1))
```

초음파 값을 읽고 화면에 출력합니다.

C의 `printf`처럼 결과를 눈으로 확인할 때 씁니다.

### 5. Python은 줄 끝에 세미콜론을 쓰지 않습니다

C:

```c
int speed = 6;
```

Python:

```python
speed = 6
```

### 6. Python은 들여쓰기가 중요합니다

C는 `{}`로 블록을 표시합니다.

```c
if (speed > 10) {
    speed = 10;
}
```

Python은 들여쓰기로 블록을 표시합니다.

```python
if speed > 10:
    speed = 10
```

## Python 라이브러리 기본 사용법

가장 기본 코드는 이렇습니다.

```python
from serbot_control import SerBot

robot = SerBot.from_env()

print(robot.status())
print(robot.forward(speed=6, seconds=0.5))
print(robot.stop())
```

후진:

```python
robot.backward(speed=6, seconds=0.5)
robot.stop()
```

진단부터 하는 코드는 이렇습니다.

```python
from serbot_control import SerBot

robot = SerBot.from_env()

print(robot.config_snapshot())
print(robot.doctor())
```

로봇 후보 주소를 찾는 코드는 이렇습니다.

```python
from serbot_control import SerBot

for item in SerBot.discover_targets():
    print(item)
```

좌회전:

```python
robot.spin_left(speed=5, seconds=0.4)
robot.stop()
```

우회전:

```python
robot.spin_right(speed=5, seconds=0.4)
robot.stop()
```

초음파 읽기:

```python
ultra = robot.ultrasonic(count=3)
print(ultra)
```

카메라 한 장 저장:

```python
robot.camera_frame("artifacts/camera/my-photo.jpg")
```

LiDAR 한 프레임 읽기:

```python
scan = robot.lidar_scan()
print(scan["summary"])
```

## 조금 더 세밀한 제어

`drive`는 전진/후진과 회전을 섞어서 제어합니다.

```python
robot.drive(linear=6, angular=0, seconds=0.5)
robot.drive(linear=6, angular=2, seconds=0.5)
robot.drive(linear=0, angular=5, seconds=0.4, guard_direction="none")
```

의미는 다음과 같습니다.

| 값 | 의미 |
| --- | --- |
| `linear > 0` | 앞으로 가려는 힘 |
| `linear < 0` | 뒤로 가려는 힘 |
| `angular > 0` | 왼쪽으로 돌려는 힘 |
| `angular < 0` | 오른쪽으로 돌려는 힘 |

`raw`는 바퀴 값을 직접 넣습니다.

```python
robot.wheels(left=6, right=-6, seconds=0.4)
```

이 로봇은 좌우 바퀴 부호가 거울처럼 동작합니다.

| 동작 | 왼쪽 바퀴 | 오른쪽 바퀴 |
| --- | ---: | ---: |
| 전진 | 양수 | 음수 |
| 후진 | 음수 | 양수 |
| 오른쪽 제자리 회전 | 양수 | 양수 |
| 왼쪽 제자리 회전 | 음수 | 음수 |

처음에는 `raw`보다 `forward`, `backward`, `drive`를 쓰는 것이 안전합니다.

## 여러 동작을 한 번에 실행하기

SSH 명령을 여러 번 보내면 반응이 끊겨 보일 수 있습니다. 짧은 동작을 여러 개 연결할 때는 `sequence`를 씁니다.

```python
from serbot_control import MotionStep, SerBot

robot = SerBot.from_env()

robot.sequence([
    MotionStep.vector(6, 0, seconds=0.4),
    MotionStep.vector(6, 2, seconds=0.4),
    MotionStep.wheels(0, 0, seconds=0.1, guard_direction="none"),
])
```

`MotionStep.vector(6, 2, seconds=0.4)`는 `linear=6`, `angular=2`로 `0.4초` 움직이라는 뜻입니다.

## 중요한 안전 규칙

- 처음에는 속도 `10` 이하만 씁니다.
- 처음에는 시간을 `0.3초 ~ 0.8초` 정도로 짧게 씁니다.
- 이상하면 바로 `./goal stop`을 보냅니다.
- `99` 같은 최대 속도는 쓰지 않습니다.
- 초음파는 얇은 책상 다리 같은 물체를 놓칠 수 있습니다.
- 로봇 앞을 손이나 얼굴로 막고 테스트하지 않습니다.
- 바닥에 케이블이 걸리지 않게 합니다.

## 개발자가 알아야 할 구현 메모

모터 제어는 `pop.Pilot` 대신 저수준 CAN 프레임을 사용합니다.

- 모터 CAN ID: `0x101`
- 초음파 enable CAN ID: `0x114`
- 초음파 data CAN ID: `0x201`
- 전진 부호: `left=+speed`, `right=-speed`

로봇 쪽 저수준 런타임:

```text
/home/soda/ros_ws/tools/serbot_runtime.py
```

맥/PC 쪽 고수준 라이브러리:

```text
serbot_control/client.py
```

## 검증 명령

문법 검사는 아래처럼 합니다.

```bash
python3 -m py_compile serbot_control/*.py tools/*.py dashboard/server.py examples/*.py
node --check dashboard/static/app.js
```

대시보드 서버 확인:

```bash
python3 dashboard/server.py
curl http://127.0.0.1:8765/api/status
```

## 더 자세한 문서

- `SERBOT_CONTROL.md`: CLI와 Python API 요약
- `dashboard/README.md`: 웹 대시보드 설명
- `tools/README.md`: Jetson 런타임 설명
- `docs/serbot-control-guide.html`: 브라우저로 보는 한국어 HTML 설명서
