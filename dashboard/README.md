# SerBot Localhost Dashboard

이 폴더는 `localhost`에서 띄우는 제어 대시보드입니다.

포함 기능:

- 방향 제어 버튼
- 속도 / 지속시간 조절
- 블록 코딩 기반 Python 실행실
- 초음파 상태 카드
- 점유 맵 뷰어
- 최근 명령 로그
- SSH 브리지 설정
- 직접 MJPEG 카메라 스트림 시작 / 중지 / 재연결 패널

실행:

```bash
cd /Users/sinsuhyeong/Downloads/AIoT-SerBOT
python3 dashboard/server.py
```

브라우저:

```text
http://127.0.0.1:8765
```

SSH 브리지 없이 데모 모드로 먼저 뜹니다.

SSH 제어를 바로 켜고 싶으면:

```bash
python3 dashboard/server.py \
  --bridge-mode ssh \
  --ssh-target soda@threnody.local \
  --ssh-password soda \
  --runtime-path /home/soda/ros_ws/tools/serbot_runtime.py \
  --camera-stream-url http://threnody.local:8766/stream.mjpg
```

페이지 안에서도 같은 설정을 바꿀 수 있습니다.

블록 코딩:

- 왼쪽 블록 팔레트에서 `이동`, `정밀 주행`, `기다리기`, `초음파`, `사진`, `LiDAR`, `정지` 블록을 추가합니다.
- 블록 순서를 위/아래 버튼으로 조정합니다.
- `블록 실행`을 누르면 Python 코드가 자동 생성되고 서버에서 실행됩니다.
- 속도 입력 UI와 서버 제어 API는 최대 `10`으로 제한합니다.

카메라 스트림 서버는 로봇이 연결된 뒤 별도로 시작합니다.

```bash
./goal camera-stream start --port 8766 --width 640 --height 360 --fps 10 --quality 70
./goal camera-stream url
```

대시보드 안의 `로봇 스트림 시작` 버튼도 같은 MJPEG 서버를 SSH로 배포하고 실행합니다.

전체 사용 설명서는 `docs/serbot-control-guide.html`에 있습니다.

주의:

- SSH 브리지는 로봇 쪽에 `serbot_runtime.py`가 있어야 합니다.
- 카메라 패널은 스트림 URL이 있으면 로봇의 MJPEG 서버에 직접 연결합니다.
- 스트림 URL이 없으면 기존 SSH 스냅샷 경로를 fallback으로 사용합니다.
- 로봇에 `/dev/video0` 또는 OpenCV가 없으면 패널에는 안내용 placeholder가 보입니다.
- 맵 업로드는 `nav_msgs/OccupancyGrid` 스타일 JSON 또는 `width/height/resolution/data` 형식 JSON을 받습니다.
- 코드 실행은 이 Mac의 localhost 서버에서 실행되며, `SERBOT_TARGET`, `SERBOT_PASSWORD`, `SERBOT_RUNTIME`, `SERBOT_CAMERA_STREAM` 환경값은 SSH 브리지 설정을 따라갑니다.
