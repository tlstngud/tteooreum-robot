const MAX_SAFE_SPEED = 10;

const state = {
  status: null,
  mapPayload: null,
  cameraPolling: false,
  cameraTimer: null,
  cameraBusy: false,
  zoomPercent: 100,
  blocks: [],
  nextBlockId: 1,
};

const refs = {};

const DIRECTION_LABELS = {
  forward: "전진",
  backward: "후진",
  "spin-left": "좌회전",
  "spin-right": "우회전",
  stop: "정지",
};

const CONNECTION_LABELS = {
  "mock-ready": "로컬 대기",
  "command-sent": "명령 전송",
  "stop-requested": "정지 요청",
  "bridge-error": "브리지 오류",
  "ssh-live": "원격 연결 중",
};

const CODE_TEMPLATES = {
  status: `from serbot_control import SerBot

robot = SerBot.from_env()
print(robot.status())
print(robot.ultrasonic(count=1))
`,
  safeMove: `from serbot_control import SerBot

robot = SerBot.from_env()
print(robot.forward(speed=6, seconds=0.5))
print(robot.stop())
`,
  camera: `from serbot_control import SerBot

robot = SerBot.from_env()
robot.camera_frame("artifacts/camera/dashboard-snapshot.jpg")
print("저장 완료: artifacts/camera/dashboard-snapshot.jpg")
`,
  lidar: `from serbot_control import SerBot

robot = SerBot.from_env()
scan = robot.lidar_scan()
print(scan.get("summary", scan))
`,
};

document.addEventListener("DOMContentLoaded", async () => {
  bindRefs();
  bindEvents();
  seedBlocks();
  syncSliders();
  await refreshAll();
  setCameraPolling(true);
  setInterval(refreshStatus, 1200);
  setInterval(refreshMap, 1800);
});

function bindRefs() {
  [
    "bridgeStatePill",
    "bridgeSummary",
    "uptimeText",
    "mapName",
    "mapFrame",
    "poseText",
    "batteryText",
    "speedRange",
    "speedValue",
    "durationRange",
    "durationValue",
    "controlFlash",
    "bridgeForm",
    "bridgeMode",
    "sshTarget",
    "sshPassword",
    "runtimePath",
    "cameraStreamPath",
    "cameraStreamUrl",
    "bridgeMessage",
    "frontLeftValue",
    "frontRightValue",
    "leftValue",
    "rightValue",
    "rearLeftValue",
    "rearRightValue",
    "bridgeTarget",
    "bridgeError",
    "resolutionText",
    "mapSizeText",
    "cameraFrame",
    "cameraToggleButton",
    "cameraRefreshButton",
    "streamStartButton",
    "streamStopButton",
    "streamStablePreset",
    "streamFastPreset",
    "streamWidth",
    "streamHeight",
    "streamFps",
    "streamQuality",
    "streamUrlOpen",
    "streamUrlText",
    "cameraStatus",
    "mapFileInput",
    "zoomRange",
    "mapCanvas",
    "logList",
    "resetDemoButton",
    "blockPalette",
    "blockProgram",
    "clearBlocksButton",
    "runBlocksButton",
    "codeTemplateSelect",
    "codeTimeout",
    "runCodeButton",
    "copyCodeButton",
    "clearOutputButton",
    "codeEditor",
    "codeOutput",
    "codeStatus",
  ].forEach((id) => {
    refs[id] = document.getElementById(id);
  });
}

function bindEvents() {
  document.querySelectorAll("[data-direction]").forEach((button) => {
    button.addEventListener("click", () => sendControl(button.dataset.direction));
  });

  document.querySelectorAll(".preset-btn").forEach((button) => {
    button.addEventListener("click", () => {
      refs.speedRange.value = Math.min(MAX_SAFE_SPEED, Number(button.dataset.speed));
      refs.durationRange.value = Math.round(Number(button.dataset.duration) * 10);
      syncSliders();
    });
  });

  refs.speedRange.addEventListener("input", syncSliders);
  refs.durationRange.addEventListener("input", syncSliders);
  refs.zoomRange.addEventListener("input", () => {
    state.zoomPercent = Number(refs.zoomRange.value);
    renderMap();
  });

  refs.bridgeForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    await applyBridgeSettings();
  });

  refs.mapFileInput.addEventListener("change", importMapFile);
  refs.resetDemoButton.addEventListener("click", resetDemoMap);

  refs.cameraToggleButton.addEventListener("click", () => setCameraPolling(!state.cameraPolling));
  refs.cameraRefreshButton.addEventListener("click", refreshCameraFrame);
  refs.streamStartButton.addEventListener("click", startCameraStream);
  refs.streamStopButton.addEventListener("click", stopCameraStream);
  refs.streamStablePreset.addEventListener("click", () => setStreamPreset(640, 360, 10, 70));
  refs.streamFastPreset.addEventListener("click", () => setStreamPreset(424, 240, 30, 65));

  refs.blockPalette.addEventListener("click", (event) => {
    const button = event.target.closest("[data-block-type]");
    if (!button) return;
    addBlock(button.dataset.blockType);
  });
  refs.blockProgram.addEventListener("input", handleBlockInput);
  refs.blockProgram.addEventListener("click", handleBlockAction);
  refs.clearBlocksButton.addEventListener("click", () => {
    state.blocks = [];
    renderBlocks();
    generateCodeFromBlocks();
  });
  refs.runBlocksButton.addEventListener("click", async () => {
    generateCodeFromBlocks();
    await runCode();
  });

  refs.codeTemplateSelect.addEventListener("change", () => {
    refs.codeEditor.value = CODE_TEMPLATES[refs.codeTemplateSelect.value] || CODE_TEMPLATES.status;
  });
  refs.runCodeButton.addEventListener("click", runCode);
  refs.copyCodeButton.addEventListener("click", copyCode);
  refs.clearOutputButton.addEventListener("click", () => {
    refs.codeOutput.textContent = "출력을 비웠습니다.";
  });

  window.addEventListener("keydown", handleKeydown);
  new ResizeObserver(renderMap).observe(refs.mapCanvas);
}

function seedBlocks() {
  state.blocks = [
    makeBlock("ultra"),
    makeBlock("stop"),
  ];
  renderBlocks();
  generateCodeFromBlocks();
}

function makeBlock(type, patch = {}) {
  const base = {
    id: state.nextBlockId++,
    type,
    direction: "forward",
    speed: 6,
    seconds: 0.5,
    linear: 6,
    angular: 0,
    output: "artifacts/camera/dashboard-block.jpg",
  };
  return { ...base, ...patch };
}

function addBlock(type) {
  state.blocks.push(makeBlock(type));
  renderBlocks();
  generateCodeFromBlocks();
}

function renderBlocks() {
  if (!state.blocks.length) {
    refs.blockProgram.innerHTML = `<div class="hint">왼쪽 블록을 눌러 실행 순서를 만드세요.</div>`;
    return;
  }
  refs.blockProgram.innerHTML = state.blocks.map(renderBlock).join("");
}

function renderBlock(block) {
  const typeLabels = {
    move: "이동",
    drive: "정밀",
    wait: "대기",
    ultra: "초음파",
    camera: "사진",
    lidar: "LiDAR",
    stop: "정지",
  };
  return `
    <div class="block-row" data-block-id="${block.id}">
      <strong>${typeLabels[block.type] || block.type}</strong>
      <div class="block-fields">${renderBlockFields(block)}</div>
      <div class="block-actions">
        <button data-block-action="up" title="위로">↑</button>
        <button data-block-action="down" title="아래로">↓</button>
        <button data-block-action="delete" title="삭제">×</button>
      </div>
    </div>
  `;
}

function renderBlockFields(block) {
  if (block.type === "move") {
    return `
      <label>방향 ${directionSelect(block.direction)}</label>
      <label>속도 <input data-field="speed" type="number" min="0" max="${MAX_SAFE_SPEED}" value="${block.speed}" /></label>
      <label>초 <input data-field="seconds" type="number" min="0.1" max="10" step="0.1" value="${block.seconds}" /></label>
    `;
  }
  if (block.type === "drive") {
    return `
      <label>선속도 <input data-field="linear" type="number" min="-${MAX_SAFE_SPEED}" max="${MAX_SAFE_SPEED}" value="${block.linear}" /></label>
      <label>회전 <input data-field="angular" type="number" min="-${MAX_SAFE_SPEED}" max="${MAX_SAFE_SPEED}" value="${block.angular}" /></label>
      <label>초 <input data-field="seconds" type="number" min="0.1" max="10" step="0.1" value="${block.seconds}" /></label>
    `;
  }
  if (block.type === "wait") {
    return `<label>초 <input data-field="seconds" type="number" min="0.1" max="10" step="0.1" value="${block.seconds}" /></label>`;
  }
  if (block.type === "camera") {
    return `<label>저장 위치 <input data-field="output" type="text" value="${escapeHtml(block.output)}" /></label>`;
  }
  if (block.type === "ultra") return `<span class="hint">초음파 값을 한 번 읽어서 출력합니다.</span>`;
  if (block.type === "lidar") return `<span class="hint">LiDAR /scan 한 프레임을 읽고 요약합니다.</span>`;
  if (block.type === "stop") return `<span class="hint">모터 정지 명령을 보냅니다.</span>`;
  return "";
}

function directionSelect(value) {
  const options = ["forward", "backward", "spin-left", "spin-right"]
    .map((direction) => `<option value="${direction}" ${direction === value ? "selected" : ""}>${DIRECTION_LABELS[direction]}</option>`)
    .join("");
  return `<select data-field="direction">${options}</select>`;
}

function handleBlockInput(event) {
  const row = event.target.closest("[data-block-id]");
  const field = event.target.dataset.field;
  if (!row || !field) return;
  const block = findBlock(row.dataset.blockId);
  if (!block) return;
  if (["speed", "linear", "angular"].includes(field)) {
    block[field] = clamp(Number(event.target.value), -MAX_SAFE_SPEED, MAX_SAFE_SPEED);
    if (field === "speed") block[field] = clamp(block[field], 0, MAX_SAFE_SPEED);
  } else if (field === "seconds") {
    block[field] = clamp(Number(event.target.value), 0.1, 10);
  } else {
    block[field] = event.target.value;
  }
  generateCodeFromBlocks();
}

function handleBlockAction(event) {
  const button = event.target.closest("[data-block-action]");
  const row = event.target.closest("[data-block-id]");
  if (!button || !row) return;
  const index = state.blocks.findIndex((block) => String(block.id) === String(row.dataset.blockId));
  if (index < 0) return;
  const action = button.dataset.blockAction;
  if (action === "delete") {
    state.blocks.splice(index, 1);
  } else if (action === "up" && index > 0) {
    [state.blocks[index - 1], state.blocks[index]] = [state.blocks[index], state.blocks[index - 1]];
  } else if (action === "down" && index < state.blocks.length - 1) {
    [state.blocks[index + 1], state.blocks[index]] = [state.blocks[index], state.blocks[index + 1]];
  }
  renderBlocks();
  generateCodeFromBlocks();
}

function findBlock(id) {
  return state.blocks.find((block) => String(block.id) === String(id));
}

function generateCodeFromBlocks() {
  const lines = [
    "from serbot_control import SerBot",
    "import time",
    "",
    "robot = SerBot.from_env()",
    "print('대상:', robot.config.target)",
  ];

  state.blocks.forEach((block, index) => {
    lines.push("", `# ${index + 1}. ${block.type}`);
    if (block.type === "move") {
      const direction = ["forward", "backward", "spin-left", "spin-right"].includes(block.direction) ? block.direction : "forward";
      lines.push(`print(robot.goal(${JSON.stringify(direction)}, speed=${clamp(block.speed, 0, MAX_SAFE_SPEED)}, seconds=${clamp(block.seconds, 0.1, 10)}))`);
    } else if (block.type === "drive") {
      lines.push(`print(robot.drive(linear=${clamp(block.linear, -MAX_SAFE_SPEED, MAX_SAFE_SPEED)}, angular=${clamp(block.angular, -MAX_SAFE_SPEED, MAX_SAFE_SPEED)}, seconds=${clamp(block.seconds, 0.1, 10)}))`);
    } else if (block.type === "wait") {
      lines.push(`time.sleep(${clamp(block.seconds, 0.1, 10)})`);
    } else if (block.type === "ultra") {
      lines.push("print(robot.ultrasonic(count=1))");
    } else if (block.type === "camera") {
      const output = block.output || "artifacts/camera/dashboard-block.jpg";
      lines.push(`robot.camera_frame(${JSON.stringify(output)})`);
      lines.push(`print('사진 저장:', ${JSON.stringify(output)})`);
    } else if (block.type === "lidar") {
      lines.push("scan = robot.lidar_scan()");
      lines.push("print(scan.get('summary', scan))");
    } else if (block.type === "stop") {
      lines.push("print(robot.stop())");
    }
  });

  refs.codeEditor.value = `${lines.join("\n")}\n`;
}

function syncSliders() {
  const safeSpeed = Math.min(MAX_SAFE_SPEED, Number(refs.speedRange.value));
  refs.speedRange.value = safeSpeed;
  refs.speedValue.textContent = safeSpeed;
  refs.durationValue.textContent = `${(Number(refs.durationRange.value) / 10).toFixed(1)}초`;
}

function handleKeydown(event) {
  if (event.repeat) return;
  const targetTag = document.activeElement?.tagName;
  if (["INPUT", "SELECT", "TEXTAREA"].includes(targetTag)) return;
  const map = {
    ArrowUp: "forward",
    ArrowDown: "backward",
    ArrowLeft: "spin-left",
    ArrowRight: "spin-right",
    " ": "stop",
  };
  if (!map[event.key]) return;
  event.preventDefault();
  sendControl(map[event.key]);
}

async function refreshAll() {
  await Promise.all([refreshStatus(), refreshMap()]);
}

async function refreshStatus() {
  try {
    state.status = await apiFetch("/api/status");
    renderStatus();
    renderLogs();
  } catch (error) {
    flash(`상태 갱신 실패: ${error.message}`, "error");
  }
}

async function refreshMap() {
  try {
    state.mapPayload = await apiFetch("/api/map");
    renderMap();
  } catch (error) {
    flash(`맵 갱신 실패: ${error.message}`, "error");
  }
}

async function sendControl(direction) {
  const payload = {
    direction,
    speed: Math.min(MAX_SAFE_SPEED, Number(refs.speedRange.value)),
    duration: Number(refs.durationRange.value) / 10,
  };

  try {
    const result = await apiFetch("/api/control", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    const suffix = result.bridge_error ? ` / 브리지 오류: ${result.bridge_error}` : "";
    flash(`${DIRECTION_LABELS[direction] || direction} · 속도 ${payload.speed}${suffix}`, result.ok ? "ok" : "error");
    await refreshAll();
  } catch (error) {
    flash(`제어 실패: ${error.message}`, "error");
  }
}

async function applyBridgeSettings(silent = false) {
  const payload = {
    mode: refs.bridgeMode.value,
    target: refs.sshTarget.value.trim(),
    runtime_path: refs.runtimePath.value.trim(),
    camera_stream_path: refs.cameraStreamPath.value.trim(),
    camera_stream_url: refs.cameraStreamUrl.value.trim(),
  };
  if (refs.sshPassword.value) {
    payload.password = refs.sshPassword.value;
  }

  const result = await apiFetch("/api/bridge", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  refs.bridgeMessage.textContent = `브리지 적용: ${result.mode} / ${result.target || "미설정"}`;
  refs.sshPassword.value = "";
  if (!silent) flash("브리지 설정을 적용했습니다.", "ok");
  await refreshStatus();
  return result;
}

async function importMapFile(event) {
  const file = event.target.files?.[0];
  if (!file) return;
  try {
    const payload = JSON.parse(await file.text());
    const result = await apiFetch("/api/map/import", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    flash(`맵 로드 완료: ${result.name}`, "ok");
    await refreshAll();
  } catch (error) {
    flash(`맵 로드 실패: ${error.message}`, "error");
  } finally {
    refs.mapFileInput.value = "";
  }
}

async function resetDemoMap() {
  await apiFetch("/api/reset", { method: "POST" });
  flash("데모 맵으로 리셋했습니다.", "ok");
  await refreshAll();
}

function renderStatus() {
  if (!state.status) return;
  const { connection, robot, map, uptime_sec } = state.status;
  const telemetry = robot.telemetry;
  const displayState =
    connection.bridge.mode === "ssh" && !connection.last_bridge_error
      ? "ssh-live"
      : connection.state;

  refs.bridgeSummary.textContent = `브리지: ${connection.bridge.mode === "ssh" ? "SSH 원격" : "로컬 데모"}`;
  refs.uptimeText.textContent = `가동 시간: ${uptime_sec.toFixed(1)}초`;
  refs.bridgeStatePill.textContent = CONNECTION_LABELS[displayState] || displayState;
  refs.bridgeStatePill.className = `status-pill state-${displayState}`;

  refs.mapName.textContent = map.name;
  refs.mapFrame.textContent = map.frame_id;
  refs.poseText.textContent = `x ${robot.pose.x.toFixed(2)} / y ${robot.pose.y.toFixed(2)} / 방향 ${Math.round((robot.pose.heading * 180) / Math.PI)}°`;
  refs.batteryText.textContent = `${telemetry.battery_percent.toFixed(1)}%`;
  refs.resolutionText.textContent = `${map.resolution.toFixed(2)} m`;
  refs.mapSizeText.textContent = `${map.width} × ${map.height}`;
  refs.bridgeTarget.textContent = connection.bridge.target || "로컬 데모 모드";
  refs.bridgeError.textContent = connection.last_bridge_error || connection.bridge.last_camera_error || "없음";

  refs.frontLeftValue.textContent = `${telemetry.front_left_cm.toFixed(1)} cm`;
  refs.frontRightValue.textContent = `${telemetry.front_right_cm.toFixed(1)} cm`;
  refs.leftValue.textContent = `${telemetry.left_cm.toFixed(1)} cm`;
  refs.rightValue.textContent = `${telemetry.right_cm.toFixed(1)} cm`;
  refs.rearLeftValue.textContent = `${telemetry.rear_left_cm.toFixed(1)} cm`;
  refs.rearRightValue.textContent = `${telemetry.rear_right_cm.toFixed(1)} cm`;

  setIfIdle(refs.bridgeMode, connection.bridge.mode);
  setIfIdle(refs.sshTarget, connection.bridge.target || "");
  setIfIdle(refs.runtimePath, connection.bridge.runtime_path || "/home/soda/ros_ws/tools/serbot_runtime.py");
  setIfIdle(refs.cameraStreamPath, connection.bridge.camera_stream_path || "/home/soda/ros_ws/tools/camera_stream_server.py");
  setIfIdle(refs.cameraStreamUrl, connection.bridge.camera_stream_url || "");
  updateStreamLink(getCameraStreamUrl());

  if (!state.cameraPolling) {
    refs.cameraStatus.textContent = "실시간 영상이 중지되었습니다";
  } else if (connection.bridge.camera_stream_url) {
    refs.cameraStatus.textContent = `직접 스트리밍 연결 중`;
  } else if (connection.bridge.last_camera_error) {
    refs.cameraStatus.textContent = `영상 오류: ${connection.bridge.last_camera_error}`;
  } else if (connection.bridge.last_camera_at) {
    refs.cameraStatus.textContent = `스냅샷 수신 · ${new Date(connection.bridge.last_camera_at * 1000).toLocaleTimeString()}`;
  } else if (!state.cameraBusy) {
    refs.cameraStatus.textContent = "카메라 연결 중...";
  }
}

function renderLogs() {
  if (!state.status) return;
  refs.logList.innerHTML = "";
  state.status.logs.forEach((entry) => {
    const item = document.createElement("li");
    item.innerHTML = `
      <span class="log-kind">${escapeHtml(entry.kind)}</span>
      <span class="log-message">${escapeHtml(entry.message)}</span>
      <span class="log-time">${new Date(entry.timestamp * 1000).toLocaleTimeString()}</span>
    `;
    refs.logList.appendChild(item);
  });
}

function renderMap() {
  if (!state.mapPayload) return;
  const canvas = refs.mapCanvas;
  const context = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 800;
  const height = canvas.clientHeight || 460;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  context.clearRect(0, 0, width, height);

  const { map, pose, path } = state.mapPayload;
  const scale = Math.min((width - 48) / map.width, (height - 48) / map.height) * (state.zoomPercent / 100);
  const offsetX = (width - map.width * scale) / 2;
  const offsetY = (height - map.height * scale) / 2;

  context.fillStyle = "#111714";
  context.fillRect(0, 0, width, height);

  for (let y = 0; y < map.height; y += 1) {
    for (let x = 0; x < map.width; x += 1) {
      const value = map.data[y * map.width + x];
      if (value === 0) continue;
      if (value === -1) {
        context.fillStyle = "rgba(133, 144, 138, 0.42)";
      } else {
        const alpha = 0.2 + (value / 100) * 0.72;
        context.fillStyle = `rgba(237, 106, 50, ${alpha.toFixed(3)})`;
      }
      context.fillRect(offsetX + x * scale, offsetY + (map.height - y - 1) * scale, scale, scale);
    }
  }

  context.strokeStyle = "rgba(255,255,255,0.06)";
  context.lineWidth = 1;
  for (let gx = 0; gx <= map.width; gx += 6) {
    const x = offsetX + gx * scale;
    context.beginPath();
    context.moveTo(x, offsetY);
    context.lineTo(x, offsetY + map.height * scale);
    context.stroke();
  }
  for (let gy = 0; gy <= map.height; gy += 6) {
    const y = offsetY + gy * scale;
    context.beginPath();
    context.moveTo(offsetX, y);
    context.lineTo(offsetX + map.width * scale, y);
    context.stroke();
  }

  if (path.length > 1) {
    context.strokeStyle = "#00a383";
    context.lineWidth = Math.max(2, scale * 0.32);
    context.beginPath();
    path.forEach((point, index) => {
      const px = offsetX + (point.x / map.resolution) * scale;
      const py = offsetY + (map.height - point.y / map.resolution) * scale;
      if (index === 0) context.moveTo(px, py);
      else context.lineTo(px, py);
    });
    context.stroke();
  }

  drawRobot(context, offsetX, offsetY, scale, map, pose);
}

function drawRobot(context, offsetX, offsetY, scale, map, pose) {
  const cx = offsetX + (pose.x / map.resolution) * scale;
  const cy = offsetY + (map.height - pose.y / map.resolution) * scale;
  const radius = Math.max(10, scale * 0.9);
  context.save();
  context.translate(cx, cy);
  context.rotate(-pose.heading);
  context.fillStyle = "#fff7e8";
  context.beginPath();
  context.moveTo(radius * 1.28, 0);
  context.lineTo(-radius * 0.85, radius * 0.72);
  context.lineTo(-radius * 0.42, 0);
  context.lineTo(-radius * 0.85, -radius * 0.72);
  context.closePath();
  context.fill();
  context.strokeStyle = "#00a383";
  context.lineWidth = 3;
  context.beginPath();
  context.moveTo(0, 0);
  context.lineTo(radius * 3.2, 0);
  context.stroke();
  context.restore();
}

function setStreamPreset(width, height, fps, quality) {
  refs.streamWidth.value = width;
  refs.streamHeight.value = height;
  refs.streamFps.value = fps;
  refs.streamQuality.value = quality;
}

async function startCameraStream() {
  try {
    await applyBridgeSettings(true);
    const payload = {
      port: 8766,
      width: Number(refs.streamWidth.value),
      height: Number(refs.streamHeight.value),
      fps: Number(refs.streamFps.value),
      quality: Number(refs.streamQuality.value),
      deploy: true,
    };
    refs.cameraStatus.textContent = "로봇 스트림 시작 중...";
    const result = await apiFetch("/api/camera/stream/start", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    if (result.url) {
      refs.cameraStreamUrl.value = result.url;
      updateStreamLink(result.url);
      connectCameraStream(result.url);
    }
    refs.streamUrlText.textContent = result.running ? `스트림 실행 중: ${result.url}` : "스트림 시작 요청 완료";
    await refreshStatus();
  } catch (error) {
    refs.cameraStatus.textContent = `스트림 시작 실패: ${error.message}`;
    flash(`스트림 시작 실패: ${error.message}`, "error");
  }
}

async function stopCameraStream() {
  try {
    const result = await apiFetch("/api/camera/stream/stop", {
      method: "POST",
      body: JSON.stringify({ port: 8766 }),
    });
    refs.streamUrlText.textContent = `스트림 중지 완료 · port ${result.port}`;
    refs.cameraStatus.textContent = "스트림 중지됨";
    await refreshStatus();
  } catch (error) {
    flash(`스트림 중지 실패: ${error.message}`, "error");
  }
}

function scheduleNextCameraFrame(delayMs = 1000) {
  clearTimeout(state.cameraTimer);
  if (!state.cameraPolling) {
    state.cameraTimer = null;
    return;
  }
  state.cameraTimer = setTimeout(refreshCameraFrame, delayMs);
}

async function refreshCameraFrame() {
  const streamUrl = getCameraStreamUrl();
  if (streamUrl) {
    connectCameraStream(streamUrl);
    return;
  }
  if (state.cameraBusy) return;
  state.cameraBusy = true;
  refs.cameraStatus.textContent = "카메라 스냅샷 갱신 중...";
  refs.cameraFrame.onload = () => {
    state.cameraBusy = false;
    refs.cameraStatus.textContent = `스냅샷 수신 · ${new Date().toLocaleTimeString()}`;
    if (state.cameraPolling) scheduleNextCameraFrame(1000);
  };
  refs.cameraFrame.onerror = () => {
    state.cameraBusy = false;
    refs.cameraStatus.textContent = "영상 수신 실패, 다시 시도합니다...";
    if (state.cameraPolling) scheduleNextCameraFrame(1800);
  };
  refs.cameraFrame.src = `/api/camera/frame?ts=${Date.now()}`;
}

function setCameraPolling(enabled) {
  state.cameraPolling = enabled;
  refs.cameraToggleButton.textContent = enabled ? "실시간 중지" : "실시간 시작";
  clearTimeout(state.cameraTimer);
  if (enabled) {
    refs.cameraStatus.textContent = "실시간 영상 연결 중...";
    refreshCameraFrame();
  } else {
    state.cameraTimer = null;
    refs.cameraStatus.textContent = "실시간 영상이 중지되었습니다";
  }
}

function getCameraStreamUrl() {
  return state.status?.connection?.bridge?.camera_stream_url || refs.cameraStreamUrl?.value?.trim() || "";
}

function updateStreamLink(streamUrl) {
  refs.streamUrlOpen.href = streamUrl || "#";
  refs.streamUrlText.textContent = streamUrl || "권장 시작값: 640×360 / 10fps / quality 70";
}

function connectCameraStream(streamUrl) {
  updateStreamLink(streamUrl);
  state.cameraBusy = true;
  clearTimeout(state.cameraTimer);
  refs.cameraFrame.onload = () => {
    state.cameraBusy = false;
    refs.cameraStatus.textContent = `직접 스트리밍 중 · ${new Date().toLocaleTimeString()}`;
  };
  refs.cameraFrame.onerror = () => {
    state.cameraBusy = false;
    refs.cameraStatus.textContent = "직접 스트림 연결 실패";
  };
  const joiner = streamUrl.includes("?") ? "&" : "?";
  refs.cameraFrame.src = `${streamUrl}${joiner}ts=${Date.now()}`;
}

async function runCode() {
  const code = refs.codeEditor.value;
  const timeout = Number(refs.codeTimeout.value || 20);
  refs.runCodeButton.disabled = true;
  refs.runBlocksButton.disabled = true;
  refs.codeStatus.textContent = "실행 중";
  refs.codeOutput.textContent = "실행 중...";
  try {
    const response = await fetch("/api/code/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ code, timeout }),
    });
    const result = await response.json();
    const parts = [
      `returncode=${result.returncode} · ${result.elapsed_ms}ms`,
      result.stdout ? `\n[stdout]\n${result.stdout}` : "",
      result.stderr ? `\n[stderr]\n${result.stderr}` : "",
      `\nscript=${result.script}`,
    ];
    refs.codeOutput.textContent = parts.join("");
    refs.codeStatus.textContent = result.ok ? "성공" : "실패";
    flash(result.ok ? "코드 실행 완료" : "코드 실행 실패", result.ok ? "ok" : "error");
    await refreshStatus();
  } catch (error) {
    refs.codeOutput.textContent = `실행 요청 실패: ${error.message}`;
    refs.codeStatus.textContent = "오류";
    flash(`코드 실행 실패: ${error.message}`, "error");
  } finally {
    refs.runCodeButton.disabled = false;
    refs.runBlocksButton.disabled = false;
  }
}

async function copyCode() {
  await navigator.clipboard.writeText(refs.codeEditor.value);
  refs.codeStatus.textContent = "복사됨";
}

function flash(message, kind) {
  refs.controlFlash.textContent = message;
  refs.controlFlash.style.color = kind === "error" ? "var(--red)" : "var(--green)";
}

async function apiFetch(url, options = {}) {
  const response = await fetch(url, {
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
    ...options,
  });
  const text = await response.text();
  let payload = {};
  if (text) {
    try {
      payload = JSON.parse(text);
    } catch (error) {
      throw new Error("JSON parse failed");
    }
  }
  if (!response.ok) {
    throw new Error(payload.bridge_error || payload.error || payload.stderr || `HTTP ${response.status}`);
  }
  return payload;
}

function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function setIfIdle(element, value) {
  if (!element || document.activeElement === element) return;
  element.value = value;
}

function clamp(value, lower, upper) {
  return Math.max(lower, Math.min(upper, Number(value)));
}
