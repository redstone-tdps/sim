const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const wheelGrid = document.getElementById("wheelGrid");
const metrics = document.getElementById("metrics");
const codeInput = document.getElementById("codeInput");
const codeStatus = document.getElementById("codeStatus");
const debugLog = document.getElementById("debugLog");
const clearDebug = document.getElementById("clearDebug");
const camOffsetX = document.getElementById("camOffsetX");
const camOffsetY = document.getElementById("camOffsetY");
const camZoom = document.getElementById("camZoom");
const camOffsetXVal = document.getElementById("camOffsetXVal");
const camOffsetYVal = document.getElementById("camOffsetYVal");
const camZoomVal = document.getElementById("camZoomVal");
const camFollowToggle = document.getElementById("camFollowToggle");
const camReset = document.getElementById("camReset");
const cfgTrackWidth = document.getElementById("cfgTrackWidth");
const cfgWheelBase = document.getElementById("cfgWheelBase");
const cfgMaxWheelSpeed = document.getElementById("cfgMaxWheelSpeed");
const cfgWheelVisualScale = document.getElementById("cfgWheelVisualScale");
const cfgTrailPoints = document.getElementById("cfgTrailPoints");
const cfgMaxDt = document.getElementById("cfgMaxDt");

const wheelNames = ["FL", "FR", "RL", "RR"];
const wheelValueEls = {};
let codeEditor = null;
const dragState = {
  active: false,
  lastX: 0,
  lastY: 0,
};

const sim = {
  running: true,
  codeMode: false,
  time: 0,
  pose: { x: 0, y: 0, theta: Math.PI / 2 },
  wheelSpeeds: { FL: 0, FR: 0, RL: 0, RR: 0 },
  trackWidth: 0.2,
  wheelBase: 0.4,
  trail: [],
  lastKinematics: null,
  lastDiagnostics: null,
  controller: null,
  maxTrailPoints: 900,
  maxAbsWheelSpeed: 4.0,
  wheelVisualScale: 1.0,
  maxDt: 0.04,
  camera: {
    offsetX: 0,
    offsetY: 0,
    scale: 220,
    followCar: false,
  },
  debugLines: [],
  maxDebugLines: 180,
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function readNumberInput(inputEl, min, max, fallback) {
  const value = Number(inputEl.value);
  if (!Number.isFinite(value)) return fallback;
  return clamp(value, min, max);
}

function setWheelSpeed(name, value) {
  if (!wheelNames.includes(name)) return;
  sim.wheelSpeeds[name] = clamp(value, -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
  updateWheelValue(name);
}

function setWheelSpeeds(vFL, vFR, vRL, vRR) {
  setWheelSpeed("FL", vFL);
  setWheelSpeed("FR", vFR);
  setWheelSpeed("RL", vRL);
  setWheelSpeed("RR", vRR);
}

function n(value, digits = 2) {
  return value.toFixed(digits);
}

function updateWheelValue(name) {
  const valueEl = wheelValueEls[name];
  if (valueEl) {
    valueEl.textContent = `${n(sim.wheelSpeeds[name], 2)} m/s`;
  }
}

function buildWheelGrid() {
  wheelGrid.innerHTML = "";

  for (const name of wheelNames) {
    const label = document.createElement("div");
    label.className = "wheel-label";
    label.textContent = name;

    const value = document.createElement("div");
    value.className = "wheel-value";
    wheelValueEls[name] = value;
    updateWheelValue(name);

    const minus = document.createElement("button");
    minus.className = "small-btn secondary";
    minus.textContent = "-0.2";
    minus.addEventListener("click", () => {
      setWheelSpeed(name, sim.wheelSpeeds[name] - 0.2);
    });

    const plus = document.createElement("button");
    plus.className = "small-btn secondary";
    plus.textContent = "+0.2";
    plus.addEventListener("click", () => {
      setWheelSpeed(name, sim.wheelSpeeds[name] + 0.2);
    });

    wheelGrid.append(label, value, minus, plus);
  }
}

function updateMetrics() {
  const p = sim.pose;
  const k = sim.lastKinematics || { vx: 0, omega: 0, radius: Infinity };
  const d = sim.lastDiagnostics || { motionState: "STATIONARY", lateralSlipIndicator: 0 };

  const values = [
    ["x", `${n(p.x, 2)} m`],
    ["y", `${n(p.y, 2)} m`],
    ["theta", `${n(p.theta, 2)} rad`],
    ["vx", `${n(k.vx, 2)} m/s`],
    ["omega", `${n(k.omega, 2)} rad/s`],
    ["radius", Number.isFinite(k.radius) ? `${n(k.radius, 2)} m` : "inf"],
    ["motion", d.motionState],
    ["slip", `${n(d.lateralSlipIndicator, 3)}`],
  ];

  metrics.innerHTML = "";
  for (const [label, val] of values) {
    const row = document.createElement("div");
    const left = document.createElement("span");
    left.textContent = label;
    const right = document.createElement("strong");
    right.textContent = val;
    row.append(left, right);
    metrics.appendChild(row);
  }
}

function compileController() {
  const source = codeEditor ? codeEditor.getValue() : codeInput.value;
  sim.controller = new Function("api", source);
}

function executeController(dt) {
  if (!sim.controller) return;

  const api = {
    dt,
    time: sim.time,
    pose: { ...sim.pose },
    wheelSpeeds: { ...sim.wheelSpeeds },
    setWheelSpeed,
    setWheelSpeeds,
    log: (...parts) => addDebugLine(...parts),
    clearLog: clearDebugLog,
    clamp,
    Math,
  };

  sim.controller(api);
}

function setStatus(message, isError = false) {
  codeStatus.textContent = message;
  codeStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function renderDebugLog() {
  debugLog.textContent = sim.debugLines.join("\n");
  debugLog.scrollTop = debugLog.scrollHeight;
}

function addDebugLine(...parts) {
  const text = parts.map((part) => {
    if (typeof part === "string") return part;
    try {
      return JSON.stringify(part);
    } catch {
      return String(part);
    }
  }).join(" ");

  const stamp = `[${n(sim.time, 2)}s]`;
  sim.debugLines.push(`${stamp} ${text}`);
  if (sim.debugLines.length > sim.maxDebugLines) {
    sim.debugLines.shift();
  }
  renderDebugLog();
}

function clearDebugLog() {
  sim.debugLines = [];
  renderDebugLog();
}

function stepSimulation(dt) {
  if (sim.codeMode) {
    try {
      executeController(dt);
    } catch (err) {
      sim.codeMode = false;
      setStatus(`Code error: ${err.message}`, true);
    }
  }

  const result = calculatePositionChange(
    sim.pose,
    sim.wheelSpeeds.FL,
    sim.wheelSpeeds.FR,
    sim.wheelSpeeds.RL,
    sim.wheelSpeeds.RR,
    sim.trackWidth,
    sim.wheelBase,
    dt
  );

  sim.pose = result.pose;
  sim.lastKinematics = result.kinematics;
  sim.lastDiagnostics = result.diagnostics;

  sim.trail.push({ x: sim.pose.x, y: sim.pose.y });
  if (sim.trail.length > sim.maxTrailPoints) {
    sim.trail.shift();
  }
}

function drawGrid(scale, cx, cy) {
  const step = scale;
  const majorEvery = 5;
  const xStart = ((cx % step) + step) % step;
  const yStart = ((cy % step) + step) % step;

  ctx.save();
  ctx.lineWidth = 1;

  let index = 0;
  for (let x = xStart; x <= canvas.width; x += step, index += 1) {
    const isMajor = index % majorEvery === 0;
    ctx.strokeStyle = isMajor ? "rgba(22, 34, 47, 0.12)" : "rgba(22, 34, 47, 0.05)";
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }

  index = 0;
  for (let y = yStart; y <= canvas.height; y += step, index += 1) {
    const isMajor = index % majorEvery === 0;
    ctx.strokeStyle = isMajor ? "rgba(22, 34, 47, 0.12)" : "rgba(22, 34, 47, 0.05)";
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  // Draw world axes slightly stronger for orientation.
  ctx.strokeStyle = "rgba(230, 103, 46, 0.28)";
  ctx.lineWidth = 1.4;
  if (cx >= 0 && cx <= canvas.width) {
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, canvas.height);
    ctx.stroke();
  }
  if (cy >= 0 && cy <= canvas.height) {
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(canvas.width, cy);
    ctx.stroke();
  }
  ctx.restore();
}

function worldToScreen(x, y, camCenterX, camCenterY, scale) {
  return {
    x: camCenterX + x * scale,
    y: camCenterY - y * scale,
  };
}

function updateCameraLabels() {
  camOffsetXVal.textContent = `${sim.camera.offsetX}`;
  camOffsetYVal.textContent = `${sim.camera.offsetY}`;
  camZoomVal.textContent = `${sim.camera.scale}`;
  camFollowToggle.textContent = `Follow Car: ${sim.camera.followCar ? "On" : "Off"}`;
}

function syncSimConfigInputs() {
  cfgTrackWidth.value = String(sim.trackWidth);
  cfgWheelBase.value = String(sim.wheelBase);
  cfgMaxWheelSpeed.value = String(sim.maxAbsWheelSpeed);
  cfgWheelVisualScale.value = String(sim.wheelVisualScale);
  cfgTrailPoints.value = String(sim.maxTrailPoints);
  cfgMaxDt.value = String(sim.maxDt);
}

function applySimConfigFromInputs() {
  sim.trackWidth = readNumberInput(cfgTrackWidth, 0.2, 5, sim.trackWidth);
  sim.wheelBase = readNumberInput(cfgWheelBase, 0.2, 8, sim.wheelBase);
  sim.maxAbsWheelSpeed = readNumberInput(cfgMaxWheelSpeed, 0.2, 20, sim.maxAbsWheelSpeed);
  sim.wheelVisualScale = readNumberInput(cfgWheelVisualScale, 0.2, 3, sim.wheelVisualScale);
  sim.maxTrailPoints = Math.round(readNumberInput(cfgTrailPoints, 50, 20000, sim.maxTrailPoints));
  sim.maxDt = readNumberInput(cfgMaxDt, 0.005, 0.2, sim.maxDt);

  for (const name of wheelNames) {
    sim.wheelSpeeds[name] = clamp(sim.wheelSpeeds[name], -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
    updateWheelValue(name);
  }

  if (sim.trail.length > sim.maxTrailPoints) {
    sim.trail = sim.trail.slice(sim.trail.length - sim.maxTrailPoints);
  }

  syncSimConfigInputs();
}

function syncCameraInputs() {
  camOffsetX.value = String(sim.camera.offsetX);
  camOffsetY.value = String(sim.camera.offsetY);
  camZoom.value = String(sim.camera.scale);
}

function drawCar() {
  const scale = sim.camera.scale;
  const camCenterX = canvas.width / 2 + sim.camera.offsetX;
  const camCenterY = canvas.height / 2 + sim.camera.offsetY;
  const cameraWorldX = sim.camera.followCar ? sim.pose.x : 0;
  const cameraWorldY = sim.camera.followCar ? sim.pose.y : 0;
  const viewCenter = worldToScreen(-cameraWorldX, -cameraWorldY, camCenterX, camCenterY, scale);

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid(scale, viewCenter.x, viewCenter.y);

  if (sim.trail.length > 1) {
    ctx.save();
    ctx.strokeStyle = "rgba(230, 103, 46, 0.5)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < sim.trail.length; i += 1) {
      const p = sim.trail[i];
      const screen = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      if (i === 0) {
        ctx.moveTo(screen.x, screen.y);
      } else {
        ctx.lineTo(screen.x, screen.y);
      }
    }
    ctx.stroke();
    ctx.restore();
  }

  ctx.save();
  const carScreen = worldToScreen(
    sim.pose.x - cameraWorldX,
    sim.pose.y - cameraWorldY,
    camCenterX,
    camCenterY,
    scale
  );
  ctx.translate(carScreen.x, carScreen.y);
  ctx.rotate(-sim.pose.theta);

  const bodyLength = sim.wheelBase * scale;
  const bodyWidth = sim.trackWidth * scale;

  ctx.fillStyle = "#3b6d90";
  ctx.strokeStyle = "#1f3344";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-bodyLength / 2, -bodyWidth / 2, bodyLength, bodyWidth, 12);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#e6672e";
  ctx.beginPath();
  ctx.moveTo(bodyLength / 2 - 8, 0);
  ctx.lineTo(bodyLength / 2 - 20, -9);
  ctx.lineTo(bodyLength / 2 - 20, 9);
  ctx.closePath();
  ctx.fill();

  // Keep wheel geometry proportional to zoom so layout is stable at any scale.
  const wheelL = 0.14 * scale * sim.wheelVisualScale;
  const wheelW = 0.07 * scale * sim.wheelVisualScale;
  const xOffset = bodyLength / 2 - wheelL * 0.65;
  const yOffset = bodyWidth / 2 + wheelW * 0.45;
  ctx.fillStyle = "#1f3344";

  ctx.fillRect(-xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(-xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);

  ctx.restore();
}

camOffsetX.addEventListener("input", (event) => {
  sim.camera.offsetX = Number(event.target.value);
  updateCameraLabels();
});

camOffsetY.addEventListener("input", (event) => {
  sim.camera.offsetY = Number(event.target.value);
  updateCameraLabels();
});

camZoom.addEventListener("input", (event) => {
  sim.camera.scale = Number(event.target.value);
  updateCameraLabels();
});

camFollowToggle.addEventListener("click", () => {
  sim.camera.followCar = !sim.camera.followCar;
  updateCameraLabels();
});

camReset.addEventListener("click", () => {
  sim.camera.offsetX = 0;
  sim.camera.offsetY = 0;
  sim.camera.scale = 220;
  sim.camera.followCar = false;
  syncCameraInputs();
  updateCameraLabels();
});

clearDebug.addEventListener("click", clearDebugLog);

for (const configInput of [
  cfgTrackWidth,
  cfgWheelBase,
  cfgMaxWheelSpeed,
  cfgWheelVisualScale,
  cfgTrailPoints,
  cfgMaxDt,
]) {
  configInput.addEventListener("change", applySimConfigFromInputs);
}

canvas.addEventListener("mousedown", (event) => {
  dragState.active = true;
  dragState.lastX = event.clientX;
  dragState.lastY = event.clientY;
  canvas.classList.add("dragging");
});

window.addEventListener("mouseup", () => {
  dragState.active = false;
  canvas.classList.remove("dragging");
});

window.addEventListener("mousemove", (event) => {
  if (!dragState.active) return;
  const dx = event.clientX - dragState.lastX;
  const dy = event.clientY - dragState.lastY;
  dragState.lastX = event.clientX;
  dragState.lastY = event.clientY;

  sim.camera.offsetX = clamp(sim.camera.offsetX + dx, -5000, 5000);
  sim.camera.offsetY = clamp(sim.camera.offsetY + dy, -5000, 5000);
  syncCameraInputs();
  updateCameraLabels();
});

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const delta = event.deltaY < 0 ? 4 : -4;
  sim.camera.scale = clamp(sim.camera.scale + delta, 10, 300);
  syncCameraInputs();
  updateCameraLabels();
}, { passive: false });

document.getElementById("toggleRun").addEventListener("click", (event) => {
  sim.running = !sim.running;
  event.target.textContent = sim.running ? "Pause" : "Resume";
});

document.getElementById("zeroSpeeds").addEventListener("click", () => {
  setWheelSpeeds(0, 0, 0, 0);
});

document.getElementById("resetPose").addEventListener("click", () => {
  sim.pose = { x: 0, y: 0, theta: Math.PI / 2 };
  sim.trail = [];
  sim.lastKinematics = null;
  sim.lastDiagnostics = null;
  updateMetrics();
});

document.getElementById("enableCode").addEventListener("click", () => {
  try {
    compileController();
    sim.codeMode = true;
    setStatus("Code mode enabled. Your script is running every frame.");
  } catch (err) {
    sim.codeMode = false;
    setStatus(`Compile error: ${err.message}`, true);
  }
});

document.getElementById("runOnce").addEventListener("click", () => {
  try {
    compileController();
    executeController(0.016);
    setStatus("Script executed once.");
  } catch (err) {
    setStatus(`Run error: ${err.message}`, true);
  }
});

document.getElementById("disableCode").addEventListener("click", () => {
  sim.codeMode = false;
  setStatus("Code mode disabled.");
});

let lastTime = performance.now();

function loop(now) {
  const dt = Math.min(sim.maxDt, (now - lastTime) / 1000);
  lastTime = now;

  if (sim.running) {
    stepSimulation(dt);
    sim.time += dt;
  }

  drawCar();
  updateMetrics();

  requestAnimationFrame(loop);
}

function initializeCodeEditor() {
  if (typeof window.CodeMirror !== "function") return;

  codeEditor = window.CodeMirror.fromTextArea(codeInput, {
    mode: "javascript",
    theme: "neo",
    lineNumbers: true,
    lineWrapping: true,
    tabSize: 2,
    indentUnit: 2,
  });
}

initializeCodeEditor();
buildWheelGrid();
updateMetrics();
syncCameraInputs();
updateCameraLabels();
syncSimConfigInputs();
addDebugLine("Debug ready. Use api.log(...) and api.clearLog().");
requestAnimationFrame(loop);
