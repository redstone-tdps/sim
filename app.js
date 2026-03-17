const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const wheelGrid = document.getElementById("wheelGrid");
const metrics = document.getElementById("metrics");
const codeInput = document.getElementById("codeInput");
const codeStatus = document.getElementById("codeStatus");
const memoryView = document.getElementById("memoryView");
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
const visionCanvas = document.getElementById("visionCanvas");
const visionCtx = visionCanvas.getContext("2d", { willReadFrequently: true });
const visionCamX = document.getElementById("visionCamX");
const visionCamY = document.getElementById("visionCamY");
const visionCamZ = document.getElementById("visionCamZ");
const visionPitch = document.getElementById("visionPitch");
const visionFovH = document.getElementById("visionFovH");
const visionFovV = document.getElementById("visionFovV");
const visionStatus = document.getElementById("visionStatus");
const cfgTrackWidth = document.getElementById("cfgTrackWidth");
const cfgWheelBase = document.getElementById("cfgWheelBase");
const cfgMaxWheelSpeed = document.getElementById("cfgMaxWheelSpeed");
const cfgWheelVisualScale = document.getElementById("cfgWheelVisualScale");
const cfgTrailPoints = document.getElementById("cfgTrailPoints");
const cfgMaxDt = document.getElementById("cfgMaxDt");
const cfgTrackPreset = document.getElementById("cfgTrackPreset");
const cfgLineWidth = document.getElementById("cfgLineWidth");
const cfgSvgPath = document.getElementById("cfgSvgPath");
const cfgSensors = document.getElementById("cfgSensors");
const applyTrack = document.getElementById("applyTrack");
const applyTrackSensors = document.getElementById("applyTrackSensors");
const startTrackDraw = document.getElementById("startTrackDraw");
const finishTrackDraw = document.getElementById("finishTrackDraw");
const clearTrackDraw = document.getElementById("clearTrackDraw");
const trackStatus = document.getElementById("trackStatus");

const wheelNames = ["FL", "FR", "RL", "RR"];
const wheelValueEls = {};
let codeEditor = null;

const TRACK_MASK_SIZE = 2400;
const TRACK_PX_PER_M = 280;
const trackMask = document.createElement("canvas");
trackMask.width = TRACK_MASK_SIZE;
trackMask.height = TRACK_MASK_SIZE;
const trackMaskCtx = trackMask.getContext("2d", { willReadFrequently: true });

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
  track: {
    preset: "figure8",
    lineWidth: 0.03,
    svgPath: cfgSvgPath.value,
    sensors: [],
    sensorReadings: [],
    svgPath2D: null,
    visionPolyline: [],
    drawMode: false,
    draftPoints: [],
  },
  vision: {
    camX: 0.18,
    camY: 0,
    camZ: 0.16,
    pitchDeg: 28,
    fovHDeg: 70,
    fovVDeg: 56,
    opencvReady: false,
  },
  debugLines: [],
  maxDebugLines: 180,
  scriptMemory: {},
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

function setTrackStatus(message, isError = false) {
  trackStatus.textContent = message;
  trackStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function setVisionStatus(message, isError = false) {
  visionStatus.textContent = message;
  visionStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function bodyToWorld(localX, localY, pose) {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return {
    x: pose.x + localX * c - localY * s,
    y: pose.y + localX * s + localY * c,
  };
}

function parseSensorConfig(text) {
  let parsed;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error("Sensor JSON is invalid.");
  }
  if (!Array.isArray(parsed) || parsed.length === 0) {
    throw new Error("Sensor config must be a non-empty JSON array.");
  }

  const sensors = parsed.map((item, index) => {
    if (!item || typeof item !== "object") {
      throw new Error(`Sensor ${index} must be an object.`);
    }
    const name = typeof item.name === "string" && item.name.trim() ? item.name.trim() : `S${index}`;
    const x = Number(item.x);
    const y = Number(item.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw new Error(`Sensor ${name} must contain numeric x and y.`);
    }
    return { name, x, y };
  });

  return sensors;
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
  const sensorSummary = sim.track.sensorReadings
    .map((s) => `${s.name}:${n(s.value, 2)}`)
    .join(" ");

  const values = [
    ["x", `${n(p.x, 2)} m`],
    ["y", `${n(p.y, 2)} m`],
    ["theta", `${n(p.theta, 2)} rad`],
    ["vx", `${n(k.vx, 2)} m/s`],
    ["omega", `${n(k.omega, 2)} rad/s`],
    ["radius", Number.isFinite(k.radius) ? `${n(k.radius, 2)} m` : "inf"],
    ["motion", d.motionState],
    ["sensors", sensorSummary || "n/a"],
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

  const sensors = sim.track.sensorReadings.map((sensor) => ({ ...sensor }));
  const sensorMap = new Map(sensors.map((sensor) => [sensor.name, sensor]));

  const api = {
    dt,
    time: sim.time,
    pose: { ...sim.pose },
    wheelSpeeds: { ...sim.wheelSpeeds },
    sensors,
    getSensor: (name) => sensorMap.get(name),
    getVisionFrame: () => ({
      width: visionCanvas.width,
      height: visionCanvas.height,
      imageData: visionCtx.getImageData(0, 0, visionCanvas.width, visionCanvas.height),
    }),
    mem: createScriptMemoryApi(),
    setWheelSpeed,
    setWheelSpeeds,
    log: (...parts) => addDebugLine(...parts),
    clearLog: clearDebugLog,
    clamp,
    cv: sim.vision.opencvReady ? window.cv : null,
    Math,
  };

  sim.controller(api);
}

function syncVisionInputs() {
  visionCamX.value = String(sim.vision.camX);
  visionCamY.value = String(sim.vision.camY);
  visionCamZ.value = String(sim.vision.camZ);
  visionPitch.value = String(sim.vision.pitchDeg);
  visionFovH.value = String(sim.vision.fovHDeg);
  visionFovV.value = String(sim.vision.fovVDeg);
}

function applyVisionConfigFromInputs() {
  sim.vision.camX = readNumberInput(visionCamX, -1, 2, sim.vision.camX);
  sim.vision.camY = readNumberInput(visionCamY, -1, 1, sim.vision.camY);
  sim.vision.camZ = readNumberInput(visionCamZ, 0.05, 2, sim.vision.camZ);
  sim.vision.pitchDeg = readNumberInput(visionPitch, 0, 89, sim.vision.pitchDeg);
  sim.vision.fovHDeg = readNumberInput(visionFovH, 20, 170, sim.vision.fovHDeg);
  sim.vision.fovVDeg = readNumberInput(visionFovV, 20, 170, sim.vision.fovVDeg);
  syncVisionInputs();

  const cvStatus = sim.vision.opencvReady ? "ready" : "loading...";
  setVisionStatus(
    `Vision camera updated: x=${n(sim.vision.camX, 2)} y=${n(sim.vision.camY, 2)} z=${n(sim.vision.camZ, 2)} pitch=${n(sim.vision.pitchDeg, 1)}° fovH=${n(sim.vision.fovHDeg, 1)}° fovV=${n(sim.vision.fovVDeg, 1)}°. OpenCV.js: ${cvStatus}`
  );
}

function updateOpenCvStatus() {
  const cv = window.cv;
  const ready = !!(cv && typeof cv.getBuildInformation === "function");
  if (ready !== sim.vision.opencvReady) {
    sim.vision.opencvReady = ready;
    const status = ready ? "ready" : "loading...";
    setVisionStatus(
      `Vision ready. Camera x=${n(sim.vision.camX, 2)} y=${n(sim.vision.camY, 2)} z=${n(sim.vision.camZ, 2)} pitch=${n(sim.vision.pitchDeg, 1)}° fovH=${n(sim.vision.fovHDeg, 1)}° fovV=${n(sim.vision.fovVDeg, 1)}°. OpenCV.js: ${status}`
    );
  }
}

function renderVision() {
  const width = visionCanvas.width;
  const height = visionCanvas.height;
  const tanHalfH = Math.tan((sim.vision.fovHDeg * Math.PI / 180) * 0.5);
  const tanHalfV = Math.tan((sim.vision.fovVDeg * Math.PI / 180) * 0.5);
  const fx = width / (2 * tanHalfH);
  const fy = height / (2 * tanHalfV);
  const cx = width * 0.5;
  const cy = height * 0.5;

  const pitchRad = sim.vision.pitchDeg * Math.PI / 180;
  const cp = Math.cos(pitchRad);
  const sp = Math.sin(pitchRad);

  const horizonY = cy - fy * Math.tan(pitchRad);
  visionCtx.fillStyle = "#c9e0f3";
  visionCtx.fillRect(0, 0, width, Math.max(0, Math.min(height, horizonY)));
  visionCtx.fillStyle = "#d6cdbb";
  visionCtx.fillRect(0, Math.max(0, Math.min(height, horizonY)), width, height);

  visionCtx.strokeStyle = "rgba(20, 45, 66, 0.35)";
  visionCtx.lineWidth = 1;
  visionCtx.beginPath();
  visionCtx.moveTo(0, horizonY);
  visionCtx.lineTo(width, horizonY);
  visionCtx.stroke();

  const forward = { x: cp, y: 0, z: -sp };
  const right = { x: 0, y: -1, z: 0 };
  const down = { x: -sp, y: 0, z: -cp };

  const c = Math.cos(sim.pose.theta);
  const s = Math.sin(sim.pose.theta);

  const toBodyFromWorld = (wx, wy) => {
    const dx = wx - sim.pose.x;
    const dy = wy - sim.pose.y;
    return {
      x: dx * c + dy * s,
      y: -dx * s + dy * c,
    };
  };

  const projectWorldPoint = (wx, wy) => {
    const body = toBodyFromWorld(wx, wy);
    const vx = body.x - sim.vision.camX;
    const vy = body.y - sim.vision.camY;
    const vz = -sim.vision.camZ;

    const camX = vx * right.x + vy * right.y + vz * right.z;
    const camY = vx * down.x + vy * down.y + vz * down.z;
    const camZ = vx * forward.x + vy * forward.y + vz * forward.z;
    if (camZ <= 0.06) return null;

    const px = cx + fx * (camX / camZ);
    const py = cy + fy * (camY / camZ);
    const widthPx = clamp((fx * sim.track.lineWidth) / camZ, 1, 18);
    return { x: px, y: py, width: widthPx };
  };

  visionCtx.strokeStyle = "#232323";
  visionCtx.fillStyle = "#202020";
  visionCtx.lineCap = "round";
  visionCtx.lineJoin = "round";

  for (const line of sim.track.visionPolyline) {
    let prev = null;
    for (const point of line) {
      const projected = projectWorldPoint(point.x, point.y);
      if (!projected) {
        prev = null;
        continue;
      }

      if (projected.x >= -30 && projected.x <= width + 30 && projected.y >= -30 && projected.y <= height + 30) {
        visionCtx.beginPath();
        visionCtx.arc(projected.x, projected.y, projected.width * 0.45, 0, 2 * Math.PI);
        visionCtx.fill();
      }

      if (prev) {
        visionCtx.lineWidth = (prev.width + projected.width) * 0.5;
        visionCtx.beginPath();
        visionCtx.moveTo(prev.x, prev.y);
        visionCtx.lineTo(projected.x, projected.y);
        visionCtx.stroke();
      }
      prev = projected;
    }
  }
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

function renderMemoryView() {
  const keys = Object.keys(sim.scriptMemory);
  if (keys.length === 0) {
    memoryView.textContent = "(empty)";
    return;
  }

  const lines = keys.map((key) => {
    let valueText;
    try {
      valueText = JSON.stringify(sim.scriptMemory[key]);
    } catch {
      valueText = String(sim.scriptMemory[key]);
    }
    return `${key}: ${valueText}`;
  });
  memoryView.textContent = lines.join("\n");
}

function cloneForMemory(value) {
  if (typeof structuredClone === "function") {
    return structuredClone(value);
  }
  if (value === undefined) return undefined;
  try {
    return JSON.parse(JSON.stringify(value));
  } catch {
    return value;
  }
}

function createScriptMemoryApi() {
  return {
    get(key, defaultValue = undefined) {
      if (Object.prototype.hasOwnProperty.call(sim.scriptMemory, key)) {
        return cloneForMemory(sim.scriptMemory[key]);
      }
      return cloneForMemory(defaultValue);
    },
    set(key, value) {
      sim.scriptMemory[key] = cloneForMemory(value);
      renderMemoryView();
      return cloneForMemory(sim.scriptMemory[key]);
    },
    remove(key) {
      delete sim.scriptMemory[key];
      renderMemoryView();
    },
    clear() {
      sim.scriptMemory = {};
      renderMemoryView();
    },
    keys() {
      return Object.keys(sim.scriptMemory);
    },
  };
}

function drawPresetTrackPath(targetCtx, preset) {
  if (preset === "circle") {
    const r = 1.15;
    // Circle centered at (r, 0) so the loop goes through (0, 0).
    targetCtx.moveTo(0, 0);
    targetCtx.arc(r, 0, r, Math.PI, Math.PI + 2 * Math.PI);
    return;
  }
  if (preset === "figure8") {
    const r = 0.7;
    targetCtx.arc(-r, 0, r, 0, 2 * Math.PI);
    targetCtx.moveTo(2 * r, 0);
    targetCtx.arc(r, 0, r, 0, 2 * Math.PI);
    return;
  }

  // oval
  targetCtx.ellipse(0, 0, 1.45, 0.9, 0, 0, 2 * Math.PI);
}

function drawTrackGeometry(targetCtx) {
  targetCtx.beginPath();
  if (sim.track.preset === "svg" && sim.track.svgPath2D) {
    targetCtx.stroke(sim.track.svgPath2D);
    return;
  }
  drawPresetTrackPath(targetCtx, sim.track.preset);
  targetCtx.stroke();
}

function rebuildTrackMask() {
  trackMaskCtx.save();
  trackMaskCtx.fillStyle = "black";
  trackMaskCtx.fillRect(0, 0, trackMask.width, trackMask.height);
  trackMaskCtx.translate(trackMask.width / 2, trackMask.height / 2);
  trackMaskCtx.scale(TRACK_PX_PER_M, -TRACK_PX_PER_M);
  trackMaskCtx.lineCap = "round";
  trackMaskCtx.lineJoin = "round";
  trackMaskCtx.strokeStyle = "white";
  trackMaskCtx.lineWidth = sim.track.lineWidth;
  drawTrackGeometry(trackMaskCtx);
  trackMaskCtx.restore();
}

function updateSensorReadings() {
  sim.track.sensorReadings = sim.track.sensors.map((sensor) => {
    const world = bodyToWorld(sensor.x, sensor.y, sim.pose);
    const px = Math.round(trackMask.width / 2 + world.x * TRACK_PX_PER_M);
    const py = Math.round(trackMask.height / 2 - world.y * TRACK_PX_PER_M);

    let value = 0;
    if (px >= 0 && px < trackMask.width && py >= 0 && py < trackMask.height) {
      const rgba = trackMaskCtx.getImageData(px, py, 1, 1).data;
      value = rgba[0] / 255;
    }

    return {
      name: sensor.name,
      x: sensor.x,
      y: sensor.y,
      worldX: world.x,
      worldY: world.y,
      value,
    };
  });
}

function sampleEllipsePoints(cx, cy, rx, ry, count) {
  const points = [];
  for (let i = 0; i <= count; i += 1) {
    const t = (i / count) * 2 * Math.PI;
    points.push({ x: cx + rx * Math.cos(t), y: cy + ry * Math.sin(t) });
  }
  return points;
}

function sampleCircleArcPoints(cx, cy, radius, startAngle, endAngle, count) {
  const points = [];
  for (let i = 0; i <= count; i += 1) {
    const t = startAngle + (endAngle - startAngle) * (i / count);
    points.push({ x: cx + radius * Math.cos(t), y: cy + radius * Math.sin(t) });
  }
  return points;
}

function densifyPolyline(line, maxStep = 0.03) {
  if (!Array.isArray(line) || line.length < 2) return Array.isArray(line) ? line : [];

  const points = [line[0]];
  for (let i = 1; i < line.length; i += 1) {
    const a = line[i - 1];
    const b = line[i];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dist = Math.hypot(dx, dy);
    const segments = Math.max(1, Math.ceil(dist / maxStep));

    for (let k = 1; k <= segments; k += 1) {
      const t = k / segments;
      points.push({
        x: a.x + dx * t,
        y: a.y + dy * t,
      });
    }
  }

  return points;
}

function parseSvgPathToPolyline(pathText) {
  const tokens = pathText
    .replace(/,/g, " ")
    .match(/[a-zA-Z]|[-+]?\d*\.?\d+(?:e[-+]?\d+)?/gi);
  if (!tokens) return [];

  const lines = [];
  let index = 0;
  let command = null;
  let x = 0;
  let y = 0;
  let subStartX = 0;
  let subStartY = 0;
  let currentLine = [];

  const isCommand = (token) => /^[a-zA-Z]$/.test(token);
  const readNumber = () => {
    if (index >= tokens.length) return null;
    const token = tokens[index];
    if (isCommand(token)) return null;
    const value = Number(token);
    if (!Number.isFinite(value)) return null;
    index += 1;
    return value;
  };

  while (index < tokens.length) {
    if (isCommand(tokens[index])) {
      command = tokens[index];
      index += 1;
    }
    if (!command) break;

    if (command === "M" || command === "m") {
      const px = readNumber();
      const py = readNumber();
      if (px === null || py === null) break;
      x = command === "m" ? x + px : px;
      y = command === "m" ? y + py : py;
      subStartX = x;
      subStartY = y;
      if (currentLine.length > 1) lines.push(currentLine);
      currentLine = [{ x, y }];
      command = command === "m" ? "l" : "L";
      continue;
    }

    if (command === "L" || command === "l") {
      const px = readNumber();
      const py = readNumber();
      if (px === null || py === null) continue;
      x = command === "l" ? x + px : px;
      y = command === "l" ? y + py : py;
      currentLine.push({ x, y });
      continue;
    }

    if (command === "H" || command === "h") {
      const px = readNumber();
      if (px === null) continue;
      x = command === "h" ? x + px : px;
      currentLine.push({ x, y });
      continue;
    }

    if (command === "V" || command === "v") {
      const py = readNumber();
      if (py === null) continue;
      y = command === "v" ? y + py : py;
      currentLine.push({ x, y });
      continue;
    }

    if (command === "Z" || command === "z") {
      if (currentLine.length > 0) {
        currentLine.push({ x: subStartX, y: subStartY });
      }
      if (currentLine.length > 1) lines.push(currentLine);
      currentLine = [];
      continue;
    }

    break;
  }

  if (currentLine.length > 1) lines.push(currentLine);
  return lines;
}

function rebuildVisionTrackPolyline() {
  let lines = [];

  if (sim.track.preset === "circle") {
    lines = [sampleCircleArcPoints(1.15, 0, 1.15, Math.PI, 3 * Math.PI, 280)];
  } else if (sim.track.preset === "figure8") {
    lines = [
      sampleEllipsePoints(-0.7, 0, 0.7, 0.7, 170),
      sampleEllipsePoints(0.7, 0, 0.7, 0.7, 170),
    ];
  } else if (sim.track.preset === "oval") {
    lines = [sampleEllipsePoints(0, 0, 1.45, 0.9, 220)];
  } else if (sim.track.preset === "svg") {
    lines = parseSvgPathToPolyline(sim.track.svgPath);
  }

  sim.track.visionPolyline = lines
    .filter((line) => Array.isArray(line) && line.length > 1)
    .map((line) => densifyPolyline(line, 0.03));
}

function applyTrackAndSensorConfig() {
  try {
    sim.track.preset = cfgTrackPreset.value;
    sim.track.lineWidth = readNumberInput(cfgLineWidth, 0.005, 0.2, sim.track.lineWidth);
    sim.track.svgPath = cfgSvgPath.value.trim();
    sim.track.sensors = parseSensorConfig(cfgSensors.value);

    sim.track.svgPath2D = null;
    if (sim.track.preset === "svg") {
      if (!sim.track.svgPath) {
        throw new Error("SVG path is empty.");
      }
      sim.track.svgPath2D = new Path2D(sim.track.svgPath);
    }

    cfgLineWidth.value = String(sim.track.lineWidth);
    rebuildTrackMask();
    rebuildVisionTrackPolyline();
    updateSensorReadings();
    setTrackStatus(
      `Track applied: ${sim.track.preset}, line width ${n(sim.track.lineWidth, 3)}m, sensors ${sim.track.sensors.length}.`
    );
  } catch (err) {
    setTrackStatus(`Track/sensor config error: ${err.message}`, true);
  }
}

function stepSimulation(dt) {
  updateSensorReadings();

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

function screenToWorld(screenX, screenY, camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
  return {
    x: (screenX - camCenterX) / scale + cameraWorldX,
    y: -(screenY - camCenterY) / scale + cameraWorldY,
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
  sim.trackWidth = readNumberInput(cfgTrackWidth, 0.1, 5, sim.trackWidth);
  sim.wheelBase = readNumberInput(cfgWheelBase, 0.1, 8, sim.wheelBase);
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

function drawTrack(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
  ctx.save();
  ctx.translate(camCenterX, camCenterY);
  ctx.scale(scale, -scale);
  ctx.translate(-cameraWorldX, -cameraWorldY);
  ctx.strokeStyle = "#222831";
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.globalAlpha = 0.9;
  ctx.lineWidth = sim.track.lineWidth;
  drawTrackGeometry(ctx);
  ctx.restore();

  if (sim.track.draftPoints.length > 0) {
    ctx.save();
    ctx.strokeStyle = "rgba(31, 136, 82, 0.95)";
    ctx.fillStyle = "rgba(31, 136, 82, 0.95)";
    ctx.lineWidth = 2;

    ctx.beginPath();
    for (let i = 0; i < sim.track.draftPoints.length; i += 1) {
      const p = sim.track.draftPoints[i];
      const s = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      if (i === 0) {
        ctx.moveTo(s.x, s.y);
      } else {
        ctx.lineTo(s.x, s.y);
      }
    }
    ctx.stroke();

    for (const p of sim.track.draftPoints) {
      const s = worldToScreen(
        p.x - cameraWorldX,
        p.y - cameraWorldY,
        camCenterX,
        camCenterY,
        scale
      );
      ctx.beginPath();
      ctx.arc(s.x, s.y, 3.5, 0, 2 * Math.PI);
      ctx.fill();
    }
    ctx.restore();
  }
}

function draftPointsToSvgPath(points) {
  if (points.length < 2) return "";
  const parts = [`M ${points[0].x.toFixed(3)} ${points[0].y.toFixed(3)}`];
  for (let i = 1; i < points.length; i += 1) {
    parts.push(`L ${points[i].x.toFixed(3)} ${points[i].y.toFixed(3)}`);
  }
  return parts.join(" ");
}

function beginTrackDraw() {
  sim.track.drawMode = true;
  sim.track.draftPoints = [];
  canvas.classList.add("draw-mode");
  setTrackStatus("Draw mode enabled: click canvas to add points, then Finish Draw.");
}

function clearTrackDraft() {
  sim.track.draftPoints = [];
  setTrackStatus("Draft points cleared.");
}

function finishTrackDrawMode() {
  if (sim.track.draftPoints.length < 2) {
    setTrackStatus("Need at least 2 points to build a track path.", true);
    return;
  }

  const svgPath = draftPointsToSvgPath(sim.track.draftPoints);
  cfgSvgPath.value = svgPath;
  cfgTrackPreset.value = "svg";
  sim.track.drawMode = false;
  canvas.classList.remove("draw-mode");
  applyTrackAndSensorConfig();
  setTrackStatus(`Draw complete with ${sim.track.draftPoints.length} points. SVG path applied.`);
}

function drawCar(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale) {
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

  const wheelL = 0.14 * scale * sim.wheelVisualScale;
  const wheelW = 0.07 * scale * sim.wheelVisualScale;
  const xOffset = bodyLength / 2 - wheelL * 0.65;
  const yOffset = bodyWidth / 2 + wheelW * 0.45;
  ctx.fillStyle = "#1f3344";

  ctx.fillRect(-xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(-xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, -yOffset, wheelL, wheelW);
  ctx.fillRect(xOffset - wheelL / 2, yOffset - wheelW, wheelL, wheelW);

  // Draw IR sensor dots in local frame (x forward, y left).
  for (const sensor of sim.track.sensorReadings) {
    const sx = sensor.x * scale;
    const sy = -sensor.y * scale;
    const intensity = clamp(sensor.value, 0, 1);
    const r = 4;
    ctx.fillStyle = `rgba(${Math.round(220 - 170 * intensity)}, ${Math.round(80 + 120 * intensity)}, 52, 0.95)`;
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, 2 * Math.PI);
    ctx.fill();
  }

  ctx.restore();
}

function renderScene() {
  const scale = sim.camera.scale;
  const camCenterX = canvas.width / 2 + sim.camera.offsetX;
  const camCenterY = canvas.height / 2 + sim.camera.offsetY;
  const cameraWorldX = sim.camera.followCar ? sim.pose.x : 0;
  const cameraWorldY = sim.camera.followCar ? sim.pose.y : 0;
  const viewCenter = worldToScreen(-cameraWorldX, -cameraWorldY, camCenterX, camCenterY, scale);

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid(scale, viewCenter.x, viewCenter.y);
  drawTrack(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale);
  drawCar(camCenterX, camCenterY, cameraWorldX, cameraWorldY, scale);
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

for (const visionInput of [visionCamX, visionCamY, visionCamZ, visionPitch, visionFovH, visionFovV]) {
  visionInput.addEventListener("change", applyVisionConfigFromInputs);
}

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

if (applyTrack) applyTrack.addEventListener("click", applyTrackAndSensorConfig);
if (applyTrackSensors) applyTrackSensors.addEventListener("click", applyTrackAndSensorConfig);
startTrackDraw.addEventListener("click", beginTrackDraw);
finishTrackDraw.addEventListener("click", finishTrackDrawMode);
clearTrackDraw.addEventListener("click", clearTrackDraft);

canvas.addEventListener("mousedown", (event) => {
  if (sim.track.drawMode) {
    const rect = canvas.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / rect.width) * canvas.width;
    const y = ((event.clientY - rect.top) / rect.height) * canvas.height;
    const camCenterX = canvas.width / 2 + sim.camera.offsetX;
    const camCenterY = canvas.height / 2 + sim.camera.offsetY;
    const cameraWorldX = sim.camera.followCar ? sim.pose.x : 0;
    const cameraWorldY = sim.camera.followCar ? sim.pose.y : 0;
    const world = screenToWorld(x, y, camCenterX, camCenterY, cameraWorldX, cameraWorldY, sim.camera.scale);
    sim.track.draftPoints.push(world);
    setTrackStatus(`Added point ${sim.track.draftPoints.length}: (${n(world.x, 2)}, ${n(world.y, 2)})`);
    return;
  }

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
  updateSensorReadings();
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
    updateSensorReadings();
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
  updateOpenCvStatus();
  const dt = Math.min(sim.maxDt, (now - lastTime) / 1000);
  lastTime = now;

  if (sim.running) {
    stepSimulation(dt);
    sim.time += dt;
  }

  renderScene();
  renderVision();
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
syncVisionInputs();
renderMemoryView();
applyTrackAndSensorConfig();
applyVisionConfigFromInputs();
addDebugLine("Debug ready. Use api.log(...) and api.clearLog().");
requestAnimationFrame(loop);
