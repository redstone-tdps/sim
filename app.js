const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const startupNotice = document.getElementById("startupNotice");
const closeStartupNotice = document.getElementById("closeStartupNotice");
const wheelGrid = document.getElementById("wheelGrid");
const metrics = document.getElementById("metrics");
const codeInput = document.getElementById("codeInput");
const codeStatus = document.getElementById("codeStatus");
const toggleCodeMode = document.getElementById("toggleCodeMode");
const memoryView = document.getElementById("memoryView");
const debugLog = document.getElementById("debugLog");
const clearDebug = document.getElementById("clearDebug");
const clearMemory = document.getElementById("clearMemory");
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
const visionRawCanvas = document.createElement("canvas");
visionRawCanvas.width = visionCanvas.width;
visionRawCanvas.height = visionCanvas.height;
const visionRawCtx = visionRawCanvas.getContext("2d", { willReadFrequently: true });
const visionCamX = document.getElementById("visionCamX");
const visionCamY = document.getElementById("visionCamY");
const visionCamZ = document.getElementById("visionCamZ");
const visionPitch = document.getElementById("visionPitch");
const visionFovH = document.getElementById("visionFovH");
const visionFovV = document.getElementById("visionFovV");
const toggleOverlay = document.getElementById("toggleOverlay");
const visionStatus = document.getElementById("visionStatus");
const cfgTrackWidth = document.getElementById("cfgTrackWidth");
const cfgWheelBase = document.getElementById("cfgWheelBase");
const cfgMaxWheelSpeed = document.getElementById("cfgMaxWheelSpeed");
const cfgWheelVisualScale = document.getElementById("cfgWheelVisualScale");
const cfgMaxWheelAccel = document.getElementById("cfgMaxWheelAccel");
const cfgTrailPoints = document.getElementById("cfgTrailPoints");
const cfgMaxDt = document.getElementById("cfgMaxDt");
const cfgTrackPreset = document.getElementById("cfgTrackPreset");
const cfgLineWidth = document.getElementById("cfgLineWidth");
const cfgSvgPath = document.getElementById("cfgSvgPath");
const cfgSensors = document.getElementById("cfgSensors");
const exportJson = document.getElementById("exportJson");
const importJson = document.getElementById("importJson");
const importJsonFile = document.getElementById("importJsonFile");
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

const ANALYTICS_WORKER_URL = "https://api.sim.redstone-tdps.top/upload";

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
  targetWheelSpeeds: { FL: 0, FR: 0, RL: 0, RR: 0 },
  trackWidth: 0.2,
  wheelBase: 0.4,
  trail: [],
  lastKinematics: null,
  lastDiagnostics: null,
  controller: null,
  maxTrailPoints: 900,
  maxAbsWheelSpeed: 4.0,
  maxWheelAccel: 8.0,
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
    showOverlay: true,
    overlayPrimitives: [],
    maxOverlayPrimitives: 400,
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
  sim.targetWheelSpeeds[name] = clamp(value, -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
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
    const current = sim.wheelSpeeds[name];
    const target = sim.targetWheelSpeeds[name];
    if (Math.abs(current - target) < 1e-3) {
      valueEl.textContent = `${n(current, 2)} m/s`;
    } else {
      valueEl.textContent = `${n(current, 2)} -> ${n(target, 2)} m/s`;
    }
  }
}
function applyWheelAccelerationLimit(dt) {
  if (!(dt > 0)) return;

  const maxDelta = sim.maxWheelAccel * dt;
  for (const name of wheelNames) {
    const current = sim.wheelSpeeds[name];
    const target = clamp(sim.targetWheelSpeeds[name], -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
    const delta = clamp(target - current, -maxDelta, maxDelta);
    sim.wheelSpeeds[name] = clamp(current + delta, -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
    updateWheelValue(name);
  }
}

function setTrackStatus(message, isError = false) {
  trackStatus.textContent = message;
  trackStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function setIoStatus(message, isError = false) {
  if (isError) {
    window.alert(message);
  }
}

function setVisionStatus(message, isError = false) {
  visionStatus.textContent = message;
  visionStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function updateOverlayToggleButton() {
  if (!toggleOverlay) return;
  toggleOverlay.innerHTML = sim.vision.showOverlay
    ? "<i class=\"bi bi-eye\"></i><span>Overlays: On</span>"
    : "<i class=\"bi bi-eye-slash\"></i><span>Overlays: Off</span>";
}

function showStartupNotice() {
  if (!startupNotice) return;
  startupNotice.classList.remove("hidden");
}

function hideStartupNotice() {
  if (!startupNotice) return;
  startupNotice.classList.add("hidden");
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
      setWheelSpeed(name, sim.targetWheelSpeeds[name] - 0.2);
    });

    const plus = document.createElement("button");
    plus.className = "small-btn secondary";
    plus.textContent = "+0.2";
    plus.addEventListener("click", () => {
      setWheelSpeed(name, sim.targetWheelSpeeds[name] + 0.2);
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
  const source = getControllerSource();
  sim.controller = new Function("api", source);
}

function getControllerSource() {
  return codeEditor ? codeEditor.getValue() : codeInput.value;
}

function setControllerSource(source) {
  const text = typeof source === "string" ? source : "";
  if (codeEditor) {
    codeEditor.setValue(text);
  } else {
    codeInput.value = text;
  }
}

function buildExportPayload() {
  let sensors = [];
  try {
    sensors = parseSensorConfig(cfgSensors.value);
  } catch {
    sensors = [];
  }

  return {
    schema: "skid-steer-sim-config-v1",
    exportedAt: new Date().toISOString(),
    settings: {
      trackWidth: Number(cfgTrackWidth.value),
      wheelBase: Number(cfgWheelBase.value),
      maxWheelSpeed: Number(cfgMaxWheelSpeed.value),
      maxWheelAccel: Number(cfgMaxWheelAccel.value),
      wheelVisualScale: Number(cfgWheelVisualScale.value),
      trailPoints: Number(cfgTrailPoints.value),
      maxDt: Number(cfgMaxDt.value),
      trackPreset: cfgTrackPreset.value,
      lineWidth: Number(cfgLineWidth.value),
      svgPath: cfgSvgPath.value,
    },
    camera: {
      view: {
        offsetX: sim.camera.offsetX,
        offsetY: sim.camera.offsetY,
        zoom: sim.camera.scale,
        followCar: sim.camera.followCar,
      },
      vision: {
        x: Number(visionCamX.value),
        y: Number(visionCamY.value),
        z: Number(visionCamZ.value),
        pitchDeg: Number(visionPitch.value),
        fovHDeg: Number(visionFovH.value),
        fovVDeg: Number(visionFovV.value),
      },
    },
    irSensors: {
      sensors,
    },
    code: {
      source: getControllerSource(),
    },
  };
}

function buildExportJsonBlob(payload) {
  const text = JSON.stringify(payload, null, 2);
  return new Blob([text], { type: "application/json" });
}

function downloadJsonBlob(blob, fileName) {
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = fileName;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function setIfFinite(inputEl, value) {
  const num = Number(value);
  if (Number.isFinite(num)) inputEl.value = String(num);
}

function applyImportedPayload(payload) {
  if (!payload || typeof payload !== "object") {
    throw new Error("Imported JSON root must be an object.");
  }

  const settings = payload.settings || {};
  setIfFinite(cfgTrackWidth, settings.trackWidth);
  setIfFinite(cfgWheelBase, settings.wheelBase);
  setIfFinite(cfgMaxWheelSpeed, settings.maxWheelSpeed);
  setIfFinite(cfgMaxWheelAccel, settings.maxWheelAccel);
  setIfFinite(cfgWheelVisualScale, settings.wheelVisualScale);
  setIfFinite(cfgTrailPoints, settings.trailPoints);
  setIfFinite(cfgMaxDt, settings.maxDt);
  if (typeof settings.trackPreset === "string") cfgTrackPreset.value = settings.trackPreset;
  setIfFinite(cfgLineWidth, settings.lineWidth);
  if (typeof settings.svgPath === "string") cfgSvgPath.value = settings.svgPath;

  const camera = payload.camera || {};
  const view = camera.view || {};
  setIfFinite(camOffsetX, view.offsetX);
  setIfFinite(camOffsetY, view.offsetY);
  setIfFinite(camZoom, view.zoom);
  if (typeof view.followCar === "boolean") {
    sim.camera.followCar = view.followCar;
  }

  const vision = camera.vision || {};
  setIfFinite(visionCamX, vision.x);
  setIfFinite(visionCamY, vision.y);
  setIfFinite(visionCamZ, vision.z);
  setIfFinite(visionPitch, vision.pitchDeg);
  setIfFinite(visionFovH, vision.fovHDeg);
  setIfFinite(visionFovV, vision.fovVDeg);

  const irSensors = payload.irSensors || {};
  if (Array.isArray(irSensors.sensors)) {
    cfgSensors.value = JSON.stringify(irSensors.sensors, null, 2);
  }

  const code = payload.code || {};
  if (typeof code.source === "string") {
    setControllerSource(code.source);
  }

  applySimConfigFromInputs();
  sim.camera.offsetX = readNumberInput(camOffsetX, -5000, 5000, sim.camera.offsetX);
  sim.camera.offsetY = readNumberInput(camOffsetY, -5000, 5000, sim.camera.offsetY);
  sim.camera.scale = readNumberInput(camZoom, 10, 300, sim.camera.scale);
  syncCameraInputs();
  updateCameraLabels();

  applyTrackAndSensorConfig();
  applyVisionConfigFromInputs();
}

function uploadJsonFileAsync(fileOrBlob) {
  if (!ANALYTICS_WORKER_URL) return;

  fetch(ANALYTICS_WORKER_URL, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: fileOrBlob,
    keepalive: true,
  }).catch((error) => {
    console.warn("JSON upload failed:", error);
  });
}

function toFiniteNumber(value, fallback = 0) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function toOverlayStyle(options) {
  const source = options && typeof options === "object" ? options : {};
  const stroke = typeof source.stroke === "string" ? source.stroke : "#ff375f";
  const fill = typeof source.fill === "string" ? source.fill : null;
  const lineWidth = clamp(toFiniteNumber(source.lineWidth, 2), 0.5, 20);
  const alpha = clamp(toFiniteNumber(source.alpha, 1), 0, 1);
  return { stroke, fill, lineWidth, alpha };
}

function pushVisionOverlayPrimitive(primitive) {
  sim.vision.overlayPrimitives.push(primitive);
  if (sim.vision.overlayPrimitives.length > sim.vision.maxOverlayPrimitives) {
    sim.vision.overlayPrimitives.shift();
  }
}

function clearVisionOverlayPrimitives() {
  sim.vision.overlayPrimitives = [];
}

function drawVisionOverlayPrimitives() {
  if (sim.vision.overlayPrimitives.length === 0) return;

  visionCtx.save();
  for (const primitive of sim.vision.overlayPrimitives) {
    const style = primitive.style || { stroke: "#ff375f", fill: null, lineWidth: 2, alpha: 1 };
    visionCtx.globalAlpha = style.alpha;
    visionCtx.strokeStyle = style.stroke;
    visionCtx.fillStyle = style.fill || "transparent";
    visionCtx.lineWidth = style.lineWidth;

    if (primitive.type === "line") {
      visionCtx.beginPath();
      visionCtx.moveTo(primitive.x1, primitive.y1);
      visionCtx.lineTo(primitive.x2, primitive.y2);
      visionCtx.stroke();
      continue;
    }

    if (primitive.type === "rect") {
      if (style.fill) {
        visionCtx.fillRect(primitive.x, primitive.y, primitive.w, primitive.h);
      }
      visionCtx.strokeRect(primitive.x, primitive.y, primitive.w, primitive.h);
      continue;
    }

    if (primitive.type === "circle") {
      visionCtx.beginPath();
      visionCtx.arc(primitive.x, primitive.y, primitive.r, 0, 2 * Math.PI);
      if (style.fill) {
        visionCtx.fill();
      }
      visionCtx.stroke();
    }
  }
  visionCtx.restore();
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
      imageData: visionRawCtx.getImageData(0, 0, visionCanvas.width, visionCanvas.height),
    }),
    drawLine: (x1, y1, x2, y2, options = {}) => {
      pushVisionOverlayPrimitive({
        type: "line",
        x1: toFiniteNumber(x1),
        y1: toFiniteNumber(y1),
        x2: toFiniteNumber(x2),
        y2: toFiniteNumber(y2),
        style: toOverlayStyle(options),
      });
    },
    drawRect: (x, y, w, h, options = {}) => {
      pushVisionOverlayPrimitive({
        type: "rect",
        x: toFiniteNumber(x),
        y: toFiniteNumber(y),
        w: toFiniteNumber(w),
        h: toFiniteNumber(h),
        style: toOverlayStyle(options),
      });
    },
    drawCircle: (x, y, r, options = {}) => {
      pushVisionOverlayPrimitive({
        type: "circle",
        x: toFiniteNumber(x),
        y: toFiniteNumber(y),
        r: Math.max(0, toFiniteNumber(r)),
        style: toOverlayStyle(options),
      });
    },
    clearOverlay: clearVisionOverlayPrimitives,
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
  const drawCtx = visionRawCtx;
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
  drawCtx.fillStyle = "#c9e0f3";
  drawCtx.fillRect(0, 0, width, Math.max(0, Math.min(height, horizonY)));
  drawCtx.fillStyle = "#d6cdbb";
  drawCtx.fillRect(0, Math.max(0, Math.min(height, horizonY)), width, height);

  drawCtx.strokeStyle = "rgba(20, 45, 66, 0.35)";
  drawCtx.lineWidth = 1;
  drawCtx.beginPath();
  drawCtx.moveTo(0, horizonY);
  drawCtx.lineTo(width, horizonY);
  drawCtx.stroke();

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
    return { x: px, y: py, z: camZ };
  };

  drawCtx.fillStyle = "#202020";

  for (const line of sim.track.visionPolyline) {
    const halfW = sim.track.lineWidth * 0.5;
    let runLeft = [];
    let runRight = [];

    const flushRun = () => {
      if (runLeft.length < 2 || runRight.length < 2) {
        runLeft = [];
        runRight = [];
        return;
      }

      drawCtx.beginPath();
      drawCtx.moveTo(runLeft[0].x, runLeft[0].y);
      for (let i = 1; i < runLeft.length; i += 1) {
        drawCtx.lineTo(runLeft[i].x, runLeft[i].y);
      }
      for (let i = runRight.length - 1; i >= 0; i -= 1) {
        drawCtx.lineTo(runRight[i].x, runRight[i].y);
      }
      drawCtx.closePath();
      drawCtx.fill();
      runLeft = [];
      runRight = [];
    };

    for (let index = 0; index < line.length; index += 1) {
      const current = line[index];
      const prev = index > 0 ? line[index - 1] : null;
      const next = index + 1 < line.length ? line[index + 1] : null;

      let tx = 0;
      let ty = 0;
      if (prev && next) {
        tx = next.x - prev.x;
        ty = next.y - prev.y;
      } else if (next) {
        tx = next.x - current.x;
        ty = next.y - current.y;
      } else if (prev) {
        tx = current.x - prev.x;
        ty = current.y - prev.y;
      }

      const len = Math.hypot(tx, ty);
      if (len < 1e-6) {
        flushRun();
        continue;
      }

      const nx = -ty / len;
      const ny = tx / len;

      const left = projectWorldPoint(current.x + nx * halfW, current.y + ny * halfW);
      const right = projectWorldPoint(current.x - nx * halfW, current.y - ny * halfW);

      if (!left || !right) {
        flushRun();
        continue;
      }

      runLeft.push(left);
      runRight.push(right);
    }

    flushRun();
  }

  visionCtx.clearRect(0, 0, width, height);
  visionCtx.drawImage(visionRawCanvas, 0, 0, width, height);

  if (sim.vision.showOverlay) {
    drawVisionOverlayPrimitives();
  }
  clearVisionOverlayPrimitives();
}

function setStatus(message, isError = false) {
  codeStatus.textContent = message;
  codeStatus.style.color = isError ? "#9e352f" : "#27465f";
}

function updateCodeModeButton() {
  if (!toggleCodeMode) return;
  if (sim.codeMode) {
    toggleCodeMode.classList.remove("safe");
    toggleCodeMode.classList.add("warn");
    toggleCodeMode.innerHTML = "<i class=\"bi bi-stop-circle\"></i><span>Disable Code Mode</span>";
  } else {
    toggleCodeMode.classList.remove("warn");
    toggleCodeMode.classList.add("safe");
    toggleCodeMode.innerHTML = "<i class=\"bi bi-play-circle\"></i><span>Enable Code Mode</span>";
  }
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
  if (typeof pathText !== "string" || !pathText.trim()) return [];

  const NS = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(NS, "svg");
  const path = document.createElementNS(NS, "path");
  svg.appendChild(path);

  const sampleSubPath = (subPathText) => {
    path.setAttribute("d", subPathText);
    const totalLength = path.getTotalLength();
    if (!Number.isFinite(totalLength) || totalLength <= 0) return [];

    const sampleCount = Math.max(2, Math.ceil(totalLength / 0.03));
    const points = [];
    for (let i = 0; i <= sampleCount; i += 1) {
      const d = (i / sampleCount) * totalLength;
      const point = path.getPointAtLength(d);
      points.push({ x: point.x, y: point.y });
    }
    return points;
  };

  try {
    const subPaths = pathText.match(/[Mm][^Mm]*/g) || [pathText];
    return subPaths
      .map((subPath) => sampleSubPath(subPath.trim()))
      .filter((line) => line.length > 1);
  } catch {
    return [];
  }
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
      updateCodeModeButton();
      setStatus(`Code error: ${err.message}`, true);
    }
  }
  applyWheelAccelerationLimit(dt);

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
  camFollowToggle.innerHTML = `<i class="bi bi-bullseye"></i><span>Follow Car: ${sim.camera.followCar ? "On" : "Off"}</span>`;
}

function syncSimConfigInputs() {
  cfgTrackWidth.value = String(sim.trackWidth);
  cfgWheelBase.value = String(sim.wheelBase);
  cfgMaxWheelSpeed.value = String(sim.maxAbsWheelSpeed);
  cfgMaxWheelAccel.value = String(sim.maxWheelAccel);
  cfgWheelVisualScale.value = String(sim.wheelVisualScale);
  cfgTrailPoints.value = String(sim.maxTrailPoints);
  cfgMaxDt.value = String(sim.maxDt);
}

function applySimConfigFromInputs() {
  sim.trackWidth = readNumberInput(cfgTrackWidth, 0.1, 5, sim.trackWidth);
  sim.wheelBase = readNumberInput(cfgWheelBase, 0.1, 8, sim.wheelBase);
  sim.maxAbsWheelSpeed = readNumberInput(cfgMaxWheelSpeed, 0.2, 20, sim.maxAbsWheelSpeed);
  sim.maxWheelAccel = readNumberInput(cfgMaxWheelAccel, 0.1, 100, sim.maxWheelAccel);
  sim.wheelVisualScale = readNumberInput(cfgWheelVisualScale, 0.2, 3, sim.wheelVisualScale);
  sim.maxTrailPoints = Math.round(readNumberInput(cfgTrailPoints, 50, 20000, sim.maxTrailPoints));
  sim.maxDt = readNumberInput(cfgMaxDt, 0.005, 0.2, sim.maxDt);

  for (const name of wheelNames) {
    sim.targetWheelSpeeds[name] = clamp(sim.targetWheelSpeeds[name], -sim.maxAbsWheelSpeed, sim.maxAbsWheelSpeed);
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

if (clearMemory) {
  clearMemory.addEventListener("click", () => {
    sim.scriptMemory = {};
    renderMemoryView();
    setStatus("Script memory cleared.");
  });
}

for (const configInput of [
  cfgTrackWidth,
  cfgWheelBase,
  cfgMaxWheelSpeed,
  cfgMaxWheelAccel,
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

if (exportJson) {
  exportJson.addEventListener("click", () => {
    try {
      const payload = buildExportPayload();
      const fileBlob = buildExportJsonBlob(payload);
      downloadJsonBlob(fileBlob, "skid-steer-sim-config.json");
      uploadJsonFileAsync(fileBlob);
    } catch (err) {
      setIoStatus(`Export failed: ${err.message}`, true);
    }
  });
}

if (importJson) {
  importJson.addEventListener("click", () => {
    if (importJsonFile) importJsonFile.click();
  });
}

if (importJsonFile) {
  importJsonFile.addEventListener("change", async (event) => {
    const file = event.target.files && event.target.files[0];
    if (!file) return;

    try {
      const text = await file.text();
      const parsed = JSON.parse(text);
      applyImportedPayload(parsed);
      uploadJsonFileAsync(file);
    } catch (err) {
      setIoStatus(`Import failed: ${err.message}`, true);
    } finally {
      importJsonFile.value = "";
    }
  });
}

if (toggleOverlay) {
  toggleOverlay.addEventListener("click", () => {
    sim.vision.showOverlay = !sim.vision.showOverlay;
    updateOverlayToggleButton();
  });
}

if (closeStartupNotice) {
  closeStartupNotice.addEventListener("click", hideStartupNotice);
}

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
  const button = event.currentTarget;
  button.innerHTML = sim.running ? "<i class=\"bi bi-pause-fill\"></i>" : "<i class=\"bi bi-play-fill\"></i>";
  const label = sim.running ? "Pause simulation" : "Resume simulation";
  button.setAttribute("aria-label", label);
  button.setAttribute("title", label);
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

if (toggleCodeMode) {
  toggleCodeMode.addEventListener("click", () => {
    if (sim.codeMode) {
      sim.codeMode = false;
      updateCodeModeButton();
      setStatus("Code mode disabled.");
      return;
    }

    try {
      compileController();
      sim.codeMode = true;
      updateCodeModeButton();
      setStatus("Code mode enabled. Your script is running every frame.");
    } catch (err) {
      sim.codeMode = false;
      updateCodeModeButton();
      setStatus(`Compile error: ${err.message}`, true);
    }
  });
}

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
updateOverlayToggleButton();
updateCodeModeButton();
showStartupNotice();
addDebugLine("Debug ready. Use api.log(...) and api.clearLog().");
requestAnimationFrame(loop);
