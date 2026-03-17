# Skid-Steer Car Simulator

Interactive browser simulator for a 4-wheel skid-steer robot/car.

It lets you:
- drive each wheel manually,
- tune kinematics and visual parameters,
- build or choose line tracks,
- read virtual IR sensors,
- run custom JavaScript control logic each frame (line follower, search behavior, etc.),
- inspect telemetry and debug output.

## What this project does

This is a single-page simulation (no backend) that models skid-steer kinematics from wheel speeds:
- Left side speed: average of `FL` and `RL`
- Right side speed: average of `FR` and `RR`
- Forward speed: average of left/right side speeds
- Yaw rate: `(vRight - vLeft) / trackWidth`

Pose integration uses exact arc integration for turning and linear integration for straight motion. The simulator also classifies motion states (for example `PIVOT_LEFT`, `FORWARD_RIGHT`, `STATIONARY`) and computes a simple slip indicator from front/rear speed mismatch on each side.

## Run locally

No build step is required.

Option 1:
- Open `index.html` directly in a modern browser.

Option 2 (recommended):
- Serve the folder with a local static server, then open the served URL.

Example with Python:

```bash
python -m http.server 8000
```

Then open `http://localhost:8000`.

## Main UI features

### 1) Wheel controls
- Pause/resume simulation
- Zero all wheel speeds
- Reset pose and trail
- Increment/decrement each wheel speed (`FL`, `FR`, `RL`, `RR`)

### 2) Simulator config
- Track width `w`
- Wheelbase `l`
- Max wheel speed clamp
- Wheel visual scale
- Trail point count
- Max integration step `dt`

### 3) Track & IR sensors
- Track presets: `Oval`, `Circle`, `Figure 8`, `SVG Path`
- Adjustable line width
- Custom SVG path string input
- Sensor layout via JSON (car-local frame: `x` forward, `y` left)
- Draw mode to create a path by clicking points on the canvas and convert to SVG path

### 4) Camera controls
- Pan by dragging the canvas
- Zoom with wheel or zoom slider
- Toggle follow-car mode
- Reset camera

### 5) Code mode (embedded controller)
- Edit JavaScript in the built-in editor (CodeMirror)
- `Enable Code Mode`: run script every simulation frame
- `Run Once`: execute a single step
- `Disable Code Mode`
- `Clear Debug`

The script can read sensors, set wheel speeds, use persistent script memory, and log to the debug panel.

## Script API

When your controller runs, it receives an `api` object with:

- `api.time`
- `api.dt`
- `api.pose`
- `api.wheelSpeeds`
- `api.sensors`
- `api.getSensor(name)`
- `api.setWheelSpeed(name, value)`
- `api.setWheelSpeeds(vFL, vFR, vRL, vRR)`
- `api.clamp(value, min, max)`
- `api.log(...args)`
- `api.clearLog()`
- `api.mem.get(key, defaultValue)`
- `api.mem.set(key, value)`
- `api.mem.remove(key)`
- `api.mem.clear()`
- `api.mem.keys()`

## File overview

- `index.html`: UI layout and controls
- `app.js`: simulation loop, rendering, UI wiring, code execution sandbox, track/sensor system
- `car_pose.js`: skid-steer kinematics and motion diagnostics
- `styles.css`: styles

## Notes

- Intended for modern browsers with Canvas 2D support.
- External CDN dependencies are used for CodeMirror and Google Fonts.