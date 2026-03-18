# Skid-Steer Car Simulator

Interactive browser simulator for a 4-wheel skid-steer robot/car with scriptable control and camera-based perception.

    Copyright (C) 2026 Redstone TDPS

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.



## What this project does

This single-page app models skid-steer kinematics from wheel speeds:
- left-side speed = average of `FL` and `RL`
- right-side speed = average of `FR` and `RR`
- forward speed = average of left/right side speeds
- yaw rate = `(vRight - vLeft) / trackWidth`

Pose is integrated with exact arc motion for turns and linear motion for straight travel. The simulator reports telemetry and motion-state diagnostics while rendering a top-down scene plus a forward telemetry camera view.

## Run locally

No build step is required.

Option 1:
- Open `index.html` directly in a modern browser.

Option 2 (recommended):
- Serve the folder as static files, then open the served URL.

Example:

```bash
python -m http.server 8000
```

Then open `http://localhost:8000`.

## Current UI layout

### Top header
- Project/homepage link (logo)
- Repository link

### Main simulator panel
- Global settings:
	- kinematics/visual limits (`w`, `l`, max wheel speed, wheel visual scale, trail points, max `dt`)
	- vision camera params (`x`, `y`, `z`, pitch, horizontal/vertical FOV)
- Action group (left):
	- pause/resume (icon-only)
	- reset pose
	- enable/disable code mode (single toggle button)
- Action group (right):
	- export/import JSON config
- 2D top-down canvas
- Code control section:
	- code editor (CodeMirror)
	- run once
	- clear memory
	- clear debug
	- code status, API list, script-memory view, debug log

### Side panels
- **Wheel Controls**: manual wheel speed nudges and zero speeds
- **Telemetry**:
	- overlay visibility toggle (`Overlays: On/Off`)
	- vision status
	- telemetry camera canvas
	- numeric metrics
- **Track**: preset selection, line width, SVG path, apply/draw/finish/clear controls
- **IR Sensors**: sensor JSON + apply sensors
- **Camera Controls**: pan/zoom/follow controls for the top-down view

## JSON import/export

Import/export uses a single JSON payload and includes:
- simulator settings
- track settings (including SVG path)
- camera settings (top-down view and vision camera)
- IR sensor configuration
- current code editor source

On import/export errors, the app shows an alert dialog.

## Vision & overlays

- Telemetry camera renders a simulated forward view of the track.
- Controller overlay primitives are drawn on the displayed telemetry canvas only.
- `api.getVisionFrame()` reads from a separate raw offscreen vision buffer, so overlays are **not** included in vision input.

## Script API

When your controller runs, it receives `api` with:

- `api.time`
- `api.dt`
- `api.pose`
- `api.wheelSpeeds`
- `api.getVisionFrame()` → `{ width, height, imageData }` (raw frame, no overlays)
- `api.drawLine(x1, y1, x2, y2, options)`
- `api.drawRect(x, y, w, h, options)`
- `api.drawCircle(x, y, r, options)`
- `api.clearOverlay()`
- `api.sensors`
- `api.getSensor(name)`
- `api.setWheelSpeed(name, value)`
- `api.setWheelSpeeds(vFL, vFR, vRL, vRR)`
- `api.clamp(value, min, max)`
- `api.cv` (OpenCV object when loaded, otherwise `null`)
- `api.log(...args)`
- `api.clearLog()`
- `api.mem.get(key, defaultValue)`
- `api.mem.set(key, value)`
- `api.mem.remove(key)`
- `api.mem.clear()`
- `api.mem.keys()`

`options` for draw APIs supports `stroke`, `fill`, `lineWidth`, and `alpha`.

## File overview

- `index.html`: UI structure
- `app.js`: simulation loop, rendering, controls, script API, import/export
- `car_pose.js`: kinematics model and motion diagnostics
- `styles.css`: styling

## Notes

- Requires a modern browser with Canvas 2D support.
- Dependencies are loaded from local `./dep/` assets in this project.