# Tactile Glove Demo Viewer — Windows

A self-contained, **offline** local viewer for tactile gloves. It serves a
browser UI at `http://127.0.0.1:8877` showing:

- Per-finger bend bars
- 16×16 pressure heatmap
- A **3D hand** (anatomical model) driven by the glove IMU + finger sensors
- Orientation ("zero") calibration and finger-bend calibration
- Demo recording + replay (works with **no gloves connected**)

Everything (3D engine, hand model) is bundled locally — **no internet required** to run.

---

## Quick start

Already set up? Double-click `run_windows.bat`, or from PowerShell:

```powershell
cd C:\tactile-glove-demo-windows
.\run_windows.bat
```

Then open **http://127.0.0.1:8877** in your browser. No gloves required — you can
upload and replay recordings immediately.

---

## What's in this package

```
tactile-glove-demo-windows/
  tactile_glove_viewer.py     # main app (HTTP server + browser UI)
  linker_glove_agent.py       # serial protocol parser + sensor mapping
  probe_glove_ports.py        # COM-port auto-detection / standalone probe tool
  time_utils.py               # timestamp helper
  assets/                     # bundled 3D engine + hand model (served locally)
    three.module.js
    GLTFLoader.js
    hand_model.glb
    utils/SkeletonUtils.js
    utils/BufferGeometryUtils.js
  calibration/                # your saved calibration files land here
  recordings/                 # your saved recordings land here
  requirements.txt            # one dependency: pyserial
  run_windows.bat             # one-click launcher
  README.md
```

---

## Step-by-step setup (first use)

Follow these steps the first time you install the viewer on a Windows PC. After that,
use [Quick start](#quick-start) above.

### 1. Install Python 3.9+ (3.10 or 3.11 recommended)

- Download from <https://www.python.org/downloads/windows/>
- During install, **check the box "Add python.exe to PATH"**.
- Verify in a new terminal:

```powershell
python --version
```

### 2. Copy this folder to the laptop

Put the whole `tactile-glove-demo-windows` folder anywhere (e.g. `C:\tactile-glove-demo-windows`).
Avoid OneDrive-synced paths if you hit permission issues.

### 3. Run it

**Easiest — double-click `run_windows.bat`.**
On first run it automatically:
1. creates a local virtual environment (`.venv`),
2. installs `pyserial`,
3. starts the viewer.

**Or do it manually in PowerShell:**

```powershell
cd C:\tactile-glove-demo-windows
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe tactile_glove_viewer.py
```

### 4. Open the UI

Open a browser to:

```
http://127.0.0.1:8877
```

Use **Chrome, Edge, or Firefox**. With **no gloves connected**, the 3D hands show the
neutral pose (palm flat down on a table, fingers forward) and you can still upload/replay recordings.

### 5. Connect the gloves (optional, for live data)

- Plug the gloves into USB. They appear as COM ports.
- The viewer **auto-detects** gloves and hot-plugs them — no restart needed; just wait
  a few seconds and the badge flips to `ON`.
- The terminal window prints the serial ports it sees at startup, e.g.:

  ```
  [viewer] serial ports detected at startup:
            COM10  (USB-SERIAL CH340)
  ```

- If auto-detect misses them, force the ports:

```powershell
.\.venv\Scripts\python.exe tactile_glove_viewer.py --left COM10 --right COM9
```

  (Use Device Manager → "Ports (COM & LPT)" to find the numbers.)

### 6. Calibrate (recommended before a demo)

In the UI:
1. **Orientation**: rest the hand in the neutral pose — *palm flat down on a table, fingers
   pointing forward* — keep it still, then click **Zero Orientation**. Do this for each hand
   (select Left/Right). A flat tabletop gives the cleanest, most repeatable zero for the
   6-axis IMU. Note: heading (yaw) has no magnetometer reference, so it drifts slowly,
   especially after lifting the hand up/down — just re-zero when needed.
2. **Finger bends**: **Capture Open** (flat hand) then **Capture Closed** (full fist),
   then **Save to File**. Calibration is saved in `calibration/`.

### 7. Record & replay (no gloves required)

Use the **Demo Recording** panel in the UI:

1. Click **● Record**, perform your gestures, then **■ Stop & Save**.
2. Pick a saved recording from the dropdown and play it back — the 3D hands, bend
   bars, and pressure heatmaps replay from the file.
3. Saved recordings land in `recordings/` as JSON. Use **Open recordings folder** to
   find them, or **Upload** to import a `.json` recording from another machine.
4. For a demo laptop with no USB gloves, start in replay-only mode:

```powershell
.\.venv\Scripts\python.exe tactile_glove_viewer.py --replay-only
```

---

## Command-line options

```
--port N           HTTP port (default 8877)
--left COMx        Force the left glove port
--right COMx       Force the right glove port
--single COMx      Force a single glove (side auto-detected)
--replay-only      Disable the live watcher; only upload/replay recordings
```

Example (replay only, on a different port):

```powershell
.\.venv\Scripts\python.exe tactile_glove_viewer.py --replay-only --port 9000
```

---

## Troubleshooting

- **"Python was not found"** — Python isn't on PATH. Re-run the installer and tick
  "Add python.exe to PATH", or reinstall.
- **3D hand stuck on "Loading hand model…"** — hard-refresh the browser
  (Ctrl+Shift+R). All assets are local. Use an up-to-date Chrome/Edge/Firefox.
- **Port 8877 already in use** — another copy is running, or pick a new port with
  `--port 9001`. To kill a stale instance:

  ```powershell
  Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
    Where-Object { $_.CommandLine -like '*tactile_glove_viewer*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
  ```

- **Gloves not detected** — check the startup port list printed in the terminal.
  - If the glove's COM port is **not listed**, install the **USB-serial driver**
    (most gloves use a CH340 chip — install the "CH341SER" Windows driver), then re-plug.
  - If it **is listed** but not picked up, close any app holding the port (serial monitor,
    companion app, etc.), or force `--left/--right`.
  - You can also probe ports directly:
    ```powershell
    .\.venv\Scripts\python.exe probe_glove_ports.py
    ```

---

## Requirements summary

- Windows 10/11
- Python 3.9+
- One pip package: `pyserial`
- A modern browser (Chrome, Edge, Firefox) with WebGL enabled
- The gloves + USB cables (only for live data; replay works without them)
