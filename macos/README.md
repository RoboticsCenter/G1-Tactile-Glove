# Tactile Glove Demo Viewer — macOS

A self-contained, **offline** local viewer for tactile gloves. It serves a
browser UI at `http://127.0.0.1:8877` showing:

- Per-finger bend bars
- 16×16 pressure heatmap
- A **3D hand** (anatomical model) driven by the glove IMU + finger sensors
- Orientation ("zero") calibration and finger-bend calibration
- Demo recording + replay (works with **no gloves connected**)

Everything (3D engine, hand model) is bundled locally — **no internet required** to run.
Works on Intel and Apple Silicon (M1/M2/M3) Macs.

> This is the macOS build of the Windows demo. The application logic is identical to
> the Windows package; only the launcher (`run_macos.sh` instead of `run_windows.bat`)
> and these instructions differ.

---

## Quick start

```bash
cd ~/tactile-glove-demo-mac
bash run_macos.sh
```

Then open **http://127.0.0.1:8877** in your browser. No gloves required — you can
upload and replay recordings immediately.

---

## What's in this package

```
tactile-glove-demo-mac/
  tactile_glove_viewer.py     # main app (HTTP server + browser UI)
  linker_glove_agent.py       # serial protocol parser + sensor mapping
  probe_glove_ports.py        # port auto-detection / standalone probe tool
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
  run_macos.sh                # one-click launcher
  README.md
```

---

## Step-by-step setup (first use)

### 1. Install Python 3.9+

macOS ships with an old Python; install a current one:

- From <https://www.python.org/downloads/macos/>, **or**
- With Homebrew: `brew install python`

Verify in Terminal:

```bash
python3 --version
```

### 2. Copy this folder to the Mac

Put the whole `tactile-glove-demo-mac` folder anywhere (e.g. `~/tactile-glove-demo-mac`).

### 3. Run it

In Terminal:

```bash
cd ~/tactile-glove-demo-mac
bash run_macos.sh
```

On first run this creates a local virtual environment (`.venv`), installs `pyserial`,
and starts the viewer. (You can also `chmod +x run_macos.sh` once and then run
`./run_macos.sh`.)

**Manual alternative:**

```bash
cd ~/tactile-glove-demo-mac
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python tactile_glove_viewer.py
```

### 4. Open the UI

```
http://127.0.0.1:8877
```

Use **Safari, Chrome, or Firefox**. With **no gloves connected**, the 3D hands show the
neutral pose (palm flat down on a table, fingers forward) and you can still upload/replay
recordings.

### 5. Connect the gloves (optional, for live data)

- On macOS the gloves appear as `/dev/cu.wchusbserial-XXXX` (CH340 chip) or
  `/dev/cu.usbserial-XXXX`. A Bluetooth dongle shows as `/dev/cu.usbmodem-XXXX`.
- The viewer **auto-detects** ports and hot-plugs them — no restart needed; just wait
  a few seconds and the badge flips to `ON`. The Terminal window prints the ports it
  sees at startup, e.g.:

  ```
  [viewer] serial ports detected at startup:
            /dev/cu.wchusbserial14120  (USB Serial)
  ```

- List ports yourself any time:

```bash
ls /dev/cu.*
```

- Force ports if auto-detect misses them (use the names from `ls /dev/cu.*`):

```bash
./.venv/bin/python tactile_glove_viewer.py --left /dev/cu.wchusbserial14120 --right /dev/cu.wchusbserial14220
```

- Probe ports directly to see which one streams glove data:

```bash
./.venv/bin/python probe_glove_ports.py
```

### 6. Calibrate (recommended before a demo)

In the UI:
1. **Orientation**: rest the hand in the neutral pose — *palm flat down on a table, fingers
   pointing forward* — keep it still, then click **Zero Orientation**. Do this for each hand
   (select Left/Right). A flat tabletop gives the cleanest, most repeatable zero for the
   6-axis IMU. Note: heading (yaw) has no magnetometer reference, so it drifts slowly —
   just re-zero when needed.
2. **Finger bends**: **Capture Open** (flat hand) then **Capture Closed** (full fist),
   then **Save to File**. Calibration is saved in `calibration/`.

### 7. Record & replay (no gloves required)

Use the **Demo Recording** panel in the UI:

1. Click **● Record**, perform your gestures, then **■ Stop & Save**.
2. Pick a saved recording from the dropdown and play it back — the 3D hands, bend
   bars, and pressure heatmaps replay from the file.
3. Saved recordings land in `recordings/`. Use **Open recordings folder** to find them,
   or **Upload** to import a `.json` recording from another machine.
4. For a demo Mac with no USB gloves, start in replay-only mode:

```bash
./.venv/bin/python tactile_glove_viewer.py --replay-only
```

---

## Command-line options

```
--port N             HTTP port (default 8877)
--left /dev/cu.xxx   Force the left glove port
--right /dev/cu.xxx  Force the right glove port
--single /dev/cu.xxx Force a single glove (side auto-detected)
--replay-only        Disable the live watcher; only upload/replay recordings
```

---

## macOS-specific notes & troubleshooting

### USB-serial driver (most common cause of "no ports")

The gloves use a **CH340 / WCH** USB-serial chip (the port name contains `wchusbserial`).
If you plug in a glove and **nothing appears** under `ls /dev/cu.*` or in the viewer's
startup list, the driver isn't loaded:

1. Install the WCH **CH34x VCP** driver for macOS (from WCH's site), **or** the CP210x /
   FTDI VCP driver if your unit uses those chips.
2. On Apple Silicon / recent macOS you may need to allow the driver in
   **System Settings → Privacy & Security** ("System software was blocked"), then reboot.
3. Re-plug the glove and re-run `ls /dev/cu.*`.

### Permission to access the serial port

If the viewer prints `could not open (busy, or permission denied)`:
- Close any other app holding the port — especially the **vendor Companion App**.
- Make sure no serial monitor is connected to the same port.

### 3D hand stuck on "Loading hand model…"

- Hard-refresh the browser (Cmd+Shift+R). All assets are local, nothing to download.
- Use an up-to-date Safari, Chrome, or Firefox with WebGL enabled.

### Gatekeeper / quarantine

If macOS flags the downloaded folder ("unidentified developer"), the scripts are plain
text and safe. Running `bash run_macos.sh` from Terminal avoids the app-quarantine
prompt. If needed:

```bash
xattr -dr com.apple.quarantine ~/tactile-glove-demo-mac
```

### Port 8877 already in use

Pick another port: `./.venv/bin/python tactile_glove_viewer.py --port 9001`, or stop the
old instance:

```bash
pkill -f tactile_glove_viewer.py
```

---

## Requirements summary

- macOS 10.15+ (Intel or Apple Silicon)
- Python 3.9+
- One pip package: `pyserial`
- USB-serial driver for the glove chip (CH340/WCH for most units)
- A modern browser (Safari, Chrome, Firefox) with WebGL enabled
- The gloves + USB cables (only for live data; replay works without them)
