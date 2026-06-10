# Tactile Glove Demo Viewer

A self-contained, **offline** local viewer for the Juqiao tactile gloves. It serves a
browser UI at `http://127.0.0.1:8877` showing:

- Per-finger bend bars
- 16×16 pressure heatmap
- A **3D hand** (anatomical model) driven by the glove IMU + finger sensors
- Orientation ("zero") calibration and finger-bend calibration
- Demo recording + replay (works with **no gloves connected**)

Everything (3D engine, hand model) is bundled locally — **no internet required** to run.

---

## What's in this package

```
G1-Tactile-Glove/
  tactile_glove_viewer.py     # main app (HTTP server + browser UI)
  linker_glove_agent.py       # serial protocol parser + sensor mapping
  probe_glove_ports.py        # port auto-detection / standalone probe tool
  bt_dongle.py                # Bluetooth dongle bridge helpers (macOS)
  connect_bt_dongle.py        # Bluetooth dongle connection utility (macOS)
  time_utils.py               # timestamp helper
  assets/                     # bundled 3D engine + hand model (served locally)
    three.module.js
    GLTFLoader.js
    shroom_hand1.glb           # hand model (macOS)
    hand_model.glb             # hand model (Windows)
    utils/SkeletonUtils.js
    utils/BufferGeometryUtils.js
  calibration/                # your saved calibration files land here
  recordings/                 # your saved recordings land here
  requirements.txt            # one dependency: pyserial
  run_macos.sh                # one-click launcher (macOS)
  run_windows.bat             # one-click launcher (Windows)
  README.md
```

---

## Quick start

### Step 1 — Get the files

**macOS (Terminal):**

```bash
cd ~
git clone https://github.com/RoboticsCenter/G1-Tactile-Glove.git
```

**Windows (PowerShell or Command Prompt):**

```powershell
cd C:\
git clone https://github.com/RoboticsCenter/G1-Tactile-Glove.git
```

> Don't have Git? Download it from <https://git-scm.com/downloads>, install it, then
> reopen your terminal and run the command above.

### Step 2 — Run it

**macOS:**

```bash
cd ~/G1-Tactile-Glove
bash run_macos.sh
```

**Windows** — double-click `run_windows.bat` inside the `G1-Tactile-Glove` folder, or
from PowerShell:

```powershell
cd C:\G1-Tactile-Glove
.\run_windows.bat
```

The first run automatically sets up everything (takes ~30 seconds). After that it
starts instantly.

### Step 3 — Open the UI

Open your browser and go to:

```
http://127.0.0.1:8877
```

No gloves required — you can upload and replay recordings immediately.

---

## Prerequisites

### Install Python 3.9+

**macOS** — ships with an old Python; install a current one:
- From <https://www.python.org/downloads/macos/>, **or**
- With Homebrew: `brew install python`

```bash
python3 --version
```

**Windows:**
- Download from <https://www.python.org/downloads/windows/>
- During install, **check "Add python.exe to PATH"**

```powershell
python --version
```

### Connect the gloves (optional, for live data)

**macOS** — gloves appear as `/dev/cu.wchusbserial-XXXX` (CH340 chip) or
`/dev/cu.usbserial-XXXX`. A Bluetooth dongle shows as `/dev/cu.usbmodem-XXXX`.

```bash
# List ports
ls /dev/cu.*

# Force specific ports
./.venv/bin/python tactile_glove_viewer.py --left /dev/cu.wchusbserial14120 --right /dev/cu.wchusbserial14220
```

**Windows** — gloves appear as COM ports. Use Device Manager → "Ports (COM & LPT)" to
find the numbers.

```powershell
# Force specific ports
.\.venv\Scripts\python.exe tactile_glove_viewer.py --left COM10 --right COM9
```

The viewer **auto-detects** ports and hot-plugs them on both platforms. The terminal
window prints the ports it sees at startup.

Probe ports directly to see which one streams glove data:

```bash
# macOS
./.venv/bin/python probe_glove_ports.py
# Windows
.\.venv\Scripts\python.exe probe_glove_ports.py
```

### Calibrate (recommended before a demo)

In the UI:
1. **Orientation**: hold the hand in the neutral pose — *palm toward front, fingers up* —
   then click **Zero Orientation**. Do this for each hand (select Left/Right).
2. **Finger bends**: **Capture Open** (flat hand) then **Capture Closed** (full fist),
   then **Save to File**. Calibration is saved in `calibration/`.

### Record & replay (no gloves required)

Use the **Demo Recording** panel in the UI:

1. Click **● Record**, perform your gestures, then **■ Stop & Save**.
2. Pick a saved recording from the dropdown and play it back.
3. Saved recordings land in `recordings/` as JSON. Use **Upload** to import a `.json`
   recording from another machine.
4. For a demo laptop with no USB gloves, start in replay-only mode:

```bash
# macOS
./.venv/bin/python tactile_glove_viewer.py --replay-only
# Windows
.\.venv\Scripts\python.exe tactile_glove_viewer.py --replay-only
```

---

## Command-line options

```
--port N             HTTP port (default 8877)
--left <port>        Force the left glove port  (macOS: /dev/cu.xxx  Windows: COMx)
--right <port>       Force the right glove port
--single <port>      Force a single glove (side auto-detected)
--replay-only        Disable the live watcher; only upload/replay recordings
```

---

## Troubleshooting

### USB-serial driver (most common cause of "no ports")

The gloves use a **CH340 / WCH** USB-serial chip.

**macOS** — if nothing appears under `ls /dev/cu.*`:
1. Install the WCH **CH34x VCP** driver (from WCH's site).
2. On Apple Silicon / recent macOS you may need to allow the driver in
   **System Settings → Privacy & Security**, then reboot.
3. Re-plug the glove and re-run `ls /dev/cu.*`.

**Windows** — if the COM port is not listed in Device Manager:
1. Install the **CH341SER** Windows driver, then re-plug.

### Permission to access the serial port

If the viewer prints `could not open (busy, or permission denied)`:
- Close any other app holding the port — especially the **vendor Companion App**.
- Make sure no serial monitor is connected to the same port.

### 3D hand stuck on "Loading hand model…"

- Hard-refresh the browser (**Cmd+Shift+R** on macOS, **Ctrl+Shift+R** on Windows).
  All assets are local, nothing to download.
- Check that `assets/` is a real folder (not filenames like `assets\three.module.js`).
  The viewer auto-repairs backslash paths on startup.

### Bluetooth dongle not hot-plugging (macOS)

Per the JQ spec (§2.2, §6.3), Bluetooth is **not** native BLE. The path is:

**Glove BT MCU → RF → USB Bluetooth receiver dongle → `/dev/cu.usbserial-*` or `/dev/cu.usbmodem-*`**

Steps:
1. Confirm the dongle appears: `ls /dev/cu.*`
2. **Close the vendor Companion App** and stop any running viewer instance.
3. **Replug the BT receiver dongle** so its red LED is **flashing** (not yet RF-connected).
4. Run: `./.venv/bin/python probe_glove_ports.py` — it will attempt `AT+SCAN=1` /
   `AT+CONN=…` automatically.
5. Or pair manually (921600 baud): send `AT+SCAN=1`, then `AT+CONN=<JQ-LH or JQ-RH address>`.

Diagnostic:
```bash
./.venv/bin/python -c "from bt_dongle import diagnose_port; print(diagnose_port('/dev/cu.usbserial-XXX'))"
```

### Port 8877 already in use

Pick another port with `--port 9001`, or kill the stale instance:

```bash
# macOS
pkill -f tactile_glove_viewer.py
```

```powershell
# Windows
Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
  Where-Object { $_.CommandLine -like '*tactile_glove_viewer*' } |
  ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
```

### Gatekeeper / quarantine (macOS)

If macOS flags the downloaded folder, the scripts are plain text and safe. Running
`bash run_macos.sh` from Terminal avoids the app-quarantine prompt. If needed:

```bash
xattr -dr com.apple.quarantine ~/G1-Tactile-Glove
```

### "Python was not found" (Windows)

Python isn't on PATH. Re-run the installer and tick "Add python.exe to PATH".

---

## Requirements summary

| | macOS | Windows |
|---|---|---|
| OS | macOS 10.15+ (Intel or Apple Silicon) | Windows 10/11 |
| Python | 3.9+ | 3.9+ |
| pip packages | `pyserial` | `pyserial` |
| Driver | CH340/WCH VCP driver | CH341SER |
| Browser | Safari, Chrome, Firefox (WebGL) | Chrome, Edge, Firefox (WebGL) |
| Gloves | Optional (live data only; replay works without) | Optional |
