# Tactile Glove Demo Viewer — Linux

A self-contained, **offline** local viewer for tactile gloves. It serves a
browser UI at `http://127.0.0.1:8877` showing:

- Per-finger bend bars
- 16×16 pressure heatmap
- A **3D hand** (anatomical model) driven by the glove IMU + finger sensors
- Orientation ("zero") calibration and finger-bend calibration
- Demo recording + replay (works with **no gloves connected**)
- Recording export to **per-hand CSV** (saved alongside each recording)

Everything (3D engine, hand model) is bundled locally — **no internet required** to run.

> This is the Linux port of the Windows demo. The application logic is the same;
> the differences are the launcher (`run_linux.sh`), Linux serial-port handling
> (`/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/rfcomm*`), and Bluetooth dongle detection.

---

## Quick start

```bash
cd ~/tactile-glove-demo-linux
bash run_linux.sh
```

Then open **http://127.0.0.1:8877** in your browser. No gloves required — you can
upload and replay recordings immediately.

---

## What's in this package

```
tactile-glove-demo-linux/
  tactile_glove_viewer.py     # main app (HTTP server + browser UI)
  linker_glove_agent.py       # serial protocol parser + sensor mapping
  probe_glove_ports.py        # port auto-detection / standalone probe tool
  bt_dongle.py                # Bluetooth receiver dongle AT+SCAN/AT+CONN bridge
  time_utils.py               # timestamp helper
  assets/                     # bundled 3D engine + hand model (served locally)
    three.module.js
    GLTFLoader.js
    hand_model.glb
    utils/SkeletonUtils.js
    utils/BufferGeometryUtils.js
  calibration/                # your saved calibration files land here
  recordings/                 # your saved recordings (JSON + CSV) land here
  requirements.txt            # one dependency: pyserial
  run_linux.sh                # one-click launcher
  README.md
```

---

## Step-by-step setup (first use)

### 1. Install Python 3.9+ (with venv and pip)

- **Debian/Ubuntu:** `sudo apt install python3 python3-venv python3-pip`
- **Fedora:** `sudo dnf install python3 python3-pip`
- **Arch:** `sudo pacman -S python python-pip`

Verify:

```bash
python3 --version
```

### 2. Copy this folder to the machine

Put the whole `tactile-glove-demo-linux` folder anywhere (e.g. `~/tactile-glove-demo-linux`).

### 3. Run it

```bash
cd ~/tactile-glove-demo-linux
bash run_linux.sh
```

On first run this creates a local virtual environment (`.venv`), installs `pyserial`,
and starts the viewer. (You can also `chmod +x run_linux.sh` once and then run
`./run_linux.sh`.)

**Manual alternative:**

```bash
cd ~/tactile-glove-demo-linux
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python tactile_glove_viewer.py
```

### 4. Open the UI

```
http://127.0.0.1:8877
```

Use a modern Chrome/Chromium, Firefox, or any WebGL-capable browser. With **no gloves
connected**, the 3D hands show the neutral pose (palm flat down on a table, fingers
forward) and you can still upload/replay recordings.

### 5. Connect the gloves (optional, for live data)

- On Linux the gloves appear as `/dev/ttyUSB*` (CH340 USB-serial chip) or `/dev/ttyACM*`
  (USB-CDC). A Bluetooth receiver dongle shows as `/dev/ttyACM*` or `/dev/rfcomm*`.
- The viewer **auto-detects** ports and hot-plugs them — no restart needed. The terminal
  prints the ports it sees at startup, e.g.:

  ```
  [viewer] serial ports detected at startup:
            /dev/ttyUSB0  (USB Serial)
  ```

- List ports yourself any time:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

- Force ports if auto-detect misses them:

```bash
./.venv/bin/python tactile_glove_viewer.py --left /dev/ttyUSB0 --right /dev/ttyUSB1
```

- Probe ports directly to see which one streams glove data (also runs the BT bridge):

```bash
./.venv/bin/python probe_glove_ports.py
```

### 6. Calibrate (recommended before a demo)

In the UI:
1. **Orientation**: rest the hand in the neutral pose — *palm flat down on a table, fingers
   pointing forward* — keep it still, then click **Zero Orientation** (per hand). Heading
   (yaw) has no magnetometer reference, so it drifts slowly — re-zero when needed.
2. **Finger bends**: **Capture Open** (flat hand) then **Capture Closed** (full fist),
   then **Save to File**. Calibration is saved in `calibration/`.

### 7. Record, replay & CSV export (no gloves required)

Use the **Demo Recording** panel in the UI:

1. Click **● Record**, perform your gestures, then **■ Stop & Save**.
2. Pick a saved recording from the dropdown and play it back.
3. Each recording is saved as its own folder under `recordings/<id>/` containing the
   `<id>.json` **and** a per-hand CSV (`left_<timestamp>.csv` / `right_<timestamp>.csv`)
   with frame time, per-finger bends, peak/region/total pressure, the raw 256-point grid,
   and the IMU quaternion. Use **Open recordings folder** (opens via `xdg-open`) to find
   them, or **Upload** to import a `.json` recording from another machine.
4. For a demo box with no USB gloves, start in replay-only mode:

```bash
./.venv/bin/python tactile_glove_viewer.py --replay-only
```

---

## Command-line options

```
--port N             HTTP port (default 8877)
--left /dev/ttyXXX   Force the left glove port
--right /dev/ttyXXX  Force the right glove port
--single /dev/ttyXXX Force a single glove (side auto-detected)
--replay-only        Disable the live watcher; only upload/replay recordings
```

---

## Linux-specific notes & troubleshooting

### Serial port permissions ("could not open (busy, or permission denied)")

Accessing `/dev/ttyUSB*` / `/dev/ttyACM*` requires membership in the `dialout` group
(on some distros `uucp`):

```bash
sudo usermod -aG dialout "$USER"
# then fully log out and back in (or reboot) for it to take effect
groups   # verify 'dialout' is listed
```

Also close any other app holding the port (vendor Companion app, `screen`, `minicom`,
serial monitors).

### USB-serial driver

Most distros load the CH340 (`ch341`), CP210x (`cp210x`), and FTDI (`ftdi_sio`) drivers
out of the box. Confirm the device enumerated:

```bash
dmesg | tail -n 20          # look for ttyUSB/ttyACM attach lines
ls /dev/ttyUSB* /dev/ttyACM*
```

### Bluetooth dongle

The glove's BT path is **USB receiver dongle → serial port** (not native BlueZ/BLE):

**Glove BT MCU → RF → USB Bluetooth receiver dongle → `/dev/ttyACM*` or `/dev/ttyUSB*`**

A solid blue glove LED only means RF is paired. The viewer needs glove frames
(`AA 55 03 99`) on that serial port. Until the dongle is bridged with `AT+SCAN=1` /
`AT+CONN=<address>`, it streams a proprietary binary format. This build does the bridge
**automatically**: when the watcher (or `probe_glove_ports.py`) sees a binary-flood
dongle, it issues the AT commands and connects to the first `JQ-LH` / `JQ-RH` device.

Manual diagnostics:

```bash
./.venv/bin/python -c "from bt_dongle import diagnose_port; print(diagnose_port('/dev/ttyACM0'))"
```

If auto-bridge fails: replug the dongle so its red LED is **flashing** (not yet
RF-connected), close the vendor app, then re-run `probe_glove_ports.py` within a few
seconds so `AT+SCAN=1` runs before RF auto-connects.

### 3D hand stuck on "Loading hand model…"

- Hard-refresh the browser (Ctrl+Shift+R). All assets are local, nothing to download.
- Use an up-to-date Chrome/Chromium or Firefox with WebGL enabled. On headless/VM setups
  ensure GPU/software WebGL is available.

### Port 8877 already in use

Pick another port: `./.venv/bin/python tactile_glove_viewer.py --port 9001`, or stop the
old instance:

```bash
pkill -f tactile_glove_viewer.py
```

---

## Requirements summary

- Linux (x86_64 or arm64) with a glibc Python 3.9+
- `python3`, `python3-venv`, `python3-pip`
- One pip package: `pyserial`
- Membership in the `dialout` group for live serial access
- A modern browser (Chrome/Chromium, Firefox) with WebGL enabled
- The gloves + USB cables or BT receiver dongle (only for live data; replay works without them)
```
