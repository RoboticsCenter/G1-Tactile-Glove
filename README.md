# G1 Tactile Glove Demo Viewer

A self-contained, **offline** local viewer for the Juqiao G1 tactile gloves. It serves a
browser UI at `http://127.0.0.1:8877` showing:

- Per-finger bend bars
- 16×16 pressure heatmap
- A **3D hand** driven by the glove IMU + finger sensors
- Orientation ("zero") and finger-bend calibration
- Demo recording + replay (works with **no gloves connected**)

Everything (3D engine, hand model) is bundled locally — **no internet required** to run.

---

## Pick your OS

This repo ships a separate, self-contained copy for each operating system. Use the folder
that matches your machine — each one is complete on its own:

```
G1-Tactile-Glove/
  windows/    # Windows 10/11
  macos/      # macOS (Intel & Apple Silicon)
  linux/      # Linux (Ubuntu/Debian etc.)
```

Each folder has its own detailed `README.md` if you need more help.

---

## Quick start

### Step 1 — Get the files

Clone the repo (or download it as a ZIP from the green **Code** button and unzip it):

```bash
git clone https://github.com/RoboticsCenter/G1-Tactile-Glove.git
```

> Don't have Git? Download it from <https://git-scm.com/downloads>, install it, then
> reopen your terminal and run the command above.

### Step 2 — Run the launcher for your OS

The first run sets up everything automatically (~30 seconds). After that it starts instantly.

**Windows** — open the `windows` folder and **double-click `run_windows.bat`**
(or from PowerShell):

```powershell
cd G1-Tactile-Glove\windows
.\run_windows.bat
```

**macOS** — in Terminal:

```bash
cd G1-Tactile-Glove/macos
bash run_macos.sh
```

**Linux** — in a terminal:

```bash
cd G1-Tactile-Glove/linux
bash run_linux.sh
```

### Step 3 — Open the UI

Open your browser and go to:

```
http://127.0.0.1:8877
```

No gloves required — you can upload and replay recordings immediately.

---

## Prerequisites

You only need **Python 3.9+**. The launcher installs the one dependency (`pyserial`) into a
local virtual environment for you.

- **Windows** — install from <https://www.python.org/downloads/windows/> and check
  **"Add python.exe to PATH"** during setup. Verify with `python --version`.
- **macOS** — install from <https://www.python.org/downloads/macos/> or via Homebrew
  (`brew install python`). Verify with `python3 --version`.
- **Linux** — install with your package manager, e.g.
  `sudo apt install python3 python3-venv`. Verify with `python3 --version`.

### Connecting the gloves (optional, for live data)

The gloves use a **CH340 / WCH** USB-serial chip and are **auto-detected** at startup —
the terminal prints the ports it sees. If they don't show up, install the WCH **CH34x VCP**
driver, then re-run the launcher.

- **Windows** — gloves appear as `COM` ports (see Device Manager → "Ports (COM & LPT)").
- **macOS** — gloves appear as `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial-*`.
- **Linux** — gloves appear as `/dev/ttyUSB*` (you may need to be in the `dialout` group:
  `sudo usermod -aG dialout $USER`, then log out and back in).

See your OS folder's `README.md` for command-line options, calibration, and full
troubleshooting.
