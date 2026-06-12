#!/usr/bin/env bash
# ============================================================
#  Tactile Glove Demo - Linux launcher
#  Creates a local virtual environment on first run, installs
#  dependencies, then starts the viewer at http://127.0.0.1:8877
#
#  Run with:   bash run_linux.sh
#  (or make it executable once:  chmod +x run_linux.sh  then  ./run_linux.sh)
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

# --- Locate python3 ---
if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 was not found."
  echo "Install it with your package manager, e.g.:"
  echo "  Debian/Ubuntu:  sudo apt install python3 python3-venv python3-pip"
  echo "  Fedora:         sudo dnf install python3 python3-pip"
  echo "  Arch:           sudo pacman -S python python-pip"
  exit 1
fi

# --- Ensure the venv module is available (Debian/Ubuntu split it into python3-venv) ---
if ! python3 -c "import venv" >/dev/null 2>&1; then
  echo "[ERROR] Python's 'venv' module is missing."
  echo "On Debian/Ubuntu install it with:  sudo apt install python3-venv"
  exit 1
fi

# --- Create venv on first run ---
if [ ! -x ".venv/bin/python" ]; then
  echo "[setup] Creating virtual environment .venv ..."
  python3 -m venv .venv
  echo "[setup] Installing dependencies ..."
  ./.venv/bin/python -m pip install --upgrade pip
  ./.venv/bin/python -m pip install -r requirements.txt
fi

# --- Friendly heads-up about serial port permissions (live gloves only) ---
if ! id -nG "$USER" 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
  echo "[note] Your user is not in the 'dialout' group. Replay works regardless, but for"
  echo "[note] LIVE gloves you may need:  sudo usermod -aG dialout \$USER   (then log out/in)."
fi

echo
echo "[run] Starting Tactile Glove viewer ..."
echo "[run] Open http://127.0.0.1:8877 in your browser."
echo "[run] Press Ctrl+C to stop."
echo
exec ./.venv/bin/python tactile_glove_viewer.py "$@"
