#!/usr/bin/env bash
# ============================================================
#  Tactile Glove Demo - macOS launcher
#  Creates a local virtual environment on first run, installs
#  dependencies, then starts the viewer at http://127.0.0.1:8877
#
#  Run with:   bash run_macos.sh
#  (or make it executable once:  chmod +x run_macos.sh  then  ./run_macos.sh)
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

# --- Locate python3 ---
if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 was not found."
  echo "Install it from https://www.python.org/downloads/macos/"
  echo "or, if you use Homebrew:  brew install python"
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

echo
echo "[run] Starting Tactile Glove viewer ..."
echo "[run] Open http://127.0.0.1:8877 in your browser."
echo "[run] Press Ctrl+C to stop."
echo
exec ./.venv/bin/python tactile_glove_viewer.py "$@"
