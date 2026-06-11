@echo off
REM ============================================================
REM  Tactile Glove Demo - Windows launcher
REM  Creates a local virtual environment on first run, installs
REM  dependencies, then starts the viewer at http://127.0.0.1:8877
REM ============================================================
setlocal
cd /d "%~dp0"

REM --- Locate Python ---
where python >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python was not found on PATH.
  echo Install Python 3.9+ from https://www.python.org/downloads/windows/
  echo and make sure you check "Add python.exe to PATH" during install.
  pause
  exit /b 1
)

REM --- Create venv on first run ---
if not exist ".venv\Scripts\python.exe" (
  echo [setup] Creating virtual environment .venv ...
  python -m venv .venv
  if errorlevel 1 (
    echo [ERROR] Failed to create virtual environment.
    pause
    exit /b 1
  )
  echo [setup] Installing dependencies ...
  ".venv\Scripts\python.exe" -m pip install --upgrade pip
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt
  if errorlevel 1 (
    echo [ERROR] Failed to install dependencies.
    pause
    exit /b 1
  )
)

echo.
echo [run] Starting Tactile Glove viewer ...
echo [run] Open http://127.0.0.1:8877 in your browser.
echo [run] Press Ctrl+C in this window to stop.
echo.
".venv\Scripts\python.exe" tactile_glove_viewer_win.py %*

pause
