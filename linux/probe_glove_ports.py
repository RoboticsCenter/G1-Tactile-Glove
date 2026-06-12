#!/usr/bin/env python3
"""Probe serial ports for Tactile Glove (AA 55 03 99) data.

On Linux gloves enumerate as /dev/ttyUSB* (CH340) or /dev/ttyACM* (USB-CDC),
and BT receiver dongles as /dev/ttyACM* or /dev/rfcomm*. When a dongle is found
streaming proprietary binary instead of glove frames, this probe attempts the
AT+SCAN / AT+CONN bridge automatically.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bt_dongle import looks_like_bt_binary_flood, try_at_bridge  # noqa: E402
from linker_glove_agent import FrameParser, GLOVE_SENSOR_TYPE  # noqa: E402

BAUD = 921600
PROBE_SEC = 2.0


def probe_port(port: str, probe_sec: float = PROBE_SEC, early_frames: int = 3) -> dict:
    parser = FrameParser()
    left = right = 0
    err = ""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except Exception as exc:
        # opened=False distinguishes "busy/held by another app" from "opened, no glove data".
        return {
            "port": port,
            "ok": False,
            "opened": False,
            "frames": 0,
            "left": 0,
            "right": 0,
            "side": "none",
            "error": str(exc),
        }
    t0 = time.time()
    sample = bytearray()
    try:
        while time.time() - t0 < probe_sec:
            chunk = ser.read(4096)
            if not chunk:
                continue
            sample.extend(chunk)
            for frame in parser.feed(chunk):
                st = int(frame["sensor_type"])
                if st == GLOVE_SENSOR_TYPE["left"]:
                    left += 1
                elif st == GLOVE_SENSOR_TYPE["right"]:
                    right += 1
            # Early-exit: a streaming glove shows up within a few frames, so stop as
            # soon as we're confident instead of listening the whole window. This is
            # the main hot-plug latency win (~0.1s vs the full probe_sec timeout).
            if left + right >= early_frames:
                break
    except Exception as exc:
        err = str(exc)
    finally:
        ser.close()
    total = left + right
    side = "none"
    if left and not right:
        side = "left"
    elif right and not left:
        side = "right"
    elif left and right:
        side = "both"
    out = {
        "port": port,
        "ok": total > 0,
        "opened": True,
        "frames": total,
        "left": left,
        "right": right,
        "side": side,
        "error": err,
    }
    # BT receiver dongle: RF may show "connected" on LEDs while the USB serial port
    # still streams a proprietary binary format until AT+SCAN / AT+CONN bridges it.
    if total == 0 and not err and looks_like_bt_binary_flood(bytes(sample)):
        bridged = try_at_bridge(port, listen_sec=max(2.0, probe_sec))
        if bridged.get("ok"):
            return bridged
        out["mode"] = "bt_dongle"
        out["binary_flood"] = True
        out["error"] = bridged.get("error") or (
            "BT dongle detected but not streaming AA550399 yet."
        )
    return out


def main() -> None:
    ports = [p.device for p in list_ports.comports()]
    if not ports:
        print("No serial ports found. Is the glove USB connected?")
        sys.exit(1)
    print(f"Probing {len(ports)} port(s) @ {BAUD} for ~{PROBE_SEC}s each...\n")
    hits = []
    for port in sorted(ports):
        info = next((p for p in list_ports.comports() if p.device == port), None)
        desc = info.description if info else ""
        r = probe_port(port)
        status = "GLOVE" if r["ok"] else "no data"
        print(f"  {port:8}  {status:8}  side={r['side']:5}  frames={r['frames']:4}  {desc}")
        if r.get("mode") == "bt_dongle":
            print("           note: looks like a BT receiver dongle (binary flood, no AA550399 yet)")
        if r.get("error"):
            print(f"           error: {r['error']}")
        if r["ok"]:
            hits.append(r)
    print()
    if not hits:
        print("No glove data found. Check the USB cable / driver, and that your user is in the "
              "'dialout' group (sudo usermod -aG dialout $USER, then log out/in).")
        sys.exit(2)
    print("Detected glove port(s):")
    for h in hits:
        print(f"  {h['port']} -> {h['side']} ({h['frames']} frames)")


if __name__ == "__main__":
    main()
