#!/usr/bin/env python3
"""Probe COM/serial ports for Tactile Glove (AA 55 03 99) data."""

from __future__ import annotations

import sys
import time

import serial
from serial.tools import list_ports

from linker_glove_agent_win import FrameParser, GLOVE_SENSOR_TYPE

BAUD = 921600
PROBE_SEC = 2.0


def probe_port(port: str, probe_sec: float = PROBE_SEC) -> dict:
    parser = FrameParser()
    left = right = 0
    err = ""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except Exception as exc:
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
    try:
        while time.time() - t0 < probe_sec:
            chunk = ser.read(4096)
            if not chunk:
                continue
            for frame in parser.feed(chunk):
                st = int(frame["sensor_type"])
                if st == GLOVE_SENSOR_TYPE["left"]:
                    left += 1
                elif st == GLOVE_SENSOR_TYPE["right"]:
                    right += 1
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
    return {
        "port": port,
        "ok": total > 0,
        "opened": True,
        "frames": total,
        "left": left,
        "right": right,
        "side": side,
        "error": err,
    }


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
        if r.get("error"):
            print(f"           error: {r['error']}")
        if r["ok"]:
            hits.append(r)
    print()
    if not hits:
        print("No glove data found. Check USB cable and drivers (CH340/QinHeng).")
        sys.exit(2)
    print("Detected glove port(s):")
    for h in hits:
        print(f"  {h['port']} -> {h['side']} ({h['frames']} frames)")


if __name__ == "__main__":
    main()
