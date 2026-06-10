#!/usr/bin/env python3
"""Bluetooth receiver dongle helpers for Juqiao tactile gloves.

Per JQ spec v1.2 section 6.3 the PC-side BT receiver exposes a USB serial port.
After RF pairing (solid blue on glove, red+blue on dongle) the host must still
bridge the dongle to Juqiao frames (AA 55 03 99) using AT commands:

  AT+SCAN=1          -> list JQ-LH / JQ-RH devices
  AT+CONN=<addr>   -> start streaming sensor data @ 921600

USB gloves stream Juqiao frames directly. A BT dongle often floods the port with
a proprietary 0x81 0x10 framed binary stream until AT+CONN succeeds.
"""

from __future__ import annotations

import re
import time
from typing import List, Optional, Tuple

import serial

from linker_glove_agent import HEADER, FrameParser, GLOVE_SENSOR_TYPE

BAUD = 921600
AT_LINE_RE = re.compile(rb"(?:^|\r|\n)(OK|ERROR[^\r\n]*|\+[^\r\n]+|AT[^\r\n]*)", re.MULTILINE)
JQ_LINE_RE = re.compile(r"JQ-(LH|RH|LF|RF|WB|FB)[^\r\n]*?([0-9A-Fa-f]{10,12})", re.IGNORECASE)
ADDR_RE = re.compile(r"\b([0-9A-Fa-f]{10,12})\b")


def looks_like_bt_binary_flood(chunk: bytes) -> bool:
    """Heuristic: dongle passthrough mode (not Juqiao AA550399)."""
    if not chunk or HEADER in chunk:
        return False
    if chunk.count(b"\x81\x10") >= 2:
        return True
    # Many 0x80 padding bytes with high throughput but no Juqiao header.
    if len(chunk) >= 512 and chunk.count(0x80) > len(chunk) * 0.55:
        return True
    return False


def _read_for(ser: serial.Serial, sec: float) -> bytes:
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < sec:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def _extract_text_lines(raw: bytes) -> List[str]:
    text = raw.decode("utf-8", errors="ignore")
    return [ln.strip() for ln in re.split(r"[\r\n]+", text) if ln.strip()]


def _find_jq_addresses(lines: List[str]) -> List[Tuple[str, str]]:
    hits: List[Tuple[str, str]] = []
    for ln in lines:
        m = JQ_LINE_RE.search(ln)
        if m:
            side = m.group(1).lower()
            if side in ("lh", "rh"):
                hits.append(("left" if side == "lh" else "right", m.group(2).upper()))
            continue
        if "JQ-" in ln.upper():
            addr_m = ADDR_RE.search(ln)
            if addr_m:
                side = "left" if "LH" in ln.upper() else "right" if "RH" in ln.upper() else "auto"
                hits.append((side, addr_m.group(1).upper()))
    return hits


def _count_juqiao_frames(raw: bytes) -> Tuple[int, int, int, str]:
    parser = FrameParser()
    left = right = 0
    for frame in parser.feed(raw):
        st = int(frame["sensor_type"])
        if st == GLOVE_SENSOR_TYPE["left"]:
            left += 1
        elif st == GLOVE_SENSOR_TYPE["right"]:
            right += 1
    total = left + right
    side = "none"
    if left and not right:
        side = "left"
    elif right and not left:
        side = "right"
    elif left and right:
        side = "both"
    return total, left, right, side


def try_at_bridge(
    port: str,
    listen_sec: float = 3.0,
    prefer_side: str = "auto",
) -> dict:
    """Attempt AT+SCAN / AT+CONN on a BT dongle serial port."""
    err = ""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=True, rtscts=False)
    except Exception as exc:
        return {"port": port, "ok": False, "opened": False, "mode": "bt_at", "error": str(exc)}

    try:
        ser.reset_input_buffer()
        # Give the dongle a moment after open; command mode is easiest right after replug.
        time.sleep(0.15)
        ser.write(b"AT\r\n")
        time.sleep(0.2)
        ser.write(b"AT+SCAN=1\r\n")
        scan_raw = _read_for(ser, listen_sec)
        lines = _extract_text_lines(scan_raw)
        addrs = _find_jq_addresses(lines)

        chosen: Optional[Tuple[str, str]] = None
        if prefer_side in ("left", "right"):
            for side, addr in addrs:
                if side == prefer_side:
                    chosen = (side, addr)
                    break
        if chosen is None and addrs:
            chosen = addrs[0]

        if chosen:
            side, addr = chosen
            ser.reset_input_buffer()
            ser.write(f"AT+CONN={addr}\r\n".encode("ascii"))
            conn_raw = _read_for(ser, listen_sec)
            raw = scan_raw + conn_raw
        else:
            raw = scan_raw
            if looks_like_bt_binary_flood(raw):
                err = (
                    "BT dongle is streaming proprietary binary (0x81 0x10 frames), not Juqiao AA550399. "
                    "Replug the dongle (red LED flashing), close the vendor Companion app, then retry within "
                    "a few seconds so AT+SCAN=1 can run before RF auto-connects."
                )
            elif lines:
                err = f"AT replied but no JQ-LH/JQ-RH address found. Lines: {lines[:6]}"
            else:
                err = "No AT text response and no Juqiao frames."

        total, left, right, side_out = _count_juqiao_frames(raw)
        if side_out == "none" and chosen and chosen[0] in ("left", "right"):
            side_out = chosen[0]

        return {
            "port": port,
            "ok": total > 0,
            "opened": True,
            "frames": total,
            "left": left,
            "right": right,
            "side": side_out,
            "mode": "bt_at",
            "addresses": addrs,
            "at_lines": lines[:20],
            "binary_flood": looks_like_bt_binary_flood(raw),
            "error": err,
        }
    except Exception as exc:
        return {"port": port, "ok": False, "opened": True, "mode": "bt_at", "error": str(exc)}
    finally:
        ser.close()


def diagnose_port(port: str, sample_sec: float = 2.0) -> dict:
    """Human-readable snapshot of what the OS exposes on this serial device."""
    info: dict = {"port": port}
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except Exception as exc:
        info.update({"opened": False, "error": str(exc)})
        return info
    try:
        ser.reset_input_buffer()
        raw = _read_for(ser, sample_sec)
        total, left, right, _ = _count_juqiao_frames(raw)
        info.update(
            {
                "opened": True,
                "bytes": len(raw),
                "juqiao_frames": total,
                "left": left,
                "right": right,
                "has_aa550399": HEADER in raw,
                "binary_flood": looks_like_bt_binary_flood(raw),
                "likely_bt_dongle": looks_like_bt_binary_flood(raw) and total == 0,
            }
        )
    finally:
        ser.close()
    return info
