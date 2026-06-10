#!/usr/bin/env python3
"""Mac-friendly replacement for spec §6.3.3 (SSCOM.exe on Windows).

Usage (best right after replugging the BT receiver dongle):

  ./.venv/bin/python connect_bt_dongle.py
  ./.venv/bin/python connect_bt_dongle.py --port /dev/cu.usbserial-140
  ./.venv/bin/python connect_bt_dongle.py --side left
  ./.venv/bin/python connect_bt_dongle.py --address 3C8A1F2E9A36
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bt_dongle import BAUD, _extract_text_lines, _find_jq_addresses, _read_for, try_at_bridge
from linker_glove_agent import HEADER, FrameParser

DEFAULT_PROBE = 5.0
WATCH_PROBE = 15.0
AT_BAUDS = (921600, 115200, 460800, 230400)


def _diag_raw(raw: bytes) -> None:
    from bt_dongle import looks_like_bt_binary_flood

    print(f"  Bytes received: {len(raw)}")
    if not raw:
        print("  (dongle sent nothing — may still be booting, wrong baud, or AT not supported)")
        return
    print(f"  Binary flood: {looks_like_bt_binary_flood(raw)}")
    print(f"  Juqiao AA550399: {raw.count(HEADER)}")
    print(f"  First 48 bytes (hex): {raw[:48].hex()}")
    if _looks_like_at_text(raw):
        print("  Contains AT-like text: yes")
    else:
        print("  Contains AT-like text: no")


def _at_scan_on_port(port: str, baud: int, listen_sec: float, repeat: int = 1) -> bytes:
    import serial

    raw = bytearray()
    ser = serial.Serial(
        port, baud, timeout=0.05, dsrdtr=True, rtscts=False, exclusive=True,
    )
    try:
        # CH340 dongles often need a moment after USB enumerate before AT works.
        time.sleep(0.4)
        ser.reset_input_buffer()
        for _ in range(repeat):
            _send_at_sequence(ser, [b"AT\r\n", b"AT+SCAN=1\r\n"], pause=0.12)
            chunk = _read_for(ser, listen_sec / max(1, repeat))
            raw.extend(chunk)
            if _looks_like_at_text(chunk) or HEADER in chunk:
                break
    finally:
        ser.close()
    return bytes(raw)


def _scan_multi_baud(port: str, listen_sec: float, repeat: int = 2) -> tuple[bytes, int]:
    best_raw = b""
    best_baud = BAUD
    for baud in AT_BAUDS:
        print(f"  Trying {baud} baud …")
        raw = _at_scan_on_port(port, baud, listen_sec, repeat=repeat)
        _diag_raw(raw)
        if _find_jq_addresses(_extract_text_lines(raw)) or _looks_like_at_text(raw) or HEADER in raw:
            return raw, baud
        if len(raw) > len(best_raw):
            best_raw, best_baud = raw, baud
    return best_raw, best_baud


def pick_usb_port(explicit: str) -> str:
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    for p in ports:
        dev = p.device
        if "usbserial" in dev or "usbmodem" in dev or "wchusbserial" in dev:
            return dev
    raise SystemExit(
        "No USB serial port found. Plug in the BT receiver dongle and run: ls /dev/cu.*"
    )


def _looks_like_at_text(raw: bytes) -> bool:
    text = raw.decode("utf-8", errors="ignore")
    return any(k in text.upper() for k in ("OK", "JQ-LH", "JQ-RH", "JQ-LF", "ERROR", "AT+"))


def _send_at_sequence(ser, commands: list[bytes], pause: float = 0.15) -> None:
    for cmd in commands:
        ser.write(cmd)
        time.sleep(pause)


def scan_only(port: str, listen_sec: float, disc_first: bool = False) -> None:
    import serial

    from bt_dongle import looks_like_bt_binary_flood

    print(f"Opening {port} @ {BAUD} …")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=True, rtscts=False)
    except Exception as exc:
        raise SystemExit(f"Could not open {port}: {exc}\nClose Shroom / Companion app first.") from exc
    try:
        ser.reset_input_buffer()
        time.sleep(0.05)
        if disc_first:
            print("Trying to exit RF passthrough (AT+DISC / AT+DISCONN) …")
            _send_at_sequence(
                ser,
                [b"AT+DISC\r\n", b"AT+DISCONN\r\n", b"AT+DISCONNECT\r\n", b"AT\r\n"],
                pause=0.25,
            )
            disc_raw = _read_for(ser, 1.0)
            if _looks_like_at_text(disc_raw):
                print("  Dongle responded to disconnect command.")
            ser.reset_input_buffer()
        print("Sending AT+SCAN=1 (spec §6.3.3 step 5) …\n")
        _send_at_sequence(ser, [b"AT\r\n", b"AT+SCAN=1\r\n"], pause=0.1)
        raw = _read_for(ser, listen_sec)
        if disc_first:
            raw = disc_raw + raw
    finally:
        ser.close()

    lines = _extract_text_lines(raw)
    # Drop lines that are mostly binary garbage (mis-decoded sensor stream).
    clean = [
        ln for ln in lines
        if ("JQ-" in ln.upper() or ln.strip() in ("OK", "ERROR")
            or ln.upper().startswith(("AT", "OK", "+", "ERROR")))
        and sum(1 for c in ln if c.isprintable()) > len(ln) * 0.85
    ]
    addrs = _find_jq_addresses(clean or lines)
    binary = looks_like_bt_binary_flood(raw)
    print("--- AT / scan text (if any) ---")
    if clean:
        for ln in clean[:40]:
            print(" ", ln)
    elif _looks_like_at_text(raw):
        for ln in lines[:20]:
            print(" ", ln)
    else:
        print("  (no AT text — dongle is in binary passthrough mode)")
    print(f"\n  Port bytes received: {len(raw)}  binary_flood: {binary}")
    _diag_raw(raw)
    print()
    if addrs:
        print("Found glove(s):")
        for side, addr in addrs:
            print(f"  {side:5}  JQ address: {addr}")
        print("\nConnect with:")
        print(f"  ./.venv/bin/python connect_bt_dongle.py --port {port} --address {addrs[0][1]}")
    else:
        print("No JQ-LH / JQ-RH address found.")
        _print_recovery_steps(port, binary)


def _print_jqcy_conclusion(mac: str = "") -> None:
    print("\n" + "=" * 50)
    print("CONCLUSION — Bluetooth + this demo viewer")
    print("=" * 50)
    print("  Mac detects your dongle and receives RF data at full rate.")
    print("  Dongle model JQCY-YL-135 outputs proprietary 0x81 0x20 binary")
    print("  on USB serial — NOT Juqiao AA550399 frames.")
    print()
    print("  AT+CONN with your MAC was sent; RF still does not produce Juqiao.")
    print("  Shroom decodes this dongle internally. This demo does not (yet).")
    print()
    print("  What works today:")
    print("    • USB Type-C cable → bash run_macos.sh  (confirmed working)")
    print("    • BT live view → Shroom / vendor Companion app")
    print()
    print("  If you need BT in third-party Mac apps, ask JQ support for the")
    print("  JQCY-YL-135 USB serial protocol spec or an macOS SDK.")
    if mac:
        print(f"  (Your glove MAC: {mac.upper()})")


def _print_recovery_steps(port: str, binary_flood: bool) -> None:
    print("\nRecovery procedure (important — read carefully):")
    print("  The garbled characters you saw are SENSOR BINARY, not AT replies.")
    print("  Once RF auto-connects, AT+SCAN cannot get through until you reset.")
    print()
    print("  1. Quit Shroom / Companion app")
    print("  2. LONG-PRESS glove power → glove OFF (no LED)")
    print("  3. Unplug BT dongle from Mac, wait 5 s")
    print("  4. Run watch mode FIRST, then plug dongle when prompted:")
    print(f"       ./.venv/bin/python connect_bt_dongle.py --watch")
    print("  5. When dongle red LED is FLASHING (glove still OFF), watch sends AT+SCAN")
    print("  6. THEN turn glove ON (blue LED flashing) — it should appear in scan")
    print("  7. Script connects with AT+CONN=… automatically")
    if binary_flood:
        print("\n  Optional retry on already-plugged dongle (usually fails if RF linked):")
        print(f"       ./.venv/bin/python connect_bt_dongle.py --port {port} --scan-only --disc-first")


def _wait_for_dongle_unplug(timeout: float = 60.0) -> None:
    print("Waiting for dongle to disappear from USB (unplug it now if still connected)…")
    t0 = time.time()
    while time.time() - t0 < timeout:
        if not any(
            "usbserial" in p.device or "usbmodem" in p.device or "wchusbserial" in p.device
            for p in list_ports.comports()
        ):
            print("  Dongle unplugged.")
            return
        time.sleep(0.1)
    print("  (still seeing a USB serial device — continuing anyway)")


def _wait_for_dongle_plug(known: set[str], timeout: float = 30.0) -> str:
    t0 = time.time()
    while time.time() - t0 < timeout:
        for p in list_ports.comports():
            dev = p.device
            if dev in known:
                continue
            if any(x in dev for x in ("usbserial", "usbmodem", "wchusbserial")):
                return dev
        time.sleep(0.02)
    return ""


def _conn_with_mac(port: str, mac: str, listen_sec: float) -> bytes:
    import serial

    mac = mac.strip().upper().replace(":", "")
    variants = [mac, mac + "1"] if not mac.endswith("1") else [mac]
    raw = bytearray()
    ser = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=True, rtscts=False, exclusive=True)
    try:
        time.sleep(0.4)
        ser.reset_input_buffer()
        for addr in variants:
            ser.write(f"AT+CONN={addr}\r\n".encode("ascii"))
            time.sleep(0.2)
        print(f"  Sent AT+CONN for {variants} (glove can be OFF — dongle queues the link)")
        raw.extend(_read_for(ser, listen_sec))
    finally:
        ser.close()
    return bytes(raw)


def watch_and_pair(listen_sec: float, prefer_side: str, mac: str = "") -> None:
    """Wait for dongle USB plug-in, race AT+SCAN before RF auto-connect."""
    listen_sec = max(listen_sec, WATCH_PROBE)

    print("BT dongle watch mode")
    print("=" * 50)
    print("Prepare NOW:")
    print("  • Glove powered OFF (long-press until LED off)")
    print("  • Shroom / Companion app quit")
    print("  • BT dongle UNPLUGGED from this Mac")
    print()
    input("Press Enter when ready… ")

    _wait_for_dongle_unplug()
    known = {p.device for p in list_ports.comports()}
    print("\nPlug the BT dongle into the Mac NOW (you have 30 s) …")
    port = _wait_for_dongle_plug(known)
    if not port:
        raise SystemExit("No USB serial port appeared within 30 s.")

    print(f"\nDetected {port}")
    if mac:
        print(f"Phase 1: AT+CONN={mac.upper()} while glove is still OFF …")
        raw = _conn_with_mac(port, mac, listen_sec=2.0)
        _diag_raw(raw)
    else:
        print("Phase 1: AT+SCAN while glove is still OFF …")
        raw, baud = _scan_multi_baud(port, listen_sec=4.0, repeat=1)

    addrs = _find_jq_addresses(_extract_text_lines(raw))
    if mac and not addrs:
        addrs = [("auto", mac.strip().upper().replace(":", ""))]

    if not addrs and not mac:
        print("\nPhase 2: turn the glove ON now (blue LED flashing). Scanning 15 s …")
        raw2, baud2 = _scan_multi_baud(port, listen_sec=listen_sec, repeat=3)
        if len(raw2) > len(raw):
            raw, baud = raw2, baud2
        addrs = _find_jq_addresses(_extract_text_lines(raw))
    elif mac:
        print("\nPhase 2: turn the glove ON now (blue LED flashing). Listening 15 s @ 921600 …")
        import serial as ser_mod

        ser = ser_mod.Serial(port, BAUD, timeout=0.05, exclusive=True)
        try:
            ser.reset_input_buffer()
            listen_raw = _read_for(ser, listen_sec)
        finally:
            ser.close()
        if len(listen_raw) > len(raw):
            raw = listen_raw

    lines = _extract_text_lines(raw)
    clean = [ln for ln in lines if "JQ-" in ln.upper() or ln.strip() in ("OK", "ERROR")]
    print("\n--- Scan output ---")
    if clean:
        for ln in clean[:30]:
            print(" ", ln)
    elif lines:
        print("  (binary / non-AT data only — first line preview:)")
        print(" ", lines[0][:120])
    else:
        print("  (empty — 0 bytes from dongle across all baud rates)")

    if not addrs:
        from bt_dongle import looks_like_bt_binary_flood
        _print_recovery_steps(port, looks_like_bt_binary_flood(raw))
        print("\nLikely conclusion:")
        if len(raw) == 0:
            print("  This dongle did not answer AT on any baud rate. The spec §6.3 SSCOM flow may")
            print("  require the vendor Windows tool, or firmware that only exposes Juqiao data")
            print("  after pairing inside Shroom — not via raw AT on macOS.")
        else:
            print("  Your dongle (JQCY-YL-135) streams a proprietary 0x81 0x20 binary format over")
            print("  USB serial when RF is linked — NOT Juqiao AA550399. Shroom decodes this internally.")
            print("  This demo viewer only speaks Juqiao serial. Options:")
            print("    • USB Type-C direct to glove (works today)")
            print("    • Ask JQ for macOS BT serial spec or SDK for dongle model JQCY-YL-135")
            print("    • Try spec §6.3 on a Windows PC with SSCOM.exe + AT+CONN=" + (mac or "YOUR_MAC"))
        raise SystemExit(1)

    # If we only have MAC (no scan), skip second AT+CONN — already sent in phase 1
    if mac and addrs and addrs[0][1].upper().replace(":", "") == mac.upper().replace(":", ""):
        parser = FrameParser()
        frames = parser.feed(raw)
        if frames:
            print(f"\nSUCCESS: {len(frames)} Juqiao frame(s). Start viewer:")
            print(f"  ./.venv/bin/python tactile_glove_viewer.py --single {port}")
            return
        _diag_raw(raw)
        _print_jqcy_conclusion(mac)
        raise SystemExit(1)

    chosen = addrs[0]
    if prefer_side in ("left", "right"):
        for side, addr in addrs:
            if side == prefer_side:
                chosen = (side, addr)
                break

    side, addr = chosen
    print(f"\nConnecting AT+CONN={addr} ({side}) @ {baud} …")
    import serial as ser_mod

    ser = ser_mod.Serial(port, baud, timeout=0.05, dsrdtr=True, rtscts=False, exclusive=True)
    try:
        ser.reset_input_buffer()
        ser.write(f"AT+CONN={addr}\r\n".encode("ascii"))
        conn_raw = _read_for(ser, listen_sec)
    finally:
        ser.close()
    _diag_raw(conn_raw)
    parser = FrameParser()
    frames = parser.feed(conn_raw)
    if frames:
        print(f"\nSUCCESS: {len(frames)} Juqiao frame(s). Start viewer:")
        print(f"  ./.venv/bin/python tactile_glove_viewer.py --single {port}")
        return
    print(f"\nAT+CONN sent but no Juqiao frames yet. Try manually:")
    print(f"  ./.venv/bin/python connect_bt_dongle.py --port {port} --address {addr}")
    raise SystemExit(1)


def main() -> None:
    p = argparse.ArgumentParser(description="Pair Juqiao BT receiver dongle (spec §6.3, Mac)")
    p.add_argument("--port", default="", help="Serial port (default: auto-detect usbserial/usbmodem)")
    p.add_argument("--side", choices=("left", "right", "auto"), default="auto")
    p.add_argument("--mac", default="", help="Glove BT address from Shroom/dongle label, e.g. 789CE744812D")
    p.add_argument("--scan-only", action="store_true", help="Only run AT+SCAN=1 and print addresses")
    p.add_argument("--watch", action="store_true", help="Wait for dongle plug-in, race AT+SCAN (best method)")
    p.add_argument("--disc-first", action="store_true", help="Try AT+DISC before scan (rarely works if RF linked)")
    p.add_argument("--listen", type=float, default=DEFAULT_PROBE, help="Seconds to wait for responses (watch uses min 15s)")
    args = p.parse_args()

    if args.watch:
        watch_and_pair(args.listen, args.side, mac=args.mac)
        return

    port = pick_usb_port(args.port)
    if args.mac and not args.address:
        args.address = args.mac.strip().upper().replace(":", "")
    if args.scan_only:
        scan_only(port, args.listen, disc_first=args.disc_first)
        return

    if args.address:
        import serial

        addr = args.address.strip().upper()
        print(f"Connecting {port} -> AT+CONN={addr} …")
        try:
            ser = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=True, rtscts=False)
        except Exception as exc:
            raise SystemExit(f"Could not open {port}: {exc}") from exc
        try:
            ser.reset_input_buffer()
            ser.write(f"AT+CONN={addr}\r\n".encode("ascii"))
            raw = _read_for(ser, args.listen)
        finally:
            ser.close()
        parser = FrameParser()
        frames = parser.feed(raw)
        lines = _extract_text_lines(raw)
        if lines:
            print("Dongle said:")
            for ln in lines[:15]:
                print(" ", ln)
        if frames:
            print(f"\nSUCCESS: {len(frames)} Juqiao frame(s) received. Start the viewer:")
            print(f"  ./.venv/bin/python tactile_glove_viewer.py --single {port}")
            return
        print("\nNo Juqiao frames yet. Try --scan-only first, or replug dongle and retry.")
        sys.exit(1)

    print(f"Auto bridge on {port} (AT+SCAN -> AT+CONN) …")
    result = try_at_bridge(port, listen_sec=args.listen, prefer_side=args.side)
    if result.get("at_lines"):
        print("AT output:")
        for ln in result["at_lines"]:
            print(" ", ln)
    if result.get("addresses"):
        print("Addresses:", result["addresses"])
    if result.get("ok"):
        print(f"\nSUCCESS: {result['frames']} Juqiao frame(s), side={result['side']}")
        print(f"Start viewer: ./.venv/bin/python tactile_glove_viewer.py --single {port}")
        return
    err = result.get("error") or "Unknown error"
    print(f"\nFAILED: {err}")
    print("\nNext: run scan-only after replugging the dongle:")
    print(f"  ./.venv/bin/python connect_bt_dongle.py --port {port} --scan-only")
    sys.exit(1)


if __name__ == "__main__":
    main()
