#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Modular glove -> Teleop websocket agent.

Sources:
- glove_serial: Tactile glove serial protocol (AA 55 03 99)
- wuji_jsonl: JSON line stream from Wuji SDK bridge script
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import hashlib
import json
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

try:
    import websockets  # only needed by the live websocket agent, not the viewer
except ImportError:  # pragma: no cover - viewer-only installs skip this dep
    websockets = None

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import time_utils  # noqa: E402

HEADER = b"\xAA\x55\x03\x99"
PACKET_ONE_DATA_LEN = 128
PACKET_TWO_DATA_LEN = 144

GLOVE_SENSOR_TYPE = {"left": 0x01, "right": 0x02}
BEND_INDEX_MAP = {
    "left": {"thumb": 210, "index": 213, "middle": 216, "ring": 219, "little": 222},
    "right": {"thumb": 47, "index": 44, "middle": 41, "ring": 38, "little": 35},
}
REGION_INDEX_MAP = {
    "left": {
        "thumb": [19, 18, 17, 3, 2, 1, 243, 242, 241, 227, 226, 225],
        "index": [22, 21, 20, 6, 5, 4, 246, 245, 244, 230, 229, 228],
        "middle": [25, 24, 23, 9, 8, 7, 249, 248, 247, 233, 232, 231],
        "ring": [28, 27, 26, 12, 11, 10, 252, 251, 250, 236, 235, 234],
        "little": [31, 30, 29, 15, 14, 13, 255, 254, 253, 239, 238, 237],
        "palm": [207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196],
    },
    "right": {
        "thumb": [240, 239, 238, 256, 255, 254, 16, 15, 14, 32, 31, 30],
        "index": [237, 236, 235, 253, 252, 251, 13, 12, 11, 29, 28, 27],
        "middle": [234, 233, 232, 250, 249, 248, 10, 9, 8, 26, 25, 24],
        "ring": [231, 230, 229, 247, 246, 245, 7, 6, 5, 23, 22, 21],
        "little": [228, 227, 226, 244, 243, 242, 4, 3, 2, 20, 19, 18],
        "palm": [61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50],
    },
}
FINGERS = ["thumb", "index", "middle", "ring", "little"]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def gesture_from_close(close_ratio: float) -> str:
    if close_ratio >= 0.75:
        return "fist"
    if close_ratio <= 0.25:
        return "open"
    return "half_close"


def matrix_from_flat(values: List[float], rows: int, cols: int, fill: float = 0.0) -> List[List[float]]:
    arr = list(values[: rows * cols])
    if len(arr) < rows * cols:
        arr += [fill] * (rows * cols - len(arr))
    return [arr[r * cols : (r + 1) * cols] for r in range(rows)]


def flatten_matrix(matrix: List[List[float]]) -> List[float]:
    out: List[float] = []
    for row in matrix:
        out.extend(list(row))
    return out


def downsample_matrix(matrix: List[List[float]], out_rows: int, out_cols: int) -> List[List[float]]:
    in_rows = len(matrix)
    in_cols = len(matrix[0]) if in_rows > 0 else 0
    if in_rows <= 0 or in_cols <= 0:
        return [[0.0 for _ in range(out_cols)] for _ in range(out_rows)]
    out = [[0.0 for _ in range(out_cols)] for _ in range(out_rows)]
    for r in range(out_rows):
        r0 = int(r * in_rows / out_rows)
        r1 = max(r0 + 1, int((r + 1) * in_rows / out_rows))
        for c in range(out_cols):
            c0 = int(c * in_cols / out_cols)
            c1 = max(c0 + 1, int((c + 1) * in_cols / out_cols))
            s = 0.0
            n = 0
            for rr in range(r0, min(r1, in_rows)):
                for cc in range(c0, min(c1, in_cols)):
                    s += float(matrix[rr][cc])
                    n += 1
            out[r][c] = s / max(1, n)
    return out


def get_path(data: Any, path: str, default: Any = None) -> Any:
    cur = data
    for key in str(path or "").split("."):
        if not key:
            continue
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def normalize_wuji_pressure_map(raw: Any) -> List[List[float]]:
    if isinstance(raw, list) and raw and isinstance(raw[0], list):
        rows = len(raw)
        cols = len(raw[0]) if rows > 0 else 0
        if rows == 24 and cols == 32:
            return [[clamp(v, 0.0, 1.0) for v in row[:32]] for row in raw[:24]]
    if isinstance(raw, list):
        vals = [clamp(v, 0.0, 1.0) for v in raw]
        if len(vals) >= 24 * 32:
            return matrix_from_flat(vals, 24, 32, 0.0)
    return [[0.0 for _ in range(32)] for _ in range(24)]


def derive_bends_from_wuji(msg: Dict[str, Any], emf_key: str) -> Dict[str, float]:
    bends: Dict[str, float] = {}
    raw = msg.get("bends")
    if isinstance(raw, dict):
        for f in FINGERS:
            bends[f] = clamp(raw.get(f, 0.0), 0.0, 1.0)
        return bends
    raw = msg.get("joint_angles")
    if isinstance(raw, dict):
        for f in FINGERS:
            bends[f] = clamp(float(raw.get(f, 0.0)) / 90.0, 0.0, 1.0)
        return bends
    emf = get_path(msg, emf_key, {})
    if isinstance(emf, dict):
        for f in FINGERS:
            item = emf.get(f)
            if isinstance(item, (int, float)):
                bends[f] = clamp(item, 0.0, 1.0)
            elif isinstance(item, dict):
                if "bend" in item:
                    bends[f] = clamp(item.get("bend", 0.0), 0.0, 1.0)
                elif "angle" in item:
                    bends[f] = clamp(float(item.get("angle", 0.0)) / 90.0, 0.0, 1.0)
    for f in FINGERS:
        bends.setdefault(f, 0.0)
    return bends


def bends_to_joints_6(bends: Dict[str, float]) -> Dict[str, float]:
    """Map finger bends to generic 6-joint display payload."""
    thumb = float(bends.get("thumb", 0.0))
    index = float(bends.get("index", 0.0))
    middle = float(bends.get("middle", 0.0))
    ring = float(bends.get("ring", 0.0))
    little = float(bends.get("little", 0.0))
    avg = (thumb + index + middle + ring + little) / 5.0
    return {
        "j1": round(thumb, 4),
        "j2": round(index, 4),
        "j3": round(middle, 4),
        "j4": round(ring, 4),
        "j5": round(little, 4),
        "j6": round(avg, 4),
    }


def compute_regions_from_wuji(p24x32: List[List[float]]) -> Dict[str, float]:
    if not p24x32:
        return {f: 0.0 for f in ["thumb", "index", "middle", "ring", "little", "palm"]}
    cols = len(p24x32[0]) if p24x32[0] else 32
    seg = max(1, cols // 5)
    out: Dict[str, float] = {}
    for i, f in enumerate(FINGERS):
        c0 = i * seg
        c1 = cols if i == 4 else min(cols, (i + 1) * seg)
        vals: List[float] = []
        for r in range(max(0, len(p24x32) - 8), len(p24x32)):
            vals.extend(p24x32[r][c0:c1])
        out[f] = sum(vals) / max(1, len(vals))
    palm_vals: List[float] = []
    for r in range(10, min(20, len(p24x32))):
        palm_vals.extend(p24x32[r])
    out["palm"] = sum(palm_vals) / max(1, len(palm_vals))
    return out


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.pending: Dict[int, Dict[str, List[int]]] = {}

    def feed(self, chunk: bytes) -> List[Dict[str, object]]:
        self.buf.extend(chunk)
        out: List[Dict[str, object]] = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                if len(self.buf) > len(HEADER) - 1:
                    self.buf = self.buf[-(len(HEADER) - 1) :]
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < len(HEADER) + 2:
                break
            order = self.buf[4]
            sensor_type = self.buf[5]
            if order == 0x01:
                data_len = PACKET_ONE_DATA_LEN
            elif order == 0x02:
                data_len = PACKET_TWO_DATA_LEN
            else:
                del self.buf[0]
                continue
            total = len(HEADER) + 2 + data_len
            if len(self.buf) < total:
                break
            payload = list(self.buf[len(HEADER) + 2 : total])
            del self.buf[:total]
            p = self.pending.setdefault(sensor_type, {})
            p["p1" if order == 0x01 else "p2"] = payload
            if "p1" in p and "p2" in p:
                sensor = p["p1"] + p["p2"][:128]
                imu = p["p2"][128:]
                out.append({"sensor_type": sensor_type, "sensor": sensor, "imu": imu})
                self.pending[sensor_type] = {}
        return out


@dataclass
class SharedState:
    source: str
    glove_side: str
    model_name: str
    frame_count: int = 0
    last_update_ts: float = 0.0
    # Canonical clock pair captured at the same moment as last_update_ts.
    # T2's clock-sync protocol compares wall vs monotonic drift over a session,
    # so we keep both side-by-side. Both surface in the WS snapshot.
    last_update_ts_ns: int = 0
    last_update_mono_ns: int = 0
    last_error: str = ""
    detected_side: str = "unknown"
    bends: Dict[str, float] = field(default_factory=dict)
    pressure: Dict[str, float] = field(default_factory=dict)
    sensor_raw: List[float] = field(default_factory=lambda: [0.0] * 256)
    imu: List[float] = field(default_factory=lambda: [0.0] * 16)
    sensor_matrix_16x16: List[List[float]] = field(default_factory=lambda: [[0.0] * 16 for _ in range(16)])
    pressure_map_24x32: List[List[float]] = field(default_factory=lambda: [[0.0] * 32 for _ in range(24)])
    emf: Dict[str, Any] = field(default_factory=dict)
    joint_actual_position_5x4: List[List[float]] = field(default_factory=list)
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)

    async def update_from_glove_serial(self, sensor_type: int, sensor: List[int], imu: List[int]) -> None:
        async with self.lock:
            if self.glove_side == "auto":
                if sensor_type == 0x01:
                    side = "left"
                elif sensor_type == 0x02:
                    side = "right"
                else:
                    return
            else:
                side = self.glove_side
                if sensor_type != GLOVE_SENSOR_TYPE[side]:
                    return
            bends: Dict[str, float] = {}
            for k, idx1 in BEND_INDEX_MAP[side].items():
                bends[k] = clamp(sensor[idx1 - 1] / 255.0, 0.0, 1.0)
            pressure: Dict[str, float] = {}
            for region, idxs in REGION_INDEX_MAP[side].items():
                vals = [sensor[i - 1] for i in idxs if 1 <= i <= 256]
                pressure[region] = (sum(vals) / len(vals)) if vals else 0.0
            raw = list(sensor[:256])
            mat16 = matrix_from_flat([float(v) for v in raw], 16, 16, 0.0)
            self.detected_side = side
            self.bends = bends
            self.pressure = pressure
            self.sensor_raw = [float(v) for v in raw]
            self.imu = [float(v) for v in imu[:16]]
            self.sensor_matrix_16x16 = mat16
            self.pressure_map_24x32 = downsample_matrix(mat16, 24, 32)
            self.emf = {}
            self.joint_actual_position_5x4 = []
            self.frame_count += 1
            wall_ns, mono_ns = time_utils.wall_monotonic_pair()
            self.last_update_ts_ns = wall_ns
            self.last_update_mono_ns = mono_ns
            self.last_update_ts = time_utils.ns_to_epoch_s(wall_ns)
            self.last_error = ""

    async def update_from_wuji(self, msg: Dict[str, Any], args: argparse.Namespace) -> None:
        async with self.lock:
            side_raw = str(get_path(msg, args.wuji_side_key, msg.get("hand_side", "unknown")) or "unknown").lower()
            if side_raw in ("l", "lh"):
                side_raw = "left"
            if side_raw in ("r", "rh"):
                side_raw = "right"
            if self.glove_side in ("left", "right") and side_raw in ("left", "right") and side_raw != self.glove_side:
                return
            p24 = normalize_wuji_pressure_map(get_path(msg, args.wuji_pressure_key, []))
            p16 = downsample_matrix(p24, 16, 16)
            p16_u8 = [[clamp(v * 255.0, 0.0, 255.0) for v in row] for row in p16]
            bends = derive_bends_from_wuji(msg, args.wuji_emf_key)
            pressure = compute_regions_from_wuji(p24)
            imu_raw = get_path(msg, args.wuji_imu_key, msg.get("imu", []))
            imu_vals = [float(v) for v in imu_raw] if isinstance(imu_raw, list) else []
            imu_vals = (imu_vals + [0.0] * 16)[:16]
            joint_m = get_path(msg, "joint_actual_position_5x4", msg.get("joint_actual_position_5x4", []))
            joint_5x4: List[List[float]] = []
            if isinstance(joint_m, list):
                for row in joint_m[:5]:
                    if isinstance(row, list):
                        joint_5x4.append([float(v) for v in row[:4]])
            self.detected_side = side_raw if side_raw in ("left", "right") else self.glove_side
            self.bends = bends
            self.pressure = pressure
            self.sensor_matrix_16x16 = p16_u8
            self.pressure_map_24x32 = p24
            self.sensor_raw = flatten_matrix(p16_u8)
            self.imu = imu_vals
            self.emf = get_path(msg, args.wuji_emf_key, msg.get("emf", {}))
            self.joint_actual_position_5x4 = joint_5x4
            self.frame_count += 1
            wall_ns, mono_ns = time_utils.wall_monotonic_pair()
            self.last_update_ts_ns = wall_ns
            self.last_update_mono_ns = mono_ns
            self.last_update_ts = time_utils.ns_to_epoch_s(wall_ns)
            self.last_error = ""

    async def snapshot(self) -> Dict[str, object]:
        async with self.lock:
            close_ratio = sum(self.bends.values()) / max(1, len(self.bends)) if self.bends else 0.0
            return {
                "source": self.source,
                "frame_count": self.frame_count,
                "updated_at": self.last_update_ts,
                "updated_at_ns": self.last_update_ts_ns,
                "updated_at_mono_ns": self.last_update_mono_ns,
                "close_ratio": round(close_ratio, 4),
                "detected_side": self.detected_side,
                "bends": {k: round(v, 4) for k, v in self.bends.items()},
                "pressure": {k: round(v, 4) for k, v in self.pressure.items()},
                "sensor_raw": list(self.sensor_raw),
                "sensor_matrix_16x16": list(self.sensor_matrix_16x16),
                "pressure_map_24x32": list(self.pressure_map_24x32),
                "imu": list(self.imu),
                "emf": self.emf if isinstance(self.emf, dict) else {},
                "joint_actual_position_5x4": list(self.joint_actual_position_5x4),
                "last_error": self.last_error,
            }


async def glove_serial_reader_task(args: argparse.Namespace, state: SharedState) -> None:
    import serial

    parser = FrameParser()
    ser: Optional[serial.Serial] = None
    transient_empty_read_mark = "device reports readiness to read but returned no data"
    while True:
        try:
            if ser is None:
                if not args.serial:
                    raise RuntimeError("--serial is required when --source=glove_serial")
                ser = serial.Serial(args.serial, args.baudrate, timeout=0.05)
                print(f"[glove-agent] serial open: {args.serial} @ {args.baudrate}")
            chunk = ser.read(4096)
            if not chunk:
                await asyncio.sleep(0.001)
                continue
            frames = parser.feed(chunk)
            if frames:
                f = frames[-1]
                await state.update_from_glove_serial(int(f["sensor_type"]), f["sensor"], f["imu"])
            await asyncio.sleep(0)
        except Exception as exc:
            # Some USB-serial adapters intermittently raise this error while the stream is still valid.
            # Keep the port open and continue reading instead of forcing reconnect loops.
            if transient_empty_read_mark in str(exc).lower():
                async with state.lock:
                    state.last_error = str(exc)
                await asyncio.sleep(0.02)
                continue
            async with state.lock:
                state.last_error = str(exc)
            print(f"[glove-agent] serial error: {exc}; retry in 1s")
            if ser is not None:
                with contextlib.suppress(Exception):
                    ser.close()
                ser = None
            await asyncio.sleep(1.0)


async def wuji_jsonl_reader_task(args: argparse.Namespace, state: SharedState) -> None:
    proc: Optional[asyncio.subprocess.Process] = None
    while True:
        try:
            if args.wuji_cmd:
                if proc is None or proc.returncode is not None:
                    proc = await asyncio.create_subprocess_shell(
                        args.wuji_cmd,
                        stdout=asyncio.subprocess.PIPE,
                        stderr=asyncio.subprocess.STDOUT,
                    )
                    print(f"[glove-agent] wuji cmd started: {args.wuji_cmd}")
                if proc.stdout is None:
                    raise RuntimeError("wuji cmd stdout unavailable")
                line = await proc.stdout.readline()
                if not line:
                    await asyncio.sleep(0.2)
                    continue
                txt = line.decode("utf-8", errors="ignore").strip()
            else:
                txt = await asyncio.to_thread(sys.stdin.readline)
                txt = txt.strip()
                if not txt:
                    await asyncio.sleep(0.01)
                    continue
            try:
                msg = json.loads(txt)
            except json.JSONDecodeError:
                continue
            if isinstance(msg, dict):
                await state.update_from_wuji(msg, args)
            await asyncio.sleep(0)
        except Exception as exc:
            async with state.lock:
                state.last_error = str(exc)
            print(f"[glove-agent] wuji source error: {exc}; retry in 1s")
            if proc is not None and proc.returncode is None:
                with contextlib.suppress(Exception):
                    proc.terminate()
            proc = None
            await asyncio.sleep(1.0)


async def run_agent(args: argparse.Namespace) -> None:
    ws_url = args.backend.rstrip("/") + "/api/teleop/ws"
    model_name = "tactile_glove" if args.source == "glove_serial" else "wuji_glove"
    state = SharedState(source=args.source, glove_side=args.glove, model_name=model_name)
    if args.source == "glove_serial":
        asyncio.create_task(glove_serial_reader_task(args, state))
    else:
        asyncio.create_task(wuji_jsonl_reader_task(args, state))
    print(f"[glove-agent] connect {ws_url} session={args.session} node={args.node_id} source={args.source}")
    reconnect_wait = max(0.5, float(args.reconnect_min_s))
    reconnect_max = max(reconnect_wait, float(args.reconnect_max_s))
    if websockets is None:
        raise RuntimeError("The 'websockets' package is required for the live agent. "
                           "Install it with: pip install websockets")
    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                await ws.send(
                    json.dumps(
                        {
                            "role": "robot",
                            "session_id": args.session,
                            "member_id": args.node_id,
                            "display_name": args.node_id,
                        }
                    )
                )
                ready = json.loads(await ws.recv())
                if ready.get("type") != "ready":
                    raise RuntimeError(f"teleop ws not ready: {ready}")
                print("[glove-agent] ready")
                reconnect_wait = max(0.5, float(args.reconnect_min_s))

                async def sender() -> None:
                    interval = 1.0 / max(1.0, float(args.telemetry_hz))
                    frame_seq = 0
                    last_pressure_push_ts = 0.0
                    while True:
                        snap = await state.snapshot()
                        frame_seq += 1
                        ts_env = time_utils.make_ts_envelope()
                        now_ms = ts_env["ts_ms"]  # backcompat: same ms value the agent always emitted
                        close_ratio_now = float(snap.get("close_ratio") or 0.0)
                        gname = gesture_from_close(close_ratio_now)
                        calib_version = hashlib.sha1(b"{}").hexdigest()[:12]
                        hand_side = snap.get("detected_side") if args.glove == "auto" else args.glove
                        bends_payload = {k: v for k, v in (snap.get("bends") or {}).items()}
                        joints_payload = bends_payload
                        robot_model_payload = model_name
                        if str(args.device_kind) == "wuji_hand":
                            joints_payload = bends_to_joints_6(bends_payload)
                            robot_model_payload = "wuji_hand"
                        await ws.send(
                            json.dumps(
                                {
                                    "type": "telemetry",
                                    "member_id": args.node_id,
                                    "joints": joints_payload,
                                    "feedback": {
                                        "connected": bool(snap.get("frame_count", 0) > 0),
                                        "robot_model": robot_model_payload,
                                        "glove": snap,
                                    },
                                    "ts": now_ms,
                                    "ts_ns": ts_env["ts_ns"],
                                    "ts_mono_ns": ts_env["ts_mono_ns"],
                                    "ts_ms": ts_env["ts_ms"],
                                }
                            )
                        )
                        await ws.send(
                            json.dumps(
                                {
                                    "type": "glove_v2_frame",
                                    "schema": "glove.v2",
                                    "member_id": args.node_id,
                                    "device_id": args.node_id,
                                    "glove": {
                                        "hand_side": hand_side,
                                        "frame_id": frame_seq,
                                        "sample_hz": float(args.telemetry_hz),
                                        "quality": {
                                            "connected": bool(snap.get("frame_count", 0) > 0),
                                            "last_error": snap.get("last_error") or "",
                                        },
                                        "source": args.source,
                                        "bends": dict(snap.get("bends") or {}),
                                        "gesture": gname,
                                        "pressure": dict(snap.get("pressure") or {}),
                                        "region_pressure": dict(snap.get("pressure") or {}),
                                        "imu": list(snap.get("imu") or []),
                                        "emf": snap.get("emf") if isinstance(snap.get("emf"), dict) else {},
                                        "joint_actual_position_5x4": list(snap.get("joint_actual_position_5x4") or []),
                                        "pressure_map_16x16": list(snap.get("sensor_matrix_16x16") or []),
                                        "pressure_map_24x32": list(snap.get("pressure_map_24x32") or []),
                                        "sensor_raw": list(snap.get("sensor_raw") or []),
                                        "sensor_matrix_16x16": list(snap.get("sensor_matrix_16x16") or []),
                                        "close_ratio": close_ratio_now,
                                        "frame_count": int(snap.get("frame_count") or 0),
                                        "calibration_version": calib_version,
                                        "updated_at": snap.get("updated_at"),
                                        "last_error": snap.get("last_error") or "",
                                        # Canonical timestamps mirrored from outer envelope.
                                        "ts_ns": ts_env["ts_ns"],
                                        "ts_mono_ns": ts_env["ts_mono_ns"],
                                        "ts_ms": ts_env["ts_ms"],
                                    },
                                    "ts": now_ms,
                                    "ts_ns": ts_env["ts_ns"],
                                    "ts_mono_ns": ts_env["ts_mono_ns"],
                                    "ts_ms": ts_env["ts_ms"],
                                }
                            )
                        )
                        # Reuse the sender-loop ts_env (same ts_ns as the glove_v2_frame just sent),
                        # so dedup keys match across telemetry/frame/pressure-map for this tick.
                        now_ts = time_utils.ns_to_epoch_s(ts_env["ts_ns"])
                        if now_ts - last_pressure_push_ts >= 0.45:
                            last_pressure_push_ts = now_ts
                            await ws.send(
                                json.dumps(
                                    {
                                        "type": "pressure_map",
                                        "schema": "glove.v2",
                                        "member_id": args.node_id,
                                        "device_id": args.node_id,
                                        "hand_side": hand_side,
                                        "frame_id": frame_seq,
                                        "pressure_map_16x16": list(snap.get("sensor_matrix_16x16") or []),
                                        "pressure_map_24x32": list(snap.get("pressure_map_24x32") or []),
                                        "ts": now_ms,
                                        "ts_ns": ts_env["ts_ns"],
                                        "ts_mono_ns": ts_env["ts_mono_ns"],
                                        "ts_ms": ts_env["ts_ms"],
                                    }
                                )
                            )
                        await asyncio.sleep(interval)

                send_task = asyncio.create_task(sender())
                try:
                    while True:
                        raw = await ws.recv()
                        msg = json.loads(raw)
                        typ = msg.get("type")
                        if typ == "__ping":
                            await ws.send(json.dumps({"type": "__pong", "ts": msg.get("ts")}))
                            continue
                        if typ == "ping_latency":
                            await ws.send(
                                json.dumps({"type": "pong_latency", "ts": msg.get("ts"), "id": msg.get("id"), "member_id": args.node_id})
                            )
                            continue
                finally:
                    send_task.cancel()
                    # Python 3.11+ raises asyncio.CancelledError from BaseException,
                    # so suppress it explicitly to allow outer reconnect loop.
                    with contextlib.suppress(asyncio.CancelledError, Exception):
                        await send_task
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[glove-agent] ws disconnected: {exc}; reconnect in {reconnect_wait:.1f}s")
            await asyncio.sleep(reconnect_wait)
            reconnect_wait = min(reconnect_max, reconnect_wait * 1.7)


def main() -> None:
    parser = argparse.ArgumentParser(description="Modular glove teleop session agent")
    parser.add_argument("--backend", default="ws://127.0.0.1:8000")
    parser.add_argument("--session", required=True)
    parser.add_argument("--node-id", default="glove-node")
    parser.add_argument("--source", choices=["glove_serial", "wuji_jsonl"], default="glove_serial")
    parser.add_argument("--device-kind", choices=["glove", "wuji_hand"], default="glove")
    parser.add_argument("--serial", default="", help="required for glove_serial, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--glove", choices=["left", "right", "auto"], default="auto")
    parser.add_argument("--telemetry-hz", type=float, default=12.0)
    parser.add_argument("--reconnect-min-s", type=float, default=1.0)
    parser.add_argument("--reconnect-max-s", type=float, default=10.0)

    parser.add_argument("--wuji-cmd", default="", help="command that prints Wuji JSON lines to stdout")
    parser.add_argument("--wuji-pressure-key", default="pressure_map_24x32")
    parser.add_argument("--wuji-emf-key", default="emf")
    parser.add_argument("--wuji-imu-key", default="imu")
    parser.add_argument("--wuji-side-key", default="hand_side")
    args = parser.parse_args()
    asyncio.run(run_agent(args))


if __name__ == "__main__":
    main()

