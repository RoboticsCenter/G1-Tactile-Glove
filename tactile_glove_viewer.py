#!/usr/bin/env python3
"""
Tactile Glove local demo viewer (offline, no Fearless backend).

Reads Juqiao serial protocol (AA 55 03 99) from USB COM ports and serves
a browser UI at http://127.0.0.1:8877 with bends, pressure heatmap,
3D hand pose, calibration, and retarget pose JSON.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import struct
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, unquote, urlparse

import serial
from serial.tools import list_ports

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from linker_glove_agent import (  # noqa: E402
    BEND_INDEX_MAP,
    FINGERS,
    FrameParser,
    GLOVE_SENSOR_TYPE,
    REGION_INDEX_MAP,
    clamp,
    downsample_matrix,
    matrix_from_flat,
)

DEFAULT_PORT = 8877
BAUD = 921600
CAL_DIR = _HERE / "calibration"
ASSETS_DIR = _HERE / "assets"
_ASSET_CONTENT_TYPES = {
    ".js": "application/javascript; charset=utf-8",
    ".mjs": "application/javascript; charset=utf-8",
    ".glb": "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".bin": "application/octet-stream",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".wasm": "application/wasm",
}
_rec_env = (os.getenv("TACTILE_GLOVE_RECORDINGS_DIR", "") or "").strip()
REC_DIR = Path(_rec_env).expanduser() if _rec_env else (_HERE / "recordings")
RECORD_HZ = 15.0
RECORD_SCHEMA = "robotics_center.tactile_glove.recording.v1"

FINGER_JOINTS = {
    "thumb": ["cmc", "mcp", "ip"],
    "index": ["mcp", "pip", "dip"],
    "middle": ["mcp", "pip", "dip"],
    "ring": ["mcp", "pip", "dip"],
    "little": ["mcp", "pip", "dip"],
}


def glove_offline_message(side: str) -> str:
    label = "Left" if side == "left" else "Right" if side == "right" else side.capitalize()
    return f"{label} glove offline — check Connection"


def default_calibration() -> Dict[str, Any]:
    return {
        "version": 1,
        "calibrated_at": None,
        "capture": {"open": False, "closed": False},
        "fingers": {f: {"gain": 1.0, "offset": 0.0} for f in FINGERS},
        "imu_ref": None,
    }


def imu_bytes_to_quat(imu: Any) -> Optional[List[float]]:
    """Decode the 16 IMU bytes (4x float32 little-endian, w,x,y,z per JQ spec)
    into a normalized quaternion [w, x, y, z]. Returns None if unusable."""
    if not imu or len(imu) < 16:
        return None
    try:
        b = bytes((int(round(float(v))) % 256 + 256) % 256 for v in list(imu)[:16])
        w, x, y, z = struct.unpack("<ffff", b)
    except (ValueError, TypeError, struct.error):
        return None
    n = (w * w + x * x + y * y + z * z) ** 0.5
    if not (n > 1e-6) or n != n:  # reject zero / NaN
        return None
    return [w / n, x / n, y / n, z / n]


def compute_gains_from_open_closed(cal: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize each finger so open->0 and closed->1."""
    fingers = cal.setdefault("fingers", {})
    for f in FINGERS:
        cfg = fingers.setdefault(f, {})
        open_r = float(cfg.get("open_raw", 0.0))
        closed_r = float(cfg.get("closed_raw", 255.0))
        if closed_r <= open_r:
            closed_r = open_r + 1.0
        cfg["closed_raw"] = closed_r
        cfg["gain"] = 1.0
        cfg["offset"] = 0.0
    cal["calibrated_at"] = time.time()
    cal.setdefault("capture", {})["closed"] = True
    return cal


def evaluate_calibration_quality(cal: Dict[str, Any], raw_bends: Dict[str, float]) -> Dict[str, Any]:
    """Score how well calibration maps current pose; useful after capture."""
    calibrated = apply_calibration(raw_bends, cal)
    fingers = cal.get("fingers", {})
    per_finger: Dict[str, Any] = {}
    issues: List[str] = []
    for f in FINGERS:
        cfg = fingers.get(f, {})
        open_r = float(cfg.get("open_raw", 0.0))
        closed_r = float(cfg.get("closed_raw", 255.0))
        span = max(1.0, closed_r - open_r)
        raw = float(raw_bends.get(f, 0.0)) * 255.0
        norm = clamp((raw - open_r) / span, 0.0, 1.0)
        per_finger[f] = {
            "raw": round(raw, 1),
            "calibrated": round(float(calibrated.get(f, 0.0)), 3),
            "open_raw": round(open_r, 1),
            "closed_raw": round(closed_r, 1),
            "span": round(span, 1),
            "norm_before_gain": round(norm, 3),
            "gain": float(cfg.get("gain", 1.0)),
            "offset": float(cfg.get("offset", 0.0)),
        }
        if span < 8.0:
            issues.append(f"{f}: span too small ({span:.0f})")
        if closed_r <= open_r:
            issues.append(f"{f}: closed <= open")
    grip = sum(calibrated.values()) / max(1, len(calibrated))
    return {
        "per_finger": per_finger,
        "grip": round(grip, 3),
        "issues": issues,
        "ok": len(issues) == 0,
    }


def apply_calibration(raw_bends: Dict[str, float], cal: Dict[str, Any]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    fingers = cal.get("fingers", {})
    for f in FINGERS:
        cfg = fingers.get(f, {})
        gain = float(cfg.get("gain", 1.0))
        offset = float(cfg.get("offset", 0.0))
        open_r = float(cfg.get("open_raw", 0.0))
        closed_r = float(cfg.get("closed_raw", 255.0))
        span = max(1.0, closed_r - open_r)
        raw = raw_bends.get(f, 0.0) * 255.0
        val = ((raw - open_r) / span) * gain + offset
        out[f] = round(clamp(val, 0.0, 1.0), 4)
    return out


def build_retarget_pose(hands: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "schema": "robotics_center.tactile_glove.retarget_pose.v1",
        "coordinate": "normalized_hand",
        "range": {"open": 0.0, "closed": 1.0},
        "timestamp": time.time(),
        "hands": {},
    }
    for side, data in hands.items():
        bends = data.get("bends", {})
        grip = sum(bends.values()) / max(1, len(bends)) if bends else 0.0
        fingers: Dict[str, Any] = {}
        for f in FINGERS:
            b = float(bends.get(f, 0.0))
            joints: Dict[str, float] = {}
            for j in FINGER_JOINTS[f]:
                scale = 1.0 if j in ("mcp", "cmc") else 0.85 if j == "pip" else 0.6
                joints[f"{j}_flex"] = round(clamp(b * scale, 0.0, 1.0), 4)
            fingers[f] = {"bend": round(b, 4), "joints": joints}
        payload["hands"][side] = {"grip": round(grip, 4), "fingers": fingers}
    return payload


class GloveReader:
    def __init__(self, port: str, prefer_side: str = "auto", cal: Optional[Dict[str, Any]] = None):
        self.port = port
        self.prefer_side = prefer_side
        self.cal = cal or default_calibration()
        self.parser = FrameParser()
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.ser: Optional[serial.Serial] = None
        self.state: Dict[str, Any] = self._empty_state()

    def _empty_state(self) -> Dict[str, Any]:
        return {
            "port": self.port,
            "online": False,
            "side": "unknown",
            "fps": 0.0,
            "frame_count": 0,
            "error": "",
            "bends": {f: 0.0 for f in FINGERS},
            "bends_raw": {f: 0.0 for f in FINGERS},
            "pressure": {k: 0.0 for k in ["thumb", "index", "middle", "ring", "little", "palm"]},
            "matrix_16x16": [[0.0] * 16 for _ in range(16)],
            "imu": [0.0] * 16,
            "pose_joints": {},
        }

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def set_calibration(self, cal: Dict[str, Any]) -> None:
        with self.lock:
            self.cal = cal

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.state)

    def is_online(self) -> bool:
        with self.lock:
            return bool(self.state.get("online"))

    def _detect_side(self, sensor_type: int) -> Optional[str]:
        if self.prefer_side in ("left", "right"):
            if sensor_type == GLOVE_SENSOR_TYPE[self.prefer_side]:
                return self.prefer_side
            return None
        if sensor_type == GLOVE_SENSOR_TYPE["left"]:
            return "left"
        if sensor_type == GLOVE_SENSOR_TYPE["right"]:
            return "right"
        return None

    def _loop(self) -> None:
        fps_count = 0
        fps_t0 = time.time()
        while self.running:
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, BAUD, timeout=0.01)
                chunk = self.ser.read(4096)
                if not chunk:
                    time.sleep(0.001)
                    continue
                frames = self.parser.feed(chunk)
                if not frames:
                    continue
                f = frames[-1]
                side = self._detect_side(int(f["sensor_type"]))
                if not side:
                    continue
                sensor = f["sensor"]
                raw_bends = {
                    k: clamp(sensor[idx - 1] / 255.0, 0.0, 1.0)
                    for k, idx in BEND_INDEX_MAP[side].items()
                }
                pressure: Dict[str, float] = {}
                for region, idxs in REGION_INDEX_MAP[side].items():
                    vals = [sensor[i - 1] for i in idxs if 1 <= i <= 256]
                    pressure[region] = round((sum(vals) / len(vals)) / 255.0 if vals else 0.0, 4)
                mat = matrix_from_flat([float(v) for v in sensor[:256]], 16, 16, 0.0)
                for _c in range(16):
                    mat[0][_c] = max(mat[0][_c], mat[14][_c])
                    mat[1][_c] = max(mat[1][_c], mat[15][_c])
                mat[14] = [0.0] * 16
                mat[15] = [0.0] * 16
                bends = apply_calibration(raw_bends, self.cal)
                fps_count += 1
                now = time.time()
                if now - fps_t0 >= 1.0:
                    fps = fps_count / (now - fps_t0)
                    fps_count = 0
                    fps_t0 = now
                else:
                    with self.lock:
                        fps = self.state.get("fps", 0.0)
                with self.lock:
                    self.state.update(
                        {
                            "online": True,
                            "side": side,
                            "fps": round(fps, 1),
                            "frame_count": self.state["frame_count"] + 1,
                            "error": "",
                            "bends": bends,
                            "bends_raw": {k: round(v, 4) for k, v in raw_bends.items()},
                            "pressure": pressure,
                            "matrix_16x16": mat,
                            "imu": [float(x) for x in f["imu"][:16]],
                        }
                    )
            except Exception as exc:
                with self.lock:
                    self.state["error"] = str(exc)
                    self.state["online"] = False
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.25)


def load_calibration(side: str) -> Dict[str, Any]:
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    for name in (f"tactile_glove_{side}.json", f"juqiao_{side}.json"):
        path = CAL_DIR / name
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    return default_calibration()


def save_calibration(side: str, data: Dict[str, Any]) -> Path:
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    path = CAL_DIR / f"tactile_glove_{side}.json"
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return path


def _safe_recording_id(name: str) -> str:
    base = Path(unquote(name)).stem
    base = re.sub(r"[^A-Za-z0-9._-]", "_", base)
    return base[:120] or "demo"


_META_HEAD_CHARS = 16384


def _recording_meta_from_file(path: Path) -> Dict[str, Any]:
    """Read only the JSON header — avoids loading huge frame arrays for the list API."""
    meta: Dict[str, Any] = {
        "id": path.stem,
        "filename": path.name,
        "label": path.stem,
        "created_at": None,
        "duration_s": 0.0,
        "frame_count": 0,
        "frame_hz": RECORD_HZ,
    }
    try:
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            head = fh.read(_META_HEAD_CHARS)
        patterns = (
            ("label", r'"label"\s*:\s*"((?:\\.|[^"\\])*)"', lambda s: json.loads('"' + s + '"')),
            ("created_at", r'"created_at"\s*:\s*"([^"]*)"', str),
            ("duration_s", r'"duration_s"\s*:\s*([\d.]+)', float),
            ("frame_hz", r'"frame_hz"\s*:\s*([\d.]+)', float),
            ("frame_count", r'"frame_count"\s*:\s*(\d+)', int),
        )
        for key, pattern, conv in patterns:
            m = re.search(pattern, head)
            if m:
                meta[key] = conv(m.group(1))
    except Exception:
        pass
    return meta


def list_recordings() -> List[Dict[str, Any]]:
    REC_DIR.mkdir(parents=True, exist_ok=True)
    out: List[Dict[str, Any]] = []
    for path in sorted(REC_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        out.append(_recording_meta_from_file(path))
    return out


def load_recording(rec_id: str) -> Dict[str, Any]:
    path = REC_DIR / f"{_safe_recording_id(rec_id)}.json"
    if not path.exists():
        raise FileNotFoundError(rec_id)
    return json.loads(path.read_text(encoding="utf-8"))


def delete_recording(rec_id: str) -> bool:
    path = REC_DIR / f"{_safe_recording_id(rec_id)}.json"
    if not path.exists():
        return False
    path.unlink()
    return True


def save_recording_payload(data: Dict[str, Any], filename: str = "") -> Dict[str, Any]:
    REC_DIR.mkdir(parents=True, exist_ok=True)
    frames = data.get("frames", [])
    if not isinstance(frames, list) or not frames:
        raise ValueError("recording must contain a non-empty frames array")
    if filename:
        safe_name = Path(filename).name
        if re.fullmatch(r"[A-Za-z0-9._-]+\.json", safe_name):
            existing = REC_DIR / safe_name
            if existing.exists():
                saved = _recording_meta_from_file(existing)
                return {
                    "id": saved["id"],
                    "path": str(existing),
                    "frame_count": saved["frame_count"],
                    "duration_s": saved["duration_s"],
                    "label": saved.get("label") or saved["id"],
                    "already_exists": True,
                }
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = _safe_recording_id(Path(filename).stem if filename else data.get("label") or f"imported_{stamp}")
    if not stem.startswith("demo_") and not stem.startswith("imported_"):
        stem = f"imported_{stamp}_{stem}"[:120]
    path = REC_DIR / f"{stem}.json"
    duration = float(data.get("duration_s") or (frames[-1].get("t", 0) if frames else 0))
    payload = {
        "schema": data.get("schema") or RECORD_SCHEMA,
        "label": data.get("label") or stem,
        "created_at": data.get("created_at") or datetime.now().isoformat(),
        "duration_s": round(duration, 2),
        "frame_hz": float(data.get("frame_hz") or RECORD_HZ),
        "frame_count": len(frames),
        "frames": frames,
    }
    if data.get("imu_ref"):
        payload["imu_ref"] = data["imu_ref"]
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return {"id": path.stem, "path": str(path), "frame_count": len(frames), "duration_s": payload["duration_s"]}


def open_recordings_folder() -> str:
    REC_DIR.mkdir(parents=True, exist_ok=True)
    folder = str(REC_DIR.resolve())
    if sys.platform == "win32":
        os.startfile(folder)  # type: ignore[attr-defined]
    elif sys.platform == "darwin":
        import subprocess
        subprocess.Popen(["open", folder])
    else:
        import subprocess
        subprocess.Popen(["xdg-open", folder])
    return folder


def auto_detect_ports() -> Dict[str, str]:
    """Return {left: COMx, right: COMy} by probing."""
    from probe_glove_ports import probe_port  # local import

    mapping: Dict[str, str] = {}
    for p in list_ports.comports():
        r = probe_port(p.device)
        if not r["ok"]:
            continue
        if r["side"] == "left" and "left" not in mapping:
            mapping["left"] = p.device
        elif r["side"] == "right" and "right" not in mapping:
            mapping["right"] = p.device
    return mapping


def make_handler(app: "ViewerApp"):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code: int, obj: Any) -> None:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _html(self, html: str) -> None:
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self._html(HTML_PAGE)
                return
            if path == "/api/state":
                self._json(200, app.get_state())
                return
            if path == "/api/retarget_pose":
                self._json(200, app.get_retarget_pose())
                return
            if path == "/api/calibration":
                qs = parse_qs(urlparse(self.path).query)
                side = (qs.get("side") or ["left"])[0]
                self._json(200, app.get_calibration_status(side))
                return
            if path == "/api/recording/status":
                self._json(200, app.recording_status())
                return
            if path == "/api/recordings":
                self._json(200, {"recordings": list_recordings(), "folder": str(REC_DIR.resolve())})
                return
            if path == "/api/recordings/folder":
                self._json(200, {"folder": str(REC_DIR.resolve())})
                return
            if path.startswith("/api/recordings/"):
                rec_id = _safe_recording_id(path.split("/api/recordings/", 1)[1])
                try:
                    self._json(200, load_recording(rec_id))
                except FileNotFoundError:
                    self.send_response(404)
                    self.end_headers()
                return
            if path.startswith("/assets/"):
                self._serve_asset(path[len("/assets/"):])
                return
            self.send_response(404)
            self.end_headers()

        def _serve_asset(self, name: str) -> None:
            # Serve bundled static files (three.js, models) locally — no CDN/network.
            rel = unquote(name or "").replace("\\", "/").lstrip("/")
            if not rel or ".." in rel.split("/"):
                self.send_response(403)
                self.end_headers()
                return
            target = (ASSETS_DIR / rel).resolve()
            try:
                target.relative_to(ASSETS_DIR.resolve())
            except ValueError:
                self.send_response(403)
                self.end_headers()
                return
            if not target.is_file():
                self.send_response(404)
                self.end_headers()
                return
            ctype = _ASSET_CONTENT_TYPES.get(target.suffix.lower(), "application/octet-stream")
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "public, max-age=86400")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            n = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(n) if n else b"{}"
            if path == "/api/recordings/upload-file":
                try:
                    filename = unquote(self.headers.get("X-Filename", "imported.json"))
                    rec = json.loads(raw.decode("utf-8"))
                    saved = save_recording_payload(rec, filename)
                    self._json(200, {"ok": True, **saved})
                except Exception as exc:
                    self._json(400, {"ok": False, "error": str(exc)})
                return
            try:
                body = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                self._json(400, {"error": "invalid json"})
                return
            if path == "/api/calibration/save":
                side = body.get("side", "left")
                cal = body.get("calibration") or app.get_calibration(side)
                cal["calibrated_at"] = time.time()
                p = save_calibration(side, cal)
                app.set_calibration(side, cal)
                self._json(200, {"ok": True, "path": str(p), "calibration": cal})
                return
            if path == "/api/calibration/apply":
                side = body.get("side", "left")
                cal = body.get("calibration") or app.get_calibration(side)
                app.set_calibration(side, cal)
                snap = app.readers.get(side)
                raw = snap.snapshot().get("bends_raw", {}) if snap else {}
                self._json(200, {"ok": True, "quality": evaluate_calibration_quality(cal, raw)})
                return
            if path == "/api/calibration/reset":
                side = body.get("side", "left")
                cal = default_calibration()
                cal["imu_ref"] = app.get_calibration(side).get("imu_ref")
                app.set_calibration(side, cal)
                snap = app.readers.get(side)
                raw = snap.snapshot().get("bends_raw", {}) if snap else {}
                self._json(200, {"ok": True, "calibration": cal, "quality": evaluate_calibration_quality(cal, raw)})
                return
            if path == "/api/calibration/auto_compute":
                side = body.get("side", "left")
                cal = app.get_calibration(side)
                capture = cal.get("capture", {})
                if not capture.get("open") or not capture.get("closed"):
                    missing = []
                    if not capture.get("open"):
                        missing.append("Capture Open")
                    if not capture.get("closed"):
                        missing.append("Capture Closed")
                    self._json(400, {
                        "ok": False,
                        "error": "need_both_captures",
                        "message": f"Still need: {' + '.join(missing)} (for {side} hand)",
                        "steps": {
                            "open_captured": bool(capture.get("open")),
                            "closed_captured": bool(capture.get("closed")),
                            "ready": False,
                        },
                    })
                    return
                cal = compute_gains_from_open_closed(cal)
                app.set_calibration(side, cal)
                snap = app.readers.get(side)
                raw = snap.snapshot().get("bends_raw", {}) if snap else {}
                self._json(200, {"ok": True, "calibration": cal, "quality": evaluate_calibration_quality(cal, raw)})
                return
            if path == "/api/calibration/capture":
                side = body.get("side", "left")
                pose = body.get("pose", "open")
                samples = int(body.get("samples", 25))
                result = app.capture_pose(side, pose, samples)
                self._json(200, {"ok": True, **result})
                return
            if path == "/api/imu/calibrate":
                self._json(200, app.calibrate_imu(body.get("side", "left")))
                return
            if path == "/api/imu/reset":
                self._json(200, app.reset_imu(body.get("side", "left")))
                return
            if path == "/api/recording/start":
                label = str(body.get("label", "") or "").strip()
                self._json(200, app.start_recording(label))
                return
            if path == "/api/recording/stop":
                self._json(200, app.stop_recording())
                return
            if path == "/api/recordings/delete":
                rec_id = body.get("id", "")
                ok = delete_recording(rec_id)
                self._json(200 if ok else 404, {"ok": ok})
                return
            if path == "/api/recordings/open-folder":
                folder = open_recordings_folder()
                self._json(200, {"ok": True, "folder": folder})
                return
            if path == "/api/recordings/upload":
                try:
                    rec = body.get("recording") or body
                    saved = save_recording_payload(rec, str(body.get("filename", "") or ""))
                    self._json(200, {"ok": True, **saved})
                except Exception as exc:
                    self._json(400, {"ok": False, "error": str(exc)})
                return
            self.send_response(404)
            self.end_headers()

    return Handler


class DemoRecorder:
    def __init__(self, sample_fn, hz: float = RECORD_HZ) -> None:
        self._sample_fn = sample_fn
        self._hz = hz
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frames: List[Dict[str, Any]] = []
        self._t0 = 0.0
        self._label = ""

    def start(self, label: str = "") -> None:
        with self._lock:
            if self._running:
                return
            self._frames = []
            self._t0 = time.time()
            self._label = label
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._running = False
            frames = list(self._frames)
            label = self._label
            t0 = self._t0
        if self._thread:
            self._thread.join(timeout=2.0)
        duration = (frames[-1]["t"] if frames else 0.0)
        return {
            "frames": frames,
            "label": label,
            "started_at": t0,
            "duration_s": round(duration, 2),
            "frame_hz": self._hz,
        }

    def status(self) -> Dict[str, Any]:
        with self._lock:
            n = len(self._frames)
            elapsed = time.time() - self._t0 if self._running else 0.0
            return {
                "recording": self._running,
                "frame_count": n,
                "elapsed_s": round(elapsed, 1),
                "label": self._label,
            }

    def _loop(self) -> None:
        interval = 1.0 / max(1.0, self._hz)
        next_t = time.time()
        while True:
            with self._lock:
                if not self._running:
                    break
                t0 = self._t0
            snap = self._sample_fn()
            frame = {
                "t": round(time.time() - t0, 3),
                "hands": snap.get("hands", {}),
                "summary": snap.get("summary", {}),
            }
            with self._lock:
                self._frames.append(frame)
            next_t += interval
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)


class ViewerApp:
    # Hot-plug watcher tuning.
    WATCH_INTERVAL = 2.5       # seconds between scans when all gloves are live
    WATCH_INTERVAL_FAST = 0.5  # faster scan rate while waiting for a glove to reappear
    PROBE_SEC = 2.0        # how long to listen on a candidate port (match probe_glove_ports.py)
    PROBE_SEC_BT = 4.0     # BT dongles (/dev/cu.usbmodem-*) often need longer to start streaming
    NEG_COOLDOWN = 20.0    # re-probe a port that opened but had no glove only this often
    BUSY_RETRY = 0.0       # a port we couldn't open (busy/Shroom) is retried every tick
    _SKIP_PORT_MARKERS = ("Bluetooth-Incoming-Port", "debug-console")

    def __init__(self, ports: Dict[str, str]):
        self._cal_lock = threading.Lock()
        self._readers_lock = threading.RLock()
        self._pending_capture: Dict[str, Dict[str, List[Dict[str, float]]]] = {}
        self._calibrations: Dict[str, Dict[str, Any]] = {}
        self.readers: Dict[str, GloveReader] = {}
        self._recorder = DemoRecorder(self._build_live_state)
        # Hot-plug watcher state.
        self._watch_running = False
        self._watch_thread: Optional[threading.Thread] = None
        self._probe_cooldown: Dict[str, float] = {}
        for side, port in ports.items():
            prefer = "auto" if side == "auto" else side
            key = side if side != "auto" else "left"
            self._attach_reader(key, port, prefer)

    def _reader_items(self) -> List["tuple"]:
        with self._readers_lock:
            return list(self.readers.items())

    def _attach_reader(self, key: str, port: str, prefer_side: str) -> None:
        cal_side = key if key in ("left", "right") else "left"
        cal = self.get_calibration(cal_side)
        with self._cal_lock:
            self._calibrations[cal_side] = cal
        r = GloveReader(port, prefer_side=prefer_side, cal=cal)
        r.start()
        with self._readers_lock:
            old = self.readers.get(key)
            if old is not None and old is not r:
                old.stop()
            self.readers[key] = r

    def start_watcher(self) -> None:
        if self._watch_running:
            return
        self._watch_running = True
        self._watch_thread = threading.Thread(target=self._watch_loop, daemon=True)
        self._watch_thread.start()
        print(f"[viewer] hot-plug watcher running (scan every {self.WATCH_INTERVAL}s) — "
              "plug in a glove (or close Shroom to free the COM port) and it goes live automatically")
        try:
            ports = list(list_ports.comports())
            if ports:
                print("[viewer] serial ports detected at startup:")
                for p in ports:
                    print(f"          {p.device}  ({p.description or 'no description'})")
            else:
                print("[viewer] NO serial ports detected. If a glove is plugged in but not "
                      "listed, the USB-serial driver may be missing (e.g. CH340/WCH on macOS).")
        except Exception as exc:
            print(f"[viewer] could not enumerate serial ports: {exc}")

    def _watch_loop(self) -> None:
        while self._watch_running:
            try:
                self._scan_once()
            except Exception as exc:  # never let the watcher die
                print(f"[viewer] watcher scan error (continuing): {exc}")
            with self._readers_lock:
                all_ok = bool(self.readers) and all(r.is_online() for r in self.readers.values())
            time.sleep(self.WATCH_INTERVAL if all_ok else self.WATCH_INTERVAL_FAST)

    @classmethod
    def _should_probe_port(cls, dev: str) -> bool:
        name = dev.rsplit("/", 1)[-1]
        return not any(marker in name for marker in cls._SKIP_PORT_MARKERS)

    @classmethod
    def _probe_sec_for_port(cls, dev: str) -> float:
        # USB-serial gloves usually stream immediately; BT dongles (usbmodem) lag after plug-in.
        if "usbmodem" in dev:
            return cls.PROBE_SEC_BT
        return cls.PROBE_SEC

    def _scan_once(self) -> None:
        try:
            from probe_glove_ports import probe_port  # local import (cached after first)
        except Exception:
            return
        present = {p.device for p in list_ports.comports()}
        # 1) Drop readers whose USB device vanished and are currently offline,
        #    freeing the side so a reconnect (possibly on a new COM port) is re-detected.
        for side, r in self._reader_items():
            if r.port not in present and not r.is_online():
                r.stop()
                with self._readers_lock:
                    if self.readers.get(side) is r:
                        del self.readers[side]
        with self._readers_lock:
            owned = {r.port for r in self.readers.values()}
        # 2) Probe unowned present ports (outside the readers lock — probing is slow).
        now = time.time()
        for dev in sorted(present):
            if dev in owned:
                continue
            if not self._should_probe_port(dev):
                continue
            if self._probe_cooldown.get(dev, 0.0) > now:
                continue
            probe_sec = self._probe_sec_for_port(dev)
            info = probe_port(dev, probe_sec=probe_sec)
            if info.get("ok"):
                side = info.get("side") or "none"
                if side == "both":
                    side = "left"
                if side not in ("left", "right"):
                    self._probe_cooldown[dev] = now + self.NEG_COOLDOWN
                    continue
                with self._readers_lock:
                    ex = self.readers.get(side)
                if ex is not None and ex.is_online():
                    continue  # already have this hand live elsewhere
                self._attach_reader(side, dev, side)
                print(f"[viewer] hot-plug: {side} glove connected on {dev}")
            elif info.get("opened"):
                # Opened fine but no glove frames — don't keep disturbing this device.
                mode = info.get("mode", "")
                extra = ""
                if info.get("binary_flood") or mode == "bt_dongle":
                    extra = " [BT dongle: RF may be paired but Juqiao bridge (AT+CONN) not active]"
                print(f"[viewer] probed {dev}: opened OK but no glove frames{extra} "
                      f"(not a glove, or it isn't streaming). {info.get('error','')}".rstrip())
                self._probe_cooldown[dev] = now + self.NEG_COOLDOWN
            else:
                # Couldn't open (busy / held by another app / permissions) — retry quickly.
                print(f"[viewer] probed {dev}: could not open "
                      f"(busy, or permission denied). {info.get('error','')}".rstrip())
                self._probe_cooldown[dev] = now + self.BUSY_RETRY

    def get_calibration(self, side: str) -> Dict[str, Any]:
        with self._cal_lock:
            if side not in self._calibrations:
                self._calibrations[side] = load_calibration(side)
            return json.loads(json.dumps(self._calibrations[side]))

    def _build_live_state(self) -> Dict[str, Any]:
        hands = {side: r.snapshot() for side, r in self._reader_items()}
        online = sum(1 for h in hands.values() if h.get("online"))
        fps_vals = [h.get("fps", 0.0) for h in hands.values() if h.get("online")]
        grips = []
        peaks = []
        for h in hands.values():
            b = h.get("bends", {})
            if b:
                grips.append(sum(b.values()) / len(b))
            m = h.get("matrix_16x16", [])
            if m:
                peaks.append(max(max(row) for row in m))
        return {
            "hands": hands,
            "summary": {
                "online_gloves": online,
                "avg_fps": round(sum(fps_vals) / max(1, len(fps_vals)), 1),
                "avg_grip": round(sum(grips) / max(1, len(grips)), 3),
                "peak_pressure": round(max(peaks) if peaks else 0.0, 1),
            },
        }

    def get_state(self) -> Dict[str, Any]:
        state = self._build_live_state()
        state["imu_ref"] = {
            side: self.get_calibration(side).get("imu_ref")
            for side, _ in self._reader_items()
        }
        return state

    def calibrate_imu(self, side: str) -> Dict[str, Any]:
        with self._readers_lock:
            reader = self.readers.get(side)
        snap = reader.snapshot() if reader else None
        if not snap or not snap.get("online"):
            return {"ok": False, "side": side, "offline": True,
                    "message": glove_offline_message(side)}
        q = imu_bytes_to_quat(snap.get("imu") or [])
        if not q:
            return {"ok": False, "side": side, "message": "No IMU data received from glove"}
        cal = self.get_calibration(side)
        cal["imu_ref"] = q
        self.set_calibration(side, cal)
        return {"ok": True, "side": side, "imu_ref": q}

    def reset_imu(self, side: str) -> Dict[str, Any]:
        cal = self.get_calibration(side)
        cal["imu_ref"] = None
        save_calibration(side, cal)
        self.set_calibration(side, cal)
        return {"ok": True, "side": side, "imu_ref": None}

    def active_imu_refs(self) -> Dict[str, List[float]]:
        refs: Dict[str, List[float]] = {}
        for side, _ in self._reader_items():
            r = self.get_calibration(side).get("imu_ref")
            if r:
                refs[side] = r
        return refs

    def start_recording(self, label: str = "") -> Dict[str, Any]:
        if self._recorder.status()["recording"]:
            return {"ok": False, "message": "Already recording"}
        self._recorder.start(label)
        return {"ok": True, **self._recorder.status()}

    def stop_recording(self) -> Dict[str, Any]:
        if not self._recorder.status()["recording"]:
            return {"ok": False, "message": "Not recording"}
        data = self._recorder.stop()
        REC_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = re.sub(r"[^A-Za-z0-9_-]", "_", data.get("label") or "").strip("_")
        fname = f"demo_{stamp}" + (f"_{safe_label[:32]}" if safe_label else "") + ".json"
        saved = save_recording_payload(
            {
                "schema": RECORD_SCHEMA,
                "label": data.get("label") or fname,
                "created_at": datetime.now().isoformat(),
                "duration_s": data["duration_s"],
                "frame_hz": data["frame_hz"],
                "imu_ref": self.active_imu_refs(),
                "frames": data["frames"],
            },
            fname,
        )
        print(f"[viewer] saved recording: {saved['path']} ({saved['frame_count']} frames)")
        return {"ok": True, **saved}

    def recording_status(self) -> Dict[str, Any]:
        return self._recorder.status()

    def get_retarget_pose(self) -> Dict[str, Any]:
        hands = {side: r.snapshot() for side, r in self._reader_items()}
        pose_hands = {s: {"bends": h.get("bends", {})} for s, h in hands.items()}
        return build_retarget_pose(pose_hands)

    def set_calibration(self, side: str, cal: Dict[str, Any]) -> None:
        with self._cal_lock:
            self._calibrations[side] = cal
        with self._readers_lock:
            r = self.readers.get(side)
        if r is not None:
            r.set_calibration(cal)

    def average_raw_bends(self, side: str, samples: int = 25) -> Dict[str, float]:
        acc = {f: 0.0 for f in FINGERS}
        got = 0
        for _ in range(max(1, samples)):
            snap = self.readers.get(side)
            if not snap:
                break
            raw = snap.snapshot().get("bends_raw", {})
            if not raw:
                time.sleep(0.02)
                continue
            for f in FINGERS:
                acc[f] += float(raw.get(f, 0.0))
            got += 1
            time.sleep(0.02)
        if got == 0:
            return {f: 0.0 for f in FINGERS}
        return {f: round(acc[f] / got, 4) for f in FINGERS}

    def capture_pose(self, side: str, pose: str, samples: int = 25) -> Dict[str, Any]:
        reader = self.readers.get(side)
        if not reader or not reader.snapshot().get("online"):
            return {
                "calibration": self.get_calibration(side),
                "averaged_raw": {f: 0.0 for f in FINGERS},
                "quality": {"ok": False, "issues": [glove_offline_message(side)], "per_finger": {}, "grip": 0.0},
            }
        raw_avg = self.average_raw_bends(side, samples)
        cal = self.get_calibration(side)
        key = "open_raw" if pose == "open" else "closed_raw"
        for f in FINGERS:
            cal.setdefault("fingers", {}).setdefault(f, {})
            cal["fingers"][f][key] = float(raw_avg.get(f, 0.0)) * 255.0
        cal.setdefault("capture", {})["open" if pose == "open" else "closed"] = True
        if pose == "closed":
            cal = compute_gains_from_open_closed(cal)
        self.set_calibration(side, cal)
        quality = evaluate_calibration_quality(cal, raw_avg)
        with self._cal_lock:
            self._pending_capture.setdefault(side, {})[pose] = [raw_avg]
        return {"calibration": cal, "averaged_raw": raw_avg, "quality": quality}

    def get_calibration_status(self, side: str) -> Dict[str, Any]:
        snap = self.readers.get(side)
        raw = snap.snapshot().get("bends_raw", {}) if snap else {}
        cal = self.get_calibration(side)
        quality = evaluate_calibration_quality(cal, raw)
        capture = cal.get("capture", {})
        has_open = bool(capture.get("open"))
        has_closed = bool(capture.get("closed"))
        has_imu = cal.get("imu_ref") is not None
        saved_path = CAL_DIR / f"tactile_glove_{side}.json"
        saved = saved_path.exists() and has_open and has_closed
        return {
            "side": side,
            "calibration": cal,
            "quality": quality,
            "steps": {
                "open_captured": has_open,
                "closed_captured": has_closed,
                "ready": has_open and has_closed,
                "saved": saved,
            },
            "imu_ref": cal.get("imu_ref"),
            "imu_zeroed": has_imu,
        }

    def stop(self) -> None:
        self._watch_running = False
        if self._recorder.status()["recording"]:
            self._recorder.stop()
        for _, r in self._reader_items():
            r.stop()


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<script type="importmap">{"imports":{"three":"/assets/three.module.js"}}</script>
<title>Robotics Center · Tactile Glove Demo</title>
<style>
:root{--bg:#070d1a;--card:rgba(12,22,42,.82);--line:rgba(90,140,255,.22);--accent:#e63946;--cyan:#4cc9f0;--text:#e8eef8;--muted:#8ba3c7}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Segoe UI,system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh;overflow-x:hidden}
.bg-grid{position:fixed;inset:0;background:linear-gradient(rgba(76,201,240,.04) 1px,transparent 1px),linear-gradient(90deg,rgba(76,201,240,.04) 1px,transparent 1px);background-size:48px 48px;pointer-events:none}
.hero{position:relative;padding:24px 32px 12px;border-bottom:1px solid var(--line);background:linear-gradient(180deg,rgba(230,57,70,.12),transparent)}
.brand{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
.logo{width:48px;height:48px;border-radius:10px;background:linear-gradient(135deg,var(--accent),#1d3557);display:flex;align-items:center;justify-content:center;font-weight:800;font-size:14px}
h1{font-size:1.5rem;letter-spacing:.04em}
.sub{color:var(--muted);font-size:.92rem;margin-top:4px}
.title-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.live{display:inline-flex;align-items:center;gap:8px;background:rgba(230,57,70,.15);border:1px solid rgba(230,57,70,.4);padding:4px 12px;border-radius:999px;font-size:.78rem;flex-shrink:0}
.live i{width:8px;height:8px;border-radius:50%;background:var(--accent);animation:pulse 1.2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.toolbar{display:flex;gap:12px;align-items:center;padding:12px 32px;flex-wrap:wrap}
.sections{display:flex;flex-direction:column;gap:16px;padding:0 32px 16px}
.cal-pair{display:grid;grid-template-columns:1fr 1fr;gap:16px;align-items:stretch}
@media(max-width:960px){.cal-pair{grid-template-columns:1fr}}
.panel-section{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:18px}
.panel-section h2{font-size:1rem;margin-bottom:10px}
.panel-section>.sub{margin:4px 0 8px}
.panel-row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:10px 0}
.cal-pair .panel-section{display:flex;flex-direction:column;padding:14px}
.cal-pair h2{font-size:.95rem;margin-bottom:6px}
.cal-pair>.panel-section>.sub,.cal-pair .panel-section>.sub{margin:2px 0 6px;font-size:.85rem}
.cal-pair .cal-steps{margin:6px 0 8px;gap:6px}
.cal-pair .cal-step{min-width:0;padding:6px 8px;font-size:.72rem}
.cal-pair .cal-wizard ol{margin:0 0 8px 16px;font-size:.82rem;line-height:1.4}
.cal-pair .panel-row{margin:6px 0;gap:6px}
.cal-pair .panel-row button,.cal-pair .panel-row select{font-size:.82rem;padding:6px 10px}
.cal-pair #cal-side,.cal-pair #pose-cal-side{min-width:0;max-width:110px}
.cal-pair .cal-status{margin-top:auto;padding:8px 10px;min-height:30px;font-size:.78rem}
.metrics{display:flex;gap:10px;flex-wrap:wrap;padding:0 32px 16px}
.metric{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 16px;min-width:130px}
.metric b{display:block;font-size:1.4rem;color:var(--cyan)}
.metric span{font-size:.75rem;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.lang{margin-left:auto}
select,button{background:rgba(255,255,255,.06);border:1px solid var(--line);color:var(--text);border-radius:8px;padding:8px 12px;cursor:pointer}
select.rec-select,#rec-list,#cal-side,#pose-cal-side{min-width:280px;background:#0d1a30;color:#e8eef8;border:1px solid rgba(90,140,255,.45);color-scheme:dark}
select.rec-select option,#rec-list option,#cal-side option,#pose-cal-side option{background:#0d1a30;color:#e8eef8}
select.rec-select option:checked,#rec-list option:checked{background:#1a3a5c;color:#fff}
#lang{background:#0d1a30;color:#e8eef8;border:1px solid rgba(90,140,255,.45);color-scheme:dark}
#lang option{background:#0d1a30;color:#e8eef8}
#lang option:checked{background:#1a3a5c;color:#fff}
button.primary{background:linear-gradient(135deg,var(--accent),#c1121f);border:none}
main{padding:0 32px 32px;display:grid;gap:20px}
.retarget-panel{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:18px}
.cal-steps{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0 16px}
.cal-step{flex:1;min-width:140px;padding:10px 12px;border-radius:10px;border:1px solid var(--line);font-size:.78rem;color:var(--muted)}
.cal-step.done{border-color:rgba(76,201,240,.5);color:var(--cyan);background:rgba(76,201,240,.08)}
.cal-step.active{border-color:rgba(230,57,70,.5);color:#ffb4b4;background:rgba(230,57,70,.08)}
.cal-wizard ol{margin:0 0 14px 18px;color:var(--muted);font-size:.85rem;line-height:1.6}
.cal-status{margin-top:12px;padding:10px 12px;border-radius:8px;background:rgba(0,0,0,.25);font-size:.82rem;color:var(--muted);min-height:38px}
.cal-status.ok{color:#7dffb2}
.cal-status.warn{color:#ffcc80}
.retarget-panel[hidden]{display:none}
.rec-row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:10px 0}
.rec-status{font-size:.82rem;color:var(--muted);min-height:20px}
.rec-spinner{display:inline-flex;align-items:center;gap:8px;color:var(--cyan);font-size:.82rem}
.rec-spinner[hidden]{display:none}
.rec-spinner-ring{width:15px;height:15px;border:2px solid rgba(76,201,240,.25);border-top-color:var(--cyan);border-radius:50%;animation:rec-spin .7s linear infinite;flex-shrink:0}
@keyframes rec-spin{to{transform:rotate(360deg)}}
.rec-panel.collapsed .rec-body{display:none}
.rec-panel-head{display:flex;justify-content:space-between;align-items:flex-start;gap:12px;cursor:pointer;user-select:none}
.rec-panel-head h2{margin-bottom:4px}
.rec-panel-head .sub{margin:0}
.rec-chevron{font-size:1rem;color:var(--muted);line-height:1.4;padding:2px 4px;flex-shrink:0}
.rec-scrub{width:100%;margin:8px 0}
.live.recording i{background:#ff4444;animation:pulse .6s infinite}
.live.playback i{background:#ffd166}
.bar-raw{position:absolute;inset:0;background:rgba(255,255,255,.12);border-radius:6px}
.bar{position:relative}
.grid-hands{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:20px}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:18px;box-shadow:0 0 40px rgba(76,201,240,.06)}
.card h2{font-size:1rem;margin-bottom:12px;display:flex;justify-content:space-between;align-items:center}
.badge{font-size:.72rem;padding:3px 8px;border-radius:6px;background:rgba(76,201,240,.15);color:var(--cyan)}
.bars{display:grid;gap:8px;margin:12px 0}
.bar-row{display:grid;grid-template-columns:70px 1fr 42px;gap:8px;align-items:center;font-size:.82rem}
.bar{height:10px;background:rgba(255,255,255,.08);border-radius:6px;overflow:hidden}
.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--cyan),var(--accent));border-radius:6px;transition:width .08s}
.heat{display:grid;grid-template-columns:repeat(16,1fr);gap:2px;margin:12px 0}
.heat div{aspect-ratio:1;border-radius:2px;background:#0a1628;transition:background .1s}
.canvas3d{width:100%;height:300px;border-radius:12px;background:radial-gradient(ellipse at center,#12203a,#070d1a);border:1px solid var(--line)}
.hand3d-status{font-size:.72rem;color:var(--muted);min-height:1.1em;margin-top:4px}
.hand3d-status.ok{color:var(--cyan)}
.hand3d-status.err{color:#ffcc80}
.pressure-row{display:flex;gap:8px;flex-wrap:wrap;font-size:.78rem;color:var(--muted)}
.retarget-pre{max-height:180px;overflow:auto;background:#050a14;padding:12px;border-radius:8px;font-size:.72rem;margin-top:8px;display:none}
.footer{text-align:center;padding:24px;color:var(--muted);font-size:.8rem}
</style>
</head>
<body>
<div class="bg-grid"></div>
<header class="hero">
  <div class="brand">
    <div class="logo">RC</div>
    <div>
      <div class="title-row">
        <h1 data-i18n="title">Robotics Center · Tactile Glove Demo</h1>
        <div class="live"><i></i><span data-i18n="live">LIVE</span></div>
      </div>
      <div class="sub" data-i18n="subtitle">Tactile Glove · Live Telemetry · Offline Local Demo</div>
    </div>
    <div class="lang"><select id="lang"><option value="zh">中文</option><option value="en">English</option></select></div>
  </div>
</header>
<div class="metrics" id="metrics"></div>
<div class="sections">
<div class="cal-pair">
<section class="panel-section cal-wizard">
  <h2 data-i18n="pose_cal_title">Hand Pose Calibration (Orientation)</h2>
  <p class="sub" data-i18n="pose_cal_desc">Zero the 3D hand to your demo pose before recording.</p>
  <div class="cal-steps">
    <div class="cal-step" id="step-imu-neutral" data-i18n="step_imu_neutral">1 · Neutral pose</div>
    <div class="cal-step" id="step-imu-zero" data-i18n="step_imu_zero">2 · Zero orientation</div>
    <div class="cal-step" id="step-imu-save" data-i18n="step_imu_save">3 · Save</div>
  </div>
  <ol>
    <li data-i18n="pose_wiz1">Neutral pose — palm toward front, fingers up → Click Zero Orientation</li>
  </ol>
  <div class="panel-row">
    <select id="pose-cal-side" onchange="loadPoseCalPanel()"><option value="left" data-i18n="left">Left</option><option value="right" data-i18n="right">Right</option></select>
    <button onclick="zeroOrientation(document.getElementById('pose-cal-side').value)" data-i18n="imu_zero">Zero Orientation</button>
    <button onclick="resetOrientation(document.getElementById('pose-cal-side').value)" data-i18n="imu_reset">Reset orientation</button>
    <button onclick="savePoseCal()" data-i18n="save">Save to File</button>
  </div>
  <div class="cal-status" id="pose-cal-status" data-i18n="pose_cal_hint">Select a hand and zero orientation before recording.</div>
</section>
<section class="panel-section cal-wizard">
  <h2 data-i18n="cal_title">Finger Bend Calibration</h2>
  <p class="sub" data-i18n="cal_desc">Calibrate each hand so open ≈ 0% and fist ≈ 100%.</p>
  <div class="cal-steps">
    <div class="cal-step" id="step-open" data-i18n="step_open">1 · Capture open</div>
    <div class="cal-step" id="step-closed" data-i18n="step_closed">2 · Capture fist</div>
    <div class="cal-step" id="step-save" data-i18n="step_save">3 · Save</div>
  </div>
  <ol>
    <li data-i18n="wiz1">Hold hand flat, fingers straight → Click Capture Open</li>
    <li data-i18n="wiz2">Full fist → Click Capture Closed</li>
  </ol>
  <div class="panel-row">
    <select id="cal-side" onchange="loadCalPanel()"><option value="left" data-i18n="left">Left</option><option value="right" data-i18n="right">Right</option></select>
    <button onclick="capture('open')" data-i18n="cap_open">Capture Open</button>
    <button onclick="capture('closed')" data-i18n="cap_closed">Capture Closed</button>
    <button onclick="resetCal()" data-i18n="reset">Reset</button>
    <button onclick="saveCal()" data-i18n="save">Save to File</button>
  </div>
  <div class="cal-status" id="cal-status" data-i18n="cal_hint">Select a hand and run the wizard.</div>
</section>
</div>
<section class="panel-section rec-panel collapsed" id="rec-panel">
  <div class="rec-panel-head" onclick="toggleRecPanel()">
    <div>
      <h2 data-i18n="rec_title">Demo Recording</h2>
      <p class="sub" data-i18n="rec_desc">Record glove motion, save locally, replay anytime (no USB needed).</p>
    </div>
    <span class="rec-chevron" id="rec-chevron" aria-hidden="true">▸</span>
  </div>
  <div class="rec-body">
  <div class="rec-row">
    <input id="rec-label" type="text" placeholder="Label (optional)" style="min-width:160px;padding:8px 12px;border-radius:8px;border:1px solid var(--line);background:rgba(255,255,255,.06);color:var(--text)"/>
    <button id="btn-rec-start" onclick="startRecording()" data-i18n="rec_start">● Record</button>
    <button id="btn-rec-stop" onclick="stopRecording()" disabled data-i18n="rec_stop">■ Stop &amp; Save</button>
    <span class="rec-status" id="rec-status"></span>
  </div>
  <div class="rec-row">
    <select id="rec-list" class="rec-select" onchange="onRecordingSelected()"><option value="" data-i18n="rec_pick">— Select or upload —</option></select>
    <button id="btn-play-pause" onclick="togglePlayPause()" data-i18n="rec_play">Play</button>
    <button onclick="stopPlayback()" data-i18n="rec_live">Back to Live</button>
    <button onclick="deleteRecording()" data-i18n="rec_delete">Delete</button>
    <span class="rec-spinner" id="rec-spinner" hidden><span class="rec-spinner-ring"></span><span id="rec-spinner-text"></span></span>
  </div>
  <div class="rec-row">
    <button type="button" onclick="openRecordingsFolder()" data-i18n="rec_open_folder">Open recordings folder</button>
    <button type="button" onclick="document.getElementById('rec-upload').click()" data-i18n="rec_upload">Upload recording</button>
    <input id="rec-upload" type="file" accept=".json,application/json" hidden onchange="uploadRecording(this)"/>
  </div>
  <input class="rec-scrub" id="scrub" type="range" min="0" max="1000" value="0" disabled oninput="scrubPlayback(this.value)"/>
  <div class="rec-status" id="scrub-label">0:00 / 0:00 · 1× speed</div>
  </div>
</section>
</div>
<main>
  <div class="grid-hands" id="hands"></div>
  <div class="retarget-panel" hidden>
    <h2 data-i18n="retarget">Retarget Pose (both hands)</h2>
    <button onclick="toggleRetarget()" data-i18n="expand">Expand JSON</button>
    <button onclick="copyRetarget()" data-i18n="copy">Copy JSON</button>
    <pre class="retarget-pre" id="retarget-json"></pre>
  </div>
</main>
<footer class="footer">ROBOTICSCENTER.AI · Local demo · no network required</footer>
<script>
const I18N={
  zh:{title:"Robotics Center · 触觉手套演示",subtitle:"Tactile Glove · 实时遥测 · 本地离线演示",live:"直播中",
    pose_cal_title:"手部姿态校准（方向）",pose_cal_desc:"录制前将 3D 手部归零到演示姿势。",
    step_imu_neutral:"1 · 中性姿势",step_imu_zero:"2 · 方向归零",step_imu_save:"3 · 保存",
    pose_wiz1:"中性姿势 — 掌心朝前、手指向上 → 点击方向归零",
    pose_cal_hint:"选择手并在录制前归零方向。",pose_cal_ok:"方向已校准",
    glove_offline:"手套离线 — 请检查连接",
    cal_title:"手指弯曲校准",cal_desc:"校准每只手：张开≈0%，握拳≈100%。",left:"左手",right:"右手",
    cap_open:"采集张开",cap_closed:"采集握拳",reset:"重置",save:"保存到文件",
    step_open:"1 · 采集张开",step_closed:"2 · 采集握拳",step_save:"3 · 保存",
    wiz1:"手掌平放、五指伸直 → 点击采集张开",wiz2:"握拳 → 点击采集握拳",
    cal_hint:"选择手并开始校准向导。",cal_ok:"校准良好",cal_warn:"需要检查",
    rec_title:"演示录制",rec_desc:"录制手套动作，本地保存，随时回放（无需 USB）。",rec_start:"● 开始录制",rec_stop:"■ 停止并保存",
    rec_pick:"— 选择或上传 —",rec_play:"播放",rec_pause:"暂停",rec_live:"返回实时",rec_delete:"删除",
    rec_open_folder:"打开录制文件夹",rec_upload:"上传录制",rec_loading:"加载录制中…",rec_uploading:"上传中…",
    rec_saved:"已保存",rec_recording:"录制中",playback:"回放",live_lbl:"直播中",rec_uploaded:"已上传",
    rec_already_saved:"已在库中",
    retarget:"Retarget Pose（双手通用姿态）",expand:"展开 JSON",copy:"复制 JSON",
    online:"在线手套",fps:"平均 FPS",grip:"平均闭合度",peak:"峰值压力",
    bends:"手指弯曲",heat:"压力热力图",hand3d:"3D 手部姿态",thumb:"拇指",index:"食指",middle:"中指",ring:"无名指",little:"小指",
    imu_zero:"方向归零",imu_reset:"重置方向",
    imu_zeroed:"方向已归零 ✓",imu_reset_done:"方向已重置",
    imu_no_frame:"该帧无 IMU 数据"},
  en:{title:"Robotics Center · Tactile Glove Demo",subtitle:"Tactile Glove · Live Telemetry · Offline Local Demo",live:"LIVE",
    pose_cal_title:"Hand Pose Calibration (Orientation)",pose_cal_desc:"Zero the 3D hand to your demo pose before recording.",
    step_imu_neutral:"1 · Neutral pose",step_imu_zero:"2 · Zero orientation",step_imu_save:"3 · Save",
    pose_wiz1:"Neutral pose — palm toward front, fingers up → Click Zero Orientation",
    pose_cal_hint:"Select a hand and zero orientation before recording.",pose_cal_ok:"Orientation calibrated",
    glove_offline:"glove offline — check Connection",
    cal_title:"Finger Bend Calibration",cal_desc:"Calibrate each hand so open ≈ 0% and fist ≈ 100%.",left:"Left",right:"Right",
    cap_open:"Capture Open",cap_closed:"Capture Closed",reset:"Reset",save:"Save to File",
    step_open:"1 · Capture open",step_closed:"2 · Capture fist",step_save:"3 · Save",
    wiz1:"Hold hand flat, fingers straight → Click Capture Open",wiz2:"Full fist → Click Capture Closed",
    cal_hint:"Select a hand and run the wizard.",cal_ok:"Calibration looks good",cal_warn:"Needs attention",
    rec_title:"Demo Recording",rec_desc:"Record glove motion, save locally, replay anytime (no USB needed).",rec_start:"● Record",rec_stop:"■ Stop & Save",
    rec_pick:"— Select or upload —",rec_play:"Play",rec_pause:"Pause",rec_live:"Back to Live",rec_delete:"Delete",
    rec_open_folder:"Open recordings folder",rec_upload:"Upload recording",rec_loading:"Loading recording…",rec_uploading:"Uploading…",
    rec_saved:"Saved",rec_recording:"Recording",playback:"PLAYBACK",live_lbl:"LIVE",rec_uploaded:"Uploaded",
    rec_already_saved:"already in library",
    retarget:"Retarget Pose (both hands)",expand:"Expand JSON",copy:"Copy JSON",
    online:"Online Gloves",fps:"Avg FPS",grip:"Avg Grip",peak:"Peak Pressure",
    bends:"Finger Bends",heat:"Pressure Heatmap",hand3d:"3D Hand Pose",thumb:"Thumb",index:"Index",middle:"Middle",ring:"Ring",little:"Little",
    imu_zero:"Zero Orientation",imu_reset:"Reset",
    imu_zeroed:"Orientation zeroed ✓",imu_reset_done:"Orientation reset",
    imu_no_frame:"No IMU in this frame"}
};
let lang=localStorage.getItem('tg_lang')||'en';
document.getElementById('lang').value=lang;
function t(k){return (I18N[lang]||I18N.en)[k]||k;}
function applyI18n(){document.querySelectorAll('[data-i18n]').forEach(el=>{const k=el.dataset.i18n;el.textContent=t(k);});}
document.getElementById('lang').onchange=e=>{lang=e.target.value;localStorage.setItem('tg_lang',lang);applyI18n();renderHands(lastState);};
applyI18n();

const FINGERS=['thumb','index','middle','ring','little'];
let lastState={hands:{}};
let retargetOpen=false;
let calCache={calibration:null,quality:null,steps:null};
let poseCalCache={imu_zeroed:false,imu_ref:null};
let sessionPose={left:{zeroed:false,saved:false},right:{zeroed:false,saved:false}};
let sessionBend={left:{open:false,closed:false,saved:false},right:{open:false,closed:false,saved:false}};
let lastHandFrame={left:-1,right:-1};
const LIVE_POLL_MS=33;
let playback=null, playbackIdx=0, playbackOn=false, playbackPlaying=false;
let playbackAnchorT=0, playbackWall0=0, recordingsFolder='';
let recordingPoll=null, isRecording=false, playbackId='';
let playbackCache={};
let replayImuRef={left:null,right:null};

function setRecStatus(text){
  const el=document.getElementById('rec-status');
  if(el) el.textContent=text||'';
}

function showSpinner(text){
  const sp=document.getElementById('rec-spinner');
  const tx=document.getElementById('rec-spinner-text');
  if(tx) tx.textContent=text||'';
  if(sp) sp.hidden=false;
}

function hideSpinner(){
  const sp=document.getElementById('rec-spinner');
  if(sp) sp.hidden=true;
}

function nextFrame(){
  return new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)));
}

function xhrUploadFile(url, file){
  return new Promise((resolve,reject)=>{
    const xhr=new XMLHttpRequest();
    xhr.open('POST', url);
    xhr.setRequestHeader('Content-Type','application/json');
    xhr.setRequestHeader('X-Filename', encodeURIComponent(file.name));
    xhr.onload=()=>{
      try{
        const data=JSON.parse(xhr.responseText||'{}');
        if(xhr.status<200||xhr.status>=300||!data.ok){reject(new Error(data.error||('HTTP '+xhr.status))); return;}
        resolve(data);
      }catch(err){reject(err);}
    };
    xhr.onerror=()=>reject(new Error('network error'));
    xhr.send(file);
  });
}

function appendRecordingOption(rec){
  const sel=document.getElementById('rec-list');
  if(!sel||!rec||!rec.id) return;
  let opt=[...sel.options].find(o=>o.value===rec.id);
  if(!opt){
    opt=document.createElement('option');
    opt.value=rec.id;
    sel.insertBefore(opt, sel.options[1]||null);
  }
  opt.textContent=(rec.label||rec.id)+' · '+formatTime(rec.duration_s||0)+' · '+(rec.frame_count||0)+' frames';
  sel.value=rec.id;
}

function removeRecordingOption(id){
  const sel=document.getElementById('rec-list');
  if(!sel||!id) return;
  const opt=[...sel.options].find(o=>o.value===id);
  if(opt) opt.remove();
  sel.value='';
}

function heatColor(v,peak){
  const p=Math.max(peak,1); const x=Math.pow(Math.min(1,v/p),0.45);
  const r=Math.round(20+235*x), g=Math.round(40+180*Math.min(1,x*1.2)), b=Math.round(120-100*x);
  return `rgb(${r},${g},${b})`;
}

// --- IMU quaternion (per JQ spec 5.5.1: 16 bytes = 4x float32 LE, w,x,y,z) ---
function imuToQuat(imu){
  if(!imu||imu.length<16) return null;
  const buf=new ArrayBuffer(16), u8=new Uint8Array(buf);
  for(let i=0;i<16;i++){ u8[i]=(((imu[i]|0)%256)+256)%256; }
  const dv=new DataView(buf);
  const w=dv.getFloat32(0,true), x=dv.getFloat32(4,true), y=dv.getFloat32(8,true), z=dv.getFloat32(12,true);
  const n=Math.hypot(w,x,y,z);
  if(!isFinite(n)||n<1e-6) return null;
  return {w:w/n,x:x/n,y:y/n,z:z/n};
}
// The 3D hand itself is rendered with Three.js in a separate ES module
// (window.Hand3D). imuToQuat() above decodes the IMU bytes; renderHands()
// forwards the quaternion + finger bends to window.Hand3D.update(side,...).

function renderHands(state){
  if(!_handCardsReady) buildHandCards();
  ['left','right'].forEach(side=>{
    const d=state.hands[side]||{online:false,bends:{},matrix_16x16:[],fps:0,frame_count:0,port:'?'};
    const badge=document.getElementById(`badge-${side}`);
    if(badge) badge.textContent=`${d.port||'?'} · ${d.online?'ON':'OFF'} · ${d.fps||0} FPS`;
    const fc=d.frame_count|0;
    const unchanged=d.online && fc===lastHandFrame[side];
    if(unchanged) return;
    lastHandFrame[side]=fc;
    const bends=d.bends||{};
    FINGERS.forEach(f=>{
      const v=Math.max(0,Math.min(1,bends[f]||0));
      const bi=document.getElementById(`bari-${side}-${f}`); if(bi) bi.style.width=(v*100)+'%';
      const bv=document.getElementById(`barv-${side}-${f}`); if(bv) bv.textContent=Math.round(v*100)+'%';
    });
    const mat=d.matrix_16x16||[];
    let peak=1; mat.forEach(r=>r.forEach(v=>{if(v>peak)peak=v;}));
    for(let r=0;r<16;r++)for(let c=0;c<16;c++){
      const cell=document.getElementById(`hc-${side}-${r*16+c}`);
      const matCol=15-c;
      if(cell) cell.style.background=heatColor((mat[r]||[])[matCol]||0,peak);
    }
    if(window.Hand3D){
      if(!window.Hand3D.has(side)) window.Hand3D.init(document.getElementById(`c3d-${side}`), side);
      window.Hand3D.update(side, imuToQuat(d.imu||[]), bends, effectiveImuRef(side, state));
    }
  });
}

function asQuat(v){
  if(!v) return null;
  if(Array.isArray(v)) return v.length>=4?{w:v[0],x:v[1],y:v[2],z:v[3]}:null;
  return (typeof v.w==='number')?v:null;
}
// Effective orientation reference per side.
// Replay priority: recording's baked imu_ref -> user replay-time ref -> identity.
// Live: the active per-side calibration ref from the backend.
function effectiveImuRef(side, state){
  if(playbackOn){
    if(playback && playback.imu_ref && playback.imu_ref[side]) return asQuat(playback.imu_ref[side]);
    if(replayImuRef[side]) return asQuat(replayImuRef[side]);
    return null;
  }
  return (state && state.imu_ref) ? asQuat(state.imu_ref[side]) : null;
}
function rerenderCurrentFrame(){
  if(playback && playback.frames && playback.frames.length){
    applyPlaybackTime((playback.frames[playbackIdx]||playback.frames[0]).t||0);
  }
}
async function zeroOrientation(side){
  if(playbackOn){
    const fr=playback && playback.frames && playback.frames[playbackIdx];
    const h=fr && fr.hands && fr.hands[side];
    const q=h && imuToQuat(h.imu||[]);
    if(!q){ setPoseCalStatus(t('imu_no_frame'), 'warn'); return; }
    replayImuRef[side]=q;
    sessionPose[side].zeroed=true;
    sessionPose[side].saved=false;
    setPoseCalStatus(t('imu_zeroed'), 'ok');
    rerenderCurrentFrame();
  }else{
    setPoseCalStatus('…');
    try{
      const r=await fetch('/api/imu/calibrate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side})});
      const j=await r.json();
      const offlineMsg=t(side)+' '+t('glove_offline');
      const msg=j.ok? t('imu_zeroed') : (j.offline? offlineMsg : (j.message||offlineMsg));
      if(j.ok){
        sessionPose[side].zeroed=true;
        sessionPose[side].saved=false;
        if(document.getElementById('pose-cal-side').value===side) await loadPoseCalPanel();
        else setPoseCalStatus(msg, 'ok');
      }
      else setPoseCalStatus(msg, 'warn');
    }catch(e){ setPoseCalStatus(t(side)+' '+t('glove_offline'), 'warn'); }
  }
}
async function resetOrientation(side){
  if(playbackOn){
    replayImuRef[side]=null;
    sessionPose[side]={zeroed:false,saved:false};
    setPoseCalStatus(t('imu_reset_done'));
    rerenderCurrentFrame();
  }else{
    try{ await fetch('/api/imu/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side})}); }catch(e){}
    sessionPose[side]={zeroed:false,saved:false};
    if(document.getElementById('pose-cal-side').value===side) await loadPoseCalPanel();
    else setPoseCalStatus(t('imu_reset_done'));
  }
}

let _handCardsReady=false;
function buildHandCards(){
  const root=document.getElementById('hands');
  root.innerHTML='';
  ['left','right'].forEach(side=>{
    const card=document.createElement('div'); card.className='card';
    card.innerHTML=`<h2><span data-i18n="${side}">${t(side)}</span> <span class="badge" id="badge-${side}"></span></h2>
      <div data-i18n="bends">${t('bends')}</div><div class="bars" id="bars-${side}"></div>
      <div data-i18n="heat">${t('heat')}</div><div class="heat" id="heat-${side}"></div>
      <div data-i18n="hand3d">${t('hand3d')}</div><canvas class="canvas3d" id="c3d-${side}"></canvas>
      <div class="hand3d-status" id="hand3d-status-${side}">Loading hand model…</div>`;
    root.appendChild(card);
    const bars=card.querySelector(`#bars-${side}`);
    FINGERS.forEach(f=>{
      const row=document.createElement('div'); row.className='bar-row';
      row.innerHTML=`<span data-i18n="${f}">${t(f)}</span><div class="bar"><i id="bari-${side}-${f}" style="width:0%"></i></div><span id="barv-${side}-${f}">0%</span>`;
      bars.appendChild(row);
    });
    const heat=card.querySelector(`#heat-${side}`);
    for(let i=0;i<256;i++){ const cell=document.createElement('div'); cell.id=`hc-${side}-${i}`; heat.appendChild(cell); }
    if(window.Hand3D) window.Hand3D.init(card.querySelector(`#c3d-${side}`), side);
  });
  _handCardsReady=true;
}

function renderMetrics(s){
  const m=document.getElementById('metrics');
  const x=s.summary||{};
  m.innerHTML=`<div class="metric"><b>${x.online_gloves||0}</b><span>${t('online')}</span></div>
    <div class="metric"><b>${x.avg_fps||0}</b><span>${t('fps')}</span></div>
    <div class="metric"><b>${Math.round((x.avg_grip||0)*100)}%</b><span>${t('grip')}</span></div>
    <div class="metric"><b>${x.peak_pressure||0}</b><span>${t('peak')}</span></div>`;
}

function renderCalSteps(side){
  const s=sessionBend[side]||{open:false,closed:false,saved:false};
  const hasOpen=!!s.open, hasClosed=!!s.closed;
  document.getElementById('step-open').className='cal-step'+(hasOpen?' done':(!hasClosed?' active':''));
  document.getElementById('step-closed').className='cal-step'+(hasClosed?' done':(hasOpen&&!hasClosed?' active':''));
  document.getElementById('step-save').className='cal-step'+(s.saved?' done':(hasOpen&&hasClosed?' active':''));
}

function renderPoseCalSteps(side){
  const s=sessionPose[side]||{zeroed:false,saved:false};
  const elN=document.getElementById('step-imu-neutral');
  const elZ=document.getElementById('step-imu-zero');
  const elS=document.getElementById('step-imu-save');
  if(!elN||!elZ||!elS) return;
  elN.className='cal-step'+(s.zeroed?' done':(!s.zeroed?' active':''));
  elZ.className='cal-step'+(s.zeroed?' done':(s.zeroed?'':' active'));
  elS.className='cal-step'+(s.saved?' done':(s.zeroed&&!s.saved?' active':''));
}

function setPoseCalStatus(msg, kind){
  const el=document.getElementById('pose-cal-status');
  if(!el) return;
  el.textContent=msg;
  el.className='cal-status'+(kind?(' '+kind):'');
}

async function loadPoseCalPanel(){
  const side=document.getElementById('pose-cal-side').value;
  const r=await fetch('/api/calibration?side='+side);
  poseCalCache=await r.json();
  renderPoseCalSteps(side);
  if(sessionPose[side].zeroed){
    setPoseCalStatus(t('pose_cal_ok')+' · '+t(side), 'ok');
  }else{
    setPoseCalStatus(t('pose_cal_hint'));
  }
}

async function savePoseCal(){
  const side=document.getElementById('pose-cal-side').value;
  if(!sessionPose[side].zeroed){
    setPoseCalStatus(t('pose_cal_hint'), 'warn');
    return;
  }
  const r=await fetch('/api/calibration/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side,calibration:poseCalCache.calibration})});
  const j=await r.json();
  if(j.ok){
    sessionPose[side].saved=true;
    renderPoseCalSteps(side);
    setPoseCalStatus(t('pose_cal_ok')+' · tactile_glove_'+side+'.json', 'ok');
  }else{
    setPoseCalStatus('Save failed', 'warn');
  }
}

function setCalStatus(msg, kind){
  const el=document.getElementById('cal-status');
  el.textContent=msg;
  el.className='cal-status'+(kind?(' '+kind):'');
}

async function loadCalPanel(){
  const side=document.getElementById('cal-side').value;
  const r=await fetch('/api/calibration?side='+side);
  calCache=await r.json();
  renderCalSteps(side);
  const q=calCache.quality||{};
  const s=sessionBend[side];
  if(q.issues&&q.issues.length)setCalStatus(q.issues.join(' · '), 'warn');
  else if(s.open&&s.closed)setCalStatus(t('cal_ok')+' · grip '+Math.round((q.grip||0)*100)+'%', 'ok');
  else setCalStatus(t('cal_hint'));
}

function formatTime(s){
  s=Math.max(0,Number(s)||0);
  const m=Math.floor(s/60), sec=Math.floor(s%60);
  return m+':'+String(sec).padStart(2,'0');
}

function updateLiveBadge(){
  const el=document.querySelector('.live');
  const txt=el.querySelector('span');
  el.classList.remove('recording','playback');
  if(playbackOn){el.classList.add('playback'); txt.textContent=t('playback');}
  else if(isRecording){el.classList.add('recording'); txt.textContent=t('rec_recording');}
  else txt.textContent=t('live_lbl');
}

function showState(state){
  lastState=state;
  renderMetrics(state);
  renderHands(state);
}

function frameAtTime(frames, t){
  let i=0;
  while(i+1<frames.length && frames[i+1].t<=t) i++;
  return {idx:i, frame:frames[i]};
}

function applyPlaybackTime(t){
  if(!playback||!playback.frames.length) return;
  t=Math.max(0,Math.min(playback.duration_s,t));
  const {idx,frame}=frameAtTime(playback.frames,t);
  playbackIdx=idx;
  showState({hands:frame.hands,summary:frame.summary});
  document.getElementById('scrub').value=Math.round((t/Math.max(0.001,playback.duration_s))*1000);
  document.getElementById('scrub-label').textContent=formatTime(t)+' / '+formatTime(playback.duration_s)+' · 1× speed';
}

function playbackLoop(){
  if(!playbackOn||!playbackPlaying||!playback) return;
  const t=playbackAnchorT+(performance.now()-playbackWall0)/1000;
  if(t>=playback.duration_s){
    playbackAnchorT=playback.duration_s;
    applyPlaybackTime(playback.duration_s);
    playbackPlaying=false;
    updatePlayPauseButton();
    updateLiveBadge();
    return;
  }
  applyPlaybackTime(t);
  requestAnimationFrame(playbackLoop);
}

function scrubPlayback(v){
  if(!playback) return;
  playbackPlaying=false;
  playbackAnchorT=(Number(v)/1000)*playback.duration_s;
  applyPlaybackTime(playbackAnchorT);
  updatePlayPauseButton();
}

function updatePlayPauseButton(){
  const btn=document.getElementById('btn-play-pause');
  if(!btn) return;
  btn.textContent=(playbackOn&&playbackPlaying)?t('rec_pause'):t('rec_play');
}

let pollBusy=false;
function schedulePoll(){ requestAnimationFrame(()=>setTimeout(poll,LIVE_POLL_MS)); }
async function poll(){
  if(playbackOn) return schedulePoll();
  if(pollBusy) return schedulePoll();
  pollBusy=true;
  try{
    const r=await fetch('/api/state'); lastState=await r.json();
    showState(lastState);
    if(retargetOpen){
      const rp=await fetch('/api/retarget_pose'); document.getElementById('retarget-json').textContent=JSON.stringify(await rp.json(),null,2);
    }
  }catch(e){}
  pollBusy=false;
  schedulePoll();
}

async function startRecording(){
  if(playbackOn) stopPlayback();
  const label=document.getElementById('rec-label').value.trim();
  const r=await fetch('/api/recording/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({label})});
  const j=await r.json();
  isRecording=true;
  document.getElementById('btn-rec-start').disabled=true;
  document.getElementById('btn-rec-stop').disabled=false;
  document.getElementById('rec-status').textContent=t('rec_recording')+'...';
  updateLiveBadge();
  clearInterval(recordingPoll);
  recordingPoll=setInterval(async()=>{
    const st=await (await fetch('/api/recording/status')).json();
    document.getElementById('rec-status').textContent=t('rec_recording')+': '+st.frame_count+' frames · '+formatTime(st.elapsed_s);
  },500);
}

async function stopRecording(){
  const r=await fetch('/api/recording/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  const j=await r.json();
  clearInterval(recordingPoll);
  isRecording=false;
  document.getElementById('btn-rec-start').disabled=false;
  document.getElementById('btn-rec-stop').disabled=true;
  if(j.ok){
    setRecStatus(t('rec_saved')+': '+j.id+' ('+j.frame_count+' frames)');
    appendRecordingOption(j);
  }else{
    setRecStatus(j.message||'Error');
  }
  updateLiveBadge();
}

async function loadRecordingData(id){
  if(playbackCache[id]) return playbackCache[id];
  const r=await fetch('/api/recordings/'+encodeURIComponent(id));
  if(!r.ok) throw new Error('HTTP '+r.status);
  const data=await r.json();
  playbackCache[id]=data;
  return data;
}

function resetForNewSelection(id){
  playbackPlaying=false;
  playbackOn=false;
  playback=null;
  playbackId='';
  playbackIdx=0;
  playbackAnchorT=0;
  playbackWall0=0;
  const scrub=document.getElementById('scrub');
  if(scrub){scrub.disabled=true; scrub.value=0;}
  document.getElementById('scrub-label').textContent='0:00 / 0:00 · 1× speed';
  if(id) document.getElementById('rec-list').value=id;
  updatePlayPauseButton();
  updateLiveBadge();
  poll();
}

async function startPlayback(id){
  playbackPlaying=false;
  playbackOn=false;
  const cached=!!playbackCache[id];
  let data;
  if(cached){
    data=playbackCache[id];
  }else{
    showSpinner(t('rec_loading'));
    await nextFrame();
    try{
      data=await loadRecordingData(id);
    }finally{
      hideSpinner();
    }
  }
  playback=JSON.parse(JSON.stringify(data));
  playback.duration_s=playback.duration_s||(playback.frames?.length?playback.frames[playback.frames.length-1].t:0);
  playbackId=id;
  playbackOn=true;
  playbackPlaying=true;
  playbackAnchorT=0;
  playbackWall0=performance.now();
  document.getElementById('rec-list').value=id;
  document.getElementById('scrub').disabled=false;
  applyPlaybackTime(0);
  setRecStatus(t('playback')+': '+(playback.label||id));
  updatePlayPauseButton();
  updateLiveBadge();
  playbackLoop();
}

function onRecordingSelected(){
  resetForNewSelection(document.getElementById('rec-list').value);
}

async function togglePlayPause(){
  const id=document.getElementById('rec-list').value;
  if(!id){alert('Select a recording first'); return;}
  if(playbackOn&&playbackId===id){
    if(playbackPlaying){
      playbackAnchorT=Math.min(playback.duration_s, playbackAnchorT+(performance.now()-playbackWall0)/1000);
      playbackPlaying=false;
    }else{
      if(playbackAnchorT>=playback.duration_s-0.05){
        playbackAnchorT=0;
        applyPlaybackTime(0);
      }
      playbackWall0=performance.now();
      playbackPlaying=true;
      playbackLoop();
    }
    updatePlayPauseButton();
    return;
  }
  try{await startPlayback(id);}
  catch(e){alert('Load failed: '+e.message); resetForNewSelection(id);}
}

function stopPlayback(){
  resetForNewSelection(document.getElementById('rec-list').value);
}

async function deleteRecording(){
  const id=document.getElementById('rec-list').value;
  if(!id||!confirm('Delete recording '+id+'?')) return;
  await fetch('/api/recordings/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
  if(playbackOn&&playbackId===id) stopPlayback();
  delete playbackCache[id];
  removeRecordingOption(id);
  setRecStatus('');
}

async function openRecordingsFolder(){
  await fetch('/api/recordings/open-folder',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
}

async function uploadRecording(input){
  const file=input.files&&input.files[0];
  if(!file) return;
  if(playbackOn) stopPlayback();
  showSpinner(t('rec_uploading'));
  await nextFrame();
  try{
    const j=await xhrUploadFile('/api/recordings/upload-file', file);
    delete playbackCache[j.id];
    appendRecordingOption(j);
    const msg=j.already_exists
      ? (t('rec_uploaded')+' · '+j.id+' ('+t('rec_already_saved')+')')
      : (t('rec_uploaded')+': '+j.id+' ('+j.frame_count+' frames)');
    setRecStatus(msg);
  }catch(e){
    setRecStatus('');
    alert('Upload failed: '+e.message);
  }finally{
    hideSpinner();
    input.value='';
  }
}
try{ if(localStorage.getItem('tg_rec_collapsed')==='0'){ const p=document.getElementById('rec-panel'); if(p){ p.classList.remove('collapsed'); const c=document.getElementById('rec-chevron'); if(c) c.textContent='▾'; } } }catch(e){}
poll();
loadCalPanel();
loadPoseCalPanel();
fetch('/api/recordings/folder').then(r=>r.json()).then(j=>{recordingsFolder=j.folder||'';}).catch(()=>{});

async function capture(pose){
  const side=document.getElementById('cal-side').value;
  const labels={open:t('cap_open'),closed:t('cap_closed')};
  setCalStatus((labels[pose]||pose)+'...');
  const r=await fetch('/api/calibration/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side,pose,samples:30})});
  const j=await r.json();
  calCache.calibration=j.calibration;
  calCache.quality=j.quality;
  const st=await (await fetch('/api/calibration?side='+side)).json();
  calCache.steps=st.steps;
  if(pose==='open') sessionBend[side].open=true;
  if(pose==='closed') sessionBend[side].closed=true;
  renderCalSteps(side);
  const issues=(j.quality&&j.quality.issues)||[];
  if(issues.length){setCalStatus(issues.join(' · '),'warn'); return;}
  if(sessionBend[side].open&&sessionBend[side].closed){
    setCalStatus(t('cal_ok')+' · grip '+Math.round(((j.quality&&j.quality.grip)||0)*100)+'%', 'ok');
  }else{
    setCalStatus(t('cal_hint'));
  }
}

async function resetCal(){
  const side=document.getElementById('cal-side').value;
  if(!confirm('Reset calibration for '+side+'?')) return;
  const r=await fetch('/api/calibration/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side})});
  calCache=await r.json();
  sessionBend[side]={open:false,closed:false,saved:false};
  await loadCalPanel();
}

async function saveCal(){
  const side=document.getElementById('cal-side').value;
  const r=await fetch('/api/calibration/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side,calibration:calCache.calibration})});
  await r.json();
  sessionBend[side].saved=true;
  renderCalSteps(side);
  setCalStatus('Saved tactile_glove_'+side+'.json', 'ok');
}
function toggleRecPanel(){
  const panel=document.getElementById('rec-panel');
  if(!panel) return;
  const collapsed=panel.classList.toggle('collapsed');
  const chev=document.getElementById('rec-chevron');
  if(chev) chev.textContent=collapsed?'▸':'▾';
  try{ localStorage.setItem('tg_rec_collapsed', collapsed?'1':'0'); }catch(e){}
}
function toggleRetarget(){retargetOpen=!retargetOpen; const el=document.getElementById('retarget-json'); el.style.display=retargetOpen?'block':'none'; if(retargetOpen) poll();}
async function copyRetarget(){const r=await fetch('/api/retarget_pose'); const j=await r.json(); await navigator.clipboard.writeText(JSON.stringify(j,null,2));}
</script>
<script type="module">
// True 3D hands via Three.js (bundled locally at /assets, no CDN).
// Shroom "LeapMotion Basehand" rig (shroom_hand1.glb): one left-hand model is
// loaded for both sides; the right hand is the same mesh mirrored on X (scale.x=-1),
// exactly as the Shroom app does. Orientation comes from the IMU quaternion;
// finger curl rotates the named Leap finger bones about their local Z axis.
import * as THREE from '/assets/three.module.js';
import { GLTFLoader } from '/assets/GLTFLoader.js';
import { clone as cloneSkinned } from '/assets/utils/SkeletonUtils.js';

const clamp01=v=>Math.max(0,Math.min(1,Number(v)||0));
const REG={};
// Neutral grayscale material (GLB ships no texture); DoubleSide so the mirrored
// right hand (negative X scale flips winding) still lights correctly.
const HAND_MAT=new THREE.MeshStandardMaterial({color:0xc9ccd3, roughness:0.62, metalness:0.03, side:THREE.DoubleSide});

// Shroom Leap rig finger->bone map. thumb uses Finger_0x, index 1x, middle 2x,
// ring 3x, little 4x. Segment 0 = knuckle, 1 = mid, 2 = tip joint.
const FINGER_BONES={
  thumb:['Finger_01','Finger_02'],
  index:['Finger_10','Finger_11','Finger_12'],
  middle:['Finger_20','Finger_21','Finger_22'],
  ring:['Finger_30','Finger_31','Finger_32'],
  little:['Finger_40','Finger_41','Finger_42'],
};
const CURL=Math.PI/2; // Shroom: bone.rotation.z = -PI/2 * value

function clearHandStatus(side){
  const el=document.getElementById(`hand3d-status-${side}`);
  if(el){ el.textContent=''; el.className='hand3d-status'; }
}
function setHandStatus(side, msg, cls){
  const el=document.getElementById(`hand3d-status-${side}`);
  if(el){ el.textContent=msg||''; el.className='hand3d-status'+(cls?(' '+cls):''); }
}

let _templatePromise=null;
function loadTemplate(){
  if(!_templatePromise){
    _templatePromise=new Promise((resolve,reject)=>{
      new GLTFLoader().load('/assets/shroom_hand1.glb', g=>resolve(g.scene), undefined, reject);
    });
  }
  return _templatePromise;
}

function buildModel(side, template){
  const hand=cloneSkinned(template);
  const mat=HAND_MAT.clone();
  const bones={};
  hand.traverse(o=>{
    if(o.isMesh||o.isSkinnedMesh){ o.material=mat; o.castShadow=true; o.receiveShadow=true; o.frustumCulled=false; }
    if(o.isBone) bones[o.name]=o;
  });
  // Right hand = left mesh mirrored on X (Shroom recipe).
  if(side==='right') hand.scale.x=-1;
  // Center + scale to fit the camera. The full hand+forearm is kept (the forearm's
  // open end stays off-frame, avoiding the hollow wrist cross-section that a clip
  // plane would expose on this single-shell mesh).
  hand.updateMatrixWorld(true);
  const box=new THREE.Box3().setFromObject(hand);
  const center=box.getCenter(new THREE.Vector3());
  const size=box.getSize(new THREE.Vector3());
  const maxDim=Math.max(size.x,size.y,size.z)||1;
  hand.position.sub(center);
  const baseOrient=new THREE.Group();
  baseOrient.add(hand);
  baseOrient.scale.setScalar(2.7/maxDim);
  // Neutral pose = "palm toward front, fingers up" (the calibration pose). GLB rest
  // has fingers along +Z. R_x(-90) stands fingers up (+Y); R_y(180) then turns the
  // palm to face FRONT (away from camera) instead of toward the viewer. Defaulting
  // here keeps the common palm-front task pose near neutral, minimizing travel on
  // the unreliable vertical (yaw) axis.
  const _bx=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), -Math.PI/2);
  const _by=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), Math.PI);
  baseOrient.quaternion.copy(_by).multiply(_bx);
  const pivot=new THREE.Group();         // IMU orientation is applied here
  pivot.add(baseOrient);
  // Capture each finger bone's rest-pose quaternion so applyBends can rotate
  // relative to the rest pose rather than writing absolute Euler z — this avoids
  // the "snap to straight" that occurs when direct rotation.z conflicts with the
  // bone's baked GLB orientation at larger bend angles.
  const restQuats={};
  for(const fname in FINGER_BONES){
    restQuats[fname]=FINGER_BONES[fname].map(bname=>
      bones[bname]?bones[bname].quaternion.clone():new THREE.Quaternion()
    );
  }
  return {pivot, baseOrient, hand, bones, side, restQuats};
}

function applyBends(model, bends){
  if(!model||!model.bones||!model.restQuats) return;
  for(const fname in FINGER_BONES){
    const v=clamp01(bends[fname]);
    const segs=FINGER_BONES[fname];
    const rqs=model.restQuats[fname];
    _qCurl.setFromAxisAngle(_vCurlAxis,-CURL*v);
    for(let i=0;i<segs.length;i++){
      const b=model.bones[segs[i]];
      if(b) b.quaternion.copy(rqs[i]).multiply(_qCurl);
    }
  }
}

function initHand(canvas, side){
  if(REG[side]) return REG[side];
  const renderer=new THREE.WebGLRenderer({canvas, antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio||1));
  renderer.setClearAlpha(0);
  renderer.shadowMap.enabled=true; renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  renderer.outputColorSpace=THREE.SRGBColorSpace;
  const scene=new THREE.Scene();
  const camera=new THREE.PerspectiveCamera(32, 1, 0.1, 100);
  camera.position.set(0,0,4.6); camera.lookAt(0,0,0);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x3a4150, 0.5));
  scene.add(new THREE.AmbientLight(0xffffff, 0.3));
  const key=new THREE.DirectionalLight(0xffffff, 1.05); key.position.set(2.5,4,3);
  key.castShadow=true; key.shadow.mapSize.set(1024,1024);
  key.shadow.camera.near=0.5; key.shadow.camera.far=14;
  key.shadow.camera.left=-2.5; key.shadow.camera.right=2.5; key.shadow.camera.top=2.5; key.shadow.camera.bottom=-2.5;
  key.shadow.bias=-0.0008; scene.add(key);
  const fill=new THREE.DirectionalLight(0xffffff, 0.35); fill.position.set(-3,1,1.5); scene.add(fill);
  const rim=new THREE.DirectionalLight(0xffffff, 0.45); rim.position.set(0,1.5,-3.5); scene.add(rim);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(10,10), new THREE.ShadowMaterial({opacity:0.2}));
  ground.rotation.x=-Math.PI/2; ground.position.y=-1.55; ground.receiveShadow=true; scene.add(ground);
  const ctx={renderer,scene,camera,model:null,canvas,side,_w:0,_h:0,_last:{quat:null,bends:{},ref:null}};
  REG[side]=ctx;
  setHandStatus(side, 'Loading hand model…');
  ensureSize(ctx); ctx.renderer.render(ctx.scene, ctx.camera);
  loadTemplate().then(template=>{
    const model=buildModel(side, template);
    ctx.scene.add(model.pivot);
    ctx.model=model;
    clearHandStatus(side);
    ensureSize(ctx);
    render(ctx, ctx._last.quat, ctx._last.bends, ctx._last.ref);
  }).catch(err=>{
    console.error('[Hand3D] GLB load failed', err);
    setHandStatus(side, 'Failed to load hand model', 'err');
  });
  return ctx;
}
function ensureSize(ctx){
  const cw=ctx.canvas.clientWidth|0, ch=ctx.canvas.clientHeight|0;
  if(cw===0||ch===0) return false;
  if(cw!==ctx._w||ch!==ctx._h){
    ctx.renderer.setSize(cw,ch,false);
    ctx.camera.aspect=cw/ch; ctx.camera.updateProjectionMatrix();
    ctx._w=cw; ctx._h=ch;
  }
  return true;
}
const _qRaw=new THREE.Quaternion(), _qRef=new THREE.Quaternion();
const _qCurl=new THREE.Quaternion(), _vCurlAxis=new THREE.Vector3(0,0,1);
function render(ctx, quat, bends, ref){
  ctx._last.quat=quat||null; ctx._last.bends=bends||{}; ctx._last.ref=ref||null;
  const m=ctx.model;
  if(m){
    if(quat){
      _qRaw.set(quat.x,quat.y,quat.z,quat.w);
      if(ref){
        // corrected = q_ref^-1 * q_raw  (THREE order x,y,z,w; decode is w,x,y,z)
        _qRef.set(ref.x,ref.y,ref.z,ref.w).invert();
        _qRaw.premultiply(_qRef);
      }
      _qRaw.normalize();
      m.pivot.quaternion.copy(_qRaw);
    }else{
      m.pivot.quaternion.set(0,0,0,1);
    }
    applyBends(m, bends||{});
  }
  ctx.renderer.render(ctx.scene, ctx.camera);
}
function updateHand(side, quat, bends, ref){
  const ctx=REG[side]; if(!ctx) return;
  if(!ensureSize(ctx)) return;
  render(ctx, quat, bends, ref);
}
window.Hand3D={init:initHand, update:updateHand, has:s=>!!REG[s]};
</script>
</body>
</html>
"""


def _required_asset_paths() -> List[Path]:
    return [
        ASSETS_DIR / "three.module.js",
        ASSETS_DIR / "GLTFLoader.js",
        ASSETS_DIR / "shroom_hand1.glb",
        ASSETS_DIR / "utils" / "SkeletonUtils.js",
        ASSETS_DIR / "utils" / "BufferGeometryUtils.js",
    ]


def _fix_malformed_assets() -> bool:
    """Repair Windows-zip paths where backslashes became part of filenames."""
    fixed = False
    for bad in _HERE.glob("assets\\*"):
        rel = str(bad.relative_to(_HERE)).replace("\\", "/")
        target = _HERE / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            bad.rename(target)
            fixed = True
        elif bad.is_file():
            bad.unlink()
            fixed = True
    return fixed


def _check_assets() -> None:
    if _fix_malformed_assets():
        print("[viewer] repaired assets/ folder (Windows zip used backslashes in filenames)")
    missing = [p for p in _required_asset_paths() if not p.is_file()]
    if missing:
        print("[viewer] WARNING: missing bundled 3D assets — the hand model will not load:")
        for p in missing:
            print(f"          {p.relative_to(_HERE)}")
        print("[viewer] Re-copy the full tactile-glove-demo-macos folder from the release zip.")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Tactile Glove local demo viewer — unified live + replay. "
                    "Default: replay always works and gloves go live automatically when connected.")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    p.add_argument("--left", default="", help="Force left glove COM port (e.g. COM10)")
    p.add_argument("--right", default="", help="Force right glove COM port (e.g. COM9)")
    p.add_argument("--auto", action="store_true", default=True,
                   help="(Default) Auto-detect gloves and hot-plug watch for them")
    p.add_argument("--single", default="", help="Force a single glove port (auto side)")
    p.add_argument("--replay-only", action="store_true", default=False,
                   help="Opt out of live: disable the glove watcher, only upload/replay recordings")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    _check_assets()
    # Explicit port overrides (used regardless of auto-detect). Ignored in --replay-only.
    ports: Dict[str, str] = {}
    if not args.replay_only:
        if args.single:
            ports["auto"] = args.single
        else:
            if args.left:
                ports["left"] = args.left
            if args.right:
                ports["right"] = args.right
    # The server ALWAYS starts — no gloves required. Replay/upload/playback work either way.
    app = ViewerApp(ports)
    if args.replay_only:
        print("[viewer] Replay-only mode — live glove watcher disabled. Upload and play recordings.")
    else:
        if ports:
            print("[viewer] Forced live ports:", ports)
        print("[viewer] Unified live + replay mode — replay is always available; "
              "gloves go live automatically whenever they connect (no restart needed).")
        app.start_watcher()
    handler = make_handler(app)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    url = f"http://127.0.0.1:{args.port}"
    print(f"[viewer] Tactile Glove demo running at {url}")
    print("[viewer] Press Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[viewer] Stopping...")
    finally:
        app.stop()
        server.server_close()


if __name__ == "__main__":
    main()
