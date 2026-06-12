#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Canonical timestamp helpers for fearless agents.

This module is **vendored** into both ``teleop-agent`` and ``camera-agent``
deliberately — agents run on disparate hardware boxes (Jetson, Mac mini,
laptop) without a shared Python package install, so duplicating ~80 lines is
cheaper than dragging in a dependency. Both copies MUST stay byte-identical;
a CI diff check enforces this.

Why this exists
---------------
Today's agents emit timestamps in inconsistent units:

- ``manus_glove_agent``     -> ``ts`` = ``int(time.time() * 1000)`` (epoch ms)
- ``umi_collection_agent``  -> ``ts`` = epoch ms; ``ts_ms`` also nested
- ``linker_glove_agent``    -> ``ts`` = ``time.time()`` (epoch seconds, float)
- ``wuji_hand_sdk_stream``  -> ``ts`` = ``time.time()`` (epoch seconds, float)
- ``camera-agent`` binary   -> ``ts_ns`` (epoch ns, uint64)  [canonical]

Multi-modal fusion (vision + tactile + pose + manipulator state) needs
sub-millisecond alignment across machines. Mixing seconds, milliseconds, and
nanoseconds silently is the textbook way to make a dataset unusable.

The canonical wire field is ``ts_ns`` (epoch ns, uint64). Every outbound
envelope additionally carries:

- ``ts_mono_ns`` -- monotonic clock at sample time, for **drift detection**
  (compare wall-vs-monotonic delta over a session).
- ``ts_ms``      -- backwards-compat alias = ``ts_ns // 1_000_000``.
- legacy ``ts``  -- agents keep emitting this in whatever unit they
                    historically did; the backend normalizer falls back to it.
"""

from __future__ import annotations

import time
from typing import Optional


def now_ns() -> int:
    """Wall-clock epoch nanoseconds (UTC)."""
    return time.time_ns()


def monotonic_ns() -> int:
    """Monotonic nanoseconds. NOT epoch — only valid for deltas within a process."""
    return time.monotonic_ns()


def epoch_to_ns(epoch_s: float) -> int:
    """Convert epoch seconds (float, as from ``time.time()``) to epoch ns."""
    return int(epoch_s * 1_000_000_000)


def ns_to_ms(ns: int) -> int:
    """Truncate epoch ns to epoch ms (matches the historical ``int(time.time()*1000)`` math)."""
    return int(ns) // 1_000_000


def ns_to_epoch_s(ns: int) -> float:
    """Epoch ns -> epoch seconds (float). Loses sub-microsecond precision."""
    return float(ns) / 1_000_000_000.0


def wall_monotonic_pair() -> tuple[int, int]:
    """Atomically capture ``(wall_ns, monotonic_ns)`` for offset bookkeeping.

    The two reads happen back-to-back; on a normal box the spread is < 1 us.
    The backend stores both so clock-sync (T2) can detect drift between the
    agent's wall clock and its monotonic clock over a session — a non-zero
    drift trend means NTP/PTP is slewing under the agent and frame ordering
    can't be trusted on wall ts alone.
    """
    return time.time_ns(), time.monotonic_ns()


def make_ts_envelope(ts_ns: Optional[int] = None) -> dict:
    """Build the canonical timestamp triple to embed on every outbound frame.

    Returns ``{'ts_ns': <wall ns>, 'ts_mono_ns': <monotonic ns>, 'ts_ms': <wall ns // 1e6>}``.

    - ``ts_ns`` is the canonical wall-clock field consumed by the backend.
    - ``ts_mono_ns`` is for drift detection (NOT comparable across processes).
    - ``ts_ms`` is a kept-for-backcompat alias; the invariant
      ``ts_ms == ts_ns // 1_000_000`` always holds.

    If ``ts_ns`` is passed in (e.g. the caller already sampled the clock and
    wants to share the same timestamp across an envelope + nested payload),
    only ``ts_mono_ns`` is freshly captured. Otherwise both are captured
    atomically via ``wall_monotonic_pair()``.
    """
    if ts_ns is None:
        wall, mono = wall_monotonic_pair()
    else:
        wall = int(ts_ns)
        mono = time.monotonic_ns()
    return {
        "ts_ns": wall,
        "ts_mono_ns": mono,
        "ts_ms": wall // 1_000_000,
    }
