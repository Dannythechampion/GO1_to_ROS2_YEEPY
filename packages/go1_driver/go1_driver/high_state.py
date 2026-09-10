"""Stable, JSON-safe telemetry snapshots from Unitree's versioned HighState."""

from __future__ import annotations

import json
import math
from typing import Any


def _json_scalar(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)):
        return value if not isinstance(value, float) or math.isfinite(value) else None
    try:
        return _json_scalar(value.item())
    except (AttributeError, ValueError):
        return str(value)


def _value(source: Any, name: str) -> Any:
    value = getattr(source, name, None) if source is not None else None
    return _json_scalar(value)


def _vector(source: Any, name: str) -> list[Any] | None:
    value = getattr(source, name, None) if source is not None else None
    if value is None:
        return None
    try:
        return [_json_scalar(item) for item in value]
    except TypeError:
        return None


def high_state_snapshot(
    state: Any,
    receive_sequence: int | None,
    receive_monotonic_ns: int | None,
) -> dict[str, Any]:
    """Normalize fields shared by Unitree HighState firmware revisions."""
    imu = getattr(state, "imu", None)
    bms = getattr(state, "bms", None)
    return {
        "udp_receive_sequence": receive_sequence,
        "udp_receive_monotonic_ns": receive_monotonic_ns,
        "mode": _value(state, "mode"),
        "gait_type": _value(state, "gaitType"),
        "progress": _value(state, "progress"),
        "foot_raise_height": _value(state, "footRaiseHeight"),
        "body_height": _value(state, "bodyHeight"),
        "position": _vector(state, "position"),
        "velocity": _vector(state, "velocity"),
        "yaw_speed": _value(state, "yawSpeed"),
        "range_obstacle": _vector(state, "rangeObstacle"),
        "foot_force": _vector(state, "footForce"),
        "foot_force_est": _vector(state, "footForceEst"),
        "imu": {
            "quaternion": _vector(imu, "quaternion"),
            "gyroscope": _vector(imu, "gyroscope"),
            "accelerometer": _vector(imu, "accelerometer"),
            "rpy": _vector(imu, "rpy"),
            "temperature": _value(imu, "temperature"),
        },
        "battery": {
            "soc": _value(bms, "SOC"),
            "current": _value(bms, "current"),
            "cycle": _value(bms, "cycle"),
        },
    }


def encode_high_state_message(
    snapshot: dict[str, Any],
    now_monotonic_ns: int,
    stale_after_ms: float = 100.0,
) -> str:
    """Add transport freshness at publication time and encode compact JSON."""
    payload = dict(snapshot)
    received = payload.get("udp_receive_monotonic_ns")
    if received is None:
        payload["udp_receive_age_ms"] = None
        payload["udp_stale"] = None
    else:
        age_ms = max(0.0, (now_monotonic_ns - int(received)) / 1_000_000)
        payload["udp_receive_age_ms"] = age_ms
        payload["udp_stale"] = age_ms > stale_after_ms
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)
