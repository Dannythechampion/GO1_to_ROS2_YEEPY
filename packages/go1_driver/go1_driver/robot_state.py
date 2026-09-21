"""Read the Go1's HighState reply as plain data and decide who is in control.

Until 2026-09-18 the driver called ``GetRecv`` every cycle and then never read
the result. That afternoon the operator drove the robot back to its start with
the handheld remote while a Nav2 goal was still active. The sport controller
follows the remote over any HighCmd, so for the three `Failed to make progress`
aborts Nav2 commanded steady turns of -401, +339 and -75 degrees and the robot
executed 1%, 3% and 0% of them, and nothing in the stack could say why:
``wirelessRemote`` -- the remote's live frame -- sat in the discarded reply.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

# Both the HighState packet and the remote's frame inside it start with this.
FRAME_HEAD = (0xFE, 0xEF)
REMOTE_FRAME_BYTES = 24


@dataclass(frozen=True)
class RemoteState:
    """The handheld remote's frame (xRockerBtnDataStruct), decoded."""

    present: bool
    buttons: int = 0
    lx: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    l2: float = 0.0
    ly: float = 0.0

    def active(self, stick_deadband: float) -> bool:
        """True while the operator is pressing a key or deflecting a stick.

        A powered-on remote with centred sticks is not control: keeping one in
        hand as a stop is good practice and must not block autonomy.
        """
        if not self.present:
            return False
        if self.buttons:
            return True
        return any(abs(value) > stick_deadband for value in (self.lx, self.rx, self.ry, self.ly, self.l2))


NO_REMOTE = RemoteState(False)


def remote_bytes(raw) -> tuple[int, ...]:
    """Return ``wirelessRemote`` as plain ints, whatever the binding hands back."""
    if raw is None:
        return ()
    try:
        return tuple(int(value) & 0xFF for value in raw)
    except (TypeError, ValueError):
        return ()


def decode_remote(raw) -> RemoteState:
    data = remote_bytes(raw)
    if len(data) < REMOTE_FRAME_BYTES or data[:2] != FRAME_HEAD:
        return NO_REMOTE
    packed = bytes(data[:REMOTE_FRAME_BYTES])
    buttons = struct.unpack_from("<H", packed, 2)[0]
    lx, rx, ry, l2, ly = (
        value if math.isfinite(value) else 0.0 for value in struct.unpack_from("<5f", packed, 4)
    )
    return RemoteState(True, buttons, lx, rx, ry, l2, ly)


@dataclass(frozen=True)
class RobotState:
    """What the robot reported about itself in one HighState reply."""

    live: bool
    mode: int
    velocity: tuple[float, float]
    yaw_speed: float
    body_height: float
    range_obstacle: tuple[float, ...]
    battery_soc: Optional[int]
    remote: RemoteState


def _floats(values, count: int) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in list(values)[:count])
    except (TypeError, ValueError):
        return ()
    return result if all(math.isfinite(value) for value in result) else ()


def _float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def state_is_live(state) -> bool:
    """True when a reply filled this HighState rather than leaving it blank.

    A HighState no packet ever reached stays all zeros, and its mode 0 reads
    exactly like a robot in idle stand. Every real packet carries the frame
    head, a unit IMU quaternion, foot forces or a battery reading.
    """
    try:
        if remote_bytes(getattr(state, "head", ()))[:2] == FRAME_HEAD:
            return True
    except TypeError:
        pass
    imu = getattr(state, "imu", None)
    quaternion = _floats(getattr(imu, "quaternion", ()) if imu is not None else (), 4)
    if len(quaternion) == 4 and sum(value * value for value in quaternion) > 0.25:
        return True
    forces = _floats(getattr(state, "footForce", ()) or (), 4)
    if any(forces):
        return True
    bms = getattr(state, "bms", None)
    if bms is not None and _float(getattr(bms, "SOC", 0)) > 0:
        return True
    return any(remote_bytes(getattr(state, "wirelessRemote", None)))


def read_robot_state(state) -> RobotState:
    """Decode a HighState defensively; missing or odd fields read as absent."""
    velocity = _floats(getattr(state, "velocity", ()) or (), 2)
    ranges = _floats(getattr(state, "rangeObstacle", ()) or (), 4)
    bms = getattr(state, "bms", None)
    soc = getattr(bms, "SOC", None) if bms is not None else None
    try:
        battery = int(soc) if soc is not None else None
    except (TypeError, ValueError):
        battery = None
    try:
        mode = int(getattr(state, "mode", -1))
    except (TypeError, ValueError):
        mode = -1
    return RobotState(
        live=state_is_live(state),
        mode=mode,
        velocity=velocity if len(velocity) == 2 else (0.0, 0.0),
        yaw_speed=_float(getattr(state, "yawSpeed", 0.0)),
        body_height=_float(getattr(state, "bodyHeight", 0.0)),
        range_obstacle=ranges,
        battery_soc=battery,
        remote=decode_remote(getattr(state, "wirelessRemote", None)),
    )


class OverrideLatch:
    """Manual control holds until the remote has been idle for `release_s`.

    A stick passes through centre whenever the operator changes direction, so
    releasing on the first idle frame would hand control back mid-manoeuvre.
    """

    def __init__(self, release_s: float = 1.0) -> None:
        if not math.isfinite(release_s) or release_s <= 0.0:
            raise ValueError("release_s must be finite and positive")
        self.release_s = release_s
        self.active = False
        self._last_activity: Optional[float] = None

    def update(self, now: float, remote_active: bool) -> bool:
        if remote_active:
            self.active = True
            self._last_activity = now
        elif self.active and self._last_activity is not None and now - self._last_activity >= self.release_s:
            self.active = False
        return self.active


class LinkMonitor:
    """The robot link is up while live replies keep arriving."""

    def __init__(self, timeout_s: float = 0.5) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        self.timeout_s = timeout_s
        self._last_reply: Optional[float] = None

    def update(self, now: float, fresh_live_reply: bool) -> bool:
        if fresh_live_reply:
            self._last_reply = now
        return self.up(now)

    def up(self, now: float) -> bool:
        return self._last_reply is not None and now - self._last_reply <= self.timeout_s
