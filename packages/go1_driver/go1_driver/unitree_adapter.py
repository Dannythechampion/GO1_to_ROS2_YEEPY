"""Adapter for the legacy Unitree Go1 pybind11 UDP interface."""

from __future__ import annotations

import importlib
import os
import sys
import time

from .command_filter import MotionCommand, WALK_MODE
from .robot_state import RobotState, read_robot_state


def _fingerprint(state) -> tuple:
    """Values that change on every real packet (IMU noise, foot forces)."""
    imu = getattr(state, "imu", None)
    parts = []
    for owner, name in ((imu, "quaternion"), (imu, "accelerometer"), (imu, "gyroscope"), (state, "footForce")):
        try:
            parts.append(tuple(float(value) for value in getattr(owner, name, ()) or ()))
        except (TypeError, ValueError):
            parts.append(())
    return tuple(parts)


class UnitreeHighLevel:
    """Send filtered commands through unitree_legged_sdk's HighLevel API."""

    def __init__(
        self,
        sdk_path: str,
        robot_ip: str,
        robot_port: int,
        local_port: int,
    ) -> None:
        expanded_sdk_path = os.path.abspath(os.path.expanduser(sdk_path))
        if not os.path.isdir(expanded_sdk_path):
            raise RuntimeError(f"Unitree SDK Python directory not found: {expanded_sdk_path}")
        if expanded_sdk_path not in sys.path:
            sys.path.insert(0, expanded_sdk_path)

        try:
            sdk = importlib.import_module("robot_interface")
        except ImportError as exc:
            raise RuntimeError(
                f"Cannot import robot_interface from {expanded_sdk_path!r}"
            ) from exc

        self._udp = sdk.UDP(0xEE, local_port, robot_ip, robot_port)
        self._cmd = sdk.HighCmd()
        self._state = sdk.HighState()
        self._udp.InitCmdData(self._cmd)
        self._last_fingerprint = None
        self.last_reply_fresh = False

    def send(self, command: MotionCommand) -> RobotState:
        """Send one command and return what the robot last reported."""
        received = self._udp.Recv()
        self._udp.GetRecv(self._state)
        snapshot = read_robot_state(self._state)
        # The SDK's Recv returns the byte count of a new packet; bindings that
        # return nothing fall back to noticing that the reply changed. GetRecv
        # alone cannot tell: it keeps handing back the last packet forever.
        if isinstance(received, int) and not isinstance(received, bool):
            fresh = received > 0
        else:
            fingerprint = _fingerprint(self._state)
            fresh = fingerprint != self._last_fingerprint
            self._last_fingerprint = fingerprint
        self.last_reply_fresh = fresh and snapshot.live

        cmd = self._cmd
        cmd.mode = command.mode
        cmd.gaitType = 1 if command.mode == WALK_MODE else 0
        cmd.velocity = [float(command.vx), float(command.vy)]
        cmd.yawSpeed = float(command.yaw)
        cmd.bodyHeight = 0.0
        cmd.euler = [0.0, 0.0, 0.0]
        cmd.footRaiseHeight = 0.0
        cmd.reserve = 0
        self._udp.SetSend(cmd)
        self._udp.Send()
        return snapshot

    def stand(self, repeats: int = 30, period: float = 0.01) -> None:
        command = MotionCommand.stand("shutdown")
        for _ in range(repeats):
            self.send(command)
            time.sleep(period)
