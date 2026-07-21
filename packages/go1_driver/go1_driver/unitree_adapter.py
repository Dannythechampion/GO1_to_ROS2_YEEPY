"""Adapter for the legacy Unitree Go1 pybind11 UDP interface."""

from __future__ import annotations

import importlib
import os
import sys
import time

from .command_filter import MotionCommand, WALK_MODE


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

    def send(self, command: MotionCommand) -> None:
        self._udp.Recv()
        self._udp.GetRecv(self._state)

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

    def stand(self, repeats: int = 30, period: float = 0.01) -> None:
        command = MotionCommand.stand("shutdown")
        for _ in range(repeats):
            self.send(command)
            time.sleep(period)
