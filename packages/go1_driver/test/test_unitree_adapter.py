"""Reply freshness: the link is up while the robot's replies keep changing."""

import sys
from types import ModuleType, SimpleNamespace

import pytest

from go1_driver.command_filter import MotionCommand


class FakeUdp:
    def __init__(self, *_args):
        self.recv_result = 0
        self.tick = 0
        self.frozen = False

    def InitCmdData(self, _cmd):
        pass

    def Recv(self):
        return self.recv_result

    def GetRecv(self, state):
        if not self.frozen:
            self.tick += 1
        state.head = [0xFE, 0xEF]
        state.imu = SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.001 * self.tick],
                                    accelerometer=[0.0, 0.0, 9.8], gyroscope=[0.0, 0.0, 0.0],
                                    rpy=[0.0, 0.0, 0.0])
        state.footForce = [100, 100, 100, 100]

    def SetSend(self, _cmd):
        pass

    def Send(self):
        pass


@pytest.fixture
def adapter(monkeypatch, tmp_path):
    sdk = ModuleType("robot_interface")
    sdk.UDP = FakeUdp
    sdk.HighCmd = lambda: SimpleNamespace()
    sdk.HighState = lambda: SimpleNamespace()
    monkeypatch.setitem(sys.modules, "robot_interface", sdk)
    from go1_driver.unitree_adapter import UnitreeHighLevel
    return UnitreeHighLevel(str(tmp_path), "192.168.123.161", 8082, 8080)


def test_a_changing_reply_is_fresh_even_when_recv_reports_nothing(adapter):
    """What the robot did on 2026-11-01: Recv() never positive, IMU moving."""
    adapter.send(MotionCommand.stand())
    adapter.send(MotionCommand.stand())
    assert adapter.last_reply_fresh is True


def test_a_frozen_reply_is_stale(adapter):
    adapter.send(MotionCommand.stand())
    adapter._udp.frozen = True
    adapter.send(MotionCommand.stand())
    adapter.send(MotionCommand.stand())
    assert adapter.last_reply_fresh is False


def test_a_counted_packet_is_fresh_even_if_identical(adapter):
    adapter.send(MotionCommand.stand())
    adapter._udp.frozen = True
    adapter._udp.recv_result = 120
    adapter.send(MotionCommand.stand())
    assert adapter.last_reply_fresh is True
