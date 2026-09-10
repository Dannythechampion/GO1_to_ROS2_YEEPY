import sys
from types import ModuleType, SimpleNamespace

try:
    from go1_driver.high_state import encode_high_state_message, high_state_snapshot
except ModuleNotFoundError:
    encode_high_state_message = None
    high_state_snapshot = None

from go1_driver.command_filter import MotionCommand
from go1_driver.unitree_adapter import UnitreeHighLevel


def test_high_state_snapshot_preserves_motion_imu_contact_and_battery_evidence():
    assert high_state_snapshot is not None, "high-state snapshot support is missing"
    state = SimpleNamespace(
        mode=2,
        gaitType=1,
        progress=0.75,
        footRaiseHeight=0.08,
        bodyHeight=0.01,
        position=[1.0, 2.0, 0.31],
        velocity=[0.12, -0.03, 0.0],
        yawSpeed=0.2,
        rangeObstacle=[0.8, 1.2, 3.4, 2.1],
        footForce=[14, 15, 16, 17],
        footForceEst=[13, 14, 15, 16],
        imu=SimpleNamespace(
            quaternion=[1.0, 0.0, 0.0, 0.0],
            gyroscope=[0.1, 0.2, 0.3],
            accelerometer=[0.0, 0.0, 9.81],
            rpy=[0.01, -0.02, 0.3],
            temperature=42,
        ),
        bms=SimpleNamespace(SOC=86, current=-120, cycle=31),
    )

    assert high_state_snapshot(state, receive_sequence=7, receive_monotonic_ns=1234) == {
        "udp_receive_sequence": 7,
        "udp_receive_monotonic_ns": 1234,
        "mode": 2,
        "gait_type": 1,
        "progress": 0.75,
        "foot_raise_height": 0.08,
        "body_height": 0.01,
        "position": [1.0, 2.0, 0.31],
        "velocity": [0.12, -0.03, 0.0],
        "yaw_speed": 0.2,
        "range_obstacle": [0.8, 1.2, 3.4, 2.1],
        "foot_force": [14, 15, 16, 17],
        "foot_force_est": [13, 14, 15, 16],
        "imu": {
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "gyroscope": [0.1, 0.2, 0.3],
            "accelerometer": [0.0, 0.0, 9.81],
            "rpy": [0.01, -0.02, 0.3],
            "temperature": 42,
        },
        "battery": {"soc": 86, "current": -120, "cycle": 31},
    }


def test_high_state_snapshot_uses_null_for_sdk_fields_missing_from_a_firmware():
    assert high_state_snapshot is not None, "high-state snapshot support is missing"
    snapshot = high_state_snapshot(SimpleNamespace(), 1, 99)

    assert snapshot["mode"] is None
    assert snapshot["foot_force"] is None
    assert snapshot["imu"]["rpy"] is None
    assert snapshot["battery"]["soc"] is None


def test_high_state_message_reports_udp_age_in_milliseconds():
    assert encode_high_state_message is not None, "high-state encoding support is missing"

    encoded = encode_high_state_message(
        {"mode": 2, "udp_receive_monotonic_ns": 1_000_000_000},
        now_monotonic_ns=1_012_500_000,
    )

    assert encoded == (
        '{"mode":2,"udp_receive_age_ms":12.5,'
        '"udp_receive_monotonic_ns":1000000000,"udp_stale":false}'
    )


def test_high_state_snapshot_replaces_nonfinite_sdk_values_with_null():
    assert high_state_snapshot is not None, "high-state snapshot support is missing"

    snapshot = high_state_snapshot(
        SimpleNamespace(velocity=[float("nan"), float("inf")], yawSpeed=float("-inf")),
        1,
        10,
    )

    assert snapshot["velocity"] == [None, None]
    assert snapshot["yaw_speed"] is None


def test_unitree_send_returns_each_received_high_state_with_freshness_sequence(
    tmp_path, monkeypatch
):
    class FakeUdp:
        def __init__(self, *_args):
            pass

        def InitCmdData(self, _cmd):
            pass

        def Recv(self):
            pass

        def GetRecv(self, state):
            state.mode = 2

        def SetSend(self, _cmd):
            pass

        def Send(self):
            pass

    sdk = ModuleType("robot_interface")
    sdk.UDP = FakeUdp
    sdk.HighCmd = lambda: SimpleNamespace()
    sdk.HighState = lambda: SimpleNamespace()
    monkeypatch.setitem(sys.modules, "robot_interface", sdk)
    adapter = UnitreeHighLevel(str(tmp_path), "192.168.123.161", 8082, 8080)

    first = adapter.send(MotionCommand.stand("test"))
    second = adapter.send(MotionCommand.stand("test"))

    assert first["mode"] == 2
    assert first["udp_receive_sequence"] == 1
    assert second["udp_receive_sequence"] == 2
    assert second["udp_receive_monotonic_ns"] >= first["udp_receive_monotonic_ns"]
    assert second["udp_freshness_basis"] == "recv_return"
