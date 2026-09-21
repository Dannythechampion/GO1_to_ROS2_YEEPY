import struct
from types import SimpleNamespace

import pytest

from go1_driver.robot_state import (
    NO_REMOTE,
    LinkMonitor,
    OverrideLatch,
    decode_remote,
    read_robot_state,
    state_is_live,
)


def remote_frame(buttons=0, lx=0.0, rx=0.0, ry=0.0, l2=0.0, ly=0.0, head=(0xFE, 0xEF)):
    frame = bytes(head) + struct.pack("<H5f", buttons, lx, rx, ry, l2, ly)
    return list(frame + bytes(40 - len(frame)))


def live_state(**overrides):
    fields = dict(
        head=[0xFE, 0xEF],
        imu=SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0]),
        footForce=[20, 21, 19, 22],
        bms=SimpleNamespace(SOC=83),
        mode=2,
        velocity=[0.18, -0.01, 0.0],
        yawSpeed=0.05,
        bodyHeight=0.29,
        rangeObstacle=[0.0, 0.6, 0.11, 2.0],
        wirelessRemote=remote_frame(),
    )
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_remote_frame_decodes_keys_and_sticks():
    remote = decode_remote(remote_frame(buttons=0x0100, lx=0.5, ly=-0.25, l2=0.0))
    assert remote.present is True
    assert remote.buttons == 0x0100
    assert remote.lx == pytest.approx(0.5)
    assert remote.ly == pytest.approx(-0.25)


def test_powered_remote_with_centred_sticks_is_not_control():
    """Keeping a powered remote in hand as a stop must not block autonomy."""
    remote = decode_remote(remote_frame(lx=0.03, ly=-0.05))
    assert remote.present is True
    assert remote.active(stick_deadband=0.10) is False


@pytest.mark.parametrize("frame", (
    remote_frame(buttons=0x0001),
    remote_frame(ly=0.4),
    remote_frame(rx=-0.2),
    remote_frame(l2=0.9),
))
def test_any_key_or_deflected_stick_is_manual_control(frame):
    assert decode_remote(frame).active(stick_deadband=0.10) is True


@pytest.mark.parametrize("raw", (None, [], [0] * 40, remote_frame(head=(0x00, 0x00)), "garbage", [1, 2, 3]))
def test_missing_or_headerless_remote_reads_as_absent(raw):
    assert decode_remote(raw) == NO_REMOTE
    assert decode_remote(raw).active(0.1) is False


def test_non_finite_stick_values_read_as_centred():
    remote = decode_remote(remote_frame(lx=float("nan"), ly=float("inf")))
    assert remote.lx == 0.0 and remote.ly == 0.0
    assert remote.active(0.1) is False


def test_blank_reply_is_not_a_live_robot():
    """A powered-off robot leaves HighState all zeros, and mode 0 in it reads like idle stand."""
    blank = SimpleNamespace(head=[0, 0], imu=SimpleNamespace(quaternion=[0.0] * 4), footForce=[0] * 4,
                            bms=SimpleNamespace(SOC=0), wirelessRemote=[0] * 40, mode=0)
    assert state_is_live(blank) is False
    assert read_robot_state(blank).live is False
    assert state_is_live(live_state()) is True


def test_robot_state_carries_what_the_robot_reported():
    state = read_robot_state(live_state(wirelessRemote=remote_frame(ly=0.6)))
    assert state.mode == 2
    assert state.velocity == pytest.approx((0.18, -0.01))
    assert state.range_obstacle == pytest.approx((0.0, 0.6, 0.11, 2.0))
    assert state.battery_soc == 83
    assert state.remote.active(0.1) is True


def test_odd_binding_values_are_read_defensively():
    state = read_robot_state(SimpleNamespace(mode="x", velocity=None, yawSpeed=float("nan"), rangeObstacle="?"))
    assert state.mode == -1
    assert state.velocity == (0.0, 0.0)
    assert state.yaw_speed == 0.0
    assert state.range_obstacle == ()
    assert state.battery_soc is None
    assert state.remote == NO_REMOTE


def test_override_holds_through_a_stick_passing_centre_then_releases():
    latch = OverrideLatch(release_s=1.0)
    assert latch.update(0.0, True) is True
    assert latch.update(0.5, False) is True
    assert latch.update(0.9, True) is True
    assert latch.update(1.8, False) is True
    assert latch.update(2.0, False) is False


def test_link_is_down_until_a_live_reply_and_after_replies_stop():
    link = LinkMonitor(timeout_s=0.5)
    assert link.update(0.0, False) is False
    assert link.update(0.1, True) is True
    assert link.update(0.5, False) is True
    assert link.update(0.7, False) is False


@pytest.mark.parametrize("factory", (lambda: OverrideLatch(0.0), lambda: LinkMonitor(float("nan"))))
def test_monitors_reject_invalid_timing(factory):
    with pytest.raises(ValueError):
        factory()
