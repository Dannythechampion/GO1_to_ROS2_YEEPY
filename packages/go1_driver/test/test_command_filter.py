import math

import pytest

from go1_driver.command_filter import (
    CommandFilter,
    MotionCommand,
    STAND_MODE,
    WALK_MODE,
    apply_watchdog,
)


def test_zero_command_uses_stand_mode():
    command = CommandFilter().filter(0.0, 0.0, 0.0)
    assert command.mode == STAND_MODE
    assert (command.vx, command.vy, command.yaw) == (0.0, 0.0, 0.0)


def test_linear_residual_is_removed():
    assert CommandFilter(linear_deadband=0.03).filter(0.01, -0.01, 0.0).mode == STAND_MODE


def test_diagonal_direction_is_preserved_when_limited():
    command = CommandFilter(max_linear_speed=0.20).filter(0.30, 0.30, 0.0)
    expected = 0.20 / math.sqrt(2.0)
    assert command.mode == WALK_MODE
    assert command.vx == pytest.approx(expected)
    assert command.vy == pytest.approx(expected)
    assert math.hypot(command.vx, command.vy) == pytest.approx(0.20)


def test_lateral_direction_can_be_inverted():
    command = CommandFilter(invert_lateral=True).filter(0.0, 0.10, 0.0)
    assert command.vy == pytest.approx(-0.10)


def test_nonholonomic_mode_removes_lateral_command():
    command = CommandFilter(holonomic=False).filter(0.10, 0.10, 0.0)
    assert command.vx == pytest.approx(0.10)
    assert command.vy == 0.0


def test_small_yaw_residual_is_removed():
    assert CommandFilter(yaw_deadband=0.05).filter(0.0, 0.0, 0.01).mode == STAND_MODE


def test_turn_in_place_remains_available():
    command = CommandFilter().filter(0.0, 0.0, 0.20)
    assert command.mode == WALK_MODE
    assert command.yaw == pytest.approx(0.20)


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_command_stops(invalid):
    command = CommandFilter().filter(invalid, 0.0, 0.0)
    assert command.mode == STAND_MODE
    assert command.reason == "non-finite command"


def test_watchdog_stops_stale_command():
    active = MotionCommand(WALK_MODE, 0.10, 0.0, 0.0, "active command")
    command = apply_watchdog(active, last_command_time=1.0, now=1.5, timeout=0.35)
    assert command.mode == STAND_MODE
    assert command.reason == "cmd_vel watchdog timeout"


def test_watchdog_keeps_fresh_command():
    active = MotionCommand(WALK_MODE, 0.10, 0.0, 0.0, "active command")
    assert apply_watchdog(active, 1.0, 1.2, 0.35) == active


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        CommandFilter(max_linear_speed=0.0)
    with pytest.raises(ValueError):
        CommandFilter(max_yaw_speed=-1.0)
    with pytest.raises(ValueError):
        CommandFilter(linear_deadband=-0.1)
    with pytest.raises(ValueError):
        apply_watchdog(MotionCommand.stand(), None, 0.0, 0.0)
