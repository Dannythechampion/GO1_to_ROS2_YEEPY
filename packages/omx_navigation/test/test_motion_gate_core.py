import math

import pytest

from omx_navigation.motion_gate_core import MotionGateCore, VelocityCommand


def ready_gate(now=1.0):
    gate = MotionGateCore(max_linear_speed=0.2, max_angular_speed=0.4)
    gate.update_localization(True, now)
    gate.update_estop(False, now)
    gate.update_mission_stop(False, now)
    assert gate.arm(now).accepted
    return gate


def test_gate_defaults_disabled_and_zero():
    result = MotionGateCore().evaluate(1.0)
    assert not result.enabled
    assert result.command == VelocityCommand.zero()


def test_enabled_does_not_require_a_nav_command_but_output_remains_zero():
    result = ready_gate().evaluate(1.0)
    assert result.enabled
    assert result.command == VelocityCommand.zero()
    assert result.reason == "waiting for navigation command"


def test_fresh_valid_command_passes_when_enabled():
    gate = ready_gate()
    gate.update_command(VelocityCommand(0.1, 0.0, -0.2), 1.0)
    result = gate.evaluate(1.1)
    assert result.enabled
    assert result.command == VelocityCommand(0.1, 0.0, -0.2)


@pytest.mark.parametrize(
    "now, expected",
    [(1.31, "localization heartbeat is stale"), (1.26, "navigation command is stale")],
)
def test_freshness_failures_publish_zero(now, expected):
    gate = ready_gate()
    gate.update_command(VelocityCommand(0.1, 0.0, 0.0), 1.0)
    result = gate.evaluate(now)
    assert result.command == VelocityCommand.zero()
    assert expected in result.reason


@pytest.mark.parametrize(
    "command, reason",
    [
        (VelocityCommand(float("nan"), 0.0, 0.0), "non-finite"),
        (VelocityCommand(0.21, 0.0, 0.0), "linear speed"),
        (VelocityCommand(0.0, 0.0, 0.41), "angular speed"),
    ],
)
def test_invalid_commands_fail_closed(command, reason):
    gate = ready_gate()
    gate.update_command(command, 1.0)
    result = gate.evaluate(1.1)
    assert result.command == VelocityCommand.zero()
    assert reason in result.reason


def test_arm_requires_fresh_localization_and_released_estop():
    gate = MotionGateCore()
    assert not gate.arm(1.0).accepted
    gate.update_localization(True, 1.0)
    assert not gate.arm(1.0).accepted
    gate.update_estop(False, 1.0)
    assert not gate.arm(1.0).accepted
    gate.update_mission_stop(False, 1.0)
    assert gate.arm(1.0).accepted


def test_estop_or_localization_loss_clears_latched_arm():
    gate = ready_gate()
    gate.update_estop(True, 1.1)
    assert not gate.evaluate(1.1).enabled
    gate.update_estop(False, 1.2)
    assert not gate.evaluate(1.2).enabled
    assert gate.arm(1.2).accepted
    gate.update_localization(False, 1.3)
    assert not gate.evaluate(1.3).enabled


def test_disarm_immediately_clears_output():
    gate = ready_gate()
    gate.update_command(VelocityCommand(0.1, 0.0, 0.0), 1.0)
    assert gate.evaluate(1.1).command.vx == 0.1
    gate.disarm()
    assert gate.evaluate(1.1).command == VelocityCommand.zero()


def test_backward_clock_age_fails_closed():
    gate = ready_gate(now=2.0)
    gate.update_command(VelocityCommand(0.1, 0.0, 0.0), 2.0)
    result = gate.evaluate(1.9)
    assert not result.enabled
    assert result.command == VelocityCommand.zero()
    assert "stale" in result.reason


def test_mission_cancel_lockout_disarms_and_blocks_rearm():
    gate = ready_gate()
    gate.update_mission_stop(True, 1.1)
    assert not gate.evaluate(1.1).enabled
    assert not gate.arm(1.1).accepted
    gate.update_mission_stop(False, 1.2)
    assert not gate.evaluate(1.2).enabled
    assert gate.arm(1.2).accepted
