import math

import pytest

from omx_navigation.cmd_vel_gate_core import VelocityCommand, VelocityGate


def ready_gate(now: float = 1.0) -> VelocityGate:
    gate = VelocityGate(ready_timeout=0.30, command_timeout=0.30)
    gate.update_ready(True, now=now)
    return gate


def test_gate_never_passes_before_fresh_ready_heartbeat():
    gate = VelocityGate(ready_timeout=0.30, command_timeout=0.30)

    decision = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.0)

    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "localization_not_ready"


def test_gate_passes_only_while_ready_and_fresh():
    gate = ready_gate(now=1.0)

    moving = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.1)
    stale = gate.watchdog(now=1.31)

    assert moving.command.vx == 0.2
    assert stale.command == VelocityCommand.zero()
    assert stale.reason == "ready_heartbeat_stale"


def test_ready_false_closes_gate_and_requests_stop():
    gate = ready_gate(now=1.0)

    closed = gate.update_ready(False, now=1.1)

    assert closed.publish is True
    assert closed.command == VelocityCommand.zero()
    assert closed.reason == "localization_not_ready"


def test_gate_rejects_nonfinite_commands_with_a_stop():
    gate = ready_gate()

    decision = gate.filter(VelocityCommand(math.nan, 0.0, 0.0), now=1.1)

    assert decision.command == VelocityCommand.zero()
    assert decision.publish is True
    assert decision.reason == "command_nonfinite"


def test_false_ready_overrides_a_fresh_command():
    gate = ready_gate()
    gate.update_ready(False, now=1.1)

    decision = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.2)

    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "localization_not_ready"


def test_watchdog_stops_after_command_timeout_while_ready():
    gate = ready_gate()
    gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.1)
    gate.update_ready(True, now=1.3)

    decision = gate.watchdog(now=1.41)

    assert decision.command == VelocityCommand.zero()
    assert decision.publish is True
    assert decision.reason == "command_stale"


@pytest.mark.parametrize(
    "event",
    (
        lambda gate: gate.update_ready(True, now=10.0),
        lambda gate: gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=10.0),
        lambda gate: gate.watchdog(now=10.0),
    ),
)
def test_time_moving_backwards_closes_the_gate_for_every_public_event(event):
    gate = ready_gate(now=100.0)

    decision = event(gate)

    assert decision.command == VelocityCommand.zero()
    assert decision.publish is True
    assert decision.reason == "time_moved_backwards"
    assert gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=10.1).reason == "localization_not_ready"


def test_time_backward_recovery_requires_a_new_true_heartbeat():
    gate = ready_gate(now=100.0)

    assert gate.watchdog(now=10.0).reason == "time_moved_backwards"
    assert gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=10.1).reason == "localization_not_ready"
    assert gate.update_ready(True, now=10.2).publish is False

    decision = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=10.3)

    assert decision.command == VelocityCommand(0.2, 0.0, 0.1)
    assert decision.reason == "command_passed"


@pytest.mark.parametrize("field", ("vx", "vy", "yaw"))
@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_gate_rejects_every_nonfinite_velocity_component(field, value):
    gate = ready_gate()
    values = {"vx": 0.2, "vy": 0.1, "yaw": 0.3}
    values[field] = value

    decision = gate.filter(VelocityCommand(**values), now=1.1)

    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "command_nonfinite"


def test_ready_heartbeat_is_stale_at_exactly_three_tenths_of_a_second():
    gate = ready_gate(now=1.0)

    decision = gate.watchdog(now=1.30)

    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "ready_heartbeat_stale"


def test_command_is_stale_at_exactly_three_tenths_of_a_second():
    gate = ready_gate(now=1.0)
    gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.1)
    gate.update_ready(True, now=1.3)

    decision = gate.watchdog(now=1.4)

    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "command_stale"


@pytest.mark.parametrize("value", (True, "0.30", math.nan))
def test_gate_rejects_non_numeric_or_nonfinite_timeouts(value):
    with pytest.raises(ValueError):
        VelocityGate(ready_timeout=value)
    with pytest.raises(ValueError):
        VelocityGate(command_timeout=value)
