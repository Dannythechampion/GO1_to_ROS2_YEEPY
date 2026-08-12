import math

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
