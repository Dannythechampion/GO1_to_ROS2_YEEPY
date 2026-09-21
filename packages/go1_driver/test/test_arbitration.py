import pytest

from go1_driver.arbitration import CommandArbiter
from go1_driver.command_filter import STAND_MODE, WALK_MODE, MotionCommand

GO = MotionCommand(WALK_MODE, 0.15, 0.0, 0.3, "active command")
ZERO = MotionCommand.stand()


def test_commands_pass_when_nobody_else_is_in_control():
    assert CommandArbiter().decide(0.0, GO, override_active=False) is GO


def test_manual_override_holds_stand():
    decision = CommandArbiter().decide(0.0, GO, override_active=True)
    assert decision.mode == STAND_MODE
    assert decision.reason == "manual override: remote in use"


def test_stale_goal_does_not_resume_when_the_remote_is_released():
    arbiter = CommandArbiter(rearm_zero_s=0.5)
    arbiter.decide(0.0, GO, override_active=True)
    held = arbiter.decide(5.0, GO, override_active=False)
    assert held.mode == STAND_MODE
    assert held.reason.startswith("holding after manual override")
    assert arbiter.decide(9.0, GO, override_active=False).mode == STAND_MODE


def test_resumes_only_after_the_stream_has_been_neutral_long_enough():
    arbiter = CommandArbiter(rearm_zero_s=0.5)
    arbiter.decide(0.0, GO, override_active=True)
    assert arbiter.decide(1.0, ZERO, override_active=False).mode == STAND_MODE
    assert arbiter.decide(1.4, GO, override_active=False).mode == STAND_MODE
    # A non-zero command restarts the neutral period.
    assert arbiter.decide(1.5, ZERO, override_active=False).mode == STAND_MODE
    assert arbiter.decide(2.0, ZERO, override_active=False) is ZERO
    assert arbiter.blocked_reason is None
    assert arbiter.decide(2.1, GO, override_active=False) is GO


def test_execution_fault_blocks_the_same_way():
    arbiter = CommandArbiter(rearm_zero_s=0.5)
    arbiter.block("execution fault NOT_EXECUTING")
    held = arbiter.decide(0.0, GO, override_active=False)
    assert held.mode == STAND_MODE
    assert held.reason.startswith("holding after execution fault")
    arbiter.decide(0.1, ZERO, override_active=False)
    assert arbiter.decide(0.6, ZERO, override_active=False) is ZERO
    assert arbiter.decide(0.7, GO, override_active=False) is GO


def test_watchdog_silence_counts_as_neutral():
    arbiter = CommandArbiter(rearm_zero_s=0.5)
    arbiter.block("manual override")
    silent = MotionCommand.stand("cmd_vel watchdog timeout")
    arbiter.decide(0.0, silent, override_active=False)
    assert arbiter.decide(0.5, silent, override_active=False) is silent


def test_rejects_invalid_rearm_time():
    with pytest.raises(ValueError):
        CommandArbiter(rearm_zero_s=0.0)
