from omx_navigation.goal_gate import GoalGate


def test_goal_gate_rejects_until_ready_and_cancels_on_loss():
    gate = GoalGate()
    assert gate.accept_goal() is False
    assert gate.update_ready(True) is False
    assert gate.accept_goal() is True
    gate.set_goal_active(True)
    assert gate.update_ready(False) is True
    assert gate.accept_goal() is False


def test_goal_gate_only_requests_cancellation_once_per_active_goal():
    gate = GoalGate()
    gate.update_ready(True)
    gate.set_goal_active(True)
    assert gate.update_ready(False) is True
    assert gate.update_ready(False) is False
    gate.set_goal_active(False)
    gate.update_ready(True)
    assert gate.update_ready(False) is False


def test_any_inhibitor_refuses_goals_and_says_why():
    gate = GoalGate()
    gate.update_ready(True)
    assert gate.update_inhibit("manual override", True) is True
    assert gate.accept_goal() is False
    assert gate.refusal_reason() == "manual override"
    assert gate.update_inhibit("manual override", True) is False
    gate.update_inhibit("robot not following commands", True)
    assert gate.refusal_reason() == "manual override, robot not following commands"
    gate.update_inhibit("manual override", False)
    gate.update_inhibit("robot not following commands", False)
    assert gate.accept_goal() is True
    assert gate.refusal_reason() == ""


def test_localization_is_named_when_it_is_the_only_reason():
    gate = GoalGate()
    assert gate.refusal_reason() == "localization not ready"


def test_inhibitor_input_is_validated():
    import pytest

    gate = GoalGate()
    with pytest.raises(ValueError):
        gate.update_inhibit("", True)
    with pytest.raises(ValueError):
        gate.update_inhibit("manual override", 1)
