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
