import pytest

from omx_navigation.mission_state import (
    GoalSource,
    MissionEffect,
    MissionState,
    MissionStateMachine,
)
from omx_navigation.pose_config import PlanarPose


DESTINATION = PlanarPose("map", 4.0, 5.0, 0.5)


def safe_machine(destination=DESTINATION):
    machine = MissionStateMachine(destination)
    machine.update_localization(True, 1.0)
    machine.update_motion_gate(True, 1.0)
    machine.update_action_server(True)
    return machine


def test_missing_destination_blocks_fixed_goal_but_not_rviz_goal():
    machine = safe_machine(destination=None)
    fixed = machine.request_fixed(now=1.1, goal_valid=True)
    assert fixed.effect is MissionEffect.REJECT
    assert machine.state is MissionState.UNCONFIGURED_DESTINATION

    rviz = machine.request_rviz(PlanarPose("map", 1, 2, 0), now=1.1, goal_valid=True)
    assert rviz.effect is MissionEffect.SEND_GOAL
    assert rviz.goal == PlanarPose("map", 1, 2, 0)


@pytest.mark.parametrize(
    "setup, reason",
    [
        (lambda m: m.update_localization(False, 1.0), "localization"),
        (lambda m: m.update_motion_gate(False, 1.0), "motion gate"),
        (lambda m: m.update_action_server(False), "action server"),
    ],
)
def test_common_safety_prerequisite_rejects_goal(setup, reason):
    machine = safe_machine()
    setup(machine)
    result = machine.request_fixed(now=1.1, goal_valid=True)
    assert result.effect is MissionEffect.REJECT
    assert reason in result.reason


def test_stale_safety_heartbeat_rejects_goal():
    result = safe_machine().request_fixed(now=1.31, goal_valid=True)
    assert result.effect is MissionEffect.REJECT
    assert "stale" in result.reason


def test_goal_validation_is_common_to_both_sources():
    machine = safe_machine()
    assert machine.request_fixed(now=1.1, goal_valid=False).effect is MissionEffect.REJECT
    assert machine.request_rviz(
        PlanarPose("map", 1, 1, 0), now=1.1, goal_valid=False
    ).effect is MissionEffect.REJECT


def test_active_mission_rejects_replacement_and_preserves_source():
    machine = safe_machine()
    sent = machine.request_fixed(now=1.1, goal_valid=True)
    assert sent.effect is MissionEffect.SEND_GOAL
    assert machine.source is GoalSource.FIXED
    machine.goal_response(True)
    assert machine.state is MissionState.ACTIVE
    rejected = machine.request_rviz(
        PlanarPose("map", 1, 1, 0), now=1.2, goal_valid=True
    )
    assert rejected.effect is MissionEffect.REJECT


def test_explicit_cancel_and_safety_loss_request_cancel_once():
    machine = safe_machine()
    machine.request_fixed(now=1.1, goal_valid=True)
    machine.goal_response(True)
    assert machine.request_cancel().effect is MissionEffect.CANCEL_GOAL
    assert machine.request_cancel().effect is MissionEffect.NONE

    machine = safe_machine()
    machine.request_fixed(now=1.1, goal_valid=True)
    machine.goal_response(True)
    assert machine.update_localization(False, 1.2).effect is MissionEffect.CANCEL_GOAL
    assert machine.update_localization(False, 1.3).effect is MissionEffect.NONE


@pytest.mark.parametrize(
    "outcome, state",
    [
        ("SUCCEEDED", MissionState.SUCCEEDED),
        ("FAILED", MissionState.FAILED),
        ("CANCELED", MissionState.CANCELED),
    ],
)
def test_action_outcome_persists_until_next_accepted_goal(outcome, state):
    machine = safe_machine()
    machine.request_fixed(now=1.1, goal_valid=True)
    machine.goal_response(True)
    machine.action_result(outcome)
    assert machine.state is state
    assert machine.state is state
    machine.update_localization(True, 1.2)
    machine.update_motion_gate(True, 1.2)
    assert machine.request_fixed(now=1.2, goal_valid=True).effect is MissionEffect.SEND_GOAL
