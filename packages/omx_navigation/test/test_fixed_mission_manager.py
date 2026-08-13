import pytest

from omx_navigation.fixed_mission_manager import (
    apply_goal_response,
    mission_status_line,
    planar_pose_from_values,
)
from omx_navigation.mission_state import GoalSource, MissionState, MissionStateMachine
from omx_navigation.pose_config import PlanarPose


def test_status_line_contains_state_reason_and_source():
    machine = MissionStateMachine(PlanarPose("map", 1, 2, 0))
    machine.source = GoalSource.RVIZ
    machine.state = MissionState.FAILED
    machine.reason = "Nav2 rejected goal"
    assert mission_status_line(machine) == (
        "state=FAILED active=false source=RVIZ reason=Nav2 rejected goal"
    )


def test_goal_conversion_rejects_non_planar_or_unnormalized_quaternion():
    with pytest.raises(ValueError):
        planar_pose_from_values("map", 1, 2, 0.1, 0.0, 0.0, 1.0)
    with pytest.raises(ValueError):
        planar_pose_from_values("map", 1, 2, 0.0, 0.0, 0.0, 2.0)


def test_goal_conversion_requires_map_frame():
    with pytest.raises(ValueError):
        planar_pose_from_values("odom", 1, 2, 0.0, 0.0, 0.0, 1.0)


def test_rejected_pending_send_finishes_cancel_instead_of_hanging():
    machine = MissionStateMachine(PlanarPose("map", 1, 2, 0))
    machine.state = MissionState.CANCELING
    assert not apply_goal_response(machine, accepted=False)
    assert machine.state is MissionState.CANCELED


def test_accepted_pending_send_requires_immediate_cancel():
    machine = MissionStateMachine(PlanarPose("map", 1, 2, 0))
    machine.state = MissionState.CANCELING
    assert apply_goal_response(machine, accepted=True)
    assert machine.state is MissionState.CANCELING
