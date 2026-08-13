import pytest

from omx_navigation.fixed_mission_manager import mission_status_line, planar_pose_from_values
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
