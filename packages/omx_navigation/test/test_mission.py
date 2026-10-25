import math

import pytest

from omx_navigation.mission import (
    Mission,
    Step,
    Waypoint,
    mission_from_dict,
    mission_to_dict,
    record_waypoint,
)


SCENARIO = {
    "waypoints": {
        "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "chair": {"x": 3.2, "y": -1.1, "yaw": 1.5708},
    },
    "sequence": [{"navigate": "chair"}, "dock", "grasp", {"navigate": "start"}],
}


def test_the_scenario_reads_back_as_written():
    mission = mission_from_dict(SCENARIO)
    assert [step.kind for step in mission.sequence] == ["navigate", "dock", "grasp", "navigate"]
    assert mission.waypoint("chair").x == pytest.approx(3.2)
    assert mission.waypoint("start").yaw == pytest.approx(0.0)


def test_round_trips_through_a_dict():
    mission = mission_from_dict(SCENARIO)
    assert mission_from_dict(mission_to_dict(mission)) == mission


def test_navigating_to_an_unrecorded_waypoint_is_rejected():
    broken = {"waypoints": SCENARIO["waypoints"], "sequence": [{"navigate": "sofa"}]}
    with pytest.raises(ValueError, match="not a recorded waypoint"):
        mission_from_dict(broken)


def test_docking_must_follow_navigation():
    """Docking drives from a detection, so the markers have to be in range.

    Catching the ordering here beats discovering it beside the chair.
    """
    with pytest.raises(ValueError, match="must follow a navigate step"):
        mission_from_dict({"waypoints": SCENARIO["waypoints"], "sequence": ["dock"]})
    with pytest.raises(ValueError, match="must follow a navigate step"):
        mission_from_dict(
            {"waypoints": SCENARIO["waypoints"], "sequence": ["grasp", "dock"]}
        )


def test_unknown_step_kinds_are_rejected():
    with pytest.raises(ValueError, match="unknown step kind"):
        Step("teleport")


def test_navigate_needs_a_waypoint_and_the_others_refuse_one():
    with pytest.raises(ValueError, match="needs a waypoint"):
        Step("navigate")
    with pytest.raises(ValueError, match="does not take a waypoint"):
        Step("dock", "chair")


def test_malformed_waypoints_are_rejected():
    with pytest.raises(ValueError, match="missing"):
        mission_from_dict({"waypoints": {"a": {"x": 1.0, "y": 2.0}}, "sequence": ["grasp"]})
    with pytest.raises(ValueError, match="finite"):
        Waypoint("a", float("nan"), 0.0, 0.0)


def test_empty_missions_are_rejected():
    with pytest.raises(ValueError, match="non-empty 'waypoints'"):
        mission_from_dict({"waypoints": {}, "sequence": ["grasp"]})
    with pytest.raises(ValueError, match="non-empty 'sequence'"):
        mission_from_dict({"waypoints": SCENARIO["waypoints"], "sequence": []})


def test_re_recording_replaces_one_pose_and_leaves_the_sequence():
    """How the scenario adapts when the chair or the markers move."""
    mission = mission_from_dict(SCENARIO)
    moved = record_waypoint(mission, "chair", 3.5, -1.0, 3.0 * math.pi)
    assert moved.waypoint("chair").x == pytest.approx(3.5)
    # Wrapped into (-pi, pi]; +pi and -pi are the same heading.
    assert abs(abs(moved.waypoint("chair").yaw) - math.pi) < 1e-9
    assert moved.sequence == mission.sequence
    assert moved.waypoint("start") == mission.waypoint("start")
