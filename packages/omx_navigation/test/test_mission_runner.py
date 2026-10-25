import pytest

from omx_navigation.corridor_dock import CorridorFix
from omx_navigation.mission import mission_from_dict
from omx_navigation.mission_runner import (
    ActionKind,
    MissionRunner,
    MissionState,
)


SCENARIO = {
    "waypoints": {
        "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "chair": {"x": 3.2, "y": -1.1, "yaw": 1.5708},
    },
    "sequence": [{"navigate": "chair"}, "dock", "grasp", {"navigate": "start"}],
}


def runner():
    return MissionRunner(mission_from_dict(SCENARIO))


def fix(mid_x=0.0, mid_y=0.0):
    return CorridorFix(mid_x, mid_y, 0.0, 2.0, 60)


def test_the_scenario_runs_end_to_end():
    item = runner()
    action = item.start(0.0)
    assert action.kind is ActionKind.NAVIGATE
    assert action.waypoint.name == "chair"

    action = item.on_navigation_result(True, 1.0)
    assert action.kind is ActionKind.DRIVE
    assert item.state is MissionState.DOCKING

    # Settle on the markers.
    t = 1.0
    for _ in range(10):
        t += 0.1
        action = item.update(t, fix())
        if action.kind is not ActionKind.DRIVE:
            break
    assert action.kind is ActionKind.GRASP
    assert item.state is MissionState.GRASPING

    action = item.on_grasp_result(True, t)
    assert action.kind is ActionKind.NAVIGATE
    assert action.waypoint.name == "start"

    assert item.on_navigation_result(True, t + 1).kind is ActionKind.DONE
    assert item.state is MissionState.DONE


def test_docking_drives_toward_the_markers():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, fix(mid_x=0.30))
    assert action.kind is ActionKind.DRIVE
    assert action.vx > 0.0


def test_failed_navigation_stops_the_mission():
    item = runner()
    item.start(0.0)
    action = item.on_navigation_result(False, 1.0)
    assert action.kind is ActionKind.ABORT
    assert action.reason == "navigation failed"
    assert item.state is MissionState.ABORTED


def test_failed_grasp_stops_the_mission():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    t = 1.0
    while item.state is MissionState.DOCKING:
        t += 0.1
        item.update(t, fix())
    assert item.on_grasp_result(False, t).reason == "grasp failed"
    assert item.state is MissionState.ABORTED


def test_losing_the_markers_stops_the_mission():
    """A missing detection is a stop, never a coast."""
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, None)
    assert action.kind is ActionKind.ABORT
    assert "no detection" in action.reason


@pytest.mark.parametrize(
    "kwargs, fragment",
    [
        ({"manual_override": True}, "manual override"),
        ({"execution_fault": True}, "execution fault"),
        ({"localization_ready": False}, "localization not ready"),
    ],
)
def test_safety_inputs_stop_the_mission(kwargs, fragment):
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, fix(mid_x=0.30), **kwargs)
    assert action.kind is ActionKind.ABORT
    assert fragment in action.reason


def test_forward_clearance_stops_the_mission_before_the_control_law():
    """The net under the loop: the chair is watched directly."""
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(
        1.1, fix(mid_x=0.30), forward_clearance=0.30, minimum_clearance=0.35
    )
    assert action.kind is ActionKind.ABORT
    assert action.reason == "forward clearance breached"


def test_results_arriving_in_the_wrong_state_abort():
    item = runner()
    item.start(0.0)
    assert item.on_grasp_result(True, 1.0).kind is ActionKind.ABORT


def test_update_outside_docking_does_nothing():
    item = runner()
    item.start(0.0)
    assert item.update(1.0, fix()).kind is ActionKind.NONE


def test_reset_allows_another_run():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(False, 1.0)
    assert item.state is MissionState.ABORTED
    assert item.start(2.0).kind is ActionKind.NAVIGATE
    assert item.state is MissionState.NAVIGATING
