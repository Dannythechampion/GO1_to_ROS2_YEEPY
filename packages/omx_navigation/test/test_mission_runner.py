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

    # Settle, measure standing, arrive.
    t = 1.0
    for _ in range(40):
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
    action = None
    t = 1.0
    while t < 5.0:
        t += 0.1
        action = item.update(t, fix(mid_x=0.30))
        if action.vx:
            break
    assert action.kind is ActionKind.DRIVE
    assert action.vx > 0.0
    assert action.detail["phase"] == "DRIVE"


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
    t, action = 1.0, None
    while item.state is MissionState.DOCKING:
        t += 0.1
        action = item.update(t, None)
        assert (action.vx, action.vy) == (0.0, 0.0)
    assert action.kind is ActionKind.ABORT
    assert "never seen" in action.reason
    assert t < 1.0 + 4.0


@pytest.mark.parametrize(
    "kwargs, fragment",
    [
        ({"manual_override": True}, "manual override"),
        ({"execution_fault": True}, "not following"),
    ],
)
def test_safety_inputs_stop_the_mission(kwargs, fragment):
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, fix(mid_x=0.30), **kwargs)
    assert action.kind is ActionKind.ABORT
    assert fragment in action.reason


def test_losing_localization_pauses_then_gives_up():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    t, action = 1.0, None
    while item.state is MissionState.DOCKING:
        t += 0.1
        action = item.update(t, fix(mid_x=0.30), localization_ready=False)
        assert (action.vx, action.vy) == (0.0, 0.0)
    assert "READY" in action.reason
    assert t < 1.0 + 7.0


def test_forward_clearance_stops_a_dock_about_to_move_into_it():
    """The net under the loop: the chair is watched directly."""
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    t, action = 1.0, None
    while item.state is MissionState.DOCKING and t < 5.0:
        t += 0.1
        action = item.update(t, fix(mid_x=0.30), forward_clearance=0.30, minimum_clearance=0.35)
    assert action.kind is ActionKind.ABORT
    assert action.reason == "forward clearance breached"


def test_something_close_while_standing_does_not_abort():
    """Measuring is standing still; a passer-by behind is no reason to stop."""
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, fix(), rear_clearance=0.05, minimum_clearance=0.10)
    assert action.kind is ActionKind.DRIVE and action.detail["phase"] == "SETTLE"


def test_tape_handoff_replaces_waiting_for_nav2():
    item = runner()
    item.start(0.0)
    assert item.docks_next and item.navigation_target.name == "chair"
    action = item.on_tape_handoff(2.0)
    assert action.kind is ActionKind.DRIVE
    assert item.state is MissionState.DOCKING
    assert "handed" in action.reason


def test_failed_navigation_with_the_tape_in_view_still_docks():
    item = runner()
    item.start(0.0)
    action = item.on_navigation_result(False, 1.0, tape_in_view=True)
    assert action.kind is ActionKind.DRIVE and item.state is MissionState.DOCKING


def test_handoff_where_no_dock_follows_is_refused():
    item = runner()
    item.start(0.0)
    item.on_tape_handoff(1.0)
    t = 1.0
    while item.state is MissionState.DOCKING:
        t += 0.1
        item.update(t, fix())
    item.on_grasp_result(True, t)
    assert item.navigation_target.name == "start" and not item.docks_next
    assert item.on_tape_handoff(t).kind is ActionKind.ABORT


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
