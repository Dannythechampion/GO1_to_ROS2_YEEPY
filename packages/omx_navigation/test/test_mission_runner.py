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
    """A missing detection is a stop, never a coast -- and, past the
    detection timeout, the end of the run."""
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, None)
    assert action.kind is ActionKind.DRIVE
    assert (action.vx, action.vy) == (0.0, 0.0)
    action = item.update(2.0, None)
    assert action.kind is ActionKind.ABORT
    assert "no detection" in action.reason


def test_an_old_scan_is_judged_by_its_own_stamp():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(2.0, fix(mid_x=0.30), fix_stamp=1.0)
    assert action.kind is ActionKind.ABORT
    assert "detection stale" in action.reason


def test_grasp_carries_what_docking_left_over():
    item = runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    t = 1.0
    action = None
    while item.state is MissionState.DOCKING:
        t += 0.1
        action = item.update(t, fix(mid_x=0.01))
    assert action.kind is ActionKind.GRASP
    assert action.residual == pytest.approx((0.01, 0.0, 0.0))


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


# --- docking on the chair's latched front edge -------------------------------

import math  # noqa: E402

from omx_navigation.chair_front import (  # noqa: E402
    ChairFrontPolicy,
    OdomPose,
    StoppingModel,
    measure_front,
)
from omx_navigation.mission_runner import ChairDockPolicy  # noqa: E402

from test_chair_front import scene  # noqa: E402


def stopping(**overrides):
    values = {"latency": 0.3, "overshoot": 0.005, "min_clearance": 0.01}
    values.update(overrides)
    return StoppingModel(**values)


def chair_runner(gap=0.06, target="chair", **overrides):
    policy = ChairDockPolicy(
        stopping=stopping(**overrides), standoff_gap=gap, target=target,
        front=ChairFrontPolicy(min_frames=3),
    )
    return MissionRunner(mission_from_dict(SCENARIO), chair_policy=policy)


def at_chair(item, edge=1.0):
    """Navigate, then latch with the chair edge `edge` ahead of a still robot."""
    item.start(0.0)
    action = item.on_navigation_result(True, 1.0)
    assert item.state is MissionState.LATCHING
    pose = OdomPose(0.0, 0.0, 0.0)
    t = 1.0
    while item.state is MissionState.LATCHING:
        t += 0.1
        action = item.update(t, chair=measure_front(scene(distance=edge)), pose=pose)
        assert (action.vx, action.vy) == (0.0, 0.0)
    return action, t


def test_a_standoff_the_stop_cannot_honour_is_refused_at_construction():
    """With 0.3 s of latency the 2026-10-26 band (0.035 m) does not fit."""
    with pytest.raises(ValueError, match="standoff_gap"):
        chair_runner(gap=0.035)


def test_docking_waits_for_a_latched_edge_before_moving():
    item = chair_runner()
    action, _ = at_chair(item)
    assert action.reason == "chair front latched"
    assert item.state is MissionState.DOCKING
    assert item.latched_front.x == pytest.approx(1.0, abs=0.01)


def test_an_edge_that_never_latches_ends_the_run():
    item = chair_runner()
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    pose = OdomPose(0.0, 0.0, 0.0)
    action = item.update(2.0, chair=None, pose=pose)
    assert action.kind is ActionKind.DRIVE
    action = item.update(7.0, chair=None, pose=pose)
    assert action.kind is ActionKind.ABORT
    assert action.reason == "chair front not latched"


def drive(item, t, pose, cycles=400, dt=0.1, slide=0.0):
    """Integrate the commanded velocity; `slide` adds a stop that runs long."""
    closest = math.inf
    action = None
    for _ in range(cycles):
        t += dt
        action = item.update(t, pose=pose)
        if action.kind is not ActionKind.DRIVE:
            break
        x = pose.x + action.vx * dt + (slide if action.vx > 0 else 0.0)
        pose = OdomPose(x, pose.y + action.vy * dt, pose.yaw)
        front = item.latched_front
        closest = min(closest, front.nose_clearance(pose, 0.32, 0.15))
    return action, pose, closest


def test_the_chair_dock_stops_at_the_standoff_and_hands_over_the_residual():
    item = chair_runner(gap=0.06)
    _, t = at_chair(item, edge=1.0)
    action, pose, closest = drive(item, t, OdomPose(0.0, 0.0, 0.0))
    assert action.kind is ActionKind.GRASP
    # Nose 0.06 from an edge 1.0 ahead: body_nav ends near 0.62.
    assert pose.x == pytest.approx(0.62, abs=0.02)
    assert closest >= 0.01
    assert action.residual is not None
    assert math.hypot(action.residual[0], action.residual[1]) <= 0.02


def test_a_robot_that_overshoots_is_stopped_by_the_envelope_not_the_chair():
    """Every forward step slides 5 mm further than commanded."""
    item = chair_runner(gap=0.06)
    _, t = at_chair(item, edge=1.0)
    action, pose, closest = drive(item, t, OdomPose(0.0, 0.0, 0.0), slide=0.005)
    assert closest >= 0.0


def test_crossing_min_clearance_aborts():
    item = chair_runner(gap=0.06)
    _, t = at_chair(item, edge=1.0)
    action = item.update(t + 0.1, pose=OdomPose(0.675, 0.0, 0.0))
    assert action.kind is ActionKind.ABORT
    assert action.reason == "chair clearance breached"


def test_docking_without_a_pose_aborts():
    item = chair_runner()
    _, t = at_chair(item)
    action = item.update(t + 0.1)
    assert action.kind is ActionKind.ABORT
    assert action.reason == "odometry pose missing"


def test_tape_target_is_still_bounded_by_the_latched_edge():
    """Markers placed wrong would drive into the chair; the edge stops it."""
    item = chair_runner(gap=0.06, target="tape")
    _, t = at_chair(item, edge=1.0)
    # Nose 0.03 from the edge, and the markers claim another 0.30 m to go.
    action = item.update(t + 0.1, fix(mid_x=0.30), pose=OdomPose(0.65, 0.0, 0.0))
    assert action.kind is ActionKind.DRIVE
    assert action.reason == "limited by stopping distance"
    assert action.vx < 0.08
