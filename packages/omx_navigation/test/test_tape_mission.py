"""Start and goal marked with tape, Nav2 between them, the tape closing the rest."""

import math
from pathlib import Path

import pytest

from omx_navigation.corridor_dock import (
    CorridorDockPolicy,
    CorridorFix,
    corridor_fix,
    forward_clearance,
    median_fix,
    robot_pose_at,
    survey_strips,
    tape_pose,
    tape_span,
    unmeasured_corridor,
)
from omx_navigation.mission import mission_from_dict, mission_to_dict, record_waypoint
from omx_navigation.mission_file import load_mission, save_mission
from omx_navigation.mission_runner import ActionKind, MissionRunner, MissionState
from omx_navigation.tape_mission_core import FrameWindow, navigation_outcome, split_frame

TEMPLATE = Path(__file__).resolve().parents[1] / "config" / "tape_mission.yaml"


def strip(cx, cy, z_lo=-0.12, z_hi=0.30, n=40, bright=250.0):
    """A vertical strip on a wall: narrow along the corridor, tall."""
    return [(cx, cy, z_lo + (z_hi - z_lo) * i / (n - 1), bright) for i in range(n)]


def pair(ahead=0.0, lateral=0.0, width=2.0, **kwargs):
    return strip(ahead, width / 2 - lateral, **kwargs) + strip(ahead, -width / 2 - lateral, **kwargs)


def recorded(start_sep=2.0, goal_sep=1.6):
    data = {
        "waypoints": {
            "start": {"x": 0.0, "y": 0.0, "yaw": 0.0, "marker_separation": start_sep},
            "goal": {"x": 8.0, "y": 0.5, "yaw": 0.0, "marker_separation": goal_sep},
        },
        "sequence": [{"navigate": "goal"}, "dock", {"navigate": "start"}, "dock"],
    }
    return mission_from_dict(data)


def settle(item, t, fix):
    action = None
    for _ in range(60):
        t += 0.1
        action = item.update(t, fix)
        if action.kind is not ActionKind.DRIVE:
            break
    return action, t


# -- the tape --------------------------------------------------------------


def test_tape_span_covers_the_band_with_the_walking_pitch():
    bottom, top = tape_span(0.55)
    assert bottom == pytest.approx(0.55 - 0.10 - 0.052, abs=0.002)
    assert top == pytest.approx(0.55 + 0.20 + 0.052, abs=0.002)


def test_a_50cm_strip_from_35_to_85cm_covers_the_whole_height_estimate():
    """The LiDAR's height above the floor was never measured; 0.50-0.60 m.

    Taping 0.35-0.85 m from the floor covers the band at both ends of that
    estimate, so the strip works without anyone measuring first.
    """
    for lidar_height in (0.50, 0.55, 0.60):
        bottom, top = tape_span(lidar_height)
        assert bottom >= 0.345 and top <= 0.855


def test_survey_reports_how_much_of_each_strip_the_detector_uses():
    cloud = pair(z_lo=-0.12, z_hi=0.30)
    surveys = survey_strips(cloud)
    assert len(surveys) == 2
    for item in surveys:
        assert item.z_lo == pytest.approx(-0.12)
        assert item.z_hi == pytest.approx(0.30)
        assert 0 < item.in_band < item.points
        assert item.range == pytest.approx(1.0, abs=0.01)


def test_a_strip_hung_too_high_is_seen_by_the_survey_but_not_the_detector():
    high = pair(z_lo=0.25, z_hi=0.60)
    assert corridor_fix(high) is None
    assert len(survey_strips(high)) == 2


def test_vertical_strips_dock_like_the_horizontal_ones():
    fix = corridor_fix(pair(ahead=0.2, lateral=0.1))
    assert fix is not None
    assert fix.mid_x == pytest.approx(0.2, abs=0.01)
    assert fix.mid_y == pytest.approx(-0.1, abs=0.01)


# -- recording a spot --------------------------------------------------------


def test_recording_names_the_tape_not_where_the_robot_happened_to_stand():
    tape = (5.0, 2.0, math.radians(30))
    for robot_offset, turned in (((0.0, 0.0), 0.0), ((0.25, -0.1), math.radians(6))):
        robot_yaw = tape[2] - turned
        # The robot stands `robot_offset` from the tape in its own frame, so
        # it sees the midpoint at minus that.
        fix = CorridorFix(-robot_offset[0], -robot_offset[1], turned, 2.0, 80)
        robot_x = tape[0] + math.cos(robot_yaw) * robot_offset[0] - math.sin(robot_yaw) * robot_offset[1]
        robot_y = tape[1] + math.sin(robot_yaw) * robot_offset[0] + math.cos(robot_yaw) * robot_offset[1]
        x, y, yaw = tape_pose(robot_x, robot_y, robot_yaw, fix)
        assert (x, y) == pytest.approx(tape[:2], abs=1e-6)
        assert yaw == pytest.approx(tape[2], abs=1e-6)


def test_start_tape_gives_back_the_robots_pose():
    fix = CorridorFix(0.12, -0.05, math.radians(-4), 2.0, 80)
    robot = (1.5, -3.0, math.radians(100))
    spot = tape_pose(*robot, fix)
    assert robot_pose_at(*spot, fix) == pytest.approx(robot, abs=1e-9)


def test_median_fix_ignores_one_bad_frame():
    good = CorridorFix(0.10, 0.02, 0.01, 2.0, 60)
    bad = CorridorFix(1.50, -0.80, 0.40, 2.3, 21)
    fix = median_fix([good, good, bad, None])
    assert fix.mid_x == pytest.approx(0.10)
    assert fix.mid_y == pytest.approx(0.02)
    assert median_fix([None, None]) is None


def test_a_recorded_spot_keeps_its_wall_gap_through_the_file(tmp_path):
    mission = record_waypoint(recorded(), "goal", 7.9, 0.4, 0.1, 1.62)
    path = tmp_path / "nested" / "mission.yaml"
    save_mission(path, mission)
    text = path.read_text(encoding="utf-8")
    assert text.startswith("# Written by")
    again = load_mission(path)
    assert again.waypoint("goal").marker_separation == pytest.approx(1.62)
    assert mission_to_dict(again) == mission_to_dict(mission)
    assert not list(tmp_path.rglob("*.tmp"))


def test_the_template_refuses_to_run_until_both_spots_are_recorded():
    mission = load_mission(TEMPLATE)
    assert [step.kind for step in mission.sequence] == ["navigate", "dock", "navigate", "dock"]
    assert mission.untaped_docks() == ["goal", "start"]
    mission = record_waypoint(mission, "goal", 8.0, 0.5, 0.0, 1.6)
    assert mission.untaped_docks() == ["start"]
    mission = record_waypoint(mission, "start", 0.0, 0.0, 0.0, 2.0)
    assert mission.untaped_docks() == []


def test_marker_separation_must_be_positive():
    with pytest.raises(ValueError):
        mission_from_dict({
            "waypoints": {"a": {"x": 0, "y": 0, "yaw": 0, "marker_separation": 0.0}},
            "sequence": [{"navigate": "a"}],
        })


# -- the run ------------------------------------------------------------------


def test_goal_and_back_both_end_on_their_tape():
    item = MissionRunner(recorded())
    action = item.start(0.0)
    assert action.kind is ActionKind.NAVIGATE and action.waypoint.name == "goal"
    assert item.on_navigation_result(True, 1.0).kind is ActionKind.DRIVE
    assert item.active_dock_policy.markers.strip_separation == pytest.approx(1.6)
    action, t = settle(item, 1.0, CorridorFix(0.0, 0.0, 0.0, 1.6, 80))
    assert action.kind is ActionKind.NAVIGATE and action.waypoint.name == "start"
    assert item.on_navigation_result(True, t + 1).kind is ActionKind.DRIVE
    assert item.active_dock_policy.markers.strip_separation == pytest.approx(2.0)
    action, _ = settle(item, t + 1, CorridorFix(0.0, 0.0, 0.0, 2.0, 80))
    assert action.kind is ActionKind.DONE
    assert item.state is MissionState.DONE


def test_each_dock_is_held_to_its_own_measured_gap():
    """A 1.6 m corridor fails the 2.00 +- 0.30 default and passes once recorded."""
    narrow = pair(width=1.6)
    assert corridor_fix(narrow, CorridorDockPolicy()) is None
    held = CorridorDockPolicy().with_separation(1.6)
    assert corridor_fix(narrow, held) is not None
    # And the recorded gap rejects the other spot's pair.
    assert corridor_fix(pair(width=2.0), held) is None


def test_a_spot_of_any_reasonable_width_can_be_recorded():
    """Recording is what measures the gap, so it cannot demand the default."""
    for width in (1.2, 1.6, 2.0, 2.6):
        fix = corridor_fix(pair(width=width), unmeasured_corridor())
        assert fix is not None and fix.width == pytest.approx(width, abs=0.01)


def test_markers_far_from_where_nav2_stopped_abort_instead_of_driving():
    item = MissionRunner(recorded())
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    action = item.update(1.1, CorridorFix(0.9, 0.0, 0.0, 1.6, 80))
    assert action.kind is ActionKind.ABORT
    assert "max_correction" in action.reason


def test_something_close_behind_stops_the_dock():
    item = MissionRunner(recorded())
    item.start(0.0)
    item.on_navigation_result(True, 1.0)
    t, action = 1.0, None
    while item.state is MissionState.DOCKING and t < 5.0:
        t += 0.1
        action = item.update(t, CorridorFix(-0.2, 0.0, 0.0, 1.6, 80),
                             rear_clearance=0.05, minimum_clearance=0.10)
    assert action.kind is ActionKind.ABORT
    assert "rear" in action.reason


def test_operator_cancel_stops_the_mission():
    item = MissionRunner(recorded())
    item.start(0.0)
    action = item.abort("cancelled by the operator")
    assert action.kind is ActionKind.ABORT
    assert item.state is MissionState.ABORTED


def test_clearance_ignores_the_robots_own_body_and_can_look_behind():
    cloud = [(0.20, 0.0, 0.0, 30.0),      # the arm, inside the nose
             (0.50, 0.0, 0.0, 30.0),      # something ahead
             (-0.45, 0.0, 0.0, 30.0)]     # something behind
    assert forward_clearance(cloud) == pytest.approx(0.20)
    assert forward_clearance(cloud, ignore_within=0.32) == pytest.approx(0.18 + 0.32)
    assert forward_clearance(cloud, ignore_within=0.33, behind=True) == pytest.approx(0.45)


# -- the node's ROS-free parts ---------------------------------------------


def test_split_frame_keeps_only_tape_and_the_channel():
    points = [(1.0, 1.0, 0.0, 250.0),    # tape on the wall
              (0.5, 0.0, 0.0, 30.0),     # ahead, in the channel
              (0.5, 0.0, 1.0, 30.0),     # ahead, above the band
              (3.0, 2.0, 0.0, 30.0)]     # wall, dull
    bright, channel = split_frame(points, 180.0, 0.20, -0.15, 0.25)
    assert bright == [(1.0, 1.0, 0.0, 250.0)]
    assert channel == [(0.5, 0.0, 0.0, 30.0)]


def test_frame_window_merges_only_recent_frames():
    window = FrameWindow(0.2)
    window.add(0.00, [(1, 1, 0, 250)], [])
    window.add(0.10, [(1, 1, 0, 251)], [(0.5, 0, 0, 1)])
    assert len(window.bright()) == 2
    window.add(0.25, [(1, 1, 0, 252)], [])
    assert [p[3] for p in window.bright()] == [251, 252]
    assert window.channel() == [(0.5, 0, 0, 1)]
    with pytest.raises(ValueError):
        FrameWindow(0.0)


def test_navigation_outcome_matches_the_goal_it_sent():
    goal = recorded().waypoint("goal")
    status = {"state": "SUCCEEDED", "goal": {"x": 8.0, "y": 0.5}}
    assert navigation_outcome(status, goal) is True
    assert navigation_outcome({**status, "state": "ACCEPTED"}, goal) is None
    for state in ("ABORTED", "CANCELED", "REJECTED", "IGNORED", "PREEMPTED"):
        assert navigation_outcome({**status, "state": state}, goal) is False
    # Somebody else's goal says nothing about ours.
    assert navigation_outcome({"state": "SUCCEEDED", "goal": {"x": 1.0, "y": 0.5}}, goal) is None
    assert navigation_outcome({"state": "SUCCEEDED", "goal": {"x": None, "y": None}}, goal) is None
    assert navigation_outcome("not a dict", goal) is None
