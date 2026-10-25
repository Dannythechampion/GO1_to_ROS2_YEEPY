import math

import pytest

from omx_navigation.approach_planner import (
    DockGeometry,
    dock_standoff,
    ArmReach,
    StandoffPolicy,
    choose_approach_yaw,
    standoff_pose,
)


def policy(**overrides):
    base = dict(arm_base_offset_x=0.25, arm_base_height=0.10, approach_tolerance=0.05)
    base.update(overrides)
    return StandoffPolicy(**base)


def test_urdf_chain_reproduces_the_published_reach():
    """The link sum must match the 380 mm ROBOTIS publishes.

    If this drifts, the numbers were copied from the wrong joints: the chain is
    joint2->joint3 (hypot of 0.024 and 0.128), joint3->joint4 (0.124) and
    joint4->end_effector (0.126).
    """
    assert ArmReach().max_reach == pytest.approx(0.380, abs=0.001)


def test_usable_reach_stays_inside_the_envelope():
    arm = ArmReach()
    assert arm.usable_reach < arm.max_reach
    assert arm.usable_reach == pytest.approx(arm.max_reach * 0.75)


def test_horizontal_reach_shrinks_with_target_height():
    arm = ArmReach()
    level = arm.horizontal_reach(0.0)
    assert level == pytest.approx(arm.usable_reach)
    assert arm.horizontal_reach(0.10) < level
    assert arm.horizontal_reach(-0.10) == pytest.approx(arm.horizontal_reach(0.10))


def test_target_beyond_the_envelope_leaves_no_horizontal_reach():
    arm = ArmReach()
    assert arm.horizontal_reach(arm.usable_reach + 0.01) == 0.0


def test_standoff_is_budgeted_against_the_approach_stage_not_nav2():
    """The arm sees the approach stage's error, never Nav2's 0.35 m.

    Budgeting Nav2's tolerance out of a 0.38 m arm leaves nothing: that was the
    mistake in the first draft of the design. A 0.05 m approach tolerance
    leaves a usable standoff; a 0.35 m one does not.
    """
    height = policy().shoulder_height()
    assert policy(approach_tolerance=0.05).standoff_distance(height) > 0.0
    with pytest.raises(ValueError, match="cannot reach"):
        policy(approach_tolerance=0.35).standoff_distance(height)


def test_standoff_distance_is_measured_from_body_nav():
    height = policy().shoulder_height()
    near = policy(arm_base_offset_x=0.10)
    far = policy(arm_base_offset_x=0.30)
    assert far.standoff_distance(height) - near.standoff_distance(height) == pytest.approx(0.20)


def test_reach_budget_goes_negative_before_standoff_raises():
    """Callers check the sign; the number itself is meaningless once negative."""
    tight = policy(approach_tolerance=0.40)
    height = tight.shoulder_height()
    assert tight.reach_budget(height) < 0.0


def test_standoff_pose_sits_behind_the_target_and_faces_it():
    x, y, yaw = standoff_pose(2.0, 1.0, 0.0, 0.5)
    assert (x, y) == pytest.approx((1.5, 1.0))
    assert yaw == pytest.approx(0.0)
    # Facing the target means the heading points from the base to the target.
    assert math.atan2(1.0 - y, 2.0 - x) == pytest.approx(yaw)


def angle_gap(first, second):
    """Compare headings without tripping over the +-pi seam."""
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


def test_standoff_pose_follows_the_approach_heading():
    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7):
        x, y, out = standoff_pose(3.0, -1.0, yaw, 0.4)
        assert math.hypot(3.0 - x, -1.0 - y) == pytest.approx(0.4)
        assert angle_gap(math.atan2(-1.0 - y, 3.0 - x), out) < 1e-9


def test_standoff_pose_rejects_unusable_inputs():
    with pytest.raises(ValueError):
        standoff_pose(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        standoff_pose(float("nan"), 0.0, 0.0, 0.5)


def test_approach_yaw_skips_blocked_sides():
    # Everything with x below the target is blocked, so the base must come
    # from the far side.
    chosen = choose_approach_yaw(0.0, 0.0, 0.5, lambda x, y: x > 0.1)
    assert chosen is not None
    x, _, _ = chosen
    assert x > 0.1


def test_approach_yaw_prefers_the_stable_heading():
    """With everything free the preferred heading wins, so the side does not
    flip between calls as the costmap updates."""
    chosen = choose_approach_yaw(0.0, 0.0, 0.5, lambda x, y: True, preferred_yaw=1.2)
    assert chosen is not None
    assert abs(chosen[2] - 1.2) < 2.0 * math.pi / 24


def test_approach_yaw_reports_when_nothing_is_free():
    assert choose_approach_yaw(0.0, 0.0, 0.5, lambda x, y: False) is None


def test_mounted_arm_gives_a_workable_standoff():
    """The 2026-10-25 build: arm 0.15 m ahead of the LiDAR and 0.19 m below it."""
    mounted = StandoffPolicy()
    assert mounted.shoulder_height() == pytest.approx(-0.1305)
    assert mounted.standoff_distance(mounted.shoulder_height()) == pytest.approx(0.385, abs=0.001)


def test_floor_is_out_of_reach_from_the_current_mount():
    """Targets have to be on a raised surface, or the arm has to move.

    The shoulder sits 0.1305 m below the sensor and the usable radius is about
    0.285 m, so nothing below -0.4157 m in body_nav can be reached at any
    standoff. The sensor stands roughly 0.5-0.6 m above the floor, which puts
    the floor 0.08-0.18 m outside that. This is a property of the mount, not a
    tuning choice, so it is pinned here rather than discovered on the robot.
    """
    low, high = StandoffPolicy().reachable_height_range()
    assert low == pytest.approx(-0.4157, abs=0.001)
    assert high == pytest.approx(0.1547, abs=0.001)
    for sensor_height_above_floor in (0.50, 0.55, 0.60):
        assert -sensor_height_above_floor < low


LIDAR_ABOVE_ROBOT_TOP = 0.13


def test_target_on_a_chair_is_reachable_up_to_a_quarter_metre_above_the_robot():
    """The planned case: an object on a chair, a little above the standing Go1.

    The LiDAR sits directly on the 13 cm riser on the Go1's top, so a target
    `d` above that top is at `d - 0.13` in body_nav. The arm covers that from
    the top of the robot up to roughly a quarter metre above it, and the
    standoff shrinks as the target rises because the spherical envelope spends
    its radius on height. Past that the arm cannot reach at any standoff, which
    is a property of the mount, not a tuning choice.
    """
    mounted = StandoffPolicy()
    low, high = mounted.reachable_height_range()
    for above_top in (0.0, 0.05, 0.10, 0.20, 0.25):
        height = above_top - LIDAR_ABOVE_ROBOT_TOP
        assert low <= height <= high
        assert mounted.standoff_distance(height) > 0.0
    # Standoff shrinks monotonically as the target climbs.
    assert mounted.standoff_distance(0.0 - LIDAR_ABOVE_ROBOT_TOP) > mounted.standoff_distance(
        0.25 - LIDAR_ABOVE_ROBOT_TOP
    )
    # And the arm runs out before a third of a metre above the robot's top.
    assert 0.30 - LIDAR_ABOVE_ROBOT_TOP > high


def test_minimum_tape_distance_keeps_the_nose_off_the_seat():
    geometry = DockGeometry(clearance=0.0)
    assert geometry.minimum_tape_distance() == pytest.approx(0.82)
    assert DockGeometry(clearance=0.05).minimum_tape_distance() == pytest.approx(0.87)


def test_the_arm_reaches_barely_past_the_robots_own_nose():
    """The arm base is 0.15 m ahead of the LiDAR, the nose is at 0.32 m.

    Most of a 0.38 m arm is spent clearing the robot's own body, so the usable
    reach ends only a few centimetres past the nose. That is what makes the
    chair tight, and it is a property of the mount rather than of tuning.
    """
    mounted = StandoffPolicy()
    assert mounted.standoff_distance(mounted.shoulder_height()) - 0.32 < 0.08


def test_stopping_error_is_spent_against_both_limits():
    """Collision and reach squeeze the band from opposite sides.

    The tape has to sit a tolerance beyond the collision limit, because the
    stopping error is spent coming closer; reach caps it from the other side.
    With the object on the seat's front edge the bare gap is 0.035 m, so the
    tolerance has to be well inside that -- at 0.05 the band is negative.
    """
    height = StandoffPolicy().shoulder_height()
    flush = DockGeometry(clearance=0.0)
    with pytest.raises(ValueError, match="collision and reach"):
        dock_standoff(StandoffPolicy(approach_tolerance=0.05), flush, height, 0.50)
    assert dock_standoff(StandoffPolicy(), flush, height, 0.50) > 0.0


def test_the_default_build_leaves_a_few_centimetres_of_clearance():
    """What the robot actually gets with the object on the seat's edge."""
    mounted, geometry = StandoffPolicy(), DockGeometry(clearance=0.0)
    tape = dock_standoff(mounted, geometry, mounted.shoulder_height(), 0.50)
    seat_edge = tape - geometry.seat_depth
    clearance = seat_edge - geometry.robot_front_offset
    assert 0.03 < clearance < 0.06


def test_the_failure_names_the_tolerance_to_tighten():
    mounted = StandoffPolicy(approach_tolerance=0.05)
    with pytest.raises(ValueError, match="Tighten approach_tolerance"):
        dock_standoff(mounted, DockGeometry(clearance=0.0), mounted.shoulder_height(), 0.50)


def test_an_object_set_back_on_the_seat_is_out_of_reach():
    """Only the seat's front edge works with the arm where it is."""
    mounted, geometry = StandoffPolicy(), DockGeometry(clearance=0.0)
    height = mounted.shoulder_height()
    with pytest.raises(ValueError, match="collision and reach"):
        dock_standoff(mounted, geometry, height, 0.40)


def test_moving_the_arm_forward_opens_the_window():
    """A centimetre of mount is a centimetre of reach."""
    moved = StandoffPolicy(arm_base_offset_x=0.215)
    height = moved.shoulder_height()
    # Five centimetres of clearance and the object five back from the edge.
    assert dock_standoff(moved, DockGeometry(clearance=0.05), height, 0.45) > 0.0


def test_object_further_out_than_the_seat_is_rejected():
    mounted = StandoffPolicy()
    with pytest.raises(ValueError, match="seat is deep"):
        dock_standoff(mounted, DockGeometry(), mounted.shoulder_height(), 0.80)


def test_policy_rejects_impossible_values():
    with pytest.raises(ValueError):
        policy(approach_tolerance=-0.01)
    with pytest.raises(ValueError):
        ArmReach(usable_fraction=0.0)
    with pytest.raises(ValueError):
        ArmReach(elbow_to_wrist=0.0)
