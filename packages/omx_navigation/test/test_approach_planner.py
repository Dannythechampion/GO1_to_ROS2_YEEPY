import math

import pytest

from omx_navigation.approach_planner import (
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


def test_policy_rejects_impossible_values():
    with pytest.raises(ValueError):
        policy(approach_tolerance=-0.01)
    with pytest.raises(ValueError):
        ArmReach(usable_fraction=0.0)
    with pytest.raises(ValueError):
        ArmReach(elbow_to_wrist=0.0)
