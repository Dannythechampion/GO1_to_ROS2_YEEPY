import math

import pytest

from omx_navigation.corridor_dock import (
    CorridorDockPolicy,
    corridor_fix,
    forward_clearance,
)


def marker(cx, cy, n=30, bright=240.0, z=0.05, span=0.20):
    """A tape strip running along a wall."""
    return [(cx + (i / (n - 1) - 0.5) * span, cy, z, bright) for i in range(n)]


def corridor(ahead=0.0, lateral=0.0, yaw=0.0, width=2.00, **kwargs):
    """Markers abeam at `width` apart, then displaced as the robot would see them.

    `ahead` is how far the robot still has to travel, `lateral` how far it sits
    off the centreline, `yaw` how far it is turned.
    """
    out = []
    for cy in (+width / 2, -width / 2):
        for x, y, z, r in marker(ahead, cy - lateral, **kwargs):
            out.append((x * math.cos(yaw) + y * math.sin(yaw),
                        -x * math.sin(yaw) + y * math.cos(yaw), z, r))
    return out


def clutter(n=500, bright=60.0):
    return [(math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n), 0.05, bright)
            for i in range(n)]


def test_parked_square_reports_no_error():
    fix = corridor_fix(corridor() + clutter())
    assert fix is not None
    assert fix.mid_x == pytest.approx(0.0, abs=0.01)
    assert fix.mid_y == pytest.approx(0.0, abs=0.01)
    assert fix.yaw_error == pytest.approx(0.0, abs=math.radians(1))
    assert fix.width == pytest.approx(2.00, abs=0.02)


def test_markers_still_ahead_read_as_forward_error():
    """The controller drives the midpoint onto stop_offset, so this is the cue."""
    fix = corridor_fix(corridor(ahead=0.25))
    assert fix is not None
    assert fix.mid_x == pytest.approx(0.25, abs=0.01)


def test_sitting_off_the_centreline_reads_as_lateral_error():
    fix = corridor_fix(corridor(lateral=0.12))
    assert fix is not None
    assert fix.mid_y == pytest.approx(-0.12, abs=0.01)


def test_yaw_error_is_the_correction_not_the_robots_heading():
    """`yaw_error` is what the robot would have to turn to square up.

    A base turned counter-clockwise reports a clockwise correction. The arm
    consumes it directly, since joint1 applies the same correction.
    """
    for turned_by in (math.radians(6), math.radians(-6)):
        fix = corridor_fix(corridor(yaw=turned_by))
        assert fix is not None
        assert fix.yaw_error == pytest.approx(-turned_by, abs=math.radians(1))


def test_yaw_sign_does_not_depend_on_which_marker_sorts_first():
    """Ordering left-then-right keeps the sign stable."""
    straight = corridor_fix(corridor())
    flipped = corridor_fix(list(reversed(corridor())))
    assert straight is not None and flipped is not None
    assert straight.yaw_error == pytest.approx(flipped.yaw_error, abs=1e-9)


def test_large_yaw_is_reported_but_flagged():
    policy = CorridorDockPolicy()
    small = corridor_fix(corridor(yaw=math.radians(5)), policy)
    large = corridor_fix(corridor(yaw=math.radians(20)), policy)  # beyond the arm
    assert small is not None and small.yaw_within(policy)
    assert large is not None and not large.yaw_within(policy)


def test_ordinary_scene_alone_detects_nothing():
    assert corridor_fix(clutter(n=2000, bright=60.0)) is None
    assert corridor_fix(clutter(n=2000, bright=150.0)) is None


def test_one_wall_occluded_is_rejected():
    assert corridor_fix(marker(0.0, 1.0)) is None


def test_wrong_corridor_width_is_rejected():
    assert corridor_fix(corridor(width=1.0)) is None
    assert corridor_fix(corridor(width=3.2)) is None


def test_dim_markers_are_ignored():
    assert corridor_fix(corridor(bright=170.0)) is None
    assert corridor_fix(corridor(bright=190.0)) is not None


def test_forward_clearance_sees_the_chair_and_ignores_the_walls():
    cloud = corridor()                       # walls to the sides, abeam
    cloud += [(0.42, 0.05, 0.0, 30.0)]       # the chair, dead ahead
    assert forward_clearance(cloud) == pytest.approx(0.42, abs=0.01)


def test_forward_clearance_is_infinite_when_the_channel_is_clear():
    """Never a small number by accident: that would read as an obstacle."""
    assert forward_clearance(corridor()) == math.inf


def test_forward_clearance_ignores_returns_outside_the_channel():
    cloud = [(0.40, 0.60, 0.0, 30.0),        # wide of the robot
             (0.40, 0.00, 0.90, 30.0),       # above the band
             (-0.40, 0.00, 0.0, 30.0)]       # behind
    assert forward_clearance(cloud) == math.inf


def test_policy_rejects_impossible_values():
    with pytest.raises(ValueError):
        CorridorDockPolicy(max_absorbable_yaw=0.0)
    with pytest.raises(ValueError):
        forward_clearance([], half_width=0.0)
    with pytest.raises(ValueError):
        forward_clearance([], band_lo=0.5, band_hi=0.1)


def test_bright_things_ahead_and_behind_do_not_veto_the_wall_pair():
    """The arm, the chair and the corridor ends sit outside the side window."""
    ahead = [(0.6, (i / 29 - 0.5) * 0.2, 0.05, 240.0) for i in range(30)]
    behind = [(-1.5, (i / 29 - 0.5) * 0.2, 0.05, 240.0) for i in range(30)]
    assert corridor_fix(corridor() + ahead + behind) is not None


def test_taught_policy_drives_back_to_where_the_fix_was_read():
    from omx_navigation.corridor_dock import taught_policy

    taught = corridor_fix(corridor(ahead=0.04, lateral=0.03, yaw=math.radians(2)))
    policy = taught_policy(CorridorDockPolicy(), taught)
    assert policy.stop_offset_x == pytest.approx(taught.mid_x)
    assert policy.stop_offset_y == pytest.approx(taught.mid_y)
    assert policy.yaw_bias == pytest.approx(taught.yaw_error)
    assert taught.yaw_within(policy)


def test_forward_clearance_skips_the_robots_own_arm():
    cloud = [(0.20, 0.0, 0.0, 90.0), (0.30, 0.05, 0.0, 90.0)]   # arm and basket
    assert forward_clearance(cloud) == math.inf
    assert forward_clearance(cloud + [(0.42, 0.0, 0.0, 30.0)]) == pytest.approx(0.42)
    assert forward_clearance(cloud, ignore_within=0.0) == pytest.approx(0.20)
