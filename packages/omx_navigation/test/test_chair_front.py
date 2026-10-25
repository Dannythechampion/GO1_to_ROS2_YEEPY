import math

import pytest

from omx_navigation.chair_front import (
    ChairFrontPolicy,
    FrontLatch,
    LatchedFront,
    OdomPose,
    StoppingModel,
    limit_toward,
    measure_front,
)


def chair(distance=1.0, lateral=0.0, tilt=0.0, width=0.45, depth=0.45):
    """Seat front face and seat top, as the robot sees them in body_nav.

    `tilt` turns the chair about its own front centre, in radians.
    """
    out = []
    c, s = math.cos(tilt), math.sin(tilt)
    for i in range(16):
        u = (i / 15 - 0.5) * width
        for k in range(6):
            z = -0.25 + k * 0.04
            out.append((distance - s * u, lateral + c * u, z, 40.0))
        for k in range(1, 6):
            d = depth * k / 5
            out.append((distance - s * u + c * d, lateral + c * u + s * d, -0.04, 40.0))
    return out


def own_body():
    """The arm and basket, ahead of the LiDAR but behind the nose."""
    return [(0.15 + 0.01 * i, 0.02 * (i % 5) - 0.04, -0.05, 90.0) for i in range(15)]


def walls():
    return [(0.1 * i - 1.0, side, 0.0, 50.0) for i in range(30) for side in (-1.0, 1.0)]


def scene(**kwargs):
    return chair(**kwargs) + own_body() + walls()


def test_finds_the_seat_edge_and_ignores_the_robot_and_walls():
    reading = measure_front(scene(distance=1.0))
    assert reading is not None
    assert reading.front_x == pytest.approx(1.0, abs=0.005)
    assert reading.centre_y == pytest.approx(0.0, abs=0.01)
    assert reading.normal_yaw == pytest.approx(0.0, abs=math.radians(1))
    assert reading.width == pytest.approx(0.45, abs=0.02)


def test_off_centre_chair_reads_as_lateral_offset():
    reading = measure_front(scene(lateral=0.08))
    assert reading is not None
    assert reading.centre_y == pytest.approx(0.08, abs=0.02)


def test_a_turned_chair_reports_its_face_normal():
    reading = measure_front(scene(tilt=math.radians(8)))
    assert reading is not None
    assert reading.normal_yaw == pytest.approx(math.radians(8), abs=math.radians(1.5))


def test_one_stray_return_does_not_pull_the_edge_in():
    reading = measure_front(scene(distance=1.0) + [(0.55, 0.0, -0.10, 40.0)])
    assert reading is not None
    assert reading.front_x == pytest.approx(1.0, abs=0.01)


def test_range_bias_moves_the_edge_along_its_normal():
    reading = measure_front(scene(distance=1.0), ChairFrontPolicy(range_bias=-0.02))
    assert reading.front_x == pytest.approx(0.98, abs=0.005)


def test_nothing_ahead_reads_as_nothing():
    assert measure_front(own_body() + walls()) is None
    assert measure_front(scene(distance=2.5)) is None


def test_policy_rejects_impossible_values():
    for kwargs in ({"half_width": 0.0}, {"band_lo": 0.1, "band_hi": 0.0},
                   {"range_min": 2.0}, {"front_quantile": 0.6}, {"min_frames": 0},
                   {"range_bias": float("nan")}):
        with pytest.raises(ValueError):
            ChairFrontPolicy(**kwargs)


def test_pose_round_trip():
    pose = OdomPose(1.0, -2.0, 0.7)
    assert pose.to_body(*pose.to_odom(0.3, -0.1)) == pytest.approx((0.3, -0.1))


def test_latch_waits_for_enough_frames_and_fixes_the_edge_in_odom():
    latch = FrontLatch(ChairFrontPolicy(min_frames=3))
    pose = OdomPose(5.0, 1.0, math.pi / 2)
    reading = measure_front(scene(distance=1.0))
    assert latch.add(reading, pose) is None
    assert latch.add(None, pose) is None
    assert latch.add(reading, pose) is None
    latched = latch.add(reading, pose)
    assert latched is not None
    # One metre ahead of a robot facing +y.
    assert (latched.x, latched.y) == pytest.approx((5.0, 2.0), abs=0.01)
    assert latched.normal_yaw == pytest.approx(math.pi / 2, abs=0.02)


def test_latch_averages_through_body_sway():
    """Each scan is moved into odom with its own pose before averaging."""
    latch = FrontLatch(ChairFrontPolicy(min_frames=4))
    latched = None
    for sway in (0.004, -0.004, 0.004, -0.004):
        latched = latch.add(measure_front(scene(distance=1.0 - sway)), OdomPose(sway, 0.0, 0.0))
    assert latched is not None
    assert latched.x == pytest.approx(1.0, abs=0.005)


def test_latch_refuses_a_wandering_edge():
    latch = FrontLatch(ChairFrontPolicy(min_frames=4, max_spread=0.01))
    pose = OdomPose(0.0, 0.0, 0.0)
    for distance in (0.95, 1.05, 0.95, 1.05):
        assert latch.add(measure_front(scene(distance=distance)), pose) is None


def front_at(x=1.0, normal_yaw=0.0):
    return LatchedFront(x, 0.0, normal_yaw, 0.45, 10, 0.0)


def test_nose_clearance_uses_the_corner_that_touches_first():
    front = front_at(1.0)
    assert front.nose_clearance(OdomPose(0.0, 0.0, 0.0), 0.32, 0.15) == pytest.approx(0.68)
    turned = front.nose_clearance(OdomPose(0.0, 0.0, math.radians(10)), 0.32, 0.15)
    expected = 1.0 - (0.32 * math.cos(math.radians(10)) + 0.15 * math.sin(math.radians(10)))
    assert turned == pytest.approx(expected)


def test_standoff_leaves_the_requested_gap():
    front = front_at(1.0)
    x, y = front.standoff(0.03, 0.32)
    assert (x, y) == pytest.approx((0.65, 0.0))
    assert front.nose_clearance(OdomPose(x, y, 0.0), 0.32, 0.15) == pytest.approx(0.03)


def model(**overrides):
    values = {"latency": 0.3, "overshoot": 0.005, "min_clearance": 0.01}
    values.update(overrides)
    return StoppingModel(**values)


def test_stopping_model_has_no_guessed_defaults():
    with pytest.raises(TypeError):
        StoppingModel()
    with pytest.raises(ValueError):
        model(latency=0.0)
    with pytest.raises(ValueError):
        model(overshoot=-0.01)


def test_far_from_the_chair_the_command_passes_untouched():
    assert limit_toward(0.08, 0.0, OdomPose(0.0, 0.0, 0.0), front_at(1.0), model(), 0.32, 0.15) \
        == (0.08, 0.0, False)


def test_near_the_chair_forward_speed_is_cut_to_what_can_still_stop():
    pose = OdomPose(0.65, 0.0, 0.0)          # nose 0.03 from the edge
    vx, vy, cut = limit_toward(0.08, 0.0, pose, front_at(1.0), model(), 0.32, 0.15)
    assert cut
    assert vx == pytest.approx((0.03 - 0.01 - 0.005) / 0.3)
    assert model().stopping_distance(vx) == pytest.approx(0.03 - 0.01)


def test_at_the_limit_nothing_more_goes_toward_the_chair():
    pose = OdomPose(0.672, 0.0, 0.0)         # nose 0.008 from the edge
    vx, vy, cut = limit_toward(0.08, 0.03, pose, front_at(1.0), model(), 0.32, 0.15)
    assert cut and vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.03)          # sliding along the face passes


def test_backing_away_is_never_cut():
    pose = OdomPose(0.672, 0.0, 0.0)
    assert limit_toward(-0.05, 0.0, pose, front_at(1.0), model(), 0.32, 0.15) == (-0.05, 0.0, False)
