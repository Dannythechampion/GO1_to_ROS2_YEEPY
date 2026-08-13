import math

import pytest

from omx_navigation.localization_state import (
    LocalizationObservation,
    LocalizationState,
    LocalizationSupervisorCore,
)
from omx_navigation.pose_config import PlanarPose, PoseConfigurationError


START = PlanarPose("map", 1.0, 2.0, 0.1)


def good_observation(now=1.0, *, x=1.02, covariance=0.01, scan_count=10):
    return LocalizationObservation(
        now=now,
        amcl_stamp=now - 0.01,
        x=x,
        y=2.01,
        yaw=0.11,
        covariance_x=covariance,
        covariance_y=covariance,
        covariance_yaw=0.01,
        pose_age=0.01,
        scan_age=0.01,
        odom_age=0.01,
        tf_age=0.01,
        tf_ok=True,
        valid_beams=120,
        consecutive_scans=scan_count,
        median_residual=0.10,
        p80_residual=0.20,
    )


def seeded_core():
    core = LocalizationSupervisorCore(START, initial_pose_arm=True)
    core.set_inputs_available(True)
    assert core.state is LocalizationState.SEEDING
    assert core.consume_seed_request(0.5) == START
    assert core.state is LocalizationState.VERIFYING
    return core


def test_missing_pose_is_uncommissioned_when_not_armed():
    core = LocalizationSupervisorCore(None, initial_pose_arm=False)
    core.set_inputs_available(True)
    assert core.state is LocalizationState.UNCOMMISSIONED
    assert core.consume_seed_request(0.0) is None
    assert not core.ready


def test_missing_pose_is_configuration_error_when_armed():
    with pytest.raises(PoseConfigurationError):
        LocalizationSupervisorCore(None, initial_pose_arm=True)


def test_configured_pose_remains_unseeded_when_initial_pose_is_disarmed():
    core = LocalizationSupervisorCore(START, initial_pose_arm=False)
    core.set_inputs_available(True)
    assert core.state is LocalizationState.WAITING_FOR_INPUTS
    assert core.consume_seed_request(1.0) is None
    assert "disarmed" in core.reason


def test_seed_is_emitted_once_and_requires_post_seed_amcl_pose():
    core = seeded_core()
    assert core.consume_seed_request(0.6) is None
    stale = good_observation(now=1.0)
    stale = LocalizationObservation(**{**stale.__dict__, "amcl_stamp": 0.4})
    result = core.observe(stale)
    assert result.state is LocalizationState.VERIFYING
    assert result.reason == "amcl pose predates current seed"


@pytest.mark.parametrize(
    "changes, reason",
    [
        ({"x": 1.4}, "start pose delta"),
        ({"covariance_x": 0.05}, "covariance"),
        ({"pose_age": 0.31}, "stale"),
        ({"tf_ok": False}, "TF"),
        ({"valid_beams": 99}, "beams"),
        ({"consecutive_scans": 9}, "scans"),
        ({"median_residual": 0.16}, "median"),
        ({"p80_residual": 0.31}, "p80"),
    ],
)
def test_each_readiness_condition_fails_closed(changes, reason):
    core = seeded_core()
    observation = good_observation()
    observation = LocalizationObservation(**{**observation.__dict__, **changes})
    status = core.observe(observation)
    assert status.state is LocalizationState.VERIFYING
    assert reason.lower() in status.reason.lower()


def test_all_checks_must_hold_continuously_for_two_seconds():
    core = seeded_core()
    assert not core.observe(good_observation(now=1.0)).ready
    assert not core.observe(good_observation(now=2.99)).ready
    assert core.observe(good_observation(now=3.01)).ready


def test_distance_from_start_is_not_checked_after_initial_readiness():
    core = seeded_core()
    core.observe(good_observation(now=1.0))
    assert core.observe(good_observation(now=3.1)).ready
    assert core.observe(good_observation(now=3.2, x=10.0)).ready


def test_runtime_failure_immediately_degrades_ready_state():
    core = seeded_core()
    core.observe(good_observation(now=1.0))
    core.observe(good_observation(now=3.1))
    status = core.observe(good_observation(now=3.2, covariance=0.05))
    assert status.state is LocalizationState.DEGRADED
    assert not status.ready


@pytest.mark.parametrize(
    "second",
    [
        {"stamp": 0.9, "frame_id": "camera_init", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"stamp": 1.2, "frame_id": "new_odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"stamp": 1.2, "frame_id": "camera_init", "x": 0.6, "y": 0.0, "yaw": 0.0},
        {"stamp": 1.2, "frame_id": "camera_init", "x": 0.0, "y": 0.0, "yaw": math.radians(21)},
    ],
)
def test_session_restart_signals_clear_readiness(second):
    core = seeded_core()
    core.observe(good_observation(now=1.0))
    core.observe(good_observation(now=3.1))
    core.observe_odometry(1.0, "camera_init", 0.0, 0.0, 0.0, commanded=False)
    assert core.observe_odometry(commanded=False, **second)
    assert core.state is LocalizationState.RELOCALIZING
    assert not core.ready


def test_commanded_motion_does_not_trigger_jump_restart():
    core = seeded_core()
    core.observe_odometry(1.0, "camera_init", 0.0, 0.0, 0.0, commanded=False)
    assert not core.observe_odometry(
        1.2, "camera_init", 0.6, 0.0, 0.0, commanded=True
    )


def test_explicit_reset_clears_ready_and_requests_reseed():
    core = seeded_core()
    core.observe(good_observation(now=1.0))
    core.observe(good_observation(now=3.1))
    assert core.request_reset()
    assert core.state is LocalizationState.RELOCALIZING
    assert not core.ready
    assert core.consume_seed_request(4.0) == START
