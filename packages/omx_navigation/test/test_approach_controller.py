import math

import pytest

from omx_navigation.approach_controller import (
    ApproachController,
    ApproachPolicy,
    ApproachState,
    Detection,
)


def controller(**overrides):
    return ApproachController(ApproachPolicy(**overrides)) if overrides else ApproachController()


def started(**overrides):
    item = controller(**overrides)
    item.start(0.0, 0.30)
    return item


def test_policy_refuses_a_tolerance_the_driver_would_stall_inside():
    """The translation copy of the yaw dead zone must be impossible to build.

    The command shrinks with the error, so under linear_deadband/kp the driver
    zeroes it. If that happens before approach_tolerance is reached the robot
    stops short and never arrives -- exactly what yaw_deadband did to the final
    turn on 2026-09-18 and 2026-10-25.
    """
    with pytest.raises(ValueError, match="linear_deadband"):
        ApproachPolicy(approach_tolerance=0.01, linear_deadband=0.025, kp=0.6)


def test_default_policy_leaves_room_above_the_stall_point():
    policy = ApproachPolicy()
    assert policy.approach_tolerance > policy.linear_deadband / policy.kp


def test_idle_controller_commands_nothing():
    command = controller().update(1.0, Detection(1.0, 1.0, 0.0))
    assert (command.vx, command.vy) == (0.0, 0.0)
    assert command.state is ApproachState.IDLE


def test_approach_drives_toward_the_desired_offset():
    item = started()
    command = item.update(0.1, Detection(0.1, 0.60, 0.0))
    # Target is 0.30 m further out than wanted, so move forward.
    assert command.vx > 0.0
    assert command.vy == pytest.approx(0.0)
    assert command.state is ApproachState.APPROACHING
    assert command.error == pytest.approx(0.30)


def test_approach_backs_off_when_the_target_is_too_close():
    item = started()
    command = item.update(0.1, Detection(0.1, 0.10, 0.0))
    assert command.vx < 0.0


def test_approach_strafes_without_rotating():
    """Lateral error is corrected with vy; the stage never commands yaw."""
    item = started()
    command = item.update(0.1, Detection(0.1, 0.30, 0.20))
    assert command.vy > 0.0
    assert command.vx == pytest.approx(0.0)
    assert not hasattr(command, "yaw")


def test_speed_is_capped_at_the_approach_limit():
    item = started()
    command = item.update(0.1, Detection(0.1, 3.0, 0.0))
    assert math.hypot(command.vx, command.vy) <= ApproachPolicy().v_approach + 1e-9


def test_command_never_falls_under_the_driver_deadband_while_outside_tolerance():
    policy = ApproachPolicy()
    item = ApproachController(policy)
    item.start(0.0, 0.30)
    # Just outside tolerance, where the proportional term is smallest.
    error = policy.approach_tolerance * 1.001
    command = item.update(0.1, Detection(0.1, 0.30 + error, 0.0))
    assert command.state is ApproachState.APPROACHING
    assert math.hypot(command.vx, command.vy) >= policy.linear_deadband


def test_arrival_needs_consecutive_in_tolerance_samples():
    policy = ApproachPolicy()
    item = ApproachController(policy)
    item.start(0.0, 0.30)
    for index in range(policy.settle_checks - 1):
        command = item.update(0.1 * (index + 1), Detection(0.1 * (index + 1), 0.31, 0.0))
        assert command.state is ApproachState.APPROACHING
        assert (command.vx, command.vy) == (0.0, 0.0)
    command = item.update(1.0, Detection(1.0, 0.31, 0.0))
    assert command.state is ApproachState.ARRIVED
    assert (command.vx, command.vy) == (0.0, 0.0)


def test_a_single_good_sample_does_not_arrive():
    """One in-tolerance sample is what let the goal checker flip on 10-25."""
    item = started()
    assert item.update(0.1, Detection(0.1, 0.31, 0.0)).state is ApproachState.APPROACHING
    # Drifting back out resets the evidence.
    assert item.update(0.2, Detection(0.2, 0.60, 0.0)).state is ApproachState.APPROACHING
    assert item.update(0.3, Detection(0.3, 0.31, 0.0)).state is ApproachState.APPROACHING


@pytest.mark.parametrize(
    "kwargs, reason",
    [
        ({"manual_override": True}, "manual override"),
        ({"execution_fault": True}, "execution fault"),
        ({"localization_ready": False}, "localization not ready"),
    ],
)
def test_safety_inputs_abort_with_zero_velocity(kwargs, reason):
    item = started()
    command = item.update(0.1, Detection(0.1, 0.60, 0.0), **kwargs)
    assert command.state is ApproachState.ABORTED
    assert command.reason == reason
    assert (command.vx, command.vy) == (0.0, 0.0)


def test_a_dropped_scan_holds_still_instead_of_aborting():
    """One scan without a clean pair is a pause; the robot does not coast."""
    item = started()
    command = item.update(0.1, Detection(0.1, 0.60, 0.0))
    assert command.vx > 0.0
    command = item.update(0.2, None)
    assert command.state is ApproachState.APPROACHING
    assert command.reason == "waiting for detection"
    assert (command.vx, command.vy) == (0.0, 0.0)
    assert item.update(0.3, Detection(0.3, 0.60, 0.0)).vx > 0.0


def test_missing_and_stale_detections_abort():
    item = started()
    timeout = ApproachPolicy().detection_timeout
    assert item.update(0.1, None).reason == "waiting for detection"
    assert item.update(timeout + 0.01, None).reason == "no detection"
    item = started()
    item.update(0.1, Detection(0.1, 0.60, 0.0))
    assert item.update(0.1 + timeout + 0.01, None).reason == "no detection"
    item = started()
    stale = ApproachPolicy().detection_timeout + 0.01
    assert item.update(stale, Detection(0.0, 0.60, 0.0)).reason == "detection stale"


def test_detection_from_the_future_is_rejected():
    item = started()
    assert item.update(0.1, Detection(5.0, 0.60, 0.0)).reason == "detection stale"


def test_approach_times_out():
    item = started()
    late = ApproachPolicy().approach_timeout + 0.01
    assert item.update(late, Detection(late, 0.60, 0.0)).reason == "approach timeout"


def test_aborted_controller_stays_aborted_and_silent():
    item = started()
    item.update(0.1, Detection(0.1, 0.60, 0.0), manual_override=True)
    command = item.update(0.2, Detection(0.2, 0.60, 0.0))
    assert command.state is ApproachState.ABORTED
    assert (command.vx, command.vy) == (0.0, 0.0)


def test_reset_allows_a_new_attempt():
    item = started()
    item.update(0.1, Detection(0.1, 0.60, 0.0), execution_fault=True)
    item.reset()
    assert item.state is ApproachState.IDLE
    item.start(1.0, 0.30)
    assert item.update(1.1, Detection(1.1, 0.60, 0.0)).state is ApproachState.APPROACHING


def test_non_finite_inputs_are_rejected():
    item = started()
    with pytest.raises(ValueError):
        item.update(float("nan"), Detection(0.1, 0.60, 0.0))
    with pytest.raises(ValueError):
        item.update(0.1, Detection(0.1, float("inf"), 0.0))
