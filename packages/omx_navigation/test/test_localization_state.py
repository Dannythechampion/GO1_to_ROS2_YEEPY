import math
from dataclasses import replace

import pytest

from omx_navigation.localization_state import (
    ErrorCode,
    LocalizationPolicy,
    LocalizationState,
    LocalizationStateMachine,
    QualityObservation,
)


def good(now: float) -> QualityObservation:
    return QualityObservation(
        now=now,
        inputs_fresh=True,
        pose_available=True,
        overlap=0.70,
        ambiguity_margin=0.20,
        position_jump=0.01,
        yaw_jump=0.01,
        odom_reset=False,
        tf_conflict=False,
    )


def ready_machine() -> LocalizationStateMachine:
    machine = LocalizationStateMachine(LocalizationPolicy(verify_duration=3.0))
    machine.receive_initial_pose(0.0)
    machine.observe(good(0.1))
    machine.observe(good(3.2))
    return machine


def test_one_input_reaches_ready_after_stable_verification():
    machine = LocalizationStateMachine(LocalizationPolicy(verify_duration=3.0))
    assert machine.receive_initial_pose(0.0).state is LocalizationState.ALIGNING
    assert machine.observe(good(0.1)).state is LocalizationState.VERIFYING
    assert machine.observe(good(3.2)).state is LocalizationState.READY


def test_alignment_timeout_retries_three_times_then_loses():
    machine = LocalizationStateMachine(
        LocalizationPolicy(alignment_timeout=2.0, max_attempts=3)
    )
    machine.receive_initial_pose(0.0)
    first = machine.observe(QualityObservation.missing(now=2.1))
    second = machine.observe(QualityObservation.missing(now=4.2))
    final = machine.observe(QualityObservation.missing(now=6.3))
    assert first.republish_initial_pose is True
    assert second.republish_initial_pose is True
    assert final.state is LocalizationState.LOST
    assert final.error is ErrorCode.ALIGNMENT_TIMEOUT


def test_ready_degrades_then_loses_after_two_seconds():
    machine = ready_machine()
    degraded = machine.observe(replace(good(4.0), overlap=0.10))
    lost = machine.observe(replace(good(6.1), overlap=0.10))
    assert degraded.state is LocalizationState.DEGRADED
    assert lost.state is LocalizationState.LOST
    assert lost.error is ErrorCode.LOW_OVERLAP


def test_odom_reset_and_tf_conflict_are_immediate_loss():
    for field, error in (
        ("odom_reset", ErrorCode.ODOM_RESET),
        ("tf_conflict", ErrorCode.TF_CONFLICT),
    ):
        machine = ready_machine()
        transition = machine.observe(replace(good(4.0), **{field: True}))
        assert transition.state is LocalizationState.LOST
        assert transition.error is error
        assert transition.publish_stop is True


@pytest.mark.parametrize(
    "kwargs",
    (
        {"alignment_timeout": 0.0},
        {"max_attempts": 0},
        {"verify_duration": -1.0},
        {"degraded_timeout": -1.0},
        {"min_overlap": 1.01},
        {"min_ambiguity_margin": -0.01},
        {"max_position_jump": 0.0},
        {"max_yaw_jump": math.pi + 0.01},
    ),
)
def test_policy_rejects_invalid_duration_count_and_ranges(kwargs):
    with pytest.raises(ValueError):
        LocalizationPolicy(**kwargs)


def test_bad_quality_during_verification_restarts_alignment_without_accepting_pose():
    machine = LocalizationStateMachine(LocalizationPolicy(verify_duration=3.0))
    machine.receive_initial_pose(0.0)
    assert machine.observe(good(0.1)).state is LocalizationState.VERIFYING
    transition = machine.observe(replace(good(1.0), ambiguity_margin=0.01))
    assert transition.state is LocalizationState.ALIGNING
    assert transition.error is ErrorCode.AMBIGUOUS
    assert transition.publish_stop is True


def test_retry_restarts_a_lost_machine_and_republishes_initial_pose():
    machine = LocalizationStateMachine(LocalizationPolicy(max_attempts=1))
    machine.receive_initial_pose(0.0)
    machine.observe(QualityObservation.missing(now=20.1))
    transition = machine.retry(now=21.0)
    assert transition.state is LocalizationState.ALIGNING
    assert transition.error is ErrorCode.NONE
    assert transition.republish_initial_pose is True
