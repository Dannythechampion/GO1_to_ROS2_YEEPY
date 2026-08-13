import math

import pytest

from omx_navigation.commission_start_pose import (
    CommissioningError,
    PoseSample,
    StartPoseEstimator,
)


def sample(index: int, *, x=1.0, y=2.0, yaw=0.1, linear=0.0, angular=0.0):
    return PoseSample(
        stamp=float(index),
        x=x,
        y=y,
        yaw=yaw,
        covariance_x=0.01,
        covariance_y=0.01,
        covariance_yaw=0.01,
        linear_speed=linear,
        angular_speed=angular,
    )


def test_estimator_requires_ten_samples_and_rejects_one_position_outlier():
    estimator = StartPoseEstimator(sample_count=10, max_position_spread=5.0)
    for index in range(9):
        assert estimator.add(sample(index, x=1.0 + index * 0.001)) is None
    result = estimator.add(sample(9, x=4.0))
    assert result.x == pytest.approx(1.0045)
    assert result.y == pytest.approx(2.0)


def test_estimator_uses_circular_yaw_mean_across_pi_boundary():
    estimator = StartPoseEstimator(sample_count=2)
    assert estimator.add(sample(0, yaw=math.pi - 0.02)) is None
    result = estimator.add(sample(1, yaw=-math.pi + 0.02))
    assert abs(abs(result.yaw) - math.pi) < 1e-6


@pytest.mark.parametrize(
    "bad, reason",
    [
        (sample(0, linear=0.02), "moving"),
        (sample(0, angular=0.02), "moving"),
        (PoseSample(0, 1, 2, 0, 0.05, 0.01, 0.01, 0, 0), "covariance"),
        (PoseSample(0, 1, 2, 0, 0.01, 0.01, 0.04, 0, 0), "covariance"),
    ],
)
def test_estimator_rejects_unsafe_sample(bad: PoseSample, reason: str):
    with pytest.raises(CommissioningError, match=reason):
        StartPoseEstimator(sample_count=2).add(bad)


def test_estimator_rejects_out_of_order_stamp():
    estimator = StartPoseEstimator(sample_count=2)
    estimator.add(sample(2))
    with pytest.raises(CommissioningError, match="timestamp"):
        estimator.add(sample(1))


def test_estimator_rejects_stale_sample_when_receive_time_is_known():
    estimator = StartPoseEstimator(sample_count=2, max_sample_age=0.30)
    with pytest.raises(CommissioningError, match="stale"):
        estimator.add(sample(1), now=1.31)


def test_estimator_rejects_internally_inconsistent_set():
    estimator = StartPoseEstimator(sample_count=3, max_position_spread=0.10)
    estimator.add(sample(0, x=0.0))
    estimator.add(sample(1, x=0.01))
    with pytest.raises(CommissioningError, match="spread"):
        estimator.add(sample(2, x=1.0))
