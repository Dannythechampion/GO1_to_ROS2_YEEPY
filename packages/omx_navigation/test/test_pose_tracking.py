import math

import pytest

from omx_navigation.pose_tracking import compose_pose
from omx_navigation.scan_map_quality import Pose2D


def test_compose_pose_rotates_child_translation():
    result = compose_pose(Pose2D(1.0, 2.0, math.pi / 2), Pose2D(2.0, 0.0, -math.pi / 2))

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(0.0)


def test_compose_pose_wraps_yaw():
    result = compose_pose(Pose2D(0.0, 0.0, math.pi), Pose2D(0.0, 0.0, math.pi / 2))

    assert result.yaw == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize("value", (float("nan"), float("inf"), -float("inf")))
def test_compose_pose_rejects_nonfinite_values(value):
    with pytest.raises(ValueError, match="finite"):
        compose_pose(Pose2D(value, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0))
