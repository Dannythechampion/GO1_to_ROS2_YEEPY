import math

import pytest

from omx_navigation.pose_tracking import compose_pose, invert_pose, pose_difference
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


def test_invert_pose_undoes_compose():
    pose = Pose2D(1.5, -2.0, 2.8)
    identity = compose_pose(pose, invert_pose(pose))
    assert identity.x == pytest.approx(0.0, abs=1e-12)
    assert identity.y == pytest.approx(0.0, abs=1e-12)
    assert identity.yaw == pytest.approx(0.0, abs=1e-12)
    back = compose_pose(invert_pose(pose), pose)
    assert back.x == pytest.approx(0.0, abs=1e-12)
    assert back.yaw == pytest.approx(0.0, abs=1e-12)


def test_pose_difference_wraps_heading():
    translation, yaw = pose_difference(Pose2D(0.0, 0.0, math.pi - 0.01), Pose2D(3.0, 4.0, -math.pi + 0.01))
    assert translation == pytest.approx(5.0)
    assert yaw == pytest.approx(0.02)


def test_invert_pose_rejects_nonfinite_values():
    with pytest.raises(ValueError, match="finite"):
        invert_pose(Pose2D(0.0, float("inf"), 0.0))

