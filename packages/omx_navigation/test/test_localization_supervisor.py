import math

import pytest

from omx_navigation.localization_state import LocalizationState, LocalizationStatus
from omx_navigation.localization_supervisor import initial_pose_values, status_line
from omx_navigation.pose_config import PlanarPose


def test_initial_pose_values_encode_planar_pose_and_conservative_covariance():
    values = initial_pose_values(PlanarPose("map", 1.0, -2.0, math.pi / 2.0))
    assert values["frame_id"] == "map"
    assert values["x"] == 1.0
    assert values["y"] == -2.0
    assert values["qz"] == pytest.approx(math.sin(math.pi / 4.0))
    assert values["qw"] == pytest.approx(math.cos(math.pi / 4.0))
    assert values["covariance_x"] == 0.04
    assert values["covariance_y"] == 0.04
    assert values["covariance_yaw"] == 0.0305


def test_status_line_is_machine_and_human_readable():
    line = status_line(
        LocalizationStatus(LocalizationState.DEGRADED, False, "scan mismatch")
    )
    assert line == "state=DEGRADED ready=false reason=scan mismatch"
