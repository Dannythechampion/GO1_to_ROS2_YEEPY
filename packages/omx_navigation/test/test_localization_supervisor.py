import math

import pytest

from omx_navigation.localization_state import LocalizationState, LocalizationStatus
from omx_navigation.localization_supervisor import (
    initial_pose_values,
    odometry_is_stationary,
    permitted_command_is_fresh,
    status_line,
)
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


def test_startup_stationarity_contract_is_strict():
    assert odometry_is_stationary(0.01, 0.0, 0.01)
    assert not odometry_is_stationary(0.011, 0.0, 0.0)
    assert not odometry_is_stationary(0.0, 0.0, 0.011)


def test_restart_suppression_requires_a_fresh_permitted_command():
    assert permitted_command_is_fresh(True, received_at=1.0, now=1.2)
    assert not permitted_command_is_fresh(True, received_at=1.0, now=1.31)
    assert not permitted_command_is_fresh(False, received_at=1.0, now=1.1)
    assert not permitted_command_is_fresh(True, received_at=2.0, now=1.9)
