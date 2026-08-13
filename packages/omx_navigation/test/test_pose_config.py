import math
from pathlib import Path

import pytest

from omx_navigation.pose_config import (
    PlanarPose,
    PoseConfigurationError,
    PoseFileState,
    load_planar_pose,
    write_planar_pose_atomic,
)


def test_empty_optional_path_is_unconfigured_not_origin():
    result = load_planar_pose("", expected_kind="start")
    assert result.state is PoseFileState.UNCONFIGURED
    assert result.pose is None


def test_configured_missing_file_fails_closed(tmp_path: Path):
    with pytest.raises(PoseConfigurationError, match="does not exist"):
        load_planar_pose(str(tmp_path / "missing.yaml"), expected_kind="start")


@pytest.mark.parametrize(
    "contents, message",
    [
        ("schema_version: 1\nkind: start\nframe_id: odom\nx: 1\ny: 2\nyaw: 0\n", "frame_id"),
        ("schema_version: 1\nkind: destination\nframe_id: map\nx: 1\ny: 2\nyaw: 0\n", "kind"),
        ("schema_version: 1\nkind: start\nframe_id: map\nx: .nan\ny: 2\nyaw: 0\n", "finite"),
        ("schema_version: 2\nkind: start\nframe_id: map\nx: 1\ny: 2\nyaw: 0\n", "schema"),
    ],
)
def test_invalid_pose_file_fails_closed(tmp_path: Path, contents: str, message: str):
    path = tmp_path / "pose.yaml"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(PoseConfigurationError, match=message):
        load_planar_pose(str(path), expected_kind="start")


def test_writer_round_trips_and_normalizes_yaw(tmp_path: Path):
    path = tmp_path / "runtime" / "start.yaml"
    pose = PlanarPose(frame_id="map", x=1.25, y=-2.5, yaw=3.0 * math.pi)
    write_planar_pose_atomic(path, pose, kind="start")

    loaded = load_planar_pose(str(path), expected_kind="start")
    assert loaded.state is PoseFileState.CONFIGURED
    assert loaded.pose == PlanarPose(frame_id="map", x=1.25, y=-2.5, yaw=-math.pi)


def test_writer_refuses_accidental_overwrite(tmp_path: Path):
    path = tmp_path / "start.yaml"
    pose = PlanarPose(frame_id="map", x=1.0, y=2.0, yaw=0.0)
    write_planar_pose_atomic(path, pose, kind="start")

    with pytest.raises(FileExistsError):
        write_planar_pose_atomic(path, pose, kind="start")

    write_planar_pose_atomic(
        path,
        PlanarPose(frame_id="map", x=3.0, y=4.0, yaw=0.5),
        kind="start",
        overwrite=True,
    )
    assert load_planar_pose(str(path), expected_kind="start").pose.x == 3.0
