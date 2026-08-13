from pathlib import Path

import pytest

from omx_navigation.pose_config import PlanarPose, load_planar_pose
from omx_navigation.record_destination_pose import save_destination


def test_save_destination_uses_destination_kind_and_refuses_overwrite(tmp_path: Path):
    path = tmp_path / "destination.yaml"
    pose = PlanarPose("map", 3.0, 4.0, 0.2)
    save_destination(path, pose, overwrite=False)
    assert load_planar_pose(str(path), expected_kind="destination").pose == pose
    with pytest.raises(FileExistsError):
        save_destination(path, pose, overwrite=False)
