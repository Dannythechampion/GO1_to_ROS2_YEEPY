import importlib.util
from pathlib import Path

import pytest


LAUNCH = Path(__file__).parents[1] / "launch" / "go1_posegraph_navigation.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("posegraph_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def valid_paths():
    package = Path(__file__).parents[1]
    map_root = package.parents[1] / "maps" / "hanyang_9f" / "20260728_204825" / "slam_toolbox"
    return {
        "map": str(map_root / "hanyang_9f_annotated.yaml"),
        "posegraph": str(map_root / "hanyang_9f"),
        "nav2_params": str(package / "config" / "nav2_posegraph_params.yaml"),
        "slam_params": str(package / "config" / "slam_toolbox_localization_hanyang_9f.yaml"),
        "scan_params": str(package / "config" / "mid360_scan.yaml"),
        "rviz_config": str(package / "rviz" / "go1_existing_map_low_load.rviz"),
    }


def test_validate_inputs_rejects_arm_before_ros_processes():
    launch = load_launch_module()
    with pytest.raises(RuntimeError, match="arm:=true"):
        launch.validate_inputs(valid_paths(), arm=True)


def test_validate_inputs_requires_posegraph_data_file():
    launch = load_launch_module()
    paths = valid_paths()
    paths["posegraph"] = paths["posegraph"] + "-missing"
    with pytest.raises(RuntimeError, match="posegraph"):
        launch.validate_inputs(paths, arm=False)


def test_launch_contract_keeps_map_server_and_slam_maps_separate():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'SetRemap(src="/initialpose", dst="/slam_localization/initialpose")' in text
    assert 'SetRemap(src="/map", dst="/slam_localization/map")' in text
    assert 'SetRemap(src="cmd_vel", dst="/cmd_vel_nav")' in text
    assert 'cmd_vel_topic": "/cmd_vel"' in text
