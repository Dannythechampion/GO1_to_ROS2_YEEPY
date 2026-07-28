import os
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "go1_mapping.launch.py"


def test_mapping_launch_uses_fast_lio_topics_and_frames():
    text = LAUNCH.read_text(encoding="utf-8")
    for expected in (
        'default_value="/cloud_registered_body"',
        'default_value="/scan"',
        'default_value="/Odometry"',
        'default_value="camera_init"',
        'default_value="body"',
        '"slam": "true"',
        '"map_frame": "map"',
        '"pointcloud_to_laserscan_node"',
    ):
        assert expected in text


def test_mapping_launch_defaults_to_manual_controller_and_disarmed_driver():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("ros_domain_id", default_value="100")' in text
    assert '"start_go1_driver",\n                default_value="false"' in text
    assert '"arm",\n                default_value="false"' in text
    assert 'condition=IfCondition(start_go1_driver)' in text


def test_mapping_launch_reuses_safe_navigation_configuration():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"config", "nav2_existing_map_params.yaml"' in text
    assert '"config", "slam_toolbox.yaml"' in text
    assert '"config", "mid360_scan.yaml"' in text


def test_mapping_launch_records_a_replayable_bag_by_default():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"record_bag",\n                default_value="true"' in text
    assert '"ros2", "bag", "record",' in text
    # Raw sensor topics are what make an offline FAST-LIO rerun possible.
    for topic in ('"/livox/lidar"', '"/livox/imu"', '"/tf_static"'):
        assert topic in text
    assert 'condition=IfCondition(LaunchConfiguration("record_bag"))' in text


def test_bag_output_default_is_unique_per_run():
    import importlib.util

    spec = importlib.util.spec_from_file_location("_go1_mapping_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError:  # launch/ament not installed in this environment
        import pytest

        pytest.skip("ROS 2 launch packages unavailable")
    first = module._default_bag_output()
    assert first.endswith(os.path.join("bag", "raw"))
    assert os.path.isabs(first)
