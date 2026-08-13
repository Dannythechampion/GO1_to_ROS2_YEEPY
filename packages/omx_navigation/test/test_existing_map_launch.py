from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "go1_existing_map.launch.py"
SCAN = ROOT / "config" / "mid360_scan.yaml"


def test_launch_defaults_are_safe():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n                "map",' in text
    assert '"20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml"' in text
    assert 'DeclareLaunchArgument("arm", default_value="false")' in text
    assert 'DeclareLaunchArgument("ros_domain_id", default_value="100")' in text
    assert 'default_value="/cloud_registered_body"' in text
    assert 'default_value="/Odometry"' in text
    assert 'default_value="camera_init"' in text
    assert 'default_value="body"' in text
    assert "OpaqueFunction(function=validate_runtime_configuration)" in text
    assert "Existing map YAML does not exist" in text


def test_scan_projection_is_low_load_and_body_framed():
    text = SCAN.read_text(encoding="utf-8")
    for expected in (
        "target_frame: body",
        "min_height: -0.20",
        "max_height: 0.60",
        "angle_increment: 0.0174533",
        "scan_time: 0.10",
        "range_max: 10.0",
        "queue_size: 1",
    ):
        assert expected in text
