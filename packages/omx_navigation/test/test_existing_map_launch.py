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
    assert "OpaqueFunction(function=validate_map)" in text
    assert "Existing map YAML does not exist" in text


def test_scan_projection_matches_the_mapping_scan_and_is_planar_framed():
    """The three matching-critical values must equal go1_mapping's scan config.

    Localization scores the live scan against a map built from
    go1_mapping/config/pointcloud_to_scan_mapping.yaml. Cheaper values were used
    here once -- range_max 10.0, angle_increment 0.0174533, max_height 0.60 --
    and the live scan then could not see features the map contains: the pose
    score went flat, the supervisor held 0.010-0.027 ambiguity margin against a
    required 0.05, and it never left ALIGNING. Matching the mapping values
    reaches READY on the first attempt at 0.092. The extra cost measured well
    inside the Jetson's headroom, so quality wins over load here.
    """
    text = SCAN.read_text(encoding="utf-8")
    for expected in (
        "min_height: -0.20",
        "max_height: 1.30",
        "angle_increment: 0.0087266",
        "scan_time: 0.10",
        "range_max: 20.0",
    ):
        assert expected in text
    assert "target_frame: body_nav" in text
    assert "target_frame: body\n" not in text


def test_existing_map_launch_registers_planar_frame_before_scan_projection():
    text = LAUNCH.read_text(encoding="utf-8")
    for expected in (
        'executable="planar_base_frame"',
        'planar_base_frame = LaunchConfiguration("planar_base_frame")',
        '"odom_frame": odom_frame',
        '"source_base_frame": base_frame',
        '"planar_base_frame": planar_base_frame',
        'DeclareLaunchArgument("planar_base_frame", default_value="body_nav")',
        'parameters=[scan_params_file, {"target_frame": planar_base_frame}]',
    ):
        assert expected in text
    assert text.index("planar_base_frame,") < text.index("scan_projection,")
