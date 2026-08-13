from pathlib import Path


ROOT = Path(__file__).parents[1]
DOC = ROOT / "docs" / "GO1_NAV2_END_TO_END.md"
SAVE = ROOT / "migration" / "save_nav2_map.sh"
VERIFY = ROOT / "migration" / "verify_mapping_pipeline.sh"
STAGE = ROOT / "migration" / "stage_local_ros2_packages.sh"


def test_end_to_end_doc_contains_every_operating_gate():
    text = DOC.read_text(encoding="utf-8")
    for expected in (
        "go1_mapping.launch.py",
        "save_nav2_map.sh",
        "go1_existing_map.launch.py",
        "verify_mapping_pipeline.sh",
        "verify_existing_map_navigation.sh preflight",
        "verify_existing_map_navigation.sh localized",
        "arm:=false",
        "arm:=true",
        "2D Pose Estimate",
    ):
        assert expected in text


def test_map_saver_is_versioned_and_refuses_overwrite():
    text = SAVE.read_text(encoding="utf-8")
    assert 'map_root="${GO1_MAP_ROOT:-/mnt/t500/maps}"' in text
    assert 'map_root="$(realpath -m "$map_root")"' in text
    assert 'map_prefix="$(realpath -m "$map_prefix")"' in text
    assert "ros2 run nav2_map_server map_saver_cli" in text
    assert '[[ -e "${map_prefix}.yaml" || -e "${map_prefix}.pgm" ]]' in text
    assert "refusing to overwrite an existing map version" in text
    assert 'test -s "${map_prefix}.yaml"' in text
    assert 'test -s "${map_prefix}.pgm"' in text


def test_mapping_verifier_requires_sensor_slam_topics_and_tf():
    text = VERIFY.read_text(encoding="utf-8")
    for topic in (
        "/livox/lidar",
        "/livox/imu",
        "/cloud_registered_body",
        "/scan",
        "/Odometry",
        "/map",
    ):
        assert topic in text
    assert "require_tf camera_init body" in text
    assert "require_tf map camera_init" in text
    assert "PASS: MID-360, FAST-LIO, scan projection" in text


def test_package_stager_includes_navigation_tests_for_colcon():
    text = STAGE.read_text(encoding="utf-8")
    entries = text[text.index("omx_entries=(") : text.index(")", text.index("omx_entries=("))]
    assert "  test\n" in entries
