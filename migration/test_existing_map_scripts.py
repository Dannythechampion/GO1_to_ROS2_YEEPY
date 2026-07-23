from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_stage_script_includes_go1_mapping():
    text = (ROOT / "migration" / "stage_local_ros2_packages.sh").read_text()
    assert 'mapping_source="$repo_root/packages/go1_mapping"' in text
    assert 'mapping_target="$src_dir/go1_mapping"' in text
    assert '"$mapping_source/package.xml"' in text
    assert 'cp -a "$mapping_source" "$mapping_target"' in text


def test_stage_script_uses_lf_line_endings():
    script = ROOT / "migration" / "stage_local_ros2_packages.sh"
    assert b"\r\n" not in script.read_bytes()


def test_map_script_requires_both_map_files():
    text = (ROOT / "migration" / "prepare_existing_map.sh").read_text()
    assert 'test -f "$source_yaml"' in text
    assert 'test -f "$source_pgm"' in text
    assert 'install -m 0644 "$source_yaml" "$target_dir/scans_new.yaml"' in text
    assert 'install -m 0644 "$source_pgm" "$target_dir/scans_new.pgm"' in text
    assert "image: scans_new.pgm" in text


def test_runtime_verifier_enforces_arm_false_and_core_topics():
    text = (
        ROOT / "migration" / "verify_existing_map_navigation.sh"
    ).read_text()
    assert "export ROS_DOMAIN_ID=100" in text
    assert "ros2 param get /go1_driver arm" in text
    for topic in (
        "/scan",
        "/Odometry",
        "/map",
        "/amcl_pose",
        "/cmd_vel",
    ):
        assert topic in text
    assert "require_tf map camera_init" in text
    assert "require_tf camera_init body" in text
    assert "timeout 10 ros2 topic list" in text
    assert "timeout 10 ros2 lifecycle get" in text
    assert "timeout 10 ros2 param get /go1_driver arm" in text
    assert "PASS: existing-map navigation is active with arm=false" in text

def test_runtime_verifier_enables_nounset_only_after_ros_setup():
    text = (
        ROOT / "migration" / "verify_existing_map_navigation.sh"
    ).read_text()
    setup_index = text.index("source /opt/ros/humble/setup.bash")
    strict_index = text.index("set -euo pipefail")
    assert setup_index < strict_index
