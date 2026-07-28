from pathlib import Path


ROOT = Path(__file__).parents[1]
SESSION_ID = "20260728_204825"
BUNDLED_MAP = ROOT / "maps" / "hanyang_9f" / SESSION_ID / "slam_toolbox"


def test_map_script_requires_both_map_files():
    text = (ROOT / "migration" / "prepare_existing_map.sh").read_text()
    assert 'test -f "$source_yaml"' in text
    assert 'test -f "$source_pgm"' in text
    assert 'session_id="${GO1_MAP_SESSION_ID:-20260728_204825}"' in text
    assert 'install -m 0644 "$source_yaml" "$target_yaml"' in text
    assert 'install -m 0644 "$source_pgm" "$target_pgm"' in text
    assert "image: hanyang_9f_annotated.pgm" in text


def test_annotated_hanyang_map_is_bundled_for_navigation():
    yaml_path = BUNDLED_MAP / "hanyang_9f_annotated.yaml"
    pgm_path = BUNDLED_MAP / "hanyang_9f_annotated.pgm"
    assert yaml_path.is_file()
    assert pgm_path.is_file()
    text = yaml_path.read_text(encoding="utf-8")
    assert "image: hanyang_9f_annotated.pgm" in text
    assert "resolution: 0.05" in text
    assert "origin: [-8.57, -19.2, 0]" in text


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
