"""Contract checks for the safe pose-graph diagnostics workflow."""

import ast
import subprocess
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "migration" / "verify_posegraph_navigation.sh"
LAUNCH = ROOT / "packages" / "omx_navigation" / "launch" / "go1_posegraph_navigation.launch.py"
STAGE = ROOT / "migration" / "stage_local_ros2_packages.sh"


def test_verifier_has_strict_shell_syntax_and_required_topics():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    for topic in (
        "/scan", "/Odometry", "/map", "/slam_toolbox/pose",
        "/localization_supervisor/status", "/localization_supervisor/ready",
        "/cmd_vel_nav", "/cmd_vel",
    ):
        assert topic in text


def test_ready_verifier_rejects_amcl_and_checks_tf_lifecycle_and_safety():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "[preflight|ready]" in text
    assert "/amcl" in text
    assert "require_tf map camera_init" in text
    assert "require_tf camera_init body_nav" in text
    for node in (
        "/map_server", "/controller_server", "/smoother_server", "/planner_server",
        "/behavior_server", "/bt_navigator", "/waypoint_follower", "/velocity_smoother",
    ):
        assert node in text
    assert "ros2 param get /go1_driver arm" in text
    assert "Boolean value is: False" in text
    assert "FAIL:" in text


def test_launch_records_a_single_safe_session_and_passes_its_csv_to_supervisor():
    text = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(text)
    imported_actions = {
        alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        and node.module == "launch.actions" for alias in node.names
    }
    assert {"OpaqueFunction", "ExecuteProcess"} <= imported_actions
    assert "record_localization" in text
    assert "record_cloud" in text
    assert "datetime" in text
    assert "localization_status.csv" in text
    assert "rosbag" in text
    assert "/cloud_registered_body" in text
    assert "cmd=[" in text


def test_stage_copies_posegraph_runtime_files_and_maps():
    text = STAGE.read_text(encoding="utf-8")
    for entry in ("package.xml", "omx_navigation", "config", "launch", "maps"):
        assert entry in text
    assert "verify_posegraph_navigation.sh" in text
