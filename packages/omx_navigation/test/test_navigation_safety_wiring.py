from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "go1_existing_map.launch.py"


def test_launch_exposes_unconfigured_pose_paths_and_safe_nodes():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("start_pose_file", default_value="")' in text
    assert 'DeclareLaunchArgument("initial_pose_arm", default_value="false")' in text
    assert 'DeclareLaunchArgument("destination_pose_file", default_value="")' in text
    assert 'executable="localization_supervisor"' in text
    assert 'executable="motion_gate"' in text
    assert 'executable="fixed_mission_manager"' in text
    assert '"cmd_vel_topic": "/cmd_vel_safe"' in text
    assert "navigation = GroupAction(" in text
    assert "IncludeLaunchDescription(" in text
    assert "SetRemap(src=\"/cmd_vel_nav\", dst=\"/cmd_vel_controller\")" in text
    assert "SetRemap(src=\"/cmd_vel\", dst=\"/cmd_vel_nav\")" in text
    assert "rviz_goal_bridge" not in text


def test_launch_validates_armed_start_configuration_before_nodes_start():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "validate_runtime_configuration" in text
    assert "initial_pose_arm requires start_pose_file" in text
    assert "configured start pose does not exist" in text
    assert "configured destination pose does not exist" in text


def test_safety_parameter_files_match_runtime_contract():
    localization = yaml.safe_load(
        (ROOT / "config" / "localization_supervisor.yaml").read_text(encoding="utf-8")
    )["localization_supervisor"]["ros__parameters"]
    gate = yaml.safe_load(
        (ROOT / "config" / "motion_gate.yaml").read_text(encoding="utf-8")
    )["motion_gate"]["ros__parameters"]
    mission = yaml.safe_load(
        (ROOT / "config" / "fixed_mission_manager.yaml").read_text(encoding="utf-8")
    )["fixed_mission_manager"]["ros__parameters"]

    assert localization["heartbeat_rate"] == 10.0
    assert gate["localization_timeout"] == 0.30
    assert gate["command_timeout"] == 0.25
    assert gate["mission_stop_timeout"] == 0.30
    assert gate["mission_stop_topic"] == "/mission/stop_required"
    assert gate["cmd_vel_input"] == "/cmd_vel_nav"
    assert gate["cmd_vel_output"] == "/cmd_vel_safe"
    assert mission["minimum_clearance"] == 0.35
    assert mission["heartbeat_rate"] == 10.0
