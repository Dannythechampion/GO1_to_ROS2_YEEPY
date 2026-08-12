import ast
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


def test_parse_launch_boolean_accepts_only_explicit_values():
    launch = load_launch_module()
    assert launch.parse_launch_boolean("false", "use_composition") is False
    assert launch.parse_launch_boolean("true", "use_composition") is True
    assert launch.parse_launch_boolean("1", "use_composition") is True
    assert launch.parse_launch_boolean("0", "use_composition") is False
    with pytest.raises(RuntimeError, match="use_composition"):
        launch.parse_launch_boolean("maybe", "use_composition")
    composition_argument = next(
        call for call in _calls("DeclareLaunchArgument")
        if ast.literal_eval(call.args[0]) == "use_composition"
    )
    assert _keyword_value(composition_argument, "default_value") == "false"


def test_yaml_and_pgm_pure_validation_rejects_malformed_content():
    launch = load_launch_module()
    with pytest.raises(RuntimeError, match="mapping"):
        launch.parse_yaml_mapping("- not-a-mapping", "Nav2 parameters")
    with pytest.raises(RuntimeError, match="YAML"):
        launch.parse_yaml_mapping("[", "SLAM Toolbox parameters")
    with pytest.raises(RuntimeError, match="PGM"):
        launch.validate_pgm_bytes(b"P3\n1 1\n255\n0")
    with pytest.raises(RuntimeError, match="pixel"):
        launch.validate_pgm_bytes(b"P5\n2 2\n255\n\x00")
    with pytest.raises(RuntimeError, match="pixel"):
        launch.validate_pgm_bytes(b"P2\n1 1\n255\n256")


def test_relative_map_image_cannot_escape_map_directory():
    launch = load_launch_module()
    with pytest.raises(RuntimeError, match="escapes"):
        launch.resolve_map_image(Path("/maps/hanyang/map.yaml"), "../outside.pgm")


def _calls(name):
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    return [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == name
    ]


def _keyword_value(call, keyword):
    value = next(item.value for item in call.keywords if item.arg == keyword)
    return ast.literal_eval(value)


def _expression_value(value):
    try:
        return ast.literal_eval(value)
    except ValueError:
        return ast.unparse(value)


def _assignment_value(name):
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    assignment = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        )
    )
    return ast.literal_eval(assignment.value)


def test_launch_registers_explicit_nav2_actions_and_velocity_contract():
    containers = _calls("ComposableNodeContainer")
    assert len(containers) == 1
    container = containers[0]
    assert _keyword_value(container, "package") == "rclcpp_components"
    assert _keyword_value(container, "executable") == "component_container_isolated"
    assert _keyword_value(container, "name") == "nav2_container"

    node_specs = _assignment_value("nav2_node_specs")
    nodes = {(package, executable) for _name, package, executable, _plugin in node_specs}
    for node in (
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother"),
    ):
        assert node in nodes
    assert any(
        _keyword_value(call, "package") == "nav2_lifecycle_manager"
        and _keyword_value(call, "executable") == "lifecycle_manager"
        for call in _calls("Node")
    )
    assert not _calls("PythonExpression")
    gate = next(call for call in _calls("Node") if _keyword_value(call, "executable") == "cmd_vel_safety_gate")
    remaps = _assignment_value("nav2_remaps")
    assert ("cmd_vel", "/nav2_controller_cmd_vel") in remaps["controller_server"]
    assert ("cmd_vel", "/nav2_controller_cmd_vel") in remaps["velocity_smoother"]
    assert ("cmd_vel_smoothed", "/cmd_vel_nav") in remaps["velocity_smoother"]
    assert ("cmd_vel_nav", "/cmd_vel") not in remaps["velocity_smoother"]
    assert ("'input_topic'", "'/cmd_vel_nav'") in _dict_pairs(gate)
    assert ("'output_topic'", "'/cmd_vel'") in _dict_pairs(gate)


def _remap_pairs(call):
    remappings = next(item.value for item in call.keywords if item.arg == "remappings")
    return {tuple(ast.unparse(part) for part in item.elts) for item in remappings.elts}


def _dict_pairs(call):
    parameters = next(item.value for item in call.keywords if item.arg == "parameters")
    mapping = next(item for item in parameters.elts if isinstance(item, ast.Dict))
    return {(ast.unparse(key), ast.unparse(value)) for key, value in zip(mapping.keys, mapping.values)}


def test_supervisor_receives_configured_scan_and_odometry_topics():
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    supervisor = next(
        call for call in _calls("Node")
        if _keyword_value(call, "executable") == "localization_supervisor"
    )
    remappings = next(item.value for item in supervisor.keywords if item.arg == "remappings")
    pairs = {tuple(ast.literal_eval(item)) if isinstance(item, ast.Constant) else tuple(ast.unparse(part) for part in item.elts) for item in remappings.elts}
    assert ("'/scan'", "scan_topic") in pairs
    assert ("'/Odometry'", "odom_topic") in pairs
