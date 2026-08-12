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


def test_normalize_use_composition_uses_humble_spelling():
    launch = load_launch_module()
    assert launch.normalize_use_composition("false") == "False"
    assert launch.normalize_use_composition("true") == "True"
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


def test_launch_registers_composition_container_and_humble_velocity_contract():
    containers = _calls("ComposableNodeContainer")
    assert len(containers) == 1
    container = containers[0]
    assert _keyword_value(container, "package") == "rclcpp_components"
    assert _keyword_value(container, "executable") == "component_container_isolated"
    assert _keyword_value(container, "name") == "nav2_container"

    remaps = {
        (_expression_value(next(item.value for item in call.keywords if item.arg == "src")),
         _expression_value(next(item.value for item in call.keywords if item.arg == "dst")))
        for call in _calls("SetRemap")
    }
    assert ("/initialpose", "/slam_localization/initialpose") in remaps
    assert ("/map", "/slam_localization/map") in remaps
    assert ("cmd_vel_nav", "/nav2_controller_cmd_vel") in remaps
    assert ("cmd_vel", "/cmd_vel_nav") in remaps
    assert ("cmd_vel_nav", "/cmd_vel") not in remaps


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
