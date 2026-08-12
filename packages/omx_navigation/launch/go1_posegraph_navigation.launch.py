"""Safe existing-map navigation with SLAM Toolbox pose-graph localization.

The map server owns the display/costmap ``/map`` topic.  SLAM Toolbox keeps a
private map topic and is the sole publisher of ``map -> camera_init``.
"""

import os
from pathlib import Path

import yaml

try:  # Keeping input validation importable makes dry-run checks ROS-independent.
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
    from launch.conditions import IfCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
    from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
    from nav2_common.launch import RewrittenYaml
except ImportError:  # pragma: no cover - exercised on ROS 2 targets.
    get_package_share_directory = None


def normalize_use_composition(value: str) -> str:
    """Return the capitalized boolean spelling Humble launch code expects."""
    return "True" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "False"


def parse_yaml_mapping(text: str, label: str) -> dict:
    """Parse a YAML document and require the mapping shape used by ROS params."""
    try:
        parsed = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise RuntimeError(f"Required {label} has invalid YAML: {error}") from error
    if not isinstance(parsed, dict):
        raise RuntimeError(f"Required {label} YAML must be a mapping")
    return parsed


def _regular_nonempty(path: Path, label: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Required {label} does not exist or is empty: {path}")


def _load_yaml_mapping(path: Path, label: str) -> dict:
    _regular_nonempty(path, label)
    try:
        return parse_yaml_mapping(path.read_text(encoding="utf-8"), label)
    except OSError as error:
        raise RuntimeError(f"Required {label} cannot be read: {path}") from error


def resolve_map_image(map_yaml: Path, image: str) -> Path:
    """Resolve a portable relative map image without allowing directory escape.

    Absolute image paths are deliberately rejected: package-share artifacts must
    remain relocatable between the development host and the Jetson deployment.
    """
    if not isinstance(image, str) or not image.strip():
        raise RuntimeError(f"Map YAML has no nonempty image reference: {map_yaml}")
    if Path(image).is_absolute():
        raise RuntimeError(f"Absolute map image path is not portable: {image}")
    parent = map_yaml.resolve().parent
    candidate = (parent / image).resolve()
    if not candidate.is_relative_to(parent):
        raise RuntimeError(f"Map image escapes its map directory: {image}")
    return candidate


def validate_pgm_bytes(data: bytes) -> None:
    """Accept only nonempty Netpbm gray-map P2/P5 files."""
    if len(data) < 3 or data[:2] not in {b"P2", b"P5"} or data[2:3] not in b" \t\r\n":
        raise RuntimeError("Map image must be a nonempty PGM file with P2 or P5 magic")


def validate_inputs(paths, arm: bool) -> None:
    """Validate every static input before ROS actions can start processes."""
    if arm:
        raise RuntimeError("arm:=true is rejected by pose-graph navigation bringup")
    required = {
        "map": "map YAML",
        "nav2_params": "Nav2 parameters",
        "slam_params": "SLAM Toolbox parameters",
        "scan_params": "scan projection parameters",
        "rviz_config": "RViz config",
    }
    resolved = {key: Path(paths.get(key, "")) for key in required}
    map_config = _load_yaml_mapping(resolved["map"], required["map"])
    image_path = resolve_map_image(resolved["map"], map_config.get("image"))
    _regular_nonempty(image_path, "map image")
    try:
        validate_pgm_bytes(image_path.read_bytes())
    except OSError as error:
        raise RuntimeError(f"Map image cannot be read: {image_path}") from error

    expected_roots = {
        "nav2_params": ("map_server", "controller_server", "velocity_smoother"),
        "slam_params": ("slam_toolbox",),
        "scan_params": ("pointcloud_to_laserscan",),
    }
    for key, roots in expected_roots.items():
        config = _load_yaml_mapping(resolved[key], required[key])
        missing = [root for root in roots if root not in config]
        if missing:
            raise RuntimeError(f"Required {required[key]} YAML is missing roots: {', '.join(missing)}")
    _regular_nonempty(resolved["rviz_config"], required["rviz_config"])

    posegraph = paths.get("posegraph", "")
    for suffix in (".posegraph", ".data"):
        candidate = Path(posegraph + suffix)
        _regular_nonempty(candidate, "posegraph artifact")


def _validate_launch_inputs(context, *_args, **_kwargs):
    paths = {
        "map": LaunchConfiguration("map").perform(context),
        "posegraph": LaunchConfiguration("posegraph").perform(context),
        "nav2_params": LaunchConfiguration("nav2_params_file").perform(context),
        "slam_params": LaunchConfiguration("slam_params_file").perform(context),
        "scan_params": LaunchConfiguration("scan_params_file").perform(context),
        "rviz_config": LaunchConfiguration("rviz_config").perform(context),
    }
    arm = LaunchConfiguration("arm").perform(context).strip().lower() in {"1", "true", "yes"}
    validate_inputs(paths, arm)
    return []


def generate_launch_description() -> "LaunchDescription":
    if get_package_share_directory is None:
        raise RuntimeError("ROS 2 launch dependencies are unavailable")
    package_share = get_package_share_directory("omx_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    slam_share = get_package_share_directory("slam_toolbox")
    go1_share = get_package_share_directory("go1_driver")

    map_yaml = LaunchConfiguration("map")
    posegraph = LaunchConfiguration("posegraph")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    source_base_frame = LaunchConfiguration("source_base_frame")
    base_frame = LaunchConfiguration("base_frame")
    nav2_params = LaunchConfiguration("nav2_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    scan_params = LaunchConfiguration("scan_params_file")
    normalized_composition = PythonExpression(
        [
            "'True' if '", LaunchConfiguration("use_composition"),
            "'.lower() in ['1', 'true', 'yes', 'on'] else 'False'",
        ]
    )

    configured_nav2 = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            "map_server.ros__parameters.yaml_filename": map_yaml,
            "bt_navigator.ros__parameters.odom_topic": odom_topic,
            "local_costmap.local_costmap.ros__parameters.global_frame": odom_frame,
            "behavior_server.ros__parameters.global_frame": odom_frame,
        },
        convert_types=True,
    )
    configured_slam = RewrittenYaml(
        source_file=slam_params,
        param_rewrites={
            "slam_toolbox.ros__parameters.map_file_name": posegraph,
            "slam_toolbox.ros__parameters.odom_frame": odom_frame,
            "slam_toolbox.ros__parameters.base_frame": base_frame,
            "slam_toolbox.ros__parameters.scan_topic": scan_topic,
        },
        convert_types=True,
    )

    planar_frame = Node(
        package="omx_navigation", executable="planar_base_frame", name="planar_base_frame", output="screen",
        parameters=[{"odom_frame": odom_frame, "source_base_frame": source_base_frame, "planar_base_frame": base_frame}],
    )
    scan_projection = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", LaunchConfiguration("cloud_topic")), ("scan", scan_topic)],
        parameters=[scan_params, {"target_frame": base_frame}],
    )
    slam_localization = GroupAction(actions=[
        SetRemap(src="/initialpose", dst="/slam_localization/initialpose"),
        SetRemap(src="/map", dst="/slam_localization/map"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "localization_launch.py")),
            launch_arguments={"slam_params_file": configured_slam, "use_sim_time": "false"}.items(),
        ),
    ])
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen", parameters=[configured_nav2]
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="map_server_lifecycle_manager", output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )
    nav2_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="nav2_container",
        namespace="",
        output="screen",
        condition=IfCondition(normalized_composition),
    )
    navigation = GroupAction(actions=[
        SetRemap(src="scan", dst=scan_topic),
        SetRemap(src="/scan", dst=scan_topic),
        SetRemap(src="odom", dst=odom_topic),
        SetRemap(src="/odom", dst=odom_topic),
        # Humble navigation_launch uses cmd_vel_nav between controller and
        # smoother, then cmd_vel from smoother to the outside world.
        SetRemap(src="cmd_vel_nav", dst="/nav2_controller_cmd_vel"),
        SetRemap(src="cmd_vel", dst="/cmd_vel_nav"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": configured_nav2,
                "autostart": "true",
                "use_composition": normalized_composition,
                "container_name": "nav2_container",
            }.items(),
        ),
    ])
    supervisor = Node(
        package="omx_navigation", executable="localization_supervisor", name="localization_supervisor", output="screen",
        parameters=[{"camera_init_frame": odom_frame, "source_base_frame": source_base_frame, "base_frame": base_frame,
                     "diagnostics_csv": PathJoinSubstitution([LaunchConfiguration("diagnostics_root"), "localization.csv"])}],
        remappings=[("/scan", scan_topic), ("/Odometry", odom_topic)],
    )
    gate = Node(
        package="omx_navigation", executable="cmd_vel_safety_gate", name="cmd_vel_safety_gate", output="screen",
        parameters=[{"input_topic": "/cmd_vel_nav", "output_topic": "/cmd_vel"}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", LaunchConfiguration("rviz_config")],
        remappings=[("/scan", scan_topic)], condition=IfCondition(LaunchConfiguration("rviz")),
    )
    go1_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go1_share, "launch", "go1_driver.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_go1_driver")),
        launch_arguments={"arm": LaunchConfiguration("arm"), "cmd_vel_topic": "/cmd_vel"}.items(),
    )

    installed_maps = os.path.join(package_share, "maps", "hanyang_9f", "20260728_204825", "slam_toolbox")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(installed_maps, "hanyang_9f_annotated.yaml")),
        DeclareLaunchArgument("posegraph", default_value=os.path.join(installed_maps, "hanyang_9f")),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("source_base_frame", default_value="body"),
        DeclareLaunchArgument("base_frame", default_value="body_nav"),
        DeclareLaunchArgument("nav2_params_file", default_value=os.path.join(package_share, "config", "nav2_posegraph_params.yaml")),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(package_share, "config", "slam_toolbox_localization_hanyang_9f.yaml")),
        DeclareLaunchArgument("scan_params_file", default_value=os.path.join(package_share, "config", "mid360_scan.yaml")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(package_share, "rviz", "go1_existing_map_low_load.rviz")),
        DeclareLaunchArgument("diagnostics_root", default_value=os.path.expanduser("~/go1_localization/diagnostics")),
        DeclareLaunchArgument("record_localization", default_value="false"),
        DeclareLaunchArgument("record_cloud", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_go1_driver", default_value="false"),
        DeclareLaunchArgument("arm", default_value="false"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument("ros_domain_id", default_value="100"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        OpaqueFunction(function=_validate_launch_inputs),
        # Register the component container before the Nav2 include. This is
        # launch ordering rather than a readiness barrier; Humble waits for
        # the target service when loading composable nodes.
        planar_frame, scan_projection, map_server, map_lifecycle, slam_localization, supervisor, nav2_container, navigation, gate, rviz, go1_driver,
    ])
