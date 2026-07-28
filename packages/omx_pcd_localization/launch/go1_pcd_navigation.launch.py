"""Start guarded 3D PCD localization and Nav2 without AMCL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


DEFAULT_PROJECT_ROOT = "/mnt/t500/go1_ros2_project"
DEFAULT_SESSION = "20260728_204825"


def validate_inputs(context):
    """Fail before ROS nodes start when either navigation map is missing."""
    paths = {
        "2D map YAML": LaunchConfiguration("map").perform(context),
        "3D PCD map": LaunchConfiguration("pcd_map").perform(context),
        "PCD localizer parameters": LaunchConfiguration("localizer_params_file").perform(
            context
        ),
        "Nav2 parameters": LaunchConfiguration("nav2_params_file").perform(context),
    }
    missing = [f"{label}: {path}" for label, path in paths.items() if not os.path.isfile(path)]
    if missing:
        raise RuntimeError("Required localization inputs are missing:\n  " + "\n  ".join(missing))
    return []


def generate_launch_description() -> LaunchDescription:
    localizer_share = get_package_share_directory("omx_pcd_localization")
    navigation_share = get_package_share_directory("omx_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    go1_share = get_package_share_directory("go1_driver")

    map_yaml = LaunchConfiguration("map")
    pcd_map = LaunchConfiguration("pcd_map")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    localizer_params = LaunchConfiguration("localizer_params_file")
    nav2_params = LaunchConfiguration("nav2_params_file")
    scan_params = LaunchConfiguration("scan_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_go1_driver = LaunchConfiguration("start_go1_driver")
    arm = LaunchConfiguration("arm")

    scan_projection = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
        parameters=[scan_params],
    )

    localizer = Node(
        package="omx_pcd_localization",
        executable="pcd_localizer",
        name="pcd_localizer",
        output="screen",
        parameters=[
            localizer_params,
            {
                "map_path": pcd_map,
                "cloud_topic": cloud_topic,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_pcd_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": autostart,
            "use_composition": "False",
        }.items(),
    )

    navigation_group = GroupAction(
        actions=[
            SetRemap(src="scan", dst=scan_topic),
            SetRemap(src="/scan", dst=scan_topic),
            SetRemap(src="odom", dst=odom_topic),
            SetRemap(src="/odom", dst=odom_topic),
            map_server,
            map_lifecycle,
            navigation,
            Node(
                package="omx_navigation",
                executable="rviz_goal_bridge",
                name="rviz_goal_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/scan", scan_topic)],
        condition=IfCondition(start_rviz),
    )

    go1_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_share, "launch", "go1_driver.launch.py")
        ),
        condition=IfCondition(start_go1_driver),
        launch_arguments={"arm": arm, "cmd_vel_topic": "/cmd_vel"}.items(),
    )

    default_session_root = os.path.join(
        DEFAULT_PROJECT_ROOT, "maps", "hanyang_9f", DEFAULT_SESSION
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    default_session_root, "slam_toolbox", "hanyang_9f.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value=os.path.join(default_session_root, "pcd", "merged.pcd"),
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("ros_domain_id", default_value="100"),
            DeclareLaunchArgument(
                "localizer_params_file",
                default_value=os.path.join(localizer_share, "config", "hanyang_9f.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    navigation_share, "config", "nav2_existing_map_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scan_params_file",
                default_value=os.path.join(
                    navigation_share, "config", "mid360_scan.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    navigation_share, "rviz", "go1_existing_map_low_load.rviz"
                ),
            ),
            DeclareLaunchArgument("start_go1_driver", default_value="true"),
            DeclareLaunchArgument("arm", default_value="false"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            OpaqueFunction(function=validate_inputs),
            scan_projection,
            localizer,
            navigation_group,
            rviz,
            go1_driver,
        ]
    )
