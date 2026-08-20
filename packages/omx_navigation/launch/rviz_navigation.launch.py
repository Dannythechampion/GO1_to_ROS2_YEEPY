"""Start SLAM/localization, Nav2, RViz, and the RViz goal bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("omx_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    start_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    frame_rewrites = {
        "use_sim_time": use_sim_time,
        "global_frame_id": map_frame,
        "map_frame": map_frame,
        "map_frame_id": map_frame,
        "odom_frame": odom_frame,
        "odom_frame_id": odom_frame,
        "robot_base_frame": base_frame,
        "base_frame": base_frame,
        "base_frame_id": base_frame,
        # global_frame means map for global nodes and odom for local nodes.
        # Full YAML paths avoid incorrectly assigning one frame to every node.
        "bt_navigator.ros__parameters.global_frame": map_frame,
        "global_costmap.global_costmap.ros__parameters.global_frame": map_frame,
        "local_costmap.local_costmap.ros__parameters.global_frame": odom_frame,
        "behavior_server.ros__parameters.global_frame": odom_frame,
    }
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=frame_rewrites,
        convert_types=True,
    )
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        param_rewrites=frame_rewrites,
        convert_types=True,
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        condition=IfCondition(slam),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": configured_slam_params,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "localization_launch.py")
        ),
        condition=UnlessCondition(slam),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": configured_nav2_params,
            "autostart": autostart,
            "use_composition": use_composition,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_nav2_params,
            "autostart": autostart,
            "use_composition": use_composition,
        }.items(),
    )

    navigation_group = GroupAction(
        actions=[
            SetRemap(src="scan", dst=scan_topic),
            SetRemap(src="/scan", dst=scan_topic),
            SetRemap(src="odom", dst=odom_topic),
            SetRemap(src="/odom", dst=odom_topic),
            slam_launch,
            localization_launch,
            navigation_launch,
            Node(
                package="omx_navigation",
                executable="rviz_goal_bridge",
                name="rviz_goal_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/scan", scan_topic)],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description="True: build a map online. False: localize in map:= YAML.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map YAML used when slam:=false.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    package_share, "config", "slam_toolbox.yaml"
                ),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "rviz", "nav2.rviz"),
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            navigation_group,
            rviz_node,
        ]
    )
