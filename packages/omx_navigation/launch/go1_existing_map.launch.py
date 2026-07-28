"""Bring up point-cloud projection, existing-map Nav2, RViz, and safe Go1 I/O."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_map(context):
    """Fail before starting ROS nodes when the existing map is unavailable."""
    map_path = LaunchConfiguration("map").perform(context)
    if not os.path.isfile(map_path):
        raise RuntimeError(f"Existing map YAML does not exist: {map_path}")
    return []


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("omx_navigation")
    go1_share = get_package_share_directory("go1_driver")

    map_yaml = LaunchConfiguration("map")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    nav2_params_file = LaunchConfiguration("params_file")
    scan_params_file = LaunchConfiguration("scan_params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_go1_driver = LaunchConfiguration("start_go1_driver")
    arm = LaunchConfiguration("arm")

    scan_projection = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
        parameters=[scan_params_file],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rviz_navigation.launch.py")
        ),
        launch_arguments={
            "slam": "false",
            "map": map_yaml,
            "params_file": nav2_params_file,
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "map_frame": "map",
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "rviz": rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    go1_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_share, "launch", "go1_driver.launch.py")
        ),
        condition=IfCondition(start_go1_driver),
        launch_arguments={"arm": arm, "cmd_vel_topic": "/cmd_vel"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=(
                    "/mnt/t500/go1_ros2_project/maps/hanyang_9f/"
                    "20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml"
                ),
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("ros_domain_id", default_value="100"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    package_share, "config", "nav2_existing_map_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scan_params_file",
                default_value=os.path.join(
                    package_share, "config", "mid360_scan.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    package_share,
                    "rviz",
                    "go1_existing_map_low_load.rviz",
                ),
            ),
            DeclareLaunchArgument("start_go1_driver", default_value="true"),
            DeclareLaunchArgument("arm", default_value="false"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            OpaqueFunction(function=validate_map),
            scan_projection,
            navigation,
            go1_driver,
        ]
    )
