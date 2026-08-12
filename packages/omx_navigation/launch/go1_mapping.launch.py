"""Bring up scan projection, SLAM Toolbox, Nav2, RViz, and optional Go1 I/O."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BAG_TOPICS = (
    "/livox/lidar",
    "/livox/imu",
    "/Odometry",
    "/cloud_registered_body",
    "/tf",
    "/tf_static",
)


def _default_bag_output() -> str:
    """Timestamped directory so a rerun never collides with an earlier bag."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(
        os.path.expanduser("~"), "go1_maps", stamp, "bag", "raw"
    )


def _rosbag_recorder(context, *_args, **_kwargs):
    """Create the parent directory, then hand ros2 bag an unused output path."""
    output = context.perform_substitution(LaunchConfiguration("bag_output"))
    os.makedirs(os.path.dirname(output), exist_ok=True)
    return [
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--output", output,
                "--max-bag-size", "4294967296",
                "--compression-mode", "file",
                "--compression-format", "zstd",
                *BAG_TOPICS,
            ],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("omx_navigation")
    go1_share = get_package_share_directory("go1_driver")

    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    planar_base_frame = LaunchConfiguration("planar_base_frame")
    nav2_params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
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
        parameters=[scan_params_file, {"target_frame": planar_base_frame}],
    )

    planar_base_frame = Node(
        package="omx_navigation",
        executable="planar_base_frame",
        name="planar_base_frame",
        output="screen",
        parameters=[
            {
                "odom_frame": odom_frame,
                "source_base_frame": base_frame,
                "planar_base_frame": planar_base_frame,
            }
        ],
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rviz_navigation.launch.py")
        ),
        launch_arguments={
            "slam": "true",
            "params_file": nav2_params_file,
            "slam_params_file": slam_params_file,
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

    rosbag = OpaqueFunction(
        function=_rosbag_recorder,
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("planar_base_frame", default_value="body_nav"),
            DeclareLaunchArgument("ros_domain_id", default_value="100"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    package_share, "config", "nav2_existing_map_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    package_share, "config", "slam_toolbox.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scan_params_file",
                default_value=os.path.join(
                    package_share, "config", "mid360_scan.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="true",
                description="Record raw LiDAR/IMU so a session can be replayed offline.",
            ),
            DeclareLaunchArgument(
                "bag_output",
                default_value=_default_bag_output(),
                description="Bag destination; must not already exist.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "rviz", "nav2.rviz"),
            ),
            DeclareLaunchArgument(
                "start_go1_driver",
                default_value="false",
                description="Keep false while mapping with the Unitree hand controller.",
            ),
            DeclareLaunchArgument(
                "arm",
                default_value="false",
                description="Never arm unless start_go1_driver is intentional and verified.",
            ),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            rosbag,
            # This is action registration order, not a TF readiness barrier.
            # Early clouds may be dropped transiently; conversion resumes once
            # FAST-LIO publishes the source transform and the planar TF arrives.
            planar_base_frame,
            scan_projection,
            mapping,
            go1_driver,
        ]
    )
