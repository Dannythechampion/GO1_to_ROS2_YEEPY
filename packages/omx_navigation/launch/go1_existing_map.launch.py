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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import SetRemap


def validate_runtime_configuration(context):
    """Fail before starting ROS nodes when runtime files are unsafe."""
    map_path = LaunchConfiguration("map").perform(context)
    if not os.path.isfile(map_path):
        raise RuntimeError(f"Existing map YAML does not exist: {map_path}")
    start_pose_file = LaunchConfiguration("start_pose_file").perform(context)
    destination_pose_file = LaunchConfiguration("destination_pose_file").perform(context)
    initial_pose_arm = LaunchConfiguration("initial_pose_arm").perform(context)
    armed = initial_pose_arm.strip().lower() in {"1", "true", "yes", "on"}
    if armed and not start_pose_file:
        raise RuntimeError("initial_pose_arm requires start_pose_file")
    if start_pose_file and not os.path.isfile(start_pose_file):
        raise RuntimeError(f"configured start pose does not exist: {start_pose_file}")
    if destination_pose_file and not os.path.isfile(destination_pose_file):
        raise RuntimeError(
            f"configured destination pose does not exist: {destination_pose_file}"
        )
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
    start_pose_file = LaunchConfiguration("start_pose_file")
    initial_pose_arm = LaunchConfiguration("initial_pose_arm")
    destination_pose_file = LaunchConfiguration("destination_pose_file")
    estop_topic = LaunchConfiguration("estop_topic")

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

    localization_supervisor = Node(
        package="omx_navigation",
        executable="localization_supervisor",
        name="localization_supervisor",
        output="screen",
        parameters=[
            os.path.join(package_share, "config", "localization_supervisor.yaml"),
            {
                "start_pose_file": start_pose_file,
                "initial_pose_arm": ParameterValue(initial_pose_arm, value_type=bool),
                "scan_topic": scan_topic,
                "odom_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
            },
        ],
    )

    motion_gate = Node(
        package="omx_navigation",
        executable="motion_gate",
        name="motion_gate",
        output="screen",
        parameters=[
            os.path.join(package_share, "config", "motion_gate.yaml"),
            {"estop_topic": estop_topic},
        ],
    )

    mission_manager = Node(
        package="omx_navigation",
        executable="fixed_mission_manager",
        name="fixed_mission_manager",
        output="screen",
        parameters=[
            os.path.join(package_share, "config", "fixed_mission_manager.yaml"),
            {"destination_pose_file": destination_pose_file},
        ],
    )

    go1_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_share, "launch", "go1_driver.launch.py")
        ),
        condition=IfCondition(start_go1_driver),
        launch_arguments={"arm": arm, "cmd_vel_topic": "/cmd_vel_safe"}.items(),
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
            DeclareLaunchArgument("start_pose_file", default_value=""),
            DeclareLaunchArgument("initial_pose_arm", default_value="false"),
            DeclareLaunchArgument("destination_pose_file", default_value=""),
            DeclareLaunchArgument("estop_topic", default_value="/emergency_stop"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            OpaqueFunction(function=validate_runtime_configuration),
            scan_projection,
            # Humble Nav2 uses cmd_vel_nav between controller and velocity smoother,
            # then publishes the smoothed result on cmd_vel. Keep those paths
            # distinct so the smoother cannot subscribe to its own output.
            SetRemap(src="/cmd_vel_nav", dst="/cmd_vel_controller"),
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
            navigation,
            localization_supervisor,
            motion_gate,
            mission_manager,
            go1_driver,
        ]
    )
