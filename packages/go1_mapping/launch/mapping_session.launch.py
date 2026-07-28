"""Launch one guarded mapping session without motion or navigation."""
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go1_mapping.manifest import (
    build_running_manifest,
    create_session,
    write_manifest_atomic,
)


def _critical_exit_actions(returncode, process_name):
    """Return shutdown actions only for an unexpected non-zero process exit."""
    if returncode == 0:
        return []
    reason = f"critical process {process_name} exited with code {returncode}"
    return [
        LogInfo(msg=f"ERROR: {reason}"),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def _critical_exit_handler(process_name):
    def on_exit(event, context):
        del context
        return _critical_exit_actions(event.returncode, process_name)

    return on_exit


def _critical_exit_registration(action, process_name):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=action,
            on_exit=_critical_exit_handler(process_name),
        )
    )


def _launch_session(context):
    mapping_share = Path(get_package_share_directory("go1_mapping"))
    mapping_config_dir = str(mapping_share / "config")
    mapping_config = mapping_share / "config" / "mapping_session.yaml"
    session_config = yaml.safe_load(
        mapping_config.read_text(encoding="utf-8")
    )["mapping_session"]

    session_root = context.perform_substitution(
        LaunchConfiguration("session_root")
    )
    session_id = context.perform_substitution(LaunchConfiguration("session_id"))
    ros_domain_id = int(
        context.perform_substitution(LaunchConfiguration("ros_domain_id"))
    )

    resolved_session_root = Path(session_root).resolve()
    paths = create_session(resolved_session_root, session_id)
    manifest = build_running_manifest(paths.root.name, ros_domain_id)
    write_manifest_atomic(
        paths.validation / "session_manifest.yaml",
        manifest,
    )

    livox_share = Path(get_package_share_directory("livox_ros_driver2"))
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(livox_share / "launch_ROS2" / "msg_MID360_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_livox")),
    )

    bag_command = [
        "ros2", "bag", "record", "--output", str(paths.bag / "raw"),
        "--max-bag-size", "4294967296", "--compression-mode", "file",
        "--compression-format", "zstd", "/livox/lidar", "/livox/imu", "/Odometry", "/tf", "/tf_static",
    ]
    rosbag = ExecuteProcess(cmd=bag_command, output="screen")

    fast_lio_share = Path(get_package_share_directory("fast_lio"))
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fast_lio_share / "launch" / "mapping.launch.py")
        ),
        launch_arguments={
            "config_path": mapping_config_dir,
            "config_file": "fast_lio_mapping_safe.yaml",
            "rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_fast_lio")),
    )

    writer = Node(
        package="go1_mapping",
        executable="pcd_chunk_writer",
        name="pcd_chunk_writer",
        output="screen",
        parameters=[
            {
                "allowed_root": str(resolved_session_root),
                "output_dir": str(paths.pcd),
                "frames_per_chunk": int(session_config["frames_per_chunk"]),
                "max_buffer_bytes": int(session_config["max_buffer_bytes"]),
            }
        ],
    )
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            str(mapping_share / "config" / "pointcloud_to_scan_mapping.yaml")
        ],
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            ("scan", "/scan"),
        ],
    )
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            str(mapping_share / "config" / "slam_toolbox_hanyang_9f.yaml")
        ],
        remappings=[
            ("scan", "/scan"),
            ("map", "/map_slam"),
        ],
    )
    guard = Node(
        package="go1_mapping",
        executable="session_guard.py",
        name="mapping_session_guard",
        output="screen",
        parameters=[
            {
                "session_dir": str(paths.root),
                "abort_free_gib": float(session_config["abort_free_gib"]),
                "lidar_min_hz": float(session_config["lidar_min_hz"]),
                "imu_min_hz": float(session_config["imu_min_hz"]),
                "odom_min_hz": float(session_config["odom_min_hz"]),
                "max_gap_sec": float(session_config["max_input_gap_sec"]),
                "initialization_sec": float(
                    session_config["initialization_sec"]
                ),
            }
        ],
    )

    fast_lio_timer = TimerAction(period=3.0, actions=[fast_lio])
    mapping_timer = TimerAction(period=8.0, actions=[
        writer, pointcloud_to_laserscan, slam_toolbox, guard,
    ])

    return [
        _critical_exit_registration(rosbag, "rosbag"),
        _critical_exit_registration(writer, "pcd_chunk_writer"),
        _critical_exit_registration(guard, "session_guard"),
        livox,
        rosbag,
        fast_lio_timer,
        mapping_timer,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("session_root", default_value=
                                  "/mnt/t500/maps/hanyang_9f"),
            DeclareLaunchArgument("session_id", default_value=""),
            DeclareLaunchArgument("ros_domain_id", default_value="100"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID",
                LaunchConfiguration("ros_domain_id"),
            ),
            OpaqueFunction(function=_launch_session),
        ]
    )