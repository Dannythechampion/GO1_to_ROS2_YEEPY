from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory("go1_driver")
    default_config = os.path.join(package_share, "config", "go1_driver.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("arm", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_safe"),
            Node(
                package="go1_driver",
                executable="go1_driver",
                name="go1_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "arm": ParameterValue(
                            LaunchConfiguration("arm"), value_type=bool
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    },
                ],
            ),
        ]
    )
