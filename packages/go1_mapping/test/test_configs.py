from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_package_declares_mapping_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    names = {element.text for element in package}
    for dependency in {
        "ament_cmake",
        "ament_cmake_python",
        "rclcpp",
        "rclpy",
        "sensor_msgs",
        "std_srvs",
        "pcl_conversions",
        "pointcloud_to_laserscan",
        "slam_toolbox",
        "rosbag2",
    }:
        assert dependency in names


def test_cmake_installs_launch_config_rviz_and_python_package():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "ament_python_install_package(${PROJECT_NAME})" in cmake
    assert "install(DIRECTORY launch config rviz" in cmake
    assert "add_executable(pcd_chunk_writer" in cmake
    assert "add_executable(pcd_to_grid" in cmake
