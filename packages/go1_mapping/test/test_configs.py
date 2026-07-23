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


def test_cmake_closes_testing_block_and_declares_package():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "ament_add_pytest_test(test_configs test/test_configs.py)" in cmake
    testing_index = cmake.index("if(BUILD_TESTING)")
    pytest_index = cmake.index("ament_add_pytest_test", testing_index)
    assert cmake.index("endif()", pytest_index) > pytest_index
    assert cmake.index("ament_package()", pytest_index) > pytest_index
    assert cmake.rstrip().endswith("ament_package()")


def test_cmake_guards_future_artifacts_until_they_exist():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert (
        'if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_buffer.cpp"'
        in cmake
    )
    assert "src/pcd_chunk_writer.cpp" in cmake
    assert "src/pcd_projection.cpp" in cmake
    assert "src/pcd_to_grid.cpp" in cmake
    assert 'if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch"' in cmake
    assert 'IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config"' in cmake
    assert 'IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rviz")' in cmake
    assert "launch config rviz" in cmake
    assert 'if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${program}")' in cmake
    for program in (
        "go1_mapping/session_guard.py",
        "go1_mapping/map_finalizer.py",
        "go1_mapping/validation_report.py",
    ):
        assert program in cmake
