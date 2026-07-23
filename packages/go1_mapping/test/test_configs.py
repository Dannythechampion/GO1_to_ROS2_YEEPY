from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]


def load(name):
    return yaml.safe_load((ROOT / "config" / name).read_text())


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


def test_fast_lio_mapping_disables_internal_pcd_and_heavy_map():
    params = load("fast_lio_mapping_safe.yaml")["/**"]["ros__parameters"]
    assert params["pcd_save"]["pcd_save_en"] is False
    assert params["pcd_save"]["interval"] == 300
    assert params["publish"]["map_en"] is False
    assert params["publish"]["effect_map_en"] is False
    assert params["publish"]["dense_publish_en"] is False
    assert params["publish"]["scan_publish_en"] is True
    assert params["publish"]["scan_bodyframe_pub_en"] is True


def test_scan_projection_is_low_latency_body_frame():
    params = load("pointcloud_to_scan_mapping.yaml")[
        "pointcloud_to_laserscan"
    ]["ros__parameters"]
    assert params["target_frame"] == "body"
    assert params["queue_size"] == 1
    assert params["scan_time"] == 0.1
    assert params["range_min"] == 0.5
    assert params["range_max"] == 20.0


def test_slam_owns_only_map_slam_to_camera_init():
    params = load("slam_toolbox_hanyang_9f.yaml")["slam_toolbox"]["ros__parameters"]
    assert params["map_frame"] == "map_slam"
    assert params["odom_frame"] == "camera_init"
    assert params["base_frame"] == "body"
    assert params["scan_topic"] == "/scan"
    assert params["mode"] == "mapping"
    assert params["do_loop_closing"] is True
    assert params["resolution"] == 0.05
    assert params["scan_queue_size"] == 1


def test_session_limits_match_design():
    params = load("mapping_session.yaml")["mapping_session"]
    assert params["frames_per_chunk"] == 300
    assert params["max_buffer_bytes"] == 268435456
    assert params["preflight_free_gib"] == 100
    assert params["abort_free_gib"] == 50
    assert params["bag_split_bytes"] == 4294967296
