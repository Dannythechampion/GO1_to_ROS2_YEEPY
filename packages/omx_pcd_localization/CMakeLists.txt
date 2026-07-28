cmake_minimum_required(VERSION 3.8)
project(omx_pcd_localization)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)

add_executable(pcd_localizer src/pcd_localizer.cpp)
ament_target_dependencies(
  pcd_localizer
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_include_directories(pcd_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_localizer ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS pcd_localizer DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pcd_localization_contract
    test/test_pcd_localization_contract.py
    TIMEOUT 60
  )
endif()

ament_package()
