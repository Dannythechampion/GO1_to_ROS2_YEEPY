cmake_minimum_required(VERSION 3.8)
project(go1_mapping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_python_install_package(${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_buffer.cpp"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_writer.cpp"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_projection.cpp"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_to_grid.cpp")
  add_library(pcd_chunk_buffer src/pcd_chunk_buffer.cpp)
  target_include_directories(pcd_chunk_buffer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcd_chunk_buffer ${PCL_LIBRARIES})

  add_executable(pcd_chunk_writer src/pcd_chunk_writer.cpp)
  target_link_libraries(pcd_chunk_writer pcd_chunk_buffer ${PCL_LIBRARIES})
  ament_target_dependencies(
    pcd_chunk_writer rclcpp sensor_msgs std_srvs pcl_conversions)

  add_library(pcd_projection src/pcd_projection.cpp)
  target_include_directories(pcd_projection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcd_projection ${PCL_LIBRARIES})

  add_executable(pcd_to_grid src/pcd_to_grid.cpp)
  target_link_libraries(pcd_to_grid pcd_projection ${PCL_LIBRARIES})

  install(TARGETS
    pcd_chunk_buffer pcd_chunk_writer pcd_projection pcd_to_grid
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/ DESTINATION include)
endif()

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch"
   AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config"
   AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
endif()

set(mapping_python_programs)
foreach(program IN ITEMS
    go1_mapping/session_guard.py
    go1_mapping/map_finalizer.py
    go1_mapping/validation_report.py)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${program}")
    list(APPEND mapping_python_programs ${program})
  endif()
endforeach()

if(mapping_python_programs)
  install(PROGRAMS
    ${mapping_python_programs}
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_configs test/test_configs.py)
endif()

ament_package()
