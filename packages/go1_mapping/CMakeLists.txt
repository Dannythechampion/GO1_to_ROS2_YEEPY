cmake_minimum_required(VERSION 3.8)
project(go1_mapping)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_python_install_package(${PROJECT_NAME})

set(mapping_targets)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_buffer.cpp")
  add_library(pcd_chunk_buffer src/pcd_chunk_buffer.cpp)
  target_compile_features(pcd_chunk_buffer PUBLIC cxx_std_17)
  target_include_directories(pcd_chunk_buffer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcd_chunk_buffer ${PCL_LIBRARIES})
  list(APPEND mapping_targets pcd_chunk_buffer)
endif()

if(TARGET pcd_chunk_buffer
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_storage.cpp")
  add_library(pcd_chunk_storage src/pcd_chunk_storage.cpp)
  target_compile_features(pcd_chunk_storage PUBLIC cxx_std_17)
  target_include_directories(pcd_chunk_storage PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcd_chunk_storage pcd_chunk_buffer ${PCL_LIBRARIES})
  ament_target_dependencies(pcd_chunk_storage sensor_msgs)
  list(APPEND mapping_targets pcd_chunk_storage)
endif()

if(TARGET pcd_chunk_storage
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_chunk_writer.cpp")
  add_executable(pcd_chunk_writer src/pcd_chunk_writer.cpp)
  target_compile_features(pcd_chunk_writer PRIVATE cxx_std_17)
  target_link_libraries(
    pcd_chunk_writer pcd_chunk_storage pcd_chunk_buffer ${PCL_LIBRARIES})
  ament_target_dependencies(
    pcd_chunk_writer rclcpp rcl_interfaces sensor_msgs std_srvs pcl_conversions)
  list(APPEND mapping_targets pcd_chunk_writer)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_projection.cpp")
  add_library(pcd_projection src/pcd_projection.cpp)
  target_compile_features(pcd_projection PUBLIC cxx_std_17)
  target_include_directories(pcd_projection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcd_projection ${PCL_LIBRARIES})
  list(APPEND mapping_targets pcd_projection)
endif()

if(TARGET pcd_projection
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_to_grid.cpp")
  add_executable(pcd_to_grid src/pcd_to_grid.cpp)
  target_compile_features(pcd_to_grid PRIVATE cxx_std_17)
  target_link_libraries(pcd_to_grid pcd_projection ${PCL_LIBRARIES})
  list(APPEND mapping_targets pcd_to_grid)
endif()

if(mapping_targets)
  install(TARGETS
    ${mapping_targets}
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
else()
  foreach(directory IN ITEMS launch config rviz)
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
      install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
    endif()
  endforeach()
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
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
  ament_add_pytest_test(test_session_guard test/test_session_guard.py)

  if(TARGET pcd_chunk_buffer
     AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pcd_chunk_buffer.cpp")
    ament_add_gtest(
      test_pcd_chunk_buffer test/test_pcd_chunk_buffer.cpp)
    target_include_directories(test_pcd_chunk_buffer PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS})
    target_link_libraries(
      test_pcd_chunk_buffer pcd_chunk_buffer ${PCL_LIBRARIES})
  endif()

  if(TARGET pcd_chunk_storage
     AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pcd_chunk_storage.cpp")
    ament_add_gtest(
      test_pcd_chunk_storage test/test_pcd_chunk_storage.cpp)
    target_include_directories(test_pcd_chunk_storage PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS})
    target_link_libraries(
      test_pcd_chunk_storage pcd_chunk_storage pcd_chunk_buffer ${PCL_LIBRARIES})
    ament_target_dependencies(test_pcd_chunk_storage sensor_msgs)
  endif()

  if(TARGET pcd_projection
     AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pcd_projection.cpp")
    ament_add_gtest(
      test_pcd_projection test/test_pcd_projection.cpp)
    target_include_directories(test_pcd_projection PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS})
    target_link_libraries(
      test_pcd_projection pcd_projection ${PCL_LIBRARIES})
  endif()
endif()

ament_package()
