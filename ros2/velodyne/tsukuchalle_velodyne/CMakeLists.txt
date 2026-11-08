cmake_minimum_required(VERSION 3.22)
project(tsukuchalle_velodyne)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# -ffast-math and -march=native are deliberately absent: they change the rounding that the
# reference conversion (ros-drivers rawdata.cpp) defines.
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Static, so the node executable is the only file that needs shipping.
add_library(tsukuchalle_velodyne_core STATIC
  src/hdl32e_calibration.cpp
  src/hdl32e_converter.cpp)
target_include_directories(tsukuchalle_velodyne_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tsukuchalle_velodyne_core PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${velodyne_msgs_TARGETS}
  yaml-cpp::yaml-cpp)

add_executable(velodyne_pointcloud_node src/velodyne_pointcloud_node.cpp)
target_link_libraries(velodyne_pointcloud_node PRIVATE tsukuchalle_velodyne_core)

install(TARGETS velodyne_pointcloud_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  function(add_velodyne_gtest test_name)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} tsukuchalle_velodyne_core)
    target_compile_definitions(${test_name} PRIVATE
      TSUKUCHALLE_VELODYNE_CONFIG_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endfunction()

  add_velodyne_gtest(test_hdl32e_calibration)
  add_velodyne_gtest(test_hdl32e_converter)
  add_velodyne_gtest(benchmark_converter)
endif()

ament_package()
