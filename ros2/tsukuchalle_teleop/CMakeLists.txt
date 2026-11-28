cmake_minimum_required(VERSION 3.16)
project(tsukuchalle_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# The exported teleop_twist_joy target links these (transitive, already in the image); find them first.
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(teleop_twist_joy REQUIRED)

add_executable(limited_teleop_node src/limited_teleop_node.cpp)
target_include_directories(limited_teleop_node PRIVATE include)
target_link_libraries(limited_teleop_node PRIVATE rclcpp::rclcpp teleop_twist_joy::teleop_twist_joy)

install(TARGETS limited_teleop_node DESTINATION lib/${PROJECT_NAME})

ament_package()
