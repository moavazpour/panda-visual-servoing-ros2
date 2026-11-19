cmake_minimum_required(VERSION 3.8)
project(arm_vs_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(move_to_poses src/move_to_poses.cpp)
ament_target_dependencies(move_to_poses
  rclcpp rclcpp_action control_msgs trajectory_msgs)

install(TARGETS move_to_poses
  DESTINATION lib/${PROJECT_NAME})

ament_package()