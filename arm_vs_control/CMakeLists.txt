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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Library: kinematics ---
add_library(arm_vs_kinematics src/kinematics.cpp)
target_include_directories(arm_vs_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_vs_kinematics kdl_parser orocos_kdl)
target_link_libraries(arm_vs_kinematics Eigen3::Eigen)

# --- Program from day 4 ---
add_executable(move_to_poses src/move_to_poses.cpp)
ament_target_dependencies(move_to_poses
  rclcpp rclcpp_action control_msgs trajectory_msgs)

# --- Program from day 5 ---
add_executable(fk_monitor src/fk_monitor.cpp)
target_link_libraries(fk_monitor arm_vs_kinematics)
ament_target_dependencies(fk_monitor rclcpp sensor_msgs std_msgs)

# --- Program from day 6 ---
add_executable(cartesian_velocity_node src/cartesian_velocity_node.cpp)
target_link_libraries(cartesian_velocity_node arm_vs_kinematics)
ament_target_dependencies(cartesian_velocity_node
  rclcpp sensor_msgs std_msgs geometry_msgs)

install(TARGETS arm_vs_kinematics move_to_poses fk_monitor cartesian_velocity_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()