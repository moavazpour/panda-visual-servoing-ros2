cmake_minimum_required(VERSION 3.8)
project(arm_vs_servoing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(visual_servo src/visual_servo.cpp)
ament_target_dependencies(visual_servo
  rclcpp geometry_msgs visualization_msgs std_msgs tf2 tf2_ros tf2_eigen)
target_link_libraries(visual_servo Eigen3::Eigen)

add_executable(pick_cube src/pick_cube.cpp)
ament_target_dependencies(pick_cube
  rclcpp geometry_msgs std_msgs sensor_msgs tf2 tf2_ros tf2_eigen)
target_link_libraries(pick_cube Eigen3::Eigen)

install(TARGETS visual_servo pick_cube
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()