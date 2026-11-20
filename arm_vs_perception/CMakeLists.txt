cmake_minimum_required(VERSION 3.8)
project(arm_vs_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

add_executable(marker_detector src/marker_detector.cpp)
target_include_directories(marker_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(marker_detector ${OpenCV_LIBS})
ament_target_dependencies(marker_detector
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge tf2 tf2_ros)

install(TARGETS marker_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()