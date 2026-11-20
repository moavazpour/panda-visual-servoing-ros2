cmake_minimum_required(VERSION 3.8)
project(arm_vs_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds models rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()