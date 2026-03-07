cmake_minimum_required(VERSION 3.8)
project(auto_ws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Find all packages in the workspace
find_package(navigation REQUIRED)
find_package(object_detection REQUIRED)
find_package(realsense_launch REQUIRED)
find_package(ros2_mpu6500 REQUIRED)
# Rover Assembly URDF is for URDF/visualization, included implicitly

ament_package()
