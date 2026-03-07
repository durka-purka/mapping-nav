cmake_minimum_required(VERSION 3.8)
project(realsense_launch)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install launch files (if any are added at this level)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# Install configuration files (if any are added at this level)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# Install the realsense-ros packages as subdependencies
install(DIRECTORY realsense-ros/
  DESTINATION share/${PROJECT_NAME}/realsense-ros
)

ament_package()
