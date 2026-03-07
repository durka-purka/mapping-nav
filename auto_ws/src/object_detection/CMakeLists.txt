cmake_minimum_required(VERSION 3.8)
project(object_detection)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Python package is handled by setup.py, not ament_python_install_package

# Install Python executables
# Uncomment when ready to install
# install(PROGRAMS
#   object_detection/object_reporter.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install directories
install(DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
  OPTIONAL
)

ament_package()
