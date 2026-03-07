# cmake_minimum_required(VERSION 2.8.3)

# project(Rover Assembly URDF)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

cmake_minimum_required(VERSION 3.8)
project(rover_assembly_urdf)

# Find Ament dependencies
find_package(ament_cmake REQUIRED)

# Install directories (config, launch, meshes, urdf)
# This replaces your foreach loop and ensures folders are copied to the install space
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export the package to the Ament index
ament_package()