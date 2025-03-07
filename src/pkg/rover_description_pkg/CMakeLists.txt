cmake_minimum_required(VERSION 3.8)
project(rover_description_pkg)

set(CMAKE_CXX_STANDARD 14)


## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)

ament_package()