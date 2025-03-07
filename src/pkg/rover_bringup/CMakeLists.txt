cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(nav2_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2 2.50.0)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(rtabmap_ros REQUIRED)
find_package(teb_local_planner REQUIRED)


nav2_package()




    set(dependencies
    cv_bridge
    image_transport
    rclcpp
    rclcpp_components
    realsense2_camera_msgs
    std_msgs
    std_srvs
    sensor_msgs
    nav_msgs
    tf2
    realsense2
    tf2_ros
    diagnostic_updater
    teb_local_planner
  )

install(
  DIRECTORY launch params maps config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
