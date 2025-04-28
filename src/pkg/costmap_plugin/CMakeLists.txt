cmake_minimum_required(VERSION 3.8)
project(costmap_plugin)
set(CMAKE_CXX_STANDARD 14)

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)



set(lib_name ${PROJECT_NAME}_core)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pcl_conversions
    pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/gradient_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()