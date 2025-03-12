cmake_minimum_required(VERSION 3.8)
project(nav2_astar_planner)
set(CMAKE_CXX_STANDARD 14)

# Dipendenze necessarie
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)


set(library_name ${PROJECT_NAME}_plugin)

# Aggiunta della libreria del plugin
add_library(${library_name} SHARED
  src/a_star_planner.cpp
)
include_directories(
        include
)


# Collegamento delle dipendenze alla libreria
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Esportazione del plugin
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# Installazione del plugin
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Installazione del file XML del plugin
install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Aggiunta della dichiarazione Ament
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
