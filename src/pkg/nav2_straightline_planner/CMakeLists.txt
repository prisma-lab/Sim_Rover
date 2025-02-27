cmake_minimum_required(VERSION 3.8)
project(nav2_straightline_planner)
set(CMAKE_CXX_STANDARD 14)

# Dipendenze necessarie
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)

# Aggiunta della libreria del plugin
add_library(${PROJECT_NAME} SHARED
  src/straight_line_planner.cpp)

include_directories(
        include
)


# Collegamento delle dipendenze alla libreria
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  pluginlib
  nav2_core
  tf2_ros
)

# Esportazione del plugin
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# Installazione del plugin
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installazione del file XML del plugin
install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Aggiunta della dichiarazione Ament
ament_package()
