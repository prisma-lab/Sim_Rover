cmake_minimum_required(VERSION 3.5)
project(rover_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# target_link_libraries(rover_manager
#   ${rclcpp_LIBRARIES}
#   ${rclcpp_action_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
#   ${nav2_msgs_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${std_msgs_LIBRARIES}
# )

include_directories(
  include
)

add_executable(rover_manager src/rover_manager.cpp)
ament_target_dependencies(rover_manager rclcpp std_msgs tf2 tf2_ros geometry_msgs nav2_msgs)

install(TARGETS
  rover_manager
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
