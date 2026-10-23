cmake_minimum_required(VERSION 3.5)
project(nav2_custom_behaviors)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
  ${nav2_behaviors_INCLUDE_DIRS}
)

add_library(
  nav2_back_up_safe SHARED
  plugins/back_up_safe.cpp
)

target_include_directories(nav2_back_up_safe PUBLIC include)

ament_target_dependencies(nav2_back_up_safe
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS nav2_back_up_safe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES behavior_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
