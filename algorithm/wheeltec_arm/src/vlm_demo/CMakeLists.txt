cmake_minimum_required(VERSION 3.8)
project(vlm_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GroundTargets.srv"
  "srv/ResolveTargetPose.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(geometry_resolver_node src/geometry_resolver_node.cpp)
target_include_directories(geometry_resolver_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(geometry_resolver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
)
target_link_libraries(geometry_resolver_node
  "${cpp_typesupport_target}"
  ${OpenCV_LIBS}
)
install(TARGETS geometry_resolver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/command_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_node
)
install(PROGRAMS scripts/vision_grounding_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vision_grounding_node
)
install(PROGRAMS scripts/pick_task_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_task_manager_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
