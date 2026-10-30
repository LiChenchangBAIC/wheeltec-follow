cmake_minimum_required(VERSION 3.8)
project(grab_demo)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV 4.8 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(arm_demo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)


include_directories(
  include #找到本功能包下的include 里的头文件
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/

)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GrabObject.srv"
  "srv/PlaceObject.srv"
  "srv/MoveNamed.srv"
  "srv/MoveRelative.srv"
  "srv/MovePose.srv"
  "srv/MoveToFrame.srv"
  "srv/MoveCartesian.srv"
  "srv/MoveJoints.srv"
  "srv/PlaceToFrame.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)



add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)

ament_target_dependencies(pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(pointcloud_filter_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_filter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(arm_motion_service src/arm_motion_service.cpp)

target_include_directories(arm_motion_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_motion_service
  arm_demo
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(
  arm_motion_service
  "${cpp_typesupport_target}"
  ${arm_demo_LIBRARIES}
)

install(TARGETS arm_motion_service
  DESTINATION lib/${PROJECT_NAME}
)






add_executable(aruco_marker_tf_pub_node src/aruco_marker_tf_pub_node.cpp)

ament_target_dependencies(aruco_marker_tf_pub_node
  rclcpp
  aruco_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS aruco_marker_tf_pub_node
  DESTINATION lib/${PROJECT_NAME}
)







add_executable(grab_place_service src/grab_place_service.cpp)
target_include_directories(grab_place_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(grab_place_service
  arm_demo
  rclcpp
  tf2 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rm_ros_interfaces
)
install(TARGETS grab_place_service
  DESTINATION lib/${PROJECT_NAME})

  rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(
  grab_place_service 
  "${cpp_typesupport_target}"
  ${arm_demo_LIBRARIES}
)
 


add_executable(charuco_dectet_node src/charuco_dectet_node.cpp)
target_include_directories(charuco_dectet_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(charuco_dectet_node
${OpenCV_LIBS}

)

ament_target_dependencies(charuco_dectet_node
 rclcpp
 Eigen3
 image_transport
 geometry_msgs
 sensor_msgs
 cv_bridge
 OpenCV
 moveit_ros_planning_interface
)
install(TARGETS charuco_dectet_node
  DESTINATION lib/${PROJECT_NAME})

  

add_executable(hand_eye src/hand_eye.cpp)
target_include_directories(hand_eye PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hand_eye
${OpenCV_LIBS}

)
ament_target_dependencies(hand_eye
 arm_demo
 rclcpp
 Eigen3
 image_transport
 geometry_msgs
 sensor_msgs
 cv_bridge
 OpenCV
 moveit_ros_planning_interface
 moveit_core
 tf2_ros
 tf2_geometry_msgs
 tf2
)
install(TARGETS hand_eye
  DESTINATION lib/${PROJECT_NAME})


add_executable(camera_undistort_node src/camera_undistort_node.cpp)
target_link_libraries(camera_undistort_node
${OpenCV_LIBS}

)
ament_target_dependencies(camera_undistort_node
 arm_demo
 rclcpp
 Eigen3
 image_transport
 geometry_msgs
 sensor_msgs
 cv_bridge
 OpenCV
 moveit_ros_planning_interface
 moveit_core
 tf2_ros
 tf2_geometry_msgs
 tf2
)
install(TARGETS camera_undistort_node
  DESTINATION lib/${PROJECT_NAME})




  add_executable(s300_hand_eye src/s300_hand_eye.cpp)
  target_include_directories(s300_hand_eye PUBLIC ${OpenCV_INCLUDE_DIRS})

  target_link_libraries(s300_hand_eye
  ${OpenCV_LIBS}
  
  )
  ament_target_dependencies(s300_hand_eye
   arm_demo
   rclcpp
   Eigen3
   image_transport
   geometry_msgs
   sensor_msgs
   cv_bridge
   OpenCV
   moveit_ros_planning_interface
   moveit_core
   tf2_ros
   tf2_geometry_msgs
   tf2
  )
  install(TARGETS s300_hand_eye
    DESTINATION lib/${PROJECT_NAME})




add_executable(color_grab_and_place_service src/color_grab_and_place_service.cpp)
target_include_directories(color_grab_and_place_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(color_grab_and_place_service
  arm_demo
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rm_ros_interfaces
)
install(TARGETS color_grab_and_place_service
  DESTINATION lib/${PROJECT_NAME})

  rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(
  color_grab_and_place_service 
  "${cpp_typesupport_target}"
  ${arm_demo_LIBRARIES}
)




add_executable(hsv_range src/hsv_range.cpp)
target_link_libraries(hsv_range
${OpenCV_LIBS}
  opencv_highgui 
)
ament_target_dependencies(hsv_range
  rclcpp
  Eigen3
  image_transport
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  message_filters
  tf2_geometry_msgs
)
install(TARGETS hsv_range
  DESTINATION lib/${PROJECT_NAME})





add_executable(visual_servo src/visual_servo.cpp)
target_link_libraries(visual_servo
${OpenCV_LIBS}
  opencv_highgui 
)
ament_target_dependencies(visual_servo
  rclcpp
  Eigen3
  image_transport 
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  message_filters
  tf2_geometry_msgs
  rm_ros_interfaces
  kdl_parser
  orocos_kdl
)
install(TARGETS visual_servo
  DESTINATION lib/${PROJECT_NAME})











add_executable(start_grab src/start_grab.cpp)
ament_target_dependencies(start_grab
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  moveit_ros_planning_interface
)
install(TARGETS start_grab
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(start_grab "${cpp_typesupport_target}")


add_executable(camera_info_node src/camera_info_node.cpp)

ament_target_dependencies(camera_info_node
 rclcpp
 sensor_msgs
)
install(TARGETS camera_info_node
  DESTINATION lib/${PROJECT_NAME})

  target_link_libraries(camera_info_node
  ${YAML_CPP_LIBRARIES}
)


  add_executable(nav_grab src/nav_grab.cpp)
  ament_target_dependencies(nav_grab
   rclcpp
   geometry_msgs
   nav2_msgs
   rclcpp_action
  )
  install(TARGETS nav_grab
    DESTINATION lib/${PROJECT_NAME})
  
    rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  
  target_link_libraries(nav_grab "${cpp_typesupport_target}")




install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)





install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
