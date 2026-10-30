# 基础设置
cmake_minimum_required(VERSION 3.8)
project(arm_demo)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_BUILD_TYPE Release)


# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)

# 添加可执行文件：fk_demo
add_executable(fk_demo src/fk_demo.cpp)
target_include_directories(fk_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(fk_demo PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(fk_demo
  rclcpp
  moveit_ros_planning_interface
)

# 添加可执行文件：ik_demo
add_executable(ik_demo src/ik_demo.cpp)
target_include_directories(ik_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_demo PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(ik_demo
  rclcpp
  moveit_ros_planning_interface
)

add_executable(move_circle src/move_circle.cpp)
target_include_directories(move_circle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(move_circle PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(move_circle
  rclcpp
  moveit_ros_planning_interface
)

add_executable(moveit_cartesian src/moveit_cartesian.cpp)
target_include_directories(moveit_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(moveit_cartesian PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(moveit_cartesian
  rclcpp
  moveit_ros_planning_interface
)

add_executable(move_look src/move_look.cpp)
target_include_directories(move_look PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(move_look PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(move_look
  rclcpp
  moveit_ros_planning_interface
)

add_executable(move_right_side src/move_right_side.cpp)
target_include_directories(move_right_side PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(move_right_side PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(move_right_side
  rclcpp
  moveit_ros_planning_interface
)


# 安装可执行文件
install(TARGETS 
  fk_demo
  ik_demo
  move_circle
  moveit_cartesian
  move_look
  move_right_side
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

#output include
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()