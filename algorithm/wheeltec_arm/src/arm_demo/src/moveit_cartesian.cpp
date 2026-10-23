#include <arm_demo/moveit_base.hpp>
int main(int argc, char* argv[])
{ 
    // 初始化ROS节点
  rclcpp::init(argc, argv);

    // 创建并初始化节点
  auto move_group_node = rclcpp::Node::make_shared("moveit_cartesian_node");

    // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
    // 启动执行器
  std::thread([&executor]() { executor.spin(); }).detach();
  
    // 定义规划组和末端执行器
  static const std::string PLANNING_GROUP = "rm_group";
  static const std::string PLANNING_EEF = "gripper";

    // 创建MoveGroupInterface对象
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // 创建RealMan对象
  RealMan rm(&move_group, PLANNING_EEF);

    // 设置最大速度和加速度
  rm.set_MaxVelocity_Accelerate(1.0, 1.0); // 设置运动速度，如需更快的速度可在rm_description/urdf/rm_eco65.urdf 文件修改各个joint的velocity值

  rm.move_Look();

  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> poses;
  pose = rm.get_current_pose();
  poses.push_back(pose);

  pose.position.y = 0.2;
  poses.push_back(pose);

  pose.position.y = 0;
  pose.position.z = 0.3;
  poses.push_back(pose);

  pose.position.y = -0.2;
  pose.position.z = 0.2;
  poses.push_back(pose);

  pose.position.y = 0;
  poses.push_back(pose);

    // 执行笛卡尔路径规划
  rm.moveCartesian(poses);

  rm.move_Look();

    // 关闭ROS节点
  rclcpp::shutdown();
  return 0;
}
