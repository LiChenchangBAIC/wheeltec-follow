#include <arm_demo/moveit_base.hpp>
int main(int argc, char* argv[])
{ 
    // 初始化ROS节点
  rclcpp::init(argc, argv);

    // 创建并共享一个名为"move_right_side_node"的节点
  auto move_group_node = rclcpp::Node::make_shared("move_right_side_node");

    // 创建一个单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
    // 将节点添加到执行器中
  executor.add_node(move_group_node);
    // 启动执行器，使其在单独的线程中运行
  std::thread([&executor]() { executor.spin(); }).detach();
  
    // 定义规划组名称
  static const std::string PLANNING_GROUP = "rm_group";
    // 定义末端执行器名称
  static const std::string PLANNING_EEF = "gripper";

    // 创建MoveGroupInterface对象，用于运动规划
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // 创建RealMan对象，用于控制机械臂
  RealMan rm(&move_group, PLANNING_EEF);
  rm.set_MaxVelocity_Accelerate(1.0,1.0);

  rm.move_Right_side_look_vert();
    
    // 关闭ROS节点
  rclcpp::shutdown();
  return 0;
}
