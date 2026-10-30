#include <arm_demo/moveit_base.hpp>
int main(int argc, char* argv[])
{
    // 初始化rclcpp  
  rclcpp::init(argc, argv);

    // 创建名为"fk_demo_node"的节点
  auto move_group_node = rclcpp::Node::make_shared("move_circle_node");

    // 创建一个单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
    // 将节点添加到执行器中
  executor.add_node(move_group_node);
    // 启动执行器，并在新的线程中运行
  std::thread([&executor]() { executor.spin(); }).detach();
  
    // 定义规划组名称
  static const std::string PLANNING_GROUP = "rm_group";
    // 定义末端执行器名称
  static const std::string PLANNING_EEF = "gripper";

    // 创建MoveGroupInterface对象
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // 创建RealMan对象
  RealMan rm(&move_group, PLANNING_EEF);
  rm.set_MaxVelocity_Accelerate(1.0,1.0);
  rm.move_Look();
    // 执行移动操作
  rm.move_C("zx", 0.05);
    // 等待1秒
  rclcpp::sleep_for(std::chrono::seconds(1));

  rm.move_Look();

    // 关闭rclcpp
  rclcpp::shutdown();
  return 0;
}
