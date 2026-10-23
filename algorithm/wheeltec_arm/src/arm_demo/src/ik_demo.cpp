#include <arm_demo/moveit_base.hpp>
int main(int argc, char* argv[])
{ 
    // 初始化ROS节点
  rclcpp::init(argc, argv);

    // 创建ROS节点
  auto move_group_node = rclcpp::Node::make_shared("ik_demo_node");

    // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
    // 启动执行器
  std::thread([&executor]() { executor.spin(); }).detach();
  
    // 定义规划组和末端执行器
  static const std::string PLANNING_GROUP = "rm_group";
  static const std::string PLANNING_EEF = "Link6";

    // 创建MoveGroup接口对象
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // 创建RealMan对象
  RealMan rm(&move_group, PLANNING_EEF);
  rm.set_MaxVelocity_Accelerate(1.0,1.0);
  rm.move_Look();
  geometry_msgs::msg::Pose current_pose=rm.get_current_pose();
    // 调用RealMan对象的moveP方法
  rm.moveP(current_pose.position.x,
           current_pose.position.y,
           current_pose.position.z+0.05,
           current_pose.orientation.x,
           current_pose.orientation.y,
           current_pose.orientation.z,
           current_pose.orientation.w);
    // 等待2秒
  rclcpp::sleep_for(std::chrono::seconds(2));

    // move_Look
  rm.move_Look();

    // 关闭ROS
  rclcpp::shutdown();
  return 0;
}
