#include "rclcpp/rclcpp.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
using namespace std::chrono_literals;
int main(int argc, char* argv[])
{  
  // 读取命令行参数中的目标tf坐标
  std::string target_tf_frame=argv[1];

  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 创建节点实例
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client_node");

  // 创建服务客户端
  rclcpp::Client<grab_demo::srv::GrabObject>::SharedPtr client =node->create_client<grab_demo::srv::GrabObject>("/obj_grab_service");

  // 创建服务请求对象
  auto request =std::make_shared<grab_demo::srv::GrabObject::Request>();
  request->obj_link=target_tf_frame;

  // 设置等待时间
  std::chrono::seconds myPause = 1s;

  // 循环等待服务可用
  while (!client->wait_for_service(1s))
  {
    // 检查ROS2节点是否运行正常
    if (!rclcpp::ok())
    {
      // 输出错误信息并退出
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // 输出服务不可用信息
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "SERVICE NOT AVAILABLE, waiting again...");
  }

  // 异步发送服务请求
  auto result = client->async_send_request(request);

  // 等待服务响应
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    // 输出服务调用成功信息
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sucess");
  } else 
  {
    // 输出服务调用失败信息
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_cap_full_name");
  }

  // 关闭ROS2节点
  rclcpp::shutdown();
  return 0;
}
