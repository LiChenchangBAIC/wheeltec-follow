#include "arm_demo/moveit_base.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include "grab_demo/srv/place_object.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rm_ros_interfaces/msg/gripperset.hpp"
 
int main(int argc, char* argv[])
{ 
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grab_service_node"); 
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("moveit_base_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // init some important variables
  static const std::string PLANNING_GROUP = "rm_group";
  static const std::string PLANNING_EEF = "grasp_point";

  // start moveit
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  RealMan rm(&move_group,PLANNING_EEF);
  
  // 创建 TF 监听器
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  
  // 创建抓取服务 
  auto grab_service = node->create_service<grab_demo::srv::GrabObject>(
    "obj_grab_service",
    [&rm, &tf_buffer,&node](
      const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,
      std::shared_ptr<grab_demo::srv::GrabObject::Response> res)
    {
      rclcpp::Publisher<rm_ros_interfaces::msg::Gripperset>::SharedPtr girpper_pub=node->create_publisher<rm_ros_interfaces::msg::Gripperset>("/rm_driver/set_gripper_position_cmd",10);
      auto girpper_message=rm_ros_interfaces::msg::Gripperset();
      girpper_message.position=1000;
      girpper_pub->publish(girpper_message);

      try {
        // 获取目标物体的变换
        geometry_msgs::msg::TransformStamped tfs = tf_buffer->lookupTransform(
          "base_link", req->obj_link, rclcpp::Time(0), rclcpp::Duration::from_seconds(10));

        // 获取当前末端位姿
        auto cur_pose = rm.get_current_pose();
        tf2::Quaternion q(tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z, tfs.transform.rotation.w);
        
        double roll, pitch, yaw;

        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        yaw=fabs(yaw)-1.5708;
        std::cout << "yaw: " << yaw*57.295 << std::endl;
        if(yaw>0.7076)
        {
          yaw=(1.5708-yaw);
        }
        else
        {
          yaw=-yaw;
        }


        rclcpp::sleep_for(std::chrono::seconds(1));

        // 移动到目标位置
        rm.moveP(tfs.transform.translation.x,
                 tfs.transform.translation.y,
                 tfs.transform.translation.z,
                 0,
                 0,
                 yaw*57.295779513082);
        rclcpp::sleep_for(std::chrono::seconds(1));
        girpper_message.position=100;
        girpper_pub->publish(girpper_message);

        rclcpp::sleep_for(std::chrono::seconds(2));

        // 返回观察位置
        rm.moveP(cur_pose);

      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node->get_logger(), "Could not transform: %s", ex.what());

      }
    });

auto place_service = node->create_service<grab_demo::srv::PlaceObject>(
  "obj_place_service",
  [&rm, &node](const std::shared_ptr<grab_demo::srv::PlaceObject::Request> /*req*/,
               std::shared_ptr<grab_demo::srv::PlaceObject::Response> /*res*/) 
  { 
    auto pub = node->create_publisher<rm_ros_interfaces::msg::Gripperset>("/rm_driver/set_gripper_position_cmd",10);
    rm.move_Place(); 
    rclcpp::sleep_for(std::chrono::seconds(1));
    rm_ros_interfaces::msg::Gripperset msg; 
    msg.position = 1000;
    pub->publish(msg); 
    rclcpp::sleep_for(std::chrono::seconds(1));
    rm.move_Look();
  });


  // 等待服务请求
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
