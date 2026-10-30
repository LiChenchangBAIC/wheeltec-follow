#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <exception>

#include "rclcpp/rclcpp.hpp"

#include "arm_demo/moveit_base.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include "grab_demo/srv/place_object.hpp"
#include "grab_demo/srv/place_to_frame.hpp"
#include "rm_ros_interfaces/msg/gripperset.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

void publishGripper(
  const rclcpp::Publisher<rm_ros_interfaces::msg::Gripperset>::SharedPtr & pub,
  int position)
{
  rm_ros_interfaces::msg::Gripperset msg;
  msg.position = position;
  pub->publish(msg);
}

geometry_msgs::msg::Pose makePoseKeepOrientation(
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::TransformStamped & tfs,
  double offset_x,
  double offset_y,
  double offset_z)
{
  geometry_msgs::msg::Pose target_pose = current_pose;

  target_pose.position.x = tfs.transform.translation.x + offset_x;
  target_pose.position.y = tfs.transform.translation.y + offset_y;
  target_pose.position.z = tfs.transform.translation.z + offset_z;

  // 只使用 TF 的位置，姿态保持当前末端姿态。
  // 垂直观察时就是垂直抓，水平观察时就是水平抓。
  target_pose.orientation = current_pose.orientation;

  return target_pose;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions service_options;

  rclcpp::NodeOptions moveit_options;
  moveit_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("grab_service_node", service_options);
  auto move_group_node = rclcpp::Node::make_shared("moveit_base_node", moveit_options);

  node->declare_parameter<std::string>("base_frame", "base_link");
  node->declare_parameter<double>("tf_timeout_sec", 10.0);

  node->declare_parameter<double>("grab_offset_x", 0.0);
  node->declare_parameter<double>("grab_offset_y", 0.0);
  node->declare_parameter<double>("grab_offset_z", 0.0);

  node->declare_parameter<bool>("return_after_grab", true);

  node->declare_parameter<double>("place_approach_z", 0.05);
  node->declare_parameter<bool>("return_look_after_fixed_place", true);
  node->declare_parameter<bool>("return_look_after_frame_place", false);

  node->declare_parameter<int>("gripper_open_position", 1000);
  node->declare_parameter<int>("gripper_close_position", 200);

  const std::string base_frame = node->get_parameter("base_frame").as_string();
  const double tf_timeout_sec = node->get_parameter("tf_timeout_sec").as_double();

  const double grab_offset_x = node->get_parameter("grab_offset_x").as_double();
  const double grab_offset_y = node->get_parameter("grab_offset_y").as_double();
  const double grab_offset_z = node->get_parameter("grab_offset_z").as_double();

  const bool return_after_grab = node->get_parameter("return_after_grab").as_bool();

  const double place_approach_z = node->get_parameter("place_approach_z").as_double();
  const bool return_look_after_fixed_place =
    node->get_parameter("return_look_after_fixed_place").as_bool();
  const bool return_look_after_frame_place =
    node->get_parameter("return_look_after_frame_place").as_bool();

  const int gripper_open_position = node->get_parameter("gripper_open_position").as_int();
  const int gripper_close_position = node->get_parameter("gripper_close_position").as_int();

  rclcpp::executors::SingleThreadedExecutor moveit_executor;
  moveit_executor.add_node(move_group_node);

  std::thread moveit_spin_thread([&moveit_executor]() {
    moveit_executor.spin();
  });

  static const std::string PLANNING_GROUP = "rm_group";
  static const std::string PLANNING_EEF = "grasp_point";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  RealMan rm(&move_group, PLANNING_EEF);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto gripper_pub = node->create_publisher<rm_ros_interfaces::msg::Gripperset>(
    "/rm_driver/set_gripper_position_cmd",
    10);

  auto grab_service = node->create_service<grab_demo::srv::GrabObject>(
    "obj_grab_service",
    [
      &rm,
      &tf_buffer,
      &node,
      gripper_pub,
      base_frame,
      tf_timeout_sec,
      grab_offset_x,
      grab_offset_y,
      grab_offset_z,
      return_after_grab,
      gripper_open_position,
      gripper_close_position
    ](
      const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,
      std::shared_ptr<grab_demo::srv::GrabObject::Response> res)
    {
      if (req->obj_link.empty()) {
        res->success = false;
        res->message = "grab failed: obj_link is empty";
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
        return;
      }

      try {
        RCLCPP_INFO(
          node->get_logger(),
          "Grab service called. target=%s",
          req->obj_link.c_str());

        publishGripper(gripper_pub, gripper_open_position);
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        geometry_msgs::msg::TransformStamped tfs = tf_buffer->lookupTransform(
          base_frame,
          req->obj_link,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(tf_timeout_sec));

        auto current_pose = rm.get_current_pose();

        geometry_msgs::msg::Pose target_pose = makePoseKeepOrientation(
          current_pose,
          tfs,
          grab_offset_x,
          grab_offset_y,
          grab_offset_z);

        RCLCPP_INFO(
          node->get_logger(),
          "Grab target pose: frame=%s, x=%.3f, y=%.3f, z=%.3f",
          req->obj_link.c_str(),
          target_pose.position.x,
          target_pose.position.y,
          target_pose.position.z);

        rm.moveP(target_pose);

        rclcpp::sleep_for(std::chrono::seconds(1));

        publishGripper(gripper_pub, gripper_close_position);

        rclcpp::sleep_for(std::chrono::seconds(1));

        if (return_after_grab) {
          rm.moveP(current_pose);
        }

        res->success = true;
        res->message = std::string("grab finished: ") + req->obj_link;

        RCLCPP_INFO(node->get_logger(), "%s", res->message.c_str());
      } catch (const tf2::TransformException & ex) {
        res->success = false;
        res->message = std::string("grab failed, TF lookup failed: ") + ex.what();
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      } catch (const std::exception & ex) {
        res->success = false;
        res->message = std::string("grab failed: ") + ex.what();
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      } catch (...) {
        res->success = false;
        res->message = "grab failed: unknown exception";
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      }
    });

  auto place_service = node->create_service<grab_demo::srv::PlaceObject>(
    "obj_place_service",
    [
      &rm,
      &node,
      gripper_pub,
      gripper_open_position,
      return_look_after_fixed_place
    ](
      const std::shared_ptr<grab_demo::srv::PlaceObject::Request> /*req*/,
      std::shared_ptr<grab_demo::srv::PlaceObject::Response> res)
    {
      try {
        RCLCPP_INFO(node->get_logger(), "Fixed place service called.");

        rm.move_Place();

        rclcpp::sleep_for(std::chrono::seconds(1));

        publishGripper(gripper_pub, gripper_open_position);

        rclcpp::sleep_for(std::chrono::seconds(1));

        if (return_look_after_fixed_place) {
          rm.move_Look();
        }

        res->success = true;
        res->message = return_look_after_fixed_place
          ? "fixed place finished and returned to look"
          : "fixed place finished";

        RCLCPP_INFO(node->get_logger(), "%s", res->message.c_str());
      } catch (const std::exception & ex) {
        res->success = false;
        res->message = std::string("fixed place failed: ") + ex.what();
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      } catch (...) {
        res->success = false;
        res->message = "fixed place failed: unknown exception";
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      }
    });

  auto place_to_frame_service = node->create_service<grab_demo::srv::PlaceToFrame>(
    "obj_place_to_frame_service",
    [
      &rm,
      &tf_buffer,
      &node,
      gripper_pub,
      base_frame,
      tf_timeout_sec,
      place_approach_z,
      return_look_after_frame_place,
      gripper_open_position
    ](
      const std::shared_ptr<grab_demo::srv::PlaceToFrame::Request> req,
      std::shared_ptr<grab_demo::srv::PlaceToFrame::Response> res)
    {
      if (req->target_frame.empty()) {
        res->success = false;
        res->message = "place_to_frame failed: target_frame is empty";
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
        return;
      }

      try {
        geometry_msgs::msg::TransformStamped tfs = tf_buffer->lookupTransform(
          base_frame,
          req->target_frame,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(tf_timeout_sec));

        auto current_pose = rm.get_current_pose();

        geometry_msgs::msg::Pose approach_pose = makePoseKeepOrientation(
          current_pose,
          tfs,
          req->offset_x,
          req->offset_y,
          req->offset_z + place_approach_z);

        geometry_msgs::msg::Pose release_pose = makePoseKeepOrientation(
          current_pose,
          tfs,
          req->offset_x,
          req->offset_y,
          req->offset_z);

        RCLCPP_INFO(
          node->get_logger(),
          "Place to frame target=%s release=(%.3f, %.3f, %.3f), keep current orientation",
          req->target_frame.c_str(),
          release_pose.position.x,
          release_pose.position.y,
          release_pose.position.z);

        rm.moveP(approach_pose);

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        rm.moveP(release_pose);

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        publishGripper(gripper_pub, gripper_open_position);

        rclcpp::sleep_for(std::chrono::seconds(1));

        rm.moveP(approach_pose);

        if (return_look_after_frame_place) {
          rm.move_Look();
        }

        res->success = true;
        res->message = std::string("place_to_frame finished: ") + req->target_frame;

        RCLCPP_INFO(node->get_logger(), "%s", res->message.c_str());
      } catch (const tf2::TransformException & ex) {
        res->success = false;
        res->message = std::string("place_to_frame failed, TF lookup failed: ") + ex.what();
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      } catch (const std::exception & ex) {
        res->success = false;
        res->message = std::string("place_to_frame failed: ") + ex.what();
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      } catch (...) {
        res->success = false;
        res->message = "place_to_frame failed: unknown exception";
        RCLCPP_ERROR(node->get_logger(), "%s", res->message.c_str());
      }
    });

  RCLCPP_INFO(node->get_logger(), "Universal grab/place service ready.");
  RCLCPP_INFO(
    node->get_logger(),
    "Grab rule: target TF provides position, current end-effector pose provides orientation.");

  rclcpp::spin(node); 

  moveit_executor.cancel();

  if (moveit_spin_thread.joinable()) {
    moveit_spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}