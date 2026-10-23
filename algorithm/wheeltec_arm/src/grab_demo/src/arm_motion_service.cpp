#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "grab_demo/srv/move_named.hpp"
#include "grab_demo/srv/move_relative.hpp"
#include "grab_demo/srv/move_pose.hpp"
#include "grab_demo/srv/move_to_frame.hpp"
#include "grab_demo/srv/move_cartesian.hpp"
#include "grab_demo/srv/move_joints.hpp"

class ArmMotionService
{
public:
  ArmMotionService(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
    const std::string & end_effector_link)
  : node_(node),
    move_group_(move_group),
    end_effector_link_(end_effector_link)
  {
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<double>("default_velocity_scale", 0.3);
    node_->declare_parameter<double>("default_acceleration_scale", 0.3);
    node_->declare_parameter<double>("tf_timeout_sec", 2.0);
    node_->declare_parameter<bool>("invert_x", false);
    node_->declare_parameter<bool>("invert_y", false);

    node_->get_parameter("invert_x", invert_x_);
    node_->get_parameter("invert_y", invert_y_);


    RCLCPP_INFO(
    node_->get_logger(), 
    "Robot Config Loaded: invert_x = %s, invert_y = %s", 
    invert_x_ ? "true" : "false", 
    invert_y_ ? "true" : "false"
  );

    base_frame_ = node_->get_parameter("base_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->allowReplanning(true);
    move_group_->setPlanningTime(10.0);

    move_named_srv_ = node_->create_service<grab_demo::srv::MoveNamed>(
      "/arm/move_named",
      std::bind(&ArmMotionService::handleMoveNamed, this, std::placeholders::_1, std::placeholders::_2));

    move_relative_srv_ = node_->create_service<grab_demo::srv::MoveRelative>(
      "/arm/move_relative",
      std::bind(&ArmMotionService::handleMoveRelative, this, std::placeholders::_1, std::placeholders::_2));

    move_pose_srv_ = node_->create_service<grab_demo::srv::MovePose>(
      "/arm/move_pose",
      std::bind(&ArmMotionService::handleMovePose, this, std::placeholders::_1, std::placeholders::_2));

    move_to_frame_srv_ = node_->create_service<grab_demo::srv::MoveToFrame>(
      "/arm/move_to_frame",
      std::bind(&ArmMotionService::handleMoveToFrame, this, std::placeholders::_1, std::placeholders::_2));

    move_cartesian_srv_ = node_->create_service<grab_demo::srv::MoveCartesian>(
      "/arm/move_cartesian",
      std::bind(&ArmMotionService::handleMoveCartesian, this, std::placeholders::_1, std::placeholders::_2));

    move_joints_srv_ = node_->create_service<grab_demo::srv::MoveJoints>(
      "/arm/move_joints",
      std::bind(&ArmMotionService::handleMoveJoints, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/arm/stop",
      std::bind(&ArmMotionService::handleStop, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "Arm motion services are ready.");
  }

private:
  static double degToRad(double value)
  {
    return value * M_PI / 180.0;
  }
  
  static double clampScale(double value, double fallback)
  {
    if (value <= 0.0) {
      value = fallback;
    }
    if (value < 0.01) {
      return 0.01;
    }
    if (value > 1.0) {
      return 1.0;
    }
    return value;
  }

  static bool isZeroQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::abs(q.x) < 1e-9 &&
           std::abs(q.y) < 1e-9 &&
           std::abs(q.z) < 1e-9 &&
           std::abs(q.w) < 1e-9;
  }

  void applyScaling(double velocity_scale, double acceleration_scale)
  {
    const double default_v = node_->get_parameter("default_velocity_scale").as_double();
    const double default_a = node_->get_parameter("default_acceleration_scale").as_double();

    move_group_->setMaxVelocityScalingFactor(clampScale(velocity_scale, default_v));
    move_group_->setMaxAccelerationScalingFactor(clampScale(acceleration_scale, default_a));
  }

  geometry_msgs::msg::Pose currentPose()
  {
    return move_group_->getCurrentPose(end_effector_link_).pose;
  }

  bool transformPoseToBase(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output,
    std::string & message)
  {
    std::string source_frame = input.header.frame_id.empty() ? base_frame_ : input.header.frame_id;

    if (source_frame == base_frame_) {
      output = input;
      output.header.frame_id = base_frame_;
      return true;
    }

    try {
      const double timeout = node_->get_parameter("tf_timeout_sec").as_double();
      auto tfs = tf_buffer_->lookupTransform(
        base_frame_,
        source_frame,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(timeout));

      tf2::doTransform(input, output, tfs);
      output.header.frame_id = base_frame_;
      return true;
    } catch (const tf2::TransformException & ex) {
      message = ex.what();
      return false;
    }
  }

  bool transformPointToBase(
    const geometry_msgs::msg::PointStamped & input,
    geometry_msgs::msg::PointStamped & output,
    std::string & message)
  {
    std::string source_frame = input.header.frame_id.empty() ? base_frame_ : input.header.frame_id;

    if (source_frame == base_frame_) {
      output = input;
      output.header.frame_id = base_frame_;
      return true;
    }

    try {
      const double timeout = node_->get_parameter("tf_timeout_sec").as_double();
      auto tfs = tf_buffer_->lookupTransform(
        base_frame_,
        source_frame,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(timeout));

      tf2::doTransform(input, output, tfs);
      output.header.frame_id = base_frame_;
      return true;
    } catch (const tf2::TransformException & ex) {
      message = ex.what();
      return false;
    }
  }

  bool planAndExecutePose(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    double acceleration_scale,
    std::string & message)
  {
    applyScaling(velocity_scale, acceleration_scale);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPoseTarget(target_pose, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    move_group_->clearPoseTargets();

    if (!plan_result) {
      message = "MoveIt pose planning failed.";
      return false;
    }

    auto exec_result = move_group_->execute(plan);
    if (!exec_result) {
      message = "MoveIt pose execution failed.";
      return false;
    }

    message = "Move pose succeeded.";
    return true;
  }

  void handleMoveNamed(
    const std::shared_ptr<grab_demo::srv::MoveNamed::Request> req,
    std::shared_ptr<grab_demo::srv::MoveNamed::Response> res)
  {
    applyScaling(req->velocity_scale, req->acceleration_scale);

    if (!move_group_->setNamedTarget(req->target_name)) {
      res->success = false;
      res->message = "Unknown named target: " + req->target_name;
      return;
    }

    move_group_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (!plan_result) {
      res->success = false;
      res->message = "Move named planning failed.";
      return;
    }

    auto exec_result = move_group_->execute(plan);

    res->success = static_cast<bool>(exec_result);
    res->message = res->success ? "Move named succeeded." : "Move named execution failed.";
  }

  void handleMoveRelative(
    const std::shared_ptr<grab_demo::srv::MoveRelative::Request> req,
    std::shared_ptr<grab_demo::srv::MoveRelative::Response> res)
  {
    const std::string reference_frame =
      req->reference_frame.empty() ? base_frame_ : req->reference_frame;

    double input_dx = invert_x_ ? -(req->dx) : req->dx;
    double input_dy = invert_y_ ? -(req->dy) : req->dy;

    geometry_msgs::msg::PointStamped origin_ref;
    origin_ref.header.frame_id = reference_frame;
    origin_ref.header.stamp = node_->now();
    origin_ref.point.x = 0.0;
    origin_ref.point.y = 0.0;
    origin_ref.point.z = 0.0;

    geometry_msgs::msg::PointStamped offset_ref = origin_ref;
    offset_ref.point.x = input_dx;
    offset_ref.point.y = input_dy;
    offset_ref.point.z = req->dz;

    geometry_msgs::msg::PointStamped origin_base;
    geometry_msgs::msg::PointStamped offset_base;

    std::string tf_message;
    if (!transformPointToBase(origin_ref, origin_base, tf_message) ||
        !transformPointToBase(offset_ref, offset_base, tf_message))
    {
      res->success = false;
      res->message = "TF transform failed: " + tf_message;
      return;
    }

    const double dx = offset_base.point.x - origin_base.point.x;
    const double dy = offset_base.point.y - origin_base.point.y;
    const double dz = offset_base.point.z - origin_base.point.z;

    auto target_pose = currentPose();
    target_pose.position.x += dx;
    target_pose.position.y += dy;
    target_pose.position.z += dz;

    res->success = planAndExecutePose(
      target_pose,
      req->velocity_scale,
      req->acceleration_scale,
      res->message);
  }

  void handleMovePose(
    const std::shared_ptr<grab_demo::srv::MovePose::Request> req,
    std::shared_ptr<grab_demo::srv::MovePose::Response> res)
  {
    geometry_msgs::msg::PoseStamped target_base;
    std::string message;

    if (!transformPoseToBase(req->target_pose, target_base, message)) {
      res->success = false;
      res->message = "TF transform failed: " + message;
      return;
    }

    if (isZeroQuaternion(target_base.pose.orientation)) {
      target_base.pose.orientation = currentPose().orientation;
    }

    res->success = planAndExecutePose(
      target_base.pose,
      req->velocity_scale,
      req->acceleration_scale,
      res->message);
  }

  void handleMoveToFrame(
    const std::shared_ptr<grab_demo::srv::MoveToFrame::Request> req,
    std::shared_ptr<grab_demo::srv::MoveToFrame::Response> res)
  {
    const std::string reference_frame =
      req->reference_frame.empty() ? base_frame_ : req->reference_frame;

    geometry_msgs::msg::TransformStamped tfs;

    try {
      const double timeout = node_->get_parameter("tf_timeout_sec").as_double();
      tfs = tf_buffer_->lookupTransform(
        reference_frame,
        req->target_frame,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(timeout));
    } catch (const tf2::TransformException & ex) {
      res->success = false;
      res->message = "TF lookup failed: " + std::string(ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_ref;
    pose_ref.header.frame_id = reference_frame;
    pose_ref.header.stamp = node_->now();

    pose_ref.pose.position.x = tfs.transform.translation.x + req->offset_x;
    pose_ref.pose.position.y = tfs.transform.translation.y + req->offset_y;
    pose_ref.pose.position.z = tfs.transform.translation.z + req->offset_z;
    pose_ref.pose.orientation = tfs.transform.rotation;

    geometry_msgs::msg::PoseStamped pose_base;
    std::string message;

    if (!transformPoseToBase(pose_ref, pose_base, message)) {
      res->success = false;
      res->message = "TF transform failed: " + message;
      return;
    }

    if (req->keep_current_orientation) {
      pose_base.pose.orientation = currentPose().orientation;
    }

    res->success = planAndExecutePose(
      pose_base.pose,
      req->velocity_scale,
      req->acceleration_scale,
      res->message);
  }

  void handleMoveCartesian(
    const std::shared_ptr<grab_demo::srv::MoveCartesian::Request> req,
    std::shared_ptr<grab_demo::srv::MoveCartesian::Response> res)
  {
    if (req->waypoints.empty()) {
      res->success = false;
      res->fraction = 0.0;
      res->message = "No waypoints provided.";
      return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(req->waypoints.size());

    const auto current_pose = currentPose();

    for (const auto & waypoint : req->waypoints) {
      geometry_msgs::msg::PoseStamped waypoint_base;
      std::string message;

      if (!transformPoseToBase(waypoint, waypoint_base, message)) {
        res->success = false;
        res->fraction = 0.0;
        res->message = "TF transform failed: " + message;
        return;
      }

      if (req->keep_current_orientation || isZeroQuaternion(waypoint_base.pose.orientation)) {
        waypoint_base.pose.orientation = current_pose.orientation;
      }

      waypoints.push_back(waypoint_base.pose);
    }

    const double eef_step = req->eef_step > 0.0 ? req->eef_step : 0.01;
    const double jump_threshold = req->jump_threshold >= 0.0 ? req->jump_threshold : 0.0;
    const double min_fraction = req->min_fraction > 0.0 ? req->min_fraction : 0.95;

    moveit_msgs::msg::RobotTrajectory trajectory;

    move_group_->setStartStateToCurrentState();

    const double fraction = move_group_->computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory);

    res->fraction = fraction;

    if (fraction < min_fraction) {
      res->success = false;
      res->message = "Cartesian path fraction too low.";
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto exec_result = move_group_->execute(plan);

    res->success = static_cast<bool>(exec_result);
    res->message = res->success ? "Cartesian move succeeded." : "Cartesian execution failed.";
  }

  void handleMoveJoints(
    const std::shared_ptr<grab_demo::srv::MoveJoints::Request> req,
    std::shared_ptr<grab_demo::srv::MoveJoints::Response> res)
  {
    const auto all_joint_names = move_group_->getJointNames();
    const auto current_values = move_group_->getCurrentJointValues();

    if (all_joint_names.empty()) {
      res->success = false;
      res->message = "MoveIt returned empty joint names.";
      return;
    }

    if (current_values.size() != all_joint_names.size()) {
      res->success = false;
      res->message = "Current joint state size mismatch.";
      return;
    }

    if (req->values.empty()) {
      res->success = false;
      res->message = "values is empty.";
      return;
    }

    std::vector<double> target_values = current_values;

    auto convert_value = [req](double value) {
      return req->degrees ? degToRad(value) : value;
    };

    if (req->joint_names.empty()) {
      if (req->values.size() != all_joint_names.size()) {
        res->success = false;
        res->message =
          "Full joint target requires " +
          std::to_string(all_joint_names.size()) +
          " values, got " +
          std::to_string(req->values.size()) +
          ".";
        return;
      }

      for (size_t i = 0; i < req->values.size(); ++i) {
        const double value = convert_value(req->values[i]);
        target_values[i] = req->relative ? current_values[i] + value : value;
      }
    } else {
      if (req->joint_names.size() != req->values.size()) {
        res->success = false;
        res->message = "joint_names size must match values size.";
        return;
      }

      for (size_t i = 0; i < req->joint_names.size(); ++i) {
        const std::string & joint_name = req->joint_names[i];

        const auto it = std::find(
          all_joint_names.begin(),
          all_joint_names.end(),
          joint_name);

        if (it == all_joint_names.end()) {
          res->success = false;
          res->message = "Unknown joint name: " + joint_name;
          return;
        }

        const size_t joint_index = std::distance(all_joint_names.begin(), it);
        const double value = convert_value(req->values[i]);

        target_values[joint_index] =
          req->relative ? current_values[joint_index] + value : value;
      }
    }

    applyScaling(req->velocity_scale, req->acceleration_scale);

    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(target_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (!plan_result) {
      res->success = false;
      res->message = "Move joints planning failed.";
      return;
    }

    auto exec_result = move_group_->execute(plan);

    res->success = static_cast<bool>(exec_result);
    res->message = res->success ? "Move joints succeeded." : "Move joints execution failed.";
  }

  void handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    move_group_->stop();
    res->success = true;
    res->message = "Arm stop command sent.";
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string end_effector_link_;
  std::string base_frame_;
  bool invert_x_;
  bool invert_y_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<grab_demo::srv::MoveNamed>::SharedPtr move_named_srv_;
  rclcpp::Service<grab_demo::srv::MoveRelative>::SharedPtr move_relative_srv_;
  rclcpp::Service<grab_demo::srv::MovePose>::SharedPtr move_pose_srv_;
  rclcpp::Service<grab_demo::srv::MoveToFrame>::SharedPtr move_to_frame_srv_;
  rclcpp::Service<grab_demo::srv::MoveCartesian>::SharedPtr move_cartesian_srv_;
  rclcpp::Service<grab_demo::srv::MoveJoints>::SharedPtr move_joints_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // service_node 是我们自己写的服务节点。
  // 它里面会手动 declare_parameter，所以这里不要自动声明 launch 传入参数。
  rclcpp::NodeOptions service_options;

  // moveit_node 是给 MoveGroupInterface 用的。
  // MoveIt 经常需要自动接收 robot_description、robot_description_semantic 等参数。
  rclcpp::NodeOptions moveit_options;
  moveit_options.automatically_declare_parameters_from_overrides(true);

  auto service_node = rclcpp::Node::make_shared(
    "arm_motion_service_node",
    service_options);

  auto moveit_node = rclcpp::Node::make_shared(
    "arm_motion_moveit_node",
    moveit_options);

  rclcpp::executors::SingleThreadedExecutor moveit_executor;
  moveit_executor.add_node(moveit_node);

  std::thread moveit_spin_thread([&moveit_executor]() {
    moveit_executor.spin();
  });

  static const std::string PLANNING_GROUP = "rm_group";
  static const std::string PLANNING_EEF = "grasp_point";

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node,
      PLANNING_GROUP);

  ArmMotionService services(service_node, move_group, PLANNING_EEF);

  rclcpp::spin(service_node);

  moveit_executor.cancel();

  if (moveit_spin_thread.joinable()) {
    moveit_spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}