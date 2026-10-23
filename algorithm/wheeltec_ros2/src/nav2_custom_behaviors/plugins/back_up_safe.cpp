// Copyright (c) 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Custom backup behavior with:
// 1. Configurable costmap collision threshold (not just LETHAL_OBSTACLE=254)
// 2. Direction search: if straight back is blocked, try angled retreat paths
// 3. Graduated deceleration when approaching the cost threshold

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_custom_behaviors/back_up_safe.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_custom_behaviors
{

BackUpSafe::BackUpSafe()
: TimedBehavior<BackUpAction>(),
  feedback_(std::make_shared<BackUpAction::Feedback>()),
  command_x_(0.0),
  command_speed_(0.0),
  simulate_ahead_time_(2.0),
  max_cost_threshold_(200.0),
  search_angle_(0.26),     // ~15 degrees
  search_steps_(3),
  linear_acc_lim_(0.0),
  linear_decel_lim_(0.0),
  min_linear_vel_(0.05),
  max_linear_vel_(0.3),
  last_vel_(0.0)
{
}

void BackUpSafe::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // --- Declare and read parameters ---
  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".max_cost_threshold",
    rclcpp::ParameterValue(200.0));
  node->get_parameter(behavior_name_ + ".max_cost_threshold", max_cost_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".search_angle",
    rclcpp::ParameterValue(0.26));
  node->get_parameter(behavior_name_ + ".search_angle", search_angle_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".search_steps",
    rclcpp::ParameterValue(3));
  node->get_parameter(behavior_name_ + ".search_steps", search_steps_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".linear_acc_lim",
    rclcpp::ParameterValue(0.3));
  node->get_parameter(behavior_name_ + ".linear_acc_lim", linear_acc_lim_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".linear_decel_lim",
    rclcpp::ParameterValue(-0.5));
  node->get_parameter(behavior_name_ + ".linear_decel_lim", linear_decel_lim_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".min_linear_vel",
    rclcpp::ParameterValue(0.05));
  node->get_parameter(behavior_name_ + ".min_linear_vel", min_linear_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".max_linear_vel",
    rclcpp::ParameterValue(0.3));
  node->get_parameter(behavior_name_ + ".max_linear_vel", max_linear_vel_);

  // Validate
  if (linear_acc_lim_ < 0.0) { linear_acc_lim_ = std::abs(linear_acc_lim_); }
  if (linear_decel_lim_ > 0.0) { linear_decel_lim_ = -std::abs(linear_decel_lim_); }
  if (min_linear_vel_ < 0.0) { min_linear_vel_ = std::abs(min_linear_vel_); }

  RCLCPP_INFO(
    logger_,
    "%s configured: max_cost_threshold=%.1f, search_angle=%.2f rad (%.1f°), "
    "search_steps=%d, linear_acc=%.2f, linear_decel=%.2f",
    behavior_name_.c_str(),
    max_cost_threshold_, search_angle_, search_angle_ * 180.0 / M_PI,
    search_steps_, linear_acc_lim_, linear_decel_lim_);
}

void BackUpSafe::onCleanup()
{
  last_vel_ = 0.0;
}

void BackUpSafe::onActionCompletion()
{
  last_vel_ = 0.0;
}

nav2_behaviors::Status BackUpSafe::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "BackUpSafe: Y and Z movement not supported, only X.");
    return nav2_behaviors::Status::FAILED;
  }

  // Force negative direction: backward
  command_x_ = -std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  command_dist_ = std::fabs(command->target.x);
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "BackUpSafe: Initial robot pose not available.");
    return nav2_behaviors::Status::FAILED;
  }

  RCLCPP_INFO(
    logger_,
    "BackUpSafe running: distance=%.2f m, speed=%.2f m/s, threshold=%.0f",
    command_dist_, std::fabs(command_speed_), max_cost_threshold_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpSafe::onCycleUpdate()
{
  // --- Timeout check ---
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "BackUpSafe: Time allowance exceeded.");
    return nav2_behaviors::Status::FAILED;
  }

  // --- Get current pose ---
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "BackUpSafe: Current robot pose not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // --- Check distance traveled ---
  double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance_traveled = std::hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance_traveled;
  action_server_->publish_feedback(feedback_);

  if (distance_traveled >= command_dist_) {
    stopRobot();
    RCLCPP_INFO(logger_, "BackUpSafe: Target distance reached.");
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // --- Compute velocity command with acceleration limits ---
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  double target_speed = command_speed_;  // negative

  if (linear_acc_lim_ == 0.0 || linear_decel_lim_ == 0.0) {
    cmd_vel.linear.x = target_speed;
  } else {
    double current_speed = last_vel_;
    double min_feasible =
      current_speed + linear_decel_lim_ / cycle_frequency_;  // more negative = faster decel
    double max_feasible =
      current_speed + linear_acc_lim_ / cycle_frequency_;    // less negative

    cmd_vel.linear.x = std::clamp(target_speed, min_feasible, max_feasible);

    // Decelerate to stop before reaching target distance
    double remaining = command_dist_ - distance_traveled;
    double max_vel_to_stop = -std::sqrt(
      2.0 * std::abs(linear_decel_lim_) * remaining);
    if (cmd_vel.linear.x < max_vel_to_stop) {
      cmd_vel.linear.x = max_vel_to_stop;
    }
  }

  // Enforce minimum velocity
  if (std::fabs(cmd_vel.linear.x) < min_linear_vel_) {
    cmd_vel.linear.x = -min_linear_vel_;
  }
  // Enforce maximum velocity
  if (std::fabs(cmd_vel.linear.x) > max_linear_vel_) {
    cmd_vel.linear.x = -max_linear_vel_;
  }

  // --- Safety check with direction search ---
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!findSafeDirection(pose2d, cmd_vel)) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "BackUpSafe: No safe backward direction found (cost threshold %.0f).",
      max_cost_threshold_);
    return nav2_behaviors::Status::FAILED;
  }

  last_vel_ = cmd_vel.linear.x;
  vel_pub_->publish(cmd_vel);

  return nav2_behaviors::Status::RUNNING;
}

bool BackUpSafe::isPoseSafe(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_data)
{
  try {
    double score = collision_checker_->scorePose(pose, fetch_data);
    return score < max_cost_threshold_;
  } catch (...) {
    return false;
  }
}

bool BackUpSafe::findSafeDirection(
  const geometry_msgs::msg::Pose2D & pose2d,
  geometry_msgs::msg::Twist & cmd_vel)
{
  const double dt = 1.0 / cycle_frequency_;
  const int max_steps = static_cast<int>(simulate_ahead_time_ * cycle_frequency_);

  // --- Candidate generator ---
  struct Candidate {
    double vx_scale;     // 1.0 = full request speed
    double angular_z;    // rad/s, 0 = straight
  };

  std::vector<Candidate> candidates;
  candidates.push_back({1.0, 0.0});  // straight back first

  for (int step = 1; step <= search_steps_; ++step) {
    double angle = search_angle_ * static_cast<double>(step) / search_steps_;
    double speed_scale = 1.0 - 0.15 * step;
    if (speed_scale < 0.3) { speed_scale = 0.3; }
    candidates.push_back({speed_scale, angle});
    candidates.push_back({speed_scale, -angle});
  }

  // --- Try each candidate ---
  // Only the very first pose check across all candidates fetches costmap
  bool costmap_fetched = false;

  for (const auto & candidate : candidates) {
    geometry_msgs::msg::Twist test_vel;
    test_vel.linear.x = cmd_vel.linear.x * candidate.vx_scale;
    test_vel.linear.y = 0.0;
    test_vel.angular.z = candidate.angular_z;

    // Iterative simulation — handles curved paths when angular_z != 0
    geometry_msgs::msg::Pose2D sim_pose = pose2d;
    bool safe = true;

    for (int step = 0; step < max_steps; ++step) {
      // One time-step of differential drive kinematics
      sim_pose.theta += test_vel.angular.z * dt;
      sim_pose.x += test_vel.linear.x * cos(sim_pose.theta) * dt;
      sim_pose.y += test_vel.linear.x * sin(sim_pose.theta) * dt;

      // Normalize yaw
      sim_pose.theta = atan2(sin(sim_pose.theta), cos(sim_pose.theta));

      bool fetch = !costmap_fetched;
      if (!isPoseSafe(sim_pose, fetch)) {
        safe = false;
        RCLCPP_DEBUG(
          logger_,
          "BackUpSafe: (vx=%.2f, wz=%.2f) blocked at step %d/%d, "
          "sim_pose=(%.2f, %.2f, %.2f)",
          test_vel.linear.x, test_vel.angular.z, step + 1, max_steps,
          sim_pose.x, sim_pose.y, sim_pose.theta);
        break;
      }
      costmap_fetched = true;  // subsequent checks skip fetch

      // Early exit once remaining distance is covered
      double sim_dist = std::fabs(test_vel.linear.x) * static_cast<double>(step + 1) * dt;
      double remaining = command_dist_ - feedback_->distance_traveled;
      if (sim_dist >= remaining) { break; }
    }

    if (safe) {
      cmd_vel.linear.x = test_vel.linear.x;
      cmd_vel.angular.z = test_vel.angular.z;

      if (candidate.angular_z != 0.0) {
        RCLCPP_INFO(
          logger_,
          "BackUpSafe: Angled retreat chosen (vx=%.2f, wz=%.2f rad/s).",
          cmd_vel.linear.x, cmd_vel.angular.z);
      }
      return true;
    }
  }

  return false;
}

}  // namespace nav2_custom_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_custom_behaviors::BackUpSafe,
  nav2_core::Behavior)
