#ifndef NAV2_CUSTOM_BEHAVIORS__BACK_UP_SAFE_HPP_
#define NAV2_CUSTOM_BEHAVIORS__BACK_UP_SAFE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_custom_behaviors
{

using BackUpAction = nav2_msgs::action::BackUp;

/**
 * @class BackUpSafe
 * @brief A backup behavior with configurable costmap collision threshold
 * and directional search. Unlike the default BackUp which only checks
 * for LETHAL_OBSTACLE (254), this behavior allows setting a custom
 * cost threshold so the robot stops before entering the inflation layer.
 *
 * When the straight-back direction is blocked, it searches ±search_angle
 * for a safe direction instead of immediately failing.
 */
class BackUpSafe : public nav2_behaviors::TimedBehavior<BackUpAction>
{
public:
  BackUpSafe();
  ~BackUpSafe() = default;

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const BackUpAction::Goal> command) override;

  nav2_behaviors::Status onCycleUpdate() override;

  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion() override;

protected:
  /**
   * @brief Check if a given 2D pose is collision-free based on the configured cost threshold
   * @param fetch_data Whether to fetch fresh costmap and footprint data (true for first call)
   */
  bool isPoseSafe(const geometry_msgs::msg::Pose2D & pose, bool fetch_data);

  /**
   * @brief Simulate the backup trajectory and check for collisions.
   * If blocked, search adjacent headings for a safe direction.
   * @param pose2d Current robot pose
   * @param cmd_vel Current candidate velocity command (in/out - may be modified)
   * @return true if a safe direction was found, false if all blocked
   */
  bool findSafeDirection(
    const geometry_msgs::msg::Pose2D & pose2d,
    geometry_msgs::msg::Twist & cmd_vel);

  BackUpAction::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_x_;
  double command_speed_;
  double command_dist_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

  // Configurable parameters
  double simulate_ahead_time_;
  double max_cost_threshold_;    // costmap cost above which is considered collision (default 200)
  double search_angle_;          // +/- angle in radians to search for safe direction (default 0.26 = ~15°)
  int search_steps_;             // number of steps within search_angle (default 3)
  double linear_acc_lim_;        // linear acceleration limit
  double linear_decel_lim_;      // linear deceleration limit (negative)
  double min_linear_vel_;        // minimum linear velocity
  double max_linear_vel_;        // maximum linear velocity (backward, negative)

  double last_vel_;
};

}  // namespace nav2_custom_behaviors

#endif  // NAV2_CUSTOM_BEHAVIORS__BACK_UP_SAFE_HPP_
