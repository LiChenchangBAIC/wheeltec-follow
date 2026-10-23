#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ArucoMarkersToTfNode : public rclcpp::Node
{
public:
  ArucoMarkersToTfNode()
  : Node("aruco_markers_to_tf_node")
  {
    this->declare_parameter<std::string>("markers_topic", "/arm_aruco/markers");
    this->declare_parameter<std::string>(
      "default_parent_frame", "camera_arm_color_optical_frame");
    this->declare_parameter<std::string>(
      "child_frame_prefix", "aruco_");
    this->declare_parameter<std::string>(
      "child_frame_suffix", "_frame");

    const std::string markers_topic =
      this->get_parameter("markers_topic").as_string();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    markers_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
      markers_topic,
      10,
      std::bind(
        &ArucoMarkersToTfNode::markersCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to %s, publishing marker TF as aruco_<id>_frame",
      markers_topic.c_str());
  }

private:
  void markersCallback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::string parent_frame = msg->header.frame_id;

    if (parent_frame.empty()) {
      parent_frame = this->get_parameter("default_parent_frame").as_string();
    }

    const std::string child_prefix =
      this->get_parameter("child_frame_prefix").as_string();
    const std::string child_suffix =
      this->get_parameter("child_frame_suffix").as_string();

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(msg->markers.size());

    for (const auto & marker : msg->markers) {
      geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.stamp = msg->header.stamp;
      tf_msg.header.frame_id = parent_frame;
      tf_msg.child_frame_id =
        child_prefix + std::to_string(marker.id) + child_suffix;

      tf_msg.transform.translation.x = marker.pose.pose.position.x;
      tf_msg.transform.translation.y = marker.pose.pose.position.y;
      tf_msg.transform.translation.z = marker.pose.pose.position.z;

      tf_msg.transform.rotation.x = marker.pose.pose.orientation.x;
      tf_msg.transform.rotation.y = marker.pose.pose.orientation.y;
      tf_msg.transform.rotation.z = marker.pose.pose.orientation.z;
      tf_msg.transform.rotation.w = marker.pose.pose.orientation.w;

      transforms.push_back(tf_msg);
    }

    if (!transforms.empty()) {
      tf_broadcaster_->sendTransform(transforms);
    }
  }

  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMarkersToTfNode>());
  rclcpp::shutdown();
  return 0;
}
