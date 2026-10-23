#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "vlm_demo/srv/resolve_target_pose.hpp"

namespace
{
// 把任意标签清洗成合法的 TF 帧名片段
std::string sanitizeFramePart(const std::string & input)
{
  std::string out;
  out.reserve(input.size());
  for (char c : input) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      out.push_back(static_cast<char>(std::tolower(uc)));
    } else if (c == '_' || c == '-') {
      out.push_back(c);
    }
  }
  if (out.empty()) { out = "target"; }
  return out;
}
}  // namespace

class GeometryResolverNode : public rclcpp::Node
{
public:
  GeometryResolverNode()
  : Node("geometry_resolver_node")
  {
    depth_topic_         = declare_parameter<std::string>("depth_topic", "/camera_arm/depth/image_raw");
    camera_info_topic_   = declare_parameter<std::string>("camera_info_topic", "/gemini_info");
    camera_frame_        = declare_parameter<std::string>("camera_frame", "camera_arm_color_optical_frame");
    depth_window_        = declare_parameter<int>("depth_window", 7);
    tf_keepalive_sec_    = declare_parameter<double>("tf_keepalive_sec", 15.0);
    default_target_frame_= declare_parameter<std::string>("default_target_frame", "vlm_target_frame");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GeometryResolverNode::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&GeometryResolverNode::cameraInfoCallback, this, std::placeholders::_1));
    resolve_service_ = create_service<vlm_demo::srv::ResolveTargetPose>(
      "/vlm_demo/resolve_target_pose",
      std::bind(&GeometryResolverNode::handleResolve, this,
                std::placeholders::_1, std::placeholders::_2));

    // 持续刷新已生成的 TF,保证 lookupTransform 拿得到最新时间戳
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&GeometryResolverNode::republishTransforms, this));

    RCLCPP_INFO(get_logger(), "几何反投影节点就绪。");
  }

private:
  struct StoredTransform
  {
    geometry_msgs::msg::TransformStamped transform;
    std::chrono::steady_clock::time_point created_at;
  };


  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg);
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_depth_ = cv_ptr->image.clone();
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "深度图 cv_bridge 转换失败: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_info_ = *msg;
    has_camera_info_ = true;
    if (!msg->header.frame_id.empty()) { camera_frame_ = msg->header.frame_id; }
  }

  void handleResolve(
    const std::shared_ptr<vlm_demo::srv::ResolveTargetPose::Request> req,
    std::shared_ptr<vlm_demo::srv::ResolveTargetPose::Response> res)
  {
    cv::Mat depth;
    sensor_msgs::msg::CameraInfo info;
    std::string camera_frame;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_depth_.empty()) { res->success = false; res->message = "尚未收到深度图"; return; }
      if (!has_camera_info_)     { res->success = false; res->message = "尚未收到 camera_info"; return; }
      depth = latest_depth_.clone();
      info = latest_info_;
      camera_frame = camera_frame_;
    }

    const int u = req->u, v = req->v;
    auto depth_m = getDepthMeters(depth, u, v);
    if (!depth_m.has_value()) { res->success = false; res->message = "目标像素附近无有效深度"; return; }

    const double fx = info.k[0], fy = info.k[4], cx = info.k[2], cy = info.k[5];
    if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9) {
      res->success = false; res->message = "相机内参无效"; return;
    }

    // 针孔模型反投影
    const double z = depth_m.value();
    const double x = (static_cast<double>(u) - cx) * z / fx;
    const double y = (static_cast<double>(v) - cy) * z / fy;

    std::string target_frame = req->frame_id_hint;
    if (target_frame.empty()) { target_frame = default_target_frame_; }
    if (target_frame.empty()) { target_frame = "vlm_" + sanitizeFramePart(req->target_label) + "_frame"; }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = camera_frame;
    tf.child_frame_id = target_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = 1.0;  // 仅给位置,姿态由抓取服务按需补

    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      active_tfs_[target_frame] = StoredTransform{tf, std::chrono::steady_clock::now()};
    }
    tf_broadcaster_->sendTransform(tf);

    res->success = true;
    res->message = "目标位姿已解析";
    res->target_frame = target_frame;
    res->depth_m = static_cast<float>(z);
    res->camera_x = static_cast<float>(x);
    res->camera_y = static_cast<float>(y);
    res->camera_z = static_cast<float>(z);
    res->target_pose.header = tf.header;
    res->target_pose.pose.position.x = x;
    res->target_pose.pose.position.y = y;
    res->target_pose.pose.position.z = z;
    res->target_pose.pose.orientation = tf.transform.rotation;

    RCLCPP_INFO(get_logger(), "像素(%d,%d) -> 帧=%s xyz=(%.3f,%.3f,%.3f)",
                u, v, target_frame.c_str(), x, y, z);
  }

  // 取 (u,v) 邻域窗口内的有效深度中值(米),抗噪 + 抗空洞
  std::optional<double> getDepthMeters(const cv::Mat & depth, int u, int v) const
  {
    if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) { return std::nullopt; }
    const int half = std::max(1, depth_window_ / 2);
    const int x1 = std::max(0, u - half), x2 = std::min(depth.cols - 1, u + half);
    const int y1 = std::max(0, v - half), y2 = std::min(depth.rows - 1, v + half);

    std::vector<double> values;
    values.reserve(static_cast<size_t>((x2 - x1 + 1) * (y2 - y1 + 1)));
    for (int y = y1; y <= y2; ++y) {
      for (int x = x1; x <= x2; ++x) {
        double z = 0.0;
        if      (depth.type() == CV_16UC1) { z = static_cast<double>(depth.at<uint16_t>(y, x)) / 1000.0; }
        else if (depth.type() == CV_32FC1) { z = static_cast<double>(depth.at<float>(y, x)); }
        else if (depth.type() == CV_64FC1) { z = depth.at<double>(y, x); }
        else { return std::nullopt; }
        if (std::isfinite(z) && z > 0.05) { values.push_back(z); }
      }
    }
    if (values.empty()) { return std::nullopt; }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  }

  // 定时重发未过期的 TF,过期的清理掉
  void republishTransforms()
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    const auto now_t = std::chrono::steady_clock::now();
    for (auto it = active_tfs_.begin(); it != active_tfs_.end();) {
      const double age = std::chrono::duration<double>(now_t - it->second.created_at).count();
      if (age > tf_keepalive_sec_) { it = active_tfs_.erase(it); continue; }
      it->second.transform.header.stamp = now();
      tf_broadcaster_->sendTransform(it->second.transform);
      ++it;
    }
  }

  std::string depth_topic_, camera_info_topic_, camera_frame_, default_target_frame_;
  int depth_window_ = 7;
  double tf_keepalive_sec_ = 15.0;

  std::mutex data_mutex_;
  cv::Mat latest_depth_;
  sensor_msgs::msg::CameraInfo latest_info_;
  bool has_camera_info_ = false;

  std::mutex tf_mutex_;
  std::unordered_map<std::string, StoredTransform> active_tfs_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Service<vlm_demo::srv::ResolveTargetPose>::SharedPtr resolve_service_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeometryResolverNode>());
  rclcpp::shutdown();
  return 0;
}