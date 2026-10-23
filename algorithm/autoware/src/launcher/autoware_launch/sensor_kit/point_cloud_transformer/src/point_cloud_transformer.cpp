#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "autoware/point_types/types.hpp"
#include "point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

using autoware::point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;
class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer(const rclcpp::NodeOptions &options)
  : Node("point_cloud_transformer", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    point_cloud_sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/point_cloud_raw", rclcpp::QoS(5),
      std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    // point_cloud_sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "/lidar/top/point_cloud_raw", rclcpp::QoS(5),
    //   std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated/pointcloud", 10);
    
    while (!tf_buffer_.canTransform("base_link", "laser", rclcpp::Time())) {
      //RCLCPP_WARN(get_logger(), "Waiting for TF transform between base_link and radar_frame...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    try {
      transform = tf_buffer_.lookupTransform("base_link", "laser", rclcpp::Time());

    } catch (tf2::TransformException &ex) {
      //RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //auto oldstamp = msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>); //autoware::point_types::PointXYZIRC
    pcl::fromROSMsg(*msg, *pcl_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*pcl_cloud, *pcl_transformed_cloud, tf2::transformToEigen(transform.transform).cast<float>());
    pcl::toROSMsg(*pcl_transformed_cloud, *transformed_cloud);
    //transformed_cloud->header.stamp = oldstamp;
    transformed_cloud->header.stamp = this->now();
    transformed_cloud->header.frame_id = "base_link";
    convertToXYZIRCCloud(transformed_cloud,output_cloud);
    point_cloud_pub_->publish(*output_cloud);

  }

void convertToXYZIRCCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr)
{
  output_ptr->header = input_ptr->header;

  PointCloud2Modifier<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator> output_modifier{
    *output_ptr, input_ptr->header.frame_id};
  output_modifier.reserve(input_ptr->width);

  bool has_valid_intensity =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
      return field.name == "intensity";
    });

  bool has_valid_return_type =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
      return field.name == "return_type";
    });

  bool has_valid_channel =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
      return field.name == "channel";
    });
  sensor_msgs::PointCloud2Iterator<float> it_x(*input_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*input_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*input_ptr, "z");

  if (has_valid_intensity && has_valid_return_type && has_valid_channel) {
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_i(*input_ptr, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_r(*input_ptr, "return_type");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> it_c(*input_ptr, "channel");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      point.return_type = *it_r;
      point.channel = *it_c;
      output_modifier.push_back(std::move(point));
    }
  } else if(has_valid_intensity) {
    sensor_msgs::PointCloud2Iterator<float> it_i(*input_ptr, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z,++it_i) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      output_modifier.push_back(std::move(point));
    }
  }
  else
  {
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      output_modifier.push_back(std::move(point));
    }
  }
}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transform;
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudTransformer)