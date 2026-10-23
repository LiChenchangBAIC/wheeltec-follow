#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode() : Node("pointcloud_filter_node")
    {
        this->declare_parameter<std::string>("input_topic", "/camera_arm/depth/points");
        this->declare_parameter<std::string>("output_topic", "/moveit_obstacle_cloud");

        // 工作空间裁剪范围
        this->declare_parameter<double>("x_min", -0.5);
        this->declare_parameter<double>("x_max",  0.5);
        this->declare_parameter<double>("y_min", -0.5);
        this->declare_parameter<double>("y_max",  0.5);
        this->declare_parameter<double>("z_min",  0.15);
        this->declare_parameter<double>("z_max",  1.2);

        // 体素降采样
        this->declare_parameter<double>("voxel_leaf_size", 0.02);

        // 离群点过滤
        this->declare_parameter<bool>("use_sor", true);
        this->declare_parameter<int>("sor_mean_k", 30);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 0.8);

        const auto input_topic = this->get_parameter("input_topic").as_string();
        const auto output_topic = this->get_parameter("output_topic").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud filter started");
        RCLCPP_INFO(this->get_logger(), "input: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "output: %s", output_topic.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            return;
        }

        // 去掉 NaN
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // 1. PassThrough 裁剪 X
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(
            this->get_parameter("x_min").as_double(),
            this->get_parameter("x_max").as_double());
        pass_x.filter(*cloud_x);

        // 2. PassThrough 裁剪 Y
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(
            this->get_parameter("y_min").as_double(),
            this->get_parameter("y_max").as_double());
        pass_y.filter(*cloud_y);

        // 3. PassThrough 裁剪 Z
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud_y);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(
            this->get_parameter("z_min").as_double(),
            this->get_parameter("z_max").as_double());
        pass_z.filter(*cloud_z);

        // 4. VoxelGrid 降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        const double leaf = this->get_parameter("voxel_leaf_size").as_double();
        voxel.setInputCloud(cloud_z);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*cloud_voxel);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_voxel;

        // 5. StatisticalOutlierRemoval 去离群点
        if (this->get_parameter("use_sor").as_bool()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_voxel);
            sor.setMeanK(this->get_parameter("sor_mean_k").as_int());
            sor.setStddevMulThresh(this->get_parameter("sor_stddev_mul_thresh").as_double());
            sor.filter(*cloud_sor);
            cloud_filtered = cloud_sor;
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        pub_->publish(output);

        RCLCPP_DEBUG(
            this->get_logger(),
            "cloud in=%zu, out=%zu",
            cloud->size(), cloud_filtered->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
