#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

class CameraUndistortNode : public rclcpp::Node
{
public:
    CameraUndistortNode() : Node("camera_undistort_node")
    {
        // 声明参数
        this->declare_parameter("input_image_topic", "/camera_arm/color/image_raw");
        this->declare_parameter("input_camera_info_topic", "/camera_arm/color/camera_info");
        this->declare_parameter("output_image_topic", "/camera_arm/color/image_undistorted");        
        // 获取参数
        std::string input_image_topic = this->get_parameter("input_image_topic").as_string();
        std::string input_camera_info_topic = this->get_parameter("input_camera_info_topic").as_string();
        std::string output_image_topic = this->get_parameter("output_image_topic").as_string();
        
        // 创建订阅者
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_image_topic, 10,
            std::bind(&CameraUndistortNode::image_callback, this, std::placeholders::_1));
            
        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            input_camera_info_topic, 10,
            std::bind(&CameraUndistortNode::camera_info_callback, this, std::placeholders::_1));
        
        // 创建发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic, 10);
        
        // 初始化标志
        camera_info_received_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Camera undistort node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", input_image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera info topic: %s", input_camera_info_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing undistorted image to: %s", output_image_topic.c_str());
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            // 提取相机内参矩阵
            camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
            camera_matrix_.at<double>(0, 0) = msg->k[0]; // fx
            camera_matrix_.at<double>(0, 2) = msg->k[2]; // cx
            camera_matrix_.at<double>(1, 1) = msg->k[4]; // fy
            camera_matrix_.at<double>(1, 2) = msg->k[5]; // cy
            camera_matrix_.at<double>(2, 2) = 1.0;
            
            // 提取畸变系数
            distortion_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
            for (size_t i = 0; i < std::min(msg->d.size(), static_cast<size_t>(5)); ++i) {
                distortion_coeffs_.at<double>(0, i) = msg->d[i];
            }
            
            // 保存原始相机信息用于发布矫正后的信息
            original_camera_info_ = *msg;
            
            // 图像尺寸
            image_size_ = cv::Size(msg->width, msg->height);
            
            camera_info_received_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Camera info received and processed");
            RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image_size_.width, image_size_.height);
        }
    }
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Camera info not received yet, skipping image");
            return;
        }
        
        try {
            // 将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

            // 执行去畸变
            cv::Mat undistorted_image;
            cv::undistort(cv_ptr->image, undistorted_image, camera_matrix_, distortion_coeffs_);
            

            
            // 将去畸变后的图像转换回ROS消息
            cv_bridge::CvImage undistorted_cv_image;
            undistorted_cv_image.header = msg->header;
            undistorted_cv_image.encoding = msg->encoding;
            undistorted_cv_image.image = undistorted_image;
            
            // 发布去畸变后的图像
            auto undistorted_msg = undistorted_cv_image.toImageMsg();
            image_publisher_->publish(*undistorted_msg);
 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Size image_size_;
    sensor_msgs::msg::CameraInfo original_camera_info_;
    bool camera_info_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraUndistortNode>());
    rclcpp::shutdown();
    return 0;
}
