
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor(): Node("image_processor")
  {
        // 创建参数描述符
        auto create_descriptor = [](int min_val, int max_val) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Range: " + std::to_string(min_val) + " - " + std::to_string(max_val);
          descriptor.integer_range.resize(1);
          descriptor.integer_range[0].from_value = min_val;
          descriptor.integer_range[0].to_value = max_val;
          descriptor.integer_range[0].step = 1;
          return descriptor;
        };
    
        // 声明带范围的参数
        this->declare_parameter("color_h_min", 0, create_descriptor(0, 255));
        this->declare_parameter("color_h_max", 255, create_descriptor(0, 255));
        this->declare_parameter("color_s_min", 0, create_descriptor(0, 255));
        this->declare_parameter("color_s_max", 255, create_descriptor(0, 255));
        this->declare_parameter("color_v_min", 0, create_descriptor(0, 255));
        this->declare_parameter("color_v_max", 255, create_descriptor(0, 255));
    
        // 获取初始参数值
        hue_min_ = this->get_parameter("color_h_min").as_int();
        hue_max_ = this->get_parameter("color_h_max").as_int();
        saturation_min_ = this->get_parameter("color_s_min").as_int();
        saturation_max_ = this->get_parameter("color_s_max").as_int();
        value_min_ = this->get_parameter("color_v_min").as_int();
        value_max_ = this->get_parameter("color_v_max").as_int();
        
        // 设置参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&ImageProcessor::parametersCallback, this, std::placeholders::_1));
        
        // 创建图像发布器（用于可视化）
        threshold_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/hsv_threshold", 10);
        
        // 订阅RGB话题
        rgb_sub_.subscribe(this, "/camera_arm/color/image_raw");

        // 订阅深度话题
        depth_sub_.subscribe(this, "/camera_arm/depth/image_raw");

        // 创建同步策略
        approxSync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), rgb_sub_, depth_sub_);
        approxSync->registerCallback(std::bind(&ImageProcessor::imageCallback, this, 
                                              std::placeholders::_1, std::placeholders::_2));
        
        camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
          "/gemini_info", 1, 
          std::bind(&ImageProcessor::rgb_info_callback, this, std::placeholders::_1));
        
        // 创建TF广播器
        tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "Image processor node initialized with HSV thresholds: H[%d,%d] S[%d,%d] V[%d,%d]",
                    hue_min_, hue_max_, saturation_min_, saturation_max_, value_min_, value_max_);
  }

private:
  // 参数回调函数
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters)
    {
      if (param.get_name() == "color_h_min")
        hue_min_ = param.as_int();
      else if (param.get_name() == "color_h_max")
        hue_max_ = param.as_int();
      else if (param.get_name() == "color_s_min")
        saturation_min_ = param.as_int();
      else if (param.get_name() == "color_s_max")
        saturation_max_ = param.as_int();
      else if (param.get_name() == "color_v_min")
        value_min_ = param.as_int();
      else if (param.get_name() == "color_v_max")
        value_max_ = param.as_int();
    }
    
    RCLCPP_INFO(this->get_logger(), "Updated HSV thresholds: H[%d,%d] S[%d,%d] V[%d,%d]",
                hue_min_, hue_max_, saturation_min_, saturation_max_, value_min_, value_max_);
    
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
                    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    if (!camera_info)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Camera info not received yet, skipping frame");
      return;
    }
    
    // 转换ROS图像消息为OpenCV格式
    cv_bridge::CvImagePtr cv_rgb_ptr;
    try
    {
      cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 将图像从BGR颜色空间转换到HSV颜色空间
    cv::Mat hsv_image_raw;
    cv::cvtColor(cv_rgb_ptr->image, hsv_image_raw, cv::COLOR_BGR2HSV);
    
    // 形态学操作的内核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    
    // 设置颜色阈值
    cv::Scalar lower(hue_min_, saturation_min_, value_min_);
    cv::Scalar upper(hue_max_, saturation_max_, value_max_);
    
    // 对HSV图像应用颜色阈值
    cv::Mat threshold_image;
    cv::inRange(hsv_image_raw, lower, upper, threshold_image);
    
    // 形态学操作
    cv::Mat hsv_image_erode;
    cv::Mat hsv_image_dilate;
    cv::erode(threshold_image, hsv_image_erode, kernel);
    cv::dilate(hsv_image_erode, hsv_image_dilate, kernel);
    
    // 发布处理后的图像用于可视化
    cv_bridge::CvImage threshold_img_bridge;
    threshold_img_bridge.header = rgb_msg->header;
    threshold_img_bridge.encoding = sensor_msgs::image_encodings::MONO8;
    threshold_img_bridge.image = hsv_image_dilate;
    threshold_pub_->publish(*threshold_img_bridge.toImageMsg());
    
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(hsv_image_dilate, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 寻找面积最大的物体
    if (!contours.empty())
    { 
      std::vector<cv::Point> contour=contours[0];
      cv::Moments moments=cv::moments(contour);

      cv::RotatedRect rect = cv::minAreaRect(contour);

      double yaw = rect.angle * CV_PI / 180.0;          // 单位: rad
      
      tf2::Quaternion tf_quat;
      tf_quat.setRPY(0, 0, yaw);
      geometry_msgs::msg::Quaternion geom_quat;
      tf2::convert(tf_quat, geom_quat);
      geometry_msgs::msg::TransformStamped obj_msg;
      obj_msg.transform.rotation = geom_quat;
      std::sort(contours.begin(), contours.end(), 
                [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                  return cv::contourArea(c1) > cv::contourArea(c2);
                });
      
      // 确保moments.m00不为0，避免除以0错误
      if (moments.m00 > 0)
      {
        cv::Point newpos(moments.m10 / moments.m00, moments.m01 / moments.m00);
        
        // 确保点在图像范围内
        if (newpos.x >= 0 && newpos.x < cv_depth_ptr->image.cols && 
            newpos.y >= 0 && newpos.y < cv_depth_ptr->image.rows)
        {
          // 获取深度值（毫米转米）
          dis = (cv_depth_ptr->image.at<ushort>(newpos.y, newpos.x)) / 1000.0;
          
          // 通过相机内参将像素坐标转换成物体相对相机的三维坐标
          double x = (newpos.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0) * dis;
          double y = (newpos.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1) * dis;
          
          RCLCPP_DEBUG(this->get_logger(), "Object position: x=%.3f, y=%.3f, z=%.3f", x, y, dis);
          
          // 发布TF变换
          if (dis > 0)
          {
            // std::cout << "yaw: " << yaw * 180.0 / CV_PI << std::endl;
            obj_msg.transform.translation.x = x;
            obj_msg.transform.translation.y = y;
            obj_msg.transform.translation.z = dis;
            obj_msg.header.stamp = this->now();
            obj_msg.header.frame_id = "camera_arm_color_optical_frame";  // 设置参考坐标
            obj_msg.child_frame_id = "color_link";  // 物体的tf坐标
            tf_pub->sendTransform(obj_msg);
          }
          else
          {
            // RCLCPP_ERROR(this->get_logger(), "Failed detecting object");
          }
        }
      }
    }
  }

  void rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    bool K_valid = false;
    bool D_valid = false;
    
    if (!camera_info)
    {
      for (uint8_t i = 0; i < msg->k.size(); i++)
      {
        if (msg->k.at(i) != 0)
        {
          K_valid = true;
          break;
        }
      }
      
      for (uint8_t i = 0; i < msg->d.size(); i++)
      {
        if (msg->d.at(i) != 0)
        {
          D_valid = true;
          break;
        }
      }
      
      if (K_valid && D_valid)
      {
        camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        camera_dis = cv::Mat::zeros(1, 5, CV_64F);
        
        for (uint8_t i = 0; i < 3; i++)
        {
          for (uint8_t j = 0; j < 3; j++)
          {
            camera_matrix.at<double>(i, j) = msg->k[i * 3 + j];
          }
        }
        
        for (uint8_t i = 0; i < 5; i++)
        {
          camera_dis.at<double>(0, i) = msg->d[i];
        }
        
        camera_info = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration information received");
      }
    }
  }

  // 消息同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> syncPolicy;
  message_filters::Synchronizer<syncPolicy> *approxSync;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  
  // 相机参数
  cv::Mat camera_matrix;
  cv::Mat camera_dis;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  
  // HSV阈值参数
  int hue_min_ = 0;
  int hue_max_ = 255;
  int saturation_min_ = 0;
  int saturation_max_ = 255;
  int value_min_ = 0;
  int value_max_ = 255;
  
  // 参数回调句柄
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr threshold_pub_;
  
  // 其他成员变量
  bool camera_info = false;
  volatile double dis = 0;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
