#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class CameraInfoPub : public rclcpp::Node
{
public:
    CameraInfoPub() : Node("camera_info_pub")
    {
        try {
            std::string package_share_dir =
                ament_index_cpp::get_package_share_directory("grab_demo");
            std::string yaml_path = package_share_dir + "/config/ost.yaml";

            RCLCPP_INFO(this->get_logger(), "Loading calibration from: %s", yaml_path.c_str());

            YAML::Node config = YAML::LoadFile(yaml_path);

            camera_info_msg_.width = config["image_width"].as<uint32_t>();
            camera_info_msg_.height = config["image_height"].as<uint32_t>();
            camera_info_msg_.distortion_model = config["distortion_model"].as<std::string>();

            auto camera_matrix_node = config["camera_matrix"]["data"];
            for (int i = 0; i < 9; ++i) {
                camera_info_msg_.k[i] = camera_matrix_node[i].as<double>();
            }

            auto distortion_node = config["distortion_coefficients"]["data"];
            camera_info_msg_.d.clear();
            for (int i = 0; i < 5; ++i) {
                camera_info_msg_.d.push_back(distortion_node[i].as<double>());
            }

            auto rectification_node = config["rectification_matrix"]["data"];
            for (int i = 0; i < 9; ++i) {
                camera_info_msg_.r[i] = rectification_node[i].as<double>();
            }

            auto projection_node = config["projection_matrix"]["data"];
            for (int i = 0; i < 12; ++i) {
                camera_info_msg_.p[i] = projection_node[i].as<double>();
            }

            info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("gemini_info", 10);

            timer_ = create_wall_timer(50ms, [this]() {
                camera_info_msg_.header.stamp = this->now();
                camera_info_msg_.header.frame_id = "camera_arm_color_optical_frame";
                info_pub_->publish(camera_info_msg_);
            });
        }
        catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

private:
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraInfoPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
