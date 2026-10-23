#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/carteposcustom.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class VisualServoTracker : public rclcpp::Node
{
public:
    VisualServoTracker() : Node("visual_servo_tracker")
    {
        // 创建 TF 缓冲区和监听器，用于读取 camera_frame -> target_frame 的实时位姿
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // 机械臂笛卡尔透传控制指令
        cmd_pub_ = this->create_publisher<rm_ros_interfaces::msg::Carteposcustom>(
            "/rm_driver/movep_canfd_custom_cmd", 10);

        // TF 相关参数：相机坐标系、目标坐标系
        this->declare_parameter<std::string>("camera_frame", "camera_arm_color_optical_frame");
        this->declare_parameter<std::string>("target_frame", "aruco_marker_frame");

        // 机械臂初始位姿（web_base 控制坐标系下）
        this->declare_parameter<double>("init_web_x", -0.284);
        this->declare_parameter<double>("init_web_y", -0.001);
        this->declare_parameter<double>("init_web_z", 0.287);

        // 末端姿态固定值：当前代码只做位置跟踪，姿态锁死
        this->declare_parameter<double>("fixed_roll", 3.13);
        this->declare_parameter<double>("fixed_pitch", -0.012);
        this->declare_parameter<double>("fixed_yaw", 1.576);

        // 目标在相机坐标系下的期望位置
        // 即希望 marker 最终停在 camera_frame 下的哪个位置
        this->declare_parameter<double>("desired_cam_x", 0.0);
        this->declare_parameter<double>("desired_cam_y", 0.0);
        this->declare_parameter<double>("desired_cam_z", 0.37);

        // 三个方向的控制增益
        this->declare_parameter<double>("gain_x", 0.02);
        this->declare_parameter<double>("gain_y", 0.02);
        this->declare_parameter<double>("gain_z", 0.015);

        // 三个方向的误差死区，小于这个值就认为误差为 0，避免小抖动来回修正
        this->declare_parameter<double>("deadband_cam_x", 0.001);
        this->declare_parameter<double>("deadband_cam_y", 0.001);
        this->declare_parameter<double>("deadband_cam_z", 0.001);

        // 每次控制循环允许的最大步进，防止一下走太大
        this->declare_parameter<double>("max_step_x", 0.003);
        this->declare_parameter<double>("max_step_y", 0.003);
        this->declare_parameter<double>("max_step_z", 0.002);

        // 误差低通滤波参数
        // 值越大，跟得越快；值越小，越平滑但更慢
        this->declare_parameter<double>("filter_alpha", 0.55);
        this->declare_parameter<double>("filter_alpha_z", 0.50);

        // Z 轴最小有效步进，防止步长太小时 Z 轴不动作
        this->declare_parameter<double>("min_step_z", 0.00010);

        // 目标 TF 超过这个时间没有更新，就认为目标丢失
        this->declare_parameter<double>("max_tf_age_sec", 0.3);

        // 三个方向开关：调试时可以单独关闭某一轴
        this->declare_parameter<bool>("enable_web_x", true);
        this->declare_parameter<bool>("enable_web_y", true);
        this->declare_parameter<bool>("enable_web_z", true);

        // 三个方向的正负号映射
        // 用于修正 camera 坐标误差与机械臂控制方向之间的符号关系
        this->declare_parameter<int>("sign_web_x_from_cam_y", 1);
        this->declare_parameter<int>("sign_web_y_from_cam_x", 1);
        this->declare_parameter<int>("sign_web_z_from_cam_z", 1);

        // 初始化控制命令位置
        current_x_ = this->get_parameter("init_web_x").as_double();
        current_y_ = this->get_parameter("init_web_y").as_double();
        current_z_ = this->get_parameter("init_web_z").as_double();

        // 定时控制循环
        timer_ = this->create_wall_timer(
            12ms, std::bind(&VisualServoTracker::trackingLoop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Visual servo started, camera_frame=%s, target_frame=%s",
            this->get_parameter("camera_frame").as_string().c_str(),
            this->get_parameter("target_frame").as_string().c_str());
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Carteposcustom>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};

    double filtered_err_cam_x_{0.0};
    double filtered_err_cam_y_{0.0};
    double filtered_err_cam_z_{0.0};

    bool target_visible_{false};

    static double applyDeadband(double value, double deadband)
    {
        return std::abs(value) < deadband ? 0.0 : value;
    }

    static double clampStep(double step, double limit)
    {
        return std::clamp(step, -limit, limit);
    }

    static int normalizeSign(int value)
    {
        return value >= 0 ? 1 : -1;
    }

    void resetFilters()
    {
        filtered_err_cam_x_ = 0.0;
        filtered_err_cam_y_ = 0.0;
        filtered_err_cam_z_ = 0.0;
    }

    void publishCurrentCommand(bool follow_enabled)
    {
        rm_ros_interfaces::msg::Carteposcustom msg;
        msg.pose.position.x = current_x_;
        msg.pose.position.y = current_y_;
        msg.pose.position.z = current_z_;

        tf2::Quaternion q;
        q.setRPY(
            this->get_parameter("fixed_roll").as_double(),
            this->get_parameter("fixed_pitch").as_double(),
            this->get_parameter("fixed_yaw").as_double());

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        msg.follow = follow_enabled;
        msg.trajectory_mode = 0;
        msg.radio = 0;

        cmd_pub_->publish(msg);
    }

    void trackingLoop()
    {
        const std::string camera_frame = this->get_parameter("camera_frame").as_string();
        const std::string target_frame = this->get_parameter("target_frame").as_string();

        const double desired_cam_x = this->get_parameter("desired_cam_x").as_double();
        const double desired_cam_y = this->get_parameter("desired_cam_y").as_double();
        const double desired_cam_z = this->get_parameter("desired_cam_z").as_double();

        const double gain_x = this->get_parameter("gain_x").as_double();
        const double gain_y = this->get_parameter("gain_y").as_double();
        const double gain_z = this->get_parameter("gain_z").as_double();

        const double deadband_cam_x = this->get_parameter("deadband_cam_x").as_double();
        const double deadband_cam_y = this->get_parameter("deadband_cam_y").as_double();
        const double deadband_cam_z = this->get_parameter("deadband_cam_z").as_double();

        const double max_step_x = this->get_parameter("max_step_x").as_double();
        const double max_step_y = this->get_parameter("max_step_y").as_double();
        const double max_step_z = this->get_parameter("max_step_z").as_double();

        const double filter_alpha = std::clamp(
            this->get_parameter("filter_alpha").as_double(), 0.0, 1.0);
        const double filter_alpha_z = std::clamp(
            this->get_parameter("filter_alpha_z").as_double(), 0.0, 1.0);
        const double min_step_z = std::max(
            0.0, this->get_parameter("min_step_z").as_double());
        const double max_tf_age_sec = std::max(
            0.0, this->get_parameter("max_tf_age_sec").as_double());

        const bool enable_web_x = this->get_parameter("enable_web_x").as_bool();
        const bool enable_web_y = this->get_parameter("enable_web_y").as_bool();
        const bool enable_web_z = this->get_parameter("enable_web_z").as_bool();

        const int sign_web_x_from_cam_y = normalizeSign(static_cast<int>(
            this->get_parameter("sign_web_x_from_cam_y").as_int()));
        const int sign_web_y_from_cam_x = normalizeSign(static_cast<int>(
            this->get_parameter("sign_web_y_from_cam_x").as_int()));
        const int sign_web_z_from_cam_z = normalizeSign(static_cast<int>(
            this->get_parameter("sign_web_z_from_cam_z").as_int()));

        geometry_msgs::msg::TransformStamped transform;
        bool tf_ok = true;
        double tf_age_sec = 0.0;

        try {
            transform = tf_buffer_->lookupTransform(
                camera_frame, target_frame, tf2::TimePointZero);

            const rclcpp::Time tf_stamp(
                transform.header.stamp, this->get_clock()->get_clock_type());
            tf_age_sec = (this->now() - tf_stamp).seconds();

            if (tf_age_sec > max_tf_age_sec) {
                tf_ok = false;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "TF stale: %s -> %s, age=%.3f s",
                    camera_frame.c_str(), target_frame.c_str(), tf_age_sec);
            }
        } catch (const tf2::TransformException & ex) {
            tf_ok = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "TF lookup failed: %s -> %s, %s",
                camera_frame.c_str(), target_frame.c_str(), ex.what());
        }

        if (!tf_ok) {
            if (target_visible_) {
                RCLCPP_WARN(this->get_logger(), "Target lost. Holding current pose.");
                target_visible_ = false;
            }

            resetFilters();
            publishCurrentCommand(true);
            return;
        }

        if (!target_visible_) {
            target_visible_ = true;
            resetFilters();
            RCLCPP_INFO(this->get_logger(), "Target acquired. Servo resumed.");
        }

        const double actual_cam_x = transform.transform.translation.x;
        const double actual_cam_y = transform.transform.translation.y;
        const double actual_cam_z = transform.transform.translation.z;

        double err_cam_x = actual_cam_x - desired_cam_x;
        double err_cam_y = actual_cam_y - desired_cam_y;
        double err_cam_z = actual_cam_z - desired_cam_z;

        err_cam_x = applyDeadband(err_cam_x, deadband_cam_x);
        err_cam_y = applyDeadband(err_cam_y, deadband_cam_y);
        err_cam_z = applyDeadband(err_cam_z, deadband_cam_z);

        filtered_err_cam_x_ =
            (1.0 - filter_alpha) * filtered_err_cam_x_ + filter_alpha * err_cam_x;
        filtered_err_cam_y_ =
            (1.0 - filter_alpha) * filtered_err_cam_y_ + filter_alpha * err_cam_y;

        if (err_cam_z * filtered_err_cam_z_ < 0.0) {
            filtered_err_cam_z_ = 0.0;
        }
        filtered_err_cam_z_ =
            (1.0 - filter_alpha_z) * filtered_err_cam_z_ + filter_alpha_z * err_cam_z;

        double step_x = 0.0;
        double step_y = 0.0;
        double step_z = 0.0;

        // cam_y -> web_x
        if (enable_web_x) {
            step_x = gain_x * sign_web_x_from_cam_y * filtered_err_cam_y_;
            step_x = clampStep(step_x, max_step_x);
            current_x_ += step_x;
        }

        // cam_x -> web_y
        if (enable_web_y) {
            step_y = gain_y * sign_web_y_from_cam_x * filtered_err_cam_x_;
            step_y = clampStep(step_y, max_step_y);
            current_y_ += step_y;
        }

        // cam_z -> web_z
        if (enable_web_z) {
            step_z = -gain_z * sign_web_z_from_cam_z * filtered_err_cam_z_;
            step_z = clampStep(step_z, max_step_z);

            if (std::abs(filtered_err_cam_z_) > deadband_cam_z &&
                std::abs(step_z) < min_step_z) {
                const double direction =
                    -sign_web_z_from_cam_z * filtered_err_cam_z_;
                step_z = std::copysign(min_step_z, direction);
            }

            current_z_ += step_z;
        }

        publishCurrentCommand(true);

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(), *this->get_clock(), 200,
        //     "tf_age=%.3f cam_rel=(%.4f, %.4f, %.4f) err=(%.4f, %.4f, %.4f) "
        //     "filt=(%.4f, %.4f, %.4f) step=(%.5f, %.5f, %.5f) "
        //     "cmd=(%.4f, %.4f, %.4f) enable=(x:%s y:%s z:%s) sign=(x:%d y:%d z:%d)",
        //     tf_age_sec,
        //     actual_cam_x, actual_cam_y, actual_cam_z,
        //     err_cam_x, err_cam_y, err_cam_z,
        //     filtered_err_cam_x_, filtered_err_cam_y_, filtered_err_cam_z_,
        //     step_x, step_y, step_z,
        //     current_x_, current_y_, current_z_,
        //     enable_web_x ? "on" : "off",
        //     enable_web_y ? "on" : "off",
        //     enable_web_z ? "on" : "off",
        //     sign_web_x_from_cam_y, sign_web_y_from_cam_x, sign_web_z_from_cam_z);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServoTracker>());
    rclcpp::shutdown();
    return 0;
}
