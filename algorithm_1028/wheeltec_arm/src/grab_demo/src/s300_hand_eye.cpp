#include "arm_demo/moveit_base.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
// #include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
class HandEyeCalibrator : public rclcpp::Node
{
public:
    HandEyeCalibrator(const rclcpp::NodeOptions & options) 
        : Node("hand_eye_calibrator", options)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // 阻塞式获取 TF 树中的位姿，并转换为 OpenCV 需要的 Mat
    bool wait_for_charuco_pose(cv::Mat &R_cam, cv::Mat &t_cam, geometry_msgs::msg::TransformStamped &t_out)
    {
        try {
            t_out = tf_buffer_->lookupTransform("camera_arm_link", "charuco_marker", tf2::TimePointZero, tf2::durationFromSec(1.5));
            
            Eigen::Quaterniond q(t_out.transform.rotation.w, t_out.transform.rotation.x, t_out.transform.rotation.y, t_out.transform.rotation.z);
            cv::eigen2cv(q.toRotationMatrix(), R_cam);
            t_cam = (cv::Mat_<double>(3, 1) << t_out.transform.translation.x, t_out.transform.translation.y, t_out.transform.translation.z);
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "未能在 TF 树中找到变换: %s", ex.what());
            return false;
        }
    }
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto calibrator_node = std::make_shared<HandEyeCalibrator>(node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(calibrator_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "rm_group";
    static const std::string PLANNING_EEF = "Link6";

    moveit::planning_interface::MoveGroupInterface move_group(calibrator_node, PLANNING_GROUP);
    RealMan rm(&move_group, PLANNING_EEF);

    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;
 
    // 16 点圆锥面采样轨迹 (绕 Z 轴环绕，高度波浪变化，附带 Yaw 旋转)
    const int hand_eye_cout = 16;
    float data_cout[hand_eye_cout][6] = {
        {0.2900,  0.0000,  0.2800,  179.982, -0.339,  -100.418},
        {0.3800,  0.0000,  0.2800,  163.538, -3.304,  -99.892 },
        {0.3677,  0.0346,  0.2990,  169.668, -11.119, -111.576},
        {0.3391,  0.0546,  0.2918,  177.932, -13.397, -122.451},
        {0.3094,  0.0598,  0.2682, -175.390, -11.784, -129.423},
        {0.2837,  0.0600,  0.2610, -171.023, -8.646,  -130.946},
        {0.2562,  0.0585,  0.2800, -168.212, -5.348,  -126.977},
        {0.2256,  0.0468,  0.2990, -166.025, -2.238,  -118.364},
        {0.2032,  0.0184,  0.2918, -164.150,  0.947,  -106.571},
        {0.2032, -0.0184,  0.2682, -163.597,  4.380,  -93.595 },
        {0.2256, -0.0468,  0.2610, -166.180,  7.236,  -81.937 },
        {0.2562, -0.0585,  0.2800, -171.045,  8.547,  -73.777 },
        {0.2837, -0.0600,  0.2990, -175.548,  9.377,  -70.213 },
        {0.3094, -0.0598,  0.2918,  179.845,  11.111, -71.884 },
        {0.3391, -0.0546,  0.2682,  172.577,  12.025, -78.872 },
        {0.3677, -0.0346,  0.2610,  164.678,  6.923,  -89.099 },
    };

    

    rm.move_Look();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    rm.move_Look();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    for (int i = 0; i < hand_eye_cout; i++)
    {
        RCLCPP_INFO(calibrator_node->get_logger(), "\033[1;32m--- 正在前往采集点 %d/%d ---\033[0m", i + 1, hand_eye_cout);

        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(data_cout[i][3] * M_PI / 180.0, 
                            data_cout[i][4] * M_PI / 180.0, 
                            data_cout[i][5] * M_PI / 180.0);
        
        target_pose.orientation.x = myQuaternion.getX();
        target_pose.orientation.y = myQuaternion.getY();
        target_pose.orientation.z = myQuaternion.getZ();
        target_pose.orientation.w = myQuaternion.getW();
        target_pose.position.x = data_cout[i][0];
        target_pose.position.y = data_cout[i][1];
        target_pose.position.z = data_cout[i][2];

        rm.moveP(target_pose);
        std::this_thread::sleep_for(std::chrono::seconds(3)); // 必须等待机械臂停稳

        cv::Mat R_cam, t_cam;
        geometry_msgs::msg::TransformStamped tf_cam; // 用于接收原始 tf 数据

        if (calibrator_node->wait_for_charuco_pose(R_cam, t_cam, tf_cam))
        {
            geometry_msgs::msg::Pose temp_p = rm.get_current_pose();
            Eigen::Quaterniond q_robot(temp_p.orientation.w, temp_p.orientation.x, temp_p.orientation.y, temp_p.orientation.z);
            
            cv::Mat R_robot;
            cv::eigen2cv(q_robot.toRotationMatrix(), R_robot);
            cv::Mat t_robot = (cv::Mat_<double>(3, 1) << temp_p.position.x, temp_p.position.y, temp_p.position.z);

            R_gripper2base.push_back(R_robot);
            t_gripper2base.push_back(t_robot);
            R_target2cam.push_back(R_cam);
            t_target2cam.push_back(t_cam);

            RCLCPP_INFO(calibrator_node->get_logger(), "点 %d 记录成功！", i + 1);
        }
        else
        {
            RCLCPP_WARN(calibrator_node->get_logger(), "\033[1;31m点 %d 未获取到有效 TF，已安全跳过。\033[0m", i + 1);
        }
    }


    if (R_gripper2base.size() < 4)
    {
        RCLCPP_ERROR(calibrator_node->get_logger(), "有效点少于4个，无法求解 Tsai 矩阵！");
        rclcpp::shutdown();
        return -1;
    }

    cv::Mat R_cam2gripper = cv::Mat(3, 3, CV_64FC1);  
    cv::Mat t_cam2gripper = cv::Mat(3, 1, CV_64FC1);
    
    cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                         R_target2cam, t_target2cam,
                         R_cam2gripper, t_cam2gripper,
                         cv::CALIB_HAND_EYE_DANIILIDIS);

    tf2::Matrix3x3 rotation_matrix(
        R_cam2gripper.at<double>(0,0), R_cam2gripper.at<double>(0,1), R_cam2gripper.at<double>(0,2),
        R_cam2gripper.at<double>(1,0), R_cam2gripper.at<double>(1,1), R_cam2gripper.at<double>(1,2),
        R_cam2gripper.at<double>(2,0), R_cam2gripper.at<double>(2,1), R_cam2gripper.at<double>(2,2));

    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    rm.move_Look();

    RCLCPP_INFO(calibrator_node->get_logger(), 
        "\n\033[1;33m====== 标定结果 (Link6 -> camera_link) ======\n"
        "XYZ XYZW : \"%f\", \"%f\", \"%f\" ,\"%f\" ,\"%f\" ,\"%f\" ,\"%f\"   \n"
        " please copy to rm_bringup/s300_pro/mini_real_robot.launch.py。\033[0m", 
        t_cam2gripper.at<double>(0,0), t_cam2gripper.at<double>(1,0), t_cam2gripper.at<double>(2,0),
        quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

    rclcpp::shutdown();
    return 0;
}