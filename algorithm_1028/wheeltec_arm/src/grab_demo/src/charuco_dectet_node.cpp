#include "grab_demo/charuco_dectet_node.hpp"

 
int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto charuco_detector=std::make_shared<charuco_dectet>();
    charuco_detector->start_devtet();
    rclcpp::spin(charuco_detector);
    rclcpp::shutdown();
    return 0;
}
 
//根据位姿绘制坐标轴
void charuco_dectet::draw_axis(cv::InputOutputArray &image,cv::InputArray camera_K,cv::InputArray camera_D,cv::InputArray rotation,cv::InputArray translation,float length)
{
    std::vector<cv::Point3f> axis_point;
    axis_point.push_back(cv::Point3f(0,0,0));
    axis_point.push_back(cv::Point3f(length,0,0));
    axis_point.push_back(cv::Point3f(0,length,0));
    axis_point.push_back(cv::Point3f(0,0,length));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_point,rotation,translation,camera_K,camera_D,image_points);
    cv::line(image,image_points[0],image_points[1],cv::Scalar(255,0,0),3);
    cv::line(image,image_points[0],image_points[2],cv::Scalar(0,255,0),3);
    cv::line(image,image_points[0],image_points[3],cv::Scalar(0,0,255),3);
}

cv::Mat charuco_dectet::vec2matrix(cv::Vec3d &camera_rotation,cv::Vec3d &camera_translation) // 相机旋转向量  
{
    cv::Mat temp_matrix;
    cv::Rodrigues(camera_rotation,temp_matrix);
    cv::Mat transform_matrix=cv::Mat::eye(4,4,CV_64F);
    for(uint8_t i=0;i<3;i++)
    {
        for(uint8_t j=0;j<3;j++)
        {
            transform_matrix.at<double>(i,j)=temp_matrix.at<double>(i,j);
        }
    }

    for(uint8_t i=0;i<3;i++)
    {
        transform_matrix.at<double>(i,3)=camera_translation[i];
    }
    return transform_matrix;
    

}


void charuco_dectet::rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    std::vector<int> marker_ids,charuco_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners,rejected_markers;
    std::vector<cv::Point2f> charuco_corners;
    cv::Mat image_rgb;
    cv::Scalar color(255,0,0);
    if(camera_info && camera_matrix.total() != 0)
    {
        image_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        
        cv::aruco::CharucoParameters charucoParams;
        cv::aruco::DetectorParameters detectorParams;
        detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        // cv::aruco::RefineParameters refineParams;

        cv::aruco::CharucoDetector dynamic_detector(*(this->charuco_board), charucoParams, detectorParams);

        dynamic_detector.detectBoard(image_rgb, charuco_corners, charuco_ids, marker_corners, marker_ids);
        
        bool valid_pose = false;
        if (camera_matrix.total() != 0 && charuco_ids.size() >= 6)
        {
            std::vector<cv::Point3f> objPoints; 
            std::vector<cv::Point2f> imgPoints; 

            this->charuco_board->matchImagePoints(charuco_corners, charuco_ids, objPoints, imgPoints);

            if (imgPoints.size() >= 6) {
                try {
                   valid_pose = cv::solvePnP(objPoints, imgPoints, camera_matrix, camera_dis, 
                          camera_rotation, camera_translation, false, cv::SOLVEPNP_ITERATIVE);
                } catch (const cv::Exception& e) {
                    RCLCPP_WARN(this->get_logger(), "计算姿态失败，跳过这一帧: %s", e.what());
                    valid_pose = false; 
                }
            }
        } 
       
        
        if(valid_pose)
        {
            // 1. 将 OpenCV 的旋转向量 (rvec) 直接转换为旋转矩阵
            cv::Mat rotation_matrix;
            cv::Rodrigues(camera_rotation, rotation_matrix);

            // 2. 将旋转矩阵转换为 Eigen 矩阵，方便求四元数
            Eigen::Matrix3d eigen_rotation_matrix;
            cv::cv2eigen(rotation_matrix, eigen_rotation_matrix);
            Eigen::Quaterniond q(eigen_rotation_matrix);

            geometry_msgs::msg::TransformStamped t;
            
            t.header.stamp = msg->header.stamp; 
            
            // 如果你的图像 msg 里面没有正确的 frame_id，你可以手动写死，比如 "camera_color_optical_frame"
            t.header.frame_id = msg->header.frame_id; 
            
            // 子坐标系：标定板
            t.child_frame_id = "charuco_marker";

            // 填入平移 (OpenCV 算出来的 tvec 就是平移)
            t.transform.translation.x = camera_translation[0];
            t.transform.translation.y = camera_translation[1];
            t.transform.translation.z = camera_translation[2];

            // 填入四元数
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            // 广播 TF
            tf_broadcaster_->sendTransform(t);

            // 
            charuco_pose_.position.x = t.transform.translation.x;
            charuco_pose_.position.y = t.transform.translation.y;
            charuco_pose_.position.z = t.transform.translation.z;

            charuco_pose_.orientation.x=q.x();
            charuco_pose_.orientation.y=q.y();
            charuco_pose_.orientation.z=q.z();
            charuco_pose_.orientation.w=q.w();
            charuco_pose_pub->publish(charuco_pose_);             
            // 绘制坐标轴
            draw_axis(image_rgb, camera_matrix, camera_dis, camera_rotation, camera_translation, 0.1);
        }
        for(size_t i=0;i<charuco_ids.size();i++)
        {
            cv::circle(image_rgb,charuco_corners.at(i),3,cv::Scalar(255,0,255),-1);
        }
        std_msgs::msg::Header header;
        header.stamp=this->get_clock()->now();
        result_img=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,image_rgb).toImageMsg();
        image_result_pub.publish(result_img);
    }
}

void charuco_dectet::rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
{
    bool K_valid=0;
    bool D_valid=0;
    if(!camera_info)
    {

            for(uint8_t i=0;i<msg->k.size();i++)
        {
            if(msg->k.at(i)!=0)
            {

                K_valid=1;
                break;
            }
        }
        for(uint8_t i=0;i<msg->d.size();i++)
        {
            if(msg->d.at(i)!=0)
            {

                D_valid=1;
                break;
            }
        }
        if(K_valid && D_valid)
        {
            camera_matrix=cv::Mat::zeros(3,3,CV_64F);
            camera_dis=cv::Mat::zeros(1,5,CV_64F);
            for(uint8_t i=0;i<3;i++)
            {
                for(uint8_t j=0;j<3;j++)
                {
                    camera_matrix.at<double>(i,j)=msg->k[i*3+j];
                }
            }
            for(uint8_t i=0;i<5;i++)
            {
                camera_dis.at<double>(0,i)=msg->d[i];
            }
            camera_info=1;
        }
    }
    

}
void charuco_dectet::start_devtet()
{
    image_transport_ptr_sub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_sub=image_transport_ptr_sub->subscribe(rgb_image_topic,10,std::bind(&charuco_dectet::rgb_image_callback,this,std::placeholders::_1));
    image_transport_result_ptr_pub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_result_pub=image_transport_result_ptr_pub->advertise(rgb_result_pub_topic,1);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
