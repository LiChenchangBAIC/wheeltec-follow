#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
class RealMan
{
private:
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    std::string end_effector_link; 
public: 
    geometry_msgs::msg::Pose current_pose;
    RealMan(moveit::planning_interface::MoveGroupInterface * move_group_,std::string end_effector_link_):move_group(move_group_),end_effector_link(end_effector_link_)
    {
        // 设置位置误差容忍度，单位：米
        move_group->setGoalPositionTolerance(0.005);
        // 设置姿态误差容忍度，单位：弧度
        move_group->setGoalOrientationTolerance(0.005);
        // 设置关节角度误差容忍度，单位：弧度
        move_group->setGoalJointTolerance(0.005);
        // 允许重新规划
        move_group->allowReplanning(true);
        // 设置最大规划时间
        move_group->setPlanningTime(10.0);
        // 设置ompl算法
        move_group->setPlanningPipelineId("ompl");
        move_group->setPlannerId("TRRT");// RRTConnect
        move_group->setPoseReferenceFrame("base_link");
        // 设置最大速度缩放因子
        move_group->setMaxVelocityScalingFactor(0.5);
        // 设置最大加速度缩放因子
        move_group->setMaxAccelerationScalingFactor(0.5);
        // 设置末端执行器
        move_group->setEndEffectorLink(end_effector_link);
    }
    ~RealMan()
    {
        move_group = nullptr;
    }

    geometry_msgs::msg::Pose get_current_pose()//获取当前末端相对base_link位姿
    {
        // 定义roll, pitch, yaw变量
        double roll, pitch, yaw;

        // 定义Quaternion对象q1
        geometry_msgs::msg::Quaternion q1;

        // 获取当前末端执行器的位姿
        current_pose = move_group->getCurrentPose(end_effector_link).pose;

        // 将位姿中的四元数赋值给q1
        q1.x = current_pose.orientation.x;
        q1.y = current_pose.orientation.y;
        q1.z = current_pose.orientation.z;
        q1.w = current_pose.orientation.w;

        // 将四元数q1转换为tf2::Quaternion类型
        tf2::Quaternion quat;
        tf2::fromMsg(q1, quat);

        // 从四元数中计算roll, pitch, yaw值
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // 打印roll, pitch, yaw值（可选）
        // printf("roll:%f pitch:%f  yaw:%f   \n", roll, pitch, yaw);

        // 返回当前位姿
        return current_pose;
    }


    void set_end_link(std::string end_link)
    {
        move_group->setEndEffectorLink(end_link);
    }

    void set_MaxVelocity_Accelerate(double max_velocity_scaling_factor,double max_acceleration_scaling_factor)//设置最大速度
    {
        move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    }

    void move_Zero()//移动到预先设置好的零点位置
    {
        move_group->setNamedTarget("zero"); 
        move_group->move();
    }

    void move_Look()//移动到预先设置好的观测位置
    {
        move_group->setNamedTarget("look"); 
        move_group->move();
    }

    void move_Place()//移动到预先设置好的放置点
    {
        move_group->setNamedTarget("place"); 
        move_group->move();
    }

    void move_Right_side_look_vert()//移动到预先设置好的放置点
    {
        move_group->setNamedTarget("right_side_look_vert"); 
        move_group->move();
    }

    void moveJ(double j1,double j2,double j3,double j4,double j5,double j6)//正解，机械臂六个关节角度值
    {
        std::vector<double> joint={j1,j2,j3,j4,j5,j6};
        move_group->setJointValueTarget(joint);
        move_group->move();
    }

    void moveP(double x,double y,double z)//逆解，相对base_link的xyz位置，以起始点姿态作为目标点姿态
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose = move_group->getCurrentPose(end_effector_link).pose;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        
        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(target_pose);

        
        moveit::core::MoveItErrorCode success = move_group->plan(arm_plan);

        
        if (success) {
            move_group->execute(arm_plan);
            sleep(1);
        }
    
    }

    void moveP(double x,double y,double z,double rx,double ry,double rz)//逆解，x，y，z以及欧拉角
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        rx = rx/57.29578;
        ry = ry/57.29578;
        rz = rz/57.29578;
        
        tf2::Quaternion qx,qy,qz;
        qx.setRotation(tf2::Vector3(1,0,0),rx);
        qy.setRotation(tf2::Vector3(0,1,0),ry);
        qz.setRotation(tf2::Vector3(0,0,1),rz);
        tf2::Quaternion q_final=qz*qy*qx;

        // myQuaternion.setRPY(rx/57.29578, ry/57.29578, rz/57.29578);
        target_pose.orientation.x = q_final.x();
        target_pose.orientation.y = q_final.y();
        target_pose.orientation.z = q_final.z();
        target_pose.orientation.w = q_final.w();
        
        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(target_pose);

        
        moveit::core::MoveItErrorCode success = move_group->plan(arm_plan);
        
        if (success) {
            move_group->execute(arm_plan);
            sleep(1);
        }
    }

    void moveP(double x,double y,double z,double qx,double qy,double qz,double qw)//逆解，x，y，z以及四元数xyzw
    {
        tf2::Quaternion quat;
        geometry_msgs::msg::Pose target_pose;

        quat = tf2::Quaternion( qx,qy,qz,qw); //设置目标姿态
        quat.normalize();
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = quat.getX();
        target_pose.orientation.y = quat.getY();
        target_pose.orientation.z = quat.getZ();
        target_pose.orientation.w = quat.getW();

        
        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(target_pose);

        
        moveit::core::MoveItErrorCode success = move_group->plan(arm_plan);

        
        if (success) {
            move_group->execute(arm_plan);
            sleep(1);
        }
    }

    bool moveP(geometry_msgs::msg::Pose target_pose)
    {
        
       
        move_group->setPoseTarget(target_pose);

        
        moveit::core::MoveItErrorCode success = move_group->plan(arm_plan);

        
        if (success) {
            move_group->execute(arm_plan);
            return true;
        }else{
            return false;
        }
    }

    bool move_C(std::string axis,float radius)
    {
        geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose(end_effector_link).pose;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);
        if(axis=="xy" || axis=="yx")//在x,y平面内生成圆路点位置,姿态保持与起始点一致
        {
            float a=  start_pose.position.x;
            float b = start_pose.position.y;
            for(float th=0.0; th<6.28; th=th+0.01)
            {
                start_pose.position.x = a + (radius - radius * cos(th));
                start_pose.position.y = b + radius * sin(th);
                waypoints.push_back(start_pose);
            }
        }else if (axis=="yz" || axis=="zy")//在y,z平面内生成圆路点位置,姿态保持与起始点一致
        {
            float a=  start_pose.position.y;
            float b = start_pose.position.z;
            for(float th=0.0; th<6.28; th=th+0.01)
            {
                start_pose.position.z = b + (radius - radius * cos(th));
                start_pose.position.y = a + radius * sin(th);
                waypoints.push_back(start_pose);
            }
        }else if (axis=="xz" || axis=="zx")//在x,z平面内生成圆路点位置,姿态保持与起始点一致
        {
            double a=  start_pose.position.x;
            double b = start_pose.position.z;
            for(double th=0.0; th<6.28; th=th+0.01)
            {
                start_pose.position.z = b + radius * sin(th);
                start_pose.position.x = a + (radius - radius * cos(th));
                waypoints.push_back(start_pose);
            }

        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;
        int maxtries = 100;   
        int attempts = 0;     
    
        while (fraction < 1.0 && attempts < maxtries)
        {
            fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
        }
    
        if(fraction == 1)
        {   
            printf("Path computed successfully. Moving the arm.");

            // 运动规划数据
            arm_plan.trajectory_ = trajectory;
            // 执行
            move_group->execute(arm_plan);
            sleep(1);
            return true;
        }
        else
        {
            printf("Move_C Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
            return false;
        }
        
    
    }


    void moveCartesian(std::vector<geometry_msgs::msg::Pose>  &poses)//笛卡尔路径规划
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose(end_effector_link).pose;
        waypoints.push_back(start_pose);
        for(uint8_t i=0;i<poses.size();i++)
        {
            poses[i].orientation=start_pose.orientation;
            waypoints.push_back(poses[i]);
        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;
        int maxtries = 1000;   //最大尝试规划次数
        int attempts = 0;     //已经尝试规划次数

        while(fraction < 1.0 && attempts < maxtries)
        {
            fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
        }
        
        if(fraction == 1)
        {   
            printf("Path computed successfully. Moving the arm.");

            // 运动规划数据
            arm_plan.trajectory_ = trajectory;

            // 执行
            move_group->execute(arm_plan);
            sleep(1);
        }
        else
        {
            printf("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }

    }




};
