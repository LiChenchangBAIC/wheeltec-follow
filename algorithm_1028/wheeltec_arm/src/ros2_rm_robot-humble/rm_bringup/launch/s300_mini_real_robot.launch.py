import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    start_moveit = LaunchConfiguration('start_moveit')

    start_moveit_arg = DeclareLaunchArgument(
        'start_moveit',
        default_value='true',
        description='Whether to start MoveIt'
    )

    rm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_driver'),
                'launch',
                'rm_eco65_driver.launch.py'
            )
        )
    )

    rm_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_description'),
                'launch',
                's300_mini_display.launch.py'
            )
        ),
    )

    rm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_control'),
                'launch',
                'rm_eco65_control.launch.py'
            )
        )
    )

    rm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('s300_mini_moveit2_config'),
                'launch',
                'real_robot_moveit.launch.py'
            )
        ),
        condition=IfCondition(start_moveit)
    )

    static_tf_hand_eye = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="hand_eye",
        output="log",
        arguments=[
            "-0.017375", "-0.119170", "0.010673" ,"-0.506872" ,"0.524057" ,"-0.474803" ,"-0.492957",
            "Link6", "camera_arm_link"
        ],
    ) 

    static_tf_base_link_web = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_web",
        output="log",
        arguments=[
            "0", "0", "0",
            "3.1415926", "0", "0",
            "base_link", "web_base_link"
        ],
    )

    return LaunchDescription([
        start_moveit_arg,
        rm_driver,
        rm_description,
        rm_control,
        rm_moveit,
        static_tf_hand_eye,
        static_tf_base_link_web
    ])
