#!/usr/bin/env python3
# ros2 launch wheeltec_nav2 save_map.launch.py slam_toolbox:=True
# ros2 launch wheeltec_nav2 save_map.launch.py cartographer:=True
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    slam_toolbox_arg = DeclareLaunchArgument(
        'slam_toolbox',
        default_value='False',
        description='When set to True, the slam_toolbox serialize_map service is additionally called to save the map.'
    )

    cartographer_arg = DeclareLaunchArgument(
        'cartographer',
        default_value='False',
        description='When set to True, an additional call is made to Cartographer write_state service to save the map.'
    )

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', '/home/wheeltec/wheeltec_ros2/install/wheeltec_nav2/share/wheeltec_nav2/map/WHEELTEC'],

        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    map_backup = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_backup',
        output='screen',
        arguments=['-f', '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC'],
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]
        )

    slam_toolbox_save = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/slam_toolbox/serialize_map',
            'slam_toolbox/srv/SerializePoseGraph',
            '{filename: /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/slam_toolbox_map}'
        ],
        output='screen',
        name='slam_toolbox_save',
        condition=IfCondition(LaunchConfiguration('slam_toolbox'))
    )

    cartographer_save = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/write_state',
            'cartographer_ros_msgs/srv/WriteState',
            '{filename: /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/cartographer_map.pbstream}'
        ],
        output='screen',
        name='cartographer_save',
        condition=IfCondition(LaunchConfiguration('cartographer'))
    )

    ld = LaunchDescription()

    ld.add_action(slam_toolbox_arg)
    ld.add_action(cartographer_arg)

    ld.add_action(map_saver)
    ld.add_action(map_backup)
    ld.add_action(slam_toolbox_save)
    ld.add_action(cartographer_save)
    return ld