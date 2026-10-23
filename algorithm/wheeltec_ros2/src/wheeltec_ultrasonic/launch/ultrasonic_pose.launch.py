import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_rear_ultrasonic = LaunchConfiguration("use_rear_ultrasonic")
    declare_use_rear_ultrasonic = DeclareLaunchArgument(
        "use_rear_ultrasonic",default_value="false",
        description="Whether to start rear ultrasonic point node")

    ultrasonic_pose = launch_ros.actions.Node(
            package='wheeltec_ultrasonic', 
            executable='ultrasonic_enum_node', 
            name='ultrasonic_pose',
            #output='screen',
            parameters=[
            {
            "sensor_x": [-0.205, -0.065, 0.065, 0.205],
            "sensor_y": 0.0,
            "uldis_max_range": 1.8,
            "single_sensor_penalty": 0.1,
            "cluster_count_penalty": 0.05,
            "merge_threshold": 0.05,
            "outlier_residual": 0.15,
            }]

    )
    ultrasonic_point = launch_ros.actions.Node(
            package='wheeltec_ultrasonic', 
            executable='ultrasonic_points_node', 
            name='ultrasonic_point',
            #output='screen',
            parameters=[{
                    'input_topic': '/ultrasonic/obstacles_pos',
                    'output_topic': '/ultrasonic/points',
                    'frame_id': 'obstacles_link',
                    'max_range': 1.5,
                    'min_points': 2,
                    'max_points': 10,
                    'spread_radius': 0.01,
                    "use_rear_ultrasonic": use_rear_ultrasonic,
                }]
    )
    
    base_to_obstacles = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_obstacles',
            arguments=['0.10', '0.00', '0.15','0', '0','0','base_link','obstacles_link'],
    )

    rear_ultrasonic_group = GroupAction(
        condition=IfCondition(use_rear_ultrasonic),
        actions=[
            launch_ros.actions.Node(
                package='turn_on_wheeltec_robot',
                executable='rear_ultrasonic.py',
                name='rear_ultrasonic_pose',
                parameters=[{
                    "sensor_x": [-0.12, 0.12],
                    "sensor_y": [0.0,0.0],
                    "uldis_max_range": 1.5,
                    "single_sensor_penalty": 0.08,
                }]
            ),

            launch_ros.actions.Node(
                package='tf2_ros', 
                executable='static_transform_publisher', 
                name='base_to_rearobstacles',
                arguments=['-0.30', '0.00', '0.15','0', '0','0','base_link','rear_obstacles_link'],
            ),
            ]
    )
   
    return LaunchDescription([
        declare_use_rear_ultrasonic,ultrasonic_pose,ultrasonic_point,base_to_obstacles,rear_ultrasonic_group,
    ])

