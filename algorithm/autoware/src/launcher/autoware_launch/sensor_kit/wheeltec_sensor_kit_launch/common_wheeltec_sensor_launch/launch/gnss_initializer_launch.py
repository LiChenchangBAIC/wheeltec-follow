import os
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    gnss_initializer = launch_ros.actions.Node(
            package='common_wheeltec_sensor_launch', 
            executable='gnss_initializer_node', 
            name='gnss_initializer_node',
            #output='screen',
    )

    return LaunchDescription([
        gnss_initializer
    ])

