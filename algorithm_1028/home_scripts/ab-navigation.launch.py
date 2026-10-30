import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = get_package_share_directory('turn_on_wheeltec_robot')
    nav = get_package_share_directory('wheeltec_nav2')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'turn_on_wheeltec_robot.launch.py')),
            launch_arguments={'carto_slam': 'false', 'robot_nav': 'false',
                              'start_state_publisher': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'wheeltec_lidar.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'bringup_launch.py')),
            launch_arguments={'slam': 'False', 'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': 'false', 'autostart': 'true',
                              'use_composition': 'False'}.items()),
    ])
