from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('auto_relocalization')
    param_file = os.path.join(
        pkg_dir,
        'config',
        'relocalization.yaml'
    )
    relocalization_node = Node(
        package='auto_relocalization',
        executable='simple_relocalization',
        name='simple_relocalization',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        relocalization_node
    ])