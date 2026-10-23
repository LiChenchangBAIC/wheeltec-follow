from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = LaunchConfiguration('model')
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='yolov8n-seg.engine',
            description='分割模型路径，车上使用 TensorRT engine'),
        DeclareLaunchArgument(
            'port',
            default_value='8090',
            description='点选页面端口'),
        Node(
            package='yolo_same_follow',
            executable='yolo_same_follow',
            name='yolo_same_follow',
            output='screen',
            parameters=[{
                'model': model,
                'port': ParameterValue(port, value_type=int),
            }],
        ),
        Node(
            package='simple_follower_ros2',
            executable='visualfollow',
            name='visualfollow',
            output='screen',
        ),
    ])
