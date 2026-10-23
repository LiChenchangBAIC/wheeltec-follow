import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_robot_config():
    # 从 wheeltec_param.yaml 读取 car_mode
    try:
        wheeltec_param = load_yaml('turn_on_wheeltec_robot', 'config/wheeltec_param.yaml')
        car_mode = wheeltec_param.get('car_mode', 'S300_pro').strip()
    except Exception as e:
        print(f"Warning: Failed to read wheeltec_param.yaml: {e}, using default 'S300_pro'")
        car_mode = 'S300_pro'

    # 将 car_mode 转换为小写用于匹配
    robot_type = car_mode.lower()

    # 定义不同机器人型号对应的配置
    robot_configs = {
        'eco65': {
            'moveit2_package': 'rm_eco65_moveit2_config',
            'srdf_file': 'config/rm_eco65_description.srdf',

            'real_robot_package': 'rm_bringup',
            'launch_file': 'eco65_real_robot.launch.py'
        },
        's300_pro': {
            'moveit2_package': 's300_pro_moveit2_config',
            'srdf_file': 'config/s300_pro_description.srdf',

            'real_robot_package': 'rm_bringup',
            'launch_file': 's300_pro_real_robot.launch.py'
        },
        's300_mini': {
            'moveit2_package': 's300_mini_moveit2_config',
            'srdf_file': 'config/s300_mini_description.srdf',

            'real_robot_package': 'rm_bringup',
            'launch_file': 's300_mini_real_robot.launch.py'
        }
    }

    # 获取对应配置，如果不存在则使用默认配置
    config = robot_configs.get(robot_type)
    if config is None:
        print(f"Warning: Unknown robot type '{robot_type}' from car_mode '{car_mode}', using default 'S300_pro' configuration")
        config = robot_configs['s300_pro']

    return config



def generate_launch_description():


    # 获取机器人配置
    robot_config = get_robot_config()
    
    # 使用对应的配置文件
    robot_description_semantic_config = load_file(
        robot_config['moveit2_package'],
        robot_config['srdf_file']
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # 启动对应的机器人
    realman = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_config['real_robot_package']),
                'launch',
                robot_config['launch_file']
            ])
        ])
    )

    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
        )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('grab_demo'),
                'launch',
                'arm_camera.launch.py'
            ])
        ])
    )

    move_look_node=Node(
            package="arm_demo",
            executable="move_look",
        )
    # 从配置获取 robot_type 用于判断是否需要反转
    try:
        wheeltec_param = load_yaml('turn_on_wheeltec_robot', 'config/wheeltec_param.yaml')
        car_mode = wheeltec_param.get('car_mode', 'S300_pro').strip()
        robot_type = car_mode.lower()
    except:
        robot_type = 's300_pro'
    is_inverted = (robot_type == 'eco65')

    grab_place_service_node = Node(
            package="grab_demo",
            executable="grab_place_service",
            parameters=[
                robot_description_semantic,
                {
                    "base_frame": "base_link",
                    "tf_timeout_sec": 10.0,

                    "grab_offset_x": 0.0,
                    "grab_offset_y": 0.0,
                    "grab_offset_z": 0.0,

                    "return_after_grab": True,

                    "gripper_open_position": 1000,
                    "gripper_close_position": 200,

                    "place_approach_z": 0.05,
                    "return_look_after_fixed_place": True,
                    "return_look_after_frame_place": False,
                }
            ],
            output="screen"
    )

    
    delay_task=TimerAction(period=15.0,actions=[grab_place_service_node])

    arm_motion_service = Node(
    package='grab_demo',
    executable='arm_motion_service',
    parameters=[
        robot_description_semantic,
        {
            'base_frame': 'base_link',
            'default_velocity_scale': 0.3,
            'default_acceleration_scale': 0.3,
            'max_relative_step_m': 0.10,
            'max_down_step_m': 0.05,
            'tf_timeout_sec': 2.0,
            'invert_x': is_inverted,
            'invert_y': is_inverted,
        }
    ],
    output='screen'
    )



    config = os.path.join(
        get_package_share_directory('vlm_demo'),
        'config',
        'vlm_config.yaml',
    )
    
    vision_grounding = Node(
        package='vlm_demo',
        executable='vision_grounding_node',
        name='vision_grounding_node',
        output='screen',
        parameters=[config],
    )

    geometry_resolver = Node(
        package='vlm_demo',
        executable='geometry_resolver_node',
        name='geometry_resolver_node',
        output='screen',
        parameters=[config],
    )

    pick_task_manager = Node(
        package='vlm_demo',
        executable='pick_task_manager_node',
        name='pick_task_manager_node',
        output='screen',
        parameters=[config],
    )


    return LaunchDescription([
        realman,#启动机械臂
        camera_info,
        camera_launch,
        delay_task,
        move_look_node,
        arm_motion_service,
        vision_grounding,
        geometry_resolver, 
        TimerAction(period=3.0, actions=[pick_task_manager]),
    ])
