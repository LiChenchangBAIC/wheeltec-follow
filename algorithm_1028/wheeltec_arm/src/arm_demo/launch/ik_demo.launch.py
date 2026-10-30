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



    ik_node = TimerAction(
        period=3.0,  # 3秒延时
        actions=[
            Node(
                package="arm_demo",  # 节点所在的软件包名
                executable="ik_demo",  # 节点的可执行文件名              
                parameters=[  # 节点参数
                    robot_description_semantic  # 机器人模型参数
                ]
            )
        ]
    )

    return LaunchDescription([
            
            realman,#启动机械臂
            ik_node
    ])

    
