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
import signal

from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events.process import SignalProcess
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
    ]),
    launch_arguments={'start_moveit': 'false'}.items()
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
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.07',
        description='ArUco marker size in meters'
    )

    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size':  LaunchConfiguration('marker_size'),
        'marker_id': 8,
        'reference_frame': '',
        'camera_frame': 'camera_arm_color_optical_frame',
        'marker_frame': 'aruco_marker_frame',
        'corner_refinement': 'LINES'
    }

    aruco_dectet = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[
            ('/camera_info', '/gemini_info'),
            ('/image', '/camera_arm/color/image_raw')
        ],
    )

    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
        )
    
    moveit_process = ExecuteProcess(
    cmd=[
        'ros2', 'launch',
        robot_config['moveit2_package'],
        'real_robot_moveit.launch.py'
    ],
    output='screen',
    sigterm_timeout='5',
    sigkill_timeout='5'
    )

    move_look_node=Node(
            package="arm_demo",
            executable="move_look",
            output='screen'
        )

    start_move_look = TimerAction(
    period=3.0,
    actions=[move_look_node]
    )

    stop_moveit_after_move_look = RegisterEventHandler(
        OnProcessExit(
            target_action=move_look_node,
            on_exit=[
                EmitEvent(
                    event=SignalProcess(
                        process_matcher=lambda action: action == moveit_process,
                        signal_number=signal.SIGINT,
                    )
                ),
                TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg='已运动到初始点并关闭moveit，节省系统占用，接下来可运行伺服跟随节点')
                ]
                )
            ]
        )
    )
 
    return LaunchDescription([
            marker_size_arg,
            camera_launch,#启动相机
            camera_info,#相机内参发布节点
            realman,#启动机械臂
            aruco_dectet,#aruco识别节点
            moveit_process,
            start_move_look,
            stop_moveit_after_move_look,
            
    ])

    
