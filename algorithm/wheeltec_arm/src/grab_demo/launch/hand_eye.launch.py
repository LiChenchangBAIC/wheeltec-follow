import os
import yaml
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction
from launch import LaunchDescription
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
            'launch_file': 'eco65_real_robot.launch.py',
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
    config = robot_configs.get(robot_type)
    if config is None:
        print(f"Warning: Unknown robot type '{robot_type}' from car_mode '{car_mode}', using default 'S300_pro' configuration")
        config = robot_configs['s300_pro']

    print(f"[hand_eye_launch] car_mode = {car_mode}, robot_type = {robot_type}")
    print(f"[hand_eye_launch] real_robot launch = {config['real_robot_package']}/launch/{config['launch_file']}")
    print(f"[hand_eye_launch] moveit package = {config['moveit2_package']}")

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

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('grab_demo'),
                'launch',
                'arm_camera.launch.py'
            ])
        ])
    )


    # 从配置获取 robot_type 用于选择可执行文件
    try:
        wheeltec_param = load_yaml('turn_on_wheeltec_robot', 'config/wheeltec_param.yaml')
        car_mode = wheeltec_param.get('car_mode', 'S300_pro').strip()
        robot_type = car_mode.lower()
    except:
        robot_type = 's300_pro'

    if robot_type.startswith('s300'):
        executable = 's300_hand_eye'
    else:  # eco65
        executable = 'hand_eye'
    print(f"[hand_eye_launch] hand_eye executable = {executable}")

    hand_eye = TimerAction(
        period=5.0,  # 5秒延时
        actions=[
            Node(
                package="grab_demo",
                executable=executable,
            )
        ]
    )

    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
        )

    camera_undis=Node(
            package="grab_demo",
            executable="camera_undistort_node",
        )

    charuco_dectet=Node(
            package="grab_demo", 
            executable="charuco_dectet_node",
             parameters=[
                {'squaresX': 7},#charuco纵向个数
                {'squaresY': 5},#charuco横向个数
                {'squareLength': 0.03655},#单个棋盘格边长0.035 
                {'markerLength': 0.0235},#单个aruco码边长0.022
                {'rgb_image_topic': '/camera_arm/color/image_raw'},#订阅的rgb话题名
                {'rgb_image_info': '/gemini_info'},#相机内参话题名
                {'rgb_result_pub': '/charuco_dect'},#识别结果发布话题名
                {'dictionary_id': 6}#aruco字典
            ], 
             output="screen"
         
        ) 


 
    return LaunchDescription([
            camera_launch,#启动相机
            camera_info,#相机内参发布节点
            charuco_dectet,#charuco标定板识别节点
            realman,#启动机械臂
            hand_eye,#手眼标定节点，订阅相机话题名以及相机内参话题名，订阅charuco位姿信息，控制机械臂采样计算手眼位姿
    ])

    
