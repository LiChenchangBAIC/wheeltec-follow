import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_yaml(file_path: Path) -> dict:
    """加载 YAML 文件"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # 获取功能包路径
    pkg_dir = get_package_share_directory('road_network_planner')
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, 'launch')
    
    # 加载配置参数
    cfg_params = load_yaml(os.path.join(wheeltec_robot_dir, 'config', 'wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode: {car_mode}")
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    roadnet_file = LaunchConfiguration('roadnet_file', default='/home/wheeltec/wheeltec_ros2/src/road_network_planner/maps/manual_roadnet.geojson')
    
    # 地图文件路径
    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'WHEELTEC.yaml'))
    
    # 参数文件路径
    param_dir = os.path.join(wheeltec_nav_dir, 'param', 'wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(param_dir, f'param_{car_mode}.yaml'))
    print(f"param_file: {os.path.join(param_dir, f'param_{car_mode}.yaml')}")
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'roadnet_file',
            default_value='/home/wheeltec/wheeltec_ros2/src/road_network_planner/maps/manual_roadnet.geojson',
            description='Path to GeoJSON file for saving/loading road network'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'
        ),
        
        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'
        ),
        
        # ========== 机器人硬件启动 ==========
        # IMU 处理器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/imu_processor.launch.py']
            ),
        ),
        
        # 机器人底盘启动（不启动 SLAM 和导航）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/turn_on_wheeltec_robot.launch.py']
            ),
            launch_arguments={'carto_slam': 'false', 'robot_nav': 'false'}.items(),
        ),
        
        # 激光雷达启动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/wheeltec_lidar.launch.py']
            ),
        ),
        
        # ========== Nav2 导航栈 ==========
        # 包含完整的导航启动（地图服务器、AMCL、控制器等）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launch_dir, '/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file
            }.items(),
        ),
        
        # ========== 手动路网编辑器节点 ==========
        Node(
            package='road_network_planner',
            executable='manual_network_editor.py',
            name='manual_network_editor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'roadnet_file': roadnet_file,
                'COST_THRESHOLD': 60,
                'PATH_OBSTACLES_THRESHOLD': 5
            }]
        ),
        
        # ========== RViz2 已移除 - 用户手动启动 ==========
        # 请在虚拟机端手动打开 RViz2: ros2 run rviz2 rviz2
    ])
