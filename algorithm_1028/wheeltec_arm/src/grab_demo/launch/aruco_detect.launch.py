from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_aruco = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='arm_aruco',
        parameters=[{
            'image_is_rectified': False,
            'marker_size': 0.045,
            'reference_frame': '',
            'camera_frame': 'camera_arm_color_optical_frame',
        }],
        remappings=[
            ('/camera_info', '/gemini_info'),
            ('/image', '/camera_arm/color/image_raw'),
            ('/markers', '/arm_aruco/markers'),
        ],
        output='screen',
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

    aruco_markers_to_tf = Node(
        package='grab_demo',
        executable='aruco_marker_tf_pub_node',
        name='aruco_markers_to_tf_node',
        parameters=[{
            'markers_topic': '/arm_aruco/markers',
            'default_parent_frame': 'camera_arm_color_optical_frame',
            'child_frame_prefix': 'aruco_',
            'child_frame_suffix': '_frame',
        }],
        output='screen',
    )

    return LaunchDescription([
        arm_aruco,
        aruco_markers_to_tf,
        camera_info,
        camera_launch
    ])