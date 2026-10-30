from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    moveit_config = (
        MoveItConfigsBuilder(
            "s300_pro_description",
            package_name="s300_pro_moveit2_config"
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    octomap_config = {
        "octomap_frame": "base_link",
        "octomap_resolution": 0.02,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        trajectory_execution,
        octomap_config,
    ]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=move_group_params,
        )
    )

    return ld
