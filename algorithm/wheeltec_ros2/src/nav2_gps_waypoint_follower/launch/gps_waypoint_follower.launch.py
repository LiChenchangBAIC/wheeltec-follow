# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    #wheeltec robot's footprint
    S200_4wd = '[[-0.55, -0.3280], [-0.55, 0.3280], [0.144, 0.3280], [0.144, -0.3280]]'
    autoware_senior_akm = '[ [-0.09, -0.185], [-0.09, 0.185], [0.4,0.185], [0.4, -0.185] ]'
    autoware_top_akm = '[ [-0.15, -0.38], [-0.15, 0.38],[0.67,0.38],[0.67, -0.38] ]'
    mini_diff = '[ [-0.03, -0.0900], [-0.03, 0.0900], [0.2, 0.0900], [0.2, -0.0900] ]'

    footprint = autoware_senior_akm

    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key="", 
        param_rewrites={
        'local_costmap.local_costmap.ros__parameters.footprint': footprint,
        'global_costmap.global_costmap.ros__parameters.footprint': footprint,
        }, 
        convert_types=True
    )
    wheeltec_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'wheeltec_sensors.launch.py'))
    )
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    waypoint_cycle = Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(wheeltec_sensors)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)
    ld.add_action(waypoint_cycle)

    return ld
