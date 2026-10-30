"""Create a Nav2 session with the requested motion limits; sends no pose or goal."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory

map_path = Path('/home/wheeltec/slam_sessions/slam_20261022_c144ea1h/final_map.yaml')
assert map_path.is_file(), map_path
peer = os.environ.get('LOCAL_DDS_PEER')
if not peer:
    raise SystemExit('Set LOCAL_DDS_PEER to the current development computer IPv4 address.')
processes = subprocess.check_output(['ps', '-eo', 'args'], text=True)
for owner in ('/wheeltec_robot_node', '/async_slam_toolbox_node', '/lslidar_driver_node'):
    if owner in processes:
        raise SystemExit('Hardware is already in use: ' + owner)
root = Path.home() / 'navigation_sessions'
root.mkdir(exist_ok=True)
session = Path(tempfile.mkdtemp(prefix='ab_20261022_', dir=root))
source = Path(get_package_share_directory('wheeltec_nav2')) / 'param/wheeltec_params/param_S200_4wd.yaml'
params = yaml.safe_load(source.read_text())
params['amcl']['ros__parameters']['set_initial_pose'] = False
params['controller_server']['ros__parameters']['goal_checker']['stateful'] = False
params['controller_server']['ros__parameters']['progress_checker'].update(
    required_movement_radius=0.15, movement_time_allowance=20.0)
follow = params['controller_server']['ros__parameters']['FollowPath']
follow.update(vx_max=0.5, vx_min=-0.2, vy_max=0.0, wz_max=0.5,
              ax_max=1.5, ax_min=-1.5, az_max=0.5,
              rotate_to_heading_angular_vel=0.5, max_angular_accel=0.5,
              closed_loop=False, forward_sampling_distance=0.2)
for key, layer in [('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')]:
    params[key][key]['ros__parameters'][layer]['observation_sources'] = 'scan'
    params[key][key]['ros__parameters']['inflation_layer']['inflation_radius'] = 0.2
# Keep the MPPI obstacle critic's cost-distance model aligned with the maps.
if 'ObstaclesCritic' in follow:
    follow['ObstaclesCritic']['inflation_radius'] = 0.2
params['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = 'odom_combined'
params['behavior_server']['ros__parameters']['global_frame'] = 'odom_combined'
params['behavior_server']['ros__parameters']['backup']['max_linear_vel'] = 0.2
params['velocity_smoother'] = {'ros__parameters': {
    'use_sim_time': False, 'smoothing_frequency': 20.0, 'feedback': 'OPEN_LOOP',
    'scale_velocities': False, 'max_velocity': [0.5, 0.0, 0.5],
    'min_velocity': [-0.2, 0.0, -0.5], 'max_accel': [1.5, 0.0, 0.5],
    'max_decel': [-1.5, 0.0, -0.5], 'odom_topic': '/odom_combined',
    'deadband_velocity': [0.0, 0.0, 0.0], 'velocity_timeout': 1.0}}
profile = session / 'ab_params.yaml'
profile.write_text(yaml.safe_dump(params, sort_keys=False))
launch = session / 'ab-navigation.launch.py'
shutil.copyfile(Path(__file__).with_name(launch.name), launch)
command = ['ros2', 'launch', str(launch), 'map:=' + str(map_path), 'params_file:=' + str(profile)]
dds_profile = session / 'cyclonedds.xml'
subprocess.check_call(['python3', str(Path(__file__).with_name('cyclone-network-config.py')),
                       peer, str(dds_profile)], stdout=subprocess.DEVNULL)
environment = dict(os.environ, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=dds_profile.as_uri())
environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
with (session / 'launch.log').open('w') as log:
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True, env=environment)
info = {'session': str(session), 'pid': process.pid, 'pgid': process.pid,
        'map': str(map_path), 'params': str(profile), 'dds_profile': str(dds_profile), 'command': command,
        'initial_pose_sent': False, 'goal_sent': False}
(session / 'session.json').write_text(json.dumps(info, indent=2))
print(json.dumps(info, indent=2))
