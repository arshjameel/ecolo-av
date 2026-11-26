"""
ego: /control/cmd_vel (TwistStamped from twist_mux) -> /ego/actuator_cmd (AckermannDriveStamped).

  ros2 launch aav_ego ego.launch.py [use_sim_time:=false] [max_speed:=0.3] [look_ahead:=true]
                                [road_ahead:=false] [ego_impl:=rust]
                                [speed_loop:=default]

ego_impl (rust): aav_ego_rs, ego and look_ahead in one Rust process, the default on the
car and in the simulator (Ben, 16 Nov). This package's two Python nodes went on 22 Nov
(tag save/python-fallbacks-1); its maths, config and this file stay. The process's
parameters go in a file of their own (both nodes have a rate_hz), with vehicle.yaml's path
and the body outline.

road_ahead (false): true holds Nav2's commands to /world/road_ahead's speed limit
(aav_ego_rs; road_limit.py is the Python yardstick); sim.launch.py sets it with
behaviour:=true, its writer.

speed_loop (default): the spine's speed loop profile the car runs (default, smooth, lqi,
firm), which the trace's twin assumes (trace_speed_loop, over ego.yaml's); sim.launch.py
and bench.launch.py pass theirs on (TODO 163). An unknown name is refused.

look_ahead (true) also starts look_ahead: the squiggly line's numbers for the screens
(/ego/plan_ahead, /ego/ahead; aav_ego_rs). It only listens and publishes those two.

max_speed (m/s) caps the forward speed below config/ego.yaml's, for a first drive on
stands; empty (the default) keeps ego.yaml's. It can only lower the cap, never raise it.
It caps reverse too: max_reverse_speed becomes the smaller of it and vehicle.yaml's.

The wheelbase, the steering limit, the reverse speed and the body (mass, track, centre of
mass, tyre grip, the throttle's and brake's accelerations) come from
aav_description/config/vehicle.yaml; the other limits and the payload from config/ego.yaml.
"""

import os
import tempfile

from aav_description import vehicle
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

with open(os.path.join(get_package_share_directory('aav_ego'), 'config', 'ego.yaml')) as f:
    CONFIG = yaml.safe_load(f)['ego']['ros__parameters']
CONFIG_MAX_SPEED = float(CONFIG['max_speed'])
SIM_MAX_SPEED = 5.0  # m/s: sim_max_speed's ceiling, about 18 km/h


def ego_node(context):
    car = vehicle.load()
    params = [
        # ego.yaml as a dict, not the file: the file names the node (ego:), and a named
        # entry beats the later caps below whenever the launch also sets a global
        # parameter (sim.launch.py's use_sim_time). As dicts, the last one wins.
        dict(CONFIG),
        vehicle.ego_params(car),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    cap = context.launch_configurations.get('max_speed', '').strip()
    if cap:
        limit = float(cap)
        if not 0.0 < limit <= CONFIG_MAX_SPEED:
            raise ValueError(f'max_speed:={cap} must be above 0 and at most '
                             f'{CONFIG_MAX_SPEED} m/s (config/ego.yaml)')
        # Backwards too: "the speed capped at 0.3 m/s" on stands (car day A9) means both
        # ways. Only ever lowers vehicle.yaml's max_reverse_speed (rehearsal, 17 Nov: with
        # max_speed:=0.3 the stick held back still reversed at 0.5 m/s).
        reverse = min(limit, float(car['vehicle']['max_reverse_speed']))
        params.append({'max_speed': limit, 'max_reverse_speed': reverse})
    sim = context.launch_configurations.get('sim_max_speed', '').strip()
    if sim:
        # Above ego.yaml's cap, so only ever in a simulator: refused on wall-clock time.
        if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
            raise ValueError('sim_max_speed:= is for simulators only (use_sim_time:=true)')
        limit = float(sim)
        if not 0.0 < limit <= SIM_MAX_SPEED:
            raise ValueError(f'sim_max_speed:={sim} must be above 0 and at most '
                             f'{SIM_MAX_SPEED} m/s')
        params.append({'max_speed': limit})
    params.append({'trace_speed_loop': vehicle.check_speed_loop(
        context.launch_configurations.get('speed_loop', vehicle.DEFAULT_SPEED_LOOP))})
    if context.launch_configurations.get('road_ahead', 'false').strip().lower() == 'true':
        params.append({'road_ahead': True})
    look_ahead = context.launch_configurations.get('look_ahead', 'true').strip().lower() == 'true'
    # rust only since the Python nodes went (22 Nov); the check refuses anything else.
    vehicle.check_ego_impl(context.launch_configurations.get(
        'ego_impl', vehicle.DEFAULT_EGO_IMPL))
    return [rust_node(context, params, car, look_ahead)]


def rust_node(context, params, car, look_ahead):
    """
    Return aav_ego_rs's process: the nodes ego and look_ahead, each with its own parameters.

    One file, by node name: a plain dict would reach both nodes, and each has a rate_hz.
    """
    sim = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    ego = {}
    for p in params:
        ego.update({k: v for k, v in p.items() if k != 'use_sim_time'})
    ego.update(use_sim_time=sim, vehicle_yaml=vehicle.config_path())
    (front, half), _, (rear, _), _ = vehicle.footprint_points(car['vehicle'])
    look = {'use_sim_time': sim, 'body': [float(front), float(rear), float(half)]}
    with tempfile.NamedTemporaryFile('w', prefix='ego_rs_', suffix='.yaml',
                                     delete=False) as f:
        yaml.safe_dump({'/ego': {'ros__parameters': ego},
                        '/look_ahead': {'ros__parameters': look}}, f)
    return Node(package=vehicle.EGO_IMPLS['rust'], executable='ego', parameters=[f.name],
                arguments=[] if look_ahead else ['--ego-only'])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description="m/s: a lower forward speed cap than ego.yaml's"),
        DeclareLaunchArgument('sim_max_speed', default_value='',
                              description='m/s: a higher cap, in a simulator only '
                                          '(sim.launch.py sim_speed:=)'),
        DeclareLaunchArgument('road_ahead', default_value='false',
                              description='true: Nav2 never faster than /world/road_ahead '
                                          'allows (needs the behaviour planner)'),
        DeclareLaunchArgument('look_ahead', default_value='true',
                              description='also start look_ahead (the squiggly line)'),
        DeclareLaunchArgument('ego_impl', default_value=vehicle.DEFAULT_EGO_IMPL,
                              description='rust: which ego runs (aav_ego_rs: ego and look_ahead '
                                          'in one process; the Python removed 22 Nov)'),
        DeclareLaunchArgument('speed_loop', default_value=vehicle.DEFAULT_SPEED_LOOP,
                              description="the spine's speed loop profile the car runs "
                                          '(default, smooth, lqi, firm): the trace assumes it'),
        OpaqueFunction(function=ego_node),
    ])
