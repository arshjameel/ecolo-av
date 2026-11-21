"""
aav_kinematic_icp's kinematic_icp.launch.py: the mount and body from vehicle.yaml.

Nothing starts: the launch file's own setup runs, and the node's parameters are read.
Needs ROS sourced: `colcon test` runs it.
"""

import importlib.util
import os

import pytest

# Not pytest.importorskip: ROS's launch_testing pytest plugin turns a module-level skip
# into a skip of every test in the folder.
try:
    from aav_description import vehicle
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.descriptions import ParameterFile
    from launch_ros.utilities import evaluate_parameters
    import yaml
except ImportError:
    vehicle = None

pytestmark = pytest.mark.skipif(vehicle is None, reason='needs ROS sourced: colcon test')


def load():
    path = os.path.join(get_package_share_directory('aav_kinematic_icp'), 'launch',
                        'kinematic_icp.launch.py')
    spec = importlib.util.spec_from_file_location('kinematic_icp_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parameters(**args):
    """Return the node and its parameters, the config file's and the launch file's."""
    launch_file = load()
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(
            context, declared.default_value)
    context.launch_configurations.update(args)
    (node,) = launch_file.nodes(context)
    found = {}
    for p in node._Node__parameters:
        if not isinstance(p, ParameterFile):
            found.update(evaluate_parameters(context, [p])[0])
    config = os.path.join(get_package_share_directory('aav_kinematic_icp'), 'config',
                          'kinematic_icp.yaml')
    with open(config) as f:
        return node, {**yaml.safe_load(f)['kinematic_icp']['ros__parameters'], **found}


def test_the_mount_and_body_come_from_vehicle_yaml():
    node, p = parameters(use_sim_time='true')
    v = vehicle.load()
    lidar, body = v['sensors']['lidar_link'], v['vehicle']['body']
    assert list(p['lidar_xyz']) == pytest.approx(lidar['xyz'])
    assert list(p['lidar_rpy']) == pytest.approx(lidar['rpy'])
    assert list(p['body_min']) == pytest.approx([-body['rear_overhang'], -body['width'] / 2])
    assert p['body_max'][0] == pytest.approx(body['length'] - body['rear_overhang'])
    assert p['use_sim_time'] is True
    assert node._Node__node_executable == 'kinematic_icp'


def test_the_prior_is_the_wheels_not_ekf_locals_own_output():
    # ekf_local reads this node's twist (kiss_ekf:=true): its prior must not be ekf_local's
    # output, or the two would feed each other.
    _, p = parameters()
    assert p['wheel_topic'] == '/vehicle/odom'
    assert p['min_height'] == 0.3
