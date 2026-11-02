"""
aav_kiss_icp's kiss_icp.launch.py: the mount and body from vehicle.yaml; no kiss env, no node.

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
    path = os.path.join(get_package_share_directory('aav_kiss_icp'), 'launch',
                        'kiss_icp.launch.py')
    spec = importlib.util.spec_from_file_location('kiss_icp_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parameters(**args):
    """Return the node's parameters, the config file's and the launch file's together."""
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
    config = os.path.join(get_package_share_directory('aav_kiss_icp'), 'config',
                          'kiss_icp.yaml')
    with open(config) as f:
        return {**yaml.safe_load(f)['kiss_icp']['ros__parameters'], **found}


def test_no_kiss_environment_is_refused_with_how_to_install_it(tmp_path):
    with pytest.raises(ValueError, match='pixi install --locked -e kiss'):
        parameters(kiss_python=str(tmp_path / 'none'))


def test_the_mount_and_body_come_from_vehicle_yaml(tmp_path):
    python = tmp_path / 'python'
    python.write_text('')
    p = parameters(kiss_python=str(python), use_sim_time='true')
    v = vehicle.load()
    lidar, body = v['sensors']['lidar_link'], v['vehicle']['body']
    assert list(p['lidar_xyz']) == pytest.approx(lidar['xyz'])
    assert list(p['lidar_rpy']) == pytest.approx(lidar['rpy'])
    assert list(p['body_min']) == pytest.approx([-body['rear_overhang'], -body['width'] / 2])
    assert p['body_max'][0] == pytest.approx(body['length'] - body['rear_overhang'])
    assert p['kiss_python'] == str(python)
    assert p['use_sim_time'] is True


def test_the_config_keeps_kiss_icps_own_defaults():
    config = os.path.join(get_package_share_directory('aav_kiss_icp'), 'config',
                          'kiss_icp.yaml')
    with open(config) as f:
        p = yaml.safe_load(f)['kiss_icp']['ros__parameters']
    assert (p['max_range'], p['voxel_size'], p['max_points_per_voxel']) == (100.0, 0.0, 20)
    assert p['deskew'] is True
    assert p['min_height'] == 0.3      # the ground out, as NDT's
