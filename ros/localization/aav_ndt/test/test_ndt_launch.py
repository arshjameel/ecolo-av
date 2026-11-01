"""
aav_ndt's ndt.launch.py: the car's measurements from vehicle.yaml, the map, the start.

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
    path = os.path.join(get_package_share_directory('aav_ndt'), 'launch', 'ndt.launch.py')
    spec = importlib.util.spec_from_file_location('ndt_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(launch_file, **args):
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(
            context, declared.default_value)
    context.launch_configurations.update(args)
    return context


def parameters(**args):
    """Return the node's parameters, the config file's and the launch file's together."""
    launch_file = load()
    context = context_for(launch_file, **args)
    (node,) = launch_file.nodes(context)
    found = {}
    for p in node._Node__parameters:
        if isinstance(p, ParameterFile):
            continue
        for key, value in evaluate_parameters(context, [p])[0].items():
            found[key] = value
    config = os.path.join(get_package_share_directory('aav_ndt'), 'config', 'ndt.yaml')
    with open(config) as f:
        return {**yaml.safe_load(f)['ndt']['ros__parameters'], **found}


def test_a_missing_map_is_refused_with_how_to_make_it(tmp_path):
    with pytest.raises(ValueError, match='pixi run ndt-map drive'):
        parameters(ndt_map=str(tmp_path / 'none.pcd'))


def test_the_mount_and_body_come_from_vehicle_yaml(tmp_path):
    map_file = tmp_path / 'lot.pcd'
    map_file.write_bytes(b'')
    p = parameters(ndt_map=str(map_file), ndt_y='4.0', ndt_yaw='1.5')
    v = vehicle.load()
    lidar, body = v['sensors']['lidar_link'], v['vehicle']['body']
    assert list(p['lidar_xyz']) == pytest.approx(lidar['xyz'])
    assert list(p['lidar_rpy']) == pytest.approx(lidar['rpy'])
    assert list(p['body_min']) == pytest.approx([-body['rear_overhang'], -body['width'] / 2])
    assert p['body_max'][0] == pytest.approx(body['length'] - body['rear_overhang'])
    assert p['map'] == str(map_file)
    assert (p['start_x'], p['start_y'], p['start_yaw']) == (0.0, 4.0, 1.5)
    assert p['start_from_global'] is False


def test_the_config_keeps_the_designs_numbers():
    config = os.path.join(get_package_share_directory('aav_ndt'), 'config', 'ndt.yaml')
    with open(config) as f:
        p = yaml.safe_load(f)['ndt']['ros__parameters']
    assert p['cubes'] == [4.0, 2.0, 1.0]
    assert (p['min_height'], p['max_range'], p['thin']) == (0.3, 45.0, 1.0)
    assert (p['lost_after'], p['step_m']) == (5, 0.1)
