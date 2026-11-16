"""ego.launch.py's speed caps: max_speed only lowers; sim_max_speed only in a simulator."""

import importlib.util
from pathlib import Path

import pytest

try:
    from launch import LaunchContext
    from launch_ros.actions import Node
    LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'ego.launch.py'
    spec = importlib.util.spec_from_file_location('ego_launch', LAUNCH)
    ego_launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ego_launch)   # reads ego.yaml from the installed share
    HAVE_ROS = True
except Exception:   # noqa: BLE001 - no ROS, or the package isn't installed
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS and the built workspace')


def nodes(**config):
    context = LaunchContext()
    context.launch_configurations.update({'use_sim_time': 'false', **config})
    return ego_launch.ego_node(context)


def test_no_caps_by_default():
    ego, look_ahead = nodes()
    assert isinstance(ego, Node) and isinstance(look_ahead, Node)


def test_look_ahead_can_be_left_out():
    (ego,) = nodes(look_ahead='false')
    assert isinstance(ego, Node)


def test_max_speed_can_only_lower():
    nodes(max_speed='0.3')
    with pytest.raises(ValueError):
        nodes(max_speed='2.0')


def test_sim_max_speed_is_refused_on_the_car():
    with pytest.raises(ValueError, match='simulators only'):
        nodes(sim_max_speed='3.0')


def test_sim_max_speed_in_a_simulator():
    nodes(use_sim_time='true', sim_max_speed='3.0')
    with pytest.raises(ValueError):
        nodes(use_sim_time='true', sim_max_speed='9.0')


def rust_params(node):
    """The Rust process's parameter file, read back: {'/ego': {...}, '/look_ahead': {...}}."""
    import yaml
    (path,) = node._Node__parameters   # the file ego.launch.py wrote, as given
    with open(path.evaluate(LaunchContext())) as f:   # a launch_ros ParameterFile
        return yaml.safe_load(f)


def test_ego_impl_rust_is_one_process_with_both_nodes():
    (process,) = nodes(ego_impl='rust', use_sim_time='true', sim_max_speed='3.0')
    assert isinstance(process, Node)
    assert process.node_package == 'aav_ego_rs' and process.node_executable == 'ego'
    files = rust_params(process)
    ego, look = files['/ego']['ros__parameters'], files['/look_ahead']['ros__parameters']
    assert ego['max_speed'] == 3.0 and ego['use_sim_time'] is True
    assert ego['rate_hz'] == 50.0 and ego['vehicle_yaml'].endswith('vehicle.yaml')
    assert look['use_sim_time'] is True and len(look['body']) == 3
    assert 'rate_hz' not in look


def test_ego_impl_rust_without_look_ahead():
    (process,) = nodes(ego_impl='rust', look_ahead='false')
    assert process._Node__arguments == ['--ego-only']


def test_ego_impl_is_checked():
    with pytest.raises(ValueError, match='ego_impl'):
        nodes(ego_impl='cpp')
