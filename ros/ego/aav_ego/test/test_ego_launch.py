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
    """Python's two nodes unless ego_impl is given (the launch default is rust)."""
    context = LaunchContext()
    context.launch_configurations.update(
        {'use_sim_time': 'false', 'ego_impl': 'python', **config})
    return ego_launch.ego_node(context)


def test_rust_is_the_default():
    # Ben, 16 Nov: the Rust ego on the car too; ego_impl:=python falls back.
    context = LaunchContext()
    context.launch_configurations.update({'use_sim_time': 'false'})
    (process,) = ego_launch.ego_node(context)
    assert process.node_package == 'aav_ego_rs'


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
    """Return the Rust process's parameter file: {'/ego': {...}, '/look_ahead': {...}}."""
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


def last_value(node, name):
    """Return the last value a Python node's parameter dicts give `name` (the one that wins)."""
    context = LaunchContext()

    def text(part):   # launch_ros keeps names and text values as tuples of substitutions
        if isinstance(part, tuple):
            return ''.join(s.perform(context) for s in part)
        return part

    import yaml
    found = [text(value) for p in node._Node__parameters if isinstance(p, dict)
             for key, value in p.items() if text(key) == name]
    # A text value comes back as YAML ('firm\n...\n').
    return yaml.safe_load(found[-1]) if found else None


def test_speed_loop_reaches_the_trace_in_either_ego():
    # TODO 163: the trace's twin assumes the loop the car runs; default unchanged.
    ego, _ = nodes()
    assert last_value(ego, 'trace_speed_loop') == 'default'
    ego, _ = nodes(speed_loop='firm')
    assert last_value(ego, 'trace_speed_loop') == 'firm'
    (process,) = nodes(ego_impl='rust', speed_loop='smooth')
    assert rust_params(process)['/ego']['ros__parameters']['trace_speed_loop'] == 'smooth'


def test_an_unknown_speed_loop_is_refused():
    with pytest.raises(ValueError, match='speed_loop'):
        nodes(speed_loop='fast')
