"""
stack.launch.py (target:=, source:=), and the two muxes in every profile.

Each test runs a launch file's own setup and looks at what it would start. Nothing
starts: no ROS. Every profile runs the sensor mux and the output mux with its own target,
so no backend is ever reached straight from ego.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


def load(package, name):
    path = os.path.join(get_package_share_directory(package), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STACK = load('aav_bringup', 'stack.launch.py')
IO_MUX = load('aav_io_mux', 'io_mux.launch.py')


def context_for(launch_file, **args):
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        if declared.default_value is not None:
            context.launch_configurations[declared.name] = perform_substitutions(
                context, declared.default_value)
    context.launch_configurations.update(args)
    return context


def started(launch_file, **args):
    return launch_file.launch_setup(context_for(launch_file, **args))


def text(value):
    """Return a launch argument as text: as given, or its substitutions performed."""
    return value if isinstance(value, str) else perform_substitutions(LaunchContext(), value)


def included(actions):
    """Map each launch file included to the arguments it is given."""
    found = {}
    for a in actions:
        if isinstance(a, IncludeLaunchDescription):
            # Loading the file expands its path; it also checks that the file loads.
            a.launch_description_source.get_launch_description(LaunchContext())
            found[os.path.basename(a.launch_description_source.location)] = {
                text(k): text(v) for k, v in a.launch_arguments}
    return found


@pytest.mark.parametrize('target, profile', [
    ('turtle', 'turtle.launch.py'), ('ravensim', 'sim.launch.py'),
    ('bench', 'bench.launch.py'), ('car', 'car.launch.py'), ('carla', 'carla.launch.py')])
def test_the_target_picks_the_profile(target, profile):
    assert list(included(started(STACK, target=target))) == [profile]


@pytest.mark.parametrize('target, source', [('car', 'sim'), ('car', 'bench'),
                                            ('ravensim', 'real'), ('bench', 'sim')])
def test_a_source_the_target_cant_take_is_refused_before_anything_starts(target, source):
    with pytest.raises(ValueError, match='needs source:='):
        started(STACK, target=target, source=source)


def test_an_unknown_target_is_refused():
    with pytest.raises(ValueError, match='target:=sim'):
        started(STACK, target='sim')


@pytest.mark.parametrize('profile, target, source', [
    ('turtle.launch.py', 'turtle', 'sim'), ('bench.launch.py', 'bench', 'bench'),
    ('carla.launch.py', 'carla', 'sim'), ('replay.launch.py', '', 'replay')])
def test_each_profile_runs_the_two_muxes_with_its_own_target(profile, target, source):
    module = load('aav_bringup', profile)
    if hasattr(module, 'launch_setup'):
        actions = started(module)
    else:
        actions = module.generate_launch_description().entities
    assert included(actions)['io_mux.launch.py'] == {'target': target, 'source': source}


def test_the_io_mux_starts_both_or_only_the_sensor_mux():
    nodes = started(IO_MUX, target='car', source='')
    assert [(n.node_executable, n._Node__node_name) for n in nodes
            if isinstance(n, Node)] == [('sensor_mux', 'sensor_mux'), ('output_mux', 'output_mux')]
    replay = started(IO_MUX, target='', source='replay')
    assert [n.node_executable for n in replay] == ['sensor_mux']
    with pytest.raises(ValueError, match='needs source:=real'):
        started(IO_MUX, target='car', source='replay')


def test_the_muxes_are_rust_by_default_and_python_one_switch_away():
    # Ben, 17 Nov: "Rust tomorrow": both muxes, together; io_mux_impl:=python falls back.
    for impl, package in (('', 'aav_io_mux_rs'), ('rust', 'aav_io_mux_rs'),
                          ('python', 'aav_io_mux')):
        args = {'io_mux_impl': impl} if impl else {}
        nodes = [n for n in started(IO_MUX, target='car', source='', **args)
                 if isinstance(n, Node)]
        assert [n.node_package for n in nodes] == [package, package], impl
    with pytest.raises(ValueError, match='io_mux_impl:=c'):
        started(IO_MUX, target='car', source='', io_mux_impl='c')
