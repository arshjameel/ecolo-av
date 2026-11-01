"""
carla.launch.py (aav_bringup) and aav_carla's carla.launch.py: what they would start.

Nothing starts: no ROS, no CARLA. The whole stack's part must match sim.launch.py's, with
carla_adapter for gazebo_adapter; aav_carla's part must hand carla_adapter the right
parameters, and refuse bad ones before anything starts.
"""

import os

from launch_ros.actions import Node
import pytest
from test_sim_launch import context_for, included, load, names, node, parameters, started

BRINGUP = load('aav_bringup', 'carla.launch.py')
BACKEND = load('aav_carla', 'carla.launch.py')


def test_the_defaults_start_the_same_stack_as_gazebo():
    actions = started(BRINGUP, host='192.0.2.10')
    sent = included(actions)
    assert set(sent) == {'carla.launch.py', 'mux.launch.py', 'ego.launch.py',
                         'io_mux.launch.py', 'scripted.launch.py', 'localization.launch.py',
                         'radar_clusters.launch.py', 'navigation.launch.py',
                         'watchdog.launch.py'}
    assert sent['carla.launch.py']['host'] == '192.0.2.10'
    assert sent['io_mux.launch.py'] == {'target': 'carla', 'source': 'sim'}
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['collision_monitor']) == ('true', 'true', 'true')
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    assert sent['watchdog.launch.py']['inputs'] == 'odom,imu,gps,local,global,radar'
    # No geofence in CARLA yet, so its fail-safe lock must be off, or nothing could drive.
    assert sent['mux.launch.py']['geofence_lock'] == 'false'
    assert names(actions) == {'imu_filter_madgwick'}     # no RViz: the CARLA box may be headless


def test_without_radar_or_lidar_nav2_runs_unguarded():
    sent = included(started(BRINGUP, radar='false', lidar='false'))
    assert 'radar_clusters.launch.py' not in sent
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['collision_monitor']) == ('false', 'false', 'false')


def adapter(**args):
    actions = started(BACKEND, **args)
    return parameters(node(actions, 'carla_adapter')), actions


def test_the_adapter_gets_its_parameters_and_the_datum():
    p, actions = adapter(host='192.0.2.10', port='2001', town='Town10HD_Opt', spawn='2',
                         sync='false', control='pedals', radar='false')
    assert (p['host'], p['port'], p['town'], p['spawn']) == ('192.0.2.10', 2001,
                                                             'Town10HD_Opt', '2')
    assert (p['sync'], p['radar'], p['lidar'], p['control']) == (False, False, True, 'pedals')
    assert 45.3 < p['datum_lat'] < 45.5 and -75.8 < p['datum_lon'] < -75.6   # Carleton
    assert p['use_sim_time'] is False           # it IS the clock
    assert included(actions)['description.launch.py']['use_sim_time'] == 'true'


def test_an_osm_file_is_passed_whole(tmp_path):
    osm = tmp_path / 'carleton.osm'
    osm.write_text('<osm/>')
    p, _ = adapter(osm=str(osm))
    assert p['osm'] == str(osm)


@pytest.mark.parametrize('args', [{'frame': 'town'}, {'control': 'throttle'},
                                  {'spawn': '1,2'}, {'sync': 'yes'},
                                  {'osm': '/no/such/file.osm'},
                                  {'osm': __file__, 'xodr': __file__}])
def test_bad_arguments_are_refused_before_anything_starts(args):
    with pytest.raises(ValueError):
        BACKEND.launch_setup(context_for(BACKEND, **args))


def test_the_backend_starts_only_the_adapter_and_the_model():
    actions = started(BACKEND)
    assert [a._Node__node_name for a in actions if isinstance(a, Node)] == ['carla_adapter']
