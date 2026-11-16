"""
A node's own config file must not set what navigation.launch.py sets for it.

An entry under the node's own name beats the launch file's value whenever the launch
also sets a global parameter (use_sim_time): ego.launch.py found it first. Until 14 Nov
config/mppi_lag.yaml set costmap_topic, so with costmap:=own mppi_lag read Nav2's
costmap topic, which nobody published, and drove blind to it. No ROS needed.
"""

from pathlib import Path
import sys

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROS = PACKAGE.parents[1]
sys.path.insert(0, str(ROS / 'bringup' / 'aav_description'))
from aav_description import vehicle  # noqa: E402, I100 - from the source tree, above

# What the launch file adds beside vehicle.mppi_lag_params (navigation.launch.py).
LAUNCH_EXTRA = {'use_sim_time', 'costmap_topic', 'map_correction'}
# The same for aav_track's track node (navigation.launch.py's dict beside track_params).
TRACK_EXTRA = {'tracker', 'use_sim_time', 'road_ahead_topic', 'live_swap', 'map_correction'}


def flat(params, prefix=''):
    """Return a file's parameter names as ROS sees them: nested blocks as `a.b`."""
    names = set()
    for key, value in params.items():
        if isinstance(value, dict):
            names |= flat(value, f'{prefix}{key}.')
        else:
            names.add(prefix + key)
    return names


def test_flat_names_the_nested_parameters():
    # mpc_lag.speed_loop (TODO 163) is the launch's: a nested key in a file must count.
    assert flat({'a': 1, 'mpc_lag': {'speed_loop': 'x', 'q': [1]}}) == \
        {'a', 'mpc_lag.speed_loop', 'mpc_lag.q'}


def test_mppi_lag_yaml_sets_nothing_the_launch_sets():
    own = yaml.safe_load((ROS / 'drivers' / 'aav_mppi_lag' / 'config' / 'mppi_lag.yaml')
                         .read_text())['mppi_lag']['ros__parameters']
    nav2 = yaml.safe_load((PACKAGE / 'config' / 'nav2.yaml').read_text())
    cfg = vehicle.load(ROS / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml')
    ego = yaml.safe_load((ROS / 'ego' / 'aav_ego' / 'config' / 'ego.yaml')
                         .read_text())['ego']['ros__parameters']
    launch = set(vehicle.mppi_lag_params(nav2, cfg, ego)) | LAUNCH_EXTRA
    both = sorted(flat(own) & launch)
    assert not both, f'config/mppi_lag.yaml sets what the launch sets (the file wins): {both}'


def test_track_yaml_sets_nothing_the_launch_sets():
    own = yaml.safe_load((ROS / 'drivers' / 'aav_track' / 'config' / 'track.yaml')
                         .read_text())['track']['ros__parameters']
    nav2 = yaml.safe_load((PACKAGE / 'config' / 'nav2.yaml').read_text())
    cfg = vehicle.load(ROS / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml')
    ego = yaml.safe_load((ROS / 'ego' / 'aav_ego' / 'config' / 'ego.yaml')
                         .read_text())['ego']['ros__parameters']
    launch = set(vehicle.track_params(nav2, cfg, ego)) | TRACK_EXTRA
    both = sorted(flat(own) & launch)
    assert not both, f'config/track.yaml sets what the launch sets (the file wins): {both}'


def test_mpc_lag_rs_yaml_sets_nothing_the_launch_sets_and_races_on_the_pythons_gains():
    own = yaml.safe_load((ROS / 'drivers' / 'aav_mpc_lag' / 'config' / 'mpc_lag_rs.yaml')
                         .read_text())['mpc_lag_rs']['ros__parameters']
    nav2 = yaml.safe_load((PACKAGE / 'config' / 'nav2.yaml').read_text())
    cfg = vehicle.load(ROS / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml')
    ego = yaml.safe_load((ROS / 'ego' / 'aav_ego' / 'config' / 'ego.yaml')
                         .read_text())['ego']['ros__parameters']
    extra = {'use_sim_time', 'map_correction'}
    launch = set(vehicle.mpc_lag_rs_params(nav2, cfg, ego)) | extra
    both = sorted(flat(own) & launch)
    assert not both, f'config/mpc_lag_rs.yaml sets what the launch sets (the file wins): {both}'
    # A challenger beside the Python mpc_lag: the same gains, the same rates.
    track = yaml.safe_load((ROS / 'drivers' / 'aav_track' / 'config' / 'track.yaml')
                           .read_text())['track']['ros__parameters']
    assert own['mpc_lag'] == track['mpc_lag']
    for key in ('lateral_accel', 'odom_timeout', 'map_correction_speed',
                'map_correction_turn_rate', 'map_correction_snap_distance',
                'map_correction_snap_angle'):
        assert own[key] == track[key], key
