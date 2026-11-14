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
LAUNCH_EXTRA = {'use_sim_time', 'costmap_topic'}


def test_mppi_lag_yaml_sets_nothing_the_launch_sets():
    own = yaml.safe_load((ROS / 'drivers' / 'aav_mppi_lag' / 'config' / 'mppi_lag.yaml')
                         .read_text())['mppi_lag']['ros__parameters']
    nav2 = yaml.safe_load((PACKAGE / 'config' / 'nav2.yaml').read_text())
    cfg = vehicle.load(ROS / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml')
    ego = yaml.safe_load((ROS / 'ego' / 'aav_ego' / 'config' / 'ego.yaml')
                         .read_text())['ego']['ros__parameters']
    launch = set(vehicle.mppi_lag_params(nav2, cfg, ego)) | LAUNCH_EXTRA
    both = sorted(set(own) & launch)
    assert not both, f'config/mppi_lag.yaml sets what the launch sets (the file wins): {both}'
