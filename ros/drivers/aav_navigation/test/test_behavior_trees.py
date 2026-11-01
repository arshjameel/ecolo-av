"""
Check the car's behaviour trees: forward-only and reversing twins, and no spinning.

Each navigator has two trees: *_car.xml (forward only) and *_car_reversing.xml (with
BackUp). They must stay the same apart from BackUp, and BackUp must stay inside the car's
reverse speed (vehicle.yaml). Near the goal the to-pose trees keep their path (TODO 17).
No ROS needed.
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
# aav_description's vehicle.py (no ROS), from the source tree: tools/test.sh runs these
# tests without the workspace.
sys.path.insert(0, str(PACKAGE.parents[1] / 'bringup' / 'aav_description'))
from aav_description import vehicle  # noqa: E402, I100
TREES = PACKAGE / 'behavior_trees'
VEHICLE = PACKAGE.parents[1] / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml'
NAV2 = PACKAGE / 'config' / 'nav2.yaml'
NAVIGATORS = ['navigate_to_pose', 'navigate_through_poses']


def tree(name):
    return ET.parse(TREES / name).getroot()


def shape(element):
    """Return the tree as nested tuples: tag, attributes, children (comments dropped)."""
    return (element.tag, sorted(element.attrib.items()),
            [shape(child) for child in element])


def without_backup(element):
    for parent in element.iter():
        for child in list(parent):
            if child.tag == 'BackUp':
                parent.remove(child)
    return element


@pytest.mark.parametrize('navigator', NAVIGATORS)
def test_the_twins_differ_only_by_backup(navigator):
    forward = tree(f'{navigator}_car.xml')
    reversing = tree(f'{navigator}_car_reversing.xml')
    assert not list(forward.iter('BackUp'))
    assert len(list(reversing.iter('BackUp'))) == 1
    assert shape(without_backup(reversing)) == shape(forward)


@pytest.mark.parametrize('navigator', NAVIGATORS)
def test_backing_up_stays_inside_the_reverse_speed(navigator):
    limit = yaml.safe_load(VEHICLE.read_text())['vehicle']['max_reverse_speed']
    (backup,) = tree(f'{navigator}_car_reversing.xml').iter('BackUp')
    assert 0.0 < float(backup.get('backup_speed')) <= limit
    assert 0.0 < float(backup.get('backup_dist')) <= 2.0


@pytest.mark.parametrize('name', sorted(p.name for p in TREES.glob('*.xml')))
def test_no_tree_spins(name):
    # A car can't turn on the spot.
    assert not list(tree(name).iter('Spin'))


def test_the_behavior_server_can_back_up():
    plugins = yaml.safe_load(NAV2.read_text())['behavior_server']['ros__parameters']
    assert 'backup' in plugins['behavior_plugins']
    assert 'spin' not in plugins['behavior_plugins']


TO_POSE = ['navigate_to_pose_car.xml', 'navigate_to_pose_car_reversing.xml']


def final_approach_guard(name):
    (guard,) = [f for r in tree(name).iter('RateController') for f in r
                if f.tag == 'Fallback' and f.get('name') == 'PlanUnlessOnTheFinalApproach']
    return guard


@pytest.mark.parametrize('name', TO_POSE)
def test_no_replanning_on_the_final_approach(name):
    # TODO 17: near the goal a new path was a loop for a car, and each new path reset
    # controller_server's goal checker. The tree keeps its path there, unless it is
    # blocked, for another goal, or old. The RateController stays outside the Fallback:
    # inside it, the Fallback would restart it, and it would plan on every tick.
    guard = final_approach_guard(name)
    assert not list(guard.iter('RateController'))
    keep, plan = list(guard)
    assert keep.tag == 'ReactiveSequence'
    assert list(keep.iter('IsPathValid'))          # a blocked path is planned again
    assert list(keep.iter('PathExpiringTimer'))    # and an old one
    assert list(plan.iter('ComputePathToPose'))
    near = {n.get('ref_pose'): n for n in keep.iter('ArePosesNear')}
    # The path must still end at this goal: a new goal gets a new path.
    assert float(near['{path_end}'].get('tolerance')) < 1.0
    # The zone covers the worst S-bend back onto the line (vehicle.final_approach), and
    # is wider than the goal checker's circle.
    zone = float(near['{current_pose}'].get('tolerance'))
    v = yaml.safe_load(VEHICLE.read_text())['vehicle']
    nav2 = yaml.safe_load(NAV2.read_text())['controller_server']['ros__parameters']
    assert vehicle.final_approach(v) <= zone <= 1.5 * vehicle.final_approach(v)
    assert zone >= 3.0 * nav2['general_goal_checker']['xy_goal_tolerance']


@pytest.mark.parametrize('name', TO_POSE)
def test_the_map_window_waits_longer_than_nav2_keeps_a_path(name):
    # The map window shows NO NEW PLAN when /plan gets old. On the final approach it must
    # wait longer than the tree keeps a path, over the same zone, or it cries wolf.
    keep = final_approach_guard(name)[0]
    (timer,) = keep.iter('PathExpiringTimer')
    zone = next(n for n in keep.iter('ArePosesNear') if n.get('ref_pose') == '{current_pose}')
    window = PACKAGE.parents[1] / 'hud' / 'aav_hud' / 'config' / 'map.yaml'
    params = yaml.safe_load(window.read_text())['map']['ros__parameters']
    assert params['plan_kept_for'] > float(timer.get('seconds'))
    assert params['final_approach'] == float(zone.get('tolerance'))
