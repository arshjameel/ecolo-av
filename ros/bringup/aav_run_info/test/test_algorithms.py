"""Tests for algorithms.py: a run's choices in words, for the cockpit's table."""

from pathlib import Path
import sys

from aav_run_info.algorithms import Choices, rows, summary, use
import pytest

# aav_description's vehicle.py, from the source tree: the launch files' own lists.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'aav_description'))


def table(choices):
    return dict(rows(choices))


def test_the_default_sim_run():
    c = Choices(profile='sim world:=road', lidar=True, radar=True)
    t = table(c)
    assert t['run'] == 'sim world:=road'
    assert t['planner'].startswith('lattice')
    assert t['path follower'].startswith('MPPI')
    assert t['obstacles'] == 'lidar scan + radar'
    assert t['NDT'] == t['KISS-ICP'] == t['AMCL'] == 'off'
    assert summary(c) == 'lattice, mppi'


def test_the_rows_come_in_reading_order():
    assert [k for k, _ in rows(Choices())] == [
        'run', 'planner', 'path follower', 'obstacles', 'local EKF', 'global EKF', 'NDT',
        'KISS-ICP', 'AMCL']


def test_a_trial_tracker_takes_the_followers_place():
    c = Choices(tracker='lqr')
    assert table(c)['path follower'] == 'lqr (aav_track, trial)'
    assert summary(c) == 'lattice, lqr'


def test_voxels_replace_the_flat_scan():
    c = Choices(lidar=True, voxels=True)
    assert table(c)['obstacles'].startswith('voxels')
    assert 'voxels' in summary(c)


def test_voxels_without_a_lidar_are_refused():
    with pytest.raises(ValueError, match='lidar'):
        Choices(voxels=True)


def test_a_fused_localizer_shows_in_its_ekf():
    c = Choices(ndt=use(True, True), kiss_icp=use(True, False))
    t = table(c)
    assert 'NDT pose' in t['global EKF']
    assert 'KISS-ICP' not in t['local EKF']
    assert t['NDT'] == 'on, into the global EKF'
    assert t['KISS-ICP'].startswith('on, scored only')
    assert summary(c) == 'lattice, mppi, NDT fused, KISS-ICP scored'


def test_fusing_implies_running():
    assert use(False, True) == 'fused'
    assert use(True, False) == 'scored'
    assert use(False, False) == 'off'


def test_without_nav2_there_is_no_planner():
    c = Choices(nav=False)
    assert table(c)['planner'].startswith('off')
    assert table(c)['path follower'].startswith('off')
    assert summary(c) == 'no Nav2'


@pytest.mark.parametrize('field, value', [('planner', 'astar'), ('controller', 'dwb'),
                                          ('tracker', 'magic'), ('ndt', 'yes')])
def test_unknown_names_are_refused(field, value):
    with pytest.raises(ValueError, match=field):
        Choices(**{field: value})


def test_it_knows_every_planner_and_tracker_the_launch_does():
    # vehicle.py is the launch's list; this module must name each one.
    from aav_description import vehicle
    from aav_run_info import algorithms
    assert set(vehicle.PLANNERS) == set(algorithms.PLANNERS)
    assert set(vehicle.CONTROLLERS) == set(algorithms.CONTROLLERS)
    assert set(vehicle.TRACKERS) == set(algorithms.TRACKERS)
