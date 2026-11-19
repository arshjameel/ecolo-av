"""The lane follower's logic (scripts/lane_follow.py): no ROS needed."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
RUST = PACKAGE.parents[1] / 'drivers' / 'aav_lane_follower_rs'
sys.path[:0] = [str(PACKAGE / 'scripts'), str(RUST)]

from lane_follow import (LANE_CHANGING, LANE_KEEP, LaneFollower,  # noqa: E402, I100
                         NO_LANE_STOPPING, RESUMING, Settings)


def lane(offset=0.0, seen=True, both=True, radius=9999.0):
    return SimpleNamespace(vehicle_offset_m=offset, mean_curvature_m=radius,
                           left_lane_detected=seen, right_lane_detected=seen and both,
                           both_lanes_detected=seen and both)


def test_steers_back_towards_the_centre():
    f = LaneFollower(Settings(), 0.0, 0.0)
    reply = f.on_geometry(lane(0.2), 0.1, 0.1)
    (speed, turn), = reply.commands
    assert speed == 2.0 and turn < 0.0
    assert reply.state == LANE_KEEP
    assert reply.status.startswith('[LANE_KEEP] offset=+0.200m curv=9999.0m speed=2.00m/s')


def test_one_line_and_a_bend_slow_it():
    f = LaneFollower(Settings(), 0.0, 0.0)
    (speed, _), = f.on_geometry(lane(0.0, both=False, radius=10.0), 0.1, 0.1).commands
    assert speed == pytest.approx(2.0 * 0.7 * 0.5)


def test_a_fast_big_jump_is_a_lane_change():
    f = LaneFollower(Settings(), 0.0, 0.0)
    f.on_geometry(lane(0.0), 0.1, 0.1)
    reply = f.on_geometry(lane(1.5), 0.2, 0.2)
    assert reply.transitions == [(LANE_KEEP, LANE_CHANGING)] and not reply.commands


def test_lost_lines_slow_down_to_a_stop_then_resume():
    f = LaneFollower(Settings(), 0.0, 0.0)
    f.on_geometry(lane(0.0), 0.1, 0.1)
    f.on_geometry(lane(seen=False), 0.2, 0.2)
    assert f.state == NO_LANE_STOPPING
    speeds, t = [], 0.2
    while t < 6.0:
        t += 0.1
        f.on_geometry(lane(seen=False), t, t)
        speeds += [v for v, _ in f.tick(t, t).commands]
    assert speeds == sorted(speeds, reverse=True) and speeds[-1] == 0.0
    assert f.on_geometry(lane(0.0), t + 0.1, t + 0.1).transitions == [
        (NO_LANE_STOPPING, RESUMING)]


def test_the_watchdog_trips_on_the_node_clock():
    f = LaneFollower(Settings(), 0.0, 100.0)
    f.on_geometry(lane(0.0), 0.1, 100.1)
    assert f.tick(50.0, 100.2).quiet_for is None      # the wall clock doesn't count
    reply = f.tick(0.2, 100.5)
    assert reply.quiet_for == pytest.approx(0.4) and f.state == NO_LANE_STOPPING


def test_the_rust_vectors_are_up_to_date():
    """Check aav_lane_follower_rs's vectors.json is still these answers."""
    import gen_vectors
    want = json.loads(gen_vectors.OUT.read_text())
    assert json.loads(json.dumps(gen_vectors.build())) == want, (
        'run: pixi run python ros/drivers/aav_lane_follower_rs/gen_vectors.py')
