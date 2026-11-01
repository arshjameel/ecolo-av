"""Geofence: the fence's shapes, the stopping distance, and when the lock goes on and off."""

import math
import os

from aav_safety.geofence import (along_arc, backend_stale, braking, Circle, COASTING, coasts,
                                 Geofence, JUMP, LOST, Motion, motion_from, NEAR, OK, OUTSIDE,
                                 Polygon, Pose, reaction_time, Settings, site_fence, STANDS,
                                 STARTING, stopping_distance)
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROS = os.path.join(HERE, '..', '..', '..')

# A 2 m x 1 m car, base_link 0.5 m from the back.
CAR = [(1.5, 0.5), (1.5, -0.5), (-0.5, -0.5), (-0.5, 0.5)]
SQUARE = Polygon([(-10, -10), (10, -10), (10, 10), (-10, 10)])
# Simple numbers: rho 0.5 s, b 1 m/s^2, no speeding up, no turning.
STRAIGHT = Motion(reaction_time=0.5, braking=1.0, max_accel=0.0, max_speed=1.0,
                  max_reverse_speed=0.5, max_curvature=0.0)


def settled(fence=SQUARE, motion=STRAIGHT, x=0.0, y=0.0, yaw=0.0, **kw):
    """Return a geofence that has had 4 s of good poses (and GPS) at (x, y), still."""
    g = Geofence(CAR, fence, motion, **kw)
    for i in range(41):
        g.update(i * 0.1, Pose(x, y, yaw, 0.0, 0.2))
        g.gps_arrived(i * 0.1)
    return g


def at(g, t, x, y=0.0, yaw=0.0, v=0.0, std=0.2):
    g.update(t, Pose(x, y, yaw, v, std))
    g.gps_arrived(t)
    return g.check(t)[:2]


def load(*path):
    with open(os.path.join(ROS, *path)) as f:
        return yaml.safe_load(f)


# --- the fence -----------------------------------------------------------------------

def test_circle_and_polygon_depths():
    c = Circle(1.0, 2.0, 10.0)
    assert c.depth(1.0, 2.0) == pytest.approx(10.0)
    assert c.depth(11.0, 2.0) == pytest.approx(0.0)
    assert c.depth(1.0, 14.0) == pytest.approx(-2.0)
    assert SQUARE.depth(0.0, 0.0) == pytest.approx(10.0)
    assert SQUARE.depth(9.0, 0.0) == pytest.approx(1.0)
    assert SQUARE.depth(12.0, 0.0) == pytest.approx(-2.0)
    assert SQUARE.depth(13.0, 14.0) == pytest.approx(-5.0)     # past a corner: 3-4-5
    backwards = Polygon.from_flat([-10, 10, 10, 10, 10, -10, -10, -10])
    assert backwards.depth(9.0, 0.0) == pytest.approx(1.0)


def test_an_l_shaped_lot():
    ell = Polygon([(0, 0), (20, 0), (20, 10), (10, 10), (10, 20), (0, 20)])
    assert ell.depth(15.0, 15.0) < 0.0          # the missing corner is outside
    assert ell.depth(5.0, 15.0) == pytest.approx(5.0)


def test_bad_fences_are_refused():
    for bad in ([(0, 0), (1, 1)], [(0, 0), (1, 1), (2, 2)], [(0, 0), (1, 0), (float('nan'), 1)]):
        with pytest.raises(ValueError):
            Polygon(bad)
    with pytest.raises(ValueError):
        Polygon.from_flat([0, 0, 1, 0, 1])
    for r in (0.0, -1.0, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            Circle(0.0, 0.0, r)
    with pytest.raises(ValueError):
        Settings(max_std=0.0)
    with pytest.raises(ValueError):
        Settings(margin=float('nan'))
    with pytest.raises(ValueError):
        Motion(0.5, 0.0, 1.0, 1.0, 0.5, 0.3)


def test_a_site_needs_the_datum_it_was_drawn_for():
    corners = [0, 0, 20, 0, 20, 20, 0, 20]
    fence = site_fence('lot', corners, [45.386, -75.696], [45.386, -75.696, 0.0])
    assert fence.depth(10, 10) == pytest.approx(10.0)
    with pytest.raises(ValueError, match='datum'):
        site_fence('lot', corners, [45.387, -75.696], [45.386, -75.696, 0.0])
    with pytest.raises(ValueError, match='datum'):
        site_fence('lot', corners, [], [45.386, -75.696, 0.0])


def test_the_example_site_matches_navsat_yaml():
    cfg = load('safety', 'aav_safety', 'config', 'geofence.yaml')['geofence']['ros__parameters']
    datum = load('localization', 'aav_localization', 'config',
                 'navsat.yaml')['navsat_transform']['ros__parameters']['datum']
    for name, site in cfg['sites'].items():
        assert site_fence(name, site['corners'], site['datum'], datum).depth(0.0, 0.0) > 0.0


# --- stopping ------------------------------------------------------------------------

def test_stopping_distance_is_rss_with_the_cap():
    assert stopping_distance(0.0, 0.6, 0.0, 1.0) == 0.0
    assert stopping_distance(1.0, 0.6, 0.0, 1.0) == pytest.approx(1.1)
    assert stopping_distance(-2.0, 0.5, 0.0, 2.0) == pytest.approx(2.0)
    # Speeding up at 1 m/s^2 through rho = 1 s, from 0: 0.5 m, then 1 m/s to stop: 0.5 m.
    assert stopping_distance(0.0, 1.0, 1.0, 1.0) == pytest.approx(1.0)
    # The same capped at 0.5 m/s: 0.125 m to reach it, 0.25 m at it, 0.125 m to stop.
    assert stopping_distance(0.0, 1.0, 1.0, 1.0, cap=0.5) == pytest.approx(0.5)
    # Already at the cap: no speeding up.
    assert stopping_distance(1.0, 1.0, 1.0, 1.0, cap=1.0) == pytest.approx(1.5)


def test_rho_and_b_from_their_parts():
    assert reaction_time(0.1, 0.3, 0.2, 0.2, 0.1) == pytest.approx(0.9)
    assert reaction_time(0.1, 0.3, 0.2, 0.2, 0.1, brake_dead_time=0.05) == pytest.approx(0.95)
    with pytest.raises(ValueError):
        reaction_time(0.1, -0.3, 0.2, 0.2, 0.1)
    assert braking(2.0, 3.0) == 1.0
    assert braking(2.0, 1.0) == 0.5
    assert backend_stale('spine', spine={'cmd_timeout': 0.1}) == pytest.approx(0.2)
    assert backend_stale('uart', legacy={'cmd_timeout': 0.3}) == 0.3
    assert backend_stale('gazebo') == 0.2
    with pytest.raises(ValueError):
        backend_stale('teleport')


def test_the_open_speed_loop_coasts_until_measured():
    assert coasts('uart', {'speed_loop.mode': 'open'}, {})
    assert not coasts('uart', {'speed_loop.mode': 'pi'}, {})
    assert not coasts('uart', {'speed_loop.mode': 'open'}, {'open_loop_decel': 0.8})
    assert not coasts('spine', None, {})
    assert not coasts('gazebo_esp32', {'speed_loop.mode': 'open'}, {})


def test_motion_from_the_real_configs():
    ego = load('ego', 'aav_ego', 'config', 'ego.yaml')['ego']['ros__parameters']
    car = load('bringup', 'aav_description', 'config', 'vehicle.yaml')
    spine = load('can_bridge', 'aav_spine', 'config', 'spine.yaml')['spine_bridge'][
        'ros__parameters']
    legacy = load('can_bridge', 'aav_legacy', 'config', 'legacy.yaml')['legacy_uart_bridge'][
        'ros__parameters']
    motion, coasting = motion_from(ego, car, 'spine', 10.0, 0.1, spine=spine)
    assert not coasting
    act = car['actuators']
    expected = (0.1 + ego['cmd_timeout'] + spine['cmd_timeout'] + 0.1 + act['brake_lag']
                + 0.1)
    assert motion.reaction_time == pytest.approx(expected)
    assert motion.braking == min(ego['max_deceleration'], act['max_brake']) / 2
    assert motion.max_speed == ego['max_speed']
    assert motion_from(ego, car, 'spine', 10.0, 0.1, spine=spine, max_speed=0.3)[0].max_speed \
        == 0.3
    # Today's car: the 2025-26 ESP32 in open mode coasts.
    assert motion_from(ego, car, 'uart', 10.0, 0.1, legacy=legacy)[1] == (
        legacy['speed_loop.mode'] == 'open' and not act.get('open_loop_decel'))


def test_along_an_arc():
    assert along_arc(0, 0, 0, 2, 0) == pytest.approx((2, 0, 0))
    # A quarter circle of radius 2, left.
    x, y, yaw = along_arc(0, 0, 0, math.pi, 0.5)
    assert (x, y, yaw) == pytest.approx((2, 2, math.pi / 2))
    x, y, _ = along_arc(0, 0, 0, -math.pi, 0.5)
    assert (x, y) == pytest.approx((-2, 2))


# --- the lock ------------------------------------------------------------------------

def test_locked_until_settled_with_gps():
    g = Geofence(CAR, SQUARE, STRAIGHT)
    assert g.check(0.0)[:2] == (True, STARTING)
    g.update(0.0, Pose(0, 0, 0, 0, 0.2))
    assert g.check(0.0)[:2] == (True, LOST)        # no GPS into the EKF yet
    g.gps_arrived(0.0)
    assert g.check(1.0)[:2] == (True, STARTING)
    g.update(3.0, Pose(0, 0, 0, 0, 0.2))
    g.gps_arrived(3.0)
    assert g.check(3.0)[:2] == (False, OK)


def test_lost_when_stale_unsure_nan_or_without_gps():
    g = settled()
    assert not g.check(4.0)[0]
    assert g.check(5.5)[:2] == (True, LOST)                   # 1.5 s without a pose
    assert at(g, 6.0, 0, std=5.0) == (True, LOST)            # 5 m unsure
    assert at(g, 6.1, float('nan')) == (True, LOST)
    for i in range(32):                                       # 3 s of good poses again
        at(g, 6.2 + i * 0.1, 0)
    assert g.check(9.3)[:2] == (False, OK)
    # The EKF goes on from the wheels and the IMU, but the GPS has stopped: it drifts.
    for i in range(30):
        g.update(9.4 + i * 0.1, Pose(0, 0, 0, 0, 0.2))
    assert g.check(12.3)[:2] == (True, LOST)
    assert 'GPS' in g.why
    # Without a GPS (on stands), the GPS doesn't count.
    nogps = Geofence(CAR, SQUARE, STRAIGHT, gps=False)
    for i in range(31):
        nogps.update(i * 0.1, Pose(0, 0, 0, 0, 0.2))
    assert nogps.check(3.05)[:2] == (False, OK)


def test_a_jump_latches_until_cleared():
    g = settled()
    assert at(g, 4.1, 0.05) == (False, OK)
    assert at(g, 4.2, 4.0) == (True, JUMP)          # 4 m in 0.1 s, standing still
    assert at(g, 4.3, 4.0) == (True, JUMP)
    assert g.clear(4.3)[0]
    assert at(g, 4.4, 4.0) == (False, OK)
    # Moving fast, a long step is fine.
    g = settled()
    assert at(g, 4.1, 0.0, v=1.0) == (False, OK)
    assert at(g, 5.1, 1.0, v=1.0) == (False, OK)


def test_the_faster_the_car_the_earlier_it_stops():
    # At x = 5 facing east: the front corners at 6.5, 3.5 m from the edge.
    slow = settled(x=5.0)
    assert at(slow, 4.1, 5.0, v=0.5) == (False, OK)    # 0.375 m on
    fast = settled(x=5.0, motion=Motion(0.5, 1.0, 0.0, 3.0, 0.5, 0.0))
    assert at(fast, 4.1, 5.0, v=2.5) == (True, NEAR)   # 1.25 + 3.1 m on


def test_it_looks_back_whichever_way_the_car_moves():
    # Rear corners at -6.5 + ... : 3.5 m from the edge behind at x = -6.0.
    g = settled(x=-6.0, motion=Motion(0.5, 1.0, 0.0, 3.0, 3.0, 0.0))
    assert at(g, 4.1, -6.0, v=2.5) == (True, NEAR)     # forward fast: the back path counts too


def test_a_full_lock_arc_counts():
    # At y = 6 facing east, 1.5 m right of the north edge's margin... a straight stop is
    # fine, but turning left at full lock the front corner would reach the edge.
    straight = Motion(1.0, 1.0, 0.0, 2.0, 0.5, 0.0)
    turning = Motion(1.0, 1.0, 0.0, 2.0, 0.5, 0.35)
    assert at(settled(y=6.0, motion=straight), 4.1, 0.0, y=6.0, v=2.0) == (False, OK)
    assert at(settled(y=6.0, motion=turning), 4.1, 0.0, y=6.0, v=2.0) == (True, NEAR)


def test_the_spread_widens_the_margin():
    g = settled(x=6.5)                       # front corners 2.0 m from the edge
    assert at(g, 4.1, 6.5, std=0.5) == (False, OK)       # needs 1.0 + 0.5
    assert at(g, 4.2, 6.5, std=1.5) == (True, NEAR)      # needs 1.0 + 1.5


def test_near_lets_go_only_further_in_and_keeps_its_spread():
    g = settled(x=7.4)          # front 1.1 m from the edge: needs 1.2
    assert at(g, 4.1, 7.4) == (True, NEAR)
    assert at(g, 4.2, 7.2) == (True, NEAR)      # 1.3 m: past 1.2, not past 1.7
    assert at(g, 4.3, 6.7) == (False, OK)       # 1.8 m
    # A spread that shrinks while NEAR doesn't let it go early.
    g = settled(x=6.5)
    assert at(g, 4.1, 6.5, std=1.5) == (True, NEAR)
    assert at(g, 4.2, 6.5, std=0.1) == (True, NEAR)      # still needs 1.0 + 1.5 + 0.5


def test_outside_latches_until_a_person_clears_it_inside():
    g = settled()
    assert at(g, 4.1, 0.5) == (False, OK)
    g.settings.jump_margin = 100.0               # not a jump here: a long drive, in short
    assert at(g, 4.2, 11.0) == (True, OUTSIDE)
    assert g.clear(4.2)[0] is False             # still outside
    assert at(g, 4.3, 0.0) == (True, OUTSIDE)   # driven back in: still latched
    assert 'clears' in g.why
    assert g.clear(4.3)[0] is True
    assert g.check(4.3)[:2] == (False, OK)
    assert g.clear(4.4) == (True, 'nothing to clear')


def test_a_restart_finds_the_car_outside_again():
    # The fence is fixed (the start spot, or a site), so a new node outside says so.
    g = Geofence(CAR, SQUARE, STRAIGHT)
    for i in range(41):
        at(g, i * 0.1, 15.0)
    assert g.check(4.0)[:2] == (True, OUTSIDE)


def test_clear_needs_a_good_pose():
    g = settled()
    g.settings.jump_margin = 100.0
    at(g, 4.1, 11.0)
    g.update(4.2, Pose(0.0, 0.0, 0.0, 0.0, 9.0))
    assert g.clear(4.2)[0] is False


def test_coasting_holds_and_stands_opens_loudly():
    assert settled(coasting=True).check(4.0)[:2] == (True, COASTING)
    stands = Geofence(CAR, SQUARE, STRAIGHT, stands=True)
    locked, state, why = stands.check(0.0)
    assert (locked, state) == (False, STANDS) and 'NO FENCE' in why
