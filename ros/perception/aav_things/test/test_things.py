"""Tests for the grouping and the tracker (no ROS)."""

import math

from aav_things.groups import Group, group, GroupShape, to_frame
from aav_things.tracker import (accel_covariance, fitted_speed,  # noqa: I100
                                ground_radial_speed, predict, RadarCluster, Tracker,
                                TrackerShape, UNKNOWN)
import numpy as np
import pytest


def box(x, y, length=1.0, width=1.0, height=1.0, size=0.25):
    """Return the cube centres aav_voxels would send for an upright box."""
    xs = np.arange(x - length / 2 + size / 2, x + length / 2, size)
    ys = np.arange(y - width / 2 + size / 2, y + width / 2, size)
    zs = np.arange(0.3 + size / 2, height, size)
    return np.array([(a, b, c) for a in xs for b in ys for c in zs])


def seen(x, y, length=1.0, width=1.0):
    return Group(x=x, y=y, length=length, width=width, height=1.0, cubes=16)


# --- grouping ---

def test_two_boxes_apart_are_two_groups():
    found = group(np.vstack([box(5.0, 0.0), box(5.0, 3.0)]))
    assert len(found) == 2
    middles = sorted((round(g.x, 2), round(g.y, 2)) for g in found)
    assert middles == [(5.0, 0.0), (5.0, 3.0)]
    assert all(g.length == pytest.approx(1.0) and g.width == pytest.approx(1.0) for g in found)


def test_boxes_closer_than_the_gap_are_one():
    # 0.25 m between their sides: the nearest cubes' centres are 0.5 m apart.
    found = group(np.vstack([box(5.0, 0.0), box(5.0, 1.25)]), GroupShape(gap=0.6))
    assert len(found) == 1
    assert found[0].width == pytest.approx(2.25)
    # 0.5 m between their sides (centres 0.75 m apart): two.
    assert len(group(np.vstack([box(5.0, 0.0), box(5.0, 1.5)]), GroupShape(gap=0.6))) == 2


def test_a_lone_cube_is_noise_and_a_wall_is_not_a_thing():
    wall = box(10.0, 0.0, length=0.25, width=20.0)
    lone = np.array([[3.0, 3.0, 0.5]])
    assert group(np.vstack([wall, lone])) == []


def test_nothing_and_nans_give_nothing():
    assert group(np.zeros((0, 3))) == []
    assert group(np.array([[np.nan, 0.0, 0.5], [1.0, np.inf, 0.5]])) == []


def test_to_frame_turns_the_middle_and_the_outline():
    g = Group(x=2.0, y=0.0, length=2.0, width=1.0, height=1.0, cubes=4)
    (moved,) = to_frame([g], 10.0, 5.0, math.pi / 2)
    assert (moved.x, moved.y) == pytest.approx((10.0, 7.0))
    assert (moved.length, moved.width) == pytest.approx((1.0, 2.0))


# --- the filter ---

def test_predict_moves_on_its_speed_and_grows_the_doubt():
    x, p = predict(np.array([0.0, 0.0, 1.0, -2.0]), np.eye(4) * 0.01, 0.5, 1.0)
    assert x == pytest.approx([0.5, -1.0, 1.0, -2.0])
    assert p[0, 0] > 0.01 and p[2, 2] > 0.01


def test_a_moving_thing_gets_its_speed():
    rng = np.random.default_rng(1)
    tracker = Tracker()
    for k in range(40):              # 4 s at 10 Hz, 1.2 m/s along x, 0.4 m/s along y
        t = k * 0.1
        noise = rng.normal(0.0, 0.1, 2)
        tracker.step(t, [seen(5.0 + 1.2 * t + noise[0], -1.0 + 0.4 * t + noise[1])], (0, 0))
    (thing,) = tracker.at(3.9)
    assert thing.x[2:] == pytest.approx([1.2, 0.4], abs=0.15)
    assert thing.id == 1 and thing.seen == 40 and thing.kind == UNKNOWN
    # Between clouds the list moves on: 0.5 s later, 0.6 m further along x.
    (later,) = tracker.at(4.4)
    assert later.x[0] - thing.x[0] == pytest.approx(0.6, abs=0.1)


def test_two_things_keep_their_ids():
    tracker = Tracker()
    for k in range(30):
        t = k * 0.1
        tracker.step(t, [seen(10.0 - t, 2.0), seen(10.0 + t, -2.0)], (0, 0))
    things = {th.id: th for th in tracker.at(2.9)}
    assert len(things) == 2
    assert things[1].x[1] == pytest.approx(2.0, abs=0.1)
    assert things[1].x[2] == pytest.approx(-1.0, abs=0.15)
    assert things[2].x[2] == pytest.approx(1.0, abs=0.15)


def test_a_flash_is_never_listed_and_soon_forgotten():
    tracker = Tracker()
    tracker.step(0.0, [seen(5.0, 0.0)], (0, 0))
    assert tracker.at(0.0) == []            # seen once: not confirmed
    tracker.step(0.5, [], (0, 0))
    assert tracker.tracks == []


def test_memory_far_away_lasts_keep_and_in_the_blind_zone_longer():
    shape = TrackerShape(keep=3.0, keep_blind=30.0, blind_range=4.0)
    tracker = Tracker(shape)
    for k in range(10):                    # one parked box 2 m ahead, one 15 m ahead
        tracker.step(k * 0.1, [seen(2.0, 0.0), seen(15.0, 5.0)], (0, 0))
    # Then the lidar sees neither (the near one is in the blind zone).
    for k in range(10, 60):
        tracker.step(k * 0.1, [], (0, 0))
    left = tracker.at(5.9)
    assert [round(th.x[0]) for th in left] == [2]
    assert left[0].coasting
    # It stayed where it was: parked things don't walk off on a noisy speed.
    assert left[0].x[:2] == pytest.approx([2.0, 0.0], abs=0.05)
    tracker.expire(32.0, (0, 0))
    assert tracker.tracks == []


def test_a_coasting_mover_keeps_moving():
    tracker = Tracker()
    for k in range(30):
        tracker.step(k * 0.1, [seen(10.0 + 2.0 * k * 0.1, 0.0)], (0, 0))
    tracker.step(3.5, [], (0, 0))           # gone from view
    (thing,) = tracker.at(4.0)
    assert thing.coasting
    assert thing.x[0] == pytest.approx(10.0 + 2.0 * 4.0, abs=0.3)


def test_an_older_cloud_is_dropped():
    tracker = Tracker()
    tracker.step(1.0, [seen(5.0, 0.0)], (0, 0))
    tracker.step(0.9, [seen(50.0, 0.0)], (0, 0))
    assert len(tracker.tracks) == 1 and tracker.time == 1.0


# --- the radar ---

def test_ground_radial_speed_takes_the_car_off():
    # The car at 2 m/s along x, a parked thing straight ahead: the radar reads -2 m/s.
    assert ground_radial_speed(-2.0, (1.0, 0.0), (2.0, 0.0)) == pytest.approx(0.0)
    # Off to the side at 60 degrees: only half the car's speed shows.
    ray = (0.5, math.sqrt(3) / 2)
    assert ground_radial_speed(-1.0, ray, (2.0, 0.0)) == pytest.approx(0.0)


def test_radar_speeds_up_a_new_things_speed():
    shape = TrackerShape()
    with_radar, without = Tracker(shape), Tracker(shape)
    for k in range(4):                     # a thing moving at 3 m/s away along x
        t = k * 0.1
        for tracker in (with_radar, without):
            tracker.step(t, [seen(10.0 + 3.0 * t, 0.0)], (0, 0))
        with_radar.radar(t, [RadarCluster(x=10.0 + 3.0 * t, y=0.0, ray_x=1.0, ray_y=0.0,
                                          speed=3.0)])
    (fast,), (slow,) = with_radar.at(0.3), without.at(0.3)
    assert fast.radar and not slow.radar
    assert abs(fast.x[2] - 3.0) < abs(slow.x[2] - 3.0)
    assert fast.x[2] == pytest.approx(3.0, abs=0.2)


def test_naming_hook():
    tracker = Tracker(name=lambda track: 'person' if track.height < 2.0 else UNKNOWN)
    for k in range(3):
        tracker.step(k * 0.1, [seen(5.0, 0.0)], (0, 0))
    assert tracker.at(0.2)[0].kind == 'person'


def _held_tracks(tracker, frames):
    """Return the ids that moved 0.3 m/s or more for 1.5 s on end (the behaviour's hold)."""
    since, held = {}, set()
    for now, groups, car in frames:
        tracker.step(now, groups, car)
        for t in tracker.at(now):
            if math.hypot(t.x[2], t.x[3]) >= 0.3:
                if now - since.setdefault(t.id, now) >= 1.5:
                    held.add(t.id)
            else:
                since.pop(t.id, None)
    return held


def test_a_row_of_thin_posts_never_moves():
    # TODO 222: posts 1.5 m apart, each missed a third of the time, the car passing at
    # 2 m/s. The tracker hopped post to post (13 m/s); the gates keep each post still.
    import random
    for seed in range(10):
        rng = random.Random(seed)
        posts = [(5.0 + 1.5 * i, 4.0) for i in range(12)]
        frames = []
        for k in range(200):
            now, car = 0.1 * k, (0.2 * k - 5.0, 0.0)
            groups = [Group(px + rng.uniform(-0.15, 0.15), py + rng.uniform(-0.15, 0.15),
                            0.25 + rng.choice([0, 0.25]), 0.25 + rng.choice([0, 0.25]), 1.0, 2)
                      for px, py in posts
                      if math.hypot(px - car[0], py - car[1]) <= 25 and rng.random() > 0.35]
            frames.append((now, groups, car))
        assert _held_tracks(Tracker(), frames) == set(), seed


def test_a_walker_and_a_car_still_get_their_speed():
    # The gates must not lose what really moves, seen most of the time.
    import random
    rng = random.Random(5)
    frames = []
    for k in range(60):
        now = 0.1 * k
        groups = []
        if rng.random() > 0.3:
            groups.append(Group(10.0, -5.0 + 1.4 * now, 0.5, 0.5, 1.7, 4))     # walking
        if rng.random() > 0.3:
            groups.append(Group(-20.0 + 8.0 * now, 8.0, 4.5, 2.0, 1.5, 30))    # a car
        frames.append((now, groups, (0.0, 0.0)))
    tracker = Tracker()
    assert len(_held_tracks(tracker, frames)) == 2
    speeds = sorted(math.hypot(t.x[2], t.x[3]) for t in tracker.at(6.0))
    assert speeds == pytest.approx([1.4, 8.0], abs=0.3)


# --- a motion model per kind (TODO 217) ---

def test_a_cars_noise_is_long_along_its_heading_and_narrow_across():
    sh = TrackerShape(per_kind=True)
    assert accel_covariance('car', 5.0, 0.0, sh) == pytest.approx((4.0, 0.0, 0.25))
    assert accel_covariance('truck', 3.0, 3.0, sh) == pytest.approx((2.125, 1.875, 2.125))
    assert accel_covariance('person', 1.0, 0.0, sh) == pytest.approx((4.0, 0.0, 4.0))
    assert accel_covariance('unknown', 1.0, 0.0, sh) == pytest.approx((2.25, 0.0, 2.25))


def test_a_coasting_car_stays_in_its_lane():
    tracker = Tracker(TrackerShape(per_kind=True), name=lambda t: 'car')
    for k in range(30):
        tracker.step(k * 0.1, [seen(10.0 + 5.0 * k * 0.1, 0.0)], (0, 0))
    (car,) = tracker.at(4.0)
    assert car.p[1, 1] < 0.5 * car.p[0, 0]


def test_a_thing_found_again_refits_its_speed_from_its_sightings():
    tracker = Tracker(TrackerShape(per_kind=True))
    for k in range(10):
        tracker.step(k * 0.1, [seen(10.0 + k * 0.1, 0.0)], (0, 0))
    tracker.step(2.0, [seen(11.0, 0.5)], (0, 0))
    (thing,) = tracker.tracks
    (vx, vy), _ = fitted_speed(thing.sightings, 0.15)
    assert thing.x[2] == pytest.approx(vx) and thing.x[3] == pytest.approx(vy)
    assert fitted_speed(thing.sightings[:2], 0.15) is None
