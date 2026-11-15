"""The held map -> odom (TODO 186): the same rule as aav_mppi_lag's map_correction.rs."""

import math

from aav_track.path import moved, Path
from aav_track.slow_frame import into_map, into_odom, SlewLimits, SlowFrame


def close(a, b, eps=1e-9):
    return all(abs(x - y) < eps for x, y in zip(a, b))


def test_into_map_undoes_into_odom():
    odom = (3.0, -2.0, 0.7)
    p = (10.0, 4.0, -2.5)
    assert close(into_map(into_odom(p, odom), odom), p)


def test_off_takes_the_newest_for_each_path_and_never_moves():
    f = SlowFrame(SlewLimits(speed=0.0))
    a, b = (0.0, 0.1, 0.0), (0.0, -0.1, 0.0)
    assert f.for_new_path(a, True) == a
    assert f.update(b, (0.0, 0.0, 0.0), 0.05) is None
    assert f.held == a
    assert f.for_new_path(b, True) == b


def test_a_new_path_while_driving_lands_by_the_held_one():
    f = SlowFrame()
    a = (0.0, 0.0, 0.0)
    f.for_new_path(a, False)
    jumped = (0.0, 0.09, 0.0)
    assert f.for_new_path(jumped, True) == a
    assert f.for_new_path(jumped, False) == jumped


def test_a_sideways_step_is_taken_at_the_speed_limit():
    f = SlowFrame()
    f.for_new_path((0.0, 0.0, 0.0), False)
    car, newest = (40.0, 0.0, 0.0), (0.0, 0.5, 0.0)
    t = 0.0
    while abs(f.held[1] - 0.5) > 1e-9:
        assert f.update(newest, car, 0.05) == 'slewed'
        t += 0.05
        assert t < 10.0
    assert abs(t - 5.0) < 0.051            # 0.5 m at 0.1 m/s
    assert f.update(newest, car, 0.05) is None


def test_a_turn_is_slewed_about_the_car():
    f = SlowFrame()
    f.for_new_path((0.0, 0.0, 0.0), False)
    car = (100.0, 0.0, 0.0)
    a = math.radians(2.0)
    newest = (100.0 - 100.0 * math.cos(a), -100.0 * math.sin(a), a)
    assert f.update(newest, car, 0.1) == 'slewed'
    at = into_map(car, f.held)
    assert abs(at[0] - 100.0) < 1e-9 and abs(at[1]) < 1e-9
    assert abs(f.held[2] - math.radians(0.1)) < 1e-12


def test_a_relocalisation_is_taken_at_once():
    f = SlowFrame()
    f.for_new_path((0.0, 0.0, 0.0), False)
    assert f.update((3.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05) == 'snapped'
    assert f.held == (3.0, 0.0, 0.0)
    assert f.update((3.0, 0.0, 0.3), (0.0, 0.0, 0.0), 0.05) == 'snapped'


def test_jumps_are_held_to_the_rate():
    f = SlowFrame()
    f.for_new_path((0.0, 0.0, 0.0), False)
    worst = 0.0
    for i in range(200):
        side = 0.045 if (i // 4) % 2 == 0 else -0.045
        before = f.held
        f.update((0.0, side, 0.0), (5.0, 0.0, 0.0), 0.05)
        assert math.hypot(f.held[0] - before[0], f.held[1] - before[1]) <= 0.005 + 1e-12
        worst = max(worst, abs(f.held[1]))
    assert worst < 0.03
    assert f.gap((0.0, 0.045, 0.0), (5.0, 0.0, 0.0)) < 0.08


def test_a_bad_number_changes_nothing():
    f = SlowFrame()
    f.for_new_path((0.0, 0.0, 0.0), False)
    assert f.update((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05) is None
    assert f.held == (0.0, 0.0, 0.0)


def test_the_car_in_map_is_the_path_in_odom():
    """The node keeps a map path in map and brings the car there: the same cross track."""
    odom_in_map = (2.0, -1.0, 0.4)
    poses = [(float(x), 0.5 * x, math.atan(0.5)) for x in range(20)]
    car_odom = (4.0, -3.0, 0.1)
    # The old way: the path into odom (path.moved takes map's pose in odom).
    ox, oy, oyaw = into_odom((0.0, 0.0, 0.0), odom_in_map)
    in_odom = Path(moved(poses, (ox, oy, oyaw)))
    in_map = Path(poses)
    car_map = into_map(car_odom, odom_in_map)
    a = in_odom.project(car_odom[:2], 0)
    b = in_map.project(car_map[:2], 0)
    assert abs(a.cross - b.cross) < 1e-9 and abs(a.s - b.s) < 1e-9
