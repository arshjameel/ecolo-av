import math
import random

from aav_bench import models
import pytest

DATUM = (45.3860, -75.6960)


def truth(x=0.0, y=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0, t=0.0):
    return models.Truth(t, x, y, yaw, speed, yaw_rate)


def test_offset_turns_with_the_car():
    assert models.offset(truth(), [0.8, 0.0, 1.5]) == pytest.approx((0.8, 0.0))
    north = models.offset(truth(x=10.0, yaw=math.pi / 2), [0.8, 0.0, 1.5])
    assert north == pytest.approx((10.0, 0.8))


def test_geo_from_map_matches_the_earths_size_at_ottawa():
    lat, lon = models.geo_from_map(0.0, 0.0, *DATUM)
    assert (lat, lon) == DATUM
    # At 45.386 degrees north, 1 degree of latitude is about 111.1 km; of longitude, 78.3 km.
    lat, lon = models.geo_from_map(1000.0, 1000.0, *DATUM)
    assert (lat - DATUM[0]) * 111_132 == pytest.approx(1000.0, rel=2e-3)
    assert (lon - DATUM[1]) * 78_313 == pytest.approx(1000.0, rel=1e-3)


def test_imu_reads_speeding_up_turning_and_gravity():
    accel, gyro = models.imu_reading(truth(speed=1.5, yaw_rate=0.2, t=1.0),
                                     truth(speed=1.0, t=0.5))
    assert accel == pytest.approx((1.0, 0.3, models.G))
    assert gyro == (0.0, 0.0, 0.2)
    accel, _ = models.imu_reading(truth(speed=1.0), None)
    assert accel[0] == 0.0


def test_magnetic_field_points_north():
    east = models.magnetic_field(0.0)        # facing east, north is on the left
    assert east[0] == pytest.approx(0.0) and east[1] > 0 and east[2] < 0
    north = models.magnetic_field(math.pi / 2)
    assert north[0] == pytest.approx(models.FIELD_NORTH_T) and north[1] == pytest.approx(0.0)


def test_noise_is_seeded_and_unbiased():
    a = models.noisy((1.0, 2.0, 3.0), 0.1, random.Random(5))
    assert a == models.noisy((1.0, 2.0, 3.0), 0.1, random.Random(5))
    rng = random.Random(7)
    mean = sum(models.noisy((0.0,), 0.1, rng)[0] for _ in range(5000)) / 5000
    assert abs(mean) < 0.01


def test_battery_drains_and_stops_at_empty():
    assert models.battery_voltage(0.0, 50.9, 46.0, 2.0) == 50.9
    assert models.battery_voltage(1.0, 50.9, 46.0, 2.0) == pytest.approx(48.45)
    assert models.battery_voltage(9.0, 50.9, 46.0, 2.0) == 46.0


CAM = models.Camera([1.8, 0.29, 1.2], 640, 360, 343.0, 1.2)


def test_a_sign_ahead_grows_as_the_car_gets_closer():
    far = models.sign_in_view(CAM, truth(x=0.0), (25.0, -2.0), 2.0, 0.75)
    near = models.sign_in_view(CAM, truth(x=18.0), (25.0, -2.0), 2.0, 0.75)
    assert far is not None and near is not None
    assert near.size > far.size
    assert near.size == pytest.approx(343.0 * 0.75 / (25.0 - 18.0 - 1.8), rel=1e-6)
    assert near.u > CAM.width / 2 and near.v < CAM.height / 2, 'right of centre, above it'


def test_signs_behind_or_too_far_aside_are_not_seen():
    assert models.sign_in_view(CAM, truth(x=30.0), (25.0, -2.0), 2.0, 0.75) is None
    assert models.sign_in_view(CAM, truth(x=24.0), (25.0, -2.0), 2.0, 0.75) is None


def test_pairs():
    assert models.pairs([1.0, 2.0, 3.0, 4.0]) == [(1.0, 2.0), (3.0, 4.0)]
    with pytest.raises(ValueError):
        models.pairs([1.0])


def test_a_camera_tilted_down_sees_the_sign_higher_up():
    level = models.sign_in_view(CAM, truth(x=10.0), (25.0, -2.0), 2.0, 0.75)
    tilted = models.Camera([1.8, 0.29, 1.2], 640, 360, 343.0, 1.2, pitch=0.1)
    down = models.sign_in_view(tilted, truth(x=10.0), (25.0, -2.0), 2.0, 0.75)
    assert down.v < level.v - 30.0
    # The post's foot is on the ground: height / distance below the horizon.
    assert level.foot_v == pytest.approx(180.0 + 343.0 * 1.2 / (25.0 - 10.0 - 1.8))


def test_the_road_runs_east_and_bends_where_asked():
    straight = models.Road(y=1.0)
    assert straight.lateral(50.0, 1.5) == pytest.approx(0.5)
    assert straight.pose(10.0, 0.7, 0.1) == pytest.approx((-0.3, 0.1))
    left = models.Road(bend_x=30.0, bend_radius=60.0)
    # A quarter of the way round, the road heads north; its inside (west) is on the left.
    assert left.lateral(90.0, 60.0) == pytest.approx(0.0)
    assert left.lateral(89.0, 60.0) == pytest.approx(1.0)
    assert left.pose(90.0, 60.0, math.pi / 2) == pytest.approx((0.0, 0.0))
    assert left.lateral(20.0, 0.3) == pytest.approx(0.3), 'straight before the bend'
    right = models.Road(bend_x=30.0, bend_radius=-60.0)
    assert right.lateral(89.0, -60.0) == pytest.approx(-1.0)
    assert right.pose(90.0, -60.0, -math.pi / 2) == pytest.approx((0.0, 0.0))


RADAR = models.Radar([2.2, 0.0, 0.4], 0.0, 8.4, math.radians(120.0), math.radians(2.0))
AHEAD = (7.2, -0.5, 8.2, 0.5)   # a 1 m box whose near face is 5 m in front of the radar


def returns(t, obstacles, keep=1.0, seed=3):
    return models.radar_returns(RADAR, t, obstacles, 1.0, random.Random(seed), keep=keep)


def test_boxes():
    assert models.boxes([3.0, 1.0, 2.0, -1.0]) == [(2.0, -1.0, 3.0, 1.0)]
    assert models.boxes([]) == []
    with pytest.raises(ValueError):
        models.boxes([1.0, 2.0, 3.0])


def test_a_ray_stops_at_the_near_face_of_a_box():
    assert models.ray_hits_box(0.0, 0.0, 1.0, 0.0, AHEAD) == pytest.approx(7.2)
    assert models.ray_hits_box(0.0, 0.0, -1.0, 0.0, AHEAD) is None     # behind
    assert models.ray_hits_box(0.0, 2.0, 1.0, 0.0, AHEAD) is None      # passes beside it


def test_the_radar_sees_the_near_face_of_a_box_ahead():
    points = returns(truth(), [AHEAD])
    assert len(points) >= 5
    for x, y, z, v in points:
        assert x == pytest.approx(5.0, abs=0.25) and abs(y) <= 0.55
        assert -0.8 < z < 1.0, 'z is in radar_link: 0.4 m above the ground'


def test_nothing_to_see_means_no_points():
    assert returns(truth(), []) == []
    assert returns(truth(), [(20.0, -0.5, 21.0, 0.5)]) == [], 'beyond its 8.4 m'
    assert returns(truth(), [(-5.0, -0.5, -4.0, 0.5)]) == [], 'behind the car'
    assert returns(truth(), [(2.5, 8.0, 3.5, 9.0)]) == [], 'outside its 120 degrees'


def test_a_near_box_hides_the_one_behind_it():
    behind = (9.2, -0.5, 10.2, 0.5)
    assert all(x < 5.3 for x, *_ in returns(truth(), [AHEAD, behind]))


def test_the_points_turn_with_the_car():
    north = returns(truth(yaw=math.pi / 2), [(-0.5, 7.2, 0.5, 8.2)])
    assert north and all(x == pytest.approx(5.0, abs=0.25) for x, *_ in north)


def test_the_radar_is_sparse_seeded_and_measures_closing_speed():
    sparse = returns(truth(speed=1.0), [AHEAD], keep=0.5)
    assert 0 < len(sparse) < len(returns(truth(), [AHEAD]))
    assert sparse == returns(truth(speed=1.0), [AHEAD], keep=0.5)
    assert all(v < -0.9 for *_, v in sparse), 'closing at about the car speed'


LIDAR = models.Lidar([0.9, 0.0, 1.8], [math.radians(e) for e in (-15, -5, -1, 1, 15)], 100.0)


def test_a_lidar_sees_the_ground_with_its_low_lasers_and_nothing_with_its_high_ones():
    r = models.lidar_ranges(LIDAR, truth(), [], 1.0, 0.0)
    # -15 deg: the ground 6.72 m away along it, 1.8 / sin(15); -1 deg: 103 m, past 100.
    assert r[0] == pytest.approx(1.8 / math.sin(math.radians(15)))
    assert r[1] == pytest.approx(1.8 / math.sin(math.radians(5)))
    assert r[2:] == [0.0, 0.0, 0.0]


def test_a_lidar_sees_a_box_ahead_with_the_lasers_that_meet_it_below_its_top():
    box = models.boxes([15.9, -1.0, 16.9, 1.0])     # its face 15 m from the lidar (x 0.9)
    r = models.lidar_ranges(LIDAR, truth(), box, 1.0, 0.0)
    assert r[0] == pytest.approx(1.8 / math.sin(math.radians(15)))   # the ground first
    # -5 deg meets the face 0.49 m up; -1 deg at 1.54 m, over the 1 m box: it flies on.
    assert r[1] == pytest.approx(15.0 / math.cos(math.radians(5)))
    assert r[2] == 0.0


def test_a_lidar_bearing_is_clockwise_from_the_cars_heading():
    box = models.boxes([0.4, -5.0, 1.4, -4.0])      # 4 m to the right of the lidar
    right = models.lidar_ranges(LIDAR, truth(), box, 1.0, math.pi / 2)
    left = models.lidar_ranges(LIDAR, truth(), box, 1.0, -math.pi / 2)
    # -15 deg meets it 0.73 m up, before the ground (6.72 m); -5 deg clears its top.
    assert right[0] == pytest.approx(4.0 / math.cos(math.radians(15)))
    assert left[0] == pytest.approx(1.8 / math.sin(math.radians(15)))   # the ground


def gps_run(error, seconds, rate=5.0):
    return [error.step(1.0 / rate) for _ in range(int(seconds * rate))]


def autocorrelation(values, lag):
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    pairs = zip(values, values[lag:])
    return sum((a - mean) * (b - mean) for a, b in pairs) / ((len(values) - lag) * var)


def test_the_gps_error_wanders_slowly_with_its_steady_spread():
    # 6 hours at 1 Hz: 360 drift times of 60 s, enough for the spread to settle.
    error = models.GpsError(0.5, 60.0, 0.05, random.Random(7))
    fixes = gps_run(error, 6 * 3600.0, rate=1.0)
    for axis in (0, 1):
        values = [f[axis] for f in fixes]
        mean = sum(values) / len(values)
        spread = math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
        assert abs(mean) < 0.1
        assert spread == pytest.approx(error.stddev, rel=0.1)
        # One drift time (60 fixes) later it remembers about exp(-1) of itself.
        assert autocorrelation(values, 60) == pytest.approx(math.exp(-1.0), abs=0.1)


def test_the_gps_error_moves_centimetres_from_fix_to_fix():
    # At 5 Hz, not the 0.9 m of white noise's 0.5 m on each axis.
    fixes = gps_run(models.GpsError(0.5, 60.0, 0.05, random.Random(7)), 600.0)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(fixes, fixes[1:])]
    assert sum(steps) / len(steps) < 0.15


def test_white_noise_when_the_drift_forgets_at_once():
    error = models.GpsError(0.5, 0.0, 0.0, random.Random(3))
    values = [f[0] for f in gps_run(error, 2000.0)]
    assert autocorrelation(values, 1) == pytest.approx(0.0, abs=0.05)
    spread = math.sqrt(sum(v * v for v in values) / len(values))
    assert spread == pytest.approx(0.5, rel=0.05)


def test_the_gps_error_repeats_with_its_seed_and_refuses_negatives():
    a = gps_run(models.GpsError(0.5, 30.0, 0.05, random.Random(2)), 10.0)
    b = gps_run(models.GpsError(0.5, 30.0, 0.05, random.Random(2)), 10.0)
    assert a == b
    assert models.GpsError(0.3, 30.0, 0.4, random.Random(1)).stddev == pytest.approx(0.5)
    with pytest.raises(ValueError):
        models.GpsError(-0.5, 30.0, 0.05, random.Random(1))
