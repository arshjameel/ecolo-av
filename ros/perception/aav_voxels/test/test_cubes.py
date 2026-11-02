import math

from aav_voxels.cubes import (cube_centres, cubes, CubeShape, fit_ground, Ground,
                              lowest_per_cell, nearest_per_cube)
from lot import turn as lidar_turn     # the made-up lidar turn, beside this file
import numpy
import pytest

BODY = (-0.5, 1.9, 0.8)      # aav_lidar scan.body_box of vehicle.yaml's car
SHAPE = CubeShape(body=BODY)
POST = (10.0, 10.5, -0.25, 0.25, 1.8)        # sim_check's post, 10 m ahead
WALL = (5.0, 30.0, 8.0, 8.3, 2.5)            # a wall 8 m to the left


def near(centres, x, y, within):
    return numpy.hypot(centres[:, 0] - x, centres[:, 1] - y) <= within


def test_a_cube_holds_its_points_once():
    points = numpy.array([[0.01, 0.01, 0.01], [0.2, 0.2, 0.2], [0.3, 0.0, 0.0]])
    centres = cube_centres(points, 0.25)
    assert centres.tolist() == [[0.125, 0.125, 0.125], [0.375, 0.125, 0.125]]
    assert cube_centres(numpy.zeros((0, 3)), 0.25).shape == (0, 3)
    flat = cube_centres(points, 0.25, flat=True)
    assert flat[:, 2].tolist() == [0.0, 0.0]


def test_negative_coordinates_floor_not_truncate():
    centres = cube_centres(numpy.array([[-0.1, -0.1, 0.5]]), 0.25)
    assert centres.tolist() == [[-0.125, -0.125, 0.625]]


def test_lowest_per_cell():
    points = numpy.array([[0.5, 0.5, 1.0], [0.6, 0.4, 0.2], [3.0, 0.0, 0.7]])
    low = lowest_per_cell(points, 2.0)
    assert sorted(low[:, 2].tolist()) == [0.2, 0.7]


def test_the_post_is_cubes_and_the_road_is_not():
    c = cubes(lidar_turn([POST]), SHAPE)
    assert c.road.fitted
    assert abs(c.road.c) < 0.02 and abs(c.road.roll) < math.radians(0.2)
    assert near(c.obstacles, 10.25, 0.0, 0.6).sum() >= 4
    assert not (~near(c.obstacles, 10.25, 0.0, 0.6)).any()     # nothing else on the lot
    # The post's near face itself: each cube's nearest point, not its centre.
    nearest = c.obstacles[near(c.obstacles, 10.25, 0.0, 0.6), 0].min()
    assert nearest == pytest.approx(10.0, abs=1e-6)
    assert len(c.ground) > 500 and numpy.abs(c.ground[:, 2]).max() < 0.05


@pytest.mark.parametrize('roll, pitch', [(3.0, 0.0), (-3.0, 0.0), (0.0, 2.0), (2.0, -2.0)])
def test_a_leaning_car_still_sees_a_flat_road(roll, pitch):
    roll, pitch = math.radians(roll), math.radians(pitch)
    points = lidar_turn([POST, WALL], roll=roll, pitch=pitch)
    c = cubes(points, SHAPE)
    assert c.road.fitted
    assert c.road.roll == pytest.approx(roll, abs=math.radians(0.3))
    assert c.road.pitch == pytest.approx(pitch, abs=math.radians(0.3))
    things = near(c.obstacles, 10.25, 0.0, 0.8) | (c.obstacles[:, 1] > 7.5)
    assert things.any() and not (~things).any()
    # The flat cut, as lidar_scan's: the road itself becomes obstacles on the low side.
    flat = cubes(points, SHAPE, road=Ground())
    assert len(flat.obstacles) > len(c.obstacles) + 100


def test_the_car_itself_and_the_sky_never_count():
    roof = numpy.array([[1.75, 0.29, 1.55], [0.0, 0.0, 1.5], [-0.4, 0.7, 1.0]])   # cameras, roof
    sky = numpy.array([[numpy.inf, numpy.inf, numpy.inf], [numpy.nan, 0.0, 0.0]])
    c = cubes(numpy.vstack([roof, sky, lidar_turn()]), SHAPE)
    assert len(c.obstacles) == 0
    road = lidar_turn()
    seen = numpy.isfinite(road).all(axis=1) & (numpy.hypot(road[:, 0], road[:, 1]) <= 30.0)
    assert c.points == seen.sum()        # the road within range_max, and nothing else


def test_overhead_and_far_away_are_left_out():
    branch = numpy.array([[6.0, 0.0, 2.5]])            # over the roof plus its margin
    far = numpy.array([[40.0, 0.0, 1.0]])               # past range_max
    c = cubes(numpy.vstack([branch, far, lidar_turn()]), SHAPE)
    assert len(c.obstacles) == 0


def test_too_few_candidates_keep_the_road_flat():
    road = fit_ground(numpy.array([[5.0, 0.0, 0.0], [6.0, 3.0, 0.0]]), SHAPE)
    assert road == Ground(candidates=2)


def test_a_steep_fit_is_not_believed():
    # A ramp at 10 degrees: steeper than max_tilt, so taken as a fit gone wrong.
    xs, ys = numpy.meshgrid(numpy.arange(2.0, 20.0, 0.5), numpy.arange(-10.0, 10.0, 0.5))
    ramp = numpy.column_stack([xs.ravel(), ys.ravel(), xs.ravel() * math.tan(math.radians(10))])
    assert not fit_ground(ramp, SHAPE).fitted


def test_walls_without_road_beside_them_do_not_tilt_the_fit():
    # Walls all round at 6 m: the cells there see the walls' bottoms, and those drop out.
    walls = [(6.0, 6.3, -20, 20, 3.0), (-20, 20, 6.0, 6.3, 3.0), (-20, 20, -6.3, -6.0, 3.0)]
    c = cubes(lidar_turn(walls, roll=math.radians(2.0)), SHAPE)
    assert c.road.fitted
    assert c.road.roll == pytest.approx(math.radians(2.0), abs=math.radians(0.3))


def test_each_cube_keeps_its_nearest_point():
    points = numpy.array([[5.2, 0.1, 1.0], [5.01, 0.2, 1.1], [5.3, 0.3, 1.2], [-5.2, 0.1, 1.0]])
    kept = nearest_per_cube(points, 0.5)
    assert sorted(kept[:, 0].tolist()) == [-5.2, 5.01]
    assert nearest_per_cube(numpy.zeros((0, 3)), 0.25).shape == (0, 3)


def test_a_box_among_parked_cars_still_shows():
    # A crowded lot: rows of parked cars (1.5 m high) left and right, a 1 m box 8 m ahead.
    cars = [(x, x + 4.5, y, y + 1.8, 1.5) for x in range(-20, 25, 6) for y in (4.0, -5.8, 9.0)]
    box = (8.0, 9.0, -0.5, 0.5, 1.0)
    for roll in (0.0, math.radians(2.0)):
        c = cubes(lidar_turn(cars + [box], roll=roll), SHAPE)
        assert c.road.fitted
        assert c.road.roll == pytest.approx(roll, abs=math.radians(0.3))
        assert near(c.obstacles, 8.0, 0.0, 0.6).sum() >= 2


def test_a_crowned_road_never_lifts_the_road_or_becomes_an_obstacle():
    # The road falls away from the centre line: 0.43 m lower 12 m out. No one plane fits;
    # the fit may only put the road too low (more cubes), never too high (fewer).
    xs, ys = numpy.meshgrid(numpy.arange(2.0, 20.0, 0.25), numpy.arange(-12.0, 12.0, 0.25))
    crown = numpy.column_stack([xs.ravel(), ys.ravel(), -0.003 * ys.ravel() ** 2])
    road = fit_ground(crown, SHAPE)
    assert road.c <= 0.01
    assert len(cubes(crown, SHAPE).obstacles) == 0
