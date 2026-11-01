import math

from aav_lidar.scan import flatten, ScanShape, to_base
import numpy
import pytest

SHAPE = ScanShape()      # 720 directions, 0.3 to 2.0 m above the road
ROOF = ([0.9, 0.0, 1.8], [0.0, 0.0, 0.0])   # vehicle.yaml's lidar_link


def direction(angle):
    return int(round((angle - SHAPE.angle_min) / SHAPE.angle_increment)) % SHAPE.count


def test_720_directions_all_the_way_round():
    assert SHAPE.count == 720
    assert numpy.isinf(flatten(numpy.zeros((0, 3)), SHAPE)).all()


def test_a_post_ahead_is_its_nearest_point_from_base_link():
    # A post 5 m ahead of the lidar, from 1 m below the lidar down to the ground.
    post = numpy.array([[5.0, 0.0, -1.0], [5.2, 0.0, -1.5], [5.0, 0.0, -1.79]])
    ranges = flatten(to_base(post, *ROOF), SHAPE)
    assert ranges[direction(0.0)] == pytest.approx(5.9)   # 5 m + the lidar's 0.9 m
    assert numpy.isinf(numpy.delete(ranges, direction(0.0))).all()


def test_the_road_and_what_is_overhead_are_not_obstacles():
    road = numpy.array([[8.0, 1.0, -1.75], [10.0, -2.0, -1.6]])    # 0.05 and 0.2 m up
    branch = numpy.array([[4.0, 0.0, 0.5]])                        # 2.3 m up
    lost = numpy.array([[numpy.inf, numpy.inf, numpy.inf], [numpy.nan, 0.0, 0.0]])
    points = to_base(numpy.vstack([road, branch, lost]), *ROOF)
    assert numpy.isinf(flatten(points, SHAPE)).all()


def test_left_is_positive_and_behind_wraps():
    points = to_base(numpy.array([[0.0, 3.0, -1.0], [-4.0, 0.001, -1.0]]), *ROOF)
    ranges = flatten(points, SHAPE)
    assert math.isfinite(ranges[direction(math.atan2(3.0, 0.9))])
    assert math.isfinite(ranges[0])       # just under +pi: straight behind, as -pi is


def test_each_point_goes_to_its_nearest_direction():
    # 0.4 degrees left is nearer 0.5 than 0: rounding down put it at 0 (half a step off).
    angle = math.radians(0.4)
    points = to_base(numpy.array([[10.0 * math.cos(angle), 10.0 * math.sin(angle), -1.0]]),
                     [0.0, 0.0, 1.8], [0.0, 0.0, 0.0])
    ranges = flatten(points, SHAPE)
    assert math.isfinite(ranges[direction(math.radians(0.5))])
    assert direction(math.radians(0.5)) == direction(0.0) + 1


def test_a_turned_lidar_turns_its_points():
    # A lidar turned 90 degrees left sees ahead of the car along its own -y.
    points = to_base(numpy.array([[0.0, -5.0, 0.0]]), [0.0, 0.0, 1.0], [0.0, 0.0, math.pi / 2])
    assert points[0] == pytest.approx([5.0, 0.0, 1.0])


def test_the_car_itself_is_not_an_obstacle():
    shape = ScanShape(body=(-0.2, 1.8, 0.5))
    roof = to_base(numpy.array([[-0.5, 0.0, -0.3], [0.5, 0.4, -0.3]]), *ROOF)  # 1.5 m up
    ahead = to_base(numpy.array([[1.5, 0.0, -1.0]]), *ROOF)                     # past the bumper
    ranges = flatten(numpy.vstack([roof, ahead]), shape)
    assert numpy.isfinite(ranges).sum() == 1
    assert ranges[direction(0.0)] == pytest.approx(2.4)
