"""The lane map: the car's lane on each road, the line along the route, and its edges."""

from collections import namedtuple
import math

from aav_description.lane_map import from_dict, lane_line, lane_offset, LaneMap, Road, with_edges
from aav_description.road_ahead import build
from aav_description.stop_law import from_car_model
import pytest

Stop = namedtuple('Stop', 'reason left')
CURVE = from_car_model()

# The road world (ros/sim/aav_gazebo/worlds/road.lanes.yaml): one lane, 3.5 m between the
# lines, 0.5 m of tarmac outside each, east along y = 0.
ROAD_WORLD = {'roads': [{'name': 'road', 'width': 4.5, 'lanes': 1, 'oneway': False,
                         'points': [[-20.0, 0.0], [100.0, 0.0]]}]}
# A two-way street, 7 m kerb to kerb (two 3.5 m lanes in OpenStreetMap), north along x = 0.
STREET = Road('street', ((0.0, -50.0), (0.0, 50.0)), 7.0)


def test_one_lane_drives_down_the_middle_either_way():
    lanes = from_dict(ROAD_WORLD)
    east = lanes.lane_at(10.0, 0.4, 0.0)
    west = lanes.lane_at(10.0, 0.4, math.pi)
    for lane in (east, west):
        assert (lane.x, lane.y) == pytest.approx((10.0, 0.0))
        assert (lane.left_soft, lane.right_soft) == pytest.approx((1.75, 1.75))
        assert (lane.left_hard, lane.right_hard) == pytest.approx((2.25, 2.25))
    assert west.heading == pytest.approx(math.pi)


def test_a_two_way_street_keeps_right_whichever_way_the_route_runs():
    lanes = LaneMap([STREET])
    north = lanes.lane_at(0.0, 0.0, math.pi / 2)
    south = lanes.lane_at(0.0, 0.0, -math.pi / 2)
    # 7 m: 6 m between the edge lines, a 3 m lane each way, its centre 1.5 m from the middle.
    assert (north.x, north.y) == pytest.approx((1.5, 0.0))      # east: right, going north
    assert (south.x, south.y) == pytest.approx((-1.5, 0.0))
    assert (north.left_soft, north.right_soft) == pytest.approx((1.5, 1.5))
    assert (north.left_hard, north.right_hard) == pytest.approx((5.0, 2.0))


@pytest.mark.parametrize('road, right, width', [
    (Road('one lane', ((0, 0), (1, 0)), 3.5, oneway=True), 0.0, 2.5),
    (Road('two, one way', ((0, 0), (1, 0)), 7.0, oneway=True), 1.5, 3.0),
    (Road('three, one way', ((0, 0), (1, 0)), 10.5, oneway=True), 3.17, 3.17),
    (Road('four, two way', ((0, 0), (1, 0)), 14.0), 4.875, 3.25),
])
def test_the_rightmost_lane(road, right, width):
    assert lane_offset(road) == pytest.approx((right, width), abs=0.01)


def test_a_crossing_road_is_not_the_cars():
    lanes = LaneMap([STREET])
    assert lanes.lane_at(0.0, 0.0, 0.0) is None            # driving across it, east
    assert lanes.lane_at(10.0, 0.0, math.pi / 2) is None   # beside it, on the grass


def test_the_line_sits_on_the_lane_centre_and_has_edges():
    lanes = LaneMap([STREET])
    route = [(0.3, -40.0), (0.3, 40.0)]       # Nav2's path, near the road's middle
    line = lane_line(lanes, route, (0.3, -30.0))
    assert all(x == pytest.approx(1.5) for x, _ in line)
    strip = with_edges(build(line, (1.2, -30.0), [Stop('goal', 30.0)], CURVE, 1.0), lanes)
    assert all(x == pytest.approx(1.5) for x in strip.x)
    assert strip.end == pytest.approx(30.0, abs=0.6)
    assert {round(v, 6) for v in strip.left_soft} == {1.5}
    assert {round(v, 6) for v in strip.right_hard} == {2.0}


def test_off_every_road_the_route_stays_and_the_edges_are_unknown():
    lanes = LaneMap([STREET])
    route = [(30.0, 0.0), (60.0, 0.0)]        # a lot, east of the street
    line = lane_line(lanes, route, (30.0, 0.0), length=20.0)
    assert all(y == pytest.approx(0.0) for _, y in line)
    strip = with_edges(build(line, (30.0, 0.0), [], CURVE, 1.0), lanes)
    assert all(math.isnan(v) for v in strip.left_hard)


def test_the_line_eases_from_the_lot_onto_the_lane():
    lanes = LaneMap([Road('street', ((0.0, 0.0), (100.0, 0.0)), 7.0)])
    route = [(-20.0, 0.0), (60.0, 0.0)]       # out of a lot (x < 0) onto the road's middle
    line = lane_line(lanes, route, (-20.0, 0.0), spacing=0.5, length=40.0)
    ys = [y for _, y in line]
    assert ys[0] == pytest.approx(0.0)
    assert ys[-1] == pytest.approx(-1.5)
    steps = [abs(b - a) for a, b in zip(ys, ys[1:])]
    assert max(steps) < 0.15                  # no jump of a whole lane in one step


def test_edges_from_a_point_off_the_centre():
    lanes = LaneMap([STREET])
    strip = with_edges(build([(1.0, 0.0), (1.0, 10.0)], (1.0, 0.0), [], CURVE, 1.0), lanes)
    # 0.5 m left of the lane's centre (x = 1.5, going north): less room on the left.
    assert strip.left_soft[0] == pytest.approx(1.0)
    assert strip.right_soft[0] == pytest.approx(2.0)
    assert strip.left_hard[0] == pytest.approx(4.5)


def test_load_reads_the_campus_format(tmp_path):
    path = tmp_path / 'lanes.yaml'
    path.write_text('roads:\n- kind: tertiary\n  name: Library Road\n  oneway: false\n'
                    '  points:\n  - [0.0, 0.0]\n  - [50.0, 0.0]\n  width: 7.0\n')
    from aav_description.lane_map import load
    lanes = load(str(path))
    assert lanes.roads[0].name == 'Library Road'
    assert lanes.roads[0].lane_count() == 2
    assert lanes.lane_at(10.0, 0.0, 0.0).y == pytest.approx(-1.5)


def test_an_empty_map_finds_nothing():
    assert from_dict({}).lane_at(0.0, 0.0, 0.0) is None
    assert lane_line(from_dict({}), [], (0.0, 0.0)) == []


def test_a_route_weaving_inside_the_lane_still_gives_its_centre():
    # Nav2's plan in the road world swings a metre either side of the middle (14 Nov).
    lanes = from_dict(ROAD_WORLD)
    route = [(x * 0.5, 1.0 * math.sin(x * 0.5 / 3.0)) for x in range(0, 120)]
    line = lane_line(lanes, route, (0.0, 0.6), length=40.0)
    assert max(abs(y) for _, y in line) < 1e-9
