"""The lane map: the car's lane on each road, the line along the route, and its edges."""

from collections import namedtuple
import math

from aav_description.lane_map import check_lane_graph, lanelet2_beside, planner_block
from aav_description.lane_map import check_lane_source, LaneMap, lanes_block, Road
from aav_description.lane_map import costmap_block, from_dict, lane_line, lane_offset
from aav_description.lane_map import with_edges
from aav_description.road_ahead import build
from aav_description.stop_law import from_car_model
import pytest

Stop = namedtuple('Stop', 'reason left')
CURVE = from_car_model()

# The road world (ros/sim/aav_sim_models/worlds/road.lanes.yaml): one lane, 3.5 m between the
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


def test_roads_at_finds_each_road_under_a_point_once():
    across = Road('across', ((-50.0, 0.0), (50.0, 0.0)), 7.0)
    bent = Road('bent', ((0.0, 60.0), (10.0, 70.0), (20.0, 70.0)), 6.0)
    lanes = LaneMap([STREET, across, bent])
    assert [r.name for r in lanes.roads_at(0.0, 20.0)] == ['street']
    assert sorted(r.name for r in lanes.roads_at(1.0, 1.0)) == ['across', 'street']
    assert [r.name for r in lanes.roads_at(10.0, 70.0)] == ['bent']   # two segments, once
    assert lanes.roads_at(30.0, 30.0) == []


def test_the_costmap_block_carries_every_road_as_the_file_has_it():
    # aav_costmap reads these keys (costmap-node's lanes_from).
    lanes = from_dict({'roads': [
        {'name': 'a', 'width': 7.0, 'points': [[0, 0], [10, 0]]},
        {'kind': 'b', 'width': 4.5, 'lanes': 1, 'oneway': True, 'edge_inset': 0.4,
         'points': [[0, 5], [10, 5], [20, 8]]}]})
    block = costmap_block(lanes)
    assert [r['name'] for r in block['roads']] == ['a', 'b']
    assert block['roads'][1] == {'name': 'b', 'points': [[0.0, 5.0], [10.0, 5.0], [20.0, 8.0]],
                                 'width': 4.5, 'lanes': 1, 'oneway': True, 'edge_inset': 0.4}
    assert block['roads'][0]['lanes'] == 0


def test_the_lanes_block_by_its_source():
    # TODO 264: costmap-node's lane_sources and camera_from read these keys.
    lanes = from_dict(ROAD_WORLD)
    assert lanes_block(lanes) == dict(costmap_block(lanes), source='map')
    camera = lanes_block(None, 'camera')
    assert camera == {'roads': [], 'source': 'camera',
                      'camera': {'topic': '/perception/lane/geometry'}}
    both = lanes_block(lanes, 'both')
    assert both['roads'] == costmap_block(lanes)['roads'] and both['source'] == 'both'
    assert lanes_block(None, 'both')['roads'] == []
    with pytest.raises(ValueError):
        lanes_block(None, 'map')
    assert check_lane_source('') == 'map'
    with pytest.raises(ValueError):
        check_lane_source('radar')


def test_a_pass_keeps_the_routes_line_and_opens_the_left_soft_edge():
    # Ben, 16 Nov: the planner moves left round something parked (its status: pass). The
    # route north, on the lane's centre (x 1.5), out to x 0.0 for y 0-10 (1.5 m left).
    lanes = LaneMap([STREET])
    route = [(1.5, -20.0), (1.5, -5.0), (0.0, 0.0), (0.0, 10.0), (1.5, 15.0), (1.5, 40.0)]
    line = lane_line(lanes, route, (1.5, -20.0), length=50.0, passing=True)
    beside = [x for x, y in line if 3.5 <= y <= 6.5]     # eased over 3 m at the ends
    assert beside and all(x == pytest.approx(0.0, abs=0.01) for x in beside)
    assert max(abs(b[0] - a[0]) for a, b in zip(line, line[1:])) < 0.3     # no jump
    assert all(x == pytest.approx(1.5) for x, y in line if y < -10.0 or y > 25.0)
    # Not passing: the old way, the lane's centre all along.
    old = lane_line(lanes, route, (1.5, -20.0), length=50.0)
    assert all(x == pytest.approx(1.5) for x, _ in old)
    strip = build([(0.0, 0.0), (0.0, 10.0)], (0.0, 0.0), [], CURVE, 1.0)
    shut = with_edges(strip, lanes)
    opened = with_edges(strip, lanes, passing=True)
    assert shut.left_soft[0] == pytest.approx(0.0)       # on the centre line already
    assert opened.left_soft[0] == pytest.approx(3.5)     # to the left kerb (x -3.5)
    assert opened.left_hard[0] == pytest.approx(shut.left_hard[0])
    assert opened.right_soft[0] == pytest.approx(shut.right_soft[0])


def test_the_route_level_reads_the_lanelet2_map_beside_the_lanes(tmp_path):
    folder = tmp_path / 'campus'
    folder.mkdir()
    lanes = folder / 'lanes.yaml'
    lanes.write_text('roads:\n- {name: r, width: 7.0, points: [[0, 0], [10, 0]]}\n')
    assert check_lane_graph('') == 'lanes_yaml'
    with pytest.raises(ValueError):
        check_lane_graph('osm')
    # No map beside it: lanes.yaml's roads.
    assert lanelet2_beside(str(lanes)) is None
    assert planner_block(str(lanes), 'lanelet2')['roads'][0]['name'] == 'r'
    # The folder's own name first, else its only .osm.
    (folder / 'other.osm').write_text('<osm/>')
    assert lanelet2_beside(str(lanes)) == str(folder / 'other.osm')
    (folder / 'campus.osm').write_text('<osm/>')
    assert lanelet2_beside(str(lanes)) == str(folder / 'campus.osm')
    assert planner_block(str(lanes), 'lanelet2') == {'lanelet2': str(folder / 'campus.osm')}
    assert 'roads' in planner_block(str(lanes), 'lanes_yaml')
