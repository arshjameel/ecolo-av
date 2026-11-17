"""The campus's roads as a Lanelet2 map (tools/campus_lanelet.py)."""

import math
from pathlib import Path
import sys
from xml.etree import ElementTree

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
campus_lanelet = pytest.importorskip('campus_lanelet')


def roads():
    """Return a T: a one-way road east with 2 lanes, then a two-way road north."""
    return [
        {'points': [[0, 0], [50, 0]], 'width': 7.0, 'oneway': True, 'name': 'East'},
        {'points': [[50, 0], [50, 40]], 'width': 7.0, 'oneway': False, 'name': 'North'},
    ]


def test_lane_counts_follow_the_width():
    assert campus_lanelet.lane_counts(7.0, True) == (2, 0)
    assert campus_lanelet.lane_counts(7.0, False) == (1, 1)
    assert campus_lanelet.lane_counts(10.5, False) == (2, 1)
    assert campus_lanelet.lane_counts(3.5, False) == (1, 1)


def test_an_offset_line_keeps_its_distance():
    line = campus_lanelet.offset([(0, 0), (10, 0), (10, 10)], 1.0)
    assert line[0] == pytest.approx((0, 1))
    assert line[-1] == pytest.approx((9, 10))
    assert line[1] == pytest.approx((9, 1))     # the corner, mitred


def test_each_lane_is_a_lanelet_and_the_ends_are_joined():
    osm, report = campus_lanelet.build(roads())
    assert report['lanes'] == 4              # 2 east, 1 north, 1 south
    # Both east lanes go on north. North's top is a dead end: it turns round onto south.
    # South can't go on east against the one way: it turns round onto north too.
    assert report['joins'] == 4


def test_the_file_is_lanelet2_osm():
    osm, _ = campus_lanelet.build(roads())
    root = ElementTree.fromstring(campus_lanelet.to_xml(osm, (45.386, -75.696)))
    lanelets = [r for r in root.iter('relation')
                if any(t.get('v') == 'lanelet' for t in r.iter('tag'))]
    assert len(lanelets) == 8                # 4 lanes and 4 joins
    node = root.find('node')
    assert math.isfinite(float(node.get('lat'))) and node.find('tag').get('k') == 'local_x'


def test_roads_are_cut_where_another_road_crosses_them_mid_way():
    """Two streets crossing in the middle of both: four roads, joined at the crossing."""
    graph = campus_lanelet.campus_graph
    d = 0.0001                       # about 11 m of latitude
    nodes = {'w': (45.386, -75.696 - 2 * d), 'm': (45.386, -75.696),
             'e': (45.386, -75.696 + 2 * d), 's': (45.386 - d, -75.696),
             'n': (45.386 + d, -75.696), 'x': (45.386 + 2 * d, -75.696)}
    ways = {'1': ({'highway': 'service'}, ['w', 'm', 'e']),
            '2': ({'highway': 'service', 'oneway': '-1'}, ['s', 'm', 'n', 'x'])}
    osm_data = graph.Osm(nodes, ways, [])
    roads = campus_lanelet.roads_from_osm(osm_data, ('service',), (45.386, -75.696))
    assert len(roads) == 4                   # each street cut at m; n is only on way 2
    one_way = [r for r in roads if r['oneway']]
    assert len(one_way) == 2
    # oneway=-1: driven against the way's order, so from x down to s.
    assert one_way[0]['points'][0][1] > one_way[0]['points'][-1][1]
    _, report = campus_lanelet.build(roads)
    assert report['joins'] > 0


def test_a_stop_sign_binds_the_road_end_it_is_near():
    line = [(0, 0), (50, 0), (100, 0)]
    assert campus_lanelet.stop_ends(line, [2]) == {'stop_at_end': True}
    assert campus_lanelet.stop_ends(line, [0]) == {'stop_at_start': True}
    assert campus_lanelet.stop_ends(line, [1]) == {}      # 50 m from both: too far
    assert campus_lanelet.stop_ends(line, []) == {}


def test_a_stop_sign_gives_each_lane_ending_there_a_stop_line_and_a_rule():
    """The T, a stop sign at the one-way road's end: both its lanes stop there."""
    t = roads()
    t[0]['stop_at_end'] = True
    osm, report = campus_lanelet.build(t)
    assert report['stops'] == 2
    root = ElementTree.fromstring(campus_lanelet.to_xml(osm, (45.386, -75.696)))
    ways = {w.get('id'): {t.get('k'): t.get('v') for t in w.iter('tag')}
            for w in root.iter('way')}
    rules = [r for r in root.iter('relation')
             if any(t.get('v') == 'traffic_sign' for t in r.iter('tag'))]
    for rule in rules:
        roles = {m.get('role'): m.get('ref') for m in rule.iter('member')}
        assert ways[roles['ref_line']]['type'] == 'stop_line'
        assert ways[roles['refers']] == {'type': 'traffic_sign', 'subtype': 'stop'}
    bound = [r for r in root.iter('relation')
             if any(m.get('role') == 'regulatory_element' for m in r.iter('member'))]
    assert len(bound) == 2                   # the two east lanes, not the north road's
