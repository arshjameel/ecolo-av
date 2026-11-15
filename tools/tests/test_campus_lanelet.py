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
