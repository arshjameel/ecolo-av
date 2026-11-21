"""Tests for tools/label_core.py: camera boxes onto lidar points, into labels (TODO 252)."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import label_core as lc  # noqa: E402, I100

CAM = lc.Camera.pinhole('front_left', 640, 360, 343.0, (1.75, 0.29, 1.55))


def test_a_point_ahead_lands_where_the_pinhole_says():
    u, v, d = CAM.project(np.array([[11.75, 0.29, 1.55], [11.75, -0.71, 0.55],
                                    [-5.0, 0.0, 1.0]]))
    assert (u[0], v[0], d[0]) == pytest.approx((320.0, 180.0, 10.0))
    # 1 m right and 1 m down at 10 m: 34.3 px right and down.
    assert (u[1], v[1]) == pytest.approx((354.3, 214.3))
    assert math.isnan(u[2]), 'behind the camera'
    assert list(CAM.sees(u, v)) == [True, True, False]


def test_a_turned_camera_looks_behind():
    rear = lc.Camera.pinhole('rear', 640, 360, 343.0, (-0.2, 0.0, 0.9), (0.0, 0.0, math.pi))
    u, v, d = rear.project(np.array([[-10.2, 1.0, 0.9]]))
    assert d[0] == pytest.approx(10.0)
    assert u[0] == pytest.approx(320.0 + 34.3), 'left of the car is right in a rear picture'


def test_poses_go_there_and_back():
    p = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 0.0]])
    pose = (10.0, -3.0, 0.7)
    assert lc.to_base(lc.to_map(p, pose), pose) == pytest.approx(p)
    assert lc.to_map(np.array([[1.0, 0.0, 0.0]]), (0, 0, math.pi / 2))[0] == \
        pytest.approx([0.0, 1.0, 0.0])
    assert lc.mounted(np.array([[1.0, 0.0, 0.0]]), (0.49, 0, 1.52), (0, 0, math.pi))[0] == \
        pytest.approx([-0.51, 0.0, 1.52])


def test_a_box_takes_the_nearest_group_not_whats_behind():
    # A tree 8 m ahead in front of a wall at 20 m, both inside the box.
    tree = [[9.75, 0.29 + y, 2.0 + z] for y in (-0.3, 0, 0.3) for z in (0, 0.5)]
    wall = [[21.75, 0.29 + y, 1.0 + z] for y in np.linspace(-2, 2, 9) for z in (0, 1)]
    u, v, d = CAM.project(np.array(tree + wall))
    got = lc.in_box(u, v, d, (0, 0, 640, 360))
    assert sorted(got) == list(range(len(tree)))
    # Too few points: nothing.
    assert len(lc.in_box(u, v, d, (0, 0, 640, 360), min_points=50)) == 0
    # A box round the wall's right end alone, the tree outside it: the wall.
    assert sorted(lc.in_box(u, v, d, (340, 0, 640, 360))) == \
        [k for k in range(len(tree), len(tree) + len(wall)) if u[k] >= 340]


def test_votes_fuse_into_one_label_per_thing():
    fusion = lc.Fusion()
    building = np.array([[x, y, z] for x in np.arange(10, 20, 0.25)
                         for y in (5.0, 5.2) for z in (1.0, 6.0)])
    tree = np.array([[0.1, 0.1, 3.0], [0.3, 0.2, 4.0], [0.6, 0.1, 5.0]])
    for frame in range(3):
        fusion.add(building, 'building', 0.6, frame)
        fusion.add(tree, 'tree', 0.4, frame)
    fusion.add(tree, 'building', 0.1, 9)          # one stray vote, outvoted
    fusion.add(np.array([[50.0, 50.0, 2.0]]), 'pole', 0.9, 0)   # one frame only: dropped
    labels = fusion.labels('drive_a')
    assert [lab['class'] for lab in labels] == ['building', 'tree']
    fence = lc.Fusion()
    for frame in range(2):
        fence.add(np.array([[x, 0.0, 1.0] for x in np.arange(0, 5, 0.25)]), 'building', 0.5,
                  frame)
        fence.add(np.array([[0.0, 20.0, 1.0], [0.2, 20.5, 1.5]]), 'tree', 0.5, frame)
    assert [lab['class'] for lab in fence.labels('a')] == ['other'], \
        'a low building is a fence; a low tree a bush'
    b = labels[0]
    assert b['id'] == 1 and b['source'] == 'camera' and b['seen_in'] == ['drive_a']
    assert b['region']['centre_m'] == pytest.approx([15.0, 5.25], abs=0.3)
    assert sorted(b['region']['size_m']) == pytest.approx([1.0, 10.0], abs=0.6)
    assert b['band'] == {'min_m': 1.0, 'max_m': 6.0}
    assert b['confidence'] == pytest.approx(0.6)
    assert labels[1]['confidence'] < 0.4, 'the stray vote lowers the agreement'


def test_the_smallest_rectangle_finds_a_turned_box():
    a = math.radians(30)
    c, s = math.cos(a), math.sin(a)
    pts = np.array([[c * x - s * y + 5, s * x + c * y - 2] for x in (-4, 4) for y in (-1, 1)])
    centre, size, yaw = lc.min_rectangle(pts)
    assert centre == pytest.approx((5.0, -2.0))
    assert yaw == pytest.approx(a)
    assert size == pytest.approx((8.0, 2.0))
    region = {'centre_m': list(centre), 'size_m': list(size), 'yaw_rad': yaw}
    assert sorted(map(tuple, lc.corners(region).round(6))) == \
        sorted(map(tuple, pts.round(6)))


def test_overlap_counts_shared_ground():
    a = np.array([[0, 0], [4, 0], [4, 4], [0, 4]], float)
    assert lc.overlap(a, a + 2) == pytest.approx(4.0, rel=0.05)
    assert lc.overlap(a, a + 10) == 0.0
    ell = np.array([[0, 0], [4, 0], [4, 1], [1, 1], [1, 4], [0, 4]], float)   # not convex
    assert lc.overlap(ell, np.array([[2, 2], [3, 2], [3, 3], [2, 3]], float)) == 0.0


def label(id_, cls, centre, source='camera', seen=('a',), conf=0.5):
    return {'id': id_, 'class': cls,
            'region': {'centre_m': list(centre), 'size_m': [2.0, 2.0], 'yaw_rad': 0.0},
            'source': source, 'confidence': conf, 'seen_in': list(seen)}


def test_merge_keeps_people_and_joins_drives():
    person = {'id': 1, 'class': 'parked_car',
              'region': {'centre_m': [51.0, 15.0], 'size_m': [3.0, 32.0]}, 'source': 'person'}
    old = [person, label(2, 'tree', (0, 0), seen=['a']), label(3, 'pole', (9, 9), seen=['b'])]
    new = [label(1, 'tree', (0.5, 0), seen=['b'], conf=0.9),
           label(2, 'parked_car', (51, 20), seen=['b']),
           label(3, 'building', (30, 30), seen=['b'])]
    merged, counts = lc.merge(old, new, 'b')
    assert counts == {'added': 1, 'joined': 1, 'agreed_with_person': 1, 'named_passes': 0}
    by_id = {lab['id']: lab for lab in merged}
    assert 3 not in by_id or by_id[3]['class'] != 'pole', "b's old pole is taken back"
    assert by_id[1]['source'] == 'person' and by_id[1]['seen_in'] == ['b']
    assert by_id[2]['seen_in'] == ['a', 'b'] and by_id[2]['confidence'] == pytest.approx(0.9)
    assert [lab['class'] for lab in merged if lab.get('seen_in') == ['b']
            and lab['source'] == 'camera'] == ['building']
    again, _ = lc.merge(merged, new, 'b')
    assert again == merged, 'labelling one drive twice changes nothing'


def test_the_camera_names_what_occupancy_saw_once():
    seen_once = label(5, 'other', (10, 10), source='passes', seen=['c'])
    seen_once.pop('confidence')
    merged, counts = lc.merge([seen_once], [label(1, 'parked_car', (10.5, 10))], 'd')
    assert counts['named_passes'] == 1 and len(merged) == 1
    assert merged[0]['class'] == 'parked_car'
    assert merged[0]['source'] == 'passes' and merged[0]['seen_in'] == ['c']


def test_labels_yaml_round_trips(tmp_path):
    path = tmp_path / 'labels.yaml'
    labels = [label(1, 'tree', (1.5, -2.25))]
    lc.write_labels(path, labels)
    assert lc.read_labels(path) == labels
    assert path.read_text().startswith('# Labels') and 'version: 1' in path.read_text()
    assert lc.read_labels(tmp_path / 'none.yaml') == []


def test_scores_count_right_labels_and_found_things():
    things = [lc.Thing('building', lc.corners({'centre_m': [0, 0], 'size_m': [10, 10]})),
              lc.Thing('tree', lc.corners({'centre_m': [20, 0], 'size_m': [3, 3]})),
              lc.Thing('tree', lc.corners({'centre_m': [40, 0], 'size_m': [3, 3]}))]
    labels = [label(1, 'building', (3, 3)), label(2, 'tree', (20, 1)),
              label(3, 'tree', (0, 0)), label(4, 'pole', (60, 0))]
    s = lc.score(labels, things)
    assert s['building'] == {'labels': 1, 'right': 1, 'things': 1, 'found': 1,
                             'precision': 1.0, 'recall': 1.0}
    assert (s['tree']['precision'], s['tree']['recall']) == (0.5, 0.5)
    assert s['pole']['precision'] == 0.0 and s['pole']['recall'] is None
    assert (s['all']['right'], s['all']['found']) == (2, 2)
    assert '| building | 1 | 1 | 100 % |' in lc.score_table(s)


def test_world_boxes_become_truth_and_visibility_needs_points():
    sdf = """<sdf><world>
      <model name="building"><pose>30 27.5 2 0 0 0</pose><link><visual>
        <geometry><box><size>20 15 4</size></box></geometry></visual></link></model>
      <model name="ground_plane"><link><visual><geometry><plane/></geometry></visual></link>
      </model></world></sdf>"""
    things = lc.sdf_things(sdf, {'building': 'building'})
    assert len(things) == 1 and things[0].cls == 'building' and things[0].height == 4.0
    wall = np.array([[20.0, 27.5 + k * 0.1, 1.0] for k in range(30)])
    assert lc.visible(things, wall) == [True]
    assert lc.visible(things, wall[:5]) == [False]


def test_point_features_and_classes():
    pts = np.array([[5.0, 0.0, 0.1], [5.1, 0.1, 3.0], [5.2, 0.1, 6.0], [20.0, 0.0, 1.0]])
    f = lc.point_features(pts)
    assert f.shape == (4, len(lc.FEATURES)) and f.dtype == np.float32
    assert f[1, lc.FEATURES.index('zmax_fine')] == pytest.approx(6.0)
    assert f[1, lc.FEATURES.index('span_fine')] == pytest.approx(5.9)
    assert f[3, lc.FEATURES.index('count_fine')] == pytest.approx(math.log(2))
    tree = label(1, 'tree', (5.1, 0.0))
    tree['band'] = {'min_m': 2.0, 'max_m': 7.0}
    classes = lc.point_classes(pts, [tree])
    assert [lc.SEG_CLASSES[k] for k in classes] == ['background', 'tree', 'tree', 'background']
    s = lc.segment_scores(classes, np.array([0, 1, 0, 0]))
    assert s['tree'] == {'points': 2, 'precision': 1.0, 'recall': 0.5, 'iou': 0.5}
    assert s['all']['right'] == 0.75
    assert lc.drives_of([label(1, 'tree', (0, 0), seen=['x', 'y']),
                         label(2, 'tree', (0, 0), seen=['y', 'z'])]) == ['x', 'y', 'z']
