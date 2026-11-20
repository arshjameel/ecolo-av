"""Tests for tools/twin_world.py: a Ravensim world from a labelled map (TODO 253)."""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip('cv2')       # campus_world's meshes
import twin_world as tw  # noqa: E402


def label(id_, cls, centre, size=(4.0, 4.0), seen=('a',), source='camera', band=None):
    out = {'id': id_, 'class': cls,
           'region': {'centre_m': list(centre), 'size_m': list(size), 'yaw_rad': 0.0},
           'source': source, 'seen_in': list(seen)}
    if band:
        out['band'] = {'min_m': band[0], 'max_m': band[1]}
    return out


def test_parked_cars_and_what_moved_stay_out():
    labels = [label(1, 'building', (0, 0), seen=['a', 'b', 'c']),
              label(2, 'parked_car', (10, 0), seen=['a', 'b', 'c']),
              label(3, 'tree', (20, 0), seen=['a']),                    # 1 of 3: moved?
              label(4, 'pole', (30, 0), seen=['a', 'b']),
              label(5, 'other', (40, 0), seen=['b'], source='passes'),
              {'id': 6, 'class': 'tree', 'region': {'centre_m': [0, 9], 'size_m': [2, 2]},
               'source': 'person'}]
    keep, out = tw.kept(labels)
    assert [lab['id'] for lab in keep] == [1, 4, 6]
    assert {lab['id']: why for lab, why in out} == {
        2: 'parked car: may be gone', 3: 'seen in 1 of 3 drives',
        5: 'seen in one drive only (occupancy)'}
    keep, _ = tw.kept(labels, keep_parked=True, min_share=0.3)
    assert [lab['id'] for lab in keep] == [1, 2, 3, 4, 6]


def test_heights_come_from_the_points_else_the_band():
    b = label(1, 'building', (0, 0), band=(0.5, 3.0))
    points = np.array([[0.5, 0.5, z] for z in np.linspace(1, 12, 50)] + [[50, 50, 30.0]])
    assert tw.top_of(b, points) == pytest.approx(np.percentile(points[:50, 2], 95))
    assert tw.top_of(b, points[:3]) == 3.0, 'too few points: the band'
    assert tw.top_of(label(2, 'pole', (0, 0)), np.zeros((0, 3))) == tw.DEFAULT_TOP['pole']


def test_the_twin_has_a_shape_for_each_label():
    labels = [label(1, 'building', (0, 0), (10, 6), band=(0.5, 8)),
              label(2, 'tree', (20, 0), (3, 3), band=(2, 7)),
              label(3, 'pole', (30, 0), (0.5, 0.5), band=(0.5, 4)),
              label(4, 'other', (40, 0), (5, 0.5), band=(0.6, 1.4))]
    meshes, solids, report = tw.build(labels, np.zeros((0, 3)))
    assert report == {'tree': 1, 'building': 1, 'parked_car': 0, 'pole': 1, 'other': 1}
    tops = {n: max(v[2] for v in m.vertices) for n, m in meshes.items() if m.vertices}
    assert tops['roof'] == pytest.approx(8.0)
    assert 6.5 < tops['leaves'] <= 7.0, 'the low-poly ball has no vertex at its top'
    assert tops['fence'] == pytest.approx(4.0)
    assert len(solids.vertices) > 0
    xs = [v[0] for v in meshes['brick'].vertices]
    assert (min(xs), max(xs)) == pytest.approx((-5.0, 5.0))
    tw.ground(meshes, ((-5, -3), (40, 3)))
    lot = np.array(meshes['lot'].vertices)
    assert lot[:, 0].min() == pytest.approx(-25.0) and lot[:, 1].max() == pytest.approx(23.0)


def test_the_start_is_the_drives_first_pose(tmp_path):
    assert tw.start_pose(tmp_path) == (0.0, 0.0, 0.0)
    (tmp_path / 'poses.tum').write_text('0.0 49.25 -70.5 0 0 0 0.7071068 0.7071068\n'
                                        '1.0 50 -70 0 0 0 0 1\n')
    assert tw.start_pose(tmp_path) == pytest.approx((49.25, -70.5, 1.571))
