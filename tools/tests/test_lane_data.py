"""tools/lane_data.py: the learned lane finder's labels, decoding and ground maths."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lane_data as L  # noqa: E402, I100

CAM = L.Camera(343.0, 343.0, 320.0, 180.0, (1.75, 0.29, 1.55), (0.0, 0.0, 0.0))
ROAD = [L.Paint(L.densify([(-20.0, y), (100.0, y)]), L.WHITE) for y in (1.75, -1.75)]


def test_the_camera_is_read_from_the_config():
    cam = L.load_camera()
    assert (cam.fx, cam.cx, cam.cy) == (343.0, 320.0, 180.0)
    assert len(cam.xyz) == 3


def test_ground_undoes_project():
    pts = np.array([[5.0, 1.0, 0.0], [12.0, -2.0, 0.0], [30.0, 0.5, 0.0]])
    for rpy in [(0.0, 0.0, 0.0), (0.02, 0.05, -0.03)]:
        cam = L.Camera(343.0, 343.0, 320.0, 180.0, (1.75, 0.29, 1.55), rpy)
        back = L.ground(L.project(pts, cam), cam)
        assert np.allclose(back, pts[:, :2], atol=1e-9)


def test_above_the_horizon_is_not_ground():
    assert np.isnan(L.ground(np.array([[320.0, 100.0]]), CAM)).all()


def test_a_straight_lane_is_labelled_in_its_slots():
    cols, colour, gt = L.label(ROAD, (0.0, 0.0, 0.0), CAM)
    assert np.isnan(cols[0]).all() and np.isnan(cols[3]).all()
    assert np.allclose(gt[1], 1.75) and np.allclose(gt[2], -1.75)
    for k in (1, 2):
        ok = ~np.isnan(cols[k])
        assert ok.sum() > 20
        back = L.ground(np.column_stack([cols[k][ok], np.asarray(L.ROWS)[ok]]), CAM)
        assert np.allclose(back[:, 1], gt[k][0], atol=1e-6)
    # The left line is left in the picture, and nearer the middle further up.
    assert np.nanmax(cols[1]) < CAM.cx < np.nanmin(cols[2])
    assert list(colour) == [L.WHITE] * 4


def test_a_car_turned_towards_a_line_keeps_it_on_its_side():
    # Turned 0.25 rad left, the left line crosses y = 0 ahead: still the left line.
    cols, _, gt = L.label(ROAD, (0.0, 0.5, 0.25), CAM)
    assert not np.isnan(cols[1]).all() and not np.isnan(cols[2]).all()
    assert np.isnan(cols[0]).all()


def test_driving_the_other_way_swaps_the_lines():
    _, _, gt = L.label(ROAD, (50.0, 0.0, math.pi), CAM)
    assert np.allclose(gt[1], 1.75) and np.allclose(gt[2], -1.75)


def test_neighbours_fill_the_outer_slots():
    paint = ROAD + [L.Paint(L.densify([(-20.0, 5.25), (100.0, 5.25)]), L.YELLOW)]
    _, colour, gt = L.label(paint, (0.0, 0.0, 0.0), CAM)
    assert np.allclose(gt[0], 5.25) and colour[0] == L.YELLOW
    assert np.isnan(gt[3]).all()


def test_a_crossing_road_in_view_is_found():
    across = [L.Paint(L.densify([(15.0, -10.0), (15.0, 10.0)]), L.WHITE)]
    assert L.crossing(across, (0.0, 0.0, 0.0))
    assert not L.crossing(across, (30.0, 0.0, 0.0))
    assert not L.crossing(ROAD, (0.0, 0.0, 0.1))


def test_poses_stay_on_the_tarmac():
    import random
    roads, paint = L.painted(L.ROAD_LANES, centre=False)
    poses = L.sample_poses(roads, 50, random.Random(3), paint)
    assert len(poses) == 50
    for x, y, yaw, road in poses:
        assert -20.0 <= x <= 100.0 and abs(y) < 1.75 and road == 0


def test_decode_reads_the_column_and_skips_absent_rows():
    cells, rows = L.CELLS, 3
    loc = np.full((cells, rows, 4), -10.0)
    exist = np.zeros((2, rows, 4))
    exist[1, :, 1] = 5.0          # line 1 in every row
    loc[40, :, 1] = 10.0
    loc[41, 1, 1] = 10.0          # row 1: halfway between 40 and 41
    px = L.decode(loc, exist)
    assert np.isnan(px[[0, 2, 3]]).all()
    step = L.WIDTH / cells
    assert px[1, 0] == pytest.approx(40.5 * step, abs=1e-3)
    assert px[1, 1] == pytest.approx(41.0 * step, abs=1e-3)


def test_fit_finds_a_parabola_and_refuses_too_little():
    x = np.linspace(4, 20, 12)
    pts = np.column_stack([x, 0.01 * x * x - 0.1 * x + 1.5])
    assert np.allclose(L.fit(pts, 4.0, 25.0), [0.01, -0.1, 1.5], atol=1e-9)
    assert L.fit(pts[:3], 4.0, 25.0) is None
    short = np.column_stack([np.linspace(5, 7, 6), np.full(6, 1.0)])
    assert np.allclose(L.fit(short, 4.0, 25.0), [0.0, 0.0, 1.0], atol=1e-9)


def test_offset_line_matches_the_campus_worlds():
    cw = pytest.importorskip('campus_world', reason='campus_world needs cv2 and aav_carla')
    pts = [(0.0, 0.0), (10.0, 0.0), (20.0, 5.0), (25.0, 15.0)]
    assert np.allclose(L.offset_line(pts, 1.75), cw.offset_line(pts, 1.75))


def test_a_line_carried_on_by_the_next_roads_paint_is_one_line():
    # The left line cut in two at x = 6 m (the lane map's roads change there).
    paint = [L.Paint(L.densify([(-20.0, 1.75), (6.0, 1.75)]), L.WHITE),
             L.Paint(L.densify([(6.0, 1.75), (100.0, 1.75)]), L.WHITE), ROAD[1]]
    cols, _, gt = L.label(paint, (0.0, 0.0, 0.0), CAM)
    whole, _, _ = L.label(ROAD, (0.0, 0.0, 0.0), CAM)
    assert np.allclose(np.nan_to_num(cols, nan=-1), np.nan_to_num(whole, nan=-1))
    assert np.isnan(cols[0]).all()


def test_grass_ahead_is_not_tarmac():
    tarmac = np.full((144, 480, 3), 90, np.uint8)
    grass = tarmac.copy()
    grass[..., 1] = 140
    assert L.on_tarmac(tarmac) and not L.on_tarmac(grass)
