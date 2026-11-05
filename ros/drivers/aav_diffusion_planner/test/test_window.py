"""The window, the crop and the path codes: what the model sees and draws."""

import io
import math

from aav_diffusion_planner import window, wire
import numpy as np
import pytest


def empty_grid(value=0, rows=240, cols=320):
    return window.Grid(np.full((rows, cols), value, np.int8), -10.0, -10.0, 0.25)


def test_window_fits_start_and_goal_with_room():
    win = window.window_for((0.0, 0.0, 0.0), (40.0, 0.0, 0.0))
    assert win.side == pytest.approx(40.0 + 2 * window.MARGIN)
    assert (win.cx, win.cy) == (20.0, 0.0)
    near = window.window_for((0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert near.side == window.MIN_SIDE and near.cell == window.MIN_CELL


def test_to_window_and_back():
    win = window.Window(5.0, -3.0, 0.7, 40.0)
    x, y = np.array([1.0, 30.0, -7.0]), np.array([2.0, 4.0, -9.0])
    u, v = win.to_window(x, y)
    bx, by = win.to_map(u, v)
    assert np.allclose(bx, x) and np.allclose(by, y)


def test_the_car_heading_is_up_the_window():
    # The car faces north: a point 10 m north of it is straight ahead, +u.
    win = window.window_for((0.0, 0.0, math.pi / 2), (0.0, 10.0, 0.0))
    u0, v0 = win.to_window(0.0, 0.0)
    u1, v1 = win.to_window(0.0, 10.0)
    assert u1 - u0 == pytest.approx(10.0 / win.half)
    assert v1 - v0 == pytest.approx(0.0, abs=1e-9)


def test_crop_sees_a_wall_and_counts_off_the_map_as_lethal():
    data = np.zeros((240, 320), np.int8)
    data[:, 120] = 100            # a wall one cell thick at x = 20 m
    grid = window.Grid(data, -10.0, -10.0, 0.25)
    # A big window (cells of 0.5 m) still sees the thin wall: each cell takes its worst.
    win = window.window_for((0.0, 10.0, 0.0), (48.0, 10.0, 0.0))
    assert win.cell > 0.25
    c = window.crop(grid, win)
    assert c.shape == (window.SIZE, window.SIZE)
    col = int((20.1 - (win.cx - win.half)) / win.cell)
    assert c[window.SIZE // 2, col] == 1.0
    assert c[window.SIZE // 2, window.SIZE // 2 - 30] == 0.0
    # Beyond the map's edge (y < -10 m): lethal.
    win = window.window_for((0.0, -10.0, 0.0), (0.0, -10.0, 0.0))
    assert window.crop(grid, win)[0, window.SIZE // 2] == 1.0


def test_unknown_cells_are_lethal_in_the_crop():
    c = window.crop(empty_grid(-1), window.window_for((0, 0, 0), (5, 0, 0)))
    assert np.all(c == 1.0)


def test_gears_from_heading_and_motion():
    # Forward 3 m east, then backward 2 m (still facing east).
    poses = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [2, 0, 0], [1, 0, 0]], float)
    assert list(window.gears(poses)) == [1, 1, 1, -1, -1, -1]


def test_resample_keeps_ends_and_spacing():
    poses = np.array([[0, 0, 0], [10, 0, 0], [10, 10, math.pi / 2]], float)
    pts, g = window.resample(poses, 21)
    assert np.allclose(pts[0], poses[0]) and np.allclose(pts[-1], poses[-1])
    assert np.allclose(np.hypot(*np.diff(pts[:, :2], axis=0).T), 1.0)
    assert np.all(g == 1)


def test_encode_then_decode_gives_the_path_back():
    poses = np.array([[0, 0, 0.3], [5, 2, 0.5], [9, 8, 1.2], [6, 14, 2.5]], float)
    start, goal = tuple(poses[0]), tuple(poses[-1])
    win = window.window_for(start, goal)
    code = window.encode_path(win, poses)
    assert code.shape == (window.POINTS, window.CHANNELS)
    back, gear = window.decode_paths(win, code[None])
    want, _ = window.resample(poses)
    assert np.allclose(back[0, :, :2], want[:, :2], atol=1e-4)
    assert np.allclose(np.cos(back[0, :, 2] - want[:, 2]), 1.0, atol=1e-6)
    # The cond holds the start and goal where the path's ends are.
    c = window.cond(win, start, goal, reversing=True)
    assert np.allclose(c[0:2], code[0, :2], atol=1e-6)
    assert np.allclose(c[2:4], code[-1, :2], atol=1e-6)
    assert c[6] == 1.0 and c[7] == pytest.approx(win.cell)


def test_wire_round_trip():
    crop = np.random.rand(window.SIZE, window.SIZE).astype(np.float32)
    cond = np.arange(window.COND, dtype=np.float32)
    noise = np.random.randn(16, window.POINTS, window.CHANNELS).astype(np.float32)
    seq, c, d, n = wire.read_request(io.BytesIO(wire.encode_request(7, crop, cond, noise)))
    assert seq == 7 and np.array_equal(c[0, 0], crop) and np.array_equal(d[0], cond)
    assert np.array_equal(n, noise)
    seq, paths, ms = wire.read_reply(io.BytesIO(wire.encode_reply(7, noise, 12.5)))
    assert seq == 7 and np.array_equal(paths, noise) and ms == pytest.approx(12.5)
    assert wire.read_reply(io.BytesIO(wire.encode_reply(8, None, 1.0)))[1] is None
    with pytest.raises(EOFError):
        wire.read_reply(io.BytesIO(wire.encode_reply(7, noise, 1.0)[:-3]))


def test_smooth_keeps_the_ends_and_takes_headings_from_travel():
    xs = np.linspace(0.0, 20.0, 64)
    wobble = np.stack([xs, 0.1 * np.sin(xs * 7.0), np.full(64, 0.3)], axis=1)
    wobble[0, 1] = wobble[-1, 1] = 0.0
    out = window.smooth(wobble)
    assert np.allclose(out[0], wobble[0]) and np.allclose(out[-1], wobble[-1])
    assert np.abs(out[1:-1, 1]).max() < np.abs(wobble[1:-1, 1]).max()
    assert np.abs(out[5:-5, 2]).max() < 0.1          # east, as it travels, not 0.3
    # Backing up east to west, facing east: the headings stay east.
    back = np.stack([xs[::-1], np.zeros(64), np.zeros(64)], axis=1)
    assert np.allclose(np.cos(window.smooth(back)[:, 2]), 1.0)
