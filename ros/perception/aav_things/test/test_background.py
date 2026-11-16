"""The saved map as the background: walls drop out, new things stay (background.py)."""

import math
import os
import tempfile

from aav_things.background import (Background, ChunkBackground, new_only, read_grid,
                                   read_pcd)
import numpy as np


def lot():
    """Return a 20 x 20 m map at 0.25 m from (-10, -10): free, with a wall along x = 5."""
    width = height = 80
    data = np.zeros((height, width), dtype=np.int8)
    data[:, 60] = 100                       # x from 5.0 to 5.25
    return Background.from_grid(data.ravel(), width, height, 0.25, -10.0, -10.0, slack=0.25)


def test_a_cube_on_the_wall_drops_and_one_in_the_open_stays():
    bg = lot()
    cubes = np.array([[5.1, 0.0, 1.0], [2.0, 0.0, 1.0]])
    kept = new_only(cubes, (0.0, 0.0, 0.0), bg)
    assert kept.tolist() == [[2.0, 0.0, 1.0]]


def test_the_slack_takes_a_small_pose_error():
    bg = lot()
    # 0.2 m short of the wall's face (the pose a little off): still the wall.
    assert new_only(np.array([[4.85, 3.0, 1.0]]), (0.0, 0.0, 0.0), bg).size == 0
    # 0.6 m short: someone standing at the wall, kept.
    assert len(new_only(np.array([[4.4, 3.0, 1.0]]), (0.0, 0.0, 0.0), bg)) == 1


def test_the_car_pose_moves_the_cubes_onto_the_map():
    bg = lot()
    # The car at (3, 0) facing +y: a cube 2.1 m to its right is at x = 5.1, the wall.
    cube = np.array([[0.0, -2.1, 1.0]])
    assert new_only(cube, (3.0, 0.0, math.pi / 2), bg).size == 0
    assert len(new_only(cube, (0.0, 0.0, math.pi / 2), bg)) == 1


def test_off_the_map_is_never_background():
    bg = lot()
    assert len(new_only(np.array([[50.0, 50.0, 1.0]]), (0.0, 0.0, 0.0), bg)) == 1
    assert new_only(np.zeros((0, 3)), (0.0, 0.0, 0.0), bg).shape == (0, 3)


def test_points_make_a_background_with_slack():
    wall = np.array([[5.1, y] for y in np.arange(-3.0, 3.0, 0.1)])
    bg = Background.from_points(wall, -10.0, -10.0, 20.0, slack=0.25)
    cubes = np.array([[5.1, 0.0, 1.0], [4.85, 1.0, 1.0], [4.3, 0.0, 1.0], [5.1, 5.0, 1.0]])
    kept = new_only(cubes, (0.0, 0.0, 0.0), bg)
    assert kept.tolist() == [[4.3, 0.0, 1.0], [5.1, 5.0, 1.0]]


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4')
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii') + points.tobytes())


def test_chunks_load_round_the_car_and_again_in_a_new_chunk():
    with tempfile.TemporaryDirectory() as top:
        folder = os.path.join(top, 'ndt')
        os.mkdir(folder)
        with open(os.path.join(top, 'chunks.yaml'), 'w') as f:
            f.write('origin: [-64, -64]\nsize: 64\n')
        # A wall in chunk (1, 1) (x 0-64), a far wall in chunk (4, 1), and the ground.
        write_pcd(os.path.join(folder, '1_1.pcd'),
                  [[10.0, y, 1.0] for y in range(0, 20)] + [[30.0, 5.0, 0.0]])
        write_pcd(os.path.join(folder, '4_1.pcd'), [[200.0, 5.0, 1.0]])
        assert read_pcd(os.path.join(folder, '1_1.pcd')).shape == (21, 3)
        origin_x, origin_y, size = read_grid(folder)
        assert (origin_x, origin_y, size) == (-64.0, -64.0, 64.0)
        chunks = ChunkBackground(folder, origin_x, origin_y, size)
        bg = chunks.update(5.0, 5.0)
        cubes = np.array([[10.0, 5.0, 1.0], [30.0, 5.0, 1.0], [200.0, 5.0, 1.0]])
        # The wall drops; the ground's point is not a wall; the far chunk isn't loaded.
        assert new_only(cubes, (0.0, 0.0, 0.0), bg).tolist() == [[30.0, 5.0, 1.0],
                                                                   [200.0, 5.0, 1.0]]
        assert chunks.update(20.0, 30.0) is bg and chunks.loads == 1    # the same chunk
        bg = chunks.update(170.0, 5.0)                                   # chunk (3, 1)
        assert chunks.loads == 2
        assert new_only(cubes, (0.0, 0.0, 0.0), bg).tolist() == [[10.0, 5.0, 1.0],
                                                                   [30.0, 5.0, 1.0]]
