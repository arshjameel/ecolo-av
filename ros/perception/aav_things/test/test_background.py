"""The saved map as the background: walls drop out, new things stay (background.py)."""

import math

from aav_things.background import Background, new_only
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
