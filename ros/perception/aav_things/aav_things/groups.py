"""
Group the lidar's cubes into things: plain Euclidean clustering on a flat grid. No ROS.

aav_voxels sends the centre of every 0.25 m cube that holds an obstacle point. Seen from
above, cubes closer than `gap` belong to the same thing:

    · · ■ ■ · · · · ■          two things: the gap between them is wider than `gap`
    · · ■ ■ ■ · · · ■
    · · · ■ · · · · ■

A group with too few cubes is noise; one larger than `max_size` across is a wall or a
building, which the map holds, not the list of things.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GroupShape:
    """How cubes become groups."""

    size: float = 0.25        # m: one cube's edge (aav_voxels' size)
    gap: float = 0.6          # m: cubes this close (centre to centre, flat) join a group
    min_cubes: int = 2        # fewer is noise
    max_size: float = 8.0     # m: a group longer than this is a wall, not a thing


@dataclass(frozen=True)
class Group:
    """One group of cubes, in the frame the cubes came in."""

    x: float           # m: the middle of its outline
    y: float
    length: float      # m along x (the outline of the cubes' centres, plus one cube)
    width: float       # m along y
    height: float      # m: the highest cube's top
    cubes: int


def group(points: np.ndarray, shape: GroupShape = GroupShape()) -> list[Group]:
    """
    Return the groups in `points` (N x 3: each cube's centre, x y z in m).

    Cubes are joined when their flat distance is at most `gap`: a flood fill over the
    grid cells within reach, so the cost grows with the cube count, not the area.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        return []
    cells = np.floor(points[:, :2] / shape.size).astype(int)
    where: dict[tuple[int, int], list[int]] = {}
    for i, (cx, cy) in enumerate(map(tuple, cells)):
        where.setdefault((cx, cy), []).append(i)
    reach = max(1, math.ceil(shape.gap / shape.size))
    steps = [(dx, dy) for dx in range(-reach, reach + 1) for dy in range(-reach, reach + 1)
             if (dx, dy) != (0, 0)]
    gap2 = shape.gap * shape.gap
    label = {}
    groups = []
    for start in where:
        if start in label:
            continue
        label[start] = len(groups)
        members, todo = [start], [start]
        while todo:
            cell = todo.pop()
            here = points[where[cell][0], :2]
            for dx, dy in steps:
                near = (cell[0] + dx, cell[1] + dy)
                if near in where and near not in label:
                    there = points[where[near][0], :2]
                    if float(np.sum((here - there) ** 2)) <= gap2:
                        label[near] = len(groups)
                        members.append(near)
                        todo.append(near)
        groups.append(members)
    found = []
    for members in groups:
        rows = [i for cell in members for i in where[cell]]
        if len(rows) < shape.min_cubes:
            continue
        p = points[rows]
        low, high = p[:, :2].min(axis=0), p[:, :2].max(axis=0)
        length, width = high - low + shape.size
        if max(length, width) > shape.max_size:
            continue
        middle = (low + high) / 2.0
        found.append(Group(x=float(middle[0]), y=float(middle[1]), length=float(length),
                           width=float(width), height=float(p[:, 2].max() + shape.size / 2),
                           cubes=len(rows)))
    return found


def to_frame(groups: list[Group], x: float, y: float, yaw: float) -> list[Group]:
    """
    Return the groups moved into another frame, given the cubes' frame's pose in it.

    Only the middle turns; length and width stay along the new frame's axes as the outline
    of the turned box (an upright box that holds the turned one).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    moved = []
    for g in groups:
        moved.append(Group(
            x=x + c * g.x - s * g.y, y=y + s * g.x + c * g.y,
            length=abs(c) * g.length + abs(s) * g.width,
            width=abs(s) * g.length + abs(c) * g.width,
            height=g.height, cubes=g.cubes))
    return moved
