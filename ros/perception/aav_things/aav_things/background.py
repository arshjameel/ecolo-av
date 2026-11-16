"""
The saved map as the background: cubes on it are the walls again, not new things. No ROS.

Ben, 14 Nov: "NDT can give us the stationary things ... then all we need is occupancy graph
to track poepole". Step 5 of the three levels (docs/designs/2026-11-15_three-levels.md):

    live cubes ──► on a taken cell of the saved map (± slack for pose error)?
                      yes ─► the wall again: dropped before grouping
                      no  ─► a new occupancy: grouped and tracked as before

Two backgrounds, the same test (`Background.on_map`):

| From | What is "taken" | Made by |
|---|---|---|
| `/map` (the occupancy grid) | every wall cell; on the campus, all that isn't road | `Background.from_grid` |
| `/map`, low cubes only (`below`) | the same cells, but only cubes under `below` m drop: kerbs and raised grass go, a person's body stays | `new_only(..., below=)` |
| the NDT map's chunks (`ChunkBackground`) | only where the saved 3D map has points over `min_z`: buildings, poles, not grass or paths | `Background.from_points`, the 3 x 3 chunks round the car |

Only the tracker uses this. The stop zone and the costmaps see every point, always.
"""

from dataclasses import dataclass
import math
import os
from typing import Optional

import numpy as np

TAKEN = 65      # ROS OccupancyGrid: this or more is a wall (map_server's occupied_thresh)


@dataclass(frozen=True)
class Background:
    """The saved map's taken cells, grown by the slack: True where a cube is the map."""

    origin_x: float
    origin_y: float
    resolution: float
    taken: np.ndarray          # (height, width) bool, row 0 at origin_y

    @classmethod
    def from_grid(cls, data, width: int, height: int, resolution: float,
                  origin_x: float, origin_y: float, slack: float = 0.25):
        """Make one from a ROS OccupancyGrid's cells, growing each wall by `slack` m."""
        cells = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls(origin_x, origin_y, resolution, _grown(cells >= TAKEN, resolution, slack))

    @classmethod
    def from_points(cls, xy: np.ndarray, origin_x: float, origin_y: float, side: float,
                    resolution: float = 0.25, slack: float = 0.25):
        """Make one `side` m square from (origin_x, origin_y): taken where a point falls."""
        n = int(round(side / resolution))
        taken = np.zeros((n, n), dtype=bool)
        if len(xy):
            col = np.floor((xy[:, 0] - origin_x) / resolution).astype(np.int64)
            row = np.floor((xy[:, 1] - origin_y) / resolution).astype(np.int64)
            inside = (col >= 0) & (col < n) & (row >= 0) & (row < n)
            taken[row[inside], col[inside]] = True
        return cls(origin_x, origin_y, resolution, _grown(taken, resolution, slack))

    def on_map(self, xy: np.ndarray) -> np.ndarray:
        """Return, for each (x, y) in the map frame, whether it is on a (grown) wall."""
        if len(xy) == 0:
            return np.zeros(0, dtype=bool)
        col = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        height, width = self.taken.shape
        inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        out = np.zeros(len(xy), dtype=bool)
        out[inside] = self.taken[row[inside], col[inside]]
        return out


def _grown(taken: np.ndarray, resolution: float, slack: float) -> np.ndarray:
    """Return `taken` with every taken cell grown by `slack` m (a square)."""
    reach = max(0, math.ceil(slack / resolution - 1e-9))
    grown = taken.copy()
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            if dx or dy:
                grown |= _shifted(taken, dx, dy)
    return grown


def _shifted(a: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """Return `a` moved by (dx, dy) cells, filled with False."""
    out = np.zeros_like(a)
    h, w = a.shape
    out[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = \
        a[max(-dy, 0):h + min(-dy, 0), max(-dx, 0):w + min(-dx, 0)]
    return out


def new_only(points: np.ndarray, pose, background: Background,
             below: Optional[float] = None) -> np.ndarray:
    """
    Return the (N, 3) cubes, in the car's frame, that are not on the saved map.

    pose: (x, y, yaw) of the cubes' frame in the map's frame. below (m, the cubes' z): if
    given, only cubes lower than it may be dropped; a taller one is kept even on the map,
    so a person on the grass beside the road keeps their body (`/map` marks the grass).
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                          y + s * points[:, 0] + c * points[:, 1]])
    drop = background.on_map(xy)
    if below is not None:
        drop &= points[:, 2] < below
    return points[~drop]


def read_pcd(path: str) -> np.ndarray:
    """Return a binary PCD's x, y, z as N x 3 (as ndt-core and tools/ndt_map.py write them)."""
    with open(path, 'rb') as f:
        blob = f.read()
    end = blob.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = dict(line.split(' ', 1) for line in blob[:end].decode('ascii').splitlines()
                  if line and not line.startswith('#'))
    if header.get('FIELDS', '').split() != ['x', 'y', 'z']:
        raise ValueError(f'{path}: fields {header.get("FIELDS")}, not x y z')
    n = int(header['POINTS'])
    return np.frombuffer(blob[end:], dtype='<f4', count=3 * n).reshape(n, 3).astype(float)


def read_grid(folder: str):
    """
    Return (origin_x, origin_y, size) of the chunk grid a folder of NDT chunks was cut on.

    Its chunks.yaml is beside it (maps/campus/chunks/chunks.yaml for .../chunks/ndt), as
    aav_ndt's ndt_chunks:= reads it.
    """
    import yaml
    with open(os.path.join(os.path.dirname(os.path.abspath(folder)), 'chunks.yaml')) as f:
        index = yaml.safe_load(f)
    return float(index['origin'][0]), float(index['origin'][1]), float(index['size'])


class ChunkBackground:
    """
    The NDT map's chunks as the background: the 3 x 3 round the car, read again as it
    crosses into another chunk (TODO 175 step 5, the design's form of the subtraction).

    Points lower than `min_z` (m, the map frame) are the ground and taken as nothing.
    """

    def __init__(self, folder: str, origin_x: float, origin_y: float, size: float,
                 resolution: float = 0.25, slack: float = 0.25, min_z: float = 0.3,
                 read=read_pcd):
        self.folder, self.origin, self.size = folder, (origin_x, origin_y), size
        self.resolution, self.slack, self.min_z, self.read = resolution, slack, min_z, read
        self.here: Optional[tuple] = None
        self.background: Optional[Background] = None
        self.loads = 0                  # times the 3 x 3 was read

    def key_at(self, x: float, y: float) -> tuple:
        """Return the chunk (i, j) under (x, y), as map-chunks names it."""
        return (math.floor((x - self.origin[0]) / self.size),
                math.floor((y - self.origin[1]) / self.size))

    def update(self, x: float, y: float) -> Optional[Background]:
        """Return the background round (x, y); read the chunks again in a new chunk."""
        key = self.key_at(x, y)
        if key != self.here:
            parts = []
            for i in range(key[0] - 1, key[0] + 2):
                for j in range(key[1] - 1, key[1] + 2):
                    path = os.path.join(self.folder, f'{i}_{j}.pcd')
                    if os.path.exists(path):
                        points = self.read(path)
                        parts.append(points[points[:, 2] >= self.min_z, :2])
            xy = np.concatenate(parts) if parts else np.zeros((0, 2))
            corner_x = self.origin[0] + (key[0] - 1) * self.size
            corner_y = self.origin[1] + (key[1] - 1) * self.size
            self.background = Background.from_points(xy, corner_x, corner_y, 3 * self.size,
                                                     self.resolution, self.slack)
            self.here = key
            self.loads += 1
        return self.background
