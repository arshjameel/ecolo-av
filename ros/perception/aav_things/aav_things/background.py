"""
The saved map as the background: cubes on it are the walls again, not new things. No ROS.

Ben, 14 Nov: "NDT can give us the stationary things ... then all we need is occupancy graph
to track poepole". Step 5 of the three levels (docs/designs/2026-11-15_three-levels.md):

    live cubes ──► on a taken cell of the saved map (± slack for pose error)?
                      yes ─► the wall again: dropped before grouping
                      no  ─► a new occupancy: grouped and tracked as before

Only the tracker uses this. The stop zone and the costmaps see every point, always.
"""

from dataclasses import dataclass
import math

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
        taken = cells >= TAKEN
        reach = max(0, math.ceil(slack / resolution - 1e-9))
        grown = taken.copy()
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                if dx or dy:
                    grown |= _shifted(taken, dx, dy)
        return cls(origin_x, origin_y, resolution, grown)

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


def _shifted(a: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """Return `a` moved by (dx, dy) cells, filled with False."""
    out = np.zeros_like(a)
    h, w = a.shape
    out[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = \
        a[max(-dy, 0):h + min(-dy, 0), max(-dx, 0):w + min(-dx, 0)]
    return out


def new_only(points: np.ndarray, pose, background: Background) -> np.ndarray:
    """
    Return the (N, 3) cubes, in the car's frame, that are not on the saved map.

    pose: (x, y, yaw) of the cubes' frame in the map's frame.
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                          y + s * points[:, 0] + c * points[:, 1]])
    return points[~background.on_map(xy)]
