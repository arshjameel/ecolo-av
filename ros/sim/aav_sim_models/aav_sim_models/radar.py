"""
The Ravensim car's radar stand-in, made to look like the TI radar. No ROS imports.

Ravensim's radar stand-in (a lidar shaped like the radar) sends a point for every ray:
where a ray hit nothing, the point is endless (inf, or NaN where inf meets 0), and its
fields are Ravensim's own (x, y, z, intensity, ring). The TI driver (aav_radar_ti) sends
only what it saw, with x, y, z, intensity and velocity, the radial speed. These turn the
first into the second, so radar_clusters, the costmap and the stop zone read the
same kind of cloud in Ravensim as on the car, and PCL finds every field it expects.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# The TI driver's fields, in its order (aav_radar_ti pc_publisher.py), all float32.
FIELDS = ('x', 'y', 'z', 'intensity', 'velocity')


def hit(xyz: np.ndarray, max_range: float) -> np.ndarray:
    """Return which rays hit something: finite, no farther than max_range (m). A mask."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    near = np.zeros(len(xyz), dtype=bool)
    near[finite] = np.linalg.norm(xyz[finite], axis=1) <= max_range
    return near


def returns(xyz: np.ndarray, max_range: float) -> np.ndarray:
    """Return the rays that hit something: the finite points no farther than max_range (m)."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    return xyz[hit(xyz, max_range)]


def ti_points(xyz: np.ndarray, speed: float, intensity: float = 1.0,
              own: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Return each point as the TI driver's x, y, z, intensity and velocity (N x 5, float32).

    A point's radial speed is the car's own, seen along the line from the radar to it:
    negative while the car closes on it, as the TI radar reports; plus `own`, each point's
    own speed along that line over the ground (+ away), where it moves: our simulator's
    cloud carries it (a person walking, TODO 194); the still world doesn't move. The car's
    turning is left out, as the bench's pretend radar leaves it out.
    """
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    distance = np.linalg.norm(xyz, axis=1)
    along = np.divide(xyz[:, 0], distance, out=np.zeros_like(distance), where=distance > 0)
    out = np.empty((len(xyz), len(FIELDS)), dtype=np.float32)
    out[:, :3] = xyz
    out[:, 3] = intensity
    out[:, 4] = -speed * along
    if own is not None:
        out[:, 4] += np.asarray(own, dtype=np.float32).reshape(-1)
    return out
