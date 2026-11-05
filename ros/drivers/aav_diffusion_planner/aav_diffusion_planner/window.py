"""
What the diffusion model sees and draws: the window, the costmap crop, the paths. No ROS.

Shared by the data generator (tools/diffusion_data.py), the training (training/, in the
learn environment) and the planner node, so all three cut the same window the same way.

  the window   a square round the start and the goal, turned to the car's heading at
               the start: SIZE x SIZE cells, each at least MIN_CELL m (0.25 m, the map's)
  the crop     the costmap under it, one number a cell, 0 free .. 1 lethal or unknown
  the cond     where the start and goal are in it, the goal's heading, may it reverse,
               the cell's size: COND numbers
  a path       POINTS points, each x, y (in the window's half-sides, -1 .. 1),
               cos and sin of the heading (the window's frame), and the gear (+1 or -1)

The costmap is Nav2's OccupancyGrid: 0-98 cost, 99 inscribed, 100 lethal, -1 unknown.
"""

from dataclasses import dataclass
import math

import numpy as np

SIZE = 128              # cells a side
POINTS = 64             # points a path
CHANNELS = 5            # x, y, cos, sin, gear
COND = 8                # start x, y; goal x, y, cos, sin; reversing; cell size (m)
MIN_CELL = 0.25         # m: never finer than the map
MIN_SIDE = SIZE * MIN_CELL  # 32 m
MARGIN = 8.0            # m round the start and the goal: room to swing out

LETHAL = 100
INSCRIBED = 99
UNKNOWN = -1


@dataclass(frozen=True)
class Grid:
    """A costmap in Nav2's OccupancyGrid units: data[row, col], row 0 at the origin's y."""

    data: np.ndarray        # (rows, cols) int8 or int16
    origin_x: float
    origin_y: float
    resolution: float

    def lookup(self, x, y):
        """Return the cells under map points x, y (arrays); off the map: UNKNOWN."""
        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(np.int64)
        rows, cols = self.data.shape
        inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
        out = np.full(np.shape(col), UNKNOWN, dtype=np.int16)
        out[inside] = self.data[row[inside], col[inside]]
        return out


@dataclass(frozen=True)
class Window:
    """The square the model sees: its centre and turn in the map, its side (m)."""

    cx: float
    cy: float
    yaw: float
    side: float

    @property
    def cell(self):
        return self.side / SIZE

    @property
    def half(self):
        return self.side / 2.0

    def to_window(self, x, y):
        """Map points -> the window's frame, in half-sides (-1 .. 1 inside)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(x) - self.cx, np.asarray(y) - self.cy
        return (c * dx + s * dy) / self.half, (-s * dx + c * dy) / self.half

    def to_map(self, u, v):
        """Turn points in the window's frame, in half-sides, into map points."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        u, v = np.asarray(u) * self.half, np.asarray(v) * self.half
        return self.cx + c * u - s * v, self.cy + s * u + c * v


def window_for(start, goal, min_side=MIN_SIDE, margin=MARGIN):
    """Return the window round start and goal ((x, y, yaw), map): the car's heading up."""
    sx, sy, syaw = start
    gx, gy = goal[0], goal[1]
    side = max(min_side, math.hypot(gx - sx, gy - sy) + 2.0 * margin, SIZE * MIN_CELL)
    return Window((sx + gx) / 2.0, (sy + gy) / 2.0, syaw, side)


def crop(grid, win):
    """
    Return the costmap under the window: (SIZE, SIZE) float32, [row v, column u], 0 .. 1.

    Each window cell takes the worst map cell under it (a few samples a side when the
    window's cells are bigger than the map's), so a thin wall can't fall between samples.
    Unknown and off the map count as lethal.
    """
    per = max(1, int(math.ceil(win.cell / grid.resolution)))
    # Sample points inside each window cell: per x per of them, evenly spread.
    offs = (np.arange(per) + 0.5) / per
    idx = np.arange(SIZE)
    u = ((idx[None, :, None] + offs[None, None, :]) / SIZE) * 2.0 - 1.0   # (1, SIZE, per)
    v = ((idx[:, None, None] + offs[None, None, :]) / SIZE) * 2.0 - 1.0   # (SIZE, 1, per)
    uu = np.broadcast_to(u[:, :, :, None], (SIZE, SIZE, per, per))
    vv = np.broadcast_to(v[:, :, None, :], (SIZE, SIZE, per, per))
    x, y = win.to_map(uu, vv)
    cells = grid.lookup(x, y).astype(np.float32)
    cells[cells < 0] = LETHAL
    return (cells.reshape(SIZE, SIZE, -1).max(axis=2) / LETHAL).astype(np.float32)


def cond(win, start, goal, reversing):
    """Return the model's other inputs: (COND,) float32."""
    su, sv = win.to_window(start[0], start[1])
    gu, gv = win.to_window(goal[0], goal[1])
    rel = goal[2] - win.yaw
    return np.array([su, sv, gu, gv, math.cos(rel), math.sin(rel),
                     1.0 if reversing else 0.0, win.cell], dtype=np.float32)


def gears(poses):
    """
    Return each pose's gear: +1 driving forward, -1 backward, from heading and motion.

    poses: (N, 3) x, y, yaw. A pose takes the gear of the step after it (the last one, the
    step before); a step too short to tell keeps the gear before it.
    """
    poses = np.asarray(poses, dtype=np.float64)
    n = len(poses)
    if n < 2:
        return np.ones(n)
    d = np.diff(poses[:, :2], axis=0)
    along = d[:, 0] * np.cos(poses[:-1, 2]) + d[:, 1] * np.sin(poses[:-1, 2])
    step = np.ones(n - 1)
    last = 1.0
    for i, a in enumerate(along):
        if abs(a) > 1e-4:
            last = 1.0 if a > 0 else -1.0
        step[i] = last
    # Steps too short to tell before the first real one take that one's gear.
    first = next((i for i, a in enumerate(along) if abs(a) > 1e-4), None)
    if first is not None:
        step[:first] = step[first]
    return np.append(step, step[-1])


def resample(poses, n=POINTS):
    """
    Return n poses evenly spaced along the path (by distance), and their gears.

    poses: (N, 3) x, y, yaw. Headings are interpolated the short way round. A path of one
    point (or no length) gives n copies of it.
    """
    poses = np.asarray(poses, dtype=np.float64)
    g = gears(poses)
    seg = np.hypot(*np.diff(poses[:, :2], axis=0).T) if len(poses) > 1 else np.zeros(0)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-9:
        return np.repeat(poses[:1], n, axis=0), np.repeat(g[:1], n)
    want = np.linspace(0.0, s[-1], n)
    yaw = np.unwrap(poses[:, 2])
    out = np.stack([np.interp(want, s, poses[:, 0]), np.interp(want, s, poses[:, 1]),
                    np.interp(want, s, yaw)], axis=1)
    out[:, 2] = np.arctan2(np.sin(out[:, 2]), np.cos(out[:, 2]))
    # Each new point takes the gear of the step it lies on.
    which = np.clip(np.searchsorted(s, want, side='right') - 1, 0, len(g) - 1)
    return out, g[which]


def encode_path(win, poses, n=POINTS):
    """Return a map-frame path as the model's target: (n, CHANNELS) float32."""
    pts, g = resample(poses, n)
    u, v = win.to_window(pts[:, 0], pts[:, 1])
    rel = pts[:, 2] - win.yaw
    return np.stack([u, v, np.cos(rel), np.sin(rel), g], axis=1).astype(np.float32)


def decode_paths(win, out):
    """
    Return the model's paths in the map: (K, n, 3) x, y, yaw, and (K, n) gears (+1/-1).

    out: (K, n, CHANNELS). The heading is the angle of its cos and sin.
    """
    out = np.asarray(out, dtype=np.float64)
    x, y = win.to_map(out[..., 0], out[..., 1])
    yaw = np.arctan2(out[..., 3], out[..., 2]) + win.yaw
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    return np.stack([x, y, yaw], axis=-1), np.where(out[..., 4] < 0.0, -1.0, 1.0)


def yaw_of(qz, qw):
    """Return a yaw (rad) from a quaternion's z and w (a turn about z only)."""
    return 2.0 * math.atan2(qz, qw)


SMOOTH = 5              # points in the moving average (smooth())


def smooth(path, width=SMOOTH):
    """
    Return a drawn path (N, 3) smoothed.

    x and y by a moving average, its ends kept; each heading from the direction of travel
    (backwards where the model's heading says the car reverses). The ends keep the
    model's headings (pinned to start and goal).

    The model's points wobble a little; their headings disagree with their travel. Without
    this, 0.9 % of samples passed the guard on the held-back maps; with it, 36 % (2 Nov).
    """
    path = np.asarray(path, dtype=np.float64)
    out = path.copy()
    if width > 1 and len(path) > width:
        kernel = np.ones(width) / width
        for c in (0, 1):
            out[:, c] = np.convolve(np.pad(path[:, c], width // 2, mode='edge'), kernel,
                                    mode='valid')
        out[0, :2], out[-1, :2] = path[0, :2], path[-1, :2]
    d = np.gradient(out[:, :2], axis=0)
    travel = np.arctan2(d[:, 1], d[:, 0])
    forward = np.cos(travel - path[:, 2]) >= 0.0
    yaw = np.where(forward, travel, travel + math.pi)
    yaw[0], yaw[-1] = path[0, 2], path[-1, 2]
    out[:, 2] = np.arctan2(np.sin(yaw), np.cos(yaw))
    return out
