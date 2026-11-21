#!/usr/bin/env python3
"""
The maths behind map-drive (TODO 250): a map from a recorded drive. No ROS, no SLAM library.

tools/map_drive.py runs KISS-SLAM on a drive's lidar; everything around it is here, and
tested in tools/tests/test_slam_map.py:

- flatten(), compose(), at(): SLAM's 3D poses as the car's x, y, yaw (the stack is 2D: the
  EKFs and NDT give x, y, yaw), and a pose at any time between two.
- utm(), gps_to_map(): latitude and longitude into the map frame as navsat_transform does
  it (UTM, less the datum's UTM, turned by the meridian convergence).
- fit_2d(), georeference(): the SLAM path turned and moved onto the good GPS fixes (least
  squares), else onto the drive's start pose, else left where it is.
- write_tum(): poses in the TUM format (t x y z qx qy qz qw), one a line (ndt_map.read_tum
  reads them back).
- Occupancy: the 2D map for the planner: points 0.3-2 m above the ground are occupied; the
  lidar's rays up to them (or to the ground they hit) are free; the rest unknown. Written
  as a map server's PGM and YAML (trinary).
- distances(), map_error(): how far one map's points are from another's (median, 95 %).
- agreed(): two drives' maps merged: only the points both saw (parked cars drop out).
- trim(), write_map2d(), read_map2d(): a 2D map cut to what was seen, written and read as
  a map server's PGM and YAML (map-area and --merge write theirs this way).

Metres, radians, seconds; REP-103 and REP-105 (map: x east, y north).
"""

import math
from pathlib import Path

import numpy as np
import yaml

# WGS84, for UTM.
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996

# PGM values for a map server's trinary map (negate 0, occupied_thresh 0.65,
# free_thresh 0.25): (255 - v) / 255 is the chance a cell is occupied.
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 128

# GPS fixes good enough to place the map: a fix (status >= 0) whose horizontal standard
# deviation is at most this (m), from its covariance.
GOOD_FIX_SD = 0.5
MIN_FIXES = 20          # fewer good fixes: no GPS fit
MIN_SPREAD = 15.0       # m: the fixes must reach this far apart, or the turn is a guess


# --- Poses --------------------------------------------------------------------------------

def flatten(matrix):
    """Return (x, y, yaw) of a 4x4 pose: where it is from above, and which way x points."""
    m = np.asarray(matrix, float)
    return float(m[0, 3]), float(m[1, 3]), math.atan2(m[1, 0], m[0, 0])


def compose(a, b):
    """Return pose a then pose b (b in a's frame); poses are (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ayaw + byaw))


def wrap(angle):
    """Return the angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_quat(yaw):
    """Return the quaternion (x, y, z, w) of a turn about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def at(stamps, poses, t):
    """
    Return the pose (x, y, yaw) at time t, on a line between the two nearest, or None.

    stamps: sorted times (s); poses: [(x, y, yaw)] at those times. None outside them.
    """
    stamps = np.asarray(stamps, float)
    if len(stamps) == 0 or t < stamps[0] or t > stamps[-1]:
        return None
    i = int(np.searchsorted(stamps, t))
    if stamps[min(i, len(stamps) - 1)] == t:
        return tuple(poses[min(i, len(stamps) - 1)])
    t0, t1 = stamps[i - 1], stamps[i]
    f = (t - t0) / (t1 - t0)
    (x0, y0, a0), (x1, y1, a1) = poses[i - 1], poses[i]
    return (x0 + f * (x1 - x0), y0 + f * (y1 - y0), wrap(a0 + f * wrap(a1 - a0)))


# --- GPS ----------------------------------------------------------------------------------

def utm(lat, lon, zone=None):
    """
    Return (easting, northing, zone) of a WGS84 latitude and longitude (degrees).

    The usual series (Snyder, "Map Projections: A Working Manual", 1987, p. 61): well under
    a millimetre near the centre of a zone. zone: force one (the datum's), else its own.
    """
    if zone is None:
        zone = int((lon + 180.0) // 6.0) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    phi, lam = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1.0 - e2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = ep2 * math.cos(phi) ** 2
    a = math.cos(phi) * (lam - lon0)
    m = WGS84_A * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * phi
                   - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * phi)
                   + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * phi)
                   - (35 * e2 ** 3 / 3072) * math.sin(6 * phi))
    easting = UTM_K0 * n * (a + (1 - t + c) * a ** 3 / 6
                            + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * a ** 5 / 120) + 500000.0
    northing = UTM_K0 * (m + n * math.tan(phi) * (
        a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
        + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * a ** 6 / 720))
    if lat < 0:
        northing += 10000000.0
    return easting, northing, zone


def convergence(lat, lon, zone=None):
    """Return how far grid north turns from true north, clockwise (rad)."""
    if zone is None:
        zone = int((lon + 180.0) // 6.0) + 1
    lon0 = (zone - 1) * 6 - 180 + 3
    return math.atan(math.tan(math.radians(lon - lon0)) * math.sin(math.radians(lat)))


def gps_to_map(lat, lon, datum):
    """
    Return (x, y) in the map frame, as navsat_transform puts a fix there.

    UTM in the datum's zone, less the datum's UTM, turned by minus the convergence at the
    datum so y points to true north (aav_hud's map-core earth.rs does the same in Rust).
    """
    e0, n0, zone = utm(datum[0], datum[1])
    e, n, _ = utm(lat, lon, zone)
    turn = -convergence(datum[0], datum[1], zone)
    c, s = math.cos(turn), math.sin(turn)
    de, dn = e - e0, n - n0
    return c * de - s * dn, s * de + c * dn


def good_fix(status, covariance, max_sd=GOOD_FIX_SD):
    """Return True for a fix with a position (status >= 0) and a small enough spread."""
    if status < 0 or covariance is None or len(covariance) < 5:
        return False
    sd = math.sqrt(max(covariance[0], covariance[4]))
    return math.isfinite(sd) and 0.0 < sd <= max_sd


# --- Fits ---------------------------------------------------------------------------------

def fit_2d(source, target):
    """
    Return (x, y, yaw): the turn and move that best put source's points on target's.

    Least squares (Kabsch/Umeyama in 2D, no scale): target ~ R(yaw) source + (x, y).
    """
    p = np.asarray(source, float).reshape(-1, 2)
    q = np.asarray(target, float).reshape(-1, 2)
    if len(p) < 2:
        raise ValueError('a fit needs two points at least')
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    a, b = p - pc, q - qc
    yaw = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])),
                     float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])))
    c, s = math.cos(yaw), math.sin(yaw)
    x = qc[0] - (c * pc[0] - s * pc[1])
    y = qc[1] - (s * pc[0] + c * pc[1])
    return float(x), float(y), yaw


def apply_2d(transform, points):
    """Return points (N x 2) turned and moved by transform (x, y, yaw)."""
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, float).reshape(-1, 2)
    return np.stack([c * p[:, 0] - s * p[:, 1] + x, s * p[:, 0] + c * p[:, 1] + y], axis=1)


def spread(points):
    """Return the largest distance between any two points' bounding-box corners (m)."""
    p = np.asarray(points, float).reshape(-1, 2)
    return float(np.hypot(*(p.max(axis=0) - p.min(axis=0)))) if len(p) else 0.0


def georeference(stamps, poses, fixes=(), start=None, antenna=(0.0, 0.0),
                 min_fixes=MIN_FIXES, min_spread=MIN_SPREAD):
    """
    Return (transform (x, y, yaw), how): the move from SLAM's frame to the map frame.

    stamps, poses: SLAM's path, [(x, y, yaw)] at sorted times. fixes: good GPS fixes as
    [(t, x, y)] already in the map frame. start: the map-frame pose (x, y, yaw) of the
    car at the first stamp (the EKF's), or None. antenna: the GPS antenna in base_link.

    Least squares on the fixes when there are min_fixes of them, min_spread apart; else
    the start pose; else none (the map stays in SLAM's own frame, its origin where the
    drive began). how: a dict for the map's YAML (method, fixes used, residuals).
    """
    pairs = []
    for t, gx, gy in fixes:
        pose = at(stamps, poses, t)
        if pose is not None:
            ax, ay, _ = compose(pose, (antenna[0], antenna[1], 0.0))
            pairs.append(((ax, ay), (gx, gy)))
    if len(pairs) >= min_fixes and spread([g for _, g in pairs]) >= min_spread:
        source = np.array([s for s, _ in pairs])
        target = np.array([g for _, g in pairs])
        transform = fit_2d(source, target)
        residual = np.linalg.norm(apply_2d(transform, source) - target, axis=1)
        return transform, {
            'method': 'gps', 'fixes_used': len(pairs),
            'residual_median_m': round(float(np.median(residual)), 3),
            'residual_95_m': round(float(np.percentile(residual, 95)), 3)}
    why = (f'{len(pairs)} good GPS fixes, {spread([g for _, g in pairs]):.0f} m apart '
           f'(needs {min_fixes}, {min_spread:.0f} m)')
    if start is not None:
        first = poses[0]
        # start = transform ∘ first, so transform = start ∘ first⁻¹.
        c, s = math.cos(first[2]), math.sin(first[2])
        inverse = (-(c * first[0] + s * first[1]), s * first[0] - c * first[1], -first[2])
        return compose(start, inverse), {'method': 'start pose', 'why': why}
    return (0.0, 0.0, 0.0), {'method': 'none: local', 'why': why + '; no start pose'}


# --- Files --------------------------------------------------------------------------------

def write_tum(path, stamps, poses):
    """Write poses (x, y, yaw) at stamps in the TUM format: t x y z qx qy qz qw."""
    with open(path, 'w') as f:
        for t, (x, y, yaw) in zip(stamps, poses):
            qx, qy, qz, qw = yaw_quat(yaw)
            f.write(f'{t:.6f} {x:.4f} {y:.4f} 0.0000 {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')


# --- The 2D map ---------------------------------------------------------------------------

class Occupancy:
    """
    The 2D map, cloud by cloud: hits (points in the height band) and free (rays) per cell.

    bounds: ((x0, y0), (x1, y1)) in the map frame; cell: m. A cell is occupied when at
    least min_hits clouds hit it and hits are at least occupied_share of what saw it;
    free when a ray crossed it and it isn't occupied; else unknown.
    """

    def __init__(self, bounds, cell=0.25, band=(0.3, 2.0), max_range=45.0, bins=720):
        """Make an empty map covering bounds."""
        (x0, y0), (x1, y1) = bounds
        self.cell = float(cell)
        # Whole metres: the 64 m chunks then fall on NDT's 1 m cubes (ndt-chunks).
        self.origin = (float(math.floor(x0)), float(math.floor(y0)))
        self.width = int(math.ceil((x1 - self.origin[0]) / cell)) + 1
        self.height = int(math.ceil((y1 - self.origin[1]) / cell)) + 1
        self.hits = np.zeros((self.height, self.width), np.uint32)
        self.free = np.zeros((self.height, self.width), np.uint32)
        self.band = band
        self.max_range = max_range
        self.bins = bins

    def _cells(self, xy):
        """Return the unique (row, col) indices of map points inside the grid."""
        col = np.floor((xy[:, 0] - self.origin[0]) / self.cell).astype(np.int64)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.cell).astype(np.int64)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        flat = np.unique(row[inside] * self.width + col[inside])
        return flat // self.width, flat % self.width

    def add(self, base_points, pose, sensor=(0.0, 0.0)):
        """
        Add one cloud.

        base_points: N x 3 in base_link (the car's body already cut out); pose: (x, y,
        yaw) of base_link in the map; sensor: the lidar's (x, y) in base_link.
        """
        p = np.asarray(base_points, float).reshape(-1, 3)
        dx, dy = p[:, 0] - sensor[0], p[:, 1] - sensor[1]
        flat = np.hypot(dx, dy)
        keep = flat <= self.max_range
        p, dx, dy, flat = p[keep], dx[keep], dy[keep], flat[keep]
        bins = ((np.arctan2(dy, dx) + math.pi) / (2 * math.pi) * self.bins).astype(np.int64)
        bins = np.clip(bins, 0, self.bins - 1)
        in_band = (p[:, 2] >= self.band[0]) & (p[:, 2] <= self.band[1])
        # Each direction: free to the nearest thing in the band, else to the furthest
        # point seen (the ground the ray hit).
        seen = np.zeros(self.bins)
        np.maximum.at(seen, bins, flat)
        nearest = np.full(self.bins, np.inf)
        np.minimum.at(nearest, bins[in_band], flat[in_band])
        reach = np.where(np.isfinite(nearest), nearest - self.cell, seen)
        steps = np.arange(0.0, self.max_range, self.cell / 2.0)
        angle = (np.arange(self.bins) + 0.5) / self.bins * 2 * math.pi - math.pi
        r = steps[None, :] * np.ones((self.bins, 1))
        mask = r < reach[:, None]
        fx = sensor[0] + (r * np.cos(angle)[:, None])[mask]
        fy = sensor[1] + (r * np.sin(angle)[:, None])[mask]
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)

        def to_map(bx, by):
            return np.stack([x + c * bx - s * by, y + s * bx + c * by], axis=1)

        rows, cols = self._cells(to_map(fx, fy))
        self.free[rows, cols] += 1
        rows, cols = self._cells(to_map(p[in_band, 0], p[in_band, 1]))
        self.hits[rows, cols] += 1

    def grid(self, min_hits=2, occupied_share=0.25):
        """Return the map as PGM values, row 0 the south edge (flip to write)."""
        seen = self.hits + self.free
        occupied = (self.hits >= min_hits) & (self.hits >= occupied_share * seen)
        out = np.full(self.hits.shape, PGM_UNKNOWN, np.uint8)
        out[(self.free > 0) & ~occupied] = PGM_FREE
        out[occupied] = PGM_OCCUPIED
        return out

    def trimmed(self, min_hits=2, margin=1.0):
        """Return (grid, origin): the map cut down to what was seen (trim())."""
        return trim(self.grid(min_hits), self.origin, self.cell, margin)

    def write(self, stem, note='', min_hits=2):
        """Write <stem>.pgm and <stem>.yaml (a map server's trinary map); return the YAML."""
        grid, origin = self.trimmed(min_hits)
        return write_map2d(stem, grid, origin, self.cell, note)


def trim(grid, origin, cell, margin=1.0):
    """
    Return (grid, origin): a 2D map (PGM values) cut down to what was seen, margin m round.

    The cut is in whole metres, so the origin stays on a metre (chunks, NDT's cubes).
    """
    rows, cols = np.nonzero(grid != PGM_UNKNOWN)
    if len(rows) == 0:
        return grid, tuple(origin)
    height, width = grid.shape
    step = max(1, int(round(1.0 / cell)))
    pad = int(math.ceil(margin / cell))
    r0 = max(0, (rows.min() - pad) // step * step)
    c0 = max(0, (cols.min() - pad) // step * step)
    r1, c1 = min(height, rows.max() + pad + 1), min(width, cols.max() + pad + 1)
    return grid[r0:r1, c0:c1], (origin[0] + c0 * cell, origin[1] + r0 * cell)


def write_map2d(stem, grid, origin, cell, note=''):
    """Write <stem>.pgm and <stem>.yaml from PGM values (row 0 south); return the YAML."""
    height, width = grid.shape
    pgm = stem.with_suffix('.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(np.ascontiguousarray(grid[::-1]).tobytes())   # PGM rows run north to south
    text = (f'# {note}\n' if note else '') + (
        f'image: {pgm.name}\nmode: trinary\nresolution: {cell}\n'
        f'origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    stem.with_suffix('.yaml').write_text(text)
    return text


def read_map2d(yaml_path):
    """Return (grid PGM values row 0 south, origin (x, y), cell) of a map server's map."""
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    data = (yaml_path.parent / meta['image']).read_bytes()
    # P5, then width height, then maxval, each after whitespace (no comments: ours write none).
    parts = data.split(maxsplit=4)
    width, height = int(parts[1]), int(parts[2])
    pixels = np.frombuffer(parts[4][:width * height], np.uint8).reshape(height, width)
    return pixels[::-1].copy(), (float(meta['origin'][0]), float(meta['origin'][1])), \
        float(meta['resolution'])


# --- Comparing and merging maps -----------------------------------------------------------

def distances(points, reference):
    """Return each point's distance to the nearest point of reference (scipy's k-d tree)."""
    from scipy.spatial import cKDTree
    found, _ = cKDTree(np.asarray(reference, float)).query(np.asarray(points, float))
    return found


def map_error(points, reference):
    """Return {'median_m', 'p95_m', 'points'}: points' distances to reference's."""
    d = distances(points, reference)
    return {'median_m': round(float(np.median(d)), 3),
            'p95_m': round(float(np.percentile(d, 95)), 3), 'points': int(len(d))}


def agreed(a, b, radius=0.25):
    """
    Return the points of a and of b that the other map has within radius.

    That is what both drives saw: a car parked in one drive only has nothing near it in
    the other.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    keep_a = distances(a, b) <= radius
    keep_b = distances(b, a) <= radius
    return np.concatenate([a[keep_a], b[keep_b]])


def grid_agreement(grid, reference, reach=1):
    """
    Return {'found', 'right'}: how two 2D maps of the same cells agree on what is occupied.

    found: the share of reference's occupied cells with one of grid's within reach cells;
    right: the share of grid's occupied cells with one of reference's within reach.
    """
    def near(occupied):
        out = occupied.copy()
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                out |= np.roll(np.roll(occupied, dr, axis=0), dc, axis=1)
        return out

    mine, theirs = grid == PGM_OCCUPIED, reference == PGM_OCCUPIED
    found = float((theirs & near(mine)).sum() / max(1, theirs.sum()))
    right = float((mine & near(theirs)).sum() / max(1, mine.sum()))
    return {'found': round(found, 3), 'right': round(right, 3)}
