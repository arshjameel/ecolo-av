#!/usr/bin/env python3
"""
Occupancy across passes (TODO 259): what stays, what went, from several drives of one area.

No ROS, no SLAM library: numpy only. tools/map_area.py runs it on real drives; the tests
are tools/tests/test_passes.py.

Each pass (one drive, already aligned to the others) is ray-traced into the same 2D grid
(slam_map.Occupancy: hits 0.3-2 m above the ground, the lidar's rays free). Then, cell by
cell:

    occupied in a pass   hits >= MIN_HITS, and hits >= OCCUPIED_SHARE of what saw it
    free in a pass       rays crossed it >= MIN_FREE times, almost never hit
                         (hits <= FREE_HIT_SHARE of the rays), and nothing that pass
                         saw occupied within TOLERANCE cells (pose error at a wall's edge)

    dynamic              occupied in some passes, free in another: a parked car, a person,
                         a bin. Its points leave NDT's map; its 2D cell becomes free
    seen once            occupied in exactly one pass, never seen free: kept, flagged
                         seen_in: [that drive] for the editor and the camera's labels (252)
    static               occupied, never seen free: kept

A dynamic cell's neighbours that were seen in one pass only (the far side of the parked
car, half hidden from the other passes) go with it: growing at most GROW cells, and only
through seen-once cells, so a wall seen by every pass is never eaten.

Ideas, not code, from (docs/acknowledgements.md):
- Removert (Kim and Kim, IROS 2020; CC BY-NC-SA 4.0): a map point another scan sees
  through is gone;
- ERASOR (Lim et al., RA-L 2021; GPL-3.0): judge the column above each cell, not single
  points;
- DUFOMap (Duberg et al., RA-L 2024; BSD-3-Clause): space seen empty once is empty,
  with a margin for the sensor's and the pose's error.

Metres; the map frame of REP-105 (x east, y north).
"""

import math

import numpy as np

import slam_map as sm

MIN_HITS = 2            # clouds: a pass sees a cell occupied (slam_map.Occupancy.grid's rule)
OCCUPIED_SHARE = 0.25   # hits / (hits + free), the same
MIN_FREE = 3            # rays through a cell before a pass calls it free
FREE_HIT_SHARE = 0.05   # a free cell's hits, at most this share of its rays
TOLERANCE = 1           # cells: no free next to what that pass saw occupied
GROW = 4                # cells (1 m at 0.25 m): a dynamic cluster takes its seen-once edge
TOP = 3.0               # m: a dynamic cell's NDT points up to here go (a tree over a car stays)
MIN_LABEL_CELLS = 4     # a seen-once cluster this big or more becomes a label


def near(mask, reach):
    """Return mask grown by reach cells each way (a square), without wrapping round."""
    out = mask.copy()
    height, width = mask.shape
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if dr == 0 and dc == 0:
                continue
            src = mask[max(0, -dr):height - max(0, dr), max(0, -dc):width - max(0, dc)]
            out[max(0, dr):height - max(0, -dr), max(0, dc):width - max(0, -dc)] |= src
    return out


def occupied(hits, free, min_hits=MIN_HITS, share=OCCUPIED_SHARE):
    """Return where one pass saw something: hits and free are one pass's counts."""
    hits, free = np.asarray(hits), np.asarray(free)
    return (hits >= min_hits) & (hits >= share * (hits + free))


def seen_free(hits, free, min_free=MIN_FREE, hit_share=FREE_HIT_SHARE,
              tolerance=TOLERANCE, min_hits=MIN_HITS):
    """Return where one pass saw empty space, kept tolerance cells off what it saw occupied."""
    hits, free = np.asarray(hits), np.asarray(free)
    clear = (free >= min_free) & (hits <= hit_share * free)
    return clear & ~near(occupied(hits, free, min_hits), tolerance)


def classify(occupied_by, free_by, grow=GROW):
    """
    Return a dict of grids from each pass's occupied and free cells (P x H x W booleans).

    occupied_in, free_in: how many passes saw each cell occupied, free. dynamic: occupied
    in some, free in another. once: occupied in one pass only, never free. removed:
    dynamic, and the seen-once cells it reaches in grow steps. kept: occupied, not
    removed. free: free in some pass and not kept, or removed.
    """
    occ = np.asarray(occupied_by, bool)
    fre = np.asarray(free_by, bool)
    occupied_in = occ.sum(axis=0)
    free_in = fre.sum(axis=0)
    dynamic = (occupied_in > 0) & (free_in > 0)
    once = (occupied_in == 1) & (free_in == 0)
    removed = dynamic.copy()
    for _ in range(grow):
        more = near(removed, 1) & once & ~removed
        if not more.any():
            break
        removed |= more
    kept = (occupied_in > 0) & ~removed
    return {'occupied_in': occupied_in, 'free_in': free_in, 'dynamic': dynamic,
            'once': once & ~removed, 'removed': removed, 'kept': kept,
            'free': ((free_in > 0) | removed) & ~kept}


def to_pgm(classes):
    """Return the merged 2D map as PGM values (row 0 the south edge), as slam_map writes."""
    out = np.full(classes['kept'].shape, sm.PGM_UNKNOWN, np.uint8)
    out[classes['free']] = sm.PGM_FREE
    out[classes['kept']] = sm.PGM_OCCUPIED
    return out


def cells_of(points, origin, cell, shape):
    """Return (row, col, inside) of each point's cell: points N x 2 or more, map frame."""
    p = np.asarray(points, float)
    col = np.floor((p[:, 0] - origin[0]) / cell).astype(np.int64)
    row = np.floor((p[:, 1] - origin[1]) / cell).astype(np.int64)
    inside = (row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1])
    return np.clip(row, 0, shape[0] - 1), np.clip(col, 0, shape[1] - 1), inside


def drop(points, classes, origin, cell, top=TOP, margin=1):
    """
    Return (kept points, how many went): NDT's points over a removed cell, up to top m.

    points: N x 3, map frame; classes: classify()'s grids on origin and cell. A point in a
    cell next to a removed one (margin cells) goes too unless its cell is kept: the
    stragglers at a parked car's corners, hit too seldom to be occupied in any pass.
    """
    p = np.asarray(points, float).reshape(-1, 3)
    out = near(classes['removed'], margin) & ~classes['kept'] if margin else classes['removed']
    row, col, inside = cells_of(p, origin, cell, out.shape)
    gone = inside & out[row, col] & (p[:, 2] <= top)
    return p[~gone], int(gone.sum())


def clusters(mask, min_cells=1):
    """Return each 8-connected group of True cells as an array of (row, col), big ones only."""
    left = mask.copy()
    found = []
    height, width = mask.shape
    for start in zip(*np.nonzero(mask)):
        if not left[start]:
            continue
        left[start] = False
        group, todo = [], [start]
        while todo:
            r, c = todo.pop()
            group.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < height and 0 <= cc < width and left[rr, cc]:
                        left[rr, cc] = False
                        todo.append((rr, cc))
        if len(group) >= min_cells:
            found.append(np.array(group))
    return found


def box_of(group, origin, cell):
    """Return (centre x, y, size x, y) in metres of a cluster's cells (an upright box)."""
    r0, c0 = group.min(axis=0)
    r1, c1 = group.max(axis=0) + 1
    x0, y0 = origin[0] + c0 * cell, origin[1] + r0 * cell
    x1, y1 = origin[0] + c1 * cell, origin[1] + r1 * cell
    return float((x0 + x1) / 2.0), float((y0 + y1) / 2.0), float(x1 - x0), float(y1 - y0)


def seen_once_labels(classes, occupied_by, names, origin, cell, min_cells=MIN_LABEL_CELLS,
                     first_id=1):
    """
    Return labels.yaml's labels: one box per seen-once cluster, seen_in its drive.

    The map editor reads them (map-edit's labels.rs): class other, source passes, a
    height band of the 2D map's (0.3-2 m). A person relabels or deletes them there.
    """
    occ = np.asarray(occupied_by, bool)
    labels = []
    for group in clusters(classes['once'], min_cells):
        which = sorted({names[p] for r, c in group for p in np.nonzero(occ[:, r, c])[0]})
        x, y, sx, sy = box_of(group, origin, cell)
        labels.append({'id': first_id + len(labels), 'class': 'other',
                       'region': {'centre_m': [round(x, 3), round(y, 3)],
                                  'size_m': [round(sx, 3), round(sy, 3)], 'yaw_rad': 0.0},
                       'band': {'min_m': 0.3, 'max_m': 2.0}, 'source': 'passes',
                       'seen_in': which})
    return labels


def summary(classes, cell):
    """Return the cells and square metres of each class, for the report."""
    area = cell * cell
    out = {}
    for name in ('kept', 'dynamic', 'removed', 'once', 'free'):
        n = int(classes[name].sum())
        out[name] = {'cells': n, 'm2': round(n * area, 2)}
    return out


def resample(grid, origin, cell, transform, target_origin, target_shape, target_cell):
    """
    Return a 2D map (PGM values) moved by transform (x, y, yaw) onto another grid's cells.

    Each target cell takes the source cell under its centre (nearest); outside: unknown.
    For merging maps already made (map-drive --merge) without their drives.
    """
    rows, cols = np.indices(target_shape)
    cx = target_origin[0] + (cols + 0.5) * target_cell
    cy = target_origin[1] + (rows + 0.5) * target_cell
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    # The inverse move: target point into the source map's frame.
    dx, dy = cx - x, cy - y
    sx, sy = c * dx + s * dy, -s * dx + c * dy
    col = np.floor((sx - origin[0]) / cell).astype(np.int64)
    row = np.floor((sy - origin[1]) / cell).astype(np.int64)
    inside = (row >= 0) & (row < grid.shape[0]) & (col >= 0) & (col < grid.shape[1])
    out = np.full(target_shape, sm.PGM_UNKNOWN, np.uint8)
    out[inside] = grid[row[inside], col[inside]]
    return out


def agreed_by(maps, radius=0.25, min_maps=2):
    """
    Return the points of every map that at least min_maps maps (its own too) have nearby.

    maps: [N x 3] in one frame. slam_map.agreed for two maps; the same rule for more.
    """
    maps = [np.asarray(m, float) for m in maps]
    kept = []
    for i, points in enumerate(maps):
        votes = np.ones(len(points), int)
        for j, other in enumerate(maps):
            if i != j and len(other) and len(points):
                votes += sm.distances(points, other) <= radius
        kept.append(points[votes >= min_maps])
    return np.concatenate(kept) if kept else np.zeros((0, 3))
