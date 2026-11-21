#!/usr/bin/env python3
"""
The maths behind label-drive (TODO 252, phase F): camera boxes onto lidar points, into labels.

No ROS, no PyTorch: tools/label_drive.py runs the detector and reads the drive; everything
it decides is here, and tested in tools/tests/test_label_core.py.

- WORDS: what the open-vocabulary detector is asked to find, and the label class each word
  becomes (the map editor's five: tree, building, parked_car, pole, other). A person is
  looked for but never mapped: people move.
- Camera: a pinhole camera on the car (vehicle.yaml's mount, camera_info's lens); project()
  puts base_link points into its picture (REP-103 axes: x ahead in the camera's link frame,
  the optical frame x right, y down).
- in_box(): the points inside a detection's box, and of those only the nearest group by
  depth (a tree's box also holds the building behind it).
- Fusion: each frame's labelled points, in the map frame, voted into 0.5 m cells per class;
  labels(): the cells grown into connected clusters, each one label with a rotated box, a
  height band, a confidence and the drive that saw it.
- merge(): this drive's labels into a map's labels.yaml: a person's labels stay as they are;
  the same thing seen again gains the drive in `seen_in` (occupancy across passes, TODO 259,
  reads that list).
- Truth and scores: Ravensim's own objects (the campus's trees and buildings, a world's
  boxes), and each label's precision and each object's recall.
- point_features(), point_classes(): our small point segmenter's inputs and answers
  (tools/label_train.py).

Metres, radians; REP-103 and REP-105 (map: x east, y north; base_link on the ground under the
rear axle).
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

# The map editor's classes (map-edit labels.rs LabelClass), in its order.
CLASSES = ('tree', 'building', 'parked_car', 'pole', 'other')

# What the detector is asked for, and what each becomes on the map. None: looked for, so it
# isn't mistaken for something else, but never mapped (it moves).
WORDS: Dict[str, Optional[str]] = {
    'tree': 'tree',
    'building': 'building',
    'car': 'parked_car',
    'pole': 'pole',
    'street light': 'pole',
    'traffic sign': 'pole',
    'stop sign': 'pole',
    'traffic light': 'pole',
    'fence': 'other',
    'bench': 'other',
    'person': None,
}

GROUND = 0.3          # m above base_link: lower points are the road (ndt_map's MIN_HEIGHT)
FLOOR = 0.5           # m: labels only take points above this (kerbs and raised grass, 0.3 m)
BUILDING_TOP = 2.5    # m: a "building" lower than this is a fence or a hedge: other
TREE_TOP = 2.0        # m: a "tree" lower than this is a bush: dropped
MAX_RANGE = 45.0      # m: farther points are too sparse to label
DEPTH_GAP = 1.5       # m: a break this deep (or 10 % of the depth) ends the nearest group
MIN_POINTS = 4        # points in a box for it to count
CELL = 0.5            # m: the fusion's cells
MIN_FRAMES = 2        # pictures that must agree on a cell's class
MIN_SHARE = 0.5       # of a cell's votes its class must have
MIN_CELLS = {'tree': 2, 'building': 4, 'parked_car': 3, 'pole': 1, 'other': 3}


# --- Poses ------------------------------------------------------------------------------

def rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3 x 3 rotation of ROS's rpy (fixed axes x, y, z: R = Rz Ry Rx)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def to_map(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """Return base_link points (N x 3) in the map, the car at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, float).reshape(-1, 3)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1], p[:, 2]], 1)


def to_base(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """Return map points (N x 3) in base_link, the car at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, float).reshape(-1, 3)
    dx, dy = p[:, 0] - x, p[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy, p[:, 2]], 1)


def mounted(points: np.ndarray, xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """Return a sensor's points (N x 3, its frame) in base_link, the sensor at xyz, rpy."""
    p = np.asarray(points, float).reshape(-1, 3)
    return p @ rotation(*rpy).T + np.asarray(xyz, float)


# --- The camera ---------------------------------------------------------------------------

@dataclass
class Camera:
    """A pinhole camera on the car: its picture, lens (px) and mount in base_link."""

    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def pinhole(cls, name, width, height, focal, xyz, rpy=(0.0, 0.0, 0.0)):
        """Return a camera with the centre in the middle, as Ravensim's cameras are."""
        return cls(name, width, height, focal, focal, width / 2.0, height / 2.0,
                   tuple(xyz), tuple(rpy))

    def project(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return (u, v, depth) of base_link points in the picture, and NaN u, v behind it.

        depth: m along the camera's view (its link frame's x).
        """
        p = np.asarray(points, float).reshape(-1, 3) - np.asarray(self.xyz, float)
        c = p @ rotation(*self.rpy)          # base_link -> the camera's link frame
        depth = c[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = self.cx + self.fx * (-c[:, 1]) / depth
            v = self.cy + self.fy * (-c[:, 2]) / depth
        behind = depth <= 0.1
        u[behind] = np.nan
        v[behind] = np.nan
        return u, v, depth

    def sees(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Return which projected points land in the picture."""
        with np.errstate(invalid='ignore'):
            return (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)


def in_box(u, v, depth, box, gap=DEPTH_GAP, min_points=MIN_POINTS) -> np.ndarray:
    """
    Return the indices of the points in a box (u0, v0, u1, v1, px) that belong to it.

    Of the points inside, only the nearest group by depth: sorted by depth, a break deeper
    than `gap` (or 10 % of the depth there) ends it. A group under min_points is skipped
    for the next (a twig, a stray ray). Empty if no group is big enough.
    """
    u0, v0, u1, v1 = box
    with np.errstate(invalid='ignore'):
        inside = np.flatnonzero((u >= u0) & (u <= u1) & (v >= v0) & (v <= v1))
    if len(inside) < min_points:
        return np.zeros(0, int)
    order = inside[np.argsort(depth[inside])]
    d = depth[order]
    breaks = np.flatnonzero(np.diff(d) > np.maximum(gap, 0.1 * d[:-1])) + 1
    for group in np.split(order, breaks):
        if len(group) >= min_points:
            return group
    return np.zeros(0, int)


# --- Fusion -------------------------------------------------------------------------------

@dataclass
class Hit:
    """One cell's vote from one picture: the class, the detector's score, the heights."""

    cls: str
    score: float
    frame: int
    z_lo: float
    z_hi: float


class Fusion:
    """Each picture's labelled points voted into map cells; labels() makes the clusters."""

    def __init__(self, cell: float = CELL):
        """Start empty: cells of `cell` metres."""
        self.cell = cell
        self.hits: Dict[Tuple[int, int], Dict[Tuple[str, int], Hit]] = defaultdict(dict)

    def add(self, points_map: np.ndarray, cls: str, score: float, frame: int):
        """Vote: these map points (N x 3) are a `cls`, the detector `score` sure, in `frame`."""
        p = np.asarray(points_map, float).reshape(-1, 3)
        keys = np.floor(p[:, :2] / self.cell).astype(np.int64)
        for key in {tuple(k) for k in keys}:
            mine = p[(keys[:, 0] == key[0]) & (keys[:, 1] == key[1]), 2]
            old = self.hits[key].get((cls, frame))
            if old is None or score > old.score:
                lo, hi = float(mine.min()), float(mine.max())
                if old is not None:
                    lo, hi = min(lo, old.z_lo), max(hi, old.z_hi)
                self.hits[key][(cls, frame)] = Hit(cls, score, frame, lo, hi)

    def cells(self, min_frames=MIN_FRAMES, min_share=MIN_SHARE):
        """Return {cell: (class, confidence, z_lo, z_hi)}: the cells whose votes agree."""
        out = {}
        for key, hits in self.hits.items():
            by_class = defaultdict(list)
            for hit in hits.values():
                by_class[hit.cls].append(hit)
            total = sum(h.score for h in hits.values())
            cls, mine = max(by_class.items(), key=lambda kv: sum(h.score for h in kv[1]))
            share = sum(h.score for h in mine) / total
            if len({h.frame for h in mine}) < min_frames or share < min_share:
                continue
            confidence = float(np.mean([h.score for h in mine])) * share
            out[key] = (cls, confidence, min(h.z_lo for h in mine), max(h.z_hi for h in mine))
        return out

    def labels(self, drive: str, min_frames=MIN_FRAMES, min_share=MIN_SHARE,
               min_cells=None) -> List[dict]:
        """
        Return the clusters as labels, in labels.yaml's shape, ids from 1, largest first.

        A cluster: cells of one class touching (8 ways). Its box: the smallest rotated
        rectangle round its cells; its band: its points' heights; its confidence: the cells'
        mean. seen_in: [drive]. A building under BUILDING_TOP becomes other (a fence); a
        tree under TREE_TOP is dropped (a bush).
        """
        min_cells = MIN_CELLS if min_cells is None else min_cells
        cells = self.cells(min_frames, min_share)
        seen, found = set(), []
        for start, (cls, _, _, _) in cells.items():
            if start in seen:
                continue
            group, todo = [], [start]
            seen.add(start)
            while todo:
                key = todo.pop()
                group.append(key)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (key[0] + dx, key[1] + dy)
                        if n not in seen and n in cells and cells[n][0] == cls:
                            seen.add(n)
                            todo.append(n)
            if len(group) < min_cells.get(cls, 1):
                continue
            corners = np.array([((k[0] + a) * self.cell, (k[1] + b) * self.cell)
                                for k in group for a in (0, 1) for b in (0, 1)])
            centre, size, yaw = min_rectangle(corners)
            top = max(cells[k][3] for k in group)
            if cls == 'building' and top < BUILDING_TOP:
                cls = 'other'
            if cls == 'tree' and top < TREE_TOP:
                continue
            found.append({
                'class': cls,
                'region': {'centre_m': [round(float(centre[0]), 2), round(float(centre[1]), 2)],
                           'size_m': [round(float(size[0]), 2), round(float(size[1]), 2)],
                           'yaw_rad': round(float(yaw), 3)},
                'band': {'min_m': round(float(max(0.0, min(cells[k][2] for k in group))), 2),
                         'max_m': round(float(max(cells[k][3] for k in group)), 2)},
                'source': 'camera',
                'confidence': round(float(np.mean([cells[k][1] for k in group])), 3),
                'seen_in': [drive],
            })
        found.sort(key=lambda lab: -lab['region']['size_m'][0] * lab['region']['size_m'][1])
        for i, lab in enumerate(found, 1):
            lab['id'] = i
        return [{'id': lab.pop('id'), **lab} for lab in found]


def min_rectangle(points: np.ndarray, steps: int = 90):
    """
    Return the smallest rectangle round 2D points: centre (x, y), size (along, across), yaw.

    Tries every whole degree from 0 to 89: near enough for a box a person fixes by eye. The
    size's first side lies along yaw.
    """
    p = np.asarray(points, float).reshape(-1, 2)
    best = None
    for k in range(steps):
        a = math.radians(k * 90.0 / steps)
        c, s = math.cos(a), math.sin(a)
        along, across = p @ np.array([c, s]), p @ np.array([-s, c])
        area = np.ptp(along) * np.ptp(across)
        if best is None or area < best[0] - 1e-9:
            best = (area, a, along, across)
    _, a, along, across = best
    mid_along, mid_across = (along.max() + along.min()) / 2, (across.max() + across.min()) / 2
    c, s = math.cos(a), math.sin(a)
    centre = (c * mid_along - s * mid_across, s * mid_along + c * mid_across)
    return centre, (float(np.ptp(along)), float(np.ptp(across))), a


def corners(region: dict) -> np.ndarray:
    """Return the four corners of a label's region: 4 x 2, map frame, anticlockwise."""
    (x, y), (w, h) = region['centre_m'], region['size_m']
    yaw = region.get('yaw_rad', 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[-w, -h], [w, -h], [w, h], [-w, h]]) / 2.0
    return np.stack([x + c * local[:, 0] - s * local[:, 1],
                     y + s * local[:, 0] + c * local[:, 1]], 1)


def inside_polygon(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Return which 2D points are inside a polygon (any shape: the even-odd rule)."""
    p = np.asarray(points, float).reshape(-1, 2)
    poly = np.asarray(polygon, float).reshape(-1, 2)
    x, y = p[:, 0][:, None], p[:, 1][:, None]
    x0, y0 = poly[:, 0][None, :], poly[:, 1][None, :]
    x1, y1 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    with np.errstate(divide='ignore', invalid='ignore'):
        crosses = ((y0 > y) != (y1 > y)) & (x < (x1 - x0) * (y - y0) / (y1 - y0) + x0)
    return crosses.sum(axis=1) % 2 == 1


def overlap(a: np.ndarray, b: np.ndarray, cell: float = 0.1) -> float:
    """Return the area (m²) two polygons share, counted on a grid of `cell` metres."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    lo = np.maximum(a.min(0), b.min(0))
    hi = np.minimum(a.max(0), b.max(0))
    if (hi <= lo).any():
        return 0.0
    xs = np.arange(lo[0] + cell / 2, hi[0], cell)
    ys = np.arange(lo[1] + cell / 2, hi[1], cell)
    if len(xs) == 0 or len(ys) == 0:
        return 0.0
    grid = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    both = inside_polygon(grid, a) & inside_polygon(grid, b)
    return float(both.sum()) * cell * cell


# --- labels.yaml --------------------------------------------------------------------------

HEADER = ("# Labels on this map (the map editor's format: docs/map-editor.md). source: camera\n"
          '# came from tools/label_drive.py (TODO 252); a person keeps, fixes or deletes them\n'
          '# in pixi run map-edit. seen_in: the drives that saw each one.\n')


def read_labels(path: Path) -> List[dict]:
    """Return a labels.yaml's labels, or [] if there is no file."""
    path = Path(path)
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text()) or {}
    return list(data.get('labels') or [])


def write_labels(path: Path, labels: List[dict]):
    """Write labels.yaml (version 1, the map frame), as the editor reads it."""
    body = yaml.safe_dump({'version': 1, 'frame': 'map', 'labels': labels},
                          sort_keys=False, default_flow_style=None)
    Path(path).write_text(HEADER + body)


def same_thing(a: dict, b: dict, grow: float = 1.0) -> bool:
    """Return whether two labels of one class cover the same ground (boxes grown by `grow`)."""
    if a['class'] != b['class']:
        return False
    ra, rb = dict(a['region']), dict(b['region'])
    ra['size_m'] = [s + 2 * grow for s in ra['size_m']]
    return overlap(corners(ra), corners(rb), cell=0.25) > 0.0


def merge(old: List[dict], new: List[dict], drive: str) -> Tuple[List[dict], dict]:
    """
    Return (labels, counts): this drive's camera labels merged into a map's labels.

    - A person's labels are kept as they are, and nothing from the camera replaces them.
    - This drive's earlier camera labels are taken back first (label-drive run again).
    - A new label over an earlier camera label of the same class: one label, its drives
      joined in seen_in, its confidence the surest drive's (so labelling a drive again
      changes nothing); the box the first drive's.
    - A new label over a person's of the same class: the person's gains the drive in
      seen_in (the camera agrees), and nothing is added.
    - A new label over occupancy's (source: passes, class other: something only one drive
      saw, tools/passes.py): the camera says what it is: that label takes the camera's
      class and keeps its source and seen_in (the drives the cells were seen in); nothing
      is added.
    - Otherwise: added, with the next free id.
    """
    kept = []
    for lab in old:
        lab = dict(lab)
        if lab.get('source', 'person') == 'camera' and drive in lab.get('seen_in', []):
            lab['seen_in'] = [d for d in lab['seen_in'] if d != drive]
            if not lab['seen_in']:
                continue
        kept.append(lab)
    counts = {'added': 0, 'joined': 0, 'agreed_with_person': 0, 'named_passes': 0}
    next_id = max([lab['id'] for lab in kept], default=0) + 1
    earlier = list(kept)           # this drive's own labels never join each other
    for lab in new:
        moved = next((k for k in earlier if k.get('source') == 'passes' and k['class'] == 'other'
                      and same_thing(dict(k, **{'class': lab['class']}), lab)), None)
        if moved is not None:
            moved['class'] = lab['class']
            counts['named_passes'] += 1
            continue
        match = next((k for k in earlier if same_thing(k, lab)), None)
        if match is None:
            kept.append({**lab, 'id': next_id})
            next_id += 1
            counts['added'] += 1
            continue
        drives = list(match.get('seen_in', []))
        if match.get('source') == 'passes':
            counts['named_passes'] += 1     # named already: its seen_in is occupancy's
            continue
        if match.get('source', 'person') != 'camera':
            match['seen_in'] = drives + [drive] if drive not in drives else drives
            counts['agreed_with_person'] += 1
            continue
        match['confidence'] = max(match.get('confidence', 0.0), lab['confidence'])
        if not drives:
            match['region'], match['band'] = lab['region'], lab.get('band')
        match['seen_in'] = drives + [drive]
        counts['joined'] += 1
    return kept, counts


# --- Truth and scores ---------------------------------------------------------------------

@dataclass
class Thing:
    """One of the simulator's own objects: its class and footprint (map frame)."""

    cls: str
    polygon: np.ndarray
    height: float = 0.0
    name: str = ''


def sdf_things(text: str, names: Dict[str, str]) -> List[Thing]:
    """
    Return a world file's box models as things.

    names maps a word in the model's name to a class ('building' -> building); a model
    whose name has none of them is left out.
    """
    from xml.etree import ElementTree
    things = []
    for model in ElementTree.fromstring(text).iter('model'):
        name = model.get('name', '')
        cls = next((c for word, c in names.items() if word in name), None)
        box = model.find('.//visual/geometry/box/size')
        if cls is None or box is None:
            continue
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        sx, sy, sz = (float(v) for v in box.text.split())
        region = {'centre_m': pose[:2], 'size_m': [sx, sy], 'yaw_rad': pose[5]}
        things.append(Thing(cls, corners(region), pose[2] + sz / 2, name))
    return things


def visible(things: Sequence[Thing], points_map: np.ndarray, min_points: int = 20) -> List[bool]:
    """Return which things the drive saw: at least min_points of the pictured points on them."""
    p = np.asarray(points_map, float).reshape(-1, 3)[:, :2]
    out = []
    for t in things:
        lo, hi = t.polygon.min(0) - 0.5, t.polygon.max(0) + 0.5
        near = p[((p >= lo) & (p <= hi)).all(1)]
        grown = t.polygon + 0.4 * np.sign(t.polygon - t.polygon.mean(0))
        out.append(len(near) >= min_points and int(inside_polygon(near, grown).sum())
                   >= min_points)
    return out


def score(labels: Sequence[dict], things: Sequence[Thing]) -> Dict[str, dict]:
    """
    Return each class's precision and recall, and 'all'.

    A label is right when it overlaps a thing of its class; a thing is found when a label
    of its class overlaps it. (A row of trees in one label finds every tree in it.)
    """
    out = {}
    for cls in [c for c in CLASSES if any(t.cls == c for t in things)
                or any(lab['class'] == c for lab in labels)] + ['all']:
        mine = [lab for lab in labels if cls in ('all', lab['class'])]
        theirs = [t for t in things if cls in ('all', t.cls)]
        boxes = [corners(lab['region']) for lab in mine]
        right = sum(any(t.cls == lab['class'] and overlap(b, t.polygon, 0.2) > 0
                        for t in theirs) for lab, b in zip(mine, boxes))
        found = sum(any(t.cls == lab['class'] and overlap(b, t.polygon, 0.2) > 0
                        for lab, b in zip(mine, boxes)) for t in theirs)
        out[cls] = {'labels': len(mine), 'right': right, 'things': len(theirs), 'found': found,
                    'precision': round(right / len(mine), 3) if mine else None,
                    'recall': round(found / len(theirs), 3) if theirs else None}
    return out


def score_table(scores: Dict[str, dict]) -> str:
    """Return the scores as a Markdown table."""
    rows = ['| Class | Labels | Right | Precision | Things seen | Found | Recall |',
            '|---|---|---|---|---|---|---|']
    for cls, s in scores.items():
        def pct(v):
            return '-' if v is None else f'{100 * v:.0f} %'
        rows.append(f"| {cls} | {s['labels']} | {s['right']} | {pct(s['precision'])} | "
                    f"{s['things']} | {s['found']} | {pct(s['recall'])} |")
    return '\n'.join(rows)


# --- Our small model: a point segmenter -----------------------------------------------------

FEATURES = ('z', 'range', 'zmax_fine', 'zmin_fine', 'span_fine', 'count_fine',
            'zmax_coarse', 'span_coarse', 'count_coarse', 'above_fine')
SEG_CLASSES = ('background',) + CLASSES


def _pillars(xy: np.ndarray, z: np.ndarray, size: float):
    """Return each point's pillar's (zmax, zmin, count), pillars of `size` metres."""
    keys = np.floor(xy / size).astype(np.int64)
    _, index, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    index = index.reshape(-1)
    zmax = np.full(len(counts), -np.inf)
    zmin = np.full(len(counts), np.inf)
    np.maximum.at(zmax, index, z)
    np.minimum.at(zmin, index, z)
    return zmax[index], zmin[index], counts[index].astype(float)


def point_features(points_base: np.ndarray) -> np.ndarray:
    """
    Return each base_link point's features (N x len(FEATURES), float32).

    Its height and flat range; its 0.5 m pillar's top, bottom, height span and point
    count (log); its 2 m pillar's top, span and count; how far above its pillar's bottom
    it is. The same numbers on the car, from one scan, as in training.
    """
    p = np.asarray(points_base, float).reshape(-1, 3)
    if len(p) == 0:
        return np.zeros((0, len(FEATURES)), np.float32)
    xy, z = p[:, :2], p[:, 2]
    fmax, fmin, fcount = _pillars(xy, z, 0.5)
    cmax, cmin, ccount = _pillars(xy, z, 2.0)
    out = np.stack([z, np.hypot(xy[:, 0], xy[:, 1]), fmax, fmin, fmax - fmin,
                    np.log1p(fcount), cmax, cmax - cmin, np.log1p(ccount), z - fmin], 1)
    return out.astype(np.float32)


def point_classes(points_map: np.ndarray, labels: Sequence[dict],
                  ground: float = GROUND) -> np.ndarray:
    """
    Return each map point's class index in SEG_CLASSES.

    The label whose box (and band, if it has one) holds it; 0, background, for the ground
    and the rest. A later label wins.
    """
    p = np.asarray(points_map, float).reshape(-1, 3)
    out = np.zeros(len(p), np.int64)
    for lab in labels:
        box = corners(lab['region'])
        lo, hi = box.min(0), box.max(0)
        near = np.flatnonzero(((p[:, :2] >= lo) & (p[:, :2] <= hi)).all(1))
        if len(near) == 0:
            continue
        hit = near[inside_polygon(p[near, :2], box)]
        band = lab.get('band')
        if band:
            z = p[hit, 2]
            hit = hit[(z >= band['min_m'] - 0.2) & (z <= band['max_m'] + 0.2)]
        out[hit] = SEG_CLASSES.index(lab['class'])
    out[p[:, 2] < ground] = 0
    return out


def segment_scores(truth: np.ndarray, guess: np.ndarray) -> Dict[str, dict]:
    """Return each class's point precision, recall and IoU, and the share of points right."""
    out = {}
    for k, name in enumerate(SEG_CLASSES):
        t, g = truth == k, guess == k
        tp, fp, fn = int((t & g).sum()), int((~t & g).sum()), int((t & ~g).sum())
        if tp + fp + fn == 0:
            continue
        out[name] = {'points': int(t.sum()),
                     'precision': round(tp / (tp + fp), 3) if tp + fp else None,
                     'recall': round(tp / (tp + fn), 3) if tp + fn else None,
                     'iou': round(tp / (tp + fp + fn), 3)}
    out['all'] = {'points': int(len(truth)),
                  'right': round(float((truth == guess).mean()), 3) if len(truth) else None}
    return out


def drives_of(labels: Iterable[dict]) -> List[str]:
    """Return every drive any label was seen in, in first-seen order."""
    out: List[str] = []
    for lab in labels:
        for d in lab.get('seen_in', []):
            if d not in out:
                out.append(d)
    return out


@dataclass
class Frame:
    """One moment of a drive: the car's pose (map), its lidar points (base_link), pictures."""

    stamp: float
    pose: Tuple[float, float, float]
    points: np.ndarray
    pictures: Dict[str, object] = field(default_factory=dict)
