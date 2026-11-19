#!/usr/bin/env python3
"""
The learned lane finder's pictures and labels, from Ravensim and the lane maps (TODO 262b).

    pixi run -e ml python3 tools/lane_data.py --out models/lane-data        (about 2 minutes)
        [--road 1500] [--campus 6000] [--seed 1] [--sim-cameras BIN]

For each world (the road world, and the campus if `maps/campus/` is made: `pixi run
campus`) it picks car poses on the painted roads (anywhere across the tarmac, turned up to
0.25 rad from the road, both ways), draws the front left camera at each one with
`sim_cameras --poses` (no ROS: Bevy, offscreen), and labels each picture from the lane
map, with no hand work. A picture with grass, not tarmac, just ahead is left out (a road
in the lane map the world draws no tarmac for).

  rows      ROWS: the picture rows the model reads (row anchors, UFLD v2's idea)
  cols      for each of the 4 lines (left of the car's lane's left, its left, its right,
            right of its right) and each row: where the painted line crosses it (px,
            NaN where it doesn't)
  colour    each line's paint: 0 white, 1 yellow (the campus's dashed centre line)
  gt_y      each line's y in base_link (m) at GT_X metres ahead: for the scores

The paint comes from the same rules the worlds were made by: the road world's two white
lines (road.lanes.yaml), and the campus's white edge lines EDGE_INSET in from the kerb and
its yellow dashed centre (tools/campus_world.py). A pose is left out when a road crossing
the car's way is in view (its paint would be in the picture, unlabelled).

What it writes in --out (outside git, as models/ is):

  images.npy    uint8, N x INPUT_H x INPUT_W x 3: the picture's rows CROP_TOP..360,
                scaled to INPUT_W x INPUT_H (what the model takes)
  labels.npz    cols, colour, gt_y, held (1: held out for the scores), world, poses
  meta.json     the picture's and the model's sizes, ROWS, CELLS, the lens and mount

The maths (poses, projection, labels, decoding, the ground, the fit) has no ROS and no
PyTorch: tools/tests/test_lane_data.py. The Rust node (aav_lane_finder_learned) decodes
the model's answers the same way.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
VEHICLE = ROOT / 'ros' / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml'
CAMERAS = ROOT / 'ros' / 'sim' / 'aav_sim_cameras' / 'config' / 'cameras.yaml'
ROAD_WORLD = ROOT / 'ros' / 'sim' / 'aav_sim_models' / 'worlds' / 'road.sdf'
ROAD_LANES = ROOT / 'ros' / 'sim' / 'aav_sim_models' / 'worlds' / 'road.lanes.yaml'
CAMPUS = ROOT / 'maps' / 'campus'
SIM_CAMERAS = ROOT / 'ros' / 'sim' / 'aav_sim_cameras' / 'target' / 'release' / 'sim_cameras'

# The picture and what the model sees of it.
WIDTH, HEIGHT = 640, 360          # px: the front cameras' pictures
CROP_TOP = 168                    # px: rows above this are sky (the horizon is at 180)
INPUT_W, INPUT_H = 480, 144       # px: rows CROP_TOP..HEIGHT, scaled by 0.75
ROWS = tuple(range(192, 357, 4))  # px: the 42 row anchors, about 4 m to 50 m ahead
CELLS = 100                       # the columns each row's line is placed in (6.4 px each)
LANES = ('left2', 'left', 'right', 'right2')
GT_X = (5.0, 10.0, 15.0)          # m ahead of base_link: where the scores measure
SLOT_X = 8.0                      # m ahead: a line must start before it and reach past it

# The campus's paint (tools/campus_world.py: EDGE_INSET, LINE_WIDTH).
EDGE_INSET = 0.5
WHITE, YELLOW = 0, 1

# Poses.
MAX_TURN = 0.25                   # rad: the car turned from the road, at most
CROSSING = math.radians(25.0)     # a line more turned than this from the car's way...
CROSS_VIEW = (2.0, 45.0, 15.0)    # ...in x from, x to, |y| (m) leaves the pose out
END_MARGIN = 6.0                  # m: no pose this near a road's end
VIEW_X, VIEW_Y = 150.0, 40.0      # m: the paint looked at, ahead and either side
JOIN_GAP = 1.0                    # m: one line's end this near another's start carries it on


@dataclass(frozen=True)
class Camera:
    """A pinhole lens on the car: base_link to camera, as aav_lane_finder's Mount."""

    fx: float
    fy: float
    cx: float
    cy: float
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]


@dataclass(frozen=True)
class Paint:
    """One painted line: its centre line (map frame, m) and colour."""

    points: np.ndarray            # (n, 2)
    colour: int


def load_camera(name: str = 'front_left', cameras: Path = CAMERAS,
                vehicle: Path = VEHICLE) -> Camera:
    """Return the camera's lens (cameras.yaml) and mount (vehicle.yaml)."""
    spec = next(c for c in yaml.safe_load(cameras.read_text())['cameras'] if c['name'] == name)
    mount = yaml.safe_load(vehicle.read_text())['sensors'][spec['mount']]
    f = float(spec['focal_px'])
    return Camera(f, f, spec['width'] / 2.0, spec['height'] / 2.0,
                  tuple(float(v) for v in mount['xyz']), tuple(float(v) for v in mount['rpy']))


def rotation(rpy: Sequence[float]) -> np.ndarray:
    """Return the matrix from the camera's axes to base_link's (lane-core's `rotation`)."""
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def project(points: np.ndarray, cam: Camera) -> np.ndarray:
    """Project (n, 3) base_link points to (n, 2) pixels; NaN behind the lens (lane-core's)."""
    c = (points - np.asarray(cam.xyz)) @ rotation(cam.rpy)
    ox, oy, oz = -c[:, 1], -c[:, 2], c[:, 0]
    out = np.full((len(points), 2), np.nan)
    ahead = oz > 0.1
    out[ahead, 0] = cam.fx * ox[ahead] / oz[ahead] + cam.cx
    out[ahead, 1] = cam.fy * oy[ahead] / oz[ahead] + cam.cy
    return out


def ground(uv: np.ndarray, cam: Camera) -> np.ndarray:
    """Put (n, 2) pixels on the flat road, z = 0, in base_link (m); NaN above the horizon."""
    uv = np.atleast_2d(uv)
    ox = (uv[:, 0] - cam.cx) / cam.fx
    oy = (uv[:, 1] - cam.cy) / cam.fy
    ray = np.stack([np.ones_like(ox), -ox, -oy], axis=1) @ rotation(cam.rpy).T
    t = np.where(ray[:, 2] < -1e-9, -cam.xyz[2] / np.minimum(ray[:, 2], -1e-9), np.nan)
    return np.asarray(cam.xyz[:2]) + ray[:, :2] * t[:, None]


def offset_line(points, d):
    """Return a line moved d m to its left: campus_world.py's `offset_line` (mitred)."""
    pts = [p for i, p in enumerate(points) if i == 0 or math.dist(p, points[i - 1]) > 1e-6]
    if len(pts) < 2:
        return list(pts)

    def normal(a, b):
        length = math.dist(a, b)
        return (-(b[1] - a[1]) / length, (b[0] - a[0]) / length)

    normals = [normal(a, b) for a, b in zip(pts, pts[1:])]
    out = []
    for i, p in enumerate(pts):
        before, after = normals[max(i - 1, 0)], normals[min(i, len(normals) - 1)]
        mx, my = before[0] + after[0], before[1] + after[1]
        length = math.hypot(mx, my)
        if length < 1e-6:
            mx, my, length = before[0], before[1], 1.0
        mx, my = mx / length, my / length
        scale = min(1.0 / max(mx * after[0] + my * after[1], 1e-6), 3.0)
        out.append((p[0] + mx * d * scale, p[1] + my * d * scale))
    return out


def densify(points, step: float = 0.25) -> np.ndarray:
    """Return a polyline with a point every `step` m or less."""
    pts = np.asarray(points, dtype=float)
    out = [pts[:1]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
    return np.concatenate(out)


def painted(lanes_yaml: Path, centre: bool) -> Tuple[List[dict], List[Paint]]:
    """Return a lane map's roads and their paint: white edges; a yellow centre if `centre`."""
    roads = yaml.safe_load(Path(lanes_yaml).read_text())['roads']
    paint = []
    for road in roads:
        pts = [tuple(p) for p in road['points']]
        inset = float(road.get('edge_inset', EDGE_INSET))
        shift = float(road['width']) / 2.0 - inset
        for side in (1.0, -1.0):
            paint.append(Paint(densify(offset_line(pts, side * shift)), WHITE))
        if centre:
            paint.append(Paint(densify(pts), YELLOW))
    return roads, paint


def to_base(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """Map points (n, 2) into base_link for the car at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = points - np.array([x, y])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def runs(base: np.ndarray, x_min: float) -> List[np.ndarray]:
    """Return the line's pieces in view (x_min to VIEW_X, VIEW_Y each side), rising in x."""
    if len(base) < 2:
        return []
    ok = (base[:, 0] >= x_min) & (base[:, 0] < VIEW_X) & (np.abs(base[:, 1]) < VIEW_Y)
    if not ok.any():
        return []
    rising = np.r_[False, np.diff(base[:, 0]) > 0]
    goes_on = ok & np.r_[False, ok[:-1]] & rising
    piece = np.cumsum(ok & ~goes_on)
    out = []
    for k in np.unique(piece[ok]):
        pts = base[ok & (piece == k)]
        if len(pts) > 1:
            out.append(pts)
    return out


def ahead_pieces(base: np.ndarray, x_min: float) -> List[np.ndarray]:
    """Return the line's pieces ahead of x_min, rising in x, whichever way it was drawn."""
    return runs(base, x_min) + runs(base[::-1], x_min)


def y_at(piece: np.ndarray, x: float) -> float:
    """Return the piece's y at x (m), NaN outside it."""
    if x < piece[0, 0] or x > piece[-1, 0]:
        return math.nan
    return float(np.interp(x, piece[:, 0], piece[:, 1]))


def joined(piece: np.ndarray, colour: int, pieces, gap: float = JOIN_GAP) -> np.ndarray:
    """
    Return the piece with the pieces of the same paint that carry it on (another road's).

    Lane maps cut a road where its name or width changes; the paint runs on.
    """
    for _ in range(8):
        grown = False
        for other, c in pieces:
            if c != colour or other is piece:
                continue
            if np.hypot(*(other[-1] - piece[0])) < gap and other[-1, 0] <= piece[0, 0]:
                piece, grown = np.concatenate([other, piece]), True
            elif np.hypot(*(other[0] - piece[-1])) < gap and other[0, 0] >= piece[-1, 0]:
                piece, grown = np.concatenate([piece, other]), True
        if not grown:
            break
    return piece


def on_tarmac(picture: np.ndarray) -> bool:
    """Return True if the road just ahead is tarmac (grey), not grass. A crop, as `crop` gives."""
    h, w = picture.shape[:2]
    patch = picture[h - h // 6:, int(w * 0.4):int(w * 0.6)].reshape(-1, 3).astype(float).mean(0)
    return patch[1] - (patch[0] + patch[2]) / 2.0 < 8.0


def crossing(paint: Sequence[Paint], pose) -> bool:
    """Return True if a line runs across the car's way, in view."""
    x0, x1, ymax = CROSS_VIEW
    for line in paint:
        b = to_base(line.points, pose)
        d = np.diff(b, axis=0)
        mid = (b[1:] + b[:-1]) / 2.0
        near = (mid[:, 0] > x0) & (mid[:, 0] < x1) & (np.abs(mid[:, 1]) < ymax)
        if not near.any():
            continue
        angle = np.abs(np.arctan2(d[near, 1], np.abs(d[near, 0])))
        if (angle > CROSSING).any():
            return True
    return False


def label(paint: Sequence[Paint], pose, cam: Camera):
    """
    One picture's labels: (cols (4, R) px or NaN, colour (4,), gt_y (4, len(GT_X)) m or NaN).

    The car's lane is between the nearest line on each side where each line comes into
    view (at the camera's x, or where it starts if later, before SLOT_X): so a car turned
    towards a line still has that line on its own side.
    """
    near_x = cam.xyz[0] + 0.5
    pieces = [(piece, line.colour) for line in paint
              for piece in ahead_pieces(to_base(line.points, pose), near_x)]
    lines = []          # (y where it comes into view, piece, colour)
    for piece, colour in pieces:
        piece = joined(piece, colour, pieces)
        if piece[0, 0] <= SLOT_X and piece[-1, 0] > SLOT_X + 2.0 and \
                not any(np.array_equal(piece, ln[1]) for ln in lines):
            lines.append((float(piece[0, 1]), piece, colour))
    left = sorted((ln for ln in lines if ln[0] > 0), key=lambda ln: ln[0])
    right = sorted((ln for ln in lines if ln[0] <= 0), key=lambda ln: -ln[0])
    slots = [left[1] if len(left) > 1 else None, left[0] if left else None,
             right[0] if right else None, right[1] if len(right) > 1 else None]
    rows = np.asarray(ROWS, dtype=float)
    cols = np.full((len(LANES), len(ROWS)), np.nan)
    colour = np.zeros(len(LANES), dtype=np.int64)
    gt = np.full((len(LANES), len(GT_X)), np.nan)
    for k, slot in enumerate(slots):
        if slot is None:
            continue
        _, piece, colour[k] = slot
        uv = project(np.column_stack([piece, np.zeros(len(piece))]), cam)
        ok = ~np.isnan(uv[:, 1])
        uv = uv[ok]
        if len(uv) < 2:
            continue
        # Further along, higher in the picture: v falls as x rises.
        v, u = uv[::-1, 1], uv[::-1, 0]
        inside = (rows >= v[0]) & (rows <= v[-1])
        cols[k, inside] = np.interp(rows[inside], v, u)
        cols[k, (cols[k] < 0) | (cols[k] > WIDTH - 1)] = np.nan
        gt[k] = [y_at(piece, x) for x in GT_X]
    return cols, colour, gt


def sample_poses(roads: Sequence[dict], n: int, rng: random.Random, paint: Sequence[Paint],
                 tries: int = 20) -> List[Tuple[float, float, float, int]]:
    """Return n poses (x, y, yaw, road) across the painted roads' tarmac, none at a crossing."""
    lengths = [sum(math.dist(a, b) for a, b in zip(r['points'], r['points'][1:])) for r in roads]
    usable = [i for i, length in enumerate(lengths) if length > 2 * END_MARGIN + 1.0]
    weights = [lengths[i] for i in usable]
    out = []
    for _ in range(n * tries):
        if len(out) >= n:
            break
        i = rng.choices(usable, weights)[0]
        road = roads[i]
        s = rng.uniform(END_MARGIN, lengths[i] - END_MARGIN)
        for a, b in zip(road['points'], road['points'][1:]):
            seg = math.dist(a, b)
            if s <= seg:
                break
            s -= seg
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        half = float(road['width']) / 2.0 - float(road.get('edge_inset', EDGE_INSET)) - 0.6
        side = rng.uniform(-half, half)
        px = a[0] + (b[0] - a[0]) * s / seg - math.sin(heading) * side
        py = a[1] + (b[1] - a[1]) * s / seg + math.cos(heading) * side
        yaw = heading + rng.uniform(-MAX_TURN, MAX_TURN) + (math.pi if rng.random() < 0.5 else 0.0)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if not crossing(paint, (px, py, yaw)):
            out.append((px, py, yaw, i))
    return out


# --- Decoding the model's answers (the Rust node's `ufld-core` does the same) ---

def decode(loc: np.ndarray, exist: np.ndarray, threshold: float = 0.5,
           window: int = 1) -> np.ndarray:
    """
    Return each line's column (px) at each row from the model's answers; NaN for none.

    loc: (CELLS, R, 4) logits; exist: (2, R, 4) logits. The column is the softmax's mean over
    the best cell and `window` cells each side (UFLD v2's local expectation).
    """
    cells, nrows, nlanes = loc.shape
    p_exist = np.exp(exist[1]) / (np.exp(exist[0]) + np.exp(exist[1]))
    out = np.full((nlanes, nrows), np.nan)
    for k in range(nlanes):
        for r in range(nrows):
            if p_exist[r, k] <= threshold:
                continue
            logits = loc[:, r, k]
            best = int(np.argmax(logits))
            lo, hi = max(0, best - window), min(cells, best + window + 1)
            w = np.exp(logits[lo:hi] - logits[lo:hi].max())
            cell = float((w * np.arange(lo, hi)).sum() / w.sum())
            out[k, r] = (cell + 0.5) * WIDTH / cells
    return out


def fit(points: np.ndarray, x_min: float, x_max: float, min_points: int = 4) -> Optional[
        np.ndarray]:
    """Fit y = a x² + b x + c through the ground points from x_min to x_max; None if too few."""
    keep = points[(points[:, 0] >= x_min) & (points[:, 0] <= x_max)]
    keep = keep[~np.isnan(keep).any(axis=1)]
    if len(keep) < min_points or np.ptp(keep[:, 0]) < 2.0:
        return None
    degree = 2 if np.ptp(keep[:, 0]) >= 4.0 else 1
    p = np.polyfit(keep[:, 0], keep[:, 1], degree)
    return np.concatenate([np.zeros(3 - len(p)), p])


# --- Making the data set ---

def render(binary: Path, world: Path, poses: Sequence, out: Path, log: Path):
    """Draw the front left camera at each pose with sim_cameras --poses."""
    out.mkdir(parents=True, exist_ok=True)
    pose_file = out / 'poses.csv'
    pose_file.write_text(''.join(f'{x:.3f},{y:.3f},{yaw:.5f}\n' for x, y, yaw, *_ in poses))
    with open(log, 'a') as f:
        subprocess.run([str(binary), '--world', str(world), '--cameras', 'front_left',
                        '--poses', str(pose_file), '--save', str(out)],
                       check=True, stdout=f, stderr=f)


def crop(png: Path) -> np.ndarray:
    """Return a saved picture as the model takes it: rows CROP_TOP.., INPUT_W x INPUT_H, RGB."""
    from PIL import Image
    with Image.open(png) as im:
        im = im.convert('RGB').crop((0, CROP_TOP, WIDTH, HEIGHT))
        return np.asarray(im.resize((INPUT_W, INPUT_H), Image.BILINEAR))


def held_out(world: str, road: int, rng: random.Random) -> int:
    """Return 1 for the scores: the campus's every 7th road (never trained on); 15 % else."""
    if world == 'campus':
        return int(road % 7 == 3)
    return int(rng.random() < 0.15)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--out', type=Path, default=ROOT / 'models' / 'lane-data')
    p.add_argument('--road', type=int, default=1500, help='pictures from the road world')
    p.add_argument('--campus', type=int, default=6000, help='pictures from the campus')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--campus-dir', type=Path, default=CAMPUS,
                   help='the campus world (pixi run campus), from the main checkout in a worktree')
    p.add_argument('--sim-cameras', type=Path, default=SIM_CAMERAS,
                   help='the sim_cameras program (cargo build --release in aav_sim_cameras)')
    args = p.parse_args(argv)
    if not args.sim_cameras.exists():
        print(f'{args.sim_cameras}: missing. cd ros/sim/aav_sim_cameras && pixi run cargo '
              'build --release', file=sys.stderr)
        return 2
    rng = random.Random(args.seed)
    cam = load_camera()
    worlds = [('road', ROAD_WORLD, ROAD_LANES, False, args.road)]
    campus = args.campus_dir
    if (campus / 'lanes.yaml').exists():
        worlds.append(('campus', campus / 'campus.sdf', campus / 'lanes.yaml', True, args.campus))
    else:
        print('no maps/campus (pixi run campus): the road world only', file=sys.stderr)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    log = out / 'sim_cameras.log'
    images, cols, colours, gts, held, world_ids, poses_all = [], [], [], [], [], [], []
    for wi, (name, world, lanes, centre, n) in enumerate(worlds):
        if n <= 0:
            continue
        t0 = time.time()
        roads, paint = painted(lanes, centre)
        poses = sample_poses(roads, n, rng, paint)
        pictures = out / f'png-{name}'
        render(args.sim_cameras, world, poses, pictures, log)
        grass = 0
        for k, pose in enumerate(poses):
            picture = crop(pictures / f'front_left_{k:05d}.png')
            if not on_tarmac(picture):
                grass += 1      # a road in the lane map the world draws no tarmac for
                continue
            c, colour, gt = label(paint, pose[:3], cam)
            images.append(picture)
            cols.append(c)
            colours.append(colour)
            gts.append(gt)
            held.append(held_out(name, pose[3], rng))
            world_ids.append(wi)
            poses_all.append(pose[:3])
        shutil.rmtree(pictures)
        print(f'{name}: {len(poses) - grass} pictures ({grass} on grass left out) in '
              f'{time.time() - t0:.0f} s')
    np.save(out / 'images.npy', np.stack(images))
    np.savez_compressed(out / 'labels.npz', cols=np.stack(cols), colour=np.stack(colours),
                        gt_y=np.stack(gts), held=np.array(held), world=np.array(world_ids),
                        poses=np.array(poses_all))
    (out / 'meta.json').write_text(json.dumps(meta(cam, [w[0] for w in worlds]), indent=1))
    found = np.stack(cols)
    print(f'{len(images)} pictures ({sum(held)} held out); lines labelled: '
          + ', '.join(f'{n} {np.mean((~np.isnan(found[:, k])).any(axis=1)) * 100:.0f} %'
                      for k, n in enumerate(LANES)))
    return 0


def meta(cam: Camera, worlds: Sequence[str]) -> dict:
    """Return what the model was made for: the node reads this beside the .onnx."""
    return {'width': WIDTH, 'height': HEIGHT, 'crop_top': CROP_TOP, 'input_w': INPUT_W,
            'input_h': INPUT_H, 'rows': list(ROWS), 'cells': CELLS, 'lanes': list(LANES),
            'mean': [0.485, 0.456, 0.406], 'std': [0.229, 0.224, 0.225],
            'camera': {'fx': cam.fx, 'fy': cam.fy, 'cx': cam.cx, 'cy': cam.cy,
                       'xyz': list(cam.xyz), 'rpy': list(cam.rpy)},
            'worlds': list(worlds)}


if __name__ == '__main__':
    sys.exit(main())
