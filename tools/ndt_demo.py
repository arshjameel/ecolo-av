#!/usr/bin/env python3
"""
NDT and ICP, from above: the teaching example for matching a lidar scan to a map.

    pixi run ndt-demo                 print the numbers, draw docs/img/learn-ndt.png, learn-icp.png
    pixi run ndt-demo --no-figures    the numbers only

The page that walks through it: docs/learn-localization.md, section 8. The lot is made
up (a wall, two curbs, three posts, a parked car), seen from above, in 2D; the real
thing matches the VLP-16's 3D points the same way. No ROS needed: numpy, scipy, matplotlib.

NDT (Biber and Strasser, 2003): cut the map into squares; in each, keep only the mean and
the spread (covariance) of its points: a blob. Score a pose by how well the scan's points
land on the blobs, and move the pose to raise the score.

ICP (Besl and McKay, 1992): pair each scan point with its nearest map point, find the
move that best lines up the pairs, and repeat.
"""

import argparse
from pathlib import Path
import sys

import numpy as np
from scipy import optimize
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / 'docs' / 'img'

CELL = 2.0          # m: the side of an NDT square
MIN_POINTS = 4      # a square needs this many points for a blob
NOISE = 0.04        # m: the lidar's range noise
# The guess is off by this much: the dead reckoning's drift (x m, y m, heading rad).
ERROR = (1.2, -0.8, np.radians(9.0))

# Our colours (the docs' palette): map, blobs, scan, ink.
GREY, BLUE, ORANGE, INK = '#8a8984', '#2a78d6', '#eb6834', '#0b0b0b'


def segment(a, b, step, rng):
    """Points along a wall or curb from a to b, about step apart."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    t = np.linspace(0.0, 1.0, n)[:, None]
    return a + t * (b - a) + rng.normal(0.0, NOISE, (n, 2))


def circle(centre, radius, n, rng):
    """Points around a post."""
    angle = rng.uniform(0.0, 2 * np.pi, n)
    ring = np.c_[np.cos(angle), np.sin(angle)] * radius
    return np.asarray(centre) + ring + rng.normal(0.0, NOISE, (n, 2))


def lot(rng, step):
    """Make up the lot, in map coordinates (metres)."""
    parts = [
        segment((-12, 12), (12, 12), step, rng),        # a building's wall
        segment((-12, -6), (-12, 12), step, rng),       # the west curb
        segment((-12, -6), (4, -6), step, rng),         # the south curb
        segment((4, -6), (12, -2), step, rng),          # the curb bends
        segment((5, 4), (9, 4), step, rng),             # a parked car: four sides
        segment((9, 4), (9, 6), step, rng),
        segment((5, 6), (9, 6), step, rng),
        segment((5, 4), (5, 6), step, rng),
    ]
    parts += [circle(c, 0.15, 12, rng) for c in ((-6, 3), (-1, 3), (-6, -1))]  # posts
    return np.vstack(parts)


def transform(points, pose):
    """Move points by pose = (x, y, heading)."""
    x, y, th = pose
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return points @ rot.T + (x, y)


def ndt_blobs(points, cell=CELL):
    """Cut into squares of side cell; per square with enough points: (mean, covariance)."""
    keys = np.floor(points / cell).astype(int)
    blobs = {}
    for key in {tuple(k) for k in keys}:
        inside = points[(keys == key).all(axis=1)]
        if len(inside) < MIN_POINTS:
            continue
        cov = np.cov(inside.T)
        # A wall's blob is flat: keep it from being a line (the paper's fix).
        values, vectors = np.linalg.eigh(cov)
        values = np.maximum(values, 0.01 * values.max())
        blobs[key] = (inside.mean(axis=0), vectors @ np.diag(values) @ vectors.T)
    return blobs


NEIGHBOURS = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1)]


def ndt_score(pose, scan, blobs, cell):
    """
    How well the moved scan lands on the blobs: higher is better.

    Each point is scored against its own square and the eight around it, so a point
    just over an edge still feels the blob next door (as Autoware's ndt_omp does).
    """
    moved = transform(scan, pose)
    keys = np.floor(moved / cell).astype(int)
    score = 0.0
    for point, (kx, ky) in zip(moved, keys):
        for dx, dy in NEIGHBOURS:
            blob = blobs.get((kx + dx, ky + dy))
            if blob is not None:
                d = point - blob[0]
                score += np.exp(-0.5 * d @ np.linalg.solve(blob[1], d))
    return score


def ndt_match(scan, map_points, cells=(6.0, 4.0, CELL)):
    """
    Find the pose that best lays the scan on the blobs: big squares first, then small.

    Big squares see far, small ones are precise; each stage starts where the last ended.
    """
    pose = np.zeros(3)
    for cell in cells:
        blobs = ndt_blobs(map_points, cell)
        pose = optimize.minimize(
            lambda p: -ndt_score(p, scan, blobs, cell), pose, method='Powell',
            options={'xtol': 1e-4, 'ftol': 1e-6}).x
    return pose


def icp_match(scan, map_points, steps=30):
    """Point-to-point ICP: the pose after each step (the first is no move)."""
    tree = cKDTree(map_points)
    pose = np.zeros(3)
    history = [pose.copy()]
    for _ in range(steps):
        moved = transform(scan, pose)
        _, nearest = tree.query(moved)
        target = map_points[nearest]
        # The rotation and shift that best line up the pairs (Kabsch, with SVD).
        a, b = moved - moved.mean(axis=0), target - target.mean(axis=0)
        u, _, vt = np.linalg.svd(a.T @ b)
        rot = (u @ vt).T
        if np.linalg.det(rot) < 0:
            vt[-1] *= -1
            rot = (u @ vt).T
        shift = target.mean(axis=0) - moved.mean(axis=0) @ rot.T
        th = np.arctan2(rot[1, 0], rot[0, 0])
        # Compose: the new step after the pose so far.
        pose = np.r_[rot @ pose[:2] + shift, pose[2] + th]
        history.append(pose.copy())
        if np.allclose(history[-1], history[-2], atol=1e-6):
            break
    return history


def pose_error(found):
    """How far the found correction is from the true one: (m, degrees)."""
    # The scan was moved by the inverse of ERROR, so the right fix is ERROR itself.
    return (float(np.hypot(*(found[:2] - ERROR[:2]))),
            float(np.degrees(abs(found[2] - ERROR[2]))))


def make_scan(rng):
    """Scan the same lot afresh, as seen from the wrong guess."""
    seen = lot(rng, step=0.5)
    x, y, th = ERROR
    # Undo ERROR: the points as the car thinks it sees them.
    inverse = (-(np.cos(th) * x + np.sin(th) * y), np.sin(th) * x - np.cos(th) * y, -th)
    return transform(seen, inverse)


def style(ax, title):
    ax.set_title(title, loc='left', fontsize=13, color=INK, pad=8)
    ax.set_aspect('equal')
    ax.set_xlim(-15, 15)
    ax.set_ylim(-10, 15.5)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)


def draw_grid(ax):
    for v in np.arange(-14, 15, CELL):
        ax.axvline(v, color='#e4e3df', lw=0.8, zorder=0)
    for v in np.arange(-10, 15, CELL):
        ax.axhline(v, color='#e4e3df', lw=0.8, zorder=0)


def draw_blobs(ax, blobs):
    from matplotlib.patches import Ellipse
    for mean, cov in blobs.values():
        values, vectors = np.linalg.eigh(cov)
        angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))
        width, height = 2 * 2 * np.sqrt(values[::-1])  # two standard deviations
        ax.add_patch(Ellipse(mean, width, height, angle=angle, facecolor=BLUE, alpha=0.30,
                             edgecolor=BLUE, lw=1.0, zorder=2))


def draw_car(ax, pose, colour, label):
    """Draw an arrow: where the car is, and which way it faces."""
    x, y, th = pose
    ax.annotate('', xy=(x + 1.6 * np.cos(th), y + 1.6 * np.sin(th)), xytext=(x, y),
                arrowprops={'arrowstyle': '-|>', 'color': colour, 'lw': 2.5}, zorder=5)
    ax.text(x, y - 1.3, label, fontsize=10, color=INK, ha='left', zorder=5)


def figures(map_points, blobs, scan, ndt_pose, icp_history):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # NDT, in four steps.
    fig, axes = plt.subplots(2, 2, figsize=(11, 9.5))
    a, b, c, d = axes.ravel()
    style(a, "1. The map: the lidar's points, from above")
    a.scatter(*map_points.T, s=4, color=GREY, zorder=3)
    style(b, f'2. Cut into {CELL:.0f} m squares; each becomes a blob')
    draw_grid(b)
    draw_blobs(b, blobs)
    style(c, '3. A new scan, where the car thinks it is')
    draw_grid(c)
    draw_blobs(c, blobs)
    c.scatter(*scan.T, s=6, color=ORANGE, zorder=4)
    draw_car(c, (0, 0, 0), ORANGE, 'the car, by dead reckoning')
    c.text(-14.5, -9.5, "orange: the new scan    blue: the map's blobs", fontsize=10, color=INK)
    metres, degrees = pose_error(ndt_pose)
    style(d, '4. Slide and turn it until it sits on the blobs')
    draw_grid(d)
    draw_blobs(d, blobs)
    d.scatter(*transform(scan, ndt_pose).T, s=6, color=ORANGE, zorder=4)
    draw_car(d, ndt_pose, ORANGE, 'the car, corrected')
    d.text(-14.5, -9.5, f'The car was off by {np.hypot(*ERROR[:2]):.1f} m and '
           f'{np.degrees(ERROR[2]):.0f}°. Left over: {metres * 100:.0f} cm, {degrees:.1f}°',
           fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(IMG / 'learn-ndt.png', dpi=110, facecolor='white')
    plt.close(fig)

    # ICP, before and after.
    tree = cKDTree(map_points)
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2))
    style(a, '1. Pair each scan point with its nearest map point')
    a.scatter(*map_points.T, s=4, color=GREY, zorder=3)
    a.scatter(*scan.T, s=6, color=ORANGE, zorder=4)
    a.text(-14.5, -9.5, 'grey: the map    orange: the new scan', fontsize=10, color=INK)
    _, nearest = tree.query(scan)
    for p, q in zip(scan[::3], map_points[nearest][::3]):
        a.plot(*np.c_[p, q], color=INK, lw=0.6, zorder=2)
    final = icp_history[-1]
    metres, degrees = pose_error(final)
    style(b, f'2. Move to close the pairs; repeat ({len(icp_history) - 1} times)')
    b.scatter(*map_points.T, s=4, color=GREY, zorder=3)
    b.scatter(*transform(scan, final).T, s=6, color=ORANGE, zorder=4)
    b.text(-14.5, -9.5, f'Left over: {metres * 100:.0f} cm, {degrees:.1f}°',
           fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(IMG / 'learn-icp.png', dpi=110, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--no-figures', action='store_true', help='print the numbers only')
    args = parser.parse_args()

    rng = np.random.default_rng(7)
    map_points = lot(rng, step=0.25)
    blobs = ndt_blobs(map_points)
    scan = make_scan(rng)

    ndt_pose = ndt_match(scan, map_points)
    icp_history = icp_match(scan, map_points)

    print(f'map: {len(map_points)} points, {len(blobs)} blobs; scan: {len(scan)} points')
    print(f'the guess was off by {np.hypot(*ERROR[:2]):.2f} m, {np.degrees(ERROR[2]):.1f}°')
    for name, pose in (('NDT', ndt_pose), ('ICP', icp_history[-1])):
        metres, degrees = pose_error(pose)
        print(f'{name}: left over {metres * 100:.1f} cm, {degrees:.2f}°')
    if not args.no_figures:
        figures(map_points, blobs, scan, ndt_pose, icp_history)
        print(f'drew {IMG / "learn-ndt.png"} and {IMG / "learn-icp.png"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
