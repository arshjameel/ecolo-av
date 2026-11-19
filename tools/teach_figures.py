#!/usr/bin/env python3
"""
Draw the teaching guide's diagrams: docs/how-self-driving-works.md.

Each picture is a sketch of an algorithm's idea, drawn from small made-up worlds, so it
can be rebuilt. Where the idea is cheap to run for real (ICP, A*, Dijkstra, Hybrid-A*,
RRT, MPPI, merging two drives), the script runs it; elsewhere it draws the idea.

Run:
    pixi run python3 tools/teach_figures.py [out dir]      # default: docs/img/teach

Needs matplotlib and numpy. No ROS. Writes SVGs only.
"""

import heapq
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import (Circle, Ellipse, FancyArrowPatch,  # noqa: E402
                                FancyBboxPatch, Polygon, Rectangle)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'docs' / 'img' / 'teach'

# A calm, colour-blind-safe set (the dataviz skill's reference palette, light mode).
BLUE = '#2a78d6'     # the estimate, the car, "ours"
ORANGE = '#eb6834'   # the new scan, the answer, the chosen path
AQUA = '#1baf7a'     # the goal, the fused result
INK = '#0b0b0b'      # walls, text
INK2 = '#52514e'     # secondary text
MUTED = '#9b9a95'    # light marks
GRID = '#e4e3df'     # grid lines, empty cells
WALL = '#3d3c39'

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 13,
    'axes.titlesize': 15,
    'axes.titleweight': 'bold',
    'axes.titlepad': 10,
    'axes.edgecolor': GRID,
    'text.color': INK,
    'svg.fonttype': 'none',
    'svg.hashsalt': 'aav-teach',
})


def blues(n_or_values, lo=0.25, hi=1.0):
    """One hue, light to dark: the sequential scale."""
    cmap = matplotlib.colormaps['Blues']
    return cmap(np.clip(lo + (hi - lo) * np.asarray(n_or_values, float), 0, 1))


def clean(ax, equal=True):
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)
    if equal:
        ax.set_aspect('equal')


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, format='svg', bbox_inches='tight', metadata={'Date': None},
                facecolor='white')
    plt.close(fig)
    print('wrote', OUT / name)


def note(ax, x, y, text, color=INK2, size=12, **kw):
    ax.text(x, y, text, color=color, fontsize=size, **kw)


def car(ax, x, y, th, length=1.0, width=0.5, color=BLUE, alpha=1.0, z=5):
    """Draw a car seen from above: a box with a pointed nose."""
    c, s = math.cos(th), math.sin(th)
    pts = [(-0.2 * length, -width / 2), (0.75 * length, -width / 2), (length, 0),
           (0.75 * length, width / 2), (-0.2 * length, width / 2)]
    ax.add_patch(Polygon([(x + c * px - s * py, y + s * px + c * py) for px, py in pts],
                         closed=True, fc=color, ec='white', lw=1.5, alpha=alpha, zorder=z))


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def segment_points(a, b, step=0.15, noise=0.03, rng=None):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    t = np.linspace(0, 1, n)[:, None]
    pts = a + t * (b - a)
    if rng is not None:
        pts = pts + rng.normal(0, noise, pts.shape)
    return pts


# --------------------------------------------------------------------------- big picture

def fig_pipeline():
    fig, ax = plt.subplots(figsize=(13, 6.2))
    clean(ax, equal=False)
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.2)

    def box(x, y, w, h, title, sub, color):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc='white', ec=color, lw=2.5))
        ax.text(x + w / 2, y + h * 0.64, title, ha='center', va='center', fontsize=13,
                fontweight='bold')
        ax.text(x + w / 2, y + h * 0.28, sub, ha='center', va='center', fontsize=10.5,
                color=INK2)

    def arrow(x0, y0, x1, y1, color=INK2):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=16,
                                     color=color, lw=1.8))

    top = 4.4
    steps = [('Sense', 'lidar, cameras,\nradar, GPS, IMU'),
             ('Localize', 'where am I?'),
             ('Perceive', "what's around me?"),
             ('Plan', 'which way,\nhow fast?'),
             ('Control', 'steer, throttle,\nbrake'),
             ('The car', 'motors and\nwheels')]
    w, gap = 1.75, 0.37
    for i, (t, s) in enumerate(steps):
        x = 0.2 + i * (w + gap)
        box(x, top, w, 1.3, t, s, ORANGE if t == 'Plan' else BLUE)
        if i:
            arrow(x - gap + 0.03, top + 0.65, x - 0.03, top + 0.65)
    ax.text(0.2, 6.0, '1. The pipeline: six steps, many times a second', fontsize=15,
            fontweight='bold')

    # The three planning levels, zoomed out to close up.
    ax.text(0.2, 3.35, '2. Plan has three levels', fontsize=15, fontweight='bold')
    plan_x = 0.2 + 3 * (w + gap)
    arrow(plan_x + w / 2, top - 0.05, plan_x + w / 2, 2.75, ORANGE)
    ax.text(plan_x + w / 2 + 0.15, 3.35, 'zoomed out to close up', fontsize=11.5, color=INK2)
    levels = [('Route', 'streets and lanes to the goal', 'kilometres · once a trip',
               'Dijkstra, A* on a road graph'),
              ('Path', 'a line the car can really drive', 'tens of metres · each second',
               'A*, Hybrid-A*, lattice, RRT'),
              ('Motion', 'the next 2-3 s of steering and speed',
               'a few metres · 20 times a second',
               'Pure Pursuit, MPC, MPPI')]
    lw = 4.05
    for i, (t, what, scale, how) in enumerate(levels):
        x = 0.2 + i * (lw + 0.3)
        ax.add_patch(FancyBboxPatch((x, 0.25), lw, 2.45,
                                    boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc='white', ec=ORANGE, lw=2.5))
        ax.text(x + 0.2, 2.25, t, fontsize=14, fontweight='bold', va='center')
        ax.text(x + 0.2, 1.75, what, fontsize=11.5, va='center')
        ax.text(x + 0.2, 1.3, scale, fontsize=11, color=INK2, va='center')
        ax.text(x + 0.2, 0.75, how, fontsize=11, color=BLUE, va='center', fontweight='bold')
        if i:
            arrow(x - 0.28, 1.5, x - 0.02, 1.5, ORANGE)
    save(fig, 'pipeline.svg')


# --------------------------------------------------------------------------- localization

def fig_kalman():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={'width_ratios': [1.15, 1]})
    x = np.linspace(0, 12, 600)

    def g(mu, sd):
        return np.exp(-0.5 * ((x - mu) / sd) ** 2) / (sd * math.sqrt(2 * math.pi))

    prior_mu, prior_sd = 3.0, 0.5
    pred_mu, pred_sd = 6.0, 1.1          # moved 3 m by the wheels; less sure
    meas_mu, meas_sd = 7.0, 0.8          # the GPS says 7 m
    k = pred_sd ** 2 / (pred_sd ** 2 + meas_sd ** 2)
    fuse_mu = pred_mu + k * (meas_mu - pred_mu)
    fuse_sd = math.sqrt((1 - k) * pred_sd ** 2)
    a1.fill_between(x, g(prior_mu, prior_sd), color=BLUE, alpha=0.15)
    a1.plot(x, g(prior_mu, prior_sd), color=BLUE, lw=2)
    a1.plot(x, g(pred_mu, pred_sd), color=BLUE, lw=2, ls='--')
    a1.plot(x, g(meas_mu, meas_sd), color=ORANGE, lw=2)
    a1.fill_between(x, g(fuse_mu, fuse_sd), color=AQUA, alpha=0.2)
    a1.plot(x, g(fuse_mu, fuse_sd), color=AQUA, lw=2.5)
    a1.annotate('', xy=(pred_mu - 0.9, 0.3), xytext=(prior_mu + 0.55, 0.45),
                arrowprops={'arrowstyle': '-|>', 'color': INK2, 'lw': 1.5})
    note(a1, 0.6, 0.83, '1. where I was', BLUE)
    note(a1, 3.9, 0.64, '2. predict:\nmoved 3 m,\nless sure', BLUE)
    note(a1, 8.3, 0.4, '3. GPS says 7 m', ORANGE)
    note(a1, 8.0, 0.62, '4. update:\nbetween the two,\nsurer than either', AQUA)
    a1.set_xlabel('position along the road (m)')
    a1.set_yticks([])
    a1.set_title('One step of a Kalman filter')
    for s in ('top', 'right', 'left'):
        a1.spines[s].set_visible(False)

    rng = np.random.default_rng(3)
    t = np.linspace(0, 1, 60)
    true = np.c_[t * 20, 4 * np.sin(t * 3)]
    drift = true + np.c_[np.zeros(60), np.cumsum(np.full(60, 0.05))] + np.c_[t * 0, t ** 2 * 2]
    gps_i = np.arange(0, 60, 5)
    gps = true[gps_i] + rng.normal(0, 0.7, (len(gps_i), 2))
    ekf = true + rng.normal(0, 0.08, true.shape)
    a2.plot(*true.T, color=MUTED, lw=6, alpha=0.5, solid_capstyle='round', label='the truth')
    a2.plot(*drift.T, color=BLUE, lw=2, ls='--', label='wheels + IMU only: drifts')
    a2.scatter(*gps.T, s=60, color=ORANGE, zorder=4, label='GPS fixes: jumpy', ec='white')
    a2.plot(*ekf.T, color=AQUA, lw=2.5, label='EKF: smooth and true')
    a2.legend(loc='upper left', frameon=False, fontsize=11)
    a2.set_title('Many steps: the EKF blends them')
    clean(a2)
    a2.set_ylim(-6, 13)
    save(fig, 'kalman.svg')


def icp_world(rng):
    walls = [((0, 0), (8, 0)), ((8, 0), (8, 5)), ((0, 0), (0, 3)), ((3, 3.5), (5, 3.5)),
             ((5, 3.5), (5, 5))]
    pts = np.vstack([segment_points(a, b, 0.25, 0.02, rng) for a, b in walls])
    return pts


def run_icp(scan, model, iters=12):
    """Point-to-point ICP in 2D (Besl and McKay, 1992). Returns the scan at each step."""
    steps = [scan.copy()]
    cur = scan.copy()
    for _ in range(iters):
        d = np.linalg.norm(cur[:, None, :] - model[None, :, :], axis=2)
        nn = model[d.argmin(axis=1)]
        mc, nc = cur.mean(0), nn.mean(0)
        h = (cur - mc).T @ (nn - nc)
        u, _, vt = np.linalg.svd(h)
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[1] *= -1
            r = vt.T @ u.T
        cur = (cur - mc) @ r.T + nc
        steps.append(cur.copy())
    return steps


def fig_icp():
    rng = np.random.default_rng(1)
    model = icp_world(rng)
    scan = model[rng.choice(len(model), 45, replace=False)] + rng.normal(0, 0.03, (45, 2))
    scan = (scan - scan.mean(0)) @ rot(math.radians(14)).T + scan.mean(0) + np.array([0.9, -0.6])
    steps = run_icp(scan, model)
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    shown = [(0, '1. Pair each point with its nearest'), (1, '2. Move to close the pairs'),
             (len(steps) - 1, f'3. Repeat: lined up after {len(steps) - 1} goes')]
    for ax, (i, title) in zip(axes, shown):
        cur = steps[i]
        d = np.linalg.norm(cur[:, None, :] - model[None, :, :], axis=2)
        nn = model[d.argmin(axis=1)]
        ax.scatter(*model.T, s=10, color=MUTED, label='the map')
        if i < len(steps) - 1:
            ax.add_collection(LineCollection(np.stack([cur, nn], 1), colors=INK2, lw=0.8))
        ax.scatter(*cur.T, s=26, color=ORANGE, zorder=3, label='the new scan', ec='white', lw=0.5)
        ax.set_title(title, fontsize=13.5)
        clean(ax)
        ax.set_xlim(-0.8, 9.8)
        ax.set_ylim(-1.6, 5.8)
    axes[0].legend(loc='upper left', frameon=False, fontsize=11, markerscale=1.5)
    save(fig, 'icp.svg')


def ndt_world(rng):
    walls = [((0, 0), (9, 0)), ((9, 0), (9, 6)), ((0, 0), (0, 4)), ((3.2, 3.6), (6, 3.6)),
             ((6, 3.6), (6, 6))]
    pts = [segment_points(a, b, 0.12, 0.06, rng) for a, b in walls]
    ang = np.linspace(0.2, 1.4, 25)
    bend = np.c_[1.5 + 1.6 * np.cos(ang), 4.2 + 1.6 * np.sin(ang)]
    pts.append(bend + rng.normal(0, 0.05, (25, 2)))
    return np.vstack(pts)


def fig_ndt():
    rng = np.random.default_rng(7)
    pts = ndt_world(rng)
    cell = 1.5
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.8))
    lims = {'xlim': (-1.0, 10.3), 'ylim': (-1.2, 7.2)}

    def grid(ax):
        for gx in np.arange(-1.5, 10.6, cell):
            ax.axvline(gx, color=GRID, lw=1, zorder=0)
        for gy in np.arange(-1.5, 7.6, cell):
            ax.axhline(gy, color=GRID, lw=1, zorder=0)

    keys = np.floor((pts + 1.5) / cell).astype(int)
    cells = {}
    for k, p in zip(map(tuple, keys), pts):
        cells.setdefault(k, []).append(p)
    blobs = []
    for k, ps in cells.items():
        ps = np.array(ps)
        if len(ps) < 4:
            continue
        mu = ps.mean(0)
        cov = np.cov(ps.T) + np.eye(2) * 0.004
        blobs.append((k, mu, cov))

    def ellipse(ax, mu, cov, **kw):
        vals, vecs = np.linalg.eigh(cov)
        ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
        w, h = 2 * 2 * np.sqrt(vals[::-1])
        ax.add_patch(Ellipse(mu, w, h, angle=ang, **kw))

    a = axes[0]
    a.scatter(*pts.T, s=6, color=INK2)
    a.set_title('1. The map: points')
    a = axes[1]
    grid(a)
    for k, _, _ in blobs:
        a.add_patch(Rectangle((k[0] * cell - 1.5, k[1] * cell - 1.5), cell, cell, fc=BLUE,
                              alpha=0.12, lw=0))
    a.scatter(*pts.T, s=6, color=INK2)
    a.set_title('2. Cut into cubes')
    a = axes[2]
    grid(a)
    for _, mu, cov in blobs:
        ellipse(a, mu, cov, fc=BLUE, alpha=0.35, ec=BLUE, lw=1.5)
    a.set_title('3. One blob per cube')
    note(a, -0.8, -1.05, 'its centre and spread: a "normal distribution"', size=10.5)
    a = axes[3]
    grid(a)
    for _, mu, cov in blobs:
        ellipse(a, mu, cov, fc=BLUE, alpha=0.25, ec=BLUE, lw=1)
    scan = pts[rng.choice(len(pts), 110, replace=False)]
    c = scan.mean(0)
    for f, alpha in ((1.0, 0.45), (0.0, 1.0)):
        moved = (scan - c) @ rot(math.radians(-8 * f)).T + c + f * np.array([1.6, 1.2])
        a.scatter(*moved.T, s=16 if f == 0 else 10, color=ORANGE, alpha=alpha, zorder=3,
                  ec='none')
    a.annotate('', xy=c, xytext=c + np.array([1.6, 1.2]),
               arrowprops={'arrowstyle': '-|>', 'color': ORANGE, 'lw': 2})
    a.set_title('4. Slide the scan in')
    note(a, -0.8, -1.05, 'faint: where the scan landed; bold: fitted', size=10.5)
    for a in axes:
        clean(a)
        a.set(**lims)
    save(fig, 'ndt.svg')


def fig_ndt_wrong():
    """Ben's point: NDT can be sure, and wrong."""
    rng = np.random.default_rng(11)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.4), gridspec_kw={'width_ratios': [2.3, 1.8]})
    # A long corridor: every slide along it fits the same.
    for y in (0, 3):
        p = segment_points((0, y), (20, y), 0.2, 0.04, rng)
        a1.scatter(*p.T, s=6, color=INK2)
    for x0, color, lab, ly in ((2, BLUE, 'where the car is', 3.6),
                               (13, ORANGE, 'where NDT says', -1.0)):
        scan = np.vstack([segment_points((x0, 0), (x0 + 4, 0), 0.25, 0.03, rng),
                          segment_points((x0, 3), (x0 + 4, 3), 0.25, 0.03, rng)])
        a1.scatter(*scan.T, s=22, color=color, zorder=3)
        car(a1, x0 + 1.4, 1.5, 0, 1.4, 0.7, color=color)
        note(a1, x0, ly, lab, color, 12, fontweight='bold')
    a1.annotate('', xy=(13.5, 1.5), xytext=(4.2, 1.5),
                arrowprops={'arrowstyle': '-|>', 'color': INK2, 'lw': 1.5, 'ls': '--'})
    a1.set_title('A long corridor: the scan fits anywhere along it')
    clean(a1)
    a1.set_ylim(-3.2, 5.5)
    # Two buildings that look the same.
    for bx in (0, 9):
        for a, b in (((bx, 0), (bx + 4, 0)), ((bx + 4, 0), (bx + 4, 3)), ((bx, 0), (bx, 3)),
                     ((bx, 3), (bx + 4, 3))):
            a2.scatter(*segment_points(a, b, 0.2, 0.03, rng).T, s=6, color=INK2)
    car(a2, 2.0, -1.5, math.pi / 2, 1.0, 0.6, color=BLUE)
    car(a2, 11.0, -1.5, math.pi / 2, 1.0, 0.6, color=ORANGE)
    note(a2, 0.2, -2.6, 'the truth', BLUE, 12, fontweight='bold')
    note(a2, 9.2, -2.6, 'a confident wrong match', ORANGE, 12, fontweight='bold')
    a2.set_title('Two buildings alike: it may pick the wrong one')
    clean(a2)
    a2.set_ylim(-3.6, 5.5)
    save(fig, 'ndt-wrong-place.svg')


def amcl_map():
    walls = [((0, 0), (14, 0)), ((14, 0), (14, 9)), ((14, 9), (0, 9)), ((0, 9), (0, 0)),
             ((4, 0), (4, 5)), ((9, 9), (9, 4)), ((9, 4), (11, 4)), ((6, 2), (7.5, 2))]
    return walls


def fig_amcl():
    rng = np.random.default_rng(5)
    walls = amcl_map()
    truth = np.array([11.8, 6.6, math.radians(200)])
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))

    def draw_map(ax):
        for a, b in walls:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=WALL, lw=4, solid_capstyle='round')

    def free(p):
        x, y = p[:, 0], p[:, 1]
        ok = (x > 0.3) & (x < 13.7) & (y > 0.3) & (y < 8.7)
        ok &= ~((abs(x - 4) < 0.3) & (y < 5.3))
        ok &= ~((abs(x - 9) < 0.3) & (y > 3.7))
        return p[ok]

    n = 500
    uni = free(np.c_[rng.uniform(0, 14, n), rng.uniform(0, 9, n), rng.uniform(-np.pi, np.pi, n)])
    guesses = [(np.array([11.8, 6.6]), 200), (np.array([6.4, 6.9]), 200),
               (np.array([2.0, 7.0]), 20)]
    mid = np.vstack([np.c_[c + rng.normal(0, 0.55, (110, 2)),
                           np.radians(h) + rng.normal(0, 0.25, 110)]
                     for c, h in guesses])
    mid = free(mid)
    end = np.c_[truth[:2] + rng.normal(0, 0.18, (120, 2)), truth[2] + rng.normal(0, 0.06, 120)]
    panels = [(uni, '1. Start: guesses everywhere',
               'each dot is one guess of the pose'),
              (mid, '2. Weigh, keep the good, resample',
               'a few places fit the scan'),
              (end, '3. After more moves: found',
               'only one place fits every scan')]
    for ax, (p, title, sub) in zip(axes, panels):
        draw_map(ax)
        L = 0.45
        ax.quiver(p[:, 0], p[:, 1], np.cos(p[:, 2]) * L, np.sin(p[:, 2]) * L, color=BLUE,
                  angles='xy', scale_units='xy', scale=1, width=0.004, alpha=0.6,
                  headwidth=3, headlength=3)
        car(ax, truth[0], truth[1], truth[2], 0.9, 0.5, color=ORANGE, z=6)
        ax.set_title(title, fontsize=14)
        note(ax, 0.1, -1.0, sub, size=11.5)
        clean(ax)
        ax.set_xlim(-0.5, 14.5)
        ax.set_ylim(-1.4, 9.5)
    note(axes[0], 9.5, 7.6, 'the car', ORANGE, 12, fontweight='bold')
    save(fig, 'amcl.svg')


def fig_slam():
    rng = np.random.default_rng(2)
    n = 25
    t = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    true = np.c_[8 * np.cos(t), 5 * np.sin(t)]
    # Odometry: an error that grows with every step, so the end misses the start.
    grow = (np.arange(n) / (n - 1))[:, None] ** 1.5
    drift = np.array([2.2, 3.4])
    est = true + grow * drift + rng.normal(0, 0.08, true.shape)
    fixed = true + rng.normal(0, 0.08, true.shape)
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    titles = ['1. Drive a loop: errors add up', '2. "I have been here before"',
              '3. The pose graph pulls it true']
    for i, (ax, title) in enumerate(zip(axes, titles)):
        ax.plot(*np.vstack([true, true[:1]]).T, color=MUTED, lw=6, alpha=0.35,
                solid_capstyle='round')
        path = fixed if i == 2 else est
        if i == 2:
            ring = np.vstack([path, path[:1]])
            ax.plot(*ring.T, color=BLUE, lw=1.6, zorder=2)
        else:
            ax.plot(*path.T, color=BLUE, lw=1.6, zorder=2)
        ax.scatter(*path.T, s=30, color=BLUE, zorder=3, ec='white', lw=0.8)
        ax.scatter(*path[0], s=120, color=AQUA, zorder=4, ec='white', lw=1, marker='s')
        end = path[-1]
        if i == 0:
            note(ax, end[0] + 0.8, end[1] + 0.3, 'the end\nmisses\nthe start', BLUE, 12,
                 fontweight='bold')
            note(ax, 9.2, -1.2, 'start', AQUA, 12, fontweight='bold')
            note(ax, -9.5, -7.6, 'grey: the truth', size=11)
        if i == 1:
            ax.plot([end[0], path[0, 0]], [end[1], path[0, 1]], color=ORANGE, lw=3.5,
                    ls='--', zorder=4)
            note(ax, end[0] + 0.8, end[1] - 1.2, 'loop closure:\nthe scans\nmatch', ORANGE,
                 12, fontweight='bold')
        if i == 2:
            note(ax, -9.5, -7.6, 'each dot a pose; each line a constraint', size=11)
        ax.set_title(title, fontsize=14)
        clean(ax)
        ax.set_xlim(-10, 15.5)
        ax.set_ylim(-8.2, 7.5)
    save(fig, 'slam-loop.svg')


# --------------------------------------------------------------------------- planning

def grid_world():
    w, h = 24, 15
    occ = np.zeros((h, w), bool)
    occ[2:13, 12] = True
    occ[12, 6:13] = True
    occ[5, 16:22] = True
    return occ


def grid_search(occ, start, goal, use_h):
    h_fn = (lambda p: math.hypot(p[0] - goal[0], p[1] - goal[1])) if use_h else (lambda p: 0)
    openq = [(h_fn(start), 0.0, start)]
    g = {start: 0.0}
    came = {}
    order = {}
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414),
             (-1, 1, 1.414), (-1, -1, 1.414)]
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in order:
            continue
        order[cur] = len(order)
        if cur == goal:
            break
        for dx, dy, c in moves:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < occ.shape[1] and 0 <= ny < occ.shape[0]) or occ[ny, nx]:
                continue
            if dx and dy and (occ[cur[1], nx] or occ[ny, cur[0]]):
                continue
            ng = gc + c
            if ng < g.get((nx, ny), 1e9):
                g[(nx, ny)] = ng
                came[(nx, ny)] = cur
                heapq.heappush(openq, (ng + h_fn((nx, ny)), ng, (nx, ny)))
    path = [goal]
    while path[-1] != start:
        path.append(came[path[-1]])
    return order, path[::-1]


def fig_astar():
    occ = grid_world()
    start, goal = (3, 7), (21, 10)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2))
    for ax, use_h, title in ((axes[0], False, 'Dijkstra: spreads out evenly'),
                             (axes[1], True, 'A*: leans toward the goal')):
        order, path = grid_search(occ, start, goal, use_h)
        n = max(order.values())
        for (x, y), i in order.items():
            ax.add_patch(Rectangle((x, y), 1, 1, fc=blues(i / n, 0.15, 0.6), ec='white', lw=0.6))
        for y, x in zip(*np.nonzero(occ)):
            ax.add_patch(Rectangle((x, y), 1, 1, fc=WALL, ec='white', lw=0.6))
        p = np.array(path) + 0.5
        ax.plot(*p.T, color=ORANGE, lw=3.5, solid_capstyle='round', zorder=4)
        ax.scatter(*(np.array(start) + 0.5), s=160, color=BLUE, zorder=5, ec='white', lw=1.5)
        ax.scatter(*(np.array(goal) + 0.5), s=200, color=AQUA, marker='*', zorder=5,
                   ec='white', lw=1)
        ax.set_title(title)
        note(ax, 0, -1.2, f'{len(order)} squares looked at; a shortest path, the same length',
             size=11.5)
        ax.set_xlim(0, occ.shape[1])
        ax.set_ylim(-2.8, occ.shape[0])
        clean(ax)
        for gx in range(occ.shape[1] + 1):
            ax.plot([gx, gx], [0, occ.shape[0]], color=GRID, lw=0.6, zorder=0)
        for gy in range(occ.shape[0] + 1):
            ax.plot([0, occ.shape[1]], [gy, gy], color=GRID, lw=0.6, zorder=0)
    note(axes[1], 0, -2.4, 'darker blue = looked at later', size=10.5)
    save(fig, 'astar.svg')


def arc(x, y, th, kappa, length, n=12):
    """Drive along a circle, or straight, from a pose; return the points and end heading."""
    s = np.linspace(0, length, n)
    if abs(kappa) < 1e-9:
        return np.c_[x + s * math.cos(th), y + s * math.sin(th)], th
    xs = x + (np.sin(th + kappa * s) - math.sin(th)) / kappa
    ys = y - (np.cos(th + kappa * s) - math.cos(th)) / kappa
    return np.c_[xs, ys], th + kappa * length


def hybrid_obstacles():
    return [(5.5, 0.0, 2.0, 5.5), (5.5, 8.0, 2.0, 4.0), (11.0, 3.0, 4.5, 1.6)]


def blocked(p, obs, margin=0.45):
    for ox, oy, w, h in obs:
        inside = ((p[:, 0] > ox - margin) & (p[:, 0] < ox + w + margin)
                  & (p[:, 1] > oy - margin) & (p[:, 1] < oy + h + margin))
        if inside.any():
            return True
    return bool(((p[:, 0] < 0.3) | (p[:, 0] > 17.7) | (p[:, 1] < 0.3) | (p[:, 1] > 11.7)).any())


def hybrid_search(start, goal, obs, kmax=0.42, step=1.4):
    def key(s):
        return (round(s[0] / 0.7), round(s[1] / 0.7), round(s[2] / (math.pi / 8)) % 16)

    def h(s):
        return math.hypot(s[0] - goal[0], s[1] - goal[1])

    openq = [(h(start), 0.0, 0, start)]
    came = {key(start): (None, None, start)}
    seen = set()
    expanded = []
    tick = 1
    while openq:
        _, g, _, s = heapq.heappop(openq)
        k = key(s)
        if k in seen:
            continue
        seen.add(k)
        if h(s) < 0.9 and abs(math.remainder(s[2] - goal[2], 2 * math.pi)) < 0.5:
            path = []
            while k is not None:
                parent, pts, _ = came[k]
                if pts is not None:
                    path.append(pts)
                k = parent
            return expanded, path[::-1]
        for kappa in (kmax, 0.0, -kmax):
            pts, th = arc(s[0], s[1], s[2], kappa, step)
            if blocked(pts, obs):
                continue
            ns = (pts[-1, 0], pts[-1, 1], th)
            nk = key(ns)
            if nk in seen:
                continue
            expanded.append(pts)
            cost = g + step * (1.0 if kappa == 0 else 1.1)
            came.setdefault(nk, (k, pts, ns))
            tick += 1
            heapq.heappush(openq, (cost + 1.3 * h(ns), cost, tick, ns))
    return expanded, []


def fig_hybrid():
    obs = hybrid_obstacles()
    start, goal = (2.0, 6.5, 0.0), (15.5, 9.5, math.pi / 2)
    expanded, path = hybrid_search(start, goal, obs)
    fig, ax = plt.subplots(figsize=(11, 7.4))
    for ox, oy, w, h in obs:
        ax.add_patch(Rectangle((ox, oy), w, h, fc=WALL, ec='none'))
    ax.add_collection(LineCollection(expanded, colors=BLUE, lw=0.9, alpha=0.3))
    if path:
        p = np.vstack(path)
        ax.plot(*p.T, color=ORANGE, lw=3.5, solid_capstyle='round', zorder=4)
    car(ax, start[0], start[1], start[2], 1.2, 0.65, color=BLUE, z=6)
    car(ax, goal[0], goal[1], goal[2], 1.2, 0.65, color=AQUA, z=6)
    # The three moves, drawn big in a corner.
    ox, oy = 1.0, 2.0
    for kappa, lab in ((0.42, 'full left'), (0.0, 'straight'), (-0.42, 'full right')):
        pts, _ = arc(ox, oy, 0.0, kappa, 2.6, 20)
        ax.plot(*pts.T, color=INK, lw=2)
        note(ax, pts[-1, 0] + 0.15, pts[-1, 1] - 0.1, lab, INK, 11)
    note(ax, 0.6, 4.0, 'each step: three car moves', INK2, 11.5)
    note(ax, 2.0, 7.4, 'start', BLUE, 12, fontweight='bold')
    note(ax, 14.0, 11.0, 'goal, facing north', AQUA, 12, fontweight='bold')
    note(ax, 8.6, 5.5, 'faint: every move tried\norange: the path found', INK2, 11.5)
    ax.set_title('Hybrid-A*: A* over moves a car can really make')
    clean(ax)
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 12)
    for side in ax.spines.values():
        side.set_visible(True)
        side.set_color(GRID)
    save(fig, 'hybrid-astar.svg')


def hermite(p0, th0, p1, th1, n=30):
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * 1.1
    t = np.linspace(0, 1, n)[:, None]
    m0 = d * np.array([math.cos(th0), math.sin(th0)])
    m1 = d * np.array([math.cos(th1), math.sin(th1)])
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * p0 + (t ** 3 - 2 * t ** 2 + t) * m0
            + (-2 * t ** 3 + 3 * t ** 2) * p1 + (t ** 3 - t ** 2) * m1)


def fig_lattice():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.6))
    q = math.pi / 4
    prims0 = [((3, 0), 0), ((3, 1), 0), ((3, -1), 0), ((2, 1), q), ((2, -1), -q),
              ((2, 2), 2 * q), ((2, -2), -2 * q)]
    for ax in (a1, a2):
        gx, gy = np.meshgrid(np.arange(-1, 13), np.arange(-4, 6))
        ax.scatter(gx, gy, s=10, color=MUTED, zorder=0)
    for (ex, ey), th in prims0:
        c = hermite((0, 0), 0, (ex, ey), th)
        a1.plot(*c.T, color=BLUE, lw=2.5)
        a1.annotate('', xy=(ex + 0.35 * math.cos(th), ey + 0.35 * math.sin(th)), xytext=(ex, ey),
                    arrowprops={'arrowstyle': '-|>', 'color': BLUE, 'lw': 1.5})
    back = hermite((0, 0), math.pi, (-1, 0), math.pi)
    a1.plot(*back.T, color=ORANGE, lw=2.5, ls='--')
    note(a1, -1.2, 0.4, 'reverse', ORANGE, 11)
    car(a1, -0.3, 0, 0, 0.8, 0.45, color=INK)
    a1.set_title('1. A ready-made list of moves')
    note(a1, -1, -3.8, 'every move ends on a dot, facing one of 8 or 16 headings', size=11)
    a1.set_xlim(-1.8, 4.2)
    a1.set_ylim(-4.2, 3.2)
    clean(a1)

    chain = [((0, 0), 0), ((3, 0), 0), ((5, 1), q), ((7, 3), q), ((10, 4), 0), ((12, 4), 0)]
    for ((p0, t0), (p1, t1)) in zip(chain, chain[1:]):
        a2.plot(*hermite(p0, t0, p1, t1).T, color=ORANGE, lw=3.5, solid_capstyle='round')
    for (p, t) in chain:
        a2.scatter(*p, s=55, color=ORANGE, zorder=4, ec='white', lw=1)
    a2.add_patch(Rectangle((5.6, -1.5), 3.0, 2.2, fc=WALL, ec='none'))
    car(a2, -0.3, 0, 0, 0.8, 0.45, color=BLUE)
    a2.scatter(12, 4, s=260, marker='*', color=AQUA, zorder=5, ec='white')
    a2.set_title('2. A* joins them, dot to dot, round the box')
    a2.set_xlim(-1.8, 13)
    a2.set_ylim(-4.2, 5.8)
    clean(a2)
    save(fig, 'lattice.svg')


def fig_rrt():
    rng = np.random.default_rng(4)
    obs = [(4, 0, 1.5, 6.5), (9, 3.5, 1.5, 6.5)]

    def hit(a, b):
        for t in np.linspace(0, 1, 12):
            p = a + t * (b - a)
            for ox, oy, w, h in obs:
                if ox - 0.2 < p[0] < ox + w + 0.2 and oy - 0.2 < p[1] < oy + h + 0.2:
                    return True
        return False

    start, goal = np.array([1.0, 8.0]), np.array([13.0, 1.5])
    nodes, parent = [start], [-1]
    found = None
    for _ in range(1500):
        r = goal if rng.random() < 0.08 else rng.uniform([0, 0], [14, 10])
        d = np.linalg.norm(np.array(nodes) - r, axis=1)
        i = int(d.argmin())
        step = r - nodes[i]
        n = nodes[i] + step / max(np.linalg.norm(step), 1e-9) * min(0.8, np.linalg.norm(step))
        if hit(nodes[i], n):
            continue
        nodes.append(n)
        parent.append(i)
        if np.linalg.norm(n - goal) < 0.6:
            found = len(nodes) - 1
            break
    fig, ax = plt.subplots(figsize=(9, 6.2))
    for ox, oy, w, h in obs:
        ax.add_patch(Rectangle((ox, oy), w, h, fc=WALL, ec='none'))
    segs = [[nodes[i], nodes[parent[i]]] for i in range(1, len(nodes))]
    ax.add_collection(LineCollection(segs, colors=BLUE, lw=1, alpha=0.5))
    if found is not None:
        path = [found]
        while parent[path[-1]] != -1:
            path.append(parent[path[-1]])
        p = np.array([nodes[i] for i in path])
        ax.plot(*p.T, color=ORANGE, lw=3.5, solid_capstyle='round', zorder=4)
    ax.scatter(*start, s=150, color=BLUE, zorder=5, ec='white', lw=1.5)
    ax.scatter(*goal, s=260, marker='*', color=AQUA, zorder=5, ec='white')
    ax.set_title(f'RRT: grow a tree by random steps ({len(nodes)} points)')
    clean(ax)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 10)
    for side in ax.spines.values():
        side.set_visible(True)
        side.set_color(GRID)
    save(fig, 'rrt.svg')


# --------------------------------------------------------------------------- motion

def bicycle_rollout(steers, v=2.0, dt=0.1, wb=1.0, x0=(0.0, 0.0, 0.0)):
    x, y, th = x0
    out = [(x, y)]
    for d in steers:
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v / wb * math.tan(d) * dt
        out.append((x, y))
    return np.array(out)


def fig_mppi():
    rng = np.random.default_rng(8)
    steps, k, v, dt = 28, 400, 2.0, 0.1
    s = np.linspace(0, 8, 400)
    ref = np.c_[s, 0.06 * s ** 2.3]                 # the path, bending left
    arclen = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))]
    want = np.c_[np.interp(v * dt * np.arange(steps + 1), arclen, ref[:, 0]),
                 np.interp(v * dt * np.arange(steps + 1), arclen, ref[:, 1])]
    obstacle = np.array([3.0, 0.1])
    nominal = np.full(steps, 0.1)
    noise = np.cumsum(rng.normal(0, 0.025, (k, steps)), axis=1)
    samples = np.clip(nominal + noise, -0.5, 0.5)
    rolls = [bicycle_rollout(u, v, dt) for u in samples]
    costs = []
    for r in rolls:
        c = (np.linalg.norm(r - want, axis=1) ** 2).sum()
        dob = np.linalg.norm(r - obstacle, axis=1)
        c += 300 * np.clip(0.6 - dob, 0, None).sum()
        costs.append(c)
    costs = np.array(costs)
    lam = np.percentile(costs, 10) - costs.min() + 1e-6
    w = np.exp(-(costs - costs.min()) / lam)
    w /= w.sum()
    best = (w[:, None] * samples).sum(0)
    avg = bicycle_rollout(best, v, dt)
    fig, ax = plt.subplots(figsize=(11, 6))
    order = np.argsort(-costs)                      # costly first, so the cheap lie on top
    top = np.percentile(costs, 80)
    shade = np.clip(1 - (costs[order] - costs.min()) / (top - costs.min()), 0, 1)  # 1 = cheapest
    ax.add_collection(LineCollection([rolls[i] for i in order], colors=blues(shade, 0.12, 0.95),
                                     lw=1.0))
    ax.plot(*ref.T, color=INK, lw=2, ls='--', zorder=3)
    ax.add_patch(Circle(obstacle, 0.45, fc=WALL, ec='none', zorder=4))
    ax.plot(*avg.T, color=ORANGE, lw=4, zorder=5, solid_capstyle='round')
    car(ax, -0.5, 0, 0, 0.6, 0.35, color=INK, z=6)
    note(ax, 5.9, 3.0, 'the path\nto follow', INK, 12)
    note(ax, 2.55, -0.65, 'in the way', INK2, 11.5)
    note(ax, -0.9, -1.75, f'{k} sampled futures, 2.8 s each: darker = cheaper',
         BLUE, 12, fontweight='bold')
    note(ax, 0.0, 2.6, 'their weighted average:\nthe command sent', ORANGE, 12,
         fontweight='bold')
    ax.set_title('MPPI: try many futures, blend the good ones')
    clean(ax)
    ax.set_xlim(-1, 7.2)
    ax.set_ylim(-2.0, 4.0)
    save(fig, 'mppi.svg')


def fig_mpc():
    fig, ax = plt.subplots(figsize=(12, 5))
    t_past = np.linspace(-2, 0, 40)
    past = 0.35 * np.exp(t_past * 0.9) + 0.02 * np.sin(6 * t_past)
    t_ref = np.linspace(-2, 3.2, 200)
    ref = np.where(t_ref < 1.0, 0.0, 0.4 * (1 - np.exp(-(t_ref - 1.0) * 1.5)))
    n = 12
    t_h = np.linspace(0, 2.4, n + 1)
    pred = []
    y = past[-1]
    for tk in t_h:
        target = 0.0 if tk < 1.0 else 0.4 * (1 - np.exp(-(tk - 1.0) * 1.5))
        pred.append(y)
        y += 0.45 * (target - y + 0.06)
    pred = np.array(pred)
    ax.axvspan(0, 2.4, color=BLUE, alpha=0.07)
    ax.plot(t_ref, ref, color=INK, lw=2, ls='--', label='the path (where to be)')
    ax.plot(t_past, past, color=INK2, lw=2.5, label='what the car did')
    ax.plot(t_h, pred, color=BLUE, lw=2, marker='o', ms=6, label='the plan, step by step')
    ax.plot(t_h[:2], pred[:2], color=ORANGE, lw=5, solid_capstyle='round',
            label='only the first step is sent')
    ax.axvline(0, color=INK, lw=1)
    note(ax, 0.05, 0.62, 'now', INK, 12, fontweight='bold')
    note(ax, 0.6, 0.62, 'the horizon: the next 2.4 s', BLUE, 12, fontweight='bold')
    note(ax, 1.1, -0.2, 'next tick: slide the window on, plan again', INK2, 11.5)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('sideways offset (m)')
    ax.set_ylim(-0.3, 0.7)
    ax.set_xlim(-2, 3.2)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(loc='upper left', frameon=False, fontsize=11)
    ax.set_title('MPC: plan the next few seconds, send one step, repeat')
    save(fig, 'mpc.svg')


def fig_pure_pursuit():
    fig, ax = plt.subplots(figsize=(9.5, 7))
    s = np.linspace(-1, 9, 300)
    path = np.c_[s, 0.05 * (s + 1) ** 2]
    carp = np.array([0.0, -0.9])
    th = math.radians(5)
    ld = 3.2
    d = np.linalg.norm(path - carp, axis=1)
    ahead = path[(d > ld - 0.05) & (path[:, 0] > carp[0])][0]
    ax.plot(*path.T, color=INK, lw=2, ls='--')
    ax.add_patch(Circle(carp, ld, fill=False, ec=MUTED, lw=1.5, ls=':'))
    # The arc from the rear axle to the target.
    local = rot(-th) @ (ahead - carp)
    kappa = 2 * local[1] / ld ** 2
    pts, _ = arc(carp[0], carp[1], th, kappa, ld * 1.02, 60)
    ax.plot(*pts.T, color=BLUE, lw=3.5)
    car(ax, carp[0] - 0.25, carp[1], th, 1.4, 0.75, color=BLUE, alpha=0.9)
    ax.scatter(*carp, s=40, color=INK, zorder=7)
    ax.scatter(*ahead, s=150, color=ORANGE, zorder=7, ec='white', lw=1.5)
    ax.plot([carp[0], ahead[0]], [carp[1], ahead[1]], color=ORANGE, lw=1.5)
    hx = carp + 2.0 * np.array([math.cos(th), math.sin(th)])
    ax.plot([carp[0], hx[0]], [carp[1], hx[1]], color=INK2, lw=1.2)
    note(ax, ahead[0] + 0.2, ahead[1] - 0.5, 'the point ahead', ORANGE, 12, fontweight='bold')
    note(ax, 0.9, 0.55, 'lookahead\ndistance', ORANGE, 11.5)
    note(ax, 0.5, -1.55, 'α', INK, 15)
    note(ax, -1.8, -2.0, 'rear axle', INK, 11.5)
    note(ax, 2.6, -0.9, 'the arc it steers', BLUE, 12, fontweight='bold')
    note(ax, -3.4, -4.6, 'steering = atan(2 × wheelbase × sin α ÷ lookahead)', INK, 12.5)
    ax.set_title('Pure Pursuit: steer the arc to a point ahead')
    clean(ax)
    ax.set_xlim(-3.6, 7)
    ax.set_ylim(-5, 4.2)
    save(fig, 'pure-pursuit.svg')


def fig_costmap():
    h, w = 30, 40
    res = 0.1
    yy, xx = np.mgrid[0:h, 0:w]
    lethal = np.zeros((h, w), bool)
    lethal[10:16, 12:18] = True                      # a box
    lethal[22, 26:38] = True                         # a wall
    from_obs = np.full((h, w), 1e9)
    for y, x in zip(*np.nonzero(lethal)):
        from_obs = np.minimum(from_obs, np.hypot(xx - x, yy - y) * res)
    inscribed, scale = 0.35, 4.0
    cost = np.where(from_obs <= 0, 254,
                    np.where(from_obs <= inscribed, 253,
                             252 * np.exp(-scale * (from_obs - inscribed))))
    cost[cost < 1] = 0
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.2), gridspec_kw={'width_ratios': [1.15, 1]})
    img = np.ones((h, w, 4))
    shade = blues(cost / 253, 0.0, 0.85)
    img[:] = shade
    img[cost == 0] = (1, 1, 1, 1)
    img[cost == 253] = matplotlib.colors.to_rgba(ORANGE, 0.75)
    img[lethal] = matplotlib.colors.to_rgba(WALL)
    a1.imshow(img, origin='lower', interpolation='nearest')
    for gx in range(w + 1):
        a1.axvline(gx - 0.5, color='white', lw=0.4)
    for gy in range(h + 1):
        a1.axhline(gy - 0.5, color='white', lw=0.4)
    a1.set_title('A costmap: a grid, one cost per square')
    clean(a1)
    note(a1, 0, -3.0, 'black: lethal (something is there)   orange: the car would touch',
         size=10.5)
    note(a1, 0, -5.0, 'blue: inflation, a cost that fades with distance', size=10.5)
    d = np.linspace(0, 1.5, 300)
    c = np.where(d <= inscribed, 253, 252 * np.exp(-scale * (d - inscribed)))
    a2.plot(d, c, color=BLUE, lw=3)
    a2.axvspan(0, inscribed, color=ORANGE, alpha=0.15)
    note(a2, 0.03, 120, "the car's\nhalf-width:\nalways a hit", ORANGE, 11.5)
    note(a2, 0.6, 140, 'inflation: "close is risky,\nfar is fine"', BLUE, 12)
    a2.set_xlabel('distance from the obstacle (m)')
    a2.set_ylabel('cost (0 free, 254 lethal)')
    a2.set_ylim(0, 270)
    for s in ('top', 'right'):
        a2.spines[s].set_visible(False)
    a2.set_title('How the cost falls off')
    save(fig, 'costmap.svg')


def fig_multi_drive():
    rng = np.random.default_rng(9)

    def street(rng):
        parts = [segment_points((0, 0), (20, 0), 0.15, 0.04, rng),       # kerb
                 segment_points((0, 6), (20, 6), 0.15, 0.04, rng),       # kerb
                 segment_points((2, 7.5), (9, 7.5), 0.15, 0.04, rng),    # a building
                 segment_points((12, 7.5), (19, 7.5), 0.15, 0.04, rng),
                 np.c_[15 + 0.3 * np.cos(np.linspace(0, 6.3, 30)),
                       -1.0 + 0.3 * np.sin(np.linspace(0, 6.3, 30))]]    # a tree
        return np.vstack(parts)

    def box(x, y, w, h, rng):
        return np.vstack([segment_points((x, y), (x + w, y), 0.12, 0.03, rng),
                          segment_points((x, y + h), (x + w, y + h), 0.12, 0.03, rng),
                          segment_points((x, y), (x, y + h), 0.12, 0.03, rng),
                          segment_points((x + w, y), (x + w, y + h), 0.12, 0.03, rng)])

    a_static, b_static = street(rng), street(rng)
    parked = box(5, 0.4, 4.2, 1.8, rng)
    walker = np.c_[13 + rng.normal(0, 0.12, 40), 3.5 + rng.normal(0, 0.12, 40)]
    drive_a = np.vstack([a_static, parked])
    drive_b = np.vstack([b_static, walker])
    d_ab = np.min(np.linalg.norm(drive_a[:, None] - drive_b[None], axis=2), axis=1)
    kept = drive_a[d_ab < 0.25]
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.0))
    titles = ['Drive 1: a car parked by the kerb', 'Drive 2: the car gone, a person walking',
              'Merged: keep only what both drives saw']
    for ax, title, (pts, moving) in zip(axes, titles, ((a_static, parked), (b_static, walker),
                                                       (kept, None))):
        ax.scatter(*pts.T, s=5, color=INK2)
        if moving is not None:
            ax.scatter(*moving.T, s=9, color=ORANGE)
        ax.set_title(title, fontsize=13.5)
        clean(ax)
        ax.set_xlim(-0.5, 20.5)
        ax.set_ylim(-2, 8.5)
    note(axes[0], 4.7, 2.6, 'parked car', ORANGE, 12, fontweight='bold')
    note(axes[1], 11.6, 4.2, 'a person', ORANGE, 12, fontweight='bold')
    note(axes[2], 3.0, 2.6, 'gone: only one drive saw them', AQUA, 12, fontweight='bold')
    save(fig, 'multi-drive.svg')


# --------------------------------------------------------------------------- behaviour

def _node(ax, x, y, w, h, title, sub='', color=BLUE, fill='white', size=12.5, sub_size=10.5):
    """Draw a rounded box with a title and an optional second line, centred on (x, y)."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle='round,pad=0.02,rounding_size=0.12',
                                fc=fill, ec=color, lw=2.5, zorder=3))
    dy = 0.17 * h if sub else 0.0
    ax.text(x, y + dy, title, ha='center', va='center', fontsize=size, fontweight='bold',
            zorder=4)
    if sub:
        ax.text(x, y - 0.22 * h, sub, ha='center', va='center', fontsize=sub_size, color=INK2,
                zorder=4)


def _arrow(ax, a, b, color=INK2, rad=0.0, lw=1.8):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=16, color=color, lw=lw,
                                 connectionstyle=f'arc3,rad={rad}', zorder=2,
                                 shrinkA=2, shrinkB=2))


def fig_behaviour_place():
    fig, ax = plt.subplots(figsize=(14, 6.4))
    clean(ax, equal=False)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 6.4)

    ax.text(0.2, 6.05, '1. In most stacks: behaviour sits between the route and the motion '
            'planner', fontsize=14.5, fontweight='bold')
    general = [('Route', 'which streets?', INK2),
               ('Behaviour', 'what to do now?\nfollow, slow, stop,\nyield, wait, go', ORANGE),
               ('Motion planner', 'the line and the speed\nfor the next seconds', INK2),
               ('Control', 'steer, throttle,\nbrake', INK2)]
    w, gap, y = 2.9, 0.55, 4.25
    for i, (t, sub, c) in enumerate(general):
        x = 0.25 + w / 2 + i * (w + gap)
        _node(ax, x, y, w, 1.5, t, sub, c)
        if i:
            _arrow(ax, (x - w / 2 - gap + 0.05, y), (x - w / 2 - 0.05, y))
    bx = 0.25 + w / 2 + (w + gap)
    ax.text(bx, 5.25, 'sees: people, cars, signs, lights, stop lines', ha='center',
            fontsize=10.5, color=ORANGE)

    ax.text(0.2, 2.75, '2. In our car: behaviour reads the plan and writes the road ahead',
            fontsize=14.5, fontweight='bold')
    ours = [('Route', 'aav_routes\n(streets, lanes)', INK2),
            ('Path', 'aav_planner\n/plan', INK2),
            ('Behaviour', 'aav_behaviour_rs\nstates + stopping law', ORANGE),
            ('The road ahead', '/world/road_ahead\nits stops, its limits', AQUA),
            ('ego', 'holds every follower\n(MPPI...) under it', BLUE)]
    w2, gap2, y2 = 2.35, 0.42, 1.25
    for i, (t, sub, c) in enumerate(ours):
        x = 0.25 + w2 / 2 + i * (w2 + gap2)
        _node(ax, x, y2, w2, 1.45, t, sub, c, size=12, sub_size=10)
        if i:
            _arrow(ax, (x - w2 / 2 - gap2 + 0.05, y2), (x - w2 / 2 - 0.05, y2))
    bx2 = 0.25 + w2 / 2 + 2 * (w2 + gap2)
    ax.text(bx2, 2.18, 'sees: the things, a stop sign, stop lines, the locks, the fence',
            ha='center', fontsize=10.5, color=ORANGE)
    ax.text(bx2, 0.2, 'it caps the speed; it never steers', ha='center', fontsize=11,
            color=ORANGE, fontweight='bold')
    save(fig, 'behaviour-place.svg')


def fig_behaviour_states():
    fig, ax = plt.subplots(figsize=(14, 8.2))
    clean(ax, equal=False)
    ax.set_xlim(-0.6, 14.4)
    ax.set_ylim(-2.9, 5.6)
    w, h = 2.9, 1.3
    pos = {'DRIVE': (1.2, 4.0), 'APPROACH': (6.3, 4.0), 'STOPPED': (11.4, 4.0),
           'GO': (1.2, 0.6), 'WAIT': (11.4, 0.6), 'ASK': (6.3, -1.9)}
    caps = {'DRIVE': ('no cap', AQUA), 'APPROACH': ('cap = v_cap(left)', ORANGE),
            'STOPPED': ('the floor, 0.3 m/s', BLUE), 'WAIT': ('the floor, 0.3 m/s', BLUE),
            'GO': ('rises 0.5 m/s²', AQUA), 'ASK': ('a person is told', INK2)}
    for name, (x, y) in pos.items():
        sub, c = caps[name]
        _node(ax, x, y, w, h, name, sub, c, size=14, sub_size=11)

    def edge(a, b, label, lx, ly, rad=0.0):
        (x0, y0), (x1, y1) = pos[a], pos[b]
        dx, dy = x1 - x0, y1 - y0
        # Leave and arrive at the boxes' edges.
        k = max(abs(dx) / (w / 2 + 0.1), abs(dy) / (h / 2 + 0.1))
        _arrow(ax, (x0 + dx / k, y0 + dy / k), (x1 - dx / k, y1 - dy / k), INK2, rad)
        ax.text(lx, ly, label, ha='center', va='center', fontsize=10.5, color=INK2,
                bbox={'fc': 'white', 'ec': 'none', 'pad': 1.0}, zorder=5)

    edge('DRIVE', 'APPROACH', 'a stop binds\n(sign, obstacle, fence,\ngoal, stop line)',
         3.75, 5.15, rad=-0.3)
    edge('APPROACH', 'DRIVE', 'sign passed or lost;\nthe goal ended', 3.75, 3.1, rad=-0.3)
    edge('APPROACH', 'STOPPED', 'its lock on,\nstanding still', 8.85, 4.75)
    edge('STOPPED', 'WAIT', 'next tick', 12.35, 2.3)
    edge('WAIT', 'GO', 'no lock, nothing moving near', 8.6, 0.95)
    edge('GO', 'DRIVE', 'cap back up\nto the top speed', 0.0, 2.3)
    edge('APPROACH', 'GO', 'what it slowed for\nhas gone (an obstacle:\ngone for 1 s)',
         5.3, 1.95, rad=0.2)
    edge('GO', 'APPROACH', 'a new stop\nbinds', 2.5, 2.3, rad=0.2)
    edge('WAIT', 'ASK', "fence NEAR, or a lock\n(not the sign's) over 30 s", 10.7, -1.15)
    edge('ASK', 'GO', 'no lock', 2.8, -1.0)

    ax.text(-0.5, -2.75,
            'From any state:  the e-stop → WAIT  ·  the goal aborted → ASK  ·  a lock on and '
            'standing → WAIT  ·  something moving near → WAIT (moving), until gone 1 s',
            fontsize=10.5, color=INK2)
    ax.set_title('Our behaviour planner: six states (aav_behaviour_rs, the same as '
                 'behaviour.py)', loc='left')
    save(fig, 'behaviour-states.svg')


def behaviour_cap(t, binding, hold, base=1.3, low=1.0, fall=2.0, rise=0.5):
    """
    Return the cap over time, and the times DRIVE came back, for a flickering obstacle.

    A sketch: our rules, made-up speeds. APPROACH while the obstacle binds (the cap falls to
    `low` at `fall` m/s²); when it goes, GO (rises at `rise` m/s²), then DRIVE at `base`.
    With `hold` s, APPROACH lets go only once it has stayed gone that long (TODO 189's
    release_hold).
    """
    dt = t[1] - t[0]
    state, cap, gone, drives = 'DRIVE', base, None, 0
    caps = []
    for now, b in zip(t, binding):
        if b:
            gone = None
            state = 'APPROACH'
            cap = max(low, cap - fall * dt)
        elif state == 'APPROACH':
            gone = now if gone is None else gone
            if now - gone >= hold - 1e-9:
                state, gone = 'GO', None
        elif state == 'GO':
            cap = min(base, cap + rise * dt)
            if cap >= base:
                state = 'DRIVE'
                drives += 1
        caps.append(cap)
    return np.array(caps), drives


def fig_behaviour_hysteresis():
    t = np.arange(0.0, 12.0, 0.02)
    # A person at the lane's edge: in it, out of it, in again (made up).
    spells = [(1.0, 2.2), (3.0, 3.6), (4.4, 5.3), (6.1, 7.0)]
    binding = np.zeros_like(t, dtype=bool)
    for a, b in spells:
        binding |= (t >= a) & (t < b)
    no_hold, drives_no = behaviour_cap(t, binding, hold=0.0)
    held, drives_held = behaviour_cap(t, binding, hold=1.0)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6.6), sharex=True,
                                 gridspec_kw={'height_ratios': [3, 1]})
    a1.plot(t, no_hold, color=MUTED, lw=2.5,
            label=f'no hold: APPROACH > GO > DRIVE {drives_no} times')
    a1.plot(t, held, color=BLUE, lw=3,
            label=f'release_hold 1 s: {drives_held} time, once it has really gone')
    a1.set_ylabel('speed cap')
    a1.set_ylim(0.85, 1.42)
    a1.set_yticks([])
    a1.legend(loc='lower right', frameon=False, fontsize=11)
    note(a1, 0.1, 1.34, 'DRIVE: no cap', INK2, 11)
    note(a1, 0.1, 0.95, 'APPROACH: the cap for the stop', ORANGE, 11)
    for s in ('top', 'right'):
        a1.spines[s].set_visible(False)
    a1.set_title("Hysteresis: one person at the lane's edge, with and without the hold "
                 '(a sketch)')

    a2.fill_between(t, 0, binding.astype(float), step='post', color=ORANGE, alpha=0.8, lw=0)
    a2.set_yticks([0, 1])
    a2.set_yticklabels(['clear', 'in the lane'])
    a2.set_ylim(-0.1, 1.2)
    a2.set_xlabel('time (s)')
    for s in ('top', 'right'):
        a2.spines[s].set_visible(False)
    save(fig, 'behaviour-hysteresis.svg')


def fig_behaviour_tree():
    fig, ax = plt.subplots(figsize=(15, 7.6))
    clean(ax, equal=False)
    ax.set_xlim(-0.8, 16.6)
    ax.set_ylim(-0.9, 7.2)

    def ctrl(x, y, sym, label, w=3.0):
        _node(ax, x, y, w, 0.95, f'{sym}  {label}', '', INK2, fill='#f4f3f0', size=11.5)

    def act(x, y, label, sub='', w=2.6, color=BLUE):
        _node(ax, x, y, w, 1.05 if sub else 0.8, label, sub, color, size=11.5, sub_size=9.5)

    def link(a, b):
        ax.plot([a[0], b[0]], [a[1] - 0.48, b[1] + 0.5], color=MUTED, lw=1.6, zorder=1)

    root = (8.1, 6.4)
    ctrl(*root, '↻', 'Recovery: 6 tries', w=3.4)
    pipe, rr = (4.0, 4.9), (12.6, 4.9)
    ctrl(*pipe, '⇉', 'Pipeline: plan and follow', w=3.8)
    ctrl(*rr, '⟳', 'Round robin: the next fix', w=3.8)
    link(root, pipe)
    link(root, rr)
    rate, follow = (2.1, 3.4), (6.7, 3.4)
    ctrl(*rate, '1 Hz', 'Rate: at most once a second', w=4.3)
    act(*follow, 'FollowPath', 'the controller (MPPI)')
    link(pipe, rate)
    link(pipe, follow)
    fallback = (2.0, 1.95)
    ctrl(*fallback, '?', 'Fallback: try in turn', w=3.2)
    link(rate, fallback)
    keep, plan = (0.75, 0.25), (3.95, 0.25)
    act(*keep, 'Keep the path?', 'valid, ends at the goal,\nunder 10 s old', w=2.5,
        color=AQUA)
    act(*plan, 'ComputePathToPose', 'the planner', w=3.3)
    link(fallback, keep)
    link(fallback, plan)
    clear, wait, back = (10.4, 3.4), (12.9, 3.4), (15.2, 3.4)
    act(*clear, 'Clear costmaps', 'both', w=2.3)
    act(*wait, 'Wait', '5 s', w=1.9)
    act(*back, 'BackUp', '1 m at 0.25 m/s', w=2.3)
    for k in (clear, wait, back):
        link(rr, k)
    note(ax, 10.0, 2.35, "no Spin: a car can't turn on the spot", INK2, 11)

    legend = [('⇉', 'do the children in order, together'), ('?', 'try each until one works'),
              ('↻', 'if it fails, run a fix and try again'),
              ('green', 'a condition: a yes-or-no check'), ('blue', 'an action: calls a server')]
    for i, (sym, what) in enumerate(legend):
        y = 1.4 - i * 0.45
        ax.text(8.6, y, sym, fontsize=11, fontweight='bold',
                color=AQUA if sym == 'green' else BLUE if sym == 'blue' else INK)
        ax.text(9.8, y, what, fontsize=11, color=INK2)
    ax.set_title("A behaviour tree: Nav2's navigate-to-pose tree, as we ran it for a car "
                 '(a sketch; retired with Nav2)', loc='left')
    save(fig, 'behaviour-tree.svg')


def main():
    fig_pipeline()
    fig_kalman()
    fig_icp()
    fig_ndt()
    fig_ndt_wrong()
    fig_amcl()
    fig_slam()
    fig_astar()
    fig_hybrid()
    fig_lattice()
    fig_rrt()
    fig_mppi()
    fig_mpc()
    fig_pure_pursuit()
    fig_costmap()
    fig_multi_drive()
    fig_behaviour_place()
    fig_behaviour_states()
    fig_behaviour_hysteresis()
    fig_behaviour_tree()


if __name__ == '__main__':
    main()
