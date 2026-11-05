"""
The guard: the simple part that checks what the clever part drew. No ROS.

Every sample path must pass every check (docs/designs/2026-11-02_diffusion-planner.md,
"The guard"); the cheapest that passes is the plan. None passes: the lattice's path.

  G1 numbers    every value finite; no jumps (a step over STEP_RATIO x the mean step)
  G2 start      the first point within start_xy m and start_yaw rad of the car
  G3 end        the last point within the goal checker's tolerance
  G4 collisions the car's outline (padded as the costmap's), at every point, on no lethal
                or unknown cell: Smac's own rule for a car-shaped footprint (the design
                asked for no inscribed cell either; the lattice's own paths fail that)
  G5 turns      between cusps, every turn (a circle through points 1 m either side) at
                least min_radius: by default 0.95 x full lock (see the design: the
                teachers' own smoothed paths measure 2.8-3.2 m at their tightest)
  G6 heading    each point's heading follows its travel, forward or backward: no sliding
  G7 reversing  none at all unless reversing; with it, at most max_cusps cusps and
                max_reverse m backwards
  G8 length     no longer than length_ratio x the lattice's path (when it has one), nor
                MAX_LENGTH. A sample failing G1 or G8 is not checked further (G2-G3 are)

Paths here are (N, 3) x, y, yaw in the map frame; the costmap is window.Grid.
"""

from collections import Counter
from dataclasses import dataclass
import math

import numpy as np

from .window import LETHAL, UNKNOWN

CHECKS = ('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7', 'G8')
STEP_RATIO = 3.0        # a step longer than this x the mean step is a jump (G1)
DENSE_STEP = 0.25       # m between the points the checks look at (the map's cells)
CHORD = 4               # dense points either side for a turn's radius: about 1 m (G5)
MIN_TRAVEL = 0.02       # m: a step shorter than this has no direction (G6)
MAX_LENGTH = 300.0      # m: no plan in a lot is longer; a sample that is fails G8
MIN_RUN = 0.5           # m: a run of one gear shorter than this is noise, not a cusp (G7)


@dataclass(frozen=True)
class Limits:
    """The guard's numbers; the node fills them from vehicle.yaml and nav2.yaml."""

    min_radius: float                   # m: no turn tighter (G5)
    reversing: bool = False
    start_xy: float = 0.3
    start_yaw: float = math.radians(15.0)
    goal_xy: float = 1.0
    goal_yaw: float = 0.75
    heading_slip: float = math.radians(25.0)
    max_cusps: int = 4
    max_reverse: float = 15.0
    length_ratio: float = 1.5


def footprint_points(polygon, spacing=0.1):
    """
    Return points (M, 2) covering a footprint polygon (base_link, m): its edge and inside.

    polygon: [(x, y), ...] corners in order. The inside is a grid `spacing` apart, kept
    where it falls inside (an even-odd test); the edge is sampled at the same spacing.
    """
    poly = np.asarray(polygon, dtype=np.float64)
    edge = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = max(1, int(math.ceil(np.hypot(*(b - a)) / spacing)))
        t = np.arange(n)[:, None] / n
        edge.append(a + t * (b - a))
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    xs = np.arange(lo[0] + spacing / 2, hi[0], spacing)
    ys = np.arange(lo[1] + spacing / 2, hi[1], spacing)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
    inside = np.zeros(len(pts), dtype=bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        crosses = (a[1] > pts[:, 1]) != (b[1] > pts[:, 1])
        with np.errstate(divide='ignore', invalid='ignore'):
            x_at = a[0] + (pts[:, 1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        inside ^= crosses & (pts[:, 0] < x_at)
    return np.concatenate(edge + [pts[inside]], axis=0)


def length(path):
    """Return how long a path is (m)."""
    path = np.asarray(path, dtype=np.float64)
    if len(path) < 2:
        return 0.0
    return float(np.hypot(*np.diff(path[:, :2], axis=0).T).sum())


def densify(path, step=DENSE_STEP):
    """Return the path with points at most `step` m apart (headings the short way round)."""
    path = np.asarray(path, dtype=np.float64)
    if len(path) < 2:
        return path.copy()
    out = [path[:1]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        t = (np.arange(1, n + 1) / n)[:, None]
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        seg = np.concatenate([a[:2] + t * (b[:2] - a[:2]), a[2] + t * dyaw], axis=1)
        out.append(seg)
    dense = np.concatenate(out, axis=0)
    dense[:, 2] = np.arctan2(np.sin(dense[:, 2]), np.cos(dense[:, 2]))
    return dense


def travel_gears(path):
    """
    Return (gear of each step: +1/-1/0, step lengths) from heading against travel.

    A step shorter than MIN_TRAVEL has gear 0: it says nothing.
    """
    d = np.diff(path[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    along = d[:, 0] * np.cos(path[:-1, 2]) + d[:, 1] * np.sin(path[:-1, 2])
    gear = np.where(seg < MIN_TRAVEL, 0.0, np.sign(along))
    return gear, seg


def segments(gear, seg=None):
    """
    Return [(first step, last step + 1, gear)]: runs of one gear (0s join the run).

    seg: the steps' lengths; with it, a run shorter than MIN_RUN joins the run before it
    (or after it, the first): a heading's wobble at a path's end is not a cusp.
    """
    runs = _runs(gear)
    if seg is None:
        return runs
    while len(runs) > 1:
        short = [i for i, (a, b, _) in enumerate(runs) if seg[a:b].sum() < MIN_RUN]
        if not short:
            break
        i = short[0]
        a, b, _ = runs[i]
        if i == 0:
            runs[1] = (a, runs[1][1], runs[1][2])
        else:
            runs[i - 1] = (runs[i - 1][0], b, runs[i - 1][2])
        del runs[i]
        merged = [runs[0]]
        for r in runs[1:]:
            if r[2] == merged[-1][2]:
                merged[-1] = (merged[-1][0], r[1], r[2])
            else:
                merged.append(r)
        runs = merged
    return runs


def _runs(gear):
    runs, start, cur = [], 0, 0.0
    for i, g in enumerate(gear):
        if g == 0.0:
            continue
        if cur == 0.0:
            cur = g
        elif g != cur:
            runs.append((start, i, cur))
            start, cur = i, g
    runs.append((start, len(gear), cur if cur != 0.0 else 1.0))
    return runs


def radii(points):
    """Return the radius (m) of the circle through each point and CHORD points either side."""
    if len(points) < 2 * CHORD + 1:
        return np.full(0, np.inf)
    a, b, c = points[:-2 * CHORD], points[CHORD:-CHORD], points[2 * CHORD:]
    ab, bc, ca = (np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(a - c).T))
    cross = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) -
                   (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    with np.errstate(divide='ignore', invalid='ignore'):
        r = ab * bc * ca / (2.0 * cross)
    return np.where(cross < 1e-9, np.inf, r)


def angle_diff(a, b):
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def check(path, grid, start, goal, limits, footprint, lattice_length=None):
    """
    Return (the checks it failed, in order; facts about it) for one sample path.

    path: (N, 3) map; grid: window.Grid; start, goal: (x, y, yaw); footprint:
    footprint_points(). facts: length, cusps, reverse_m, cost_mean, min_radius, dense
    (the checked path, ready for Nav2), when the numbers allowed working them out.
    """
    path = np.asarray(path, dtype=np.float64)
    failed, facts = [], {}
    # G1: numbers. Nothing else can be judged on NaNs.
    if len(path) < 2 or not np.all(np.isfinite(path)):
        return ['G1'], facts
    steps = np.hypot(*np.diff(path[:, :2], axis=0).T)
    if steps.mean() <= 1e-6 or steps.max() > STEP_RATIO * steps.mean():
        failed.append('G1')
    # G2, G3: the ends.
    if (math.hypot(path[0, 0] - start[0], path[0, 1] - start[1]) > limits.start_xy or
            angle_diff(path[0, 2], start[2]) > limits.start_yaw):
        failed.append('G2')
    if (math.hypot(path[-1, 0] - goal[0], path[-1, 1] - goal[1]) > limits.goal_xy or
            angle_diff(path[-1, 2], goal[2]) > limits.goal_yaw):
        failed.append('G3')
    # G8 before the slow checks: a long, wild sample (G1 or G8) isn't checked further, as
    # its thousands of dense points cost the plan its time budget (2 Nov: 0.8 s).
    raw = float(steps.sum())
    if (lattice_length and raw > limits.length_ratio * lattice_length) or raw > MAX_LENGTH:
        failed.append('G8')
    if 'G1' in failed or 'G8' in failed:
        facts['length'] = raw
        return failed, facts
    dense = densify(path)
    facts['dense'] = dense
    # G4: the outline on no lethal or unknown cell.
    c, s = np.cos(dense[:, 2])[:, None], np.sin(dense[:, 2])[:, None]
    fx = dense[:, :1] + c * footprint[None, :, 0] - s * footprint[None, :, 1]
    fy = dense[:, 1:2] + s * footprint[None, :, 0] + c * footprint[None, :, 1]
    under = grid.lookup(fx, fy)
    centre = grid.lookup(dense[:, 0], dense[:, 1])
    if np.any(under >= LETHAL) or np.any(under == UNKNOWN):
        failed.append('G4')
    facts['cost_mean'] = float(np.clip(centre, 0, LETHAL).mean())
    # G5, G6, G7: gears from heading against travel, then turns within each gear.
    gear, seg = travel_gears(dense)
    runs = segments(gear, seg)
    cusps = len(runs) - 1
    reverse_m = float(sum(seg[a:b].sum() for a, b, g in runs if g < 0))
    facts.update(length=float(seg.sum()), cusps=cusps, reverse_m=reverse_m)
    tightest = math.inf
    for a, b, _ in runs:
        r = radii(dense[a:b + 1, :2])
        if len(r):
            tightest = min(tightest, float(r.min()))
    facts['min_radius'] = tightest
    if tightest < limits.min_radius:
        failed.append('G5')
    # Each step's gear is its run's: a short wobble isn't sliding sideways either.
    for a, b, g in runs:
        gear[a:b] = np.where(gear[a:b] != 0.0, g, 0.0)
    moving = gear != 0.0
    if np.any(moving):
        d = np.diff(dense[:, :2], axis=0)[moving]
        travel = np.arctan2(d[:, 1], d[:, 0])
        facing = np.where(gear[moving] > 0, dense[:-1, 2][moving],
                          dense[:-1, 2][moving] + math.pi)
        if np.any(angle_diff(travel, facing) > limits.heading_slip):
            failed.append('G6')
    if (reverse_m > 0.0 and not limits.reversing) or cusps > limits.max_cusps or (
            reverse_m > limits.max_reverse):
        failed.append('G7')
    return failed, facts


def cost(facts, cost_weight=0.05, cusp_price=5.0, reverse_price=1.0):
    """Return a passed path's price: its length, the costmap along it, its cusps, reversing."""
    return (facts['length'] * (1.0 + cost_weight * facts['cost_mean'] / 10.0) +
            cusp_price * facts['cusps'] + reverse_price * facts['reverse_m'])


def choose(paths, grid, start, goal, limits, footprint, lattice_length=None):
    """
    Return (the cheapest path that passed or None, its facts, refusals by check, passed).

    refusals: a Counter of check -> how many samples failed it (a sample may fail several);
    passed: how many samples passed every check.
    """
    best, best_facts, best_price = None, None, math.inf
    refusals, passed = Counter(), 0
    for i, path in enumerate(paths):
        failed, facts = check(path, grid, start, goal, limits, footprint, lattice_length)
        refusals.update(failed)
        if failed:
            continue
        passed += 1
        price = cost(facts)
        if price < best_price:
            best, best_facts, best_price = i, facts, price
    return best, best_facts, refusals, passed


def gap(a, b):
    """Return how far path a strays from path b: the mean distance to b's nearest point (m)."""
    a, b = np.asarray(a)[:, :2], np.asarray(b)[:, :2]
    if not len(a) or not len(b):
        return math.nan
    d = np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])
    return float(d.min(axis=1).mean())


def decide(model_ok, picked, elapsed_ms, budget_ms, lattice_ok):
    """
    Return (source, why): whose path the car gets, 'model', 'lattice' or 'none'.

    model_ok: the model answered; picked: a sample passed every check; elapsed_ms: the
    model and the guard, from the request; lattice_ok: the lattice found a path.
    The model's path only if it answered, passed, and in time; else the lattice's; and
    if the lattice found none either, none (Nav2's NO_VALID_PATH).
    """
    if not model_ok:
        why = 'no model answer'
    elif not picked:
        why = 'every sample failed the guard'
    elif elapsed_ms > budget_ms:
        why = f'late: {elapsed_ms:.0f} ms, over {budget_ms:.0f}'
    else:
        return 'model', 'a sample passed every check'
    return ('lattice', why) if lattice_ok else ('none', why + '; the lattice found none')
