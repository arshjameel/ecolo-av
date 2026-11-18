#!/usr/bin/env python3
"""
Make the State Lattice's smooth moves: curvature that changes no faster than the steering can.

Run it with pixi run python3 tools/lattice_smooth.py [option] (no pixi task):

    (no option)       writes both files below, and their pictures
    --check           are the committed files still right for the car?

A trial beside the moves Nav2's generator makes (tools/lattice_primitives.py),
picked with lattice_moves:=:

    smooth            config/lattice_ackermann_smooth.json, for 1 m/s (ego's max_speed)
    smooth_fast       config/lattice_ackermann_smooth_fast.json, for 2.5 m/s (the campus)

Why. Nav2's generator draws a move as a straight line and a circle: the curvature jumps
from 0 to 1/R where they meet, and from left to right where two moves meet. The steering
can't jump: ego turns it at most max_steering_rate (0.6 rad/s), the motor at
actuators.steer_rate (0.8 rad/s), so the follower cuts the corner or wiggles.

The rule. Curvature k = tan(steer) / wheelbase, so along the path

    dk/ds = steer_rate / (wheelbase * v * cos(steer)^2)  >=  steer_rate / (wheelbase * v)

and the moves keep dk/ds at most rate / (wheelbase * speed * RATE_MARGIN) (sharpness()).
At 0.6 rad/s (ego's ramp, tighter than the motor's 0.8) and 1.561 m: 0.384 1/m^2 at
1 m/s and 0.154 at 2.5 m/s, then 1.25 times less (0.307, 0.123). A move smooth enough
at a speed is smooth enough at any slower one. A full-lock turn (1/3.572 m) then takes
0.9 m to wind on at 1 m/s and 2.3 m at 2.5 m/s, where Nav2's moves take none.

How. Every move starts and ends straight (curvature 0), so any two moves join with no
jump. Between, a turn is a clothoid (curvature rising in a straight line), an arc at most
1/plan_turning_radius, and a clothoid back down; a straight before and after puts its end
exactly on a map cell and on one of the lattice's 16 headings. Of every such move, the
shortest is kept. Nav2's lattice and ours read the file unchanged (Nav2's JSON format).

Pure Python and numpy, no ROS: tested in tools/tests/test_lattice_smooth.py.
"""

import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros', 'bringup', 'aav_description'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aav_description import vehicle  # noqa: E402
import lattice_primitives as lp  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

EGO = os.path.join(ROOT, 'ros', 'ego', 'aav_ego', 'config', 'ego.yaml')
# Each file's design speed (m/s): smooth for ego's max_speed (1.0, the lot and Nav2's
# default), smooth_fast for the campus tours' (controller_compare --tours campus --speed
# 2.5). Slower is smoother still.
SPEEDS = {'smooth': 1.0, 'smooth_fast': 2.5}
DESIGN_SPEED = SPEEDS['smooth']
# The moves use at most 1 / RATE_MARGIN of the steering's rate, as PLAN_TURN_MARGIN leaves
# steering to spare: the follower needs some to correct, and the linkage lags (steer_lag).
RATE_MARGIN = 1.25
# The 16 headings: the directions of these cell steps (Nav2's generator's, exactly).
STEPS = ((1, 0), (2, 1), (1, 1), (1, 2), (0, 1), (-1, 2), (-1, 1), (-2, 1),
         (-1, 0), (-2, -1), (-1, -1), (-1, -2), (0, -1), (1, -2), (1, -1), (2, -1))
# The turns from each heading: this many headings left and right.
TURNS = (1, 2)
# A straight move: the shortest whole number of cell steps at least this long (m), as
# Nav2's (1.25 m east).
STRAIGHT = 1.25
# Ends are looked for in cells this far from the start (m).
REACH = 8.0
# Integration step along a move (m).
DS = 0.002
# How many clothoid shapes are tried per turn.
SHAPES = 60


def headings():
    """Return the 16 heading angles (rad, 0..2 pi), in Nav2's order."""
    return [math.atan2(y, x) % math.tau for x, y in STEPS]


def steering_rate(cfg, ego):
    """Return the steering's top rate (rad/s): the tighter of the motor's and ego's ramp."""
    rate = float(cfg['actuators']['steer_rate'])
    if ego and 'max_steering_rate' in ego:
        rate = min(rate, float(ego['max_steering_rate']))
    return rate


def sharpness(rate, wheelbase, speed=DESIGN_SPEED, margin=RATE_MARGIN):
    """Return the most curvature may change per metre (1/m^2) at `speed` (the rule above)."""
    return rate / (wheelbase * speed * margin)


def turn_profile(turn, peak, sharp):
    """
    Return a turn's pieces [(length, k_start, k_end)]: a clothoid up, an arc, one down.

    turn: the heading change (rad, + left); peak: the arc's curvature (1/m, > 0);
    sharp: the clothoids' dk/ds. The heading changes peak^2 / sharp on the two clothoids
    and the rest on the arc, so peak^2 <= sharp * |turn| (else ValueError).
    """
    sign = 1.0 if turn > 0 else -1.0
    ramp = peak / sharp
    arc = (abs(turn) - peak * ramp) / peak
    if arc < -1e-9:
        raise ValueError(f'peak {peak} at sharpness {sharp} turns more than {turn}')
    arc = max(arc, 0.0)
    k = sign * peak
    return [(ramp, 0.0, k), (arc, k, k), (ramp, k, 0.0)]


def trace(pieces, ds=DS):
    """
    Return (s, x, y, yaw) arrays along `pieces` from (0, 0, 0), every ~ds m.

    The yaw is exact (curvature is linear in s on each piece); x and y by the midpoint rule.
    """
    s_out, yaw_out, dx, dy = [np.zeros(1)], [np.zeros(1)], [], []
    s0, yaw0 = 0.0, 0.0
    for length, k0, k1 in pieces:
        if length <= 0.0:
            continue
        rate = (k1 - k0) / length
        local = np.linspace(0.0, length, max(1, math.ceil(length / ds)) + 1)
        mid = 0.5 * (local[:-1] + local[1:])
        yaw_mid = yaw0 + k0 * mid + 0.5 * rate * mid ** 2
        step = np.diff(local)
        dx.append(step * np.cos(yaw_mid))
        dy.append(step * np.sin(yaw_mid))
        s_out.append(s0 + local[1:])
        yaw_out.append(yaw0 + k0 * local[1:] + 0.5 * rate * local[1:] ** 2)
        yaw0 += k0 * length + 0.5 * rate * length ** 2
        s0 += length
    s = np.concatenate(s_out)
    yaw = np.concatenate(yaw_out)
    x = np.concatenate([[0.0], np.cumsum(np.concatenate(dx))]) if dx else np.zeros(1)
    y = np.concatenate([[0.0], np.cumsum(np.concatenate(dy))]) if dy else np.zeros(1)
    return s, x, y, yaw


def shapes(turn, sharp, k_max, count=SHAPES):
    """
    Return the turn's candidate (peak, sharpness) pairs: none sharper, none tighter.

    At the full sharpness, every peak up to the tightest; and pure clothoid pairs (no arc)
    at gentler sharpness, which make the wider turns.
    """
    top = min(k_max, math.sqrt(sharp * abs(turn)))
    out = [(top * (i + 1) / count, sharp) for i in range(count)]
    for i in range(1, count):
        gentle = sharp * i / count
        peak = math.sqrt(gentle * abs(turn))
        if peak <= k_max:
            out.append((peak, gentle))
    return out


def cells(resolution, reach=REACH):
    """Return every cell offset (x, y) in m within `reach` of the start, as an (n, 2) array."""
    n = int(reach / resolution)
    i, j = np.meshgrid(np.arange(-n, n + 1), np.arange(-n, n + 1))
    xy = np.stack([i.ravel(), j.ravel()], axis=1) * resolution
    return xy[np.hypot(xy[:, 0], xy[:, 1]) <= reach]


def best_turn(start, end, sharp, k_max, resolution):
    """
    Return the shortest smooth move from heading angle `start` to `end` that ends on a cell.

    A dict: pieces (with the straights), length, the end cell (x, y), peak curvature.
    None if no shape reaches a cell (REACH too small).
    """
    turn = math.remainder(end - start, math.tau)
    u_s = np.array([math.cos(start), math.sin(start)])
    u_e = np.array([math.cos(end), math.sin(end)])
    det = u_s[0] * u_e[1] - u_s[1] * u_e[0]
    targets = cells(resolution)
    best = None
    for peak, sharp_i in shapes(turn, sharp, k_max):
        pieces = turn_profile(turn, peak, sharp_i)
        _, x, y, _ = trace(pieces)
        # The curve's own shift, turned to the start heading.
        d = np.array([x[-1] * u_s[0] - y[-1] * u_s[1], x[-1] * u_s[1] + y[-1] * u_s[0]])
        r = targets - d
        # target = straight1 * u_s + curve + straight2 * u_e
        first = (r[:, 0] * u_e[1] - r[:, 1] * u_e[0]) / det
        second = (u_s[0] * r[:, 1] - u_s[1] * r[:, 0]) / det
        ok = (first >= -1e-9) & (second >= -1e-9)
        if not ok.any():
            continue
        curve = sum(p[0] for p in pieces)
        total = np.where(ok, first + second + curve, np.inf)
        i = int(np.argmin(total))
        if best is None or total[i] < best['length'] - 1e-9:
            best = {'length': float(total[i]), 'cell': targets[i].tolist(),
                    'first': max(0.0, float(first[i])), 'second': max(0.0, float(second[i])),
                    'peak': peak, 'curve': pieces, 'turn': turn}
    if best is None:
        return None
    best['pieces'] = ([(best['first'], 0.0, 0.0)] + best['curve'] +
                      [(best['second'], 0.0, 0.0)])
    return best


def straight_cell(index, resolution, length=STRAIGHT):
    """Return the straight move's end cell (x, y) from heading `index`."""
    sx, sy = STEPS[index]
    n = math.ceil(length / (resolution * math.hypot(sx, sy)) - 1e-9)
    return [n * sx * resolution, n * sy * resolution]


def poses(pieces, start, cell, end, resolution):
    """
    Return Nav2's poses for a move: [x, y, yaw] about every cell, the start left out.

    The move's pieces are traced from heading `start`; the last pose is exactly the end
    cell and heading.
    """
    s, x, y, yaw = trace(pieces)
    length = s[-1]
    n = max(1, math.ceil(length / resolution - 1e-9))
    out = []
    c, si = math.cos(start), math.sin(start)
    for k in range(1, n):
        at = k * length / n
        i = min(int(np.searchsorted(s, at)), len(s) - 1)
        out.append([round(float(x[i] * c - y[i] * si), 5),
                    round(float(x[i] * si + y[i] * c), 5),
                    float((yaw[i] + start) % math.tau)])
    out.append([round(cell[0], 5), round(cell[1], 5), float(end % math.tau)])
    return out


def rotate(p, quarter, count):
    """Return primitive `p` turned `quarter` quarter turns left (heading index + 4 each)."""
    a = quarter * math.pi / 2
    c, s = round(math.cos(a)), round(math.sin(a))
    q = dict(p)
    q['start_angle_index'] = (p['start_angle_index'] + 4 * quarter) % count
    q['end_angle_index'] = (p['end_angle_index'] + 4 * quarter) % count
    q['poses'] = [[round(x * c - y * s, 5) + 0.0, round(x * s + y * c, 5) + 0.0,
                   (yaw + a) % math.tau] for x, y, yaw in p['poses']]
    return q


def generate(radius, resolution, rate, wheelbase, speed=DESIGN_SPEED, turns=TURNS,
             moves='smooth'):
    """
    Return the lattice file (a dict, Nav2's format) of smooth moves for this car.

    radius: the planner's turning radius (plan_turning_radius); resolution: the map's
    cells; rate: the steering's rate (rad/s); speed: the design speed (m/s); moves: its
    lattice_moves:= name, kept in the metadata.
    """
    angles = headings()
    count = len(angles)
    sharp = sharpness(rate, wheelbase, speed)
    k_max = 1.0 / radius
    base = []
    # Headings 0-3 (east to just short of north); the rest are these turned 90 deg.
    for i in range(count // 4):
        cell = straight_cell(i, resolution)
        length = math.hypot(*cell)
        base.append({'start_angle_index': i, 'end_angle_index': i, 'left_turn': True,
                     'trajectory_radius': 0.0, 'trajectory_length': round(length, 5),
                     'arc_length': 0.0, 'straight_length': round(length, 5),
                     'poses': poses([(length, 0.0, 0.0)], angles[i], cell, angles[i],
                                    resolution)})
        for step in turns:
            for side in (1, -1):
                j = (i + side * step) % count
                move = best_turn(angles[i], angles[j], sharp, k_max, resolution)
                if move is None:
                    raise ValueError(f'no smooth move from heading {i} to {j} within {REACH} m')
                curve = sum(p[0] for p in move['curve'])
                base.append({
                    'start_angle_index': i, 'end_angle_index': j,
                    'left_turn': move['turn'] > 0,
                    'trajectory_radius': round(1.0 / move['peak'], 5),
                    'trajectory_length': round(move['length'], 5),
                    'arc_length': round(curve, 5),
                    'straight_length': round(move['first'] + move['second'], 5),
                    'poses': poses(move['pieces'], angles[i], move['cell'], angles[j],
                                   resolution)})
    prims = [rotate(p, q, count) for q in range(4) for p in base]
    prims.sort(key=lambda p: (p['start_angle_index'], p['end_angle_index']))
    for n, p in enumerate(prims):
        p['trajectory_id'] = n
    keys = ('trajectory_id', 'start_angle_index', 'end_angle_index', 'left_turn',
            'trajectory_radius', 'trajectory_length', 'arc_length', 'straight_length', 'poses')
    return {
        'version': 1.0,
        'date_generated': 'by tools/lattice_smooth.py',
        'lattice_metadata': {
            'motion_model': 'ackermann',
            'turning_radius': round(radius, 3),
            'grid_resolution': resolution,
            'stopping_threshold': 0,
            'num_of_headings': count,
            'heading_angles': angles,
            'number_of_trajectories': len(prims),
            # Ours (Nav2 and aav_planner read only the keys above):
            'moves': moves,
            'curvature_rate': round(sharp, 6),
            'design_speed': speed,
            'steering_rate': rate,
            'rate_margin': RATE_MARGIN,
        },
        'primitives': [{k: p[k] for k in keys} for p in prims],
    }


def curvature_steps(start, poses_, start_heading):
    """
    Return (curvatures, rates) along a move from its poses, as a follower would see them.

    Each step's curvature is its heading change over its length (the chord); the rate is
    the change from one step's curvature to the next over the distance between their
    middles. The move is taken as straight before and after (curvature 0 at both ends,
    where it joins the next), so a move that starts or ends on an arc shows that jump.
    """
    pts = [(start[0], start[1], start_heading)] + [tuple(p) for p in poses_]
    ks, lengths = [], []
    for (x0, y0, a0), (x1, y1, a1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        ks.append(math.remainder(a1 - a0, math.tau) / d)
        lengths.append(d)
    ks = [0.0] + ks + [0.0]
    halves = [0.0] + [d / 2 for d in lengths] + [0.0]
    rates = [abs(k1 - k0) / (h0 + h1) for k0, k1, h0, h1
             in zip(ks, ks[1:], halves, halves[1:])]
    return ks[1:-1], rates


def out_path(moves):
    """Return the moves' file (vehicle.lattice_file_name)."""
    return os.path.join(lp.CONFIG, vehicle.lattice_file_name(16, moves))


def picture_path(moves):
    """Return the picture of every move: docs/img/lattice-primitives-<moves>.png."""
    return os.path.join(ROOT, 'docs', 'img', f"lattice-primitives-{moves.replace('_', '-')}.png")


def problems(lattice, v, resolution, rate):
    """Return what is wrong with a smooth-moves file for this car, maps and steering."""
    out = lp.problems(lattice, v, resolution)
    meta = lattice['lattice_metadata']
    if meta.get('moves') not in SPEEDS:
        out.append('not a smooth-moves file (lattice_metadata.moves)')
        return out
    want = sharpness(rate, v['wheelbase'], float(meta.get('design_speed', DESIGN_SPEED)))
    if abs(float(meta.get('curvature_rate', 0.0)) - want) > 1e-5:
        out.append(f"curvature rate {meta.get('curvature_rate')} 1/m^2, but the steering "
                   f'allows {want:.6f} (steer_rate, max_steering_rate or the wheelbase changed)')
    return out


def picture(lattice, path):
    """Draw every move from the middle (as Nav2's generator's all_trajectories.png)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    for p in lattice['primitives']:
        xs = [0.0] + [q[0] for q in p['poses']]
        ys = [0.0] + [q[1] for q in p['poses']]
        ax.plot(xs, ys, color='tab:blue' if p['arc_length'] else 'tab:grey', lw=1)
    meta = lattice['lattice_metadata']
    ax.set_title(f"Smooth moves: {len(lattice['primitives'])}, curvature changes at most "
                 f"{meta['curvature_rate']:.3f} 1/m² ({meta['design_speed']} m/s)")
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    fig.savefig(path, dpi=90, bbox_inches='tight')
    plt.close(fig)


def load_car():
    """Return (vehicle.yaml, ego.yaml's ros__parameters)."""
    cfg = vehicle.load(lp.VEHICLE)
    with open(EGO) as f:
        ego = yaml.safe_load(f)['ego']['ros__parameters']
    return cfg, ego


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--check', action='store_true',
                        help='only check the committed files against vehicle.yaml and ego.yaml')
    args = parser.parse_args()
    cfg, ego = load_car()
    v = cfg['vehicle']
    rate = steering_rate(cfg, ego)
    resolution = lp.nav2_resolution()
    failed = False
    for moves, speed in SPEEDS.items():
        path = out_path(moves)
        if not args.check:
            lattice = generate(vehicle.plan_turning_radius(v), resolution, rate,
                               v['wheelbase'], speed, moves=moves)
            with open(path, 'w') as f:
                json.dump(lattice, f, indent=1)
                f.write('\n')
            picture(lattice, picture_path(moves))
        with open(path) as f:
            lattice = json.load(f)
        wrong = problems(lattice, v, resolution, rate)
        if lattice['lattice_metadata'].get('moves') != moves:
            wrong.append(f"its moves are {lattice['lattice_metadata'].get('moves')}, not {moves}")
        for line in wrong:
            print(f'FAIL  {os.path.relpath(path, ROOT)}: {line}. '
                  'Run pixi run python3 tools/lattice_smooth.py')
        if not wrong:
            meta = lattice['lattice_metadata']
            print(f"OK    {os.path.relpath(path, ROOT)}: {len(lattice['primitives'])} moves, "
                  f"radius {meta['turning_radius']} m, curvature rate "
                  f"{meta['curvature_rate']} 1/m^2 ({meta['design_speed']} m/s), "
                  f'{os.path.getsize(path) / 1024:.0f} kB')
        failed = failed or bool(wrong)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
