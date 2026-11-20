#!/usr/bin/env python3
"""
One area's map from several drives, offline (TODO 260): what stays, without parked cars.

    pixi run map-area lot bags/<drive1> bags/<drive2> bags/<drive3>     into maps/lot/
    pixi run map-area lot bags/... --out /tmp/lot --truth bags/<drive1>/map/truth.pcd

Runs in the slam environment (pixi install -e slam, once): no ROS. The page:
docs/maps-from-drives.md, "Many drives of one area".

    drive 1 ─┐  1. map-drive each (KISS-SLAM, the GPS fit, NDT's map, the 2D map)
    drive 2 ─┼─▶ 2. align: each onto the first by ICP (x, y, yaw)
    drive 3 ─┘  3. passes: each drive's rays again, on one grid; a cell hit in one drive
                   and seen through in another is gone (a parked car); hit by one drive
                   only and never seen through, kept and flagged (seen_in)
                4. NDT's map: every drive's points, less the gone ones; the 2D map merged
                5. chunks (64 m), labels.yaml (seen_in), report.md

Writes, in --out (default maps/<area>): ndt.pcd/.yaml, map2d.pgm/.yaml, poses.tum (the
first drive's path), labels.yaml (the cells only one drive saw, for map-edit), passes.yaml
(every drive's move and counts), chunks/, report.md; drives/<name>/ (each drive's own
map-drive, kept: run again, it is reused unless --redo). `pixi run map-edit <out>` opens it.

The maths: tools/passes.py (tools/tests/test_passes.py) and tools/slam_map.py.
"""

import argparse
import datetime
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import map_drive as md  # noqa: E402, I100
import ndt_map  # noqa: E402, I100
import passes as ps  # noqa: E402, I100
import slam_map as sm  # noqa: E402, I100

STAMP_GAP = 0.005       # s: a cloud and its pose in poses.tum (written to the microsecond)


def map_each(drives, out, redo, extra):
    """Run map-drive on each drive into out/drives/<name>/ (reused if there); return them."""
    folders = []
    for drive in drives:
        folder = out / 'drives' / drive.name
        if redo or not (folder / 'ndt.pcd').exists() or not (folder / 'poses.tum').exists():
            print(f'== map-drive {drive}', flush=True)
            done = subprocess.run([sys.executable, str(ROOT / 'tools' / 'map_drive.py'),
                                   str(drive), '--out', str(folder), '--no-chunks', *extra])
            if done.returncode:
                raise SystemExit(f'map-drive failed on {drive}')
        else:
            print(f'== {drive.name}: its map is there ({folder}); --redo to make it again',
                  flush=True)
        folders.append(folder)
    return folders


def read_poses(folder):
    """Return (stamps, [(x, y, yaw)]) of a map folder's poses.tum."""
    rows = ndt_map.read_tum(folder / 'poses.tum')
    return (np.array([t for t, _, _ in rows]),
            [(p[0], p[1], 2.0 * np.arctan2(q[2], q[3])) for _, p, q in rows])


def trace(drive, car, stamps, poses, grid):
    """Lay one drive's clouds on its poses into grid (a slam_map.Occupancy); return clouds."""
    used = 0
    for _, stamp, msg in md.messages(md.mcap_files(drive), (md.CLOUDS,)):
        i = int(np.searchsorted(stamps, stamp))
        best = min((j for j in (i - 1, i) if 0 <= j < len(stamps)),
                   key=lambda j: abs(stamps[j] - stamp), default=None)
        if best is None or abs(stamps[best] - stamp) > STAMP_GAP:
            continue
        points, _ = md.cloud_of(msg)
        base = md.od.to_base_link(points, car['xyz'], car['rpy'])
        base, _ = md.od.crop(base, None, car['body_min'], car['body_max'], 0.5,
                             ndt_map.MAX_RANGE, car['xyz'])
        grid.add(base, poses[best], sensor=car['xyz'][:2])
        used += 1
    return used


def truth_score(points, truth, near=0.2, off=0.3):
    """Return how a map matches the truth's: share of truth found, points off it."""
    if len(points) == 0:
        return {'points': 0}
    to_truth = sm.distances(points, truth)
    to_map = sm.distances(truth, points)
    return {'points': int(len(points)),
            'truth_found': round(float(np.mean(to_map <= near)), 4),
            'off_truth': int(np.sum(to_truth > off)),
            'median_m': round(float(np.median(to_truth)), 3)}


def make(args):
    started = time.monotonic()
    drives = [Path(d) for d in args.drives]
    names = [d.name for d in drives]
    if len(set(names)) != len(names):
        raise SystemExit('two drives with the same folder name: rename one')
    out = Path(args.out or ROOT / 'maps' / args.area).resolve()
    out.mkdir(parents=True, exist_ok=True)
    extra = ['--no-gps'] if args.no_gps else []
    folders = map_each(drives, out, args.redo, extra)

    print('== Align: each drive onto the first (ICP: x, y, yaw)', flush=True)
    clouds = [ndt_map.read_pcd(f / 'ndt.pcd') for f in folders]
    fits = [((0.0, 0.0, 0.0), {'reference': True})]
    for name, points in zip(names[1:], clouds[1:]):
        fits.append(md.icp_onto(clouds[0], points, args.icp_reach))
        print(f'   {name}: x {fits[-1][1]["x_m"]} m, y {fits[-1][1]["y_m"]} m, yaw '
              f'{fits[-1][1]["yaw_deg"]} deg, fitness {fits[-1][1]["icp_fitness"]}', flush=True)
    placed = [md.moved(points, t) for points, (t, _) in zip(clouds, fits)]
    paths = []
    for f, (t, _) in zip(folders, fits):
        stamps, poses = read_poses(f)
        paths.append((stamps, [sm.compose(t, p) for p in poses]))

    print("== Passes: each drive's rays again, on one grid", flush=True)
    xy = np.concatenate([np.array([(x, y) for x, y, _ in p]) for _, p in paths])
    bounds = (xy.min(axis=0) - ndt_map.MAX_RANGE, xy.max(axis=0) + ndt_map.MAX_RANGE)
    grids = []
    for drive, folder, (stamps, poses) in zip(drives, folders, paths):
        own = drive / 'vehicle.yaml'
        car = md.lidar_car(own if own.exists() else md.VEHICLE)
        grid = sm.Occupancy(bounds, args.cell, max_range=ndt_map.MAX_RANGE)
        used = trace(drive, car, stamps, poses, grid)
        print(f'   {drive.name}: {used} clouds', flush=True)
        grids.append(grid)
    occ = np.stack([ps.occupied(g.hits, g.free) for g in grids])
    free = np.stack([ps.seen_free(g.hits, g.free, tolerance=args.tolerance) for g in grids])
    classes = ps.classify(occ, free, grow=args.grow)
    origin, cell = grids[0].origin, grids[0].cell

    kept, removed = [], []
    for points in placed:
        left, gone = ps.drop(points, classes, origin, cell, top=args.top)
        kept.append(left)
        removed.append(gone)
    merged = ndt_map.thin(np.concatenate(kept), args.voxel)
    union = ndt_map.thin(np.concatenate(placed), args.voxel)
    ndt_map.write_pcd(out / 'ndt.pcd', merged)

    reference = yaml.safe_load((folders[0] / 'ndt.yaml').read_text()) or {}
    placed_by = reference.get('placed', {})
    lo, hi = merged.min(axis=0), merged.max(axis=0)
    (out / 'ndt.yaml').write_text(yaml.safe_dump({
        'frame': reference.get('frame', 'map'), 'placed': placed_by,
        'area': args.area, 'drives': [str(d) for d in drives],
        'poses': f"KISS-SLAM per drive, each fitted onto {names[0]}'s by ICP (passes.yaml)",
        'git_commit': ndt_map.git_commit(), 'voxel_m': args.voxel, 'points': int(len(merged)),
        'dynamic_removed': dict(zip(names, removed)),
        'ground_removed_below_m': ndt_map.MIN_HEIGHT, 'max_range_m': ndt_map.MAX_RANGE,
        'bounds_m': {'min': [round(float(v), 2) for v in lo],
                     'max': [round(float(v), 2) for v in hi]},
        'made': datetime.datetime.now().isoformat(timespec='seconds')}, sort_keys=False))
    grid, grid_origin = sm.trim(ps.to_pgm(classes), origin, cell)
    sm.write_map2d(out / 'map2d', grid, grid_origin, cell, note=(
        f'tools/map_area.py, {args.area}: {len(drives)} drives merged; occupied in a drive and '
        'never seen free in another'))
    with open(out / 'map2d.yaml', 'a') as f:
        f.write(yaml.safe_dump({'placed': placed_by}, sort_keys=False))
    sm.write_tum(out / 'poses.tum', *paths[0])

    labels = ps.seen_once_labels(classes, occ, names, origin, cell, args.min_label_cells)
    (out / 'labels.yaml').write_text(
        '# map_area.py: what only one drive saw, never seen free by another (TODO 259).\n'
        '# Keep, relabel or delete them in pixi run map-edit.\n' +
        yaml.safe_dump({'version': 1, 'frame': 'map', 'labels': labels}, sort_keys=False))
    counts = ps.summary(classes, cell)
    per_pass = []
    for i, name in enumerate(names):
        per_pass.append({
            'drive': str(drives[i]), 'onto_first': fits[i][1], 'points': int(len(placed[i])),
            'points_removed': removed[i],
            'cells_occupied': int(occ[i].sum()), 'cells_free': int(free[i].sum()),
            'cells_dynamic_it_hit': int((occ[i] & classes['dynamic']).sum()),
            'cells_only_it_saw': int((occ[i] & classes['once']).sum())})
    (out / 'passes.yaml').write_text(yaml.safe_dump({
        'area': args.area, 'cell_m': cell, 'rules': {
            'min_hits': ps.MIN_HITS, 'min_free': ps.MIN_FREE, 'tolerance_cells': args.tolerance,
            'grow_cells': args.grow, 'top_m': args.top},
        'cells': counts, 'labels': len(labels), 'passes': per_pass}, sort_keys=False))
    origin_m = yaml.safe_load((out / 'map2d.yaml').read_text())['origin'][:2]
    cut = 'skipped (--no-chunks)' if args.no_chunks else md.chunks(out, origin_m)

    scores = None
    if args.truth:
        truth = ndt_map.read_pcd(args.truth)
        scores = {'merged (this)': truth_score(merged, truth),
                  'every drive, nothing removed': truth_score(union, truth)}
    text = page(args.area, out, drives, per_pass, counts, labels, merged, cut, scores,
                args.truth, time.monotonic() - started)
    (out / 'report.md').write_text(text)
    print(text)
    return 0


def page(area, out, drives, per_pass, counts, labels, merged, cut, scores, truth, seconds):
    """Return report.md."""
    lines = [f'# The {area} map from {len(drives)} drives', '',
             f'{seconds:.0f} s. Open it: `pixi run map-edit {out}`.', '',
             '| Drive | Onto the first (x, y, yaw) | Points | Removed (dynamic) | '
             'Cells it hit that went | Cells only it saw |', '|---|---|---|---|---|---|']
    for p in per_pass:
        fit = p['onto_first']
        move = ('the reference' if fit.get('reference') else
                f'{fit["x_m"]} m, {fit["y_m"]} m, {fit["yaw_deg"]} deg '
                f'(fitness {fit["icp_fitness"]})')
        lines.append(f'| {Path(p["drive"]).name} | {move} | {p["points"]:,} | '
                     f'{p["points_removed"]:,} | {p["cells_dynamic_it_hit"]} | '
                     f'{p["cells_only_it_saw"]} |')
    lines += ['', '| Cells (2D) | How many | m² |', '|---|---|---|']
    words = {'kept': 'kept (occupied)', 'dynamic': 'dynamic: hit in one, free in another',
             'removed': 'removed (dynamic and its edge)', 'once': 'seen in one drive only, kept',
             'free': 'free'}
    lines += [f'| {words[k]} | {v["cells"]:,} | {v["m2"]} |' for k, v in counts.items()]
    lines += ['', f"NDT's map: {len(merged):,} points. labels.yaml: {len(labels)} "
              'box(es) seen in one drive only (`seen_in`), to check in the editor.',
              f'Chunks: {cut}.']
    if scores:
        lines += ['', f'## Against the truth (`{truth}`)', '',
                  '| Map | Points | Truth found (within 0.2 m) | Points off the truth '
                  '(over 0.3 m) | Median off |', '|---|---|---|---|---|']
        for name, s in scores.items():
            lines.append(f'| {name} | {s["points"]:,} | {s["truth_found"]:.1%} | '
                         f'{s["off_truth"]:,} | {s["median_m"]} m |')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('area', help="the area's name: the folder maps/<area>")
    parser.add_argument('drives', nargs='+', help='the drives (bags/<date>_<name>), two or more')
    parser.add_argument('--out', help='where the map goes (default maps/<area>)')
    parser.add_argument('--redo', action='store_true', help='map-drive each again')
    parser.add_argument('--no-gps', action='store_true', help='map-drive --no-gps')
    parser.add_argument('--no-chunks', action='store_true', help="don't cut chunks")
    parser.add_argument('--truth', help='a simulator truth map (truth.pcd) to score against')
    parser.add_argument('--cell', type=float, default=md.CELL, help=f'2D cell, m ({md.CELL})')
    parser.add_argument('--voxel', type=float, default=ndt_map.VOXEL,
                        help='NDT map thinning, m (0.1)')
    parser.add_argument('--icp-reach', type=float, default=1.0,
                        help="ICP's largest match distance, m (1.0)")
    parser.add_argument('--tolerance', type=int, default=ps.TOLERANCE,
                        help=f"cells kept off a drive's own occupied ones ({ps.TOLERANCE})")
    parser.add_argument('--grow', type=int, default=ps.GROW,
                        help=f'cells a dynamic cluster takes of its seen-once edge ({ps.GROW})')
    parser.add_argument('--top', type=float, default=ps.TOP,
                        help=f"m: a removed cell's NDT points up to here go ({ps.TOP})")
    parser.add_argument('--min-label-cells', type=int, default=ps.MIN_LABEL_CELLS,
                        help=f'cells: a smaller seen-once cluster gets no label '
                             f'({ps.MIN_LABEL_CELLS})')
    args = parser.parse_args()
    if len(args.drives) < 2:
        parser.error('two drives or more')
    return make(args)


if __name__ == '__main__':
    sys.exit(main())
