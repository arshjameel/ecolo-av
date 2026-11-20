#!/usr/bin/env python3
"""
A Ravensim world from a labelled map: the campus twin (TODO 253, phase G).

    pixi run twin-world bags/<drive>/map               maps/twins/<drive>/twin.sdf
    pixi run twin-world maps/<area> --out maps/twins/lot [--keep-parked] [--min-share 0.5]
    pixi run sim world:=maps/twins/lot/twin.sdf        drive in it

The map folder's labels.yaml (label-drive's, fixed by a person in map-edit) says what is
where; its NDT points say how tall. Into the world go, on a flat lot:

| Label | Becomes |
|---|---|
| building | its box, brick or stone, as tall as its points (a roof on top) |
| tree | a trunk and a rounded crown, as tall as its points, as wide as its box |
| pole | a thin post, as tall as its points |
| other | a block (a fence, a bench), as its band says |
| parked_car | **nothing**: it may be gone next time (--keep-parked keeps them as blocks) |

A label seen in fewer than --min-share of the map's drives (seen_in) is left out too: what
one pass saw and others didn't moved (TODO 259). A person's labels with no seen_in stay.

It reuses tools/campus_world.py: its meshes (Mesh, the low-poly ball), textures and world
file, so the twin looks like the campus world and loads the same way. Writes, in --out:
twin.sdf, meshes/, textures/, map2d.pgm/.yaml and ndt.pcd (the map's, for the sim's
localisation), twin.yaml (the start pose, what went in and what was left out).

The maths (what stays, heights, shapes) has no ROS: tools/tests/test_twin_world.py.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import label_core as lc  # noqa: E402, I100

POLE_SIDE = 0.15        # m: a post's square
DEFAULT_TOP = {'building': 6.0, 'tree': 6.0, 'pole': 3.0, 'other': 1.5, 'parked_car': 1.5}
MARGIN = 20.0           # m of lot round what's labelled


def kept(labels, keep_parked=False, min_share=0.5):
    """
    Return (labels that go in, [(label, why) left out]).

    Out: parked cars (unless keep_parked); a label seen in fewer than min_share of the
    drives any label was seen in (it moved); occupancy's own unnamed boxes (source passes,
    class other: something one drive saw).
    """
    drives = lc.drives_of(labels)
    keep, out = [], []
    for lab in labels:
        seen = lab.get('seen_in', [])
        if lab['class'] == 'parked_car' and not keep_parked:
            out.append((lab, 'parked car: may be gone'))
        elif lab.get('source') == 'passes' and lab['class'] == 'other':
            out.append((lab, 'seen in one drive only (occupancy)'))
        elif seen and len(drives) > 1 and len(seen) < min_share * len(drives):
            out.append((lab, f'seen in {len(seen)} of {len(drives)} drives'))
        else:
            keep.append(lab)
    return keep, out


def top_of(label, points):
    """Return how tall a label's thing is (m): its points' 95th percentile, else its band."""
    box = lc.corners(label['region'])
    p = np.asarray(points, float).reshape(-1, 3)
    lo, hi = box.min(0), box.max(0)
    near = p[((p[:, :2] >= lo) & (p[:, :2] <= hi)).all(1)]
    near = near[lc.inside_polygon(near[:, :2], box)] if len(near) else near
    band = label.get('band') or {}
    if len(near) >= 5:
        top = float(np.percentile(near[:, 2], 95))
        return max(top, band.get('max_m', 0.0))
    return band.get('max_m') or DEFAULT_TOP[label['class']]


def tree(mesh_bark, mesh_leaves, centre, top, width):
    """Add a tree: a six-sided trunk to 45 % of its height, a rounded crown `width` wide."""
    import campus_world as cw
    crown = max(0.8, min(width / 2.0, top * 0.38))
    trunk_top = max(0.5, top - 1.8 * crown)
    radius = 0.08 + top * 0.02
    ring = [(centre[0] + radius * math.cos(i * math.pi / 3.0),
             centre[1] + radius * math.sin(i * math.pi / 3.0)) for i in range(6)]
    mesh_bark.prism_walls(ring + ring[:1], cw.BASE_Z, trunk_top, (1.0, 2.0))
    middle = (centre[0], centre[1], top - crown)
    for a, b, c in cw.BALL:
        corners = [tuple(middle[i] + p[i] * crown for i in range(3)) for p in (a, b, c)]
        mesh_leaves.triangle(*corners, (0.0, 0.0), (1.0, 0.0), (0.5, 1.0))
    return ring


def build(labels, points):
    """Return (meshes by material, solids, report): the twin's shapes from labels and points."""
    import campus_world as cw
    names = ['lot', 'grass', 'brick', 'stone', 'roof', 'fence', 'bark', 'leaves']
    meshes = {n: cw.Mesh() for n in names}
    solids = cw.Mesh()
    report = dict.fromkeys(lc.CLASSES, 0)
    for lab in labels:
        cls = lab['class']
        ring = [tuple(p) for p in lc.corners(lab['region'])]
        top = top_of(lab, points)
        if cls == 'building':
            material = 'stone' if lab['id'] % 3 == 0 else 'brick'
            meshes[material].prism_walls(ring + ring[:1], cw.BASE_Z, top,
                                         (12.0, 3 * cw.FLOOR_HEIGHT))
            meshes['roof'].flat(ring + ring[:1], top, tile=8.0)
            solids.prism_walls(ring + ring[:1], 0.0, top)
        elif cls == 'tree':
            (x, y), size = lab['region']['centre_m'], lab['region']['size_m']
            trunk = tree(meshes['bark'], meshes['leaves'], (x, y), top, min(size))
            solids.prism_walls(trunk + trunk[:1], 0.0, top)
        elif cls == 'pole':
            x, y = lab['region']['centre_m']
            h = POLE_SIDE / 2.0
            post = [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
            meshes['fence'].prism_walls(post + post[:1], 0.0, top, (1.0, 1.0))
            solids.prism_walls(post + post[:1], 0.0, top)
        else:                                   # other, and parked cars kept as blocks
            low = (lab.get('band') or {}).get('min_m', 0.0)
            low = 0.0 if low < 0.5 else low
            meshes['stone'].prism_walls(ring + ring[:1], low, top, (3.0, 3.0))
            meshes['roof'].flat(ring + ring[:1], top, tile=4.0)
            solids.prism_walls(ring + ring[:1], low, top)
        report[cls] += 1
    return meshes, solids, report


def ground(meshes, bounds):
    """Lay the lot under everything (bounds grown by MARGIN), grass beyond to the horizon."""
    import campus_world as cw
    (x0, y0), (x1, y1) = np.asarray(bounds[0]) - MARGIN, np.asarray(bounds[1]) + MARGIN
    meshes['lot'].flat([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], cw.LOT_Z, tile=8.0)
    far = 2500.0
    for ring in ([(-far, -far), (far, -far), (far, y0), (-far, y0)],
                 [(-far, y1), (far, y1), (far, far), (-far, far)],
                 [(-far, y0), (x0, y0), (x0, y1), (-far, y1)],
                 [(x1, y0), (far, y0), (far, y1), (x1, y1)]):
        meshes['grass'].flat(ring, -0.03, tile=6.0)


def start_pose(map_dir):
    """Return where the car starts: the drive's first pose (poses.tum), else the origin."""
    tum = Path(map_dir) / 'poses.tum'
    if not tum.exists():
        return (0.0, 0.0, 0.0)
    t, x, y, _, qx, qy, qz, qw = np.loadtxt(tum, ndmin=2)[0]
    return (round(float(x), 2), round(float(y), 2), round(2 * math.atan2(qz, qw), 3))


def main():
    """Make the twin."""
    import campus_world as cw
    import ndt_map
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('map', type=Path, help='a map folder with labels.yaml (and ndt.pcd)')
    p.add_argument('--out', type=Path, help='default maps/twins/<the drive or area>')
    p.add_argument('--keep-parked', action='store_true', help='keep parked cars, as blocks')
    p.add_argument('--min-share', type=float, default=0.5,
                   help="of the map's drives a label must be seen in (0.5)")
    args = p.parse_args()
    name = args.map.parent.name if args.map.name.startswith('map') else args.map.name
    out = args.out or ROOT / 'maps' / 'twins' / name
    labels = lc.read_labels(args.map / 'labels.yaml')
    if not labels:
        raise SystemExit(f'{args.map}: no labels.yaml (pixi run label-drive first)')
    points = ndt_map.read_pcd(args.map / 'ndt.pcd') if (args.map / 'ndt.pcd').exists() \
        else np.zeros((0, 3))
    keep, left_out = kept(labels, args.keep_parked, args.min_share)
    meshes, solids, report = build(keep, points)
    every = np.concatenate([lc.corners(lab['region']) for lab in labels])
    start = start_pose(args.map)
    every = np.vstack([every, [start[:2]]])
    ground(meshes, (every.min(0), every.max(0)))
    (out / 'meshes').mkdir(parents=True, exist_ok=True)
    names = [n for n, m in meshes.items() if m.vertices]
    for n in names:
        meshes[n].write(out / 'meshes' / f'{n}.obj')
    solids.write(out / 'meshes' / 'solids.obj')
    cw.make_textures(out / 'textures')
    datum = yaml.safe_load(cw.NAVSAT.read_text())['navsat_transform']['ros__parameters']['datum']
    note = (f'A twin of {args.map}: made by tools/twin_world.py from its labels.yaml and\n'
            "  NDT's points (TODO 253). Don't edit this: make it again.")
    (out / 'twin.sdf').write_text(cw.world_sdf(names, (datum[0], datum[1]), note).replace(
        '<model name="campus">', '<model name="twin">'))
    for f in ('map2d.pgm', 'map2d.yaml', 'ndt.pcd', 'ndt.yaml'):
        if (args.map / f).exists():
            shutil.copy(args.map / f, out / f)
    (out / 'twin.yaml').write_text(yaml.safe_dump({
        'from': str(args.map), 'start': {'x': start[0], 'y': start[1], 'yaw': start[2]},
        'in': report, 'left_out': [{'id': lab['id'], 'class': lab['class'], 'why': why}
                                   for lab, why in left_out],
        'datum': [datum[0], datum[1]]}, sort_keys=False))
    print(f'{out / "twin.sdf"}: {report}; left out {len(left_out)}; start {start}')


if __name__ == '__main__':
    main()
