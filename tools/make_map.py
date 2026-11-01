#!/usr/bin/env python3
"""
Make a Nav2 map (PGM image + YAML) from a short spec. No ROS needed.

A Nav2 map is two files:
  <name>.pgm   greyscale image, one pixel per cell: white = free, black = blocked,
               grey = unknown. Row 0 is the top (the far +y edge).
  <name>.yaml  image file, resolution (metres per pixel), origin [x, y, yaw] of the
               BOTTOM-LEFT pixel in the map frame, and the thresholds that turn grey
               levels into free/occupied.

With GPS, the map frame's (0, 0) is the datum set on navsat_transform (latitude,
longitude), x points east and y north. So a map drawn in metres east/north of the
datum lines up with GPS.

Spec (YAML):
  name: example_lot
  resolution: 0.25          # m per pixel
  size: [80, 60]            # metres east, north
  origin: [-10, -10]        # map coordinates of the bottom-left corner
  default: free             # free | blocked | unknown
  border: 1.0               # metres of wall around the edge (0 = none)
  blocked:                  # rectangles [x_min, y_min, x_max, y_max] in map metres
    - [20, 15, 30, 25]
  free: []                  # rectangles carved back out after `blocked`
  amcl:                     # optional: a second map, <name>_amcl, for AMCL only
    resolution: 0.05        #   finer cells
    outline: true           #   only each obstacle's edge is blocked, its inside unknown

Why AMCL gets its own map (docs/trials.md, 29 Oct): its likelihood field scores each beam
by the distance from its end to the nearest blocked cell's centre. On 0.25 m cells a beam
that ends on a wall's face lands in the free cell before it, 0.25 m "off", and a pose that
pushes the beams into a solid building scores better than the truth. Fine cells and
outlines (what a lidar really sees, and what a SLAM map holds) put the best score at the
truth. Nav2's planner and costmaps keep the solid map.

usage: tools/make_map.py spec.yaml [output_dir]
"""

import math
from pathlib import Path
import sys

import yaml

FREE, BLOCKED, UNKNOWN = 254, 0, 205


def rect_cells(rect, origin, res, width, height):
    x0, y0, x1, y1 = rect
    c0 = max(0, int(math.floor((x0 - origin[0]) / res)))
    c1 = min(width, int(math.ceil((x1 - origin[0]) / res)))
    r0 = max(0, int(math.floor((y0 - origin[1]) / res)))
    r1 = min(height, int(math.ceil((y1 - origin[1]) / res)))
    return c0, c1, r0, r1


def make(spec):
    res = float(spec['resolution'])
    w_m, h_m = spec['size']
    origin = spec.get('origin', [0.0, 0.0])
    width, height = int(round(w_m / res)), int(round(h_m / res))
    fill = {'free': FREE, 'blocked': BLOCKED, 'unknown': UNKNOWN}[spec.get('default', 'free')]
    grid = [[fill] * width for _ in range(height)]   # grid[row from bottom][col]

    def paint(rect, value):
        c0, c1, r0, r1 = rect_cells(rect, origin, res, width, height)
        for r in range(r0, r1):
            for c in range(c0, c1):
                grid[r][c] = value

    border = float(spec.get('border', 0.0))
    if border > 0:
        x0, y0 = origin
        x1, y1 = x0 + w_m, y0 + h_m
        for rect in ([x0, y0, x1, y0 + border], [x0, y1 - border, x1, y1],
                     [x0, y0, x0 + border, y1], [x1 - border, y0, x1, y1]):
            paint(rect, BLOCKED)
    for rect in spec.get('blocked', []):
        paint(rect, BLOCKED)
    for rect in spec.get('free', []):
        paint(rect, FREE)
    if spec.get('outline'):
        grid = outline(grid)
    return grid, width, height, res, origin


def outline(grid):
    """
    Return the grid with each blocked cell whose four neighbours are all blocked unknown.

    Beyond the map's edge counts as blocked, so a wall along the edge keeps only its
    inner face.
    """
    height, width = len(grid), len(grid[0])

    def blocked(r, c):
        return not (0 <= r < height and 0 <= c < width) or grid[r][c] == BLOCKED

    sides = ((1, 0), (-1, 0), (0, 1), (0, -1))
    return [[UNKNOWN if grid[r][c] == BLOCKED and all(blocked(r + dr, c + dc)
                                                      for dr, dc in sides)
             else grid[r][c] for c in range(width)] for r in range(height)]


def amcl_spec(spec):
    """Return the spec of the AMCL map its `amcl:` section asks for, or None."""
    extra = spec.get('amcl')
    if not extra:
        return None
    return {**spec, 'amcl': None, **extra, 'name': f"{spec['name']}_amcl"}


def write(spec, out_dir):
    grid, width, height, res, origin = make(spec)
    name = spec['name']
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / f'{name}.pgm', 'wb') as f:
        f.write(f'P5\n# {name}, made by tools/make_map.py\n{width} {height}\n255\n'.encode())
        for row in reversed(grid):   # PGM rows run top to bottom
            f.write(bytes(row))
    meta = {
        'image': f'{name}.pgm',
        'mode': 'trinary',
        'resolution': res,
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out_dir / f'{name}.yaml', 'w') as f:
        f.write(f'# {name}: {width * res:g} m x {height * res:g} m at {res} m/pixel '
                f'({width} x {height} pixels). Made by tools/make_map.py.\n')
        yaml.safe_dump(meta, f, sort_keys=False)
    return width, height


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    spec_path = Path(sys.argv[1])
    spec = yaml.safe_load(spec_path.read_text())
    out_dir = sys.argv[2] if len(sys.argv) > 2 else spec_path.parent
    for one in (spec, amcl_spec(spec)):
        if one is not None:
            width, height = write(one, out_dir)
            print(f'wrote {one["name"]}.pgm/.yaml: {width} x {height} px, '
                  f'{width * one["resolution"]:g} x {height * one["resolution"]:g} m')


if __name__ == '__main__':
    main()
