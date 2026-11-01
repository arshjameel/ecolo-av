#!/usr/bin/env python3
"""
Make the car's own motion primitives for Nav2's State Lattice planner (planner:=lattice).

    pixi run lattice-primitives            (writes the file below, and its picture)
    pixi run lattice-primitives --check    (is the committed file still right for the car?)

SmacPlannerLattice joins short car-shaped moves ("primitives") into a path. Nav2 ships
primitives only for small robots (0.5 m and 1 m turning radius at 5 cm cells); ours turns
no tighter than vehicle.plan_turning_radius (3.59 m today) and Nav2's map has 25 cm
cells. So this runs Nav2's own generator (vendor/ros2-src/.../nav2_smac_planner/
lattice_primitives, Apache-2.0, at the commit in vendor/ros2-src.repos) with:

  motion_model        ackermann: forward moves; Nav2 adds the same moves backwards only
                      when allow_reverse_expansion is true (reversing:=true)
  turning_radius      vehicle.plan_turning_radius: the car's full lock, PLAN_TURN_MARGIN
                      wider, the same radius the Hybrid planner plans with
  grid_resolution     the maps' resolution (aav_navigation/maps/*.yaml: all 0.25 m). The
                      planner needs it equal to the global costmap's
  num_of_headings     16, the generator's own advice
  stopping_threshold  5, the same

Out: ros/drivers/aav_navigation/config/lattice_ackermann.json (the planner's file) and
docs/img/lattice-primitives.png (every primitive, from the generator's own picture).

The generator needs numpy, matplotlib and Rtree. Rtree isn't in our environment, so it
runs in a throwaway one: `pixi exec` (nothing changes in pixi.lock). It needs
`pixi run vendor` first, for the generator's source.
--check needs neither: it compares the committed file's turning radius and cells with
vehicle.yaml and the maps (tools/tests/test_lattice_primitives.py runs it).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros', 'bringup', 'aav_description'))
from aav_description import vehicle  # noqa: E402
import yaml  # noqa: E402

GENERATOR = os.path.join(ROOT, 'vendor', 'ros2-src', 'ecosystem', 'navigation2',
                         'nav2_smac_planner', 'lattice_primitives')
VEHICLE = os.path.join(ROOT, 'ros', 'bringup', 'aav_description', 'config', 'vehicle.yaml')
MAPS = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'maps')
OUT = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'config', 'lattice_ackermann.json')
PICTURE = os.path.join(ROOT, 'docs', 'img', 'lattice-primitives.png')
HEADINGS = 16
STOPPING = 5
# The throwaway environment's packages (conda-forge), for `pixi exec`.
SPECS = ('python=3.12', 'numpy', 'matplotlib-base', 'rtree')
# AMCL's own map is finer (5 cm), but only AMCL reads it, never Nav2's planner.
NOT_NAV2 = ('example_lot_amcl.yaml',)
TOLERANCE = vehicle.LATTICE_TOLERANCE   # m: the file's radius against vehicle.yaml's


def nav2_resolution(maps=MAPS):
    """Return the one resolution (m) of every map Nav2 plans on; raise if they differ."""
    found = {}
    for name in sorted(os.listdir(maps)):
        if name.endswith('.yaml') and not name.endswith('.spec.yaml') and name not in NOT_NAV2:
            with open(os.path.join(maps, name)) as f:
                found[name] = float(yaml.safe_load(f)['resolution'])
    if len(set(found.values())) != 1:
        raise ValueError(f'the maps differ in resolution ({found}): the lattice needs one')
    return next(iter(found.values()))


def config(v, resolution):
    """Return the generator's config.json for the car `v` (vehicle.yaml's vehicle)."""
    return {'motion_model': 'ackermann',
            'turning_radius': round(vehicle.plan_turning_radius(v), 3),
            'grid_resolution': resolution,
            'stopping_threshold': STOPPING,
            'num_of_headings': HEADINGS}


def problems(lattice, v, resolution):
    """Return what is wrong with a primitives file (loaded) for this car and these maps."""
    # The same check navigation.launch.py makes before it starts the lattice planner.
    out = vehicle.lattice_problems(lattice['lattice_metadata'], v, resolution)
    if not lattice.get('primitives'):
        out.append('no primitives')
    return out


def generate(cfg):
    """Run Nav2's generator on `cfg` in a throwaway environment; write OUT and PICTURE."""
    if not os.path.isfile(os.path.join(GENERATOR, 'generate_motion_primitives.py')):
        sys.exit(f'No generator at {GENERATOR}: run `pixi run vendor` first')
    # Its config and pictures in a folder of our own; the source stays read-only.
    work = tempfile.mkdtemp(prefix='lattice_')
    try:
        path = os.path.join(work, 'config.json')
        with open(path, 'w') as f:
            json.dump(cfg, f, indent=2)
        command = ['pixi', 'exec'] + [a for s in SPECS for a in ('--spec', s)] + [
            'python', os.path.join(GENERATOR, 'generate_motion_primitives.py'),
            '--config', path, '--output', OUT,
            '--visualizations', os.path.join(work, 'pictures')]
        print(' '.join(command), flush=True)
        # From the work folder: the generator writes nothing beside its own source.
        subprocess.run(command, cwd=work, check=True)
        shutil.copyfile(os.path.join(work, 'pictures', 'all_trajectories.png'), PICTURE)
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--check', action='store_true',
                        help='only check the committed file against vehicle.yaml and the maps')
    args = parser.parse_args()
    v = vehicle.load(VEHICLE)['vehicle']
    resolution = nav2_resolution()
    if not args.check:
        generate(config(v, resolution))
    with open(OUT) as f:
        lattice = json.load(f)
    wrong = problems(lattice, v, resolution)
    for line in wrong:
        print(f'FAIL  {os.path.relpath(OUT, ROOT)}: {line}. Run pixi run lattice-primitives')
    if not wrong:
        meta = lattice['lattice_metadata']
        print(f"OK    {os.path.relpath(OUT, ROOT)}: {len(lattice['primitives'])} primitives, "
              f"radius {meta['turning_radius']} m, cells {meta['grid_resolution']} m, "
              f"{meta['num_of_headings']} headings")
    return 1 if wrong else 0


if __name__ == '__main__':
    sys.exit(main())
