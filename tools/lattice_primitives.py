#!/usr/bin/env python3
"""
Make the car's own motion primitives for Nav2's State Lattice planner (planner:=lattice).

    pixi run lattice-primitives            (writes the file below, and its picture)
    pixi run lattice-primitives --check    (are the committed files still right for the car?)
    pixi run lattice-primitives --headings 32   (the trial: 32 headings, its own file)

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
  num_of_headings     16, the generator's own advice (the default file); --headings 32
                      writes a trial file beside it (lattice_headings:=32): finer
                      headings, more primitives, a bigger search
  stopping_threshold  5, the same, for 16 headings; 10 for 32. At 5, the 32-heading set
                      had no turn at all from the four straight headings (east, north,
                      west, south: one straight move each, 30 Oct), since the first
                      turn off them that lands on a 25 cm cell is further out than five
                      empty steps. At 10 each has a turn either way (2 s, 95 kB)

Out: ros/drivers/aav_navigation/config/lattice_ackermann.json (the planner's file) and
docs/img/lattice-primitives.png (every primitive, from the generator's own picture); with
--headings 32, lattice_ackermann_32.json and lattice-primitives-32.png.

Reverse moves are not in the file: the generator's ackermann model writes forward moves
only, and Nav2 adds each one backwards when allow_reverse_expansion is true, which the
launch sets with reversing:=true (vehicle.nav2_tuned).

The generator needs numpy, matplotlib and Rtree. Rtree isn't in our environment, so it
runs in a throwaway one: `pixi exec` (nothing changes in pixi.lock). It needs
`pixi run vendor` first, for the generator's source.
--check needs neither: it compares each committed file's turning radius and cells with
vehicle.yaml and the maps (tools/tests/test_lattice_primitives.py runs it).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros', 'bringup', 'aav_description'))
from aav_description import vehicle  # noqa: E402
import yaml  # noqa: E402

GENERATOR = os.path.join(ROOT, 'vendor', 'ros2-src', 'ecosystem', 'navigation2',
                         'nav2_smac_planner', 'lattice_primitives')
VEHICLE = os.path.join(ROOT, 'ros', 'bringup', 'aav_description', 'config', 'vehicle.yaml')
MAPS = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'maps')
CONFIG = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'config')
HEADINGS = vehicle.LATTICE_HEADINGS[0]      # the default file's (16)
OUT = os.path.join(CONFIG, vehicle.lattice_file_name(HEADINGS))
# The generator's stopping_threshold for each file (the docstring says why 32 needs 10).
STOPPING = {16: 5, 32: 10}
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


def out_path(headings=HEADINGS):
    """Return the primitives file for this many headings (vehicle.lattice_file_name)."""
    return os.path.join(CONFIG, vehicle.lattice_file_name(headings))


def picture_path(headings=HEADINGS):
    """Return the picture of every primitive: lattice-primitives[-32].png."""
    tail = '' if headings == HEADINGS else f'-{headings}'
    return os.path.join(ROOT, 'docs', 'img', f'lattice-primitives{tail}.png')


def config(v, resolution, headings=HEADINGS):
    """Return the generator's config.json for the car `v` (vehicle.yaml's vehicle)."""
    return {'motion_model': 'ackermann',
            'turning_radius': round(vehicle.plan_turning_radius(v), 3),
            'grid_resolution': resolution,
            'stopping_threshold': STOPPING[headings],
            'num_of_headings': headings}


def problems(lattice, v, resolution):
    """Return what is wrong with a primitives file (loaded) for this car and these maps."""
    # The same check navigation.launch.py makes before it starts the lattice planner.
    out = vehicle.lattice_problems(lattice['lattice_metadata'], v, resolution)
    if not lattice.get('primitives'):
        out.append('no primitives')
    # A heading with only a straight move is a dead end: the car could never turn off it.
    turns = {p['start_angle_index'] for p in lattice.get('primitives', [])
             if p['end_angle_index'] != p['start_angle_index']}
    stuck = sorted(set(range(lattice['lattice_metadata']['num_of_headings'])) - turns)
    if lattice.get('primitives') and stuck:
        out.append(f'no turn from heading(s) {stuck}: raise the stopping threshold')
    return out


def generate(cfg):
    """Run Nav2's generator on `cfg` in a throwaway environment; write its file, picture."""
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
            '--config', path, '--output', out_path(cfg['num_of_headings']),
            '--visualizations', os.path.join(work, 'pictures')]
        print(' '.join(command), flush=True)
        # From the work folder: the generator writes nothing beside its own source.
        started = time.monotonic()
        subprocess.run(command, cwd=work, check=True)
        print(f'The generator took {time.monotonic() - started:.0f} s', flush=True)
        shutil.copyfile(os.path.join(work, 'pictures', 'all_trajectories.png'),
                        picture_path(cfg['num_of_headings']))
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--check', action='store_true',
                        help='only check the committed files against vehicle.yaml and the maps')
    parser.add_argument('--headings', type=int, choices=vehicle.LATTICE_HEADINGS,
                        default=HEADINGS,
                        help=f'how many headings to make (default {HEADINGS}; 32 is the '
                             'trial file, lattice_headings:=32)')
    args = parser.parse_args()
    v = vehicle.load(VEHICLE)['vehicle']
    resolution = nav2_resolution()
    if not args.check:
        generate(config(v, resolution, args.headings))
    failed = False
    for headings in vehicle.LATTICE_HEADINGS if args.check else [args.headings]:
        path = out_path(headings)
        with open(path) as f:
            lattice = json.load(f)
        wrong = problems(lattice, v, resolution)
        found = lattice['lattice_metadata'].get('num_of_headings')
        if found != headings:
            wrong.append(f'{found} headings, not {headings}')
        flag = '' if headings == HEADINGS else f' --headings {headings}'
        for line in wrong:
            print(f'FAIL  {os.path.relpath(path, ROOT)}: {line}. '
                  f'Run pixi run lattice-primitives{flag}')
        if not wrong:
            meta = lattice['lattice_metadata']
            print(f"OK    {os.path.relpath(path, ROOT)}: {len(lattice['primitives'])} "
                  f"primitives, radius {meta['turning_radius']} m, cells "
                  f"{meta['grid_resolution']} m, {meta['num_of_headings']} headings, "
                  f'{os.path.getsize(path) / 1024:.0f} kB')
        failed = failed or bool(wrong)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
