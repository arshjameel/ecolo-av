"""lattice_primitives: the committed primitives still fit the car and Nav2's maps."""

import copy
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lattice_primitives as lp  # noqa: E402
import pytest  # noqa: E402

V = lp.vehicle.load(lp.VEHICLE)['vehicle']
# Every committed file: 16 headings (the default) and 32 (the trial, lattice_headings:=32).
HEADINGS = pytest.mark.parametrize('headings', lp.vehicle.LATTICE_HEADINGS)


def committed(headings=lp.HEADINGS):
    with open(lp.out_path(headings)) as f:
        return json.load(f)


@HEADINGS
def test_the_committed_file_fits_vehicle_yaml_and_the_maps(headings):
    # Fails after a change to the wheelbase, the steering lock or a map's cells:
    # run pixi run lattice-primitives (--headings 32) and commit what it writes.
    lattice = committed(headings)
    assert lp.problems(lattice, V, lp.nav2_resolution()) == []
    assert lattice['lattice_metadata']['num_of_headings'] == headings


@HEADINGS
def test_no_primitive_turns_tighter_than_the_planners_radius(headings):
    lattice = committed(headings)
    radius = lattice['lattice_metadata']['turning_radius']
    arcs = [p['trajectory_radius'] for p in lattice['primitives'] if p['trajectory_radius']]
    assert arcs and min(arcs) >= radius - lp.TOLERANCE
    # ... and so wider than the car's own full lock, which the follower then has to spare.
    assert min(arcs) > lp.vehicle.min_turning_radius(V)


@HEADINGS
def test_every_heading_has_moves_forward_and_a_turn(headings):
    lattice = committed(headings)
    every = set(range(lattice['lattice_metadata']['num_of_headings']))
    starts = {p['start_angle_index'] for p in lattice['primitives']}
    turns = {p['start_angle_index'] for p in lattice['primitives']
             if p['end_angle_index'] != p['start_angle_index']}
    assert starts == every and turns == every


def test_a_heading_with_no_turn_is_caught():
    # 32 headings at the generator's own stopping threshold (5): only straight moves from
    # east, north, west and south (30 Oct). Here: heading 0's turns taken out.
    lattice = copy.deepcopy(committed(32))
    lattice['primitives'] = [p for p in lattice['primitives']
                             if p['start_angle_index'] != 0
                             or p['end_angle_index'] == 0]
    found = lp.problems(lattice, V, 0.25)
    assert len(found) == 1 and 'no turn from heading(s) [0]' in found[0]


def test_a_stale_radius_or_resolution_is_caught():
    lattice = copy.deepcopy(committed())
    lattice['lattice_metadata']['turning_radius'] = 1.0
    lattice['lattice_metadata']['grid_resolution'] = 0.05
    found = lp.problems(lattice, V, 0.25)
    assert len(found) == 2 and 'turning radius' in found[0] and 'cells' in found[1]


def test_the_config_is_the_planners_radius_on_the_maps_cells():
    cfg = lp.config(V, 0.25)
    assert cfg['motion_model'] == 'ackermann' and cfg['grid_resolution'] == 0.25
    assert math.isclose(cfg['turning_radius'], lp.vehicle.plan_turning_radius(V), abs_tol=1e-3)
    assert (cfg['num_of_headings'], cfg['stopping_threshold']) == (16, 5)
    assert (lp.config(V, 0.25, 32)['num_of_headings'],
            lp.config(V, 0.25, 32)['stopping_threshold']) == (32, 10)


def test_each_file_has_its_own_name():
    assert lp.vehicle.lattice_file_name() == 'lattice_ackermann.json'
    assert lp.vehicle.lattice_file_name(32) == 'lattice_ackermann_32.json'
    with pytest.raises(ValueError):
        lp.vehicle.lattice_file_name(24)
