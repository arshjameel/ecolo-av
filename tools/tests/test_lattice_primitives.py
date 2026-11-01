"""lattice_primitives: the committed primitives still fit the car and Nav2's maps."""

import copy
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lattice_primitives as lp  # noqa: E402

V = lp.vehicle.load(lp.VEHICLE)['vehicle']


def committed():
    with open(lp.OUT) as f:
        return json.load(f)


def test_the_committed_file_fits_vehicle_yaml_and_the_maps():
    # Fails after a change to the wheelbase, the steering lock or a map's cells:
    # run pixi run lattice-primitives and commit what it writes.
    assert lp.problems(committed(), V, lp.nav2_resolution()) == []


def test_no_primitive_turns_tighter_than_the_planners_radius():
    lattice = committed()
    radius = lattice['lattice_metadata']['turning_radius']
    arcs = [p['trajectory_radius'] for p in lattice['primitives'] if p['trajectory_radius']]
    assert arcs and min(arcs) >= radius - lp.TOLERANCE
    # ... and so wider than the car's own full lock, which the follower then has to spare.
    assert min(arcs) > lp.vehicle.min_turning_radius(V)


def test_every_heading_has_moves_forward():
    lattice = committed()
    starts = {p['start_angle_index'] for p in lattice['primitives']}
    assert starts == set(range(lattice['lattice_metadata']['num_of_headings']))


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
