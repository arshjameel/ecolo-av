"""lattice_smooth: the smooth moves fit the car, keep to the steering's rate, and join."""

import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lattice_smooth as ls  # noqa: E402
import pytest  # noqa: E402

CFG, EGO = ls.load_car()
V = CFG['vehicle']
RATE = ls.steering_rate(CFG, EGO)
MOVES = pytest.mark.parametrize('moves', list(ls.SPEEDS))
# The rate measured from the poses (0.25 m apart, chords) against the file's: 3 % slack.
SLACK = 1.03


def committed(moves):
    with open(ls.out_path(moves)) as f:
        return json.load(f)


def test_the_rule_uses_the_tighter_steering_rate_and_the_margin():
    # ego's ramp (0.6 rad/s) is tighter than the motor's (0.8).
    assert RATE == pytest.approx(min(CFG['actuators']['steer_rate'],
                                     EGO['max_steering_rate']))
    # 0.6 / (1.561 * 1.0 * 1.25)
    assert ls.sharpness(0.6, 1.561, 1.0) == pytest.approx(0.3075, abs=1e-4)
    # Twice the speed, half the rate.
    assert ls.sharpness(0.6, 1.561, 2.0) == pytest.approx(ls.sharpness(0.6, 1.561, 1.0) / 2)


def test_a_turn_ends_on_its_heading_and_refuses_a_peak_it_cannot_reach():
    pieces = ls.turn_profile(0.5, 0.2, 0.3)
    s, _, _, yaw = ls.trace(pieces)
    assert yaw[-1] == pytest.approx(0.5, abs=1e-9)
    assert s[-1] == pytest.approx(sum(p[0] for p in pieces))
    with pytest.raises(ValueError):
        ls.turn_profile(0.1, 0.5, 0.3)   # 0.5^2 / 0.3 turns more than 0.1


@MOVES
def test_the_committed_file_fits_vehicle_yaml_ego_yaml_and_the_maps(moves):
    # Fails after a change to the wheelbase, the steering lock or rate, or a map's cells:
    # run pixi run python3 tools/lattice_smooth.py and commit what it writes.
    lattice = committed(moves)
    assert ls.problems(lattice, V, ls.lp.nav2_resolution(), RATE) == []
    meta = lattice['lattice_metadata']
    assert meta['moves'] == moves
    assert meta['design_speed'] == ls.SPEEDS[moves]
    assert meta['num_of_headings'] == 16
    assert meta['number_of_trajectories'] == len(lattice['primitives'])


@MOVES
def test_the_committed_file_is_what_the_generator_makes(moves):
    made = ls.generate(ls.vehicle.plan_turning_radius(V), ls.lp.nav2_resolution(), RATE,
                       V['wheelbase'], ls.SPEEDS[moves], moves=moves)
    lattice = committed(moves)
    assert len(made['primitives']) == len(lattice['primitives'])
    for a, b in zip(made['primitives'], lattice['primitives']):
        assert (a['start_angle_index'], a['end_angle_index']) == (
            b['start_angle_index'], b['end_angle_index'])
        flat = [v for pose in b['poses'] for v in pose]
        assert [v for pose in a['poses'] for v in pose] == pytest.approx(flat, abs=1e-6)


@MOVES
def test_no_move_changes_curvature_faster_than_the_steering_can(moves):
    lattice = committed(moves)
    meta = lattice['lattice_metadata']
    limit = meta['curvature_rate'] * SLACK
    k_max = 1.0 / meta['turning_radius'] * SLACK
    for p in lattice['primitives']:
        start = meta['heading_angles'][p['start_angle_index']]
        ks, rates = ls.curvature_steps((0.0, 0.0), p['poses'], start)
        # The rates include both ends: the move joins the next at curvature 0.
        assert max(rates) <= limit, (p['trajectory_id'], max(rates), limit)
        assert max(abs(k) for k in ks) <= k_max, p['trajectory_id']


def test_the_check_bites_on_nav2s_moves():
    # Nav2's moves go from a straight onto an arc at once: far over the rate.
    with open(ls.lp.out_path()) as f:
        nav2 = json.load(f)
    meta = nav2['lattice_metadata']
    worst = max(max(ls.curvature_steps((0.0, 0.0), p['poses'],
                                       meta['heading_angles'][p['start_angle_index']])[1])
                for p in nav2['primitives'])
    assert worst > 3 * committed('smooth')['lattice_metadata']['curvature_rate']


@MOVES
def test_the_moves_join_on_cells_and_headings(moves):
    lattice = committed(moves)
    meta = lattice['lattice_metadata']
    res = meta['grid_resolution']
    angles = meta['heading_angles']
    assert [p['trajectory_id'] for p in lattice['primitives']] == list(
        range(len(lattice['primitives'])))
    for p in lattice['primitives']:
        x, y, yaw = p['poses'][-1]
        # The end on a cell's corner, as Nav2's lattice needs, facing its end heading.
        assert x / res == pytest.approx(round(x / res), abs=1e-6)
        assert y / res == pytest.approx(round(y / res), abs=1e-6)
        assert math.remainder(yaw - angles[p['end_angle_index']], math.tau) == pytest.approx(
            0.0, abs=1e-9)
        # The poses about a cell apart, never more, as Nav2's own (collision checks).
        steps = [math.dist(a[:2], b[:2]) for a, b in zip([[0, 0, 0]] + p['poses'], p['poses'])]
        assert max(steps) <= res * 1.01
        assert p['trajectory_length'] == pytest.approx(
            p['arc_length'] + p['straight_length'], abs=1e-4)
        assert (p['arc_length'] == 0.0) == (p['start_angle_index'] == p['end_angle_index'])


@MOVES
def test_every_heading_goes_straight_and_turns_both_ways(moves):
    lattice = committed(moves)
    for h in range(16):
        ends = {p['end_angle_index'] for p in lattice['primitives']
                if p['start_angle_index'] == h}
        assert ends == {(h + d) % 16 for d in (0, 1, -1, 2, -2)}, h
    lefts = [p for p in lattice['primitives'] if p['start_angle_index'] != p['end_angle_index']]
    for p in lefts:
        turn = math.remainder(p['end_angle_index'] - p['start_angle_index'], 16)
        assert p['left_turn'] == (turn > 0)
