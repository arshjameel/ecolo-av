"""lane_score: the car's offset from its lane's centre, and a drive's numbers."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lane_score as s  # noqa: E402, I100

TWO_WAY = s.Road('two way', ((0.0, 0.0), (100.0, 0.0)), 8.0)
ONE_WAY = s.Road('one way', ((0.0, 50.0), (100.0, 50.0)), 8.0, oneway=True)
LANE = (4.0 - s.EDGE_INSET) / 2.0     # m: a lane centre from the centre line, 8 m roads


def test_the_road_worlds_lane_is_its_middle():
    road = s.road_world()
    assert s.lane_error(road, 10.0, 0.3, 0.0)[0] == pytest.approx(0.3)
    # Facing west, left is south.
    assert s.lane_error(road, 10.0, 0.3, math.pi)[0] == pytest.approx(-0.3)
    assert s.lane_error(road, 10.0, 3.0, 0.0) == (None, None)


def test_a_two_way_road_keeps_right_whichever_way_the_car_goes():
    east, road = s.lane_error([TWO_WAY], 50.0, -LANE, 0.0)
    assert road is TWO_WAY and east == pytest.approx(0.0)
    west, _ = s.lane_error([TWO_WAY], 50.0, LANE, math.pi)
    assert west == pytest.approx(0.0)
    # Eastbound in the westbound lane: one lane left of where it should be.
    assert s.lane_error([TWO_WAY], 50.0, LANE, 0.0)[0] == pytest.approx(2 * LANE)


def test_a_one_way_road_has_two_lanes_to_choose_from():
    assert s.lane_error([ONE_WAY], 50.0, 50.0 + LANE + 0.5, 0.0)[0] == pytest.approx(0.5)
    assert s.lane_error([ONE_WAY], 50.0, 50.0 - LANE - 0.5, 0.0)[0] == pytest.approx(-0.5)


def test_the_nearest_road_counts():
    assert s.lane_error([TWO_WAY, ONE_WAY], 50.0, 49.0, 0.0)[1] is ONE_WAY


def test_a_drive_down_the_road_scores_its_error_time_and_lost_pictures():
    track = [(0.1 * k, 0.1 * k, 0.2 if k < 50 else -0.1, 0.0) for k in range(101)]
    lanes = [(0.0, True, True), (1.0, True, False), (2.0, False, False), (3.0, True, True)]
    got = s.score(s.road_world(), track, lanes)
    assert got.seconds == pytest.approx(10.0)
    assert got.mean_speed == pytest.approx(1.0, abs=0.04)    # the one sideways step adds
    assert got.max_error == pytest.approx(0.2)
    assert got.mean_error == pytest.approx((49 * 0.2 + 51 * 0.1) / 100)
    assert got.lost == pytest.approx(0.25) and got.both == pytest.approx(0.5)
    assert got.off_road == 0.0
    assert 'lateral error mean' in got.line()


def test_lost_seconds_count_until_the_next_message():
    lanes = [(0.0, True, True), (1.0, False, False), (1.5, False, False), (3.0, True, True)]
    assert s.lost_seconds(lanes) == pytest.approx(2.0)
    assert s.lost_seconds([]) == 0.0


def test_steer_score_counts_each_change_of_side_past_the_deadband():
    turns = [0.1, 0.05, 0.01, -0.01, 0.015, -0.1, -0.2, 0.0, 0.3, 0.3]
    got = s.steer_score([(6.0 * k, w) for k, w in enumerate(turns)])
    # left, (straight), right, (straight), left: two flips in 54 s.
    assert got.flips == 2 and got.commands == 10
    assert got.flips_per_minute == pytest.approx(2 / 0.9)
    assert got.rms_turn == pytest.approx(math.sqrt(sum(w * w for w in turns) / 10))
    assert 'flips' in got.line()
    assert s.steer_score([(0.0, 0.1)]) is None


def test_lanes_yaml_reads(tmp_path):
    path = tmp_path / 'lanes.yaml'
    path.write_text('roads:\n- {name: Library Road, kind: tertiary, width: 7.0, oneway: false,'
                    ' points: [[0, 0], [10, 0]]}\n- {name: x, kind: tertiary, width: 7,'
                    ' points: [[0, 0]]}\n')
    roads = s.load_roads(str(path))
    assert len(roads) == 1 and roads[0].name == 'Library Road' and roads[0].lanes == 2


def test_pose_error_splits_sideways_and_along_and_finds_the_jitter():
    # The truth drives north (yaw pi/2) at 1 m/s; the pose is 0.2 m west (left) of it
    # and 0.1 m ahead, plus a 5 cm swing every other pose.
    track = [(0.05 * k, 0.0, 0.05 * k, math.pi / 2) for k in range(400)]
    poses = [(0.05 * k, -0.2 - (0.05 if k % 2 else 0.0), 0.05 * k + 0.1)
             for k in range(400)]
    got = s.pose_error(track, poses)
    assert got.samples == 400
    assert got.mean_across == pytest.approx(0.225, abs=1e-6)
    assert got.max_across == pytest.approx(0.25)
    assert got.rms_along == pytest.approx(0.1)
    assert got.jitter == pytest.approx(0.025, abs=1e-3)
    steady = s.pose_error(track, [(t, -0.2, y + 0.1) for t, _, y, _ in track])
    assert steady.jitter == pytest.approx(0.0, abs=1e-9)
    assert s.pose_error(track, [(99.0, 0.0, 0.0)]) is None
    assert 'jitter 2.5 cm' in got.line()


def test_keep_right_counts_the_time_over_the_centre_line_and_each_crossing():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros' / 'bringup'
                           / 'aav_description'))
    from aav_description.lane_map import from_dict
    # Two-way, 7 m: the eastbound lane's centre at y = -1.5, its left line y = 0.
    lanes = from_dict({'roads': [{'name': 'r', 'width': 7.0, 'points': [[0, 0], [100, 0]]}]})
    ys = [-1.5] * 10 + [0.5] * 5 + [-1.5] * 5 + [1.0] * 5 + [-1.5] * 5
    track = [(0.5 * i, 2.0 * i, y, 0.0) for i, y in enumerate(ys)]
    k = s.keep_right(lanes, track)
    assert k.crossings == 2
    assert k.road_s == pytest.approx(0.5 * (len(ys) - 1))
    assert k.over_centre == pytest.approx(10 * 0.5 / k.road_s)
    assert k.in_lane + k.over_centre == pytest.approx(1.0)
    assert k.over_kerb == 0.0
    # Off every road: nothing counts.
    off = s.keep_right(lanes, [(0.0, 0.0, 30.0, 0.0), (1.0, 1.0, 30.0, 0.0)])
    assert off.road_s == 0.0 and math.isnan(off.in_lane)


def test_pass_score_measures_how_far_over_how_long_and_how_soon_back():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros' / 'bringup'
                           / 'aav_description'))
    from aav_description.lane_map import from_dict
    # Two-way, 7 m: the eastbound lane's centre at y = -1.5, the centre line y = 0. A 1 m
    # box at x = 50 on the lane's centre; a 1 m wide car, 1.8 m from base_link to bumper.
    lanes = from_dict({'roads': [{'name': 'r', 'width': 7.0, 'points': [[0, 0], [100, 0]]}]})
    # Out to y = 0.2 (its left side 0.7 m over) from x = 40 to 60, back on the centre at 64.
    track = []
    for i in range(101):
        x = float(i)
        y = 0.2 if 40.0 <= x <= 60.0 else -0.5 if 60.0 < x < 64.0 else -1.5
        track.append((x, x, y, 0.0))
    p = s.pass_score(lanes, track, (50.0, -1.5), 1.0, 1.0, 1.8)
    assert p.passed
    assert p.max_over == pytest.approx(0.7)
    assert p.over_s == pytest.approx(20.0, abs=1.0)
    assert p.left_before == pytest.approx(49.5 - 41.8)
    assert p.back_after == pytest.approx(64.0 - 50.5)
    assert p.end_off == pytest.approx(0.0)
    assert 'passed: True' in p.line()
    # Stopped behind it, never over: not passed, never back.
    stopped = [(float(i), float(i), -1.5, 0.0) for i in range(45)]
    q = s.pass_score(lanes, stopped, (50.0, -1.5), 1.0, 1.0, 1.8)
    assert not q.passed and q.max_over == 0.0 and q.back_after is None
