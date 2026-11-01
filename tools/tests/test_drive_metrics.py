"""drive_metrics: the numbers controller_compare.py reports, on made-up drives."""

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import drive_metrics as m  # noqa: E402
import pytest  # noqa: E402

BODY = (2.0, 1.0, 0.2)      # length, width, rear overhang: vehicle.yaml's body


def straight(speed=1.0, seconds=10.0, rate=10.0, y=0.0, yaw=0.0):
    """Truth samples of a car driving along y at `speed`, heading `yaw` (0: east)."""
    n = int(seconds * rate)
    return [(i / rate, math.cos(yaw) * speed * i / rate, y + math.sin(yaw) * speed * i / rate,
             yaw) for i in range(n + 1)]


def test_distance_to_a_path_and_its_ends():
    path = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    assert m.path_distance((5.0, 2.0), path) == pytest.approx(2.0)
    assert m.path_distance((12.0, 5.0), path) == pytest.approx(2.0)   # beside the second leg
    assert m.path_distance((-3.0, 4.0), path) == pytest.approx(5.0)   # past the start
    assert m.path_distance((1.0, 1.0), [(1.0, 4.0)]) == pytest.approx(3.0)
    assert math.isnan(m.path_distance((0.0, 0.0), []))


def test_off_path_uses_the_newest_plan_at_each_moment():
    truth = straight(seconds=4.0, rate=1.0, y=1.0)          # t = 0..4, 1 m left of y = 0
    plans = [(0.5, [(0.0, 0.0), (10.0, 0.0)]),               # the first plan: y = 0
             (2.5, [(0.0, 1.0), (10.0, 1.0)])]               # a new one, through the car
    # t = 0: no plan yet; t = 1, 2: 1 m off; t = 3, 4: on it.
    assert m.off_path(truth, plans) == pytest.approx([1.0, 1.0, 0.0, 0.0])


def test_signed_speed_distance_stops_and_reversals():
    forward = straight(speed=1.0, seconds=5.0)
    assert m.travelled(forward) == pytest.approx(5.0)
    assert all(v == pytest.approx(1.0) for _, v in m.speeds(forward))
    # Forward 5 s, stand 2 s, back 3 s at 0.5 m/s: one stop, one reversal.
    t_end = forward[-1][0]
    still = [(t_end + i / 10.0, 5.0, 0.0, 0.0) for i in range(1, 21)]
    back = [(t_end + 2.0 + i / 10.0, 5.0 - 0.05 * i, 0.0, 0.0) for i in range(1, 31)]
    drive = forward + still + back
    assert any(v < 0 for _, v in m.speeds(drive))            # reverse: negative, heading east
    assert m.stops(drive) == 1
    assert m.reversals(drive) == 1
    # Standing still at the start or the end isn't a stop.
    waiting = [(i / 10.0, 0.0, 0.0, 0.0) for i in range(20)]
    assert m.stops(waiting + [(t + 2.0, x, y, yaw) for t, x, y, yaw in forward]) == 0
    assert m.stops(forward + still) == 0
    # Shorter than STOP_TIME: not a stop either.
    brief = forward + still[:5] + [(t + 0.5 - 2.0, x, y, yaw) for t, x, y, yaw in back]
    assert m.stops(brief) == 0


def test_the_cars_outline_and_its_gap_to_a_box():
    car = m.footprint(0.0, 0.0, 0.0, BODY)
    assert sorted(car) == sorted([(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)])
    turned = m.footprint(0.0, 0.0, math.pi / 2, BODY)      # facing north: the nose at y 1.8
    assert max(p[1] for p in turned) == pytest.approx(1.8)
    box = m.square(5.0, 0.0, 1.0)                            # x 4.5..5.5
    assert m.polygon_distance(car, box) == pytest.approx(2.7)   # nose at 1.8
    assert m.polygon_distance(m.footprint(3.0, 0.0, 0.0, BODY), box) == 0.0   # overlaps
    beside = m.footprint(4.0, 2.0, 0.0, BODY)                # 1 m left of the box's side
    assert m.polygon_distance(beside, box) == pytest.approx(1.0)
    assert m.clearance(straight(seconds=2.0), (5.0, 0.0, 1.0), BODY) == pytest.approx(0.7)
    assert m.clearance(straight(), None, BODY) is None


def test_a_leg_summary():
    truth = straight(speed=1.0, seconds=10.0)
    leg = {'truth': truth, 'plans': [(0.0, [(0.0, 0.0), (20.0, 0.0)])],
           'steering': [(0.0, 0.0), (1.0, 0.1), (2.0, 0.1)], 'start': 0.0, 'end': 12.0,
           'asked': [(0.02, 0.0), (0.04, 0.3)],
           'goal': (10.0, 0.5, math.radians(10)), 'reached': True, 'cpu': 7.5,
           'obstacle': None, 'body': BODY}
    row = m.summary(leg)
    assert row['reached'] is True
    assert row['time_s'] == pytest.approx(12.0)
    assert row['driven_m'] == pytest.approx(10.0)
    assert row['off_path_mean_m'] == pytest.approx(0.0, abs=1e-9)
    assert row['off_path_max_m'] == pytest.approx(0.0, abs=1e-9)
    assert row['end_error_m'] == pytest.approx(0.5)
    assert row['end_heading_deg'] == pytest.approx(10.0)
    assert row['steering_rms'] == pytest.approx(math.sqrt(0.01 / 2))
    assert (row['stops'], row['reversals'], row['clearance_m']) == (0, 0, None)
    assert row['cpu_percent'] == 7.5
    assert row['moving_after_s'] == pytest.approx(0.1)      # 0.1 m at 1 m/s
    assert row['asked_after_s'] == pytest.approx(0.04)
    assert row['off_route_max_m'] == pytest.approx(0.0, abs=1e-9)
    assert row['route_m'] == pytest.approx(20.0)
    assert row['detour'] == pytest.approx(0.5)
    assert math.isnan(row['lingered_s'])         # no EKF recorded


def test_a_car_that_circles_a_goal_it_reached():
    # TODO 17: the EKF has the car at the goal at t = 9 s, but Nav2 answers at 30 s.
    belief = [(t, t, 0.0, 0.0) for t in range(12)]
    assert m.lingered(belief, (10.0, 0.0, 0.0), end=30.0) == pytest.approx(21.0)
    assert m.lingered(belief, (10.0, 0.0, 0.0), end=9.5) == pytest.approx(0.5)
    assert m.lingered(belief, (50.0, 0.0, 0.0), end=30.0) is None
    assert m.path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)]) == pytest.approx(6.0)


def test_how_long_the_car_took_to_start():
    waiting = [(i / 10.0, 0.0, 0.0, 0.0) for i in range(30)]          # 3 s still
    drive = waiting + [(t + 3.0, x, y, yaw) for t, x, y, yaw in straight(seconds=2.0)]
    assert m.moving_after(drive, start=-1.0) == pytest.approx(4.1)   # sent 1 s before
    assert m.moving_after(waiting, start=0.0) is None
    assert m.moving_after([], start=0.0) is None


def test_off_the_route_sees_what_replanning_hides():
    # The car wanders 3 m left; each new plan starts where it is, so the newest plan
    # stays close, but the first route (y = 0) doesn't.
    truth = [(float(t), float(t), min(t, 3.0), 0.0) for t in range(6)]
    plans = [(t - 0.5, [(t, min(t, 3.0)), (30.0, 0.0)]) for t in range(6)]
    row = m.summary({'truth': truth, 'plans': plans, 'steering': [], 'start': 0.0,
                     'end': 5.0, 'goal': (30.0, 0.0, 0.0), 'reached': False, 'cpu': None,
                     'obstacle': None, 'body': BODY})
    assert row['off_route_max_m'] == pytest.approx(3.0, abs=0.01)
    assert row['off_path_max_m'] < 0.1


def test_thinning_keeps_the_first_and_the_last():
    samples = [(i / 10.0, i) for i in range(26)]                   # 0 .. 2.5 s
    assert [s[0] for s in m.thin(samples, 1.0)] == [0.0, 1.0, 2.0, 2.5]
    assert m.thin([], 1.0) == []


def test_the_table_has_a_column_each_and_leaves_out_numbers_nobody_has():
    row = m.summary({'truth': straight(), 'plans': [], 'steering': [], 'start': 0.0,
                     'end': 10.0, 'goal': (10.0, 0.0, 0.0), 'reached': False, 'cpu': None,
                     'obstacle': None, 'body': BODY})
    other = dict(row, reached=True, cpu_percent=12.0)
    text = m.table([('rpp', row), ('mppi', other)])
    lines = text.splitlines()
    assert lines[:4] == ['|  | rpp | mppi |', '|---|---:|---:|',
                         '| Reached the goal | **no** | yes |', '| Time (s) | 10 | 10 |']
    assert 'Closest to the box' not in text and 'Off the path' not in text
    assert '| follower CPU (%): controller_server, or aav_track | - | 12 |' in lines


def test_the_picture_puts_north_up():
    # Two plans, two lines: one leg's end isn't joined to the next one's start.
    tracks = [('rpp', 'blue', [[(0.0, 0.0), (10.0, 5.0)]], False),
              ('plan', 'grey', [[(0.0, 0.0), (10.0, 0.0)], [(20.0, 0.0), (30.0, 0.0)]], True)]
    blocks = [(20.0, 20.0, 40.0, 35.0, 'grey')]
    text = m.svg((-10.0, -10.0, 70.0, 50.0), tracks, blocks, [(10.0, 5.0, 'goal <1>')])
    root = ET.fromstring(text)                                  # well-formed, names escaped
    ns = '{http://www.w3.org/2000/svg}'
    assert float(root.get('width')) == 640.0                    # 80 m at 8 px/m
    lines = root.findall(ns + 'polyline')
    assert len(lines) == 3
    # Map (0, 0) is 10 m right of the left edge and 50 m below the top: y points up.
    assert lines[0].get('points').split()[:2] == ['80.0,400.0', '160.0,360.0']
    assert lines[1].get('stroke-dasharray') and not lines[0].get('stroke-dasharray')
    building = root.findall(ns + 'rect')[1]                     # after the white page
    assert (building.get('x'), building.get('y')) == ('240.0', '120.0')   # its north-west
    assert [t.text for t in root.findall(ns + 'text')] == ['goal <1>', 'rpp', 'plan', '10 m']


def test_where_the_ekf_put_the_car_against_the_truth():
    truth = [(t * 0.1, t * 0.1, 0.0, 0.0) for t in range(20)]       # east at 1 m/s
    # The EKF: 0.3 m to the side, then 20 degrees off, matched by the nearest time.
    belief = [(0.52, 0.5, 0.3, 0.0), (1.49, 1.5, 0.0, math.radians(20))]
    gaps, turns = m.belief_errors(truth, belief)
    assert gaps == [pytest.approx(0.3), pytest.approx(0.0, abs=1e-9)]
    assert turns == [pytest.approx(0.0), pytest.approx(math.radians(20))]
    assert m.belief_errors([], belief) == ([], [])


def test_a_plan_that_takes_another_route_counts():
    east = [(float(x), 0.0) for x in range(11)]
    shorter = [(float(x), 0.0) for x in range(2, 11)]                # the car moved on
    north = [(2.0, 0.0)] + [(2.0 + x, 4.0) for x in range(9)]        # round the other side
    plans = [(0.0, east), (1.0, shorter), (2.0, north), (3.0, north), (4.0, [])]
    assert m.route_switches(plans) == 1
    assert m.route_switches([]) == 0


# --- A to B ---

def a_to_b_profile(rate=100.0):
    """
    Speeds and commands of a known A-to-B drive, at `rate` Hz.

    Still 1 s; up at 0.5 m/s^2 to 2 m/s (t = 1..5); hold 2 m/s (5..15); down at 1 m/s^2
    (15..17); still 2 s. Asked: 2 m/s from t = 1, 1 m/s from 15, 0 from 16.
    """
    def v(t):
        if t < 1.0:
            return 0.0
        if t < 5.0:
            return 0.5 * (t - 1.0)
        if t < 15.0:
            return 2.0
        return max(0.0, 2.0 - (t - 15.0))

    def asked(t):
        return 0.0 if t < 1.0 else 2.0 if t < 15.0 else 1.0 if t < 16.0 else 0.0
    times = [i / rate for i in range(int(19 * rate) + 1)]
    return [(t, v(t)) for t in times], [(t, asked(t)) for t in times]


def test_the_stop_error_along_and_across_the_line():
    assert m.stop_error((0.0, 0.0), (10.0, 0.0), (10.3, -0.2)) == pytest.approx((0.3, -0.2))
    # A line going north: left of it is west.
    assert m.stop_error((0.0, 0.0), (0.0, 10.0), (-0.5, 9.0)) == pytest.approx((-1.0, 0.5))
    with pytest.raises(ValueError):
        m.stop_error((1.0, 1.0), (1.0, 1.0), (0.0, 0.0))


def test_overshoot_past_b_counts_the_worst_moment():
    a, b = (0.0, 0.0), (10.0, 0.0)
    rolled_back = [(0.0, 0.0, 0.0), (1.0, 10.4, 0.1), (2.0, 10.1, 0.1)]
    assert m.overshoot_past(a, b, rolled_back) == pytest.approx(0.4)
    assert m.overshoot_past(a, b, [(0.0, 0.0, 0.0), (1.0, 9.8, 0.0)]) == 0.0


def test_speeding_up_and_holding():
    speeds, commands = a_to_b_profile()
    # 90 % of 2 m/s is 1.8 m/s, at t = 4.6: 3.6 s after the first command (t = 1).
    assert m.reach_time(speeds, 2.0, start=1.0) == pytest.approx(3.6, abs=0.011)
    assert m.reach_time(speeds, 3.0) is None
    t0, t1 = m.hold_window(speeds, 2.0, commands)
    assert (t0, t1) == (pytest.approx(4.6, abs=0.011), pytest.approx(15.0))
    # Without commands: until the last moment at 1.8 m/s (t = 15.2).
    assert m.hold_window(speeds, 2.0)[1] == pytest.approx(15.2, abs=0.011)
    assert m.overshoot_percent(speeds, 2.0) == 0.0
    assert m.hold_window(speeds, 5.0) is None
    assert math.isnan(m.hold_error(speeds, 5.0, None))


def test_hold_error_and_overshoot_on_a_made_up_hold():
    # 2.1 m/s for 2 s, with one peak of 2.2 m/s: 0.1 m/s fast, 10 % over.
    held = [(i / 10.0, 2.1) for i in range(21)]
    assert m.hold_error(held, 2.0, (0.0, 2.0)) == pytest.approx(0.1)
    assert m.hold_error(held, 2.0, (0.5, 0.5)) == pytest.approx(0.1)    # one sample
    held[10] = (1.0, 2.2)
    assert m.overshoot_percent(held, 2.0) == pytest.approx(10.0)


def test_smoothing_keeps_a_straight_line_and_drops_the_half_windows():
    line = [(i / 100.0, 3.0 * i / 100.0) for i in range(101)]        # 1 s
    out = m.smooth(line, 0.5)
    assert out[0][0] == pytest.approx(0.25) and out[-1][0] == pytest.approx(0.75)
    assert all(v == pytest.approx(3.0 * t) for t, v in out)
    assert m.smooth([], 0.5) == []


def test_peak_deceleration_and_jerk():
    speeds, _ = a_to_b_profile()
    assert m.peak_deceleration(speeds) == pytest.approx(1.0)
    # The steps in acceleration (the biggest: 1 m/s^2 at t = 15 and 17) read as about
    # da / width: 1 / 0.5 s (51 samples of 0.01 s: 1 / 0.51).
    assert m.peak_jerk(speeds) == pytest.approx(1.0 / 0.51, rel=0.02)
    # A constant jerk reads exactly: v = 2 - 0.25 t^2, a = -0.5 t, jerk -0.5 m/s^3.
    curve = [(i / 100.0, 2.0 - 0.25 * (i / 100.0) ** 2) for i in range(201)]
    assert m.peak_jerk(curve) == pytest.approx(0.5)
    assert math.isnan(m.peak_deceleration(speeds[:5]))
    assert math.isnan(m.peak_jerk(speeds[:5]))


def test_the_stop_coasting_and_creeping():
    speeds, commands = a_to_b_profile()
    # Under 0.05 m/s from t = 16.95 on: stopped then.
    assert m.stopped_at(speeds) == pytest.approx(16.96, abs=0.011)
    assert m.stopped_at(speeds[:1000]) is None                       # still driving
    assert m.stopped_at([(0.0, 0.0), (1.0, 0.0)]) is None             # never moved
    # Slowing from the last moment at 1.8 m/s (t = 15.2).
    assert m.slowing_time(speeds, 2.0) == pytest.approx(1.76, abs=0.02)
    assert m.slowing_time(speeds) == pytest.approx(1.76, abs=0.02)   # the top speed: 2
    # The command went to 0 at t = 16: from 1 m/s at 1 m/s^2, 0.5 m.
    assert m.coast_distance(speeds, commands) == pytest.approx(0.5)
    assert math.isnan(m.coast_distance(speeds, []))
    assert math.isnan(m.coast_distance(speeds, commands[:1000]))      # still asking
    # The last metre, from sqrt(2) m/s at 1 m/s^2: sqrt(2) s; under 0.15 m/s for
    # 0.15^2 / 2 m. A stop counted at exactly 0 m/s, so the numbers come out clean.
    seconds, crept = m.final_metre(speeds, still=1e-6)
    assert seconds == pytest.approx(math.sqrt(2.0), abs=0.01)
    assert crept == pytest.approx(0.15 ** 2 / 2.0, abs=1e-3)
    assert all(math.isnan(x) for x in m.final_metre(speeds[:1000]))
    short = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)]                      # 0.5 m in all
    assert all(math.isnan(x) for x in m.final_metre(short))


def test_distance_from_speeds_clips_to_the_times():
    speeds = [(0.0, 1.0), (2.0, 1.0), (4.0, -1.0)]
    assert m.distance(speeds) == pytest.approx(3.0)       # 2 m; then 1 to -1: 0.5 m each way
    assert m.distance(speeds, 1.0, 2.0) == pytest.approx(1.0)


def test_the_table_takes_other_columns():
    cols = [('a', 'A (m)', '{:.1f}'), ('b', 'B', '{:d}')]
    text = m.table([('run', {'a': 1.25})], columns=cols)
    assert text.splitlines() == ['|  | run |', '|---|---:|', '| A (m) | 1.2 |']


def test_lateral_is_left_of_the_line():
    track = [(0.0, 1.0, 2.0), (1.0, 5.0, -1.0)]
    assert m.lateral(track, (0.0, 0.0), (10.0, 0.0)) == pytest.approx([(0.0, 2.0), (1.0, -1.0)])
    # A line going north: east of it is right.
    assert m.lateral([(0.0, 3.0, 4.0)], (0.0, 0.0), (0.0, 1.0)) == pytest.approx([(0.0, -3.0)])
    with pytest.raises(ValueError):
        m.lateral(track, (1.0, 1.0), (1.0, 1.0))


def test_weave_counts_fast_swings_not_slow_drift():
    rate, seconds = 20.0, 60.0
    times = [i / rate for i in range(int(seconds * rate) + 1)]
    drift = [(t, 0.02 * t) for t in times]                         # 1.2 m off, slowly
    assert m.weave(drift) == pytest.approx(0.0, abs=1e-9)
    swing = [(t, 0.3 * math.sin(2.0 * math.pi * t)) for t in times]  # 1 s weave, 0.3 m
    assert m.weave(swing) == pytest.approx(0.3 / math.sqrt(2.0), rel=0.02)
    both = [(t, a + b) for (t, a), (_, b) in zip(drift, swing)]
    assert m.weave(both) == pytest.approx(m.weave(swing), rel=0.01)
    assert math.isnan(m.weave(swing[:10]))


def test_turn_sign_flips_skip_the_deadband():
    turns = [(0, 0.1), (1, 0.01), (2, -0.1), (3, -0.005), (4, -0.2), (5, 0.3)]
    assert m.sign_flips(turns) == 2
    assert m.sign_flips([]) == 0


def test_stop_go_counts_each_stop_that_let_go():
    STOP, GO = 0, 1
    assert m.stop_go([STOP, GO], STOP) == 1
    assert m.stop_go([GO, STOP, GO, STOP, GO, STOP], STOP) == 2
    assert m.stop_go([STOP], STOP) == 0


def test_off_line_fits_a_crooked_straight_line():
    # A straight drive 10 degrees off east, with a 0.2 m bump in the middle.
    a = math.radians(10.0)
    points = [(i * math.cos(a), i * math.sin(a)) for i in range(21)]
    assert max(abs(d) for d in m.off_line(points)) == pytest.approx(0.0, abs=1e-9)
    bumped = [(x - 0.2 * math.sin(a), y + 0.2 * math.cos(a)) if i == 10 else (x, y)
              for i, (x, y) in enumerate(points)]
    offsets = m.off_line(bumped)
    assert offsets[10] - offsets[0] == pytest.approx(0.2, abs=0.01)
    assert m.off_line([(0.0, 0.0)]) == []


def test_rms_and_forward_starts():
    assert m.rms([3.0, -4.0]) == pytest.approx(math.sqrt(12.5))
    assert math.isnan(m.rms([]))
    commands = [(0, 0.0), (1, 0.5), (2, 0.0), (3, -0.3), (4, 0.0), (5, 0.2), (6, 0.4)]
    assert m.starts(commands) == 2       # reversing isn't a forward start


def test_amcl_and_the_trackers_tick_are_scored_when_recorded():
    truth = [(float(t), float(t), 0.0, 0.0) for t in range(11)]
    leg = {'truth': truth, 'plans': [(0.0, [(0.0, 0.0), (10.0, 0.0)])], 'steering': [],
           'asked': [], 'start': 0.0, 'end': 10.0, 'goal': (10.0, 0.0, 0.0),
           'reached': True, 'body': (2.0, 1.0, 0.2),
           'belief': [(float(t), t + 0.2, 0.0, 0.0) for t in range(11)],
           'amcl': [(2.0, 2.0, 0.5, 0.0), (6.0, 6.0, 0.1, math.radians(10))],
           'loops': [(1.0, 0.5, 0.5, 0), (2.0, 1.5, 2.0, 1), (3.0, 1.0, 2.0, 3)]}
    row = m.summary(leg)
    assert row['amcl_samples'] == 2
    assert row['amcl_mean_m'] == pytest.approx(0.3)
    assert row['amcl_max_m'] == pytest.approx(0.5)
    assert row['amcl_heading_max_deg'] == pytest.approx(10.0)
    assert row['belief_mean_m'] == pytest.approx(0.2)
    assert row['loop_ms_mean'] == pytest.approx(1.0)
    assert row['loop_ms_max'] == pytest.approx(2.0)
    assert row['missed_ticks'] == 3
    # Without them, no such numbers: today's table is unchanged.
    plain = m.summary({k: v for k, v in leg.items() if k not in ('amcl', 'loops')})
    assert 'amcl_mean_m' not in plain and 'loop_ms_mean' not in plain


def test_the_grid_holds_each_pose_until_the_next_and_counts_time_lost():
    truth = [(t / 10.0, t / 10.0, 0.0, 0.0) for t in range(101)]      # 1 m/s east, 10 s
    # A pose at 1 s, then nothing until 5 s: held, it falls behind; then right again.
    poses = [(1.0, 1.0, 0.0, 0.0), (5.0, 5.0, 0.0, 0.0), (9.0, 9.0, 0.0, 0.0)]
    grid = m.grid_errors(truth, poses, 0.0, 10.0)
    assert len(grid) == 101 and grid[0][1] is None
    assert grid[30][1] == pytest.approx(2.0)          # at 3 s, the 1 s pose: 2 m behind
    lost, recover = m.lost_time(grid)
    # None for 0-0.9 s (10 samples), over 1 m for 2.1-4.9 s and 6.1-8.9 s (29 each).
    assert lost == pytest.approx(1.0 + 2.9 + 2.9)
    assert recover == pytest.approx(2.9)


def test_ndt_is_scored_with_its_availability_match_time_and_the_ekf_on_the_same_grid():
    truth = [(float(t), float(t), 0.0, 0.0) for t in range(11)]
    leg = {'truth': truth, 'plans': [(0.0, [(0.0, 0.0), (10.0, 0.0)])], 'steering': [],
           'asked': [], 'start': 0.0, 'end': 10.0, 'goal': (10.0, 0.0, 0.0),
           'reached': True, 'body': (2.0, 1.0, 0.2),
           'belief': [(float(t), t + 0.2, 0.0, 0.0) for t in range(11)],
           'ndt': [(float(t), t + 0.01, 0.0, 0.0) for t in range(11)],
           'ndt_status': [(float(t), t != 3, 4.0 + t, 0 if t < 5 else 1) for t in range(10)]}
    row = m.summary(leg)
    assert row['ndt_samples'] == 11
    assert row['ndt_mean_m'] == pytest.approx(0.01)
    assert row['ndt_available'] == pytest.approx(0.9)
    assert row['ndt_match_ms_mean'] == pytest.approx(8.5)
    assert row['ndt_match_ms_max'] == pytest.approx(13.0)
    assert row['ndt_restarts'] == 1
    assert row['ndt_lost_s'] == pytest.approx(0.0)
    assert row['belief_grid_mean_m'] > row['ndt_grid_mean_m']
    assert 'amcl_mean_m' not in row
    assert '90%' in m.table([('pure_pursuit', row)])


def test_runs_combine_into_a_mean_and_a_spread():
    rows = [{'reached': True, 'time_s': 30.0, 'stops': 1, 'off_path_mean_m': 0.1,
             'end_error_m': 0.5, 'clearance_m': math.nan, 'cpu_percent': None},
            {'reached': False, 'time_s': 40.0, 'stops': 2, 'off_path_mean_m': 0.3,
             'end_error_m': 0.7, 'clearance_m': math.nan, 'cpu_percent': None}]
    out = m.combine(rows + [None])
    assert out['reached'] is False and out['runs_reached'] == '1/2'
    assert out['time_s'] == pytest.approx(35.0)
    assert out['time_s_sd'] == pytest.approx(math.sqrt(50.0))
    assert out['off_path_mean_m_sd'] == pytest.approx(math.sqrt(0.02))
    assert isinstance(out['stops'], int)
    assert math.isnan(out['clearance_m']) and out['cpu_percent'] is None
    one = m.combine(rows[:1])
    assert one['runs_reached'] == '1/1' and 'time_s_sd' not in one
    assert m.combine([]) == {}
    assert 'Runs that reached the goal' in m.table([('rpp', out)])


def arc(radius, degrees, points=200):
    """Return a path along a circle of `radius` round (0, radius), from (0, 0), turning left."""
    return [(radius * math.sin(a), radius * (1.0 - math.cos(a)))
            for a in (math.radians(degrees) * i / points for i in range(points + 1))]


def test_resampling_keeps_the_ends_and_the_spacing():
    points = m.resample([(0.0, 0.0), (1.0, 0.0), (1.0, 0.6)], step=0.25)
    assert points[0] == (0.0, 0.0) and points[-1] == (1.0, 0.6)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:-1])]
    # Along the path, not across the corner: the corner's gap is a chord, shorter.
    assert all(g == pytest.approx(0.25) or g < 0.25 for g in gaps)
    assert len(points) == 8      # 1.6 m in 0.25 m steps, the end kept


def test_a_circle_through_three_points():
    assert m.circle_radius((0, 0), (1, 1), (2, 0)) == pytest.approx(1.0)
    assert math.isinf(m.circle_radius((0, 0), (1, 0), (2, 0)))


def test_an_arc_reads_its_own_radius_and_a_grid_corner_reads_tight():
    radii = [r for _, r in m.turn_radii(arc(3.6, 90.0))]
    assert radii and min(radii) == pytest.approx(3.6, rel=0.01)
    corner = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    assert min(r for _, r in m.turn_radii(corner)) < 1.0
    assert all(math.isinf(r) for _, r in m.turn_radii([(0.0, 0.0), (5.0, 0.0)]))


def test_the_planners_numbers_count_what_the_car_cant_drive():
    lock = 2.87
    smooth = m.plan_row({'path': arc(3.6, 90.0), 'ms': 40.0, 'wall_ms': 55.0,
                         'lock_radius': lock})
    assert smooth['plan_found'] is True and smooth['plan_ms'] == 40.0
    assert smooth['plan_m'] == pytest.approx(3.6 * math.pi / 2, rel=0.01)
    assert smooth['plan_too_tight_m'] == 0.0
    sharp = m.plan_row({'path': [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)], 'ms': 3.0,
                        'wall_ms': 9.0, 'lock_radius': lock})
    assert sharp['plan_too_tight_m'] > 0.0 and sharp['plan_tightest_m'] < lock
    none = m.plan_row({'path': [], 'ms': 5000.0, 'wall_ms': 5100.0, 'lock_radius': lock})
    assert none['plan_found'] is False and 'plan_m' not in none
    # In a leg's summary, only when the leg asked the planner alone.
    leg = {'truth': straight(), 'plans': [], 'steering': [], 'start': 0.0, 'end': 10.0,
           'goal': (10.0, 0.0, 0.0), 'reached': True, 'obstacle': None, 'body': BODY}
    assert 'plan_m' not in m.summary(leg)
    assert m.summary(dict(leg, planned={'path': arc(3.6, 90.0), 'ms': 1.0, 'wall_ms': 2.0,
                                        'lock_radius': lock}))['plan_found'] is True


def test_a_whole_tour_in_one_row():
    rows = [{'reached': True, 'time_s': 30.0, 'driven_m': 20.0, 'plan_m': 19.0,
             'plan_ms': 40.0, 'plan_tightest_m': 3.6, 'plan_too_tight_m': 0.0,
             'off_path_mean_m': 0.2},
            {'reached': False, 'time_s': 60.0, 'driven_m': 5.0, 'plan_m': 30.0,
             'plan_ms': 80.0, 'plan_tightest_m': 0.7, 'plan_too_tight_m': 1.5,
             'off_path_mean_m': 0.4},
            None]                                       # a leg never driven
    out = m.overall(rows)
    assert out['legs'] == '1/3'
    assert (out['time_s'], out['driven_m'], out['plan_m']) == (90.0, 25.0, 49.0)
    assert (out['plan_ms'], out['plan_ms_max']) == (60.0, 80.0)
    assert out['plan_tightest_m'] == 0.7 and out['turns_ok'] is False
    assert out['off_path_mean_m'] == pytest.approx(0.3)
    text = m.table([('hybrid', out)], columns=m.OVERALL)
    assert 'Legs finished' in text and '**no**' in text
    assert m.overall([None]) == {'legs': '0/1'}
