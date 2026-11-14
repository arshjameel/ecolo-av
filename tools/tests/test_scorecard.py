"""The scorecard (tools/scorecard.py): the same numbers for every drive."""

from collections import namedtuple
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import scorecard as sc  # noqa: E402, I100 - scorecard puts aav_description on the path
from aav_description.road_ahead import build  # noqa: E402, I100, I202
from aav_description.stop_law import from_car_model  # noqa: E402

Stop = namedtuple('Stop', 'reason left')
LINE = [(0.0, 0.0), (50.0, 0.0)]


def straight(speed=1.0, seconds=10.0, dt=0.1, y=0.0):
    """Return a car driving along x at `speed`: poses and speeds, one sample every dt."""
    n = int(seconds / dt)
    return ([(i * dt, i * dt * speed, y) for i in range(n + 1)],
            [(i * dt, speed) for i in range(n + 1)])


def test_a_clean_drive_scores_clean():
    poses, speeds = straight()
    strip = build(LINE, (0.0, 0.0), [], from_car_model(), 1.0)
    card = sc.score(sc.Drive(poses=poses, speeds=speeds, roads=[(0.0, strip)],
                             plans=[(0.0, LINE)]))
    assert math.isclose(card['time_s'], 10.0)
    assert math.isclose(card['distance_m'], 10.0, rel_tol=1e-6)
    assert card['over_limit_s'] == 0.0 and card['past_end'] == 0
    assert card['off_plan_worst_m'] < 1e-9
    assert card['person_s'] == 0.0


def test_faster_than_the_road_ahead_allows_counts():
    poses, speeds = straight(speed=1.0)
    # A sign 5 m on: near and past it the strip allows less than 1 m/s.
    strip = build(LINE, (0.0, 0.0), [Stop('sign', 5.0)], from_car_model(), 1.0)
    card = sc.score(sc.Drive(poses=poses, speeds=speeds, roads=[(0.0, strip)]))
    assert card['over_limit_s'] > 4.0      # from about 4.5 m on, and all past the end
    assert card['past_end'] == 1


def test_rolling_on_with_an_empty_road_ahead_is_its_own_number():
    poses, speeds = straight(speed=0.5, seconds=2.0)
    empty = build([], None, [], from_car_model(), 1.0)
    card = sc.score(sc.Drive(poses=poses, speeds=speeds, roads=[(0.0, empty)]))
    assert card['over_limit_s'] == 0.0
    assert math.isclose(card['no_road_s'], 2.0)


def test_off_the_plan_is_measured():
    poses, speeds = straight(y=0.4)
    card = sc.score(sc.Drive(poses=poses, speeds=speeds, plans=[(0.0, LINE)]))
    assert math.isclose(card['off_plan_mean_m'], 0.4)


def test_stops_locks_and_people():
    poses, speeds = straight()
    drive = sc.Drive(
        poses=poses, speeds=speeds,
        states=[(0.0, 'DRIVE', ''), (2.0, 'APPROACH', 'sign'), (3.0, 'STOPPED', 'sign'),
                (3.5, 'WAIT', 'sign'), (3.6, 'WAIT', 'sign'), (5.0, 'GO', 'sign')],
        locks={'stop_sign': [(3.0, True), (5.0, False)], 'estop': [(9.0, True)]},
        people=[(6.0, 0.5, 0.0), (6.2, 0.5, 0.0), (6.4, 0.0, 0.0)])
    card = sc.score(drive)
    assert card['stops'] == ['STOPPED: sign', 'WAIT: sign']
    assert card['locks']['stop_sign'] == (1, 2.0)
    assert card['locks']['estop'][0] == 1 and math.isclose(card['locks']['estop'][1], 1.0)
    assert math.isclose(card['person_s'], 0.4)
    assert 'lock: stop_sign' in sc.table(card)


def test_the_wiggle_is_scored():
    # 20 s along x at 2 m/s, swinging 0.3 m side to side every 2 s; the steering with it.
    times = [i / 20.0 for i in range(401)]
    poses = [(t, 2.0 * t, 0.3 * math.sin(math.pi * t)) for t in times]
    steering = [(t, 0.1 * math.cos(math.pi * t)) for t in times]
    card = sc.score(sc.Drive(poses=poses, speeds=[(t, 2.0) for t in times],
                             steering=steering, turns={'asked': steering}))
    assert math.isclose(card['wiggle']['swing_m'], 0.3 / math.sqrt(2.0), rel_tol=0.1)
    assert math.isclose(card['wiggle']['flips_hz_steering'], 1.0, rel_tol=0.1)
    assert 'flips_hz_asked' in card['wiggle']
    assert 'wiggle: steering changed side' in sc.table(card)
    calm = sc.score(sc.Drive(poses=[(t, 2.0 * t, 0.0) for t in times]))
    assert calm['wiggle']['swing_m'] < 1e-6


def test_an_empty_drive_still_makes_a_table():
    card = sc.score(sc.Drive())
    text = sc.table(card)
    assert 'no road ahead' in text and 'never' in text
