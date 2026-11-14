"""
One scorecard for every drive: the same numbers from a sim run or the car's bag.

No ROS here, so the tests run anywhere (tools/tests/test_scorecard.py). The whole-car design's
third shared thing ("one scorecard": docs/designs/2026-11-14_the-whole-car.md). The reader
is tools/score_drive.py.

A drive is what tools/record.sh keeps, on the recording's clock (s):

  poses     [(t, x, y)]             where the car was, map frame: the sim's truth if there,
                                    else the global EKF
  speeds    [(t, v)]                its speed, m/s (/vehicle/odom)
  roads     [(t, Strip)]            the road ahead (aav_description road_ahead.Strip)
  plans     [(t, [(x, y), ...])]    the plan it drove: the point driver's line or /plan
  states    [(t, state, reason)]    the behaviour planner's (DRIVE, APPROACH, ...)
  locks     {name: [(t, on)]}       each lock and the e-stop
  people    [(t, v, w)]             a person's commands (joystick, keyboard, RC)

| Number | Means |
|---|---|
| time, distance, top speed | the drive itself |
| over the limit | seconds faster than the road ahead allowed where the car was (+ `SLACK`) |
| past the end | times the car went beyond the end of the road ahead it had |
| off the plan | mean and worst metres from the plan it drove |
| stops | each STOPPED or WAIT, with why |
| locks | each lock: how often it came on, and for how long |
| a person drove | seconds a person's command was moving the car |
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (os.path.join(ROOT, 'tools'),
             os.path.join(ROOT, 'ros', 'bringup', 'aav_description')):
    if path not in sys.path:
        sys.path.insert(0, path)
from aav_description.road_ahead import along_of, limit_at  # noqa: E402
import drive_metrics as dm  # noqa: E402

SLACK = 0.1         # m/s over the road ahead's limit before it counts
PAST = 0.3          # m beyond the road ahead's end before it counts
PERSON = 0.05       # m/s or rad/s: a person's command this big is driving
PERSON_FRESH = 0.5  # s: a person's command is fresh this long (twist_mux's timeout)


@dataclass
class Drive:
    """What tools/record.sh kept. Each list is oldest first."""

    poses: List[Tuple[float, float, float]] = field(default_factory=list)
    speeds: List[Tuple[float, float]] = field(default_factory=list)
    roads: List[Tuple[float, object]] = field(default_factory=list)
    plans: List[Tuple[float, List[Tuple[float, float]]]] = field(default_factory=list)
    states: List[Tuple[float, str, str]] = field(default_factory=list)
    locks: Dict[str, List[Tuple[float, bool]]] = field(default_factory=dict)
    people: List[Tuple[float, float, float]] = field(default_factory=list)


class Series:
    """Samples (t, ...) oldest first, and the newest at or before a time, by bisection."""

    def __init__(self, samples: Sequence[tuple]):
        self.samples = list(samples)
        self.times = [s[0] for s in self.samples]

    def at(self, t: float) -> Optional[tuple]:
        """Return the newest sample at or before t, or None."""
        i = bisect.bisect_right(self.times, t)
        return self.samples[i - 1] if i else None


def over_limit(drive: Drive) -> Tuple[float, int, float]:
    """
    Return the road ahead's three numbers: seconds over its limit, passes, seconds bare.

    Seconds faster than the road ahead allowed; times the car went beyond its end; seconds
    moving with an empty road ahead.

    An empty strip (no plan: a drive just ended, or none began) allows 0; the car still
    rolling to a stop then is its own number, not "over the limit".
    """
    seconds, past, beyond, bare = 0.0, 0, False, 0.0
    roads, speeds = Series(drive.roads), Series(drive.speeds)
    for (t0, x, y), (t1, _, _) in zip(drive.poses, drive.poses[1:]):
        road = roads.at(t0)
        if road is None:
            continue
        strip = road[1]
        speed = speeds.at(t0)
        moving = speed is not None and abs(speed[1]) > SLACK
        if not len(strip):
            bare += (t1 - t0) if moving else 0.0
            beyond = False
            continue
        if speed is not None and abs(speed[1]) > limit_at(strip, x, y) + SLACK:
            seconds += t1 - t0
        if len(strip):
            s, _ = along_of(strip, x, y)
            now_beyond = s > strip.end + PAST
            if now_beyond and not beyond:
                past += 1
            beyond = now_beyond
    return seconds, past, bare


def off_plan(drive: Drive) -> Tuple[float, float]:
    """Return (mean, worst) metres from the plan in force; (nan, nan) with no plan."""
    gaps, plans = [], Series(drive.plans)
    for t, x, y in drive.poses:
        plan = plans.at(t)
        if plan and len(plan[1]) >= 2:
            gaps.append(dm.path_distance((x, y), plan[1]))
    if not gaps:
        return math.nan, math.nan
    return sum(gaps) / len(gaps), max(gaps)


def stops(drive: Drive) -> List[str]:
    """Return each time the behaviour planner held the car: 'STOPPED: sign', 'WAIT: estop'."""
    out, last = [], None
    for _, state, reason in drive.states:
        if state in ('STOPPED', 'WAIT', 'ASK') and (state, reason) != last:
            out.append(f'{state}: {reason}' if reason else state)
        last = (state, reason)
    return out


def lock_times(samples: Sequence[Tuple[float, bool]], end: float) -> Tuple[int, float]:
    """Return (times it came on, seconds it was on) up to `end`."""
    count, on_since, total, on = 0, None, 0.0, False
    for t, value in samples:
        if value and not on:
            count, on_since = count + 1, t
        elif not value and on and on_since is not None:
            total += t - on_since
        on = value
    if on and on_since is not None:
        total += max(0.0, end - on_since)
    return count, total


def person_seconds(people: Sequence[Tuple[float, float, float]], end: float) -> float:
    """Return the seconds a fresh, moving person's command was there."""
    total = 0.0
    for (t, v, w), nxt in zip(people, list(people[1:]) + [(end, 0.0, 0.0)]):
        if abs(v) > PERSON or abs(w) > PERSON:
            total += max(0.0, min(nxt[0], t + PERSON_FRESH) - t)
    return total


def distance(poses: Sequence[Tuple[float, float, float]]) -> float:
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))


def score(drive: Drive) -> Dict[str, object]:
    """Return the scorecard as numbers and words."""
    start = drive.poses[0][0] if drive.poses else 0.0
    end = drive.poses[-1][0] if drive.poses else 0.0
    over, past, bare = over_limit(drive)
    mean_off, worst_off = off_plan(drive)
    return {
        'time_s': end - start,
        'distance_m': distance(drive.poses),
        'top_speed_mps': max((abs(v) for _, v in drive.speeds), default=0.0),
        'over_limit_s': over,
        'past_end': past,
        'no_road_s': bare,
        'off_plan_mean_m': mean_off,
        'off_plan_worst_m': worst_off,
        'stops': stops(drive),
        'locks': {name: lock_times(samples, end)
                  for name, samples in sorted(drive.locks.items())},
        'person_s': person_seconds(drive.people, end),
        'had_road_ahead': bool(drive.roads),
    }


def table(card: Dict[str, object]) -> str:
    """Return the scorecard as a short table for people."""
    def num(v, unit, digits=2):
        return '-' if isinstance(v, float) and math.isnan(v) else f'{v:.{digits}f} {unit}'
    rows = [
        ('time', num(card['time_s'], 's', 1)),
        ('distance', num(card['distance_m'], 'm', 1)),
        ('top speed', num(card['top_speed_mps'], 'm/s')),
        ("over the road ahead's limit",
         num(card['over_limit_s'], 's', 1) if card['had_road_ahead'] else 'no road ahead'),
        ('moving with no road ahead',
         num(card['no_road_s'], 's', 1) if card['had_road_ahead'] else 'no road ahead'),
        ("past the road ahead's end",
         f"{card['past_end']} times" if card['had_road_ahead'] else 'no road ahead'),
        ('off the plan, mean / worst',
         f"{num(card['off_plan_mean_m'], 'm')} / {num(card['off_plan_worst_m'], 'm')}"),
        ('held', '; '.join(card['stops']) or 'never'),
        ('a person drove', num(card['person_s'], 's', 1)),
    ]
    for name, (count, seconds) in card['locks'].items():
        rows.append((f'lock: {name}', f'on {count} times, {seconds:.1f} s'))
    width = max(len(r[0]) for r in rows)
    return '\n'.join(f'{name:<{width}}  {value}' for name, value in rows)
