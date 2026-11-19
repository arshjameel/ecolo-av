"""
How well the car kept its lane, by the simulator's truth. No ROS: tools/tests/ tests it.

    roads = road_world()                          # the road world's one lane
    roads = load_roads('maps/campus/lanes.yaml')  # the campus's painted roads
    error, road = lane_error(roads, x, y, yaw)    # m, + = left of the lane's centre
    print(score(samples, lanes))                  # the table's numbers
    print(keep_right(lane_map.load(path), track).line())   # kept right? (TODO 148)

Did each run of `measure controllers` keep right (the lanes scorecard: off-centre, in the
lane, over the left line and how often, past the kerb), from its results.json:

    pixi run python3 tools/lane_score.py log/controller-compare/<time>/results.json

A lane's centre:

| Road | Its lane's centre |
|---|---|
| one lane (the road world) | the road's centre line |
| two-way, two lanes | halfway from the centre line to the right edge line, as the car faces |
| one-way, two lanes | the nearer of the two lane centres, the same distance either side |

Units: m, rad, s. The map frame, x east, y north.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass
import math
import sys
from typing import List, Optional, Sequence, Tuple

import yaml

# m: from a painted road's edge to its edge line (tools/campus_world.py's EDGE_INSET).
EDGE_INSET = 0.5


@dataclass(frozen=True)
class Road:
    """One road: its centre line (map frame, m), its width, and how its lanes run."""

    name: str
    points: Tuple[Tuple[float, float], ...]
    width: float
    lanes: int = 2           # 1: the car drives down the middle
    oneway: bool = False


def road_world() -> List[Road]:
    """Return the road world's lane: 3.5 m between the lines, east along y = 0 (road.sdf)."""
    return [Road('road', ((-20.0, 0.0), (100.0, 0.0)), 3.5, lanes=1)]


def load_roads(path: str) -> List[Road]:
    """Return the campus's painted roads from lanes.yaml (tools/campus_world.py)."""
    with open(path) as f:
        data = yaml.safe_load(f)
    return [Road(r.get('name') or r.get('kind', ''),
                 tuple((float(p[0]), float(p[1])) for p in r['points']),
                 float(r['width']), 2, bool(r.get('oneway')))
            for r in data['roads'] if len(r['points']) >= 2]


def nearest(road: Road, x: float, y: float) -> Tuple[float, float, float]:
    """
    Return (distance, signed offset, direction) to the road's centre line.

    The offset is + left of the line as drawn; direction is the segment's heading (rad).
    """
    best = (math.inf, 0.0, 0.0)
    for a, b in zip(road.points, road.points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 < 1e-12:
            continue
        t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2))
        px, py = a[0] + t * dx, a[1] + t * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            side = (dx * (y - a[1]) - dy * (x - a[0])) / math.sqrt(length2)
            best = (d, side, math.atan2(dy, dx))
    return best


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def lane_error(roads: Sequence[Road], x: float, y: float,
               yaw: float) -> Tuple[Optional[float], Optional[Road]]:
    """
    Return the car's offset from its lane's centre (m, + = left of it, the way it faces).

    The road is the nearest whose surface the car is on (within half its width). Off every
    road: (None, None).
    """
    found = None
    for road in roads:
        d, side, direction = nearest(road, x, y)
        if d <= road.width / 2.0 and (found is None or d < found[0]):
            found = (d, side, direction, road)
    if found is None:
        return None, None
    _, side, direction, road = found
    along = math.cos(wrap(yaw - direction)) >= 0.0
    offset = side if along else -side           # + = left of the centre line, as the car faces
    if road.lanes == 1:
        return offset, road
    quarter = (road.width / 2.0 - EDGE_INSET) / 2.0   # between the centre and edge lines
    if road.oneway:
        centre = quarter if offset > 0.0 else -quarter
    else:
        centre = -quarter                       # keep right
    return offset - centre, road


@dataclass(frozen=True)
class Score:
    """A lane drive's numbers."""

    seconds: float
    metres: float
    mean_speed: float        # m/s
    mean_error: float        # m, |lateral error| averaged over time
    rms_error: float
    p95_error: float
    max_error: float
    off_road: float          # s with the car on no road
    lost: float              # fraction of lane messages with no line found
    both: float              # fraction with both lines found

    def line(self) -> str:
        return (f'{self.metres:.0f} m in {self.seconds:.0f} s at {self.mean_speed:.2f} m/s; '
                f'lateral error mean {100 * self.mean_error:.1f} cm, RMS '
                f'{100 * self.rms_error:.1f}, 95 % {100 * self.p95_error:.1f}, max '
                f'{100 * self.max_error:.1f} cm; off the road {self.off_road:.1f} s; lane '
                f'lost in {100 * self.lost:.0f} % of pictures, both lines in '
                f'{100 * self.both:.0f} %')


def score(roads: Sequence[Road], track: Sequence[Tuple[float, float, float, float]],
          lanes: Sequence[Tuple[float, bool, bool]] = ()) -> Score:
    """
    Score a drive.

    track: (t, x, y, yaw) from the truth, in time order. lanes: (t, any line found, both
    lines found) per lane message.
    """
    errors, off, metres = [], 0.0, 0.0
    for (t0, x0, y0, _), (t1, x1, y1, yaw1) in zip(track, track[1:]):
        dt = t1 - t0
        metres += math.hypot(x1 - x0, y1 - y0)
        error, _ = lane_error(roads, x1, y1, yaw1)
        if error is None:
            off += dt
        else:
            errors.append((abs(error), dt))
    seconds = track[-1][0] - track[0][0] if len(track) > 1 else 0.0
    weight = sum(dt for _, dt in errors) or 1.0
    ordered = sorted(e for e, _ in errors)
    n = len(lanes) or 1
    return Score(
        seconds=seconds, metres=metres, mean_speed=metres / seconds if seconds else 0.0,
        mean_error=sum(e * dt for e, dt in errors) / weight,
        rms_error=math.sqrt(sum(e * e * dt for e, dt in errors) / weight),
        p95_error=ordered[int(0.95 * (len(ordered) - 1))] if ordered else 0.0,
        max_error=ordered[-1] if ordered else 0.0, off_road=off,
        lost=sum(not seen for _, seen, _ in lanes) / n,
        both=sum(both for _, _, both in lanes) / n)


def lost_seconds(lanes: Sequence[Tuple[float, bool, bool]]) -> float:
    """Return s with no line found: each lost message counts until the next message."""
    return sum(t1 - t0 for (t0, seen, _), (t1, _, _) in zip(lanes, lanes[1:]) if not seen)


# rad/s: a turn smaller than this, either way, is driving straight: not a flip.
FLIP_DEADBAND = 0.02


@dataclass(frozen=True)
class SteerScore:
    """How the lane follower steered: /drivers/ai/cmd_vel's turn rate (TODO 262)."""

    commands: int
    flips: int               # left to right or back, past FLIP_DEADBAND
    flips_per_minute: float
    rms_turn: float          # rad/s
    rms_change: float        # rad/s per command: how jerky

    def line(self) -> str:
        return (f'steering: {self.flips} flips ({self.flips_per_minute:.1f} a minute), '
                f'turn RMS {self.rms_turn:.3f} rad/s, change RMS {self.rms_change:.4f} '
                f'rad/s a command ({self.commands} commands)')


def steer_score(commands: Sequence[Tuple[float, float]],
                deadband: float = FLIP_DEADBAND) -> Optional[SteerScore]:
    """
    Score the lane follower's steering. commands: (t, angular_z) per command, in order.

    A flip is the turn changing side: right of -deadband after left of +deadband, or back.
    None without two commands.
    """
    if len(commands) < 2:
        return None
    flips, side = 0, 0
    for _, w in commands:
        now = 1 if w > deadband else -1 if w < -deadband else 0
        if now and side and now != side:
            flips += 1
        side = now or side
    minutes = (commands[-1][0] - commands[0][0]) / 60.0
    turns = [w for _, w in commands]
    changes = [b - a for a, b in zip(turns, turns[1:])]
    return SteerScore(
        commands=len(commands), flips=flips,
        flips_per_minute=flips / minutes if minutes > 0 else 0.0,
        rms_turn=math.sqrt(sum(w * w for w in turns) / len(turns)),
        rms_change=math.sqrt(sum(c * c for c in changes) / len(changes)))


@dataclass(frozen=True)
class PoseScore:
    """The global EKF's pose against the truth, split across and along the car (TODO 155)."""

    samples: int
    mean_across: float       # m, |sideways error| averaged
    rms_across: float
    p95_across: float
    max_across: float
    jitter: float            # m, RMS of the sideways error less its own `window` s average
    rms_along: float

    def line(self) -> str:
        return (f'EKF sideways error mean {100 * self.mean_across:.1f} cm, RMS '
                f'{100 * self.rms_across:.1f}, 95 % {100 * self.p95_across:.1f}, max '
                f'{100 * self.max_across:.1f} cm; jitter {100 * self.jitter:.1f} cm RMS; '
                f'along RMS {100 * self.rms_along:.1f} cm ({self.samples} poses)')


def pose_error(track: Sequence[Tuple[float, float, float, float]],
               poses: Sequence[Tuple[float, float, float]],
               window: float = 2.0) -> Optional[PoseScore]:
    """
    Score the filter's poses (t, x, y) against the truth (t, x, y, yaw), both in time order.

    Each pose against the truth nearest in time (within 0.1 s), split into sideways (+ =
    left of the truth, the way it faces) and along. The jitter is the sideways error's
    quick part: what is left after its centred `window` s average, the swing a follower
    sees. None without a pose matched.
    """
    if not track:
        return None
    stamps = [s[0] for s in track]
    errs = []
    for t, x, y in poses:
        i = bisect.bisect_left(stamps, t)
        near = min((j for j in (i - 1, i) if 0 <= j < len(track)),
                   key=lambda j: abs(stamps[j] - t))
        tt, tx, ty, yaw = track[near]
        if abs(tt - t) > 0.1:
            continue
        ex, ey = x - tx, y - ty
        errs.append((t, -ex * math.sin(yaw) + ey * math.cos(yaw),
                     ex * math.cos(yaw) + ey * math.sin(yaw)))
    if not errs:
        return None
    across = [e[1] for e in errs]
    times = [e[0] for e in errs]
    quick = []
    lo = hi = 0
    total = 0.0
    for t, a in zip(times, across):
        while hi < len(errs) and times[hi] <= t + window / 2.0:
            total += across[hi]
            hi += 1
        while times[lo] < t - window / 2.0:
            total -= across[lo]
            lo += 1
        quick.append(a - total / (hi - lo))
    ordered = sorted(abs(a) for a in across)
    n = len(errs)
    return PoseScore(
        samples=n, mean_across=sum(ordered) / n,
        rms_across=math.sqrt(sum(a * a for a in across) / n),
        p95_across=ordered[int(0.95 * (n - 1))], max_across=ordered[-1],
        jitter=math.sqrt(sum(q * q for q in quick) / n),
        rms_along=math.sqrt(sum(e[2] ** 2 for e in errs) / n))


@dataclass(frozen=True)
class KeepRight:
    """Did the car keep right: the lanes scorecard (TODO 148), by `lane_map.py`'s lanes."""

    road_s: float            # s on a road (the rest: a lot, a junction's gap)
    off_centre: float        # m, |offset from the right lane's centre|, mean on a road
    in_lane: float           # share of road time inside the right lane's lines
    two_way_s: float         # s of that on a two-way road
    over_centre: float       # share of two-way road time over the centre line
    crossings: int           # times it went over a two-way road's centre line
    over_kerb: float         # share of road time with base_link past the right kerb

    def line(self) -> str:
        return (f'{self.road_s:.0f} s on a road: {100 * self.off_centre:.0f} cm off the '
                f"right lane's centre, in it {100 * self.in_lane:.0f} %; on two-way roads "
                f'{self.two_way_s:.0f} s, over the centre line {100 * self.over_centre:.0f} % '
                f'({self.crossings} times); past the kerb '
                f'{100 * self.over_kerb:.0f} %')


def keep_right(lanes, track: Sequence[Tuple[float, ...]]) -> KeepRight:
    """
    Score a track (t, x, y, yaw, ...) against a lane map (aav_description lane_map.LaneMap).

    Each sample's lane is lane_at's: the rightmost lane for the way the car faces. Its
    offset (+ left) against that lane's lines and kerbs. On a two-way road the lane's left
    line is the centre line: a crossing is each time the car goes over it. (On a one-way
    road the left line is only the next lane's: in_lane counts it, crossings don't.)
    """
    road_s = two_way_s = 0.0
    offs, in_lane, over, kerb = 0.0, 0.0, 0.0, 0.0
    crossings = 0
    was_over = False
    for (t0, *_), (t1, x, y, yaw, *_) in zip(track, track[1:]):
        dt = t1 - t0
        lane = lanes.lane_at(x, y, yaw)
        if lane is None:
            was_over = False
            continue
        left = (x - lane.x) * -math.sin(lane.heading) + (y - lane.y) * math.cos(lane.heading)
        road_s += dt
        offs += abs(left) * dt
        two_way = any(not r.oneway and r.lane_count() > 1 for r in lanes.roads_at(x, y)
                      if r.name == lane.road)
        if -lane.left_soft <= -left <= lane.right_soft:
            in_lane += dt
        now_over = two_way and left > lane.left_soft
        two_way_s += dt if two_way else 0.0
        if now_over:
            over += dt
            crossings += not was_over
        if -left > lane.right_hard:
            kerb += dt
        was_over = now_over

    def share(v, of=road_s):
        return v / of if of > 0.0 else math.nan
    return KeepRight(road_s=road_s, off_centre=share(offs), in_lane=share(in_lane),
                     two_way_s=two_way_s, over_centre=share(over, two_way_s),
                     crossings=crossings, over_kerb=share(kerb))


@dataclass(frozen=True)
class PassScore:
    """
    How the car passed a parked box in its lane (Ben, 16 Nov: leave the lane only to pass).

    `over` is the car's left side past a two-way road's centre line (base_link's offset
    plus half the car's width): 0 if it never crossed.
    """

    passed: bool             # the car's rear went by the box's far side
    max_over: float          # m: the most the car's left side went over the centre line
    over_s: float            # s with its left side over it
    over_m: float            # m driven with its left side over it
    left_before: Optional[float]   # m before the box's near side it first went over
    back_after: Optional[float]    # m past the box's far side it was back near its lane's centre
    end_off: Optional[float]       # m: |offset| from the lane's centre at the track's end

    def line(self) -> str:
        def m(v):
            return 'never' if v is None else f'{v:.1f} m'
        return (f'passed: {self.passed}; over the centre line by {self.max_over:.2f} m at '
                f'most, for {self.over_s:.1f} s and {self.over_m:.1f} m; left the lane '
                f'{m(self.left_before)} before the box, back {m(self.back_after)} after it; '
                f'at the end {m(self.end_off)} off the lane centre')


def pass_score(lanes, track: Sequence[Tuple[float, ...]], box: Tuple[float, float],
               box_side: float, width: float, front: float, back: float = 0.3) -> PassScore:
    """
    Score passing a square box (centre `box`, `box_side` m) by a track (t, x, y, yaw, ...).

    width: the car's (m); front: base_link (the rear axle) to its front bumper (m). The rear is
    past the box once base_link is past its far side. Back: the first sample past its far side
    within `back` m of the lane's centre. Lanes: an aav_description lane_map.LaneMap.
    """
    max_over = over_s = over_m = 0.0
    left_before = back_after = end_off = None
    passed = False
    bx, by = box
    half = box_side / 2.0
    for (t0, x0, y0, *_), (t1, x, y, yaw, *_) in zip(track, track[1:]):
        lane = lanes.lane_at(x, y, yaw)
        if lane is None:
            continue
        c, s = math.cos(lane.heading), math.sin(lane.heading)
        left = (x - lane.x) * -s + (y - lane.y) * c
        along = (x - bx) * c + (y - by) * s          # base_link past the box's centre
        two_way = any(not r.oneway and r.lane_count() > 1 for r in lanes.roads_at(x, y)
                      if r.name == lane.road)
        over = left + width / 2.0 - lane.left_soft if two_way else -math.inf
        end_off = abs(left)
        if over > 0.0:
            max_over = max(max_over, over)
            over_s += t1 - t0
            over_m += math.hypot(x - x0, y - y0)
            bumper = along + front
            if left_before is None:
                left_before = max(0.0, -half - bumper)
        if along > half:
            passed = True
            if back_after is None and abs(left) <= back:
                back_after = along - half
    return PassScore(passed=passed, max_over=max_over, over_s=over_s, over_m=over_m,
                     left_before=left_before, back_after=back_after, end_off=end_off)


def main(argv=None):
    """Score measure controllers' results.json files by a lane map: did each run keep right."""
    import argparse
    import json
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(here), 'ros', 'bringup', 'aav_description'))
    from aav_description import lane_map, vehicle
    vehicle_yaml = os.path.join(os.path.dirname(here), 'ros', 'bringup', 'aav_description',
                                'config', 'vehicle.yaml')
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('results', nargs='+', help="measure controllers' results.json files")
    parser.add_argument('--lanes', default=os.path.join('maps', 'campus', 'lanes.yaml'),
                        help='the lane map (default: the campus)')
    parser.add_argument('--box', nargs=2, type=float, metavar=('X', 'Y'),
                        help='a parked 1 m box there: how each run passed it (pass_score)')
    args = parser.parse_args(argv)
    lanes = lane_map.load(args.lanes)
    print('| results | follower | run | leg | reached | road s | off centre (m) | in lane | '
          'two-way s | over the centre line | crossings | past the kerb |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for path in args.results:
        with open(path) as f:
            data = json.load(f)
        for row in data:
            if not row.get('track'):
                continue
            k = keep_right(lanes, row['track'])
            reached = 'yes' if (row.get('row') or {}).get('reached') else 'no'
            print(f'| {os.path.basename(os.path.dirname(path))} | {row.get("controller", "?")} | '
                  f'{row.get("run", 1)} | {row.get("leg", "?")} | {reached} | '
                  f'{k.road_s:.0f} | {k.off_centre:.2f} | {100 * k.in_lane:.0f} % | '
                  f'{k.two_way_s:.0f} | {100 * k.over_centre:.0f} % | {k.crossings} | '
                  f'{100 * k.over_kerb:.0f} % |')
            if args.box:
                body = vehicle.load(vehicle_yaml)['vehicle']['body']
                p = pass_score(lanes, row['track'], tuple(args.box), 1.0, body['width'],
                               body['length'] - body['rear_overhang'])
                print(f'|  | run {row.get("run", 1)}, the box | {p.line()} |')


if __name__ == '__main__':
    main()
