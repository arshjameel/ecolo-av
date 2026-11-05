#!/usr/bin/env python3
"""
A-to-B scorer: how well the car sped up, held its speed and stopped on point B.

Scores Gazebo runs and recorded car runs from the same numbers (docs/a-to-b.md, "Scoring
a run"). Each input is one run: a bag folder (MCAP, from tools/record.sh) or a CSV file.

    pixi run measure atob bags/2026-11-02_point                 B from the point driver's feedback
    pixi run measure atob bags/... --b-latlon 45.38630 -75.69580
    pixi run measure atob bags/... --b-xy 30 5 --speed 1.0
    pixi run measure atob bags/... --taped -0.12 0.05          the tape: 12 cm short, 5 cm left
    pixi run measure atob run1.csv run2.csv --json
    pixi run measure atob bags/... --save-csv run.csv          the run as CSV, for anywhere

Where each number comes from:

  speeds        Gazebo's truth /vehicle/gazebo/truth if recorded, else the wheels
                /vehicle/odom, else the global EKF (twist.linear.x): the rise, hold,
                overshoot, slowing, jerk, coast and creep numbers
  set speed     --speed, else the top speed command: /drivers/point/cmd_vel, else
                /control/cmd_vel (the same topic says when it was asked to go, and when
                the command went to 0). Bags from before 26 Oct have
                /drivers/goto/cmd_vel instead: read too
  stop, EKF     the last pose on /localization/odom/global (map frame)
  stop, truth   the last pose on /vehicle/gazebo/truth (map frame; Gazebo only)
  stop, tape    --taped ALONG ACROSS: measured by hand from the B mark (the car)
  GPS part      real (the tape, else the truth) minus the EKF's view: localization's
                error, not control's
  B             --b-xy / --b-latlon, else the point driver's feedback
                (/drive_to_point/_action/feedback; /goto/_action/feedback in bags from
                before 26 Oct, when the driver was aav_goto)
  A             --a-xy / --a-latlon, else where the point driver lined up (its feedback),
                else the first EKF pose. Only A's direction to B matters to the stop error

What each number means: drive_metrics.py (stop_error ... final_metre) and COLUMNS below.

Bags: the run is the last drive-to-point goal in the bag (from 1 s before its first
feedback to 2 s after its last), unless --start/--end (s from the bag's first message) cut
it. Bag times are when each message was recorded. tools/record.sh records every topic
above, the feedback too (a hidden topic: it passes `--include-hidden-topics`).

CSV (one row per sample, long format; blank where it doesn't apply):

    stream,t,x,y,v,phase,left,cross
    ekf,12.30,4.51,0.02,1.01,,,         map x, y (m); forward speed (m/s)
    truth,12.30,4.50,0.00,1.00,,,
    wheel,12.30,,,1.00,,,               /vehicle/odom: speed only
    cmd,12.30,,,1.00,,,                 the speed command (m/s)
    feedback,12.30,,,,2,15.4,0.03       point_driver's phase, distance left (m), cross track (m)
    b,,30.0,5.0,,,,                     B in the map frame (optional; a too)

Reading a bag needs ROS (rosbag2_py, our messages: through tools/ros.sh); the scoring and
the CSV don't.
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import drive_metrics as dm  # noqa: E402

EKF = '/localization/odom/global'
TRUTH = '/vehicle/gazebo/truth'
WHEEL = '/vehicle/odom'
# The first one recorded. /drivers/goto/cmd_vel: the point driver's topic before 26 Oct, so
# bags recorded before the rename still score.
COMMANDS = ('/drivers/point/cmd_vel', '/drivers/goto/cmd_vel', '/control/cmd_vel')
# Its action's feedback, the new name first; /goto/... in bags from before 26 Oct.
FEEDBACKS = ('/drive_to_point/_action/feedback', '/goto/_action/feedback')
# The old action type's name (GoTo) in those bags: the same fields, read as the new one.
OLD_TYPES = {'aav_interfaces/action/GoTo_FeedbackMessage':
             'aav_interfaces/action/DriveToPoint_FeedbackMessage'}
TURN = 0                # DriveToPoint feedback phase: turning towards B, no line yet
BEFORE, AFTER = 1.0, 2.0    # s kept before a goal's first feedback and after its last
NAVSAT = os.path.join(ROOT, 'ros', 'localization', 'aav_localization', 'config',
                      'navsat.yaml')
CSV_FIELDS = ('stream', 't', 'x', 'y', 'v', 'phase', 'left', 'cross')


def empty_run():
    """
    Return a run with nothing in it.

    Its streams:

    ekf, truth  [(t, x, y, v)]                      map frame, m; forward speed m/s
    wheel, cmd  [(t, v)]                            m/s
    feedback    [(t, phase, left, cross, goal)]     the point driver's: m; goal is the goal's id
    a, b        (x, y) or None                      m, map frame
    """
    return {'ekf': [], 'truth': [], 'wheel': [], 'cmd': [], 'feedback': [],
            'a': None, 'b': None}


# --- Latitude and longitude to the map frame, as navsat_transform does it ---
# A port of aav_hud's map-core earth.rs (its tests check it against GeographicLib):
# lat/lon to UTM in the datum's zone, minus the datum's UTM point, turned by minus the
# meridian convergence at the datum, so the map's x points east and y true north.

WGS84_A, WGS84_F, UTM_K0 = 6378137.0, 1.0 / 298.257223563, 0.9996


def utm(lat, lon, zone):
    """Return (easting m, northing m, convergence rad) of lat/lon (deg) in UTM `zone`."""
    n = WGS84_F / (2.0 - WGS84_F)
    k0a = UTM_K0 * WGS84_A / (1.0 + n) * (1.0 + n * n / 4.0 + n ** 4 / 64.0)
    e = math.sqrt(WGS84_F * (2.0 - WGS84_F))
    alpha = (n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n ** 3 / 16.0,
             13.0 * n * n / 48.0 - 3.0 * n ** 3 / 5.0,
             61.0 * n ** 3 / 240.0)
    phi = math.radians(lat)
    lam = math.radians(lon) - math.radians(zone * 6.0 - 183.0)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi1 = math.atan2(t, math.cos(lam))
    eta1 = math.atanh(math.sin(lam) / math.sqrt(1.0 + t * t))
    xi, eta, sigma, tau = xi1, eta1, 1.0, 0.0
    for j, a in enumerate(alpha, start=1):
        k = 2.0 * j
        xi += a * math.sin(k * xi1) * math.cosh(k * eta1)
        eta += a * math.cos(k * xi1) * math.sinh(k * eta1)
        sigma += k * a * math.cos(k * xi1) * math.cosh(k * eta1)
        tau += k * a * math.sin(k * xi1) * math.sinh(k * eta1)
    txi, theta = math.tan(xi1), math.tanh(eta1)
    gamma = math.atan2(tau + sigma * txi * theta, sigma - tau * txi * theta)
    north = 0.0 if lat >= 0.0 else 10000000.0
    return 500000.0 + k0a * eta, north + k0a * xi, gamma


def utm_zone(lon):
    """Return the UTM zone number of a longitude (deg)."""
    return min(max(int(math.floor((lon + 180.0) / 6.0)), 0), 59) + 1


def to_map(lat, lon, datum):
    """Return map (x east, y north), m, of lat/lon (deg), as navsat_transform: datum (lat, lon)."""
    zone = utm_zone(datum[1])
    e0, n0, turn = utm(datum[0], datum[1], zone)
    e, n, _ = utm(lat, lon, zone)
    dx, dy = e - e0, n - n0
    c, s = math.cos(-turn), math.sin(-turn)
    return c * dx - s * dy, s * dx + c * dy


def load_datum(path=NAVSAT):
    """Return navsat_transform's datum (lat, lon) in degrees, from its parameters file."""
    import yaml
    with open(path, encoding='utf-8') as f:
        params = yaml.safe_load(f)['navsat_transform']['ros__parameters']
    return float(params['datum'][0]), float(params['datum'][1])


# --- B from the point driver's feedback ---

def nearest(samples, t):
    """Return the sample of [(t, ...)] nearest in time to t (samples sorted by time)."""
    times = [s[0] for s in samples]
    i = bisect.bisect_left(times, t)
    return min((samples[j] for j in (i - 1, i) if 0 <= j < len(samples)),
               key=lambda s: abs(s[0] - t))


def goal_from_feedback(ekf, feedback, baseline=1.0):
    """
    Return (A, B), each (x, y) on the map, from the point driver's feedback; None if it can't.

    Each feedback on the line (not while turning) gives the distance left to B along the
    line and the cross track (+ left); with the EKF's pose then, B = p + left u - cross n
    (u along the line, n its left). Two of them, `baseline` m apart or more along the
    line, give the line's direction; B is the mean over all of them. A is where the line
    began (the first one's foot, a few cm short of the true start): only the direction
    A -> B and B matter to the stop error.
    """
    line = [(t, left, cross) for t, phase, left, cross, _ in feedback if phase != TURN]
    if len(line) < 2 or not ekf:
        return None
    poses = [nearest(ekf, t)[1:3] for t, _, _ in line]
    (_, d0, c0), (_, d1, c1) = line[0], line[-1]
    if d0 - d1 < baseline:
        return None
    wx, wy = poses[0][0] - poses[-1][0], poses[0][1] - poses[-1][1]
    heading = math.atan2(wy, wx) - math.atan2(c0 - c1, d1 - d0)
    ux, uy = math.cos(heading), math.sin(heading)
    bs = [(x + d * ux + c * uy, y + d * uy - c * ux)
          for (x, y), (_, d, c) in zip(poses, line)]
    b = (sum(p[0] for p in bs) / len(bs), sum(p[1] for p in bs) / len(bs))
    return (b[0] - d0 * ux, b[1] - d0 * uy), b


# --- Cutting a run ---

def run_times(run):
    return [s[0] for k in ('ekf', 'truth', 'wheel', 'cmd', 'feedback') for s in run[k]]


def cut(run, start=None, end=None):
    """Return the run between start and end, in s from its first sample (None: no limit)."""
    times = run_times(run)
    if not times or (start is None and end is None):
        return run
    zero = min(times)
    lo = -math.inf if start is None else zero + start
    hi = math.inf if end is None else zero + end
    out = dict(run)
    for k in ('ekf', 'truth', 'wheel', 'cmd', 'feedback'):
        out[k] = [s for s in run[k] if lo <= s[0] <= hi]
    return out


def last_goal(run):
    """Return the run around its last goal (BEFORE s before, AFTER s after); as is if none."""
    if not run['feedback']:
        return run
    goal = run['feedback'][-1][4]
    mine = [s for s in run['feedback'] if s[4] == goal]
    lo, hi = mine[0][0] - BEFORE, mine[-1][0] + AFTER
    out = dict(run)
    for k in ('ekf', 'truth', 'wheel', 'cmd'):
        out[k] = [s for s in run[k] if lo <= s[0] <= hi]
    out['feedback'] = mine
    return out


# --- Scoring ---

def score(run, set_speed=None, a=None, b=None, taped=None):
    """
    Return one row of A-to-B numbers for a run (keys: COLUMNS); see the module docstring.

    set_speed m/s (None: the top command); a, b map (x, y) (None: from the run); taped
    (along, across) m, from the tape, as drive_metrics.stop_error signs them.
    """
    ekf, truth, cmd = run['ekf'], run['truth'], run['cmd']
    found = goal_from_feedback(ekf, run['feedback'])
    if b is not None:
        b_from = 'given'
    elif run.get('b') is not None:
        b, b_from = run['b'], 'the CSV'
    elif found is not None:
        b, b_from = found[1], "the point driver's feedback"
    else:
        raise ValueError('no B: give --b-xy or --b-latlon, or record '
                         '/drive_to_point/_action/feedback')
    if a is None:
        a = run.get('a') or (found[0] if found else None)
    if a is None:
        first = (ekf or truth or [None])[0]
        if first is None:
            raise ValueError('no A, and no pose to start from')
        a = first[1:3]
    if truth:
        speeds, speed_from = [(t, v) for t, _, _, v in truth], 'truth'
    elif run['wheel']:
        speeds, speed_from = run['wheel'], 'wheels'
    else:
        speeds, speed_from = [(t, v) for t, _, _, v in ekf], 'EKF'
    if set_speed is None and cmd:
        set_speed = max(v for _, v in cmd)
    asked = next((t for t, v in cmd if v >= dm.STILL), None)
    if asked is None:
        asked = next((t for t, v in speeds if v >= dm.STILL), None)
    window = dm.hold_window(speeds, set_speed, cmd) if set_speed else None
    seconds, crept = dm.final_metre(speeds)
    row = {
        'b_from': b_from,
        'speed_from': speed_from,
        'line_m': math.hypot(b[0] - a[0], b[1] - a[1]),
        'set_speed': set_speed,
        'top_speed': max(v for _, v in speeds) if speeds else math.nan,
        'reach_s': dm.reach_time(speeds, set_speed, asked) if set_speed else None,
        'hold_error': dm.hold_error(speeds, set_speed, window) if set_speed else math.nan,
        'overshoot_pct': dm.overshoot_percent(speeds, set_speed) if set_speed else math.nan,
        'decel_peak': dm.peak_deceleration(speeds),
        'jerk_peak': dm.peak_jerk(speeds),
        'slowing_s': dm.slowing_time(speeds, set_speed),
        'coast_m': dm.coast_distance(speeds, cmd),
        'final_metre_s': seconds,
        'creep_m': crept,
        'a': tuple(a),
        'b': tuple(b),
    }
    views = {}
    for name, track in (('ekf', ekf), ('truth', truth)):
        if track:
            views[name] = dm.stop_error(a, b, track[-1][1:3])
            row[name + '_along'], row[name + '_across'] = views[name]
            row[name + '_past'] = dm.overshoot_past(a, b, [s[:3] for s in track])
    if taped is not None:
        views['taped'] = tuple(taped)
        row['taped_along'], row['taped_across'] = taped
    real = views.get('taped', views.get('truth'))
    if real is not None and 'ekf' in views:
        row['gps_along'] = real[0] - views['ekf'][0]
        row['gps_across'] = real[1] - views['ekf'][1]
    return row


# (key, what it is, format): the table's rows, in order. "Real" is the tape, else the truth.
COLUMNS = [
    ('b_from', 'B from', '{}'),
    ('speed_from', 'Speeds from', '{}'),
    ('line_m', 'A to B (m)', '{:.1f}'),
    ('set_speed', 'Set speed (m/s)', '{:.2f}'),
    ('top_speed', 'Top speed (m/s)', '{:.2f}'),
    ('reach_s', 'Asked to go, to 90 % of the set speed (s)', '{:.1f}'),
    ('hold_error', 'Holding: speed error, mean (m/s, + fast)', '{:+.3f}'),
    ('overshoot_pct', 'Speed overshoot (%)', '{:.1f}'),
    ('decel_peak', 'Hardest slowing (m/s^2)', '{:.2f}'),
    ('jerk_peak', 'Largest jerk (m/s^3)', '{:.2f}'),
    ('slowing_s', 'First slowing to stopped (s)', '{:.1f}'),
    ('coast_m', 'Rolled after the command hit 0 (m)', '{:.2f}'),
    ('final_metre_s', 'The last metre took (s)', '{:.1f}'),
    ('creep_m', 'Crept in the last metre, under 0.15 m/s (m)', '{:.2f}'),
    ('ekf_along', 'Stop by the EKF: past B (m, - short)', '{:+.2f}'),
    ('ekf_across', 'Stop by the EKF: left of the line (m, - right)', '{:+.2f}'),
    ('ekf_past', 'By the EKF: furthest past B (m)', '{:.2f}'),
    ('truth_along', 'Stop by the truth: past B (m)', '{:+.2f}'),
    ('truth_across', 'Stop by the truth: left of the line (m)', '{:+.2f}'),
    ('truth_past', 'By the truth: furthest past B (m)', '{:.2f}'),
    ('taped_along', 'Stop by the tape: past B (m)', '{:+.2f}'),
    ('taped_across', 'Stop by the tape: left of the line (m)', '{:+.2f}'),
    ('gps_along', 'GPS part, along: real - EKF (m)', '{:+.2f}'),
    ('gps_across', 'GPS part, across: real - EKF (m)', '{:+.2f}'),
]


def plain(value):
    """Return value for JSON: NaN as None, tuples as lists."""
    if isinstance(value, float) and math.isnan(value):
        return None
    if isinstance(value, tuple):
        return [plain(v) for v in value]
    return value


# --- Reading and writing runs ---

def number(text):
    return float(text) if text not in ('', None) else None


def read_csv(path):
    """Return a run from a CSV file (the module docstring has its columns)."""
    run = empty_run()
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            kind = row['stream'].strip()
            t, x, y, v = (number(row.get(k)) for k in ('t', 'x', 'y', 'v'))
            if kind in ('ekf', 'truth'):
                run[kind].append((t, x, y, v if v is not None else math.nan))
            elif kind in ('wheel', 'cmd'):
                run[kind].append((t, v))
            elif kind == 'feedback':
                run[kind].append((t, int(float(row['phase'])), number(row['left']),
                                  number(row['cross']), row.get('goal', '') or ''))
            elif kind in ('a', 'b'):
                run[kind] = (x, y)
            else:
                raise ValueError(f'{path}: unknown stream {kind!r}')
    for k in ('ekf', 'truth', 'wheel', 'cmd', 'feedback'):
        run[k].sort(key=lambda s: s[0])
    return run


def write_csv(run, path):
    """Write a run as CSV (read_csv reads it back)."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        out = csv.writer(f)
        out.writerow(CSV_FIELDS)
        for kind in ('ekf', 'truth'):
            out.writerows([kind, t, x, y, v, '', '', ''] for t, x, y, v in run[kind])
        for kind in ('wheel', 'cmd'):
            out.writerows([kind, t, '', '', v, '', '', ''] for t, v in run[kind])
        out.writerows(['feedback', t, '', '', '', p, d, c] for t, p, d, c, _ in run['feedback'])
        for kind in ('a', 'b'):
            if run.get(kind) is not None:
                out.writerow([kind, '', run[kind][0], run[kind][1], '', '', '', ''])


def bag_topics(types):
    """
    Return (command, feedback, wanted) from a bag's {topic: type}: the topics to read.

    command and feedback are the topic names found (None if not recorded), the new names
    first, then the ones from before 26 Oct. wanted is {topic: the type to read it as}.
    """
    command = next((c for c in COMMANDS if c in types), None)
    feedback = next((f for f in FEEDBACKS if f in types), None)
    wanted = {t: OLD_TYPES.get(types[t], types[t])
              for t in (EKF, TRUTH, WHEEL, command, feedback) if t in types}
    return command, feedback, wanted


def read_bag(path):
    """
    Return a run from a bag folder (needs ROS: rosbag2_py, and our messages built).

    Times are when each message was recorded (s). The topics: the module docstring.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    command, feedback, wanted = bag_topics(types)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    classes = {t: get_message(kind) for t, kind in wanted.items()}
    run = empty_run()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg, t = deserialize_message(data, classes[topic]), stamp * 1e-9
        if topic in (EKF, TRUTH):
            p = msg.pose.pose.position
            run['ekf' if topic == EKF else 'truth'].append(
                (t, p.x, p.y, msg.twist.twist.linear.x))
        elif topic == WHEEL:
            run['wheel'].append((t, msg.twist.twist.linear.x))
        elif topic == command:
            run['cmd'].append((t, getattr(msg, 'twist', msg).linear.x))
        elif topic == feedback:
            f = msg.feedback
            run['feedback'].append((t, int(f.phase), float(f.distance_left),
                                    float(f.cross_track), bytes(msg.goal_id.uuid).hex()))
    for k in ('ekf', 'truth', 'wheel', 'cmd', 'feedback'):
        run[k].sort(key=lambda s: s[0])
    return run


def read_run(path):
    """Return a run from a CSV file or a bag folder."""
    if os.path.isdir(path):
        return read_bag(path)
    return read_csv(path)


# --- The command line ---

def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='bag folders (MCAP) or CSV files, one run each')
    for point in ('a', 'b'):
        group = ap.add_mutually_exclusive_group()
        group.add_argument(f'--{point}-xy', nargs=2, type=float, metavar=('X', 'Y'),
                           help=f'{point.upper()} on the map, m (x east, y north)')
        group.add_argument(f'--{point}-latlon', nargs=2, type=float, metavar=('LAT', 'LON'),
                           help=f'{point.upper()} as a GPS point, degrees')
    ap.add_argument('--datum', nargs=2, type=float, metavar=('LAT', 'LON'),
                    help="the map's origin for --*-latlon (default: navsat.yaml's datum)")
    ap.add_argument('--speed', type=float, help='the set speed, m/s (default: the top command)')
    ap.add_argument('--taped', nargs=2, type=float, action='append', metavar=('ALONG', 'ACROSS'),
                    help='the stop error by tape, m: + past B / - short; + left / - right. '
                         'Once per run, in order')
    ap.add_argument('--start', type=float, help='s from the recording start (default: '
                    'the last drive-to-point goal)')
    ap.add_argument('--end', type=float, help='s from the recording start')
    ap.add_argument('--json', action='store_true', help='print JSON, not a table')
    ap.add_argument('--save-csv', metavar='FILE', help='write the (cut) run as CSV; one run')
    a = ap.parse_args(argv)
    if a.taped and len(a.taped) != len(a.runs):
        ap.error(f'--taped given {len(a.taped)} times for {len(a.runs)} runs')
    if a.save_csv and len(a.runs) != 1:
        ap.error('--save-csv takes one run')
    return a


def point(xy, latlon, datum):
    """Return a map point from --*-xy or --*-latlon (None if neither)."""
    if xy:
        return tuple(xy)
    if latlon:
        return to_map(latlon[0], latlon[1], datum)
    return None


def main(argv=None):
    a = parse(sys.argv[1:] if argv is None else argv)
    datum = tuple(a.datum) if a.datum else (
        load_datum() if (a.a_latlon or a.b_latlon) else None)
    pa, pb = point(a.a_xy, a.a_latlon, datum), point(a.b_xy, a.b_latlon, datum)
    rows = []
    for i, path in enumerate(a.runs):
        run = read_run(path)
        run = cut(run, a.start, a.end) if (a.start is not None or a.end is not None) \
            else last_goal(run)
        if a.save_csv:
            write_csv(run, a.save_csv)
        taped = a.taped[i] if a.taped else None
        try:
            rows.append((os.path.basename(os.path.normpath(path)),
                         score(run, a.speed, pa, pb, taped)))
        except ValueError as e:
            print(f'{path}: {e}', file=sys.stderr)
            return 2
    if a.json:
        print(json.dumps([{'run': name, **{k: plain(v) for k, v in row.items()}}
                          for name, row in rows], indent=2))
    else:
        print(dm.table(rows, columns=COLUMNS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
