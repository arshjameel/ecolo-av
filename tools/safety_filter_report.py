"""
Score the safety filter from a recorded drive.

What it trimmed, how close each barrier came, and whether its stopping law predicts the
car's real stops.

    pixi run tools/ros.sh python3 tools/safety_filter_report.py bags/<date>_<name> [--csv]

Reads /safety/filter/status (one per command: aav_safety_filter_rs) and /vehicle/odom from
a bag. Prints three tables:

| Table | Says |
|---|---|
| per driver | commands, the share trimmed, each rule's count |
| closest calls | the smallest h_speed, h_left, h_right (m): below 0, a barrier was broken |
| stops | each stop the filter called: S(v) it predicted, the metres the car really took |

The stops table is the model check: the step tests' brake steps, recorded with the filter in
shadow, say whether `brake`, `jerk` and `delay` are right. Predicted above measured: the
law is cautious (safe, slower than needed). Below: the law trusts too much room. The sums
are plain functions (tested in tools/tests, no ROS); only read_bag needs ROS.
"""

import argparse
from dataclasses import dataclass, field
import math
import sys

STATUS, WHEEL = '/safety/filter/status', '/vehicle/odom'
# A stop: the filter sends 0 while the car moves at least this fast (m/s); it is over when
# the car is slower than STILL.
MOVING, STILL = 0.3, 0.05


@dataclass
class Row:
    """Hold one status message: one command the filter judged."""

    t: float
    driver: str
    values: dict
    rules: list = field(default_factory=list)

    def get(self, key):
        """Return a number from the status, NaN when missing or not a number."""
        try:
            return float(self.values.get(key, 'nan'))
        except ValueError:
            return math.nan


def summarize(rows):
    """Count per driver: {driver: {'commands', 'trimmed', 'rules': {rule: count}}}."""
    out = {}
    for r in rows:
        d = out.setdefault(r.driver, {'commands': 0, 'trimmed': 0, 'rules': {}})
        d['commands'] += 1
        if r.rules:
            d['trimmed'] += 1
        for rule in r.rules:
            d['rules'][rule] = d['rules'].get(rule, 0) + 1
    return out


def closest(rows):
    """Return the smallest value of each barrier over the drive (m); NaN if never checked."""
    worst = {}
    for key in ('h_speed', 'h_left', 'h_right'):
        seen = [r.get(key) for r in rows if not math.isnan(r.get(key))]
        worst[key] = min(seen) if seen else math.nan
    return worst


def distance_until_still(speeds, t0):
    """
    Return the metres the car rolls from t0 until it is slower than STILL.

    |v| integrated (trapezoid) over (t, v) samples; None if it never gets that slow in the
    recording.
    """
    after = [(t, abs(v)) for t, v in speeds if t >= t0]
    total = 0.0
    for (ta, va), (tb, vb) in zip(after, after[1:]):
        total += 0.5 * (va + vb) * (tb - ta)
        if vb < STILL:
            return total
    return None


def speed_at(speeds, t):
    """Return the newest measured speed at or before t (m/s), NaN with none."""
    best = math.nan
    for ts, v in speeds:
        if ts > t:
            break
        best = v
    return best


def stops(rows, speeds):
    """
    List each stop the filter called: (t, driver, speed, predicted m, measured m or None).

    A stop starts where the filter sends v_out 0 to a driver who asked to move, while the
    car moves at MOVING or more. The prediction is that row's stop_m, S(v).
    """
    found = []
    busy_until = -math.inf
    for r in rows:
        if r.t < busy_until:
            continue
        if not (r.get('v_out') == 0.0 and r.get('v_in') > 0.0):
            continue
        v = speed_at(speeds, r.t)
        if not (abs(v) >= MOVING):
            continue
        measured = distance_until_still(speeds, r.t)
        found.append((r.t, r.driver, v, r.get('stop_m'), measured))
        busy_until = math.inf if measured is None else r.t + 0.1 + _time_to_still(speeds, r.t)
    return found


def _time_to_still(speeds, t0):
    for t, v in speeds:
        if t >= t0 and abs(v) < STILL:
            return t - t0
    return math.inf


def read_bag(path):
    """Return (rows, speeds) from a bag folder (needs ROS)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {STATUS, WHEEL} & set(types)
    if STATUS not in wanted:
        sys.exit(f'{path}: no {STATUS} in the bag: was the safety filter running?')
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    classes = {t: get_message(types[t]) for t in wanted}
    rows, speeds = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg, t = deserialize_message(data, classes[topic]), stamp * 1e-9
        if topic == STATUS:
            values = {kv.key: kv.value for kv in msg.values}
            rules = values.pop('rules', '').split()
            rows.append(Row(t, msg.hardware_id, values, rules))
        else:
            speeds.append((t, msg.twist.twist.linear.x))
    return rows, speeds


def report(rows, speeds):
    """Return the three tables, as markdown lines."""
    lines = ['| Driver | Commands | Trimmed | Rules |', '|---|---|---|---|']
    for driver, d in sorted(summarize(rows).items()):
        share = d['trimmed'] / d['commands'] if d['commands'] else 0.0
        rules = ', '.join(f'{k} {v}' for k, v in sorted(d['rules'].items())) or 'none'
        lines.append(f"| {driver} | {d['commands']} | {share:.0%} | {rules} |")
    worst = closest(rows)
    lines += ['', '| Barrier | Closest (m) |', '|---|---|']
    lines += [f'| {k} | {v:.3f} |' for k, v in worst.items()]
    found = stops(rows, speeds)
    lines += ['',
              '| t (s) | Driver | Speed (m/s) | Predicted S(v) (m) | Measured (m) | Error (m) |',
              '|---|---|---|---|---|---|']
    t0 = rows[0].t if rows else 0.0
    for t, driver, v, pred, meas in found:
        m = 'never stopped' if meas is None else f'{meas:.2f}'
        e = '' if meas is None else f'{pred - meas:+.2f}'
        lines.append(f'| {t - t0:.1f} | {driver} | {v:.2f} | {pred:.2f} | {m} | {e} |')
    if not found:
        lines.append('| | no stops called | | | | |')
    return lines


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    p.add_argument('bag')
    p.add_argument('--csv', action='store_true', help='every status row, one line each')
    a = p.parse_args()
    rows, speeds = read_bag(a.bag)
    if a.csv:
        keys = sorted({k for r in rows for k in r.values})
        print(','.join(['t', 'driver', *keys, 'rules']))
        for r in rows:
            print(','.join([f'{r.t:.3f}', r.driver, *(r.values.get(k, '') for k in keys),
                            ' '.join(r.rules)]))
        return
    print('\n'.join(report(rows, speeds)))


if __name__ == '__main__':
    main()
