#!/usr/bin/env python3
"""
Check whether what's running is a true drop-in.

Measures each stream with a `drop_in:` rule in docs/interfaces.yaml: its rate on the run's own
clock, its frame, its publishers' QoS. Only listens. Needs a sourced Jazzy install.

usage: tools/drop_in_check.py [--seconds 10] [--measure]
       --measure  print what each sensor topic does, to write a rule from

Ravensim, CARLA and the real sensors must all pass the same rules. Names and types
are `pixi run check <profile>`'s job. Exit status 1 if anything failed.
"""

import argparse
from pathlib import Path
import sys
import time

import drop_in
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
import yaml

ROOT = Path(__file__).resolve().parents[1]
MEASURED = ('/sensors/', '/vehicle/', '/localization/odom/', '/perception/voxels/')
# How long to wait on the wall clock for a stalled simulator, as a share of --seconds.
WALL_LIMIT = 5.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--measure', action='store_true')
    args = ap.parse_args()
    spec = yaml.safe_load((ROOT / 'docs' / 'interfaces.yaml').read_text())
    rules = drop_in.rules(spec)

    rclpy.init()
    node = rclpy.create_node('drop_in_check')
    time.sleep(3.0)     # discovery
    running = dict(node.get_topic_names_and_types())
    if '/clock' in running:
        # Rates on the simulator's clock: a fast or slow run still measures true.
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    names = list(rules)
    if args.measure:
        names = sorted(n for n in running if n.startswith(MEASURED))

    counts = dict.fromkeys(names, 0)
    frames = {name: set() for name in names}
    keep = []
    for name in names:
        types = running.get(name) or [spec['topics'].get(name, {}).get('type')]
        if not types[0]:
            continue

        def heard(message, name=name):
            counts[name] += 1
            header = getattr(message, 'header', None)
            if header is not None:
                frames[name].add(header.frame_id)
        keep.append(node.create_subscription(get_message(types[0]), name, heard,
                                             qos_profile_sensor_data))
    give_up = time.monotonic() + WALL_LIMIT
    while node.get_clock().now().nanoseconds == 0 and time.monotonic() < give_up:
        rclpy.spin_once(node, timeout_sec=0.05)
    for name in counts:
        counts[name] = 0
    start = node.get_clock().now()
    stop_at = time.monotonic() + args.seconds * WALL_LIMIT
    elapsed = 0.0
    while elapsed < args.seconds and time.monotonic() < stop_at:
        rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
    seconds = max(elapsed, 1e-3)

    failed = []
    for name in names:
        hz = counts[name] / seconds
        if args.measure:
            print(f'{name}  {hz:.1f} Hz  frame {sorted(frames[name]) or "-"}')
            continue
        publishers = node.get_publishers_info_by_topic(name)
        reliable = (all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                        for p in publishers) if publishers else None)
        problems = drop_in.judge(name, rules[name], counts[name], seconds, frames[name], reliable)
        failed += problems
        print('\n'.join(f'FAIL  {p}' for p in problems) or f'ok    {name}  {hz:.1f} Hz')
    node.destroy_node()
    rclpy.shutdown()
    if not args.measure:
        print(f'\n{len(failed)} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
