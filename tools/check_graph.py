#!/usr/bin/env python3
"""
Compare the running ROS graph with docs/interfaces.yaml. Needs a sourced Jazzy install.

usage: tools/check_graph.py --profile turtle|sim|road|spine|car|virtual|replay|bench [--wait 5]

Reports, for every topic in the contract:
  FAIL  a topic marked live in this profile has no publisher, or a type differs
  ok    present with the right type (and who publishes/subscribes)
  -     not running (fine if it isn't live in this profile)
and lists topics that are running but not in the contract (a typo, or the contract
needs an entry). Exit status 1 if anything failed: usable in a test script.

Then, where aav_localization_health_rs runs, one line from /localization/health (Betsi's
warn-only checks): OK, or WARN and the checks that aren't OK. A WARN never fails the
check: those checks are warn-only, as on the car.

road is the sim in its road world (sim.launch.py world:=road), so a topic live in sim must
be live there too, and so must the road world's own: the camera and the radar. spine is
the road world with the spine in the loop (spine:=true): road's topics, plus the CAN ones.

virtual is Ravensim as today's car (sim.launch.py world:=road esp32:=true; docs/wrap-up.md,
step 1): every topic live on the car must be live there too, so the screens and tests see
the same graph. The exceptions are in the contract, each with its reason (not_virtual:).
"""

import argparse
from pathlib import Path
import sys
import time

import rclpy
from std_msgs.msg import String
import yaml

ROOT = Path(__file__).resolve().parents[1]
HEALTH = '/localization/health'
# Every ROS system has these. /diagnostics and the */set_pose topics come with
# robot_localization (the contract lists its set_pose services instead).
IGNORE = {'/parameter_events', '/rosout', '/diagnostics',
          '/ekf_local/set_pose', '/ekf_global/set_pose'}
PROFILES = ('turtle', 'sim', 'road', 'spine', 'car', 'virtual', 'replay', 'bench')
# The profiles each one runs whole, besides its own: the road world is the sim, and more.
INCLUDES = {'road': ('sim',), 'spine': ('sim', 'road'), 'virtual': ('sim', 'road', 'car')}


def must_be_live(topic, profile):
    """Return whether the contract's topic entry must have a publisher in profile."""
    live = set(topic.get('live', []))
    if profile == 'virtual' and topic.get('not_virtual'):
        # On the car only, and why: the contract says (a simulator never sends the car's
        # commands; Ravensim makes the lidar's points, not its packets...).
        live -= {'car'}
    return bool({profile, *INCLUDES.get(profile, ())} & live)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', required=True, choices=PROFILES)
    ap.add_argument('--wait', type=float, default=3.0, help='seconds to let discovery settle')
    ap.add_argument('--spec', default=str(ROOT / 'docs' / 'interfaces.yaml'))
    args = ap.parse_args()
    spec = yaml.safe_load(Path(args.spec).read_text())

    rclpy.init()
    node = rclpy.create_node('check_graph')
    time.sleep(args.wait)
    running = dict(node.get_topic_names_and_types())
    failed = 0
    for name, t in spec['topics'].items():
        live = must_be_live(t, args.profile)
        pubs = sorted({p.node_name for p in node.get_publishers_info_by_topic(name)})
        subs = sorted({s.node_name for s in node.get_subscriptions_info_by_topic(name)}
                      - {'check_graph'})
        types = running.get(name, [])
        if types and t['type'] not in types:
            print(f'FAIL  {name}: type {types}, contract says {t["type"]}')
            failed += 1
        elif live and not pubs:
            print(f'FAIL  {name}: no publisher (must be live in {args.profile})')
            failed += 1
        elif pubs or subs:
            print(f'ok    {name}  pub {pubs or "-"}  sub {subs or "-"}')
        else:
            print(f'-     {name}')
    extra = sorted(set(running) - set(spec['topics']) - IGNORE)
    # Costmap internals, the topics ROS makes inside every action, and turtlesim's own.
    internal = ('/local_costmap/', '/global_costmap/', '/turtle1/')
    extra = [n for n in extra if not n.startswith(internal) and '/_action/' not in n]
    if extra:
        print('\nRunning but not in docs/interfaces.yaml:')
        for n in extra:
            print(f'  {n}  {running[n]}')
    if node.get_publishers_info_by_topic(HEALTH):
        print('\n' + health_line(read_once(node, HEALTH, String, args.wait)))
    node.destroy_node()
    rclpy.shutdown()
    print(f'\n{failed} failed')
    sys.exit(1 if failed else 0)


def read_once(node, name, kind, timeout):
    """Return the data of the first message on name, or None after timeout seconds."""
    got = []
    sub = node.create_subscription(kind, name, lambda m: got.append(m.data), 10)
    end = time.monotonic() + timeout
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got[0] if got else None


def health_line(text):
    """Return the report's line for one /localization/health message (None: none came)."""
    if text is None:
        return f'-     {HEALTH}: nothing heard yet (it reports once a second)'
    if text == 'OK':
        return 'ok    localization health: OK'
    checks = text.removeprefix('WARN ').replace('; ', '\n      ')
    return 'WARN  localization health (warn only, not a failure):\n      ' + checks


if __name__ == '__main__':
    main()
