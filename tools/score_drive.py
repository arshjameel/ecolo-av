"""
Score a recorded drive: the same scorecard for the sim and the car.

    pixi run tools/ros.sh python3 tools/score_drive.py bags/<date>_<name> [--json]

Reads a bag made by tools/record.sh (rosbag2_py and our messages: through tools/ros.sh) and
prints tools/scorecard.py's table: time, distance, top speed, time over the road ahead's
limit and passes beyond its end, metres off the plan, every hold and lock, and how long a
person drove, and the wiggle: metres side to side, and how often ego's steering and the
follower's turn rate (before and after the smoother) changed side. The sums are
scorecard.py (no ROS, tested); this file only reads.
"""

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import scorecard as sc  # noqa: E402

TRUTH, EKF, WHEEL = '/vehicle/sim/truth', '/localization/odom/global', '/vehicle/odom'
ROAD, PLAN, POINT_PLAN = '/world/road_ahead', '/plan', '/drivers/point/plan'
BEHAVIOUR = '/drivers/nav/behaviour/state'
LOCKS = {'/safety/estop': 'estop', '/perception/stop_sign': 'stop_sign',
         '/safety/stop_hold/lock': 'stop_zone', '/safety/geofence/lock': 'geofence',
         '/safety/sensor_watchdog/lock': 'sensors'}
PEOPLE = ('/drivers/joy/cmd_vel', '/drivers/keyboard/cmd_vel', '/drivers/rc/cmd_vel')
# The wiggle's commands: ego's angle; the follower's turn before and after the smoother.
STEERING = '/ego/actuator_cmd'
TURNS = {'/cmd_vel_nav': 'asked', '/drivers/nav/cmd_vel': 'smoothed'}


def read_bag(path):
    """Return a scorecard.Drive from a bag folder (needs ROS)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from aav_description.road_ahead_ros import from_msg

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    pose_topic = TRUTH if TRUTH in types else EKF
    wanted = {pose_topic, WHEEL, ROAD, PLAN, POINT_PLAN, BEHAVIOUR, *LOCKS, *PEOPLE,
              STEERING, *TURNS}
    wanted &= set(types)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    classes = {t: get_message(types[t]) for t in wanted}
    drive = sc.Drive()
    point, nav = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg, t = deserialize_message(data, classes[topic]), stamp * 1e-9
        if topic == pose_topic:
            p = msg.pose.pose.position
            drive.poses.append((t, p.x, p.y))
        elif topic == WHEEL:
            drive.speeds.append((t, msg.twist.twist.linear.x))
        elif topic == ROAD:
            drive.roads.append((t, from_msg(msg)))
        elif topic in (PLAN, POINT_PLAN):
            points = [(s.pose.position.x, s.pose.position.y) for s in msg.poses]
            if topic == PLAN:
                nav = points
            else:
                point = points
            # The plan in force: a drive to a point's line outranks Nav2's (twist_mux).
            drive.plans.append((t, point if len(point) >= 2 else nav))
        elif topic == BEHAVIOUR:
            values = {kv.key: kv.value for kv in msg.values}
            reason = values.get('reason', '')
            drive.states.append((t, values.get('state', ''), '' if reason == '-' else reason))
        elif topic in LOCKS:
            drive.locks.setdefault(LOCKS[topic], []).append((t, bool(msg.data)))
        elif topic in PEOPLE:
            drive.people.append((t, msg.twist.linear.x, msg.twist.angular.z))
        elif topic == STEERING:
            drive.steering.append((t, msg.drive.steering_angle))
        elif topic in TURNS:
            drive.turns.setdefault(TURNS[topic], []).append((t, msg.twist.angular.z))
    drive.people.sort(key=lambda s: s[0])
    return drive, pose_topic


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('bag', help='a bag folder from tools/record.sh')
    parser.add_argument('--json', action='store_true', help='the numbers as JSON')
    args = parser.parse_args(argv)
    drive, pose_topic = read_bag(args.bag)
    card = sc.score(drive)
    if args.json:
        print(json.dumps(card, indent=2, default=str))
    else:
        print(f'{args.bag}  (where the car was: {pose_topic})')
        print(sc.table(card))
    return 0


if __name__ == '__main__':
    sys.exit(main())
