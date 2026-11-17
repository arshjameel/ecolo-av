#!/usr/bin/env python3
"""
A long lane drive in the simulator, scored by its truth.

How far the car strays from its lane's centre, how often the camera loses the lane, how
fast it goes.

    pixi run measure lane                                   # the road world, on Gazebo
    pixi run measure lane --simulator own                   # the same on our simulator
    pixi run measure lane --world campus --road "Library Road" [--from 20] [--seconds 120]

It starts sim.launch.py headless with lane keeping (lane:=true: lane_finder on the front
left camera, control_interface on the AI slot) and nothing else driving (no Nav2), the car
in its lane at the start of the road (`--from` metres along it; on the campus, a painted
road from maps/campus/lanes.yaml, `pixi run campus`). It records /vehicle/sim/truth and
/perception/lane/geometry for `--seconds` of simulated time from when the car moves, or
until it leaves the road for 3 s, then prints the numbers (tools/lane_score.py) and stops.

It also scores the global EKF against the truth, sideways and along the car, and the
sideways error's jitter (lane_score.pose_error). The EKF doesn't steer here (the lane
follower drives by the camera), so this is the pose alone. With --lane-pose the run adds
lane_pose:=true (TODO 155): the lane finder into ekf_global as a sideways correction.

    pixi run tools/ros.sh --local python3 tools/lane_drive.py --lane-pose   # against without

On its own ROS domain (AAV_SIM_DOMAIN, else a free one), this computer only. The launch
log goes to /tmp/aav-lane-drive.log (AAV_SIM_LOG moves it).
Subscribes: /vehicle/sim/truth, /perception/lane/geometry, /localization/odom/global.
Publishes nothing.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
if 'AAV_SIM_DOMAIN' in os.environ:
    DOMAIN = os.environ['AAV_SIM_DOMAIN']
else:
    from aav_io_mux.domains import free_domain
    DOMAIN = str(free_domain())
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from aav_lane_geometry.msg import LaneGeometry  # noqa: E402
from aav_sim_models.worlds import DEFAULT_SIMULATOR  # noqa: E402
import lane_score  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

LANES = os.path.join(ROOT, 'maps', 'campus', 'lanes.yaml')
OFF_ROAD = 3.0      # s off every road: the drive is over


def start_on(road, along, lane):
    """Return (x, y, yaw): `along` m down the road, `lane` m right of its centre line."""
    gone = 0.0
    for a, b in zip(road.points, road.points[1:]):
        length = math.dist(a, b)
        if gone + length >= along or b == road.points[-1]:
            t = max(0.0, min(1.0, (along - gone) / length))
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            x = a[0] + (b[0] - a[0]) * t + math.sin(yaw) * lane
            y = a[1] + (b[1] - a[1]) * t - math.cos(yaw) * lane
            return x, y, yaw
        gone += length
    raise ValueError(f'{road.name} is shorter than {along} m')


class Recorder(Node):
    """The truth and the lane finder's messages, as they come."""

    def __init__(self):
        super().__init__('lane_drive')
        self.track, self.lanes, self.speeds, self.poses = [], [], [], []
        self.create_subscription(Odometry, '/vehicle/sim/truth', self.on_truth, 50)
        self.create_subscription(Odometry, '/localization/odom/global', self.on_pose, 50)
        self.create_subscription(LaneGeometry, '/perception/lane/geometry', self.on_lane, 10)

    def on_truth(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.track.append((t, p.x, p.y, yaw))
        self.speeds.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))

    def on_pose(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.poses.append((t, m.pose.pose.position.x, m.pose.pose.position.y))

    def on_lane(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        seen = m.left_lane_detected or m.right_lane_detected
        self.lanes.append((t, seen, m.both_lanes_detected))


def main():
    parser = argparse.ArgumentParser(description='A long lane drive, scored by the truth.')
    parser.add_argument('--world', default='road', help='road or campus')
    parser.add_argument('--simulator', default=DEFAULT_SIMULATOR,
                        help=f'gazebo or own (default {DEFAULT_SIMULATOR})')
    parser.add_argument('--road', default='Library Road', help="the campus's painted road")
    parser.add_argument('--from', dest='along', type=float, default=10.0,
                        help='m down the road to start')
    parser.add_argument('--seconds', type=float, default=90.0, help='of driving, simulated')
    parser.add_argument('--speed', default='', help='sim_speed:= (m/s), the top speed')
    parser.add_argument('--offset', type=float, default=0.3,
                        help='m left of the lane centre to start (the road world)')
    parser.add_argument('--lane-pose', action='store_true',
                        help='lane_pose:=true: the lane finder into the global EKF (a trial)')
    args = parser.parse_args()
    if args.world == 'road':
        roads = lane_score.road_world()
        x, y, yaw = args.along - 20.0, args.offset, 0.0
    elif args.world == 'campus':
        roads = lane_score.load_roads(LANES)
        road = next((r for r in roads if r.name == args.road and not r.oneway), None)
        if road is None:
            print(f'no two-way painted road called {args.road} in {LANES}', file=sys.stderr)
            return 2
        lane = (road.width / 2.0 - lane_score.EDGE_INSET) / 2.0
        x, y, yaw = start_on(road, args.along, lane)
    else:
        print('--world: road or campus', file=sys.stderr)
        return 2
    log_path = os.environ.get('AAV_SIM_LOG', '/tmp/aav-lane-drive.log')
    launch_args = [f'world:={args.world}', f'simulator:={args.simulator}', 'gui:=false',
                   'rviz:=false', 'nav:=false', 'lane:=true', 'lidar:=false',
                   f'start_x:={x:.3f}', f'start_y:={y:.3f}', f'start_yaw:={yaw:.4f}']
    if args.speed:
        launch_args.append(f'sim_speed:={args.speed}')
    if args.lane_pose:
        launch_args.append('lane_pose:=true')
    print(f'Lane drive: {args.world} ({args.simulator}), from ({x:.1f}, {y:.1f}) facing '
          f'{math.degrees(yaw):.0f} deg, on ROS domain {DOMAIN}; log {log_path}', flush=True)
    log = open(log_path, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py', *launch_args],
                              cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    rclpy.init()
    rec = Recorder()
    try:
        return drive(rec, roads, args)
    finally:
        rec.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)


def spin_until(rec, done, wall):
    end = time.monotonic() + wall
    while time.monotonic() < end:
        rclpy.spin_once(rec, timeout_sec=0.1)
        if done():
            return True
    return False


def drive(rec, roads, args):
    if not spin_until(rec, lambda: rec.lanes, 240.0):
        print(f'FAIL: no lane message in 240 s (truth: {len(rec.track)} poses)')
        return 1
    if not spin_until(rec, lambda: rec.speeds and rec.speeds[-1] > 0.1, 60.0):
        print(f'FAIL: the car never moved ({len(rec.lanes)} lane messages)')
        return 1
    t0 = rec.track[-1][0]

    def over():
        t = rec.track[-1][0]
        if t - t0 >= args.seconds:
            return True
        recent = [s for s in rec.track if s[0] >= t - OFF_ROAD]
        return (t - t0 > OFF_ROAD and recent and all(
            lane_score.lane_error(roads, x, y, yaw)[0] is None for _, x, y, yaw in recent[::10]))

    spin_until(rec, over, 6.0 * args.seconds)
    track = [s for s in rec.track if s[0] >= t0]
    lanes = [m for m in rec.lanes if m[0] >= t0]
    got = lane_score.score(roads, track, lanes)
    print(f'lane drive ({args.world}, {args.simulator}): {got.line()}')
    pose = lane_score.pose_error(track, [p for p in rec.poses if p[0] >= t0])
    which = 'lane_pose on' if args.lane_pose else 'lane_pose off'
    print(f'global EKF ({which}): {pose.line() if pose else "no pose matched the truth"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
