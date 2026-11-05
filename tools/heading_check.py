#!/usr/bin/env python3
"""
Heading check: how far off the global EKF's heading is at walking pace, forward and back.

Run it through tools/ros.sh --local (no pixi task: it's rare):
pixi run tools/ros.sh --local python3 tools/heading_check.py [options]

    (no options)        one run, about 4 minutes, this computer only
    --runs 3            three runs, one after the other
    --gyro-bias 0.6     the simulated gyro reads 0.6 deg/s too much
    --gps-every 5       one GPS fix a second, the receiver's factory rate
    --wrong-start 30    the global EKF told the car faces 30 deg off, just before the
                        drive: how soon is it right?
    -- gps_heading:=false   anything after -- goes to sim.launch.py

TODO 13. Starts Gazebo's empty world headless (no Nav2, lidar or radar: a light run) on
a private ROS domain and Gazebo partition, drives a fixed walking-pace profile through
twist_mux's scripted slot (PROFILE: forward, reversing, gentle turns both ways), and
compares the global EKF's heading (/localization/odom/global, what Nav2 steers by) with
Gazebo's truth (/vehicle/gazebo/truth). It prints, for forward and for reversing, the
median, 95th percentile and worst heading error, in degrees, and a JSON line to collect.

GPS noise is random, so one run proves little: judge over several.
"""

import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MOVING = 0.2      # m/s: slower than this counts as neither forward nor reversing
# (seconds, m/s, rad/s): the drive. Walking pace (0.5 m/s, reversing's cap), gentle turns
# (0.1 rad/s at 0.5 m/s is a 5 m radius), and one faster stretch forward.
PROFILE = [
    (3.0, 0.0, 0.0),
    (12.0, 0.5, 0.0), (10.0, 0.5, 0.1), (10.0, 0.5, -0.1),
    (3.0, 0.0, 0.0),
    (12.0, -0.5, 0.0), (10.0, -0.5, 0.1), (10.0, -0.5, -0.1),
    (3.0, 0.0, 0.0),
    (15.0, 1.0, 0.05), (10.0, 0.5, -0.1),
    (3.0, 0.0, 0.0),
    (15.0, -0.5, -0.05), (10.0, -0.5, 0.1),
    (3.0, 0.0, 0.0),
]


def angle_diff(a, b):
    """Return a - b, wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values, share):
    """Return the value below which `share` (0..1) of the values lie (nearest rank)."""
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(share * len(ordered)) - 1))]


def summarise(samples, moving=MOVING):
    """
    Return the heading errors' median, 95th percentile and worst (deg), forward and reversing.

    samples: (speed m/s, believed yaw rad, true yaw rad), one per EKF message. Speeds under
    `moving` either way are left out: standing still says nothing about the heading.
    """
    out = {}
    for name, keep in (('forward', lambda v: v >= moving), ('reversing', lambda v: v <= -moving)):
        errors = [abs(math.degrees(angle_diff(b, t))) for v, b, t in samples if keep(v)]
        out[name] = ({'n': len(errors), 'median': statistics.median(errors),
                      'p95': percentile(errors, 0.95), 'worst': max(errors)}
                     if errors else {'n': 0})
    return out


def command_at(t, profile=PROFILE):
    """Return (m/s, rad/s) the profile asks for t s after it starts; None once it's done."""
    for seconds, speed, turn in profile:
        if t < seconds:
            return speed, turn
        t -= seconds
    return None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def recovered_after(track, within=5.0):
    """Return the time (s) from the first sample until the heading error is under `within` deg."""
    for t, err in track:
        if err < within:
            return t - track[0][0]
    return math.inf


def run_once(domain, launch_args, gyro_bias, log_path, wrong_start=0.0, gps_every=1):
    """Start Gazebo, drive the profile twice, stop it; return summarise()'s numbers or None."""
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from robot_localization.srv import SetPose
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from sensor_msgs.msg import Imu

    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               GZ_IP='127.0.0.1', GZ_PARTITION=f'heading_check_{domain}')
    os.environ.update(env)
    log = open(log_path, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'aav_bringup', 'sim.launch.py', 'world:=empty', 'gui:=false',
         'rviz:=false', 'nav:=false', 'lidar:=false', 'radar:=false', *launch_args],
        cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('heading_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    truth, samples, course, track = {}, [], [], []
    # AAV_HEADING_DUMP=file: every sample as "t speed believed true" (rad), to look at.
    dump = open(os.environ['AAV_HEADING_DUMP'], 'w') if 'AAV_HEADING_DUMP' in os.environ else None

    def on_truth(msg):
        truth['yaw'] = yaw_of(msg.pose.pose.orientation)
        truth['v'] = msg.twist.twist.linear.x
        truth['xy'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_global(msg):
        if 'yaw' in truth and recording[0]:
            samples.append((truth['v'], yaw_of(msg.pose.pose.orientation), truth['yaw']))
            if dump is not None:
                dump.write(f"{now():.2f} {truth['v']:.2f} {samples[-1][1]:.4f} "
                           f"{truth['yaw']:.4f}\n")
            track.append((now(), abs(math.degrees(angle_diff(samples[-1][1], truth['yaw'])))))

    def on_course(msg):
        if 'yaw' in truth and recording[0]:
            course.append((truth['v'], yaw_of(msg.orientation), truth['yaw']))

    recording = [False]
    node.create_subscription(Imu, '/localization/gps/heading', on_course, 10)
    node.create_subscription(Odometry, '/vehicle/gazebo/truth', on_truth, 10)
    node.create_subscription(Odometry, '/localization/odom/global', on_global, 10)
    cmd = node.create_publisher(TwistStamped, '/drivers/scripted/cmd_vel', 10)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin_until(test, wall):
        end = time.monotonic() + wall
        while time.monotonic() < end and not test():
            rclpy.spin_once(node, timeout_sec=0.05)
        return test()

    try:
        got = {}
        node.create_subscription(Odometry, '/localization/odom/global',
                                 lambda m: got.setdefault('global', True), 10)
        if not spin_until(lambda: 'yaw' in truth and 'global' in got, 120.0):
            print(f'  nothing from Gazebo or the global EKF in 120 s: see {log_path}')
            return None
        spin_until(lambda: False, 10.0)          # let ego and the filters settle
        if gyro_bias:
            client = AsyncParameterClient(node, 'gazebo_imu')
            client.wait_for_services(10.0)
            future = client.set_parameters([Parameter('gyro_bias', value=math.radians(gyro_bias))])
            spin_until(future.done, 10.0)
        if gps_every > 1:
            client = AsyncParameterClient(node, 'gazebo_gps')
            client.wait_for_services(10.0)
            future = client.set_parameters([Parameter('every', value=gps_every)])
            spin_until(future.done, 10.0)
        if wrong_start:
            # Where the global EKF has the car, but facing wrong_start degrees off.
            setter = node.create_client(SetPose, 'ekf_global/set_pose')
            setter.wait_for_service(timeout_sec=10.0)
            req = SetPose.Request(pose=PoseWithCovarianceStamped())
            req.pose.header.frame_id = 'map'
            req.pose.header.stamp = node.get_clock().now().to_msg()
            req.pose.pose.pose.position.x, req.pose.pose.pose.position.y = truth['xy']
            yaw = truth['yaw'] + math.radians(wrong_start)
            req.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
            for i, var in ((0, 0.25), (7, 0.25), (35, 0.1)):    # as initial_estimate_covariance
                req.pose.pose.covariance[i] = var
            future = setter.call_async(req)
            spin_until(future.done, 10.0)
            spin_until(lambda: False, 0.5)
        recording[0] = True
        for _ in range(2):
            start = now()
            while True:
                asked = command_at(now() - start)
                if asked is None:
                    break
                msg = TwistStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x, msg.twist.angular.z = asked
                cmd.publish(msg)
                spin_until(lambda: False, 0.05)
        out = summarise(samples)
        if wrong_start:
            out['recovered_s'] = recovered_after(track)
        if course:      # gps_heading's own headings, before the EKF: what it feeds in
            out['gps_heading'] = summarise(course)
        return out
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, wait in ((signal.SIGINT, 30.0), (signal.SIGKILL, 10.0)):
            try:
                os.killpg(launch.pid, sig)
            except (ProcessLookupError, PermissionError):
                break
            try:
                launch.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        log.close()
        if dump is not None:
            dump.close()


def main():
    args = sys.argv[1:]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    launch_args = []
    if '--' in args:
        launch_args = args[args.index('--') + 1:]
        args = args[:args.index('--')]
    runs = int(args[args.index('--runs') + 1]) if '--runs' in args else 1
    bias = float(args[args.index('--gyro-bias') + 1]) if '--gyro-bias' in args else 0.0
    wrong = float(args[args.index('--wrong-start') + 1]) if '--wrong-start' in args else 0.0
    every = int(args[args.index('--gps-every') + 1]) if '--gps-every' in args else 1
    domain = int(os.environ.get('AAV_SIM_DOMAIN', '69'))
    log = os.environ.get('AAV_SIM_LOG', f'/tmp/aav-heading-check-{domain}.log')
    for i in range(runs):
        print(f'Run {i + 1} of {runs}: ROS domain {domain}, {" ".join(launch_args) or "defaults"}'
              f'{f", gyro bias {bias:g} deg/s" if bias else ""}'
              f'{f", wrong start {wrong:g} deg" if wrong else ""}'
              f'{f", one GPS fix in {every}" if every > 1 else ""}', flush=True)
        out = run_once(domain, launch_args, bias, log, wrong, every)
        if out is None:
            continue
        if 'recovered_s' in out:
            print(f"  from {wrong:g} deg off: under 5 deg after {out['recovered_s']:.1f} s")
        rows = [(k, v) for k, v in out.items() if k in ('forward', 'reversing')]
        rows += [('gps_heading ' + k, v) for k, v in out.get('gps_heading', {}).items()]
        for name, s in rows:
            if s['n']:
                print(f"  {name:9s} heading off: median {s['median']:.1f} deg, 95% "
                      f"{s['p95']:.1f}, worst {s['worst']:.1f} ({s['n']} samples)")
        record = {'args': launch_args, 'gyro_bias': bias, 'wrong_start': wrong,
                  'gps_every': every, **out}
        print('  JSON ' + json.dumps(record), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
