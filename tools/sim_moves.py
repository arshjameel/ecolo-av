#!/usr/bin/env python3
"""
Two moves for the car day's baseline, run against a sim: the pedal, and 3 m and stop.

Ben, 20 Nov: "do some baseline testing like the step function stuff ... So we can do the
same tests on the car itself and see the difference", and "drive 10 feet and stop".

    pedal   a person's stick through ego's pedal (TODO 206), on the keyboard's slot
            (a people driver, so the pedal applies): half forward and hold, let off to a
            quarter, let go, pull back hard, keep holding until it reverses, let go.
    three   aav_scripted's action: 0.5 m/s, 0.3 m/s² each way, the hold worked out so the
            whole drive is 3.0 m (about 10 feet), then stop.

Each writes <out>/<time>-<move>-sim/log.csv in the actuator tests' columns (the ones it
has), so tools/step_plots.py draws it beside them. Start a sim first, on this computer:

    pixi run sim world:=empty rviz:=false nav:=false                    # terminal 1
    pixi run tools/ros.sh --local python3 tools/sim_moves.py pedal      # terminal 2

Stand-ins only: it refuses unless /system/source says SIM.
"""

import argparse
import csv
import datetime
import math
import os
import sys
import time

# The pedal's script: (seconds, stick from -1 back to +1 forward, what it is).
PEDAL = (
    (2.0, 0.0, 'still'),
    (8.0, 0.5, 'half forward, held'),
    (5.0, 0.0, 'let go'),
    (8.0, 0.5, 'half forward again'),
    (6.0, 0.25, 'let off to a quarter'),
    (2.0, -1.0, 'pulled back hard, moving'),
    (6.0, -1.0, 'held back: reverses'),
    (4.0, 0.0, 'let go'),
)
COLUMNS = ('t', 'phase', 'cmd_speed', 'ego_speed', 'speed', 'throttle', 'brake', 'ax',
           'x', 'y', 'distance')


def ramps(speed, accel, decel):
    """Return the metres spent speeding up from 0 and slowing back to 0."""
    return speed * speed / (2 * accel) + speed * speed / (2 * decel)


def hold_for(total, speed, accel, decel):
    """Return hold_distance so that the whole drive (ramps and hold) is total metres."""
    return max(0.0, total - ramps(speed, accel, decel))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('move', choices=('pedal', 'three'))
    parser.add_argument('--out', default=os.path.expanduser('~/aav-tests/baseline'),
                        help='where the run folder goes')
    parser.add_argument('--full', type=float, default=1.0,
                        help='m/s of a full stick (ego.yaml pedal_full)')
    parser.add_argument('--metres', type=float, default=3.0, help='three: the whole drive')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from ackermann_msgs.msg import AckermannDriveStamped
    from aav_interfaces.action import ScriptedDrive
    from aav_interfaces.msg import ActuatorState, DataSource, SensorSource
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = Node('sim_moves', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    now = {'speed': math.nan, 'ego_speed': math.nan, 'throttle': math.nan,
           'brake': math.nan, 'ax': math.nan, 'x': math.nan, 'y': math.nan, 'source': None}

    def keep(name, read):
        return lambda msg: now.__setitem__(name, read(msg))

    best = qos_profile_sensor_data
    node.create_subscription(Odometry, '/vehicle/odom',
                             keep('speed', lambda m: m.twist.twist.linear.x), best)
    node.create_subscription(AckermannDriveStamped, '/ego/actuator_cmd',
                             keep('ego_speed', lambda m: m.drive.speed), best)
    node.create_subscription(ActuatorState, '/vehicle/throttle',
                             keep('throttle', lambda m: m.measured), best)
    node.create_subscription(ActuatorState, '/vehicle/brake',
                             keep('brake', lambda m: m.measured), best)
    node.create_subscription(Imu, '/sensors/imu/data',
                             keep('ax', lambda m: m.linear_acceleration.x), best)
    node.create_subscription(Odometry, '/localization/odom/local',
                             keep('x', lambda m: m.pose.pose.position.x), best)
    node.create_subscription(Odometry, '/localization/odom/local',
                             keep('y', lambda m: m.pose.pose.position.y), best)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Every entry of every announcement must say SIM (the actuator tests' rule, loosely).
    node.create_subscription(DataSource, '/system/source',
                             keep('source', lambda m: {s.source for s in m.sensors}), latched)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    spin_for(3.0)
    if now['source'] != {SensorSource.SIM}:
        print('refused: /system/source does not say SIM (start the sim first, --local)')
        return 2

    stamp = datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S')
    folder = os.path.join(args.out, f'{stamp}-{args.move}-sim')
    os.makedirs(folder, exist_ok=True)
    rows, start, last = [], node.get_clock().now(), [None]
    state = {'phase': 'start', 'cmd': 0.0, 'distance': 0.0}

    def log():
        t = (node.get_clock().now() - start).nanoseconds / 1e9
        if last[0] is not None and not math.isnan(now['speed']):
            state['distance'] += abs(now['speed']) * (t - last[0])
        last[0] = t
        rows.append([round(t, 3), state['phase'], state['cmd'], now['ego_speed'],
                     now['speed'], now['throttle'], now['brake'], now['ax'], now['x'],
                     now['y'], round(state['distance'], 4)])

    if args.move == 'pedal':
        pub = node.create_publisher(TwistStamped, '/drivers/keyboard/cmd_vel', 10)
        for seconds, stick, words in PEDAL:
            state['phase'], state['cmd'] = words, stick * args.full
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                msg = TwistStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.linear.x = stick * args.full
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.02)
                log()
    else:
        speed, accel, decel = 0.5, 0.3, 0.3
        client = ActionClient(node, ScriptedDrive, '/scripted_drive')
        if not client.wait_for_server(timeout_sec=10.0):
            print('no /scripted_drive server: is aav_scripted running in this sim?')
            return 2
        goal = ScriptedDrive.Goal(target_speed=speed, acceleration=accel,
                                  deceleration=decel,
                                  hold_distance=hold_for(args.metres, speed, accel, decel))
        state['phase'], state['cmd'] = f'{args.metres:g} m and stop', speed
        future = client.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            log()
        result = future.result().get_result_async()
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            log()
        state['phase'], state['cmd'] = 'stopped', 0.0
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            log()
        r = result.result().result
        print(f'scripted: {r.message}; {r.distance_travelled:.2f} m by its odometry '
              f'(asked {args.metres:g} m)')

    with open(os.path.join(folder, 'log.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerows(rows)
    print(f'{args.move}: {len(rows)} rows, {state["distance"]:.2f} m by the wheels: {folder}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
