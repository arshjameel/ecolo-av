#!/usr/bin/env python3
"""
Record a running simulator's cubes for the vectors: gen_vectors.py --recorded reads them.

Next to a running sim (the same ROS domain, things:=true and the lidar on), it records
each cube cloud (/perception/voxels/cubes, base_link) with the car's pose in odom at its
stamp and in map (newest), the radar's clusters with the radar's pose, the car's speed,
and the saved map (/map's walls), for SECONDS of sim time. With --clouds FILE it also
keeps a few raw lidar clouds for aav_lidar_rs' gen_vectors.py --recorded.

  ROS_DOMAIN_ID=<the sim's> pixi run python ros/perception/aav_things_rs/test/capture_cubes.py \
      /tmp/cubes.json --seconds 15 --name 'recorded: road world, movers' [--clouds /tmp/c.json]
"""

import argparse
import json
import math

from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

TAKEN = 65


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def f32s(values):
    return [float(str(v)) for v in np.asarray(values, dtype=np.float32).ravel()]


class Capture(Node):

    def __init__(self, args):
        super().__init__('capture_cubes')
        self.args = args
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.frames, self.clouds, self.map = [], [], None
        self.pending_radar = None
        self.velocity = [0.0, 0.0]
        self.waiting = []
        newest = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, '/perception/voxels/cubes', self.on_cubes, newest)
        self.create_subscription(PointCloud2, '/perception/radar/clusters', self.on_radar,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odom/local', self.on_odom, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        if args.clouds:
            self.create_subscription(PointCloud2, '/sensors/lidar/points', self.on_cloud,
                                     QoSProfile(depth=1,
                                                reliability=ReliabilityPolicy.RELIABLE))
        self.start = None

    def pose(self, target, frame, stamp):
        try:
            t = self.tf.lookup_transform(target, frame, stamp)
        except tf2_ros.TransformException:
            return None
        tr = t.transform.translation
        return [tr.x, tr.y, yaw_of(t.transform.rotation)]

    def on_map(self, msg):
        data = np.asarray(msg.data, dtype=np.int16)
        self.map = {'width': msg.info.width, 'height': msg.info.height,
                    'resolution': msg.info.resolution,
                    'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
                    'slack': 0.25, 'walls': np.flatnonzero(data >= TAKEN).tolist()}

    def on_odom(self, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self.velocity = [v.x * math.cos(yaw) - v.y * math.sin(yaw),
                         v.x * math.sin(yaw) + v.y * math.cos(yaw)]

    def on_radar(self, msg):
        if msg.width * msg.height == 0:
            return
        pose = self.pose('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if pose is None:
            return
        rows = point_cloud2.read_points(msg, field_names=('x', 'y', 'velocity', 'cluster'),
                                        skip_nans=True)
        self.pending_radar = {'t': seconds(msg.header.stamp), 'pose': pose,
                              'car_velocity': list(self.velocity),
                              'rows': [[float(r[0]), float(r[1]), float(r[2]), float(r[3])]
                                       for r in rows]}

    def on_cloud(self, msg):
        if len(self.clouds) < self.args.keep_clouds and len(self.frames) % 40 == 5:
            xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                 skip_nans=False)
            self.clouds.append(f32s(xyz))

    def on_cubes(self, msg):
        self.waiting.append(msg)
        while self.waiting:
            m = self.waiting[0]
            stamp = Time.from_msg(m.header.stamp)
            if not self.tf.can_transform('odom', m.header.frame_id, stamp):
                if seconds(m.header.stamp) < seconds(msg.header.stamp) - 1.0:
                    self.waiting.pop(0)
                    continue
                return
            self.waiting.pop(0)
            t = seconds(m.header.stamp)
            if self.start is None:
                self.start = t
            if t - self.start > self.args.seconds:
                raise SystemExit
            xyz = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'))
            frame = {'t': t, 'publish': t + 0.05,
                     'pose': self.pose('odom', m.header.frame_id, stamp),
                     'map_pose': self.pose('map', m.header.frame_id, Time()),
                     'cubes': f32s(xyz)}
            if self.pending_radar is not None:
                frame['radar'], self.pending_radar = self.pending_radar, None
            self.frames.append(frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('out')
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--name', default='recorded: our simulator')
    parser.add_argument('--clouds', help='also keep raw lidar clouds here')
    parser.add_argument('--keep-clouds', type=int, default=3)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    run = {'name': args.name, 'below': 0.7, 'map': node.map, 'frames': node.frames}
    with open(args.out, 'w') as f:
        json.dump({'runs': [run]}, f)
    print(f'{args.out}: {len(node.frames)} clouds of cubes, the map '
          f'{"kept" if node.map else "not seen"}')
    if args.clouds:
        from aav_description import vehicle
        from aav_lidar.scan import body_box
        cfg = vehicle.load()
        lidar = cfg['sensors']['lidar_link']
        with open(args.clouds, 'w') as f:
            json.dump({'xyz': lidar['xyz'], 'rpy': lidar['rpy'],
                       'body': list(body_box(vehicle.footprint_points(cfg['vehicle']))),
                       'clouds': node.clouds}, f)
        print(f'{args.clouds}: {len(node.clouds)} lidar clouds')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
