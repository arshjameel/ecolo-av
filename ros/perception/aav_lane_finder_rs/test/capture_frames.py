#!/usr/bin/env python3
"""
Save front left camera pictures from a running simulator, for gen_vectors.py --recorded.

    pixi run tools/ros.sh --local python3 \
        ros/perception/aav_lane_finder_rs/test/capture_frames.py OUT_DIR [--count 8]
        [--every 5] [--where "University Drive"]

Start Ravensim first, with its cameras, on the same ROS domain (a lane drive:
a lane drive on University Drive, `measure lane` until 19 Nov). Every `--every`
seconds of the picture's own clock it writes OUT_DIR/frame_NN.png (the picture as it came,
in BGR), up to `--count`; then OUT_DIR/lens.json: the CameraInfo's size, K and D, and the
camera's place on the car from vehicle.yaml (camera_left_link), as the launch file gives it.

Subscribes /sensors/camera/front_left/image_raw (sensor_msgs/Image),
/sensors/camera/front_left/camera_info (sensor_msgs/CameraInfo). Publishes nothing.
"""

import argparse
import json
from pathlib import Path
import time

from aav_description import vehicle
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

CHANNELS = {'bgr8': (3, None), 'rgb8': (3, cv2.COLOR_RGB2BGR), 'bgra8': (4, cv2.COLOR_BGRA2BGR),
            'rgba8': (4, cv2.COLOR_RGBA2BGR), 'mono8': (1, cv2.COLOR_GRAY2BGR)}


def as_bgr(msg):
    """Return the picture as BGR, or None for an encoding the lane finder doesn't read."""
    if msg.encoding not in CHANNELS:
        return None
    n, code = CHANNELS[msg.encoding]
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * n].reshape(msg.height, msg.width, n)
    return image.copy() if code is None else cv2.cvtColor(image, code)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('out', type=Path)
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--every', type=float, default=5.0, help='s of the picture clock')
    parser.add_argument('--where', default='recorded', help='said in the vectors')
    parser.add_argument('--wall', type=float, default=600.0, help='s to wait at most')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('capture_frames')
    newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    state = {'info': None, 'saved': 0, 'last': None}

    def on_info(msg):
        state['info'] = msg

    def on_image(msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if state['saved'] >= args.count or (state['last'] is not None
                                            and t - state['last'] < args.every):
            return
        image = as_bgr(msg)
        if image is None:
            return
        cv2.imwrite(str(args.out / f'frame_{state["saved"]:02d}.png'), image)
        state['saved'] += 1
        state['last'] = t
        print(f'frame {state["saved"]} at {t:.1f} s', flush=True)

    node.create_subscription(CameraInfo, '/sensors/camera/front_left/camera_info', on_info,
                             newest)
    node.create_subscription(Image, '/sensors/camera/front_left/image_raw', on_image, newest)
    end = time.monotonic() + args.wall
    while (state['saved'] < args.count or state['info'] is None) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    info = state['info']
    camera = vehicle.load()['sensors']['camera_left_link']
    if info is not None:
        lens = {'width': info.width, 'height': info.height, 'fx': info.k[0], 'fy': info.k[4],
                'cx': info.k[2], 'cy': info.k[5],
                'distortion': [float(d) for d in info.d] if any(info.d) else []}
        lens.update(xyz=[float(v) for v in camera['xyz']],
                    rpy=[float(v) for v in camera['rpy']], where=args.where)
        (args.out / 'lens.json').write_text(json.dumps(lens, indent=2) + '\n')
    node.destroy_node()
    rclpy.shutdown()
    print(f'{state["saved"]} pictures, lens {"saved" if info else "never heard"}: {args.out}')
    return 0 if state['saved'] and info else 1


if __name__ == '__main__':
    raise SystemExit(main())
