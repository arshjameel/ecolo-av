#!/usr/bin/env python3
"""
Save the front camera's pictures of the simulated stop sign, from several distances.

    pixi run python3 tools/sim_frames.py --out ~/aav-models/frames   (about 1 minute)

For tools/compare_detectors.py: every detector gets the same pictures. Starts the road
world on Ravensim (no lidar, radar or watchdog), on its own ROS domain, on this computer
only; puts the car on the lane's centre line so that the front left camera is each
DISTANCES metres from the sign (vehicle/sim/set_entity_state:
simulation_interfaces/SetEntityState); and saves
/sensors/camera/front_left/image_raw as road_sign_<m>m.png. Stops the launch by its
process group.
"""

import argparse
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

SIGN_X = 25.0       # m east: the road world's stop sign (bench.yaml stop_signs)
CAMERA_X = 1.75     # m: the front cameras ahead of base_link (vehicle.yaml camera_left_link)
DISTANCES = (40.0, 30.0, 20.0, 15.0, 10.0, 7.0, 5.0)


def car_x(distance: float) -> float:
    """Where base_link goes for the camera to be this far from the sign."""
    return SIGN_X - CAMERA_X - distance


def place_own(node, client, x: float):
    """Put the car at x on Ravensim (SetEntityState); return (ok, why)."""
    import rclpy
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import SetEntityState
    if not client.wait_for_service(timeout_sec=3.0):
        return False, 'no vehicle/sim/set_entity_state'
    request = SetEntityState.Request(entity='aav')
    request.state.pose.position.x = x
    request.state.pose.orientation.w = 1.0
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    answer = future.result()
    if answer is None:
        return False, 'no answer in 3 s'
    return answer.result.result == Result.RESULT_OK, answer.result.error_message


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--out', type=Path, default=Path.home() / 'aav-models' / 'frames')
    p.add_argument('--log', default='/tmp/aav-sim-frames.log')
    args = p.parse_args(argv)
    out = args.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    # A private ROS domain, on this computer only, before rclpy starts.
    os.environ.setdefault('ROS_DOMAIN_ID', str(random.randint(150, 230)))
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    import cv2
    from cv_bridge import CvBridge

    print(f"Starting Ravensim's road world on ROS domain {os.environ['ROS_DOMAIN_ID']} "
          f'(log {args.log})', flush=True)
    log = open(args.log, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'aav_bringup', 'sim.launch.py', 'world:=road',
         'rviz:=false', 'nav:=false', 'lidar:=false', 'radar:=false', 'watchdog:=false'],
        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('sim_frames')
    from simulation_interfaces.srv import SetEntityState
    place = node.create_client(SetEntityState, '/vehicle/sim/set_entity_state')
    latest = {}
    node.create_subscription(Image, '/sensors/camera/front_left/image_raw',
                             lambda m: latest.update(msg=m, at=time.monotonic()),
                             qos_profile_sensor_data)
    saved = 0
    try:
        deadline = time.monotonic() + 180.0
        while 'msg' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if 'msg' not in latest:
            print('No picture from the camera in 180 s', file=sys.stderr)
            return 1
        bridge = CvBridge()
        for distance in DISTANCES:
            x = car_x(distance)
            ok = False
            for _ in range(3):      # a busy computer may drop the first call
                ok, why = place_own(node, place, x)
                if ok:
                    break
            if not ok:
                print(f'  {distance:g} m: placing the car failed ({why.strip()})', flush=True)
                continue
            moved = time.monotonic()
            while time.monotonic() < moved + 2.0 or latest['at'] < moved + 1.5:
                rclpy.spin_once(node, timeout_sec=0.2)
            frame = bridge.imgmsg_to_cv2(latest['msg'], 'bgr8')
            path = out / f'road_sign_{distance:04.1f}m.png'
            cv2.imwrite(str(path), frame)
            saved += 1
            print(f'  {path.name}: the car at x = {x:.2f} m', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()
    return 0 if saved == len(DISTANCES) else 1


if __name__ == '__main__':
    sys.exit(main())
