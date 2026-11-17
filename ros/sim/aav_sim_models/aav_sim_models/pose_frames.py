"""
The map -> odom transform that puts the car where the simulator says it is.

No ROS here (tests: test/test_pose_frames.py). truth_localizer.py uses it for the wiggle's
first test (TODO 149): the simulator's truth as the pose, in place of the global EKF.

A pose is (x, y, yaw) in metres and radians, on the ground (REP-103, REP-105):

  map_base   where the car is on the map: the simulator's truth (map -> base_link)
  odom_base  where ekf_local has it (odom -> base_link)

map -> odom = map_base composed with the inverse of odom_base, so that map -> odom ->
base_link lands exactly on the truth.
"""

import math


def compose(a, b):
    """Return the pose b, given in a's frame, in a's parent frame: a * b."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw)))


def inverse(a):
    """Return the inverse of a pose: the parent frame seen from it."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def map_to_odom(map_base, odom_base):
    """Return map -> odom from map -> base_link (the truth) and odom -> base_link."""
    return compose(map_base, inverse(odom_base))


def quaternion_of(yaw):
    """Return (z, w) of the quaternion that turns by yaw (rad) about z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_of(z, w, x=0.0, y=0.0):
    """Return the yaw (rad) of a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
