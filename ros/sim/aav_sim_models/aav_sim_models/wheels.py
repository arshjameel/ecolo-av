"""
Each wheel's ground speed from Ravensim's joint states, for /vehicle/wheels. No ROS here.

Also the steering angle Ravensim's front wheels really have (centre_steering), for

The wheel joints turn about their +y axis (aav.urdf.xacro): positive spin rolls the car
forward, so speed = spin (rad/s) x the wheel's radius.
"""

import math

# WheelSpeeds field -> the URDF's joint.
JOINTS = {
    'front_left': 'front_left_wheel_joint',
    'front_right': 'front_right_wheel_joint',
    'rear_left': 'rear_left_wheel_joint',
    'rear_right': 'rear_right_wheel_joint',
}


def ground_speeds(names, velocities, radius):
    """Return {field: m/s} for each wheel; NaN for a wheel the message doesn't have."""
    spin = dict(zip(names, velocities))
    return {field: spin[joint] * radius if joint in spin else math.nan
            for field, joint in JOINTS.items()}


STEERING_JOINTS = ('front_left_steering_joint', 'front_right_steering_joint')


def centre_steering(names, positions):
    """
    Return the bicycle model's steering angle (rad) from the two front wheels' joints.

    Ackermann: both wheels point at the same turning centre, so the centre wheel's
    cot = the mean of the two cots, whatever the track. NaN if a joint is missing.
    """
    angle = dict(zip(names, positions))
    if not all(j in angle for j in STEERING_JOINTS):
        return math.nan
    left, right = (math.tan(angle[j]) for j in STEERING_JOINTS)
    if left * right <= 1e-12:        # straight, or the two disagree in sign: no centre
        return math.atan((left + right) / 2.0)
    return math.atan(2.0 * left * right / (left + right))
