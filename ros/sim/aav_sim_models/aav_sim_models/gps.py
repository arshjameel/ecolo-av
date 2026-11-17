"""
What the car's GPS receiver says about its velocity, from Ravensim's truth. No ROS here.

A u-blox receiver reports the antenna's velocity, east, north and up (ublox_gps's
fix_velocity), from the satellites' Doppler shift: good to about 0.05 m/s. Ravensim's NavSat
sensor has a velocity too, but ros_gz_bridge's NavSatFix leaves it out, so sim_gps
works it out from the car's true motion instead, with the receiver's noise on top.
"""

import math
import random
from typing import Tuple


def antenna_velocity(yaw: float, speed: float, turn: float,
                     antenna: Tuple[float, float]) -> Tuple[float, float]:
    """
    Return the antenna's (east, north) velocity, m/s.

    yaw: the car's heading (rad, from east); speed: base_link's forward speed (m/s);
    turn: the turn rate (rad/s); antenna: (x, y) from base_link (m). In a turn an antenna
    ahead of the rear axle swings outwards: w x r.
    """
    x_a, y_a = antenna
    fwd, left = speed - turn * y_a, turn * x_a
    c, s = math.cos(yaw), math.sin(yaw)
    return c * fwd - s * left, s * fwd + c * left


def noisy(value: float, stddev: float, rng=random) -> float:
    """Return value with Gaussian noise of stddev."""
    return value + rng.gauss(0.0, stddev)
