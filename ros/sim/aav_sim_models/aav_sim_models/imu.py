"""
The IMU's acceleration in Ravensim: what a real IMU on the car would feel. No ROS here.

The simulator's IMU misses the car's own acceleration (found 15 Oct by the actuator tests,
in Gazebo; Ravensim's IMU reads the same): its car is driven by speed, and its IMU reads
only gravity, noise and the odd contact bump. Speeding up at 0.6 m/s^2 read 0.00 on average. So
sim_imu replaces the forward and sideways readings with the car's:

    forward  = dv/dt - x r^2      speeding up, braking
    left     = v r + x dr/dt      the turn's centripetal pull

v the speed, r the yaw rate, x how far the IMU sits ahead of the rear axle (imu_link,
vehicle.yaml). Up stays Ravensim's: gravity, and the bumps. v and dv/dt come from Ravensim's
odometry (Speedometer), which follows the actuator model's speed: so the IMU needs
nothing from sim_adapter, as a real IMU needs nothing from the car's computer.
"""

from typing import Optional


def body_acceleration(accel, speed, yaw_rate, yaw_accel=0.0, x=0.0):
    """Return (forward, left) acceleration (m/s^2) at a point x m ahead of the rear axle."""
    return accel - x * yaw_rate * yaw_rate, speed * yaw_rate + x * yaw_accel


class Speedometer:
    """The car's speed and acceleration from Ravensim's odometry: dv/dt between messages."""

    def __init__(self):
        self.speed = 0.0          # m/s
        self.accel = 0.0          # m/s^2
        self.stamp: Optional[float] = None   # s, the last message's

    @property
    def ready(self) -> bool:
        """Return True once a message has come: before it, Ravensim's own reading stands."""
        return self.stamp is not None

    def update(self, speed: float, stamp: float):
        """Take one odometry message: its forward speed (m/s) and stamp (s)."""
        if self.stamp is not None:
            dt = stamp - self.stamp
            if dt > 0.0:
                self.accel = (speed - self.speed) / dt
            elif dt < 0.0:
                self.accel = 0.0  # the clock went back: the world was reset
        self.speed, self.stamp = speed, stamp
