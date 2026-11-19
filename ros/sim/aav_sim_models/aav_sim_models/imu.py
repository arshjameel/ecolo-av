"""
The IMU's acceleration in Ravensim: what a real IMU on the car would feel. No ROS here.

Gazebo's IMU missed the car's own acceleration (found 15 Oct by the actuator tests): its
car was driven by speed, and its IMU read only gravity, noise and the odd contact bump.
Speeding up at 0.6 m/s^2 read 0.00 on average. So sim_imu replaced the forward and
sideways readings with the car's:

    forward  = dv/dt - x r^2      speeding up, braking
    left     = v r + x dr/dt      the turn's centripetal pull

v the speed, r the yaw rate, x how far the IMU sits ahead of the rear axle (imu_link,
vehicle.yaml). Up stays Ravensim's: gravity, and the bumps. v and dv/dt come from Ravensim's
odometry (Speedometer), which follows the actuator model's speed: so the IMU needs
nothing from sim_adapter, as a real IMU needs nothing from the car's computer.

Since 18 Nov (S3) Ravensim's own IMU reads all of it, and more: the mean acceleration
between readings, the turn's pull at the sensor, gravity on a slope, turn rates on all three
axes. So sim_imu passes it on (accel_from simulator, the default). This way stays, as
accel_from odometry. Don't pair it with noisy wheels (wheel_noise:=): dv/dt of a noisy
speed at 50 Hz is far noisier (0.05 m/s on each reading gives about 3.5 m/s^2).
"""

from typing import Optional

ACCEL_FROM = ('simulator', 'odometry')


def accel_from_odometry(accel_from: str) -> bool:
    """Return True where sim_imu works the acceleration out from the odometry (the old way)."""
    if accel_from not in ACCEL_FROM:
        raise ValueError(f'accel_from:={accel_from!r}: one of {ACCEL_FROM}')
    return accel_from == 'odometry'


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
