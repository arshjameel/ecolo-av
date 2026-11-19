"""
sim_gps: Ravensim's NavSat sensor, made to read like the car's GPS.

Subscribes  vehicle/sim/navsat             sensor_msgs/NavSatFix (bridged NavSat sensor)
            /vehicle/sim/truth     nav_msgs/Odometry: the car's true heading, speed and
                                      turn, for the receiver's velocity (gps.py)
Publishes   sensors/gps/fix           sensor_msgs/NavSatFix, in gps_link, a fix, with a
                                      covariance
            sensors/gps/fix_velocity  geometry_msgs/TwistWithCovarianceStamped, in
                                      gps_link: the antenna's velocity east, north, up
                                      (m/s), as ublox_gps publishes it; with each fix,
                                      only while the GPS has no fault (or drifts: a
                                      bias in the position leaves the Doppler speed
                                      true)

Parameters  gps_stddev (0.5 m: GPS without RTK), gps_speed_stddev (0.05 m/s: a u-blox's
            speed accuracy), antenna_x, antenna_y (m from base_link: vehicle.yaml's
            gps_link), every (1: pass on every one of Ravensim's 5 Hz fixes; 5: one a
            second, as the car's receiver sends from the factory; may change while
            running), fault.gps (ok; stop, freeze, nan, jump, drift), seed (5: the velocity's
            noise, so a run repeats)

One process of its own (relay.py).
"""

import math
import random

from aav_sim_models import faults
from aav_sim_models.gps import antenna_velocity, noisy
from aav_sim_models.relay import run, SensorRelay
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


def gps_nan(msg):
    msg.latitude = msg.longitude = msg.altitude = float('nan')


def gps_jump(msg):
    msg.latitude = faults.jumped(msg.latitude)


def gps_drift(msg, seconds):
    msg.latitude = faults.jumped(msg.latitude, faults.drifted(seconds))


class SimGps(SensorRelay):

    def __init__(self):
        super().__init__('sim_gps', 'gps')
        self.gps_var = self.declare_parameter('gps_stddev', 0.5).value ** 2
        self.speed_std = self.declare_parameter('gps_speed_stddev', 0.05).value
        self.rng = random.Random(self.declare_parameter('seed', 5).value)
        self.antenna = (self.declare_parameter('antenna_x', 0.0).value,
                        self.declare_parameter('antenna_y', 0.0).value)
        self.truth = None
        self.declare_parameter('every', 1)     # may change while running
        self.count = 0
        self.pub = self.create_publisher(NavSatFix, 'sensors/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped,
                                             'sensors/gps/fix_velocity', 10)
        self.create_subscription(Odometry, '/vehicle/sim/truth',
                                 lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(NavSatFix, 'vehicle/sim/navsat', self.on_navsat, 10)

    def on_navsat(self, msg):
        self.count += 1
        if self.count % max(1, self.get_parameter('every').value):
            return
        msg.header.frame_id = 'gps_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.position_covariance = [self.gps_var, 0.0, 0.0,
                                   0.0, self.gps_var, 0.0,
                                   0.0, 0.0, 4 * self.gps_var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        msg = self.faulty(msg, nan=gps_nan, jump=gps_jump, drift=gps_drift, now=now)
        if msg is not None:
            self.pub.publish(msg)
            if self.fault.mode in ('ok', 'drift'):
                self.publish_velocity(msg.header.stamp)

    def publish_velocity(self, stamp):
        if self.truth is None:
            return
        q, twist = self.truth.pose.pose.orientation, self.truth.twist.twist
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        east, north = antenna_velocity(yaw, twist.linear.x, twist.angular.z, self.antenna)
        out = TwistWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = 'gps_link'
        out.twist.twist.linear.x = noisy(east, self.speed_std, self.rng)
        out.twist.twist.linear.y = noisy(north, self.speed_std, self.rng)
        out.twist.twist.linear.z = noisy(0.0, self.speed_std, self.rng)
        for i in (0, 7, 14):
            out.twist.covariance[i] = self.speed_std ** 2
        out.twist.covariance[21] = -1.0      # no turn rate, as ublox_gps
        self.vel_pub.publish(out)


def main(args=None):
    run(SimGps, args)


if __name__ == '__main__':
    main()
