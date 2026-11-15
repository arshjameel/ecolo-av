"""
gazebo_imu: Gazebo's IMU, made to read like the car's.

Subscribes  gazebo/imu            sensor_msgs/Imu (bridged IMU sensor)
            gazebo/odom           nav_msgs/Odometry (bridged from the Ackermann system): the
                                  speed, and from it the car's acceleration
Publishes   sensors/imu/data_raw  sensor_msgs/Imu, in imu_link, with covariances and no
                                  orientation, like the car's IMU; forward and sideways
                                  acceleration from the car's motion (imu.py), as Gazebo's
                                  IMU misses it

Parameters  gyro_stddev (0.02 rad/s), accel_stddev (0.2 m/s^2), imu_x (m ahead of
            base_link: vehicle.yaml's imu_link), fault.imu (ok; stop, freeze, nan),
            gyro_bias (rad/s added to the turn rate, 0: a real gyro's offset, for
            tools/heading_check.py --gyro-bias; may change while running), seed (3: the
            acceleration's noise, so a run repeats)

One process of its own (relay.py), so the IMU is never late behind another sensor.
"""

import random

from aav_gazebo import imu
from aav_gazebo.relay import run, SensorRelay
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def imu_nan(msg):
    for v in (msg.angular_velocity, msg.linear_acceleration):
        v.x = v.y = v.z = float('nan')


class GazeboImu(SensorRelay):

    def __init__(self):
        super().__init__('gazebo_imu', 'imu')
        self.gyro_var = self.declare_parameter('gyro_stddev', 0.02).value ** 2
        self.accel_std = self.declare_parameter('accel_stddev', 0.2).value
        self.accel_var = self.accel_std ** 2
        self.rng = random.Random(self.declare_parameter('seed', 3).value)
        self.imu_x = self.declare_parameter('imu_x', 0.0).value
        self.declare_parameter('gyro_bias', 0.0)
        self.speedometer = imu.Speedometer()
        self.pub = self.create_publisher(Imu, 'sensors/imu/data_raw', 10)
        self.create_subscription(Odometry, 'gazebo/odom', self.on_odom, 10)
        self.create_subscription(Imu, 'gazebo/imu', self.on_imu, 10)

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.speedometer.update(msg.twist.twist.linear.x, stamp)

    def on_imu(self, msg):
        msg.header.frame_id = 'imu_link'
        s = self.speedometer
        if s.ready:
            # Gazebo's IMU misses the car's own acceleration (imu.py): the car's, plus noise.
            fwd, left = imu.body_acceleration(s.accel, s.speed, msg.angular_velocity.z,
                                              x=self.imu_x)
            msg.linear_acceleration.x = fwd + self.rng.gauss(0.0, self.accel_std)
            msg.linear_acceleration.y = left + self.rng.gauss(0.0, self.accel_std)
        msg.angular_velocity.z += self.get_parameter('gyro_bias').value
        msg.orientation_covariance[0] = -1.0     # like the real IMU: no orientation
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_var
            msg.linear_acceleration_covariance[i] = self.accel_var
        msg = self.faulty(msg, nan=imu_nan)
        if msg is not None:
            self.pub.publish(msg)


def main(args=None):
    run(GazeboImu, args)


if __name__ == '__main__':
    main()
