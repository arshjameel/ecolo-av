"""
sim_radar: Ravensim's radar stand-in (a lidar shaped like the radar), as the TI radar.

Subscribes  vehicle/sim/radar/points   sensor_msgs/PointCloud2, best effort (bridged lidar; our
                                  simulator's adds `speed`: what each ray hit moves)
            vehicle/sim/odom           nav_msgs/Odometry: the car's speed, for each point's
                                  velocity
Publishes   sensors/radar/points  sensor_msgs/PointCloud2, best effort, in radar_link: only
                                  the rays that hit something, with the TI driver's fields
                                  (radar.py)

Parameters  radar_range (8.4 m), fault.radar (ok; stop, freeze)

Only in a world that renders the radar (ravensim.launch.py, radar:=true). One process of its
own (relay.py): its cloud is the heaviest of Ravensim's sensors to convert, and the IMU
used to wait behind it.
"""

from aav_sim_models import radar
from aav_sim_models.relay import run, SensorRelay
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

# The TI driver's fields: x, y, z, intensity, velocity, float32 each.
RADAR_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(radar.FIELDS)]


class SimRadar(SensorRelay):

    def __init__(self):
        super().__init__('sim_radar', 'radar')
        self.radar_range = self.declare_parameter('radar_range', 8.4).value   # m
        self.speed = 0.0
        # Best effort both ways, as the TI driver publishes: its subscribers ask for it.
        self.pub = self.create_publisher(PointCloud2, 'sensors/radar/points',
                                         qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'vehicle/sim/radar/points', self.on_radar,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'vehicle/sim/odom', self.on_odom, 10)

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_radar(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        kept = radar.hit(xyz, self.radar_range)
        own = None
        if any(f.name == 'speed' for f in msg.fields):
            # Our simulator's: what each ray hit moves along it (a person walking).
            own = point_cloud2.read_points_numpy(msg, field_names=('speed',)).reshape(-1)[kept]
        points = radar.ti_points(xyz.reshape(-1, 3)[kept], self.speed, own=own)
        msg.header.frame_id = 'radar_link'
        out = self.faulty(point_cloud2.create_cloud(msg.header, RADAR_FIELDS, points))
        if out is not None:
            self.pub.publish(out)


def main(args=None):
    run(SimRadar, args)


if __name__ == '__main__':
    main()
