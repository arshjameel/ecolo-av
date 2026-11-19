"""
sim_lidar_relay: Ravensim's roof lidar, through its fault on purpose (TODO 246).

Subscribes  vehicle/sim/lidar/points   sensor_msgs/PointCloud2, reliable (the simulator's
                                       sensors/lidar/points, remapped here)
Publishes   sensors/lidar/points       sensor_msgs/PointCloud2, reliable, as the simulator

Parameters  fault.lidar (ok; stop, freeze)

Only with faults:=true and the lidar (ravensim.launch.py), so the sensor watchdog's lidar
input and the stop zone can be tested with a lidar that stops or freezes. The cloud is
passed on as it is: no copy, no conversion. One process of its own (relay.py): about
0.9 MB a cloud.
"""

from aav_sim_models.relay import run, SensorRelay
from sensor_msgs.msg import PointCloud2


class SimLidar(SensorRelay):

    def __init__(self):
        super().__init__('sim_lidar_relay', 'lidar')
        # Reliable, 10 deep: the simulator's own publisher's QoS (rclrs's default).
        self.pub = self.create_publisher(PointCloud2, 'sensors/lidar/points', 10)
        self.create_subscription(PointCloud2, 'vehicle/sim/lidar/points', self.on_lidar, 10)

    def on_lidar(self, msg):
        out = self.faulty(msg)
        if out is not None:
            self.pub.publish(out)


def main(args=None):
    run(SimLidar, args)


if __name__ == '__main__':
    main()
