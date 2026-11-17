"""
sim_camera_relay: the front left camera's picture, through its fault on purpose.

Subscribes  vehicle/sim/camera/front_left/image_raw   sensor_msgs/Image (the road bridge's)
Publishes   sensors/camera/front_left/image_raw  sensor_msgs/Image

Parameters  fault.camera (ok; stop, freeze)

Only with faults:=true (ravensim.launch.py camera_relay), which sends the road bridge's
picture here instead of straight to the car's topic. Reliable, the newest only: best
effort loses whole pictures over UDP (docs/mac.md). One process of its own (relay.py):
pictures are big.
"""

from aav_sim_models.relay import run, SensorRelay
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class SimCamera(SensorRelay):

    def __init__(self):
        super().__init__('sim_camera_relay', 'camera')
        picture = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Image, 'sensors/camera/front_left/image_raw', picture)
        self.create_subscription(Image, 'vehicle/sim/camera/front_left/image_raw', self.on_camera,
                                 picture)

    def on_camera(self, msg):
        out = self.faulty(msg)
        if out is not None:
            self.pub.publish(out)


def main(args=None):
    run(SimCamera, args)


if __name__ == '__main__':
    main()
