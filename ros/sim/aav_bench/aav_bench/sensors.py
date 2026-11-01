"""
The pretend sensors, for bench mode.

Each one stands where a real driver will, on the same topic, with the same message type
and frame, so nothing downstream changes.

  mock_imu      /vehicle/bench/truth -> /sensors/imu/data_raw (Imu),
                /sensors/imu/mag (MagneticField)                                imu_link
  mock_gps      /vehicle/bench/truth -> /sensors/gps/fix (NavSatFix), 5 Hz       gps_link
                with a receiver's error: a slow drift and a little jitter (GpsError)
  mock_battery  /vehicle/battery (BatteryState), 1 Hz
  mock_camera   /vehicle/bench/truth -> /sensors/camera/front_left/image_raw (Image),
                /sensors/camera/front_left/camera_info,
                /sensors/camera/front_left/image_raw/compressed (JPEG, only while
                something reads it),
                10 Hz: the bench road and its lane lines from where the car really is,
                and the bench world's stop signs when they're in view
  mock_radar    /vehicle/bench/truth -> /sensors/radar/points (PointCloud2, best effort),
                10 Hz                                                           radar_link
                the bench world's obstacle boxes, when obstacle is true; else empty
  mock_lidar    /vehicle/bench/truth -> UDP packets, as the roof's VLP-16 sends them, to
                127.0.0.1:2369, 754 a second: the bench world's boxes (obstacle) and the
                flat ground. The stock velodyne driver reads them, as on the car
                (aav_sensors sensors.launch.py lidar:=true), and publishes
                /sensors/lidar/points                                           lidar_link

Each one also says, once and latched, on /system/source (aav_interfaces/DataSource), that
its topic is BENCH: a stand-in, not the real device. The HUD shows a BENCH banner.

spine_sim publishes /vehicle/bench/truth: where the model car really is. The real drivers are
aav_imu_yahboom, ublox_gps, aav_battery_victron, usb_cam (aav_sensors), aav_radar_ti and,
for the lidar, the same velodyne driver mock_lidar feeds.
"""

import math
import random
import signal
import socket
import time

from aav_bench import models, pictures, vlp16
from aav_description import vehicle
from aav_interfaces.msg import DataSource, SensorSource
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Imu
from sensor_msgs.msg import MagneticField, NavSatFix
from sensor_msgs.msg import NavSatStatus, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

TRUTH = '/vehicle/bench/truth'


def truth_from(msg: Odometry) -> models.Truth:
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return models.Truth(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                        msg.twist.twist.linear.x, msg.twist.twist.angular.z)


class MockImu(Node):

    ANNOUNCE = ('imu', '/sensors/imu/data_raw')   # for /system/source

    def __init__(self):
        super().__init__('mock_imu')
        self.frame = self.declare_parameter('frame_id', 'imu_link').value
        self.gyro_sd = self.declare_parameter('gyro_stddev', 0.02).value
        self.accel_sd = self.declare_parameter('accel_stddev', 0.2).value
        self.gyro_bias = self.declare_parameter('gyro_bias', 0.002).value
        self.rng = random.Random(self.declare_parameter('seed', 1).value)
        self.imu = self.create_publisher(Imu, '/sensors/imu/data_raw', 10)
        self.mag = self.create_publisher(MagneticField, '/sensors/imu/mag', 10)
        self.before = None
        self.create_subscription(Odometry, TRUTH, self.on_truth, 10)

    def on_truth(self, msg: Odometry):
        now = truth_from(msg)
        accel, gyro = models.imu_reading(now, self.before)
        self.before = now
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.frame
        a = models.noisy(accel, self.accel_sd, self.rng)
        w = models.noisy(gyro, self.gyro_sd, self.rng, (0.0, 0.0, self.gyro_bias))
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = a
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = w
        imu.orientation_covariance[0] = -1.0   # no orientation: the Madgwick filter adds it
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = self.gyro_sd ** 2
            imu.linear_acceleration_covariance[i] = self.accel_sd ** 2
        self.imu.publish(imu)
        mag = MagneticField()
        mag.header = imu.header
        m = models.magnetic_field(now.yaw)
        mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = m
        self.mag.publish(mag)


class MockGps(Node):

    ANNOUNCE = ('gps', '/sensors/gps/fix')   # for /system/source

    def __init__(self):
        super().__init__('mock_gps')
        self.frame = self.declare_parameter('frame_id', 'gps_link').value
        self.datum = self.declare_parameter('datum', [45.3860, -75.6960, 0.0]).value
        drift = self.declare_parameter('drift_stddev', 0.5).value
        drift_time = self.declare_parameter('drift_time', 60.0).value
        jitter = self.declare_parameter('jitter_stddev', 0.05).value
        rate = self.declare_parameter('rate_hz', 5.0).value
        self.dt = 1.0 / rate
        rng = random.Random(self.declare_parameter('seed', 2).value)
        self.error = models.GpsError(drift, drift_time, jitter, rng)
        self.sd = self.error.stddev
        self.antenna = vehicle.load()['sensors']['gps_link']['xyz']
        self.pub = self.create_publisher(NavSatFix, '/sensors/gps/fix', 10)
        self.truth = None
        self.create_subscription(Odometry, TRUTH, self.on_truth, 10)
        self.create_timer(self.dt, self.tick)

    def on_truth(self, msg: Odometry):
        self.truth = (truth_from(msg), msg.header.stamp)

    def tick(self):
        if self.truth is None:
            return
        truth, stamp = self.truth
        x, y = models.offset(truth, self.antenna)
        east, north = self.error.step(self.dt)
        x, y = x + east, y + north
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self.frame
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = models.geo_from_map(x, y, self.datum[0], self.datum[1])
        fix.altitude = self.datum[2] + self.antenna[2]
        fix.position_covariance = [self.sd ** 2, 0.0, 0.0, 0.0, self.sd ** 2, 0.0,
                                   0.0, 0.0, (2.0 * self.sd) ** 2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)


class MockBattery(Node):

    ANNOUNCE = ('battery', '/vehicle/battery')   # for /system/source

    def __init__(self):
        super().__init__('mock_battery')
        self.full = self.declare_parameter('full_voltage', 50.9).value
        self.empty = self.declare_parameter('empty_voltage', 46.0).value
        self.hours = self.declare_parameter('hours', 2.0).value
        self.pub = self.create_publisher(BatteryState, '/vehicle/battery', 10)
        self.start = self.get_clock().now()
        self.create_timer(1.0, self.tick)

    def tick(self):
        hours = (self.get_clock().now() - self.start).nanoseconds / 3.6e12
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = models.battery_voltage(hours, self.full, self.empty, self.hours)
        msg.percentage = (msg.voltage - self.empty) / (self.full - self.empty)
        msg.current = math.nan
        msg.charge = msg.capacity = msg.design_capacity = math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.pub.publish(msg)


class MockCamera(Node):

    ANNOUNCE = ('camera_front_left', '/sensors/camera/front_left/image_raw')   # for /system/source

    def __init__(self):
        super().__init__('mock_camera')
        self.frame = self.declare_parameter('frame_id', 'camera_left_link').value
        width = self.declare_parameter('width', 640).value
        height = self.declare_parameter('height', 360).value
        focal = self.declare_parameter('focal_px', 343.0).value
        rate = self.declare_parameter('rate_hz', 10.0).value
        self.signs = models.pairs(self.declare_parameter('stop_signs', [25.0, -2.0]).value)
        self.sign_height = self.declare_parameter('sign_height', 2.0).value
        self.sign_size = self.declare_parameter('sign_size', 0.75).value
        d = models.Road()
        self.road = models.Road(
            y=self.declare_parameter('road_y', d.y).value,
            lane_width=self.declare_parameter('lane_width', d.lane_width).value,
            line_width=self.declare_parameter('line_width', d.line_width).value,
            shoulder=self.declare_parameter('shoulder', d.shoulder).value,
            bend_x=self.declare_parameter('bend_x', d.bend_x).value,
            bend_radius=self.declare_parameter('bend_radius', d.bend_radius).value)
        mount = vehicle.load()['sensors'][self.frame]
        xyz, rpy = mount['xyz'], mount.get('rpy', [0.0, 0.0, 0.0])
        self.cam = models.Camera(xyz, width, height, focal, xyz[2], rpy[1])
        self.info = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
        self.info.header.frame_id = self.frame
        self.info.d = [0.0] * 5
        self.info.k = [focal, 0.0, width / 2.0, 0.0, focal, height / 2.0, 0.0, 0.0, 1.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [focal, 0.0, width / 2.0, 0.0, 0.0, focal, height / 2.0, 0.0,
                       0.0, 0.0, 1.0, 0.0]
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/sensors/camera/front_left/image_raw', 2)
        # The JPEG pictures, as usb_cam's image_transport publishes them: a laptop's
        # detector on the car's WiFi reads these (detector.launch.py compressed:=true).
        self.jpeg_pub = self.create_publisher(
            CompressedImage, '/sensors/camera/front_left/image_raw/compressed', 2)
        self.info_pub = self.create_publisher(CameraInfo,
                                              '/sensors/camera/front_left/camera_info', 2)
        self.truth = None
        self.create_subscription(Odometry, TRUTH, self.on_truth, 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_truth(self, msg: Odometry):
        self.truth = truth_from(msg)

    def tick(self):
        if self.truth is None:
            return
        views = [v for v in (models.sign_in_view(self.cam, self.truth, s, self.sign_height,
                                                 self.sign_size) for s in self.signs) if v]
        frame = pictures.scene(self.cam, views, self.truth, self.road)
        image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame
        self.info.header.stamp = image.header.stamp
        self.image_pub.publish(image)
        self.info_pub.publish(self.info)
        if self.jpeg_pub.get_subscription_count():
            jpeg = self.bridge.cv2_to_compressed_imgmsg(frame, 'jpeg')
            jpeg.header = image.header
            self.jpeg_pub.publish(jpeg)


# The fields the TI radar driver publishes (aav_radar_ti pc_publisher.py).
RADAR_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(('x', 'y', 'z', 'intensity', 'velocity'))]


class MockRadar(Node):

    ANNOUNCE = ('radar', '/sensors/radar/points')   # for /system/source

    def __init__(self):
        super().__init__('mock_radar')
        self.frame = self.declare_parameter('frame_id', 'radar_link').value
        rate = self.declare_parameter('rate_hz', 10.0).value
        # Read on every frame, so a test can move the boxes or take them away:
        # ros2 param set /mock_radar obstacle false
        self.declare_parameter('obstacle', False)
        self.declare_parameter('boxes', [15.0, -0.5, 16.0, 0.5])
        self.declare_parameter('box_height', 1.0)
        mount = vehicle.load()['sensors'][self.frame]
        self.radar = models.Radar(
            mount['xyz'], mount['rpy'][2],
            self.declare_parameter('max_range', 8.4).value,
            math.radians(self.declare_parameter('fov_deg', 120.0).value),
            math.radians(self.declare_parameter('ray_step_deg', 2.0).value))
        self.keep = self.declare_parameter('keep', 0.8).value
        self.range_noise = self.declare_parameter('range_noise', 0.05).value
        self.height_noise = self.declare_parameter('height_noise', 0.1).value
        self.rng = random.Random(self.declare_parameter('seed', 4).value)
        self.add_on_set_parameters_callback(self.check)
        # Best effort, as the TI driver publishes: its subscribers must match it.
        self.pub = self.create_publisher(PointCloud2, '/sensors/radar/points',
                                         qos_profile_sensor_data)
        self.truth = None
        self.create_subscription(Odometry, TRUTH, self.on_truth, 10)
        self.create_timer(1.0 / rate, self.tick)

    def check(self, params):
        for p in params:
            if p.name == 'boxes' and len(p.value) % 4:
                return SetParametersResult(successful=False,
                                           reason='boxes needs x1, y1, x2, y2 per box')
        return SetParametersResult(successful=True)

    def on_truth(self, msg: Odometry):
        self.truth = truth_from(msg)

    def tick(self):
        if self.truth is None:
            return
        obstacles = (models.boxes(self.get_parameter('boxes').value)
                     if self.get_parameter('obstacle').value else [])
        points = models.radar_returns(self.radar, self.truth, obstacles,
                                      self.get_parameter('box_height').value, self.rng,
                                      self.keep, self.range_noise, self.height_noise)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)
        self.pub.publish(point_cloud2.create_cloud(
            header, RADAR_FIELDS, [(x, y, z, 1.0, v) for x, y, z, v in points]))


class MockLidar(Node):
    """
    The roof's VLP-16, as UDP packets: below the driver, so the driver itself is tested.

    Its pose comes from vehicle.yaml (lidar_link); it sees the same boxes as mock_radar
    (bench.yaml), read on every packet, so a test can place them:
    ros2 param set /mock_lidar obstacle true
    """

    ANNOUNCE = ('lidar', '/sensors/lidar/points')   # for /system/source

    def __init__(self):
        super().__init__('mock_lidar')
        self.address = (self.declare_parameter('host', '127.0.0.1').value,
                        self.declare_parameter('port', 2369).value)
        self.declare_parameter('obstacle', False)
        self.declare_parameter('boxes', [15.0, -0.5, 16.0, 0.5])
        self.declare_parameter('box_height', 1.0)
        mount = vehicle.load()['sensors']['lidar_link']
        self.lidar = models.Lidar(mount['xyz'], [math.radians(e) for e in vlp16.ELEVATIONS],
                                  self.declare_parameter('max_range', 100.0).value)
        self.add_on_set_parameters_callback(self.check)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender = vlp16.Sender(lambda data: self.sock.sendto(data, self.address),
                                   time.monotonic())
        self.truth = None
        self.create_subscription(Odometry, TRUTH, self.on_truth, 10)
        # 100 times a second, the packets due since the last time: about 8 each.
        self.create_timer(0.01, self.tick)

    def check(self, params):
        for p in params:
            if p.name == 'boxes' and len(p.value) % 4:
                return SetParametersResult(successful=False,
                                           reason='boxes needs x1, y1, x2, y2 per box')
        return SetParametersResult(successful=True)

    def on_truth(self, msg: Odometry):
        self.truth = truth_from(msg)

    def tick(self):
        if self.truth is None:
            self.sender.start = time.monotonic()   # nothing due before the car is there
            return
        truth = self.truth
        obstacles = (models.boxes(self.get_parameter('boxes').value)
                     if self.get_parameter('obstacle').value else [])
        height = self.get_parameter('box_height').value

        def ranges(bearing_deg):
            return models.lidar_ranges(self.lidar, truth, obstacles, height,
                                       math.radians(bearing_deg))
        self.sender.catch_up(time.monotonic(), ranges, int(time.time() * 1e6))


def announce(node, name, topic):
    """Say once, latched, that this topic comes from a bench stand-in (/system/source)."""
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.source_pub = node.create_publisher(DataSource, '/system/source', latched)
    msg = DataSource(announcer=node.get_name())
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.sensors = [SensorSource(name=name, topic=topic, source=SensorSource.BENCH)]
    node.source_pub.publish(msg)


def run(node_class):
    rclpy.init()
    node = None
    try:
        node = node_class()
        announce(node, *node_class.ANNOUNCE)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def imu():
    run(MockImu)


def gps():
    run(MockGps)


def battery():
    run(MockBattery)


def camera():
    run(MockCamera)


def radar():
    run(MockRadar)


def lidar():
    run(MockLidar)
