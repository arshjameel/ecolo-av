"""
The CARLA car's sensors, made to look like ours. No ROS and no `carla` imports.

Each sensor is a CARLA blueprint attached where vehicle.yaml puts ours, with settings
that match Gazebo's stand-ins (aav_description aav.sensors.gazebo.xacro), so the rest of
the stack can't tell the simulators apart:

  name                blueprint             link               our topic
  imu                 sensor.other.imu      imu_link           /sensors/imu/data_raw
  camera_front_left   sensor.camera.rgb     camera_left_link   /sensors/camera/front_left/...
  camera_front_right  sensor.camera.rgb     camera_right_link  /sensors/camera/front_right/...
  camera_rear         sensor.camera.rgb     camera_rear_link   /sensors/camera/rear/...
  radar               sensor.other.radar    radar_link         /sensors/radar/points
  lidar               sensor.lidar.ray_cast lidar_link         /sensors/lidar/points

GPS is not a CARLA sensor here: carla_adapter makes it from where the antenna (gps_link)
really is, in the map frame of frames.SpawnFrame, plus noise. CARLA's own GNSS gives
the town's latitude and longitude (often 0, 0), not our datum's.

The raw data functions take CARLA's buffers (sensor data .raw_data) as bytes, so they
are tested without CARLA.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

# The bench camera's picture and lens, as Gazebo's cameras (aav.sensors.gazebo.xacro).
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360
CAMERA_FOCAL = 343.0                  # px
SENSOR_TICK = 0.1                     # s: 10 Hz, as Gazebo's cameras, radar and lidar


def fov_deg(width: int, focal: float) -> float:
    """Return the horizontal field of view (degrees) of a pinhole camera."""
    return math.degrees(2 * math.atan(width / 2.0 / focal))


@dataclass(frozen=True)
class Sensor:
    """One CARLA sensor on the car."""

    name: str              # as on /system/source
    blueprint: str
    link: str              # vehicle.yaml's sensors: entry, and the message's frame_id
    topic: str             # ours: the image topic for a camera
    attributes: dict = field(default_factory=dict)   # blueprint attributes, as strings


def _strings(d):
    return {k: str(v) for k, v in d.items()}


CAMERA = _strings({'image_size_x': CAMERA_WIDTH, 'image_size_y': CAMERA_HEIGHT,
                   'fov': round(fov_deg(CAMERA_WIDTH, CAMERA_FOCAL), 3),
                   'sensor_tick': SENSOR_TICK})
# The TI radar as Gazebo's stand-in: 120 degrees wide, 8 high, out to 8.4 m.
RADAR = _strings({'horizontal_fov': 120, 'vertical_fov': 8, 'range': 8.4,
                  'points_per_second': 1500, 'sensor_tick': SENSOR_TICK})
# A 16-beam roof lidar like a VLP-16, as Gazebo's: 1800 x 16 rays at 10 Hz, to 100 m.
LIDAR = _strings({'channels': 16, 'range': 100, 'upper_fov': 15, 'lower_fov': -15,
                  'rotation_frequency': 10, 'points_per_second': 1800 * 16 * 10,
                  'sensor_tick': SENSOR_TICK})
IMU = _strings({'sensor_tick': 0.01})            # 100 Hz, as the real IMU is read

CAMERAS = (
    Sensor('camera_front_left', 'sensor.camera.rgb', 'camera_left_link',
           '/sensors/camera/front_left/image_raw', CAMERA),
    Sensor('camera_front_right', 'sensor.camera.rgb', 'camera_right_link',
           '/sensors/camera/front_right/image_raw', CAMERA),
    Sensor('camera_rear', 'sensor.camera.rgb', 'camera_rear_link',
           '/sensors/camera/rear/image_raw', CAMERA),
)
IMU_SENSOR = Sensor('imu', 'sensor.other.imu', 'imu_link', '/sensors/imu/data_raw', IMU)
RADAR_SENSOR = Sensor('radar', 'sensor.other.radar', 'radar_link',
                      '/sensors/radar/points', RADAR)
LIDAR_SENSOR = Sensor('lidar', 'sensor.lidar.ray_cast', 'lidar_link',
                      '/sensors/lidar/points', LIDAR)

# What carla_adapter itself publishes, whatever sensors it attaches.
ALWAYS = [
    ('vehicle', '/vehicle/status'),
    ('odometry', '/vehicle/odom'),
    ('wheels', '/vehicle/wheels'),
    ('gps', '/sensors/gps/fix'),
]


def chosen(cameras: bool = True, radar: bool = True, lidar: bool = True) -> list[Sensor]:
    """Return the CARLA sensors to attach: the IMU always, the others as asked."""
    out = [IMU_SENSOR]
    if cameras:
        out += list(CAMERAS)
    if radar:
        out.append(RADAR_SENSOR)
    if lidar:
        out.append(LIDAR_SENSOR)
    return out


def sources(sensors: list[Sensor]) -> list[tuple[str, str]]:
    """(name, topic) of everything carla_adapter publishes, for /system/source (all SIM)."""
    return list(ALWAYS) + [(s.name, s.topic) for s in sensors]


def info_topic(image_topic: str) -> str:
    """Return a camera's camera_info topic, next to its image_raw."""
    return image_topic.rsplit('/', 1)[0] + '/camera_info'


def camera_matrix(width: int, height: int, fov: float) -> dict:
    """
    Return a CARLA camera's lens as sensor_msgs/CameraInfo's fields.

    A CARLA camera's lens as sensor_msgs/CameraInfo's fields: a perfect pinhole (no
    distortion), with the focal length from its horizontal field of view (degrees).
    """
    f = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    cx, cy = width / 2.0, height / 2.0
    return {'width': width, 'height': height, 'distortion_model': 'plumb_bob',
            'd': [0.0] * 5,
            'k': [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
            'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'p': [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]}


def bgra_to_bgr(raw: bytes, width: int, height: int) -> bytes:
    """Return a CARLA camera picture (BGRA) as bgr8, the encoding ours use."""
    pixels = np.frombuffer(raw, dtype=np.uint8)
    if pixels.size != width * height * 4:
        raise ValueError(f'{pixels.size} bytes is not a {width} x {height} BGRA picture')
    return pixels.reshape(height, width, 4)[:, :, :3].tobytes()


def lidar_points(raw: bytes) -> np.ndarray:
    """
    Return a CARLA lidar sweep as N x 4 float32 in ROS axes.

    A CARLA lidar sweep (x, y, z, intensity: float32 each, CARLA axes, in the sensor's
    frame) as N x 4 float32 in ROS axes: y flips.
    """
    points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4).copy()
    points[:, 1] *= -1.0
    return points


def radar_points(raw: bytes, intensity: float = 1.0) -> np.ndarray:
    """
    Return a CARLA radar's detections as the TI driver's points.

    A CARLA radar's detections as the TI driver's points: x, y, z, intensity, velocity
    (N x 5 float32, ROS axes, radar_link), as aav_sim_models's radar.FIELDS.

    Each CARLA detection is velocity (m/s, along the ray), azimuth and altitude (rad;
    azimuth + to the right, Unreal's y) and depth (m), float32 each. The point goes along
    the ray; y flips, as everywhere. The velocity passes on: CARLA and the TI radar both
    read negative for something coming closer (to confirm on a server: docs/carla.md).
    """
    d = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)
    velocity, azimuth, altitude, depth = d[:, 0], d[:, 1], d[:, 2], d[:, 3]
    out = np.empty((len(d), 5), dtype=np.float32)
    out[:, 0] = depth * np.cos(altitude) * np.cos(azimuth)
    out[:, 1] = -depth * np.cos(altitude) * np.sin(azimuth)
    out[:, 2] = depth * np.sin(altitude)
    out[:, 3] = intensity
    out[:, 4] = velocity
    return out


def wheel_speeds(speed: float, yaw_rate: float, wheelbase: float, track: float) -> dict:
    """
    Return each wheel's ground speed (m/s) from the car's motion.

    Each wheel's ground speed (m/s), from the car's motion: a rigid body rolling without
    slip (CARLA's Python API has no wheel speeds). speed is the rear axle's centre
    (base_link) forward speed, yaw_rate rad/s + left, both cars' sizes our vehicle.yaml's.
    So /vehicle/wheels agrees with /vehicle/odom by design: it can't catch a skid.
    """
    half = track / 2.0
    rear_left = speed - yaw_rate * half
    rear_right = speed + yaw_rate * half
    # A front wheel moves as the point it sits on: base_link's velocity plus the turn.
    lateral = yaw_rate * wheelbase                  # leftward speed at the front axle
    front_left = math.copysign(math.hypot(speed - yaw_rate * half, lateral), speed)
    front_right = math.copysign(math.hypot(speed + yaw_rate * half, lateral), speed)
    return {'front_left': front_left, 'front_right': front_right,
            'rear_left': rear_left, 'rear_right': rear_right}
