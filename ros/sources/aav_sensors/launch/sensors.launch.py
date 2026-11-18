"""
Sensor drivers: each publishes raw data in its own frame; nothing here fuses anything.

  ros2 launch aav_sensors sensors.launch.py [imu:=true] [gps:=true] [radar:=false]
      [lidar:=false] [battery:=false] [gps_driver:=ublox]

  IMU     imu_node -> /sensors/imu/data_raw -> imu_filter_madgwick -> /sensors/imu/data
          imu_node is aav_imu_yahboom, in Rust (it replaced the 2025-26 C++ driver).
  GPS     one of three drivers, each -> /sensors/gps/fix (NavSatFix, frame gps_link):
            ublox        ublox_gps (KumarRobotics' ublox): a u-blox on /dev/aav/gps, as
                         the receiver is already set up (config/ublox_gps.yaml). The
                         default, Tentative until the receiver is known (Q18). In Pixi too
            nmea         nmea_navsat_driver: any NMEA receiver (gps_nmea.launch.py)
            ublox_dgnss  ublox_dgnss -> ublox_nav_sat_fix_hp: a u-blox F9P over libusb,
                         the 2024-25 choice. apt only: the car, not Pixi
  radar   aav_radar_ti pcl_pub -> /sensors/radar/points (PointCloud2, frame radar_link)
  lidar   the stock velodyne driver, for the roof's Velodyne VLP-16 (found at the 16 Oct
          audit: 192.168.1.201, sending UDP to the Jetson only):
            lidar_driver (velodyne_driver)   UDP packets -> /sensors/lidar/packets
            lidar_cloud (velodyne_pointcloud) -> /sensors/lidar/points (PointCloud2,
                                                 frame lidar_link)
          lidar_ip and lidar_port pick the sender: the lidar, or tools/fake_vlp16.py
  battery aav_battery_victron battery_monitor -> /vehicle/battery (BatteryState)

Frames (imu_link, gps_link, radar_link, lidar_link) come from aav_description; run its
description.launch.py too, or nothing can place these readings on the car.
Run this file on the machine the sensors are plugged into.
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Each GPS driver, and the package it needs.
GPS_DRIVERS = {'ublox': 'ublox_gps', 'nmea': 'nmea_navsat_driver',
               'ublox_dgnss': 'ublox_dgnss_node'}

# u-blox receiver settings for ublox_dgnss, as in its ublox_rover_hpposllh_navsatfix.launch.py
# (0.7.4) except the rate. The NavSatFix node needs HPPOSLLH, STATUS and COV.
UBLOX_DGNSS_PARAMS = {
    'DEVICE_FAMILY': 'F9P',             # TODO confirm the team's receiver (F9P, F9R or X20P)
    'DEVICE_SERIAL_STRING': '',         # '' = the first u-blox found on USB
    'FRAME_ID': 'gps_link',
    'CFG_USBOUTPROT_NMEA': False,
    'CFG_RATE_MEAS': 200,               # ms between measurements: 5 Hz. TODO tune
    'CFG_RATE_NAV': 1,                  # one solution per measurement
    'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1,
    'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1,
    'CFG_MSGOUT_UBX_NAV_COV_USB': 1,
    'CFG_MSGOUT_UBX_NAV_PVT_USB': 1,
}


def installed(package):
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def gps(driver, config, use_sim_time):
    """Return the GPS driver's actions: every driver publishes /sensors/gps/fix in gps_link."""
    if not installed(GPS_DRIVERS[driver]):
        hint = (' ublox_dgnss is apt only, for the car: gps_driver:=ublox runs anywhere'
                if driver == 'ublox_dgnss' else '')
        get_logger('sensors.launch.py').warning(
            f'gps: not started: {GPS_DRIVERS[driver]} is not installed here.{hint}')
        return []
    if driver == 'ublox':
        return [Node(
            package='ublox_gps', executable='ublox_gps_node', name='ublox_gps',
            parameters=[os.path.join(config, 'ublox_gps.yaml'), use_sim_time],
            remappings=[('fix', '/sensors/gps/fix'),
                        ('fix_velocity', '/sensors/gps/fix_velocity')],
        )]
    if driver == 'nmea':
        nmea = os.path.join(get_package_share_directory('aav_sensors'), 'launch',
                            'gps_nmea.launch.py')
        # Pass both: included files share launch arguments.
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(nmea),
                                         launch_arguments={'port': '/dev/aav/gps',
                                                           'baud': '9600'}.items())]
    # ublox_dgnss: the driver and the NavSatFix converter, in one process.
    return [ComposableNodeContainer(
        name='gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                parameters=[UBLOX_DGNSS_PARAMS, use_sim_time],
            ),
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                parameters=[use_sim_time],
                remappings=[('fix', '/sensors/gps/fix')],
            ),
        ],
    )]


def lidar(ip, port, use_sim_time):
    """Return the stock velodyne driver's two nodes, set up for the car's VLP-16."""
    calibration = os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params',
                               'VLP16db.yaml')
    return [
        # The UDP packets as they come, a scan's worth (one turn, 0.1 s) per message.
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='lidar_driver',
             parameters=[{'device_ip': ip, 'port': port, 'model': 'VLP16', 'rpm': 600.0,
                          'frame_id': 'lidar_link', 'gps_time': False}, use_sim_time],
             remappings=[('velodyne_packets', '/sensors/lidar/packets')]),
        # The packets as points: the VLP-16's 16 lasers, through its factory calibration.
        # Unorganised: rays that hit nothing are left out, as Gazebo's cloud has them.
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='lidar_cloud',
             parameters=[{'calibration': calibration, 'model': 'VLP16', 'min_range': 0.9,
                          'max_range': 100.0, 'organize_cloud': False, 'fixed_frame': '',
                          'target_frame': ''}, use_sim_time],
             remappings=[('velodyne_packets', '/sensors/lidar/packets'),
                         ('velodyne_points', '/sensors/lidar/points')]),
    ]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def on(name):
        # As IfCondition reads it: true or 1, false or 0; anything else is an error.
        return evaluate_condition_expression(context, [LaunchConfiguration(name)])

    def choice(name, options):
        value = arg(name)
        if value not in options:
            raise RuntimeError(f'{name}:={value}: pick one of {", ".join(options)}')
        return value

    config = os.path.join(get_package_share_directory('aav_sensors'), 'config')
    use_sim_time = {'use_sim_time': on('use_sim_time')}
    actions = []
    if on('imu'):
        actions += [
            Node(package='aav_imu_yahboom', executable='imu_node', name='imu_node', namespace='/',
                 parameters=[os.path.join(config, 'imu.yaml'), use_sim_time]),
            Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
                 name='imu_filter_madgwick',
                 parameters=[os.path.join(config, 'imu_filter.yaml'), use_sim_time],
                 remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                             ('imu/data', '/sensors/imu/data'),
                             ('imu/mag', '/sensors/imu/mag')]),
        ]
    if on('gps'):
        actions += gps(choice('gps_driver', GPS_DRIVERS), config, use_sim_time)
    if on('radar'):
        # Perception, later: the out-of-box cfg, which publishes x forward, y left.
        actions.append(Node(
            package='aav_radar_ti', executable='pcl_pub', name='radar',
            parameters=[{
                'cfg_path': os.path.join(get_package_share_directory('aav_radar_ti'),
                                         'cfg', 'base.cfg'),
                'command_port': '/dev/aav/radar_cmd',
                'data_port': '/dev/aav/radar_data',
                'frame_id': 'radar_link',
                'topic': '/sensors/radar/points',
            }, use_sim_time],
        ))
    if on('lidar'):
        actions += lidar(arg('lidar_ip'), int(arg('lidar_port')), use_sim_time)
    if on('battery'):
        # The Victron VE.Direct cable. Its `port` parameter defaults to the udev name
        # /dev/aav/battery.
        actions.append(Node(package='aav_battery_victron', executable='battery_monitor'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('radar', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='false',
                              description='the roof Velodyne VLP-16, through the stock '
                                          'velodyne driver'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.201',
                              description="the lidar's address: only its packets are read"),
        DeclareLaunchArgument('lidar_port', default_value='2368',
                              description='the UDP port the lidar sends its data to'),
        DeclareLaunchArgument('battery', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gps_driver', default_value='ublox',
                              description='ublox: ublox_gps; nmea: nmea_navsat_driver; '
                                          'ublox_dgnss: ublox_dgnss (apt only). Tentative: '
                                          'Q18'),
        OpaqueFunction(function=launch_setup),
    ])
