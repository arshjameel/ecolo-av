"""
The lane finder, on the front camera's pictures.

  ros2 launch aav_lane_finder lane_finder.launch.py [debug_image:=false]
      [lane_finder_impl:=rust]

  /sensors/camera/front_left/image_raw, /sensors/camera/front_left/camera_info
      -> lane_finder -> /perception/lane/geometry

It finds the lane; the costmap draws its lines as lane costs (lane_source:=camera, TODO
264) and aav_lane_pose reads them. sim.launch.py and bench.launch.py lane:=true start it
(sim's lanes:=camera too). The lane follower that steered by it went on 19 Nov.

lane_finder_impl: rust (aav_lane_finder_rs, the default since 18 Nov: the same node name,
topics, parameters and lanes, no OpenCV), python (this package's own node), or learned
(aav_lane_finder_learned, a challenger: our model through ONNX Runtime, the same message;
it needs models/lane-ufld.onnx, made by tools/lane_train.py).

The camera's place on the car (camera_left_link) and the wheelbase come from
aav_description's vehicle.yaml; the rest from config/lane_finder.yaml (both languages).
"""

import os

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def finder(context):
    """Return lane_finder, in the language lane_finder_impl picks."""
    impl = vehicle.check_lane_finder_impl(context.launch_configurations.get(
        'lane_finder_impl', vehicle.DEFAULT_LANE_FINDER_IMPL))
    car = vehicle.load()
    camera = car['sensors']['camera_left_link']
    share = FindPackageShare('aav_lane_finder').find('aav_lane_finder')
    # namespace '/': launch then writes the dict below under /lane_finder, not /**, so it
    # beats lane_finder.yaml in rclrs too (rclrs lets a file's node name beat /**, so
    # debug_image:=true was lost in the Rust node).
    return [Node(package=vehicle.LANE_FINDER_IMPLS[impl], executable='lane_finder',
                 name='lane_finder', namespace='/', output='screen',
                 parameters=[os.path.join(share, 'config', 'lane_finder.yaml'),
                             {'camera_xyz': [float(v) for v in camera['xyz']],
                              'camera_rpy': [float(v) for v in camera['rpy']],
                              'wheelbase': float(car['vehicle']['wheelbase']),
                              'debug_image': LaunchConfiguration('debug_image')}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug_image', default_value='false',
                              description="/perception/lane/image: the bird's-eye view, and "
                                          'what the finder found in it'),
        DeclareLaunchArgument('lane_finder_impl',
                              default_value=vehicle.DEFAULT_LANE_FINDER_IMPL,
                              description='lane_finder in rust (aav_lane_finder_rs, the '
                                          'default), python (aav_lane_finder) or learned '
                                          '(aav_lane_finder_learned, a challenger)'),
        OpaqueFunction(function=finder),
    ])
