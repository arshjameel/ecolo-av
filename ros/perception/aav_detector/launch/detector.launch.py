"""
The object detector, on the camera's images: RF-DETR by default, or YOLO.

  pixi run detect                       # RF-DETR Nano, on /sensors/camera/front_left/image_raw
  pixi run detect picture:=ros/perception/aav_detector/test/stop_sign.jpg   # a test picture
  pixi run detect detector:=yolo        # YOLO instead (models/yolo11n.pt)
  pixi run detect detector:=yolo model:=models/best.pt labels:="stop sign"

What you can set:
  detector    rfdetr (the default: RF-DETR, Apache-2.0) or yolo (YOLO, AGPL-3.0)
  model       the model file; empty (the default) = the detector's own:
              rfdetr: models/rf-detr-nano.pth (or rf-detr-small.pth...)
              yolo:   models/yolo11n.pt (or a .mlpackage, Core ML; or a .engine, TensorRT)
  device      auto (the default): the GPU this computer has, CUDA or a Mac's Metal, else
              the CPU. Or PyTorch's own names: cuda, cuda:N, mps (Metal) or cpu. A .engine
              or .mlpackage model picks its own hardware
  confidence  the lowest score kept, 0 to 1
  labels      the labels to keep, comma-separated; empty keeps them all
  image       the camera topic to read
  extra_images  more camera topics for the same model, comma-separated (the rear camera:
              /sensors/camera/rear/image_raw); empty, the default: none
  compressed  true: read <image>/compressed (JPEG) instead: for a laptop on the car's WiFi
  picture     an image or video file, published as the camera (image_publisher), for tests
  node_name   the node's name: object_detector, or object_detector_<machine> on the car. One
              detector per computer, each under its own name, all on /perception/detections
  use_sim_time  true in Gazebo (sim.launch.py): the detector is a process, not a launch_ros
              Node, so launch_ros's SetParameter doesn't reach it
  detector_impl  python (the default: aav_detector, PyTorch) or rust (aav_detector_rs, a
              challenger: RF-DETR's ONNX through ONNX Runtime, models/rf-detr-nano.onnx;
              device: auto, cpu, coreml, tensorrt; raw pictures only. docs/onnx.md)

The detector runs as `python3 -m aav_detector.detector`: the python3 of the environment that
started the launch, so the ml environment's PyTorch is the one it finds. The Rust one is
aav_detector_rs's `detector`, with the same arguments, name and topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    arg = LaunchConfiguration
    picture = PythonExpression(["'", arg('picture'), "' != ''"])
    rust = PythonExpression(["'", arg('detector_impl'), "'.strip().lower() == 'rust'"])

    def ros_args(node_name):
        # The same for both: the node's name, its parameters and its topics.
        return ['--ros-args',
                '-r', ['__node:=', node_name],
                '-p', ['detector:=', arg('detector')],
                '-p', ['model:="', arg('model'), '"'],
                '-p', ['device:=', arg('device')],
                '-p', ['confidence:=', arg('confidence')],
                '-p', ['labels:="', arg('labels'), '"'],
                '-p', ['compressed:=', arg('compressed')],
                '-p', ['extra_images:="', arg('extra_images'), '"'],
                '-p', ['use_sim_time:=', arg('use_sim_time')],
                '-r', ['image:=', arg('image')],
                '-r', 'detections:=/perception/detections',
                '-r', 'detections/image:=/perception/detections/image']

    return LaunchDescription([
        DeclareLaunchArgument('detector', default_value='rfdetr',
                              description='rfdetr (RF-DETR) or yolo (YOLO)'),
        DeclareLaunchArgument('model', default_value='',
                              description="the model file; empty = the detector's own"),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('labels', default_value=''),
        DeclareLaunchArgument('image', default_value='/sensors/camera/front_left/image_raw'),
        DeclareLaunchArgument('extra_images', default_value='',
                              description='more camera topics, comma-separated'),
        DeclareLaunchArgument('compressed', default_value='false',
                              description='read <image>/compressed (JPEG), for WiFi'),
        DeclareLaunchArgument('picture', default_value=''),
        DeclareLaunchArgument('node_name', default_value='object_detector'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('detector_impl', default_value='python',
                              description='python (aav_detector) or rust (aav_detector_rs, '
                              'ONNX Runtime: a challenger)'),
        Node(
            package='image_publisher', executable='image_publisher_node', name='test_camera',
            parameters=[{'filename': arg('picture'), 'publish_rate': 10.0,
                         'frame_id': 'camera_left_link'}],
            remappings=[('image_raw', arg('image')),
                        ('camera_info', '/sensors/camera/front_left/camera_info')],
            condition=IfCondition(picture),
        ),
        ExecuteProcess(
            name=arg('node_name'), output='screen', emulate_tty=True,
            cmd=['python3', '-m', 'aav_detector.detector'] + ros_args(arg('node_name')),
            condition=UnlessCondition(rust),
        ),
        ExecuteProcess(
            name=arg('node_name'), output='screen', emulate_tty=True,
            cmd=[ExecutableInPackage('detector', 'aav_detector_rs')] + ros_args(arg('node_name')),
            condition=IfCondition(rust),
        ),
    ])
