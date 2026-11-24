"""
camera_stream.launch.py's values reach the node it starts, Rust (the default) or Python.

A launch value must not be lost on its way (a Rust node can let a file's entry under its
own name beat launch's `/**` ones): this starts the real launch file with a test stream
(GStreamer's videotestsrc as the decoder's pipeline) and reads the node's diagnostics and
pictures, which show the URL, decoder, delay and size it was given. Its own ROS domain
(200-219), this computer only; stopped by its process group. Needs the workspace built
and sourced: `colcon test` runs it.
"""

import os
import shutil
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
except ImportError:
    rclpy = None

GST = shutil.which('gst-launch-1.0')


def installed():
    """Whether both nodes are built and sourced (not under tools/test.sh)."""
    try:
        return all(get_package_prefix(p) for p in ('aav_camera_stream', 'aav_camera_stream_rs'))
    except (LookupError, ValueError):
        return False


pytestmark = pytest.mark.skipif(
    rclpy is None or GST is None or not shutil.which('ros2') or not installed(),
    reason='needs the workspace built and sourced (colcon test) and gst-launch-1.0')

PIPELINE = ('videotestsrc is-live=true pattern=colors ! video/x-raw,width={width},'
            'height={height},framerate=15/1 ! videoconvert ! video/x-raw,format=BGR '
            '! fdsink sync=false')


def stop(process):
    """Stop a process group: SIGINT, then SIGKILL; gone already is fine."""
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except (ProcessLookupError, PermissionError):
            break               # macOS: EPERM for a group whose members have all exited
        try:
            process.wait(wait)
            break
        except subprocess.TimeoutExpired:
            continue
    process.wait(5)


@pytest.mark.parametrize('impl', ['rust', 'python'])
def test_launch_values_reach_the_node(impl, tmp_path):
    if impl == 'python':
        pytest.importorskip('cv2')
    ros_domain = os.environ.get('AAV_TEST_DOMAIN') or str(200 + (os.getpid() + 7) % 20)
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain,
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    log = open(tmp_path / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'aav_camera_stream', 'camera_stream.launch.py',
         f'camera_stream_impl:={impl}', 'cameras:=front_left', 'host:=10.9.8.7', 'port:=8999',
         'decoder:=gst-soft', f'program:={GST}', f'pipeline:={PIPELINE}', 'width:=160',
         'height:=96', 'delay:=0.123', 'detector:=none', 'publish_image:=true',
         'source:=bench'],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    me = rclpy.create_node(f'camera_stream_launch_test_{impl}')
    got = {'diag': [], 'image': []}
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    me.create_subscription(DiagnosticArray, '/diagnostics', lambda m: got['diag'].extend(
        s for s in m.status if s.name == 'camera_stream: front_left'), reliable)
    me.create_subscription(Image, '/sensors/camera/front_left/image_raw',
                           lambda m: got['image'].append(m), reliable)
    try:
        start = time.monotonic()
        while time.monotonic() - start < 60 and not (got['diag'] and got['image']):
            rclpy.spin_once(me, timeout_sec=0.1)
        text = (tmp_path / 'launch.log').read_text()
        assert got['diag'] and got['image'], text
        status = got['diag'][-1]
        values = {kv.key: kv.value for kv in status.values}
        assert status.name == 'camera_stream: front_left'
        assert status.hardware_id == 'rtsp://10.9.8.7:8999/front_left'
        assert values['decoder'] == 'gst-soft' and values['delay_s'] == '0.123'
        assert (got['image'][-1].width, got['image'][-1].height) == (160, 96)
        # Which node ran: the Rust one says so in its log.
        assert ('(Rust)' in text) == (impl == 'rust'), text
    finally:
        me.destroy_node()
        rclpy.shutdown()
        stop(launch)
        log.close()
