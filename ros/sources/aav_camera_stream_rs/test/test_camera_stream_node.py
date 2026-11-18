"""
The camera_stream node itself, Rust and Python side by side.

A test stream in, the same pictures, lens, detections and diagnostics out.

The stream: GStreamer's test pattern (`videotestsrc`, its colour fields, 15 fps, live) as the
decoder's `pipeline`, so the node runs its real child program and pipe, with no camera Pi
and no RTSP server. Each node runs on its own ROS domain (200-219), on this computer
only, with a parameters file under its own name, as camera_stream.launch.py writes it;
a 2024-25 calibration (aav_sensors' left.ini) makes it undistort. The Python modules
(aav_camera_stream's lens.py, with OpenCV) give the picture and lens it must publish.

aav_camera_stream's modules are read from its source folder beside this one: that package
depends on this one (its launch file starts either), so colcon doesn't put it in this
package's test environment. Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

SOURCES = Path(__file__).resolve().parents[2]
PYTHON = SOURCES / 'aav_camera_stream'
DETECTOR = SOURCES.parent / 'perception' / 'aav_detector'
LEFT_INI = SOURCES / 'aav_sensors' / 'config' / 'cameras' / 'left.ini'
sys.path[:0] = [str(PYTHON)]

try:
    from aav_camera_stream import lens as lenses
    from aav_interfaces.msg import DataSource
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    import numpy as np
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from vision_msgs.msg import Detection2DArray
except ImportError:
    rclpy = None

GST = shutil.which('gst-launch-1.0')
pytestmark = pytest.mark.skipif(rclpy is None or GST is None,
                                reason='needs ROS sourced (colcon test) and gst-launch-1.0')

WIDTH, HEIGHT = 320, 180
CAPS = (f'video/x-raw,width={WIDTH},height={HEIGHT},framerate=15/1 ! videoconvert '
        '! video/x-raw,format=BGR')
PIPELINE = f'videotestsrc is-live=true pattern=colors ! {CAPS} ! fdsink sync=false'


def environment():
    """Return this test's environment: its own domain, this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(200 + os.getpid() % 20),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    env['PYTHONPATH'] = os.pathsep.join(
        [str(PYTHON), str(DETECTOR), env.get('PYTHONPATH', '')])
    return env


def command(impl, params):
    if impl == 'rust':
        start = [os.path.join(get_package_prefix('aav_camera_stream_rs'), 'lib',
                              'aav_camera_stream_rs', 'camera_stream')]
    else:
        start = [sys.executable, '-m', 'aav_camera_stream.camera_stream']
    return start + ['--ros-args', '-r', '__node:=camera_stream_front_left',
                    '--params-file', str(params)]


def reference():
    """One raw picture of the test pattern, as the decoder gives it (BGR)."""
    raw = subprocess.run([GST, '-q', 'videotestsrc', 'num-buffers=1', 'pattern=colors', '!',
                          *CAPS.split(), '!', 'fdsink'], capture_output=True, check=True,
                         timeout=60).stdout
    return np.frombuffer(raw[:WIDTH * HEIGHT * 3], np.uint8).reshape(HEIGHT, WIDTH, 3)


def stop(process):
    """Stop a process group: SIGINT, then SIGKILL; gone already is fine."""
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGKILL, 5)):
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


class Watch:
    """Everything the node publishes, by topic."""

    def __init__(self, node):
        self.got = {}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        base = '/sensors/camera/front_left'
        for kind, topic, qos in [
                (Image, f'{base}/image_raw', reliable),
                (CameraInfo, f'{base}/camera_info', reliable),
                (Detection2DArray, '/perception/detections', reliable),
                (CompressedImage, '/perception/camera/front_left/preview/compressed', reliable),
                (DiagnosticArray, '/diagnostics', reliable),
                (DataSource, '/system/source', latched)]:
            self.got[topic] = []
            node.create_subscription(kind, topic, lambda m, t=topic: self.keep(t, m), qos)

    def keep(self, topic, msg):
        # Only this node's report: another test's nodes may share the domain.
        if topic != '/diagnostics' or any(
                s.name == 'camera_stream: front_left' for s in msg.status):
            self.got[topic].append((time.monotonic(), msg))


@pytest.mark.parametrize('impl', ['rust', 'python'])
def test_a_test_stream_comes_out_undistorted(impl, tmp_path):
    if impl == 'python':
        pytest.importorskip('cv2')
    env = environment()
    params = tmp_path / 'params.yaml'
    params.write_text(
        'camera_stream_front_left:\n  ros__parameters:\n'
        '    camera: front_left\n    url: rtsp://127.0.0.1:1/front_left\n'
        '    frame_id: camera_left_link\n    decoder: gst-soft\n'
        f'    program: {GST}\n    pipeline: "{PIPELINE}"\n'
        f'    width: {WIDTH}\n    height: {HEIGHT}\n    fps: 15.0\n'
        f'    calibration: {LEFT_INI}\n    delay: 0.05\n    detector: none\n'
        '    preview_period: 0.5\n    preview_width: 160\n    publish_image: true\n'
        '    source: bench\n')
    log = open(tmp_path / 'node.log', 'w')
    node = subprocess.Popen(command(impl, params), env=env, stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    me = rclpy.create_node(f'camera_stream_test_{impl}')
    watch = Watch(me)
    try:
        start = time.monotonic()
        images = watch.got['/sensors/camera/front_left/image_raw']
        while time.monotonic() - start < 60 and (
                len(images) < 50 or not watch.got['/diagnostics']):
            rclpy.spin_once(me, timeout_sec=0.1)
            assert node.poll() is None, (tmp_path / 'node.log').read_text()
        assert len(images) >= 50, (tmp_path / 'node.log').read_text()

        # The rate: the stream's 15 fps, over the last 30 pictures.
        times = [t for t, _ in images[-30:]]
        assert 12.0 < 29 / (times[-1] - times[0]) < 18.0
        # The picture: bgr8, the stream's size, undistorted as the Python's OpenCV does it.
        picture = images[-1][1]
        assert (picture.width, picture.height, picture.encoding, picture.step) == (
            WIDTH, HEIGHT, 'bgr8', 3 * WIDTH)
        assert picture.header.frame_id == 'camera_left_link'
        lens = lenses.scaled(lenses.load(str(LEFT_INI)), WIDTH, HEIGHT)
        undistorter = lenses.Undistorter(lens, 0.0)
        want = undistorter.apply(reference())
        got = np.frombuffer(bytes(picture.data), np.uint8).reshape(HEIGHT, WIDTH, 3)
        diff = np.abs(got.astype(int) - want.astype(int))
        assert diff.max() <= 2 and (diff > 0).mean() < 0.001, (diff.max(), (diff > 0).mean())

        # The lens, with every picture: the undistorted one's K, no distortion.
        info = watch.got['/sensors/camera/front_left/camera_info'][-1][1]
        assert (info.width, info.height, info.distortion_model) == (WIDTH, HEIGHT, 'plumb_bob')
        assert np.allclose(info.k, undistorter.lens().k, rtol=1e-12, atol=1e-9)
        assert list(info.d) == [0.0] * 5
        assert np.allclose(info.p, lenses.projection(undistorter.lens()))

        # A detections message per picture, empty (detector none), stamped as the picture.
        detections = watch.got['/perception/detections']
        assert len(detections) >= 0.9 * len(images)
        assert all(not d.detections for _, d in detections)
        assert detections[-1][1].header.frame_id == 'camera_left_link'

        # The preview: a JPEG, 160 px wide.
        previews = watch.got['/perception/camera/front_left/preview/compressed']
        assert previews and previews[-1][1].format == 'jpeg'
        assert bytes(previews[-1][1].data[:2]) == b'\xff\xd8'

        # The report and the announcement, as the Python's.
        status = watch.got['/diagnostics'][-1][1].status[0]
        values = {kv.key: kv.value for kv in status.values}
        assert status.name == 'camera_stream: front_left'
        assert status.hardware_id == 'rtsp://127.0.0.1:1/front_left'
        assert values['connected'] == 'true' and values['decoder'] == 'gst-soft'
        assert values['delay_s'] == '0.050' and values['decoder_starts'] == '1'
        assert set(values) >= {'frames_in', 'fps_in', 'fps_out', 'dropped', 'work_ms'}
        source = watch.got['/system/source'][-1][1]
        assert source.announcer == 'camera_stream_front_left'
        assert source.sensors[0].name == 'camera_front_left'
        assert source.sensors[0].topic == '/sensors/camera/front_left/image_raw'
        assert source.sensors[0].source == 2                          # BENCH
    finally:
        me.destroy_node()
        rclpy.shutdown()
        stop(node)
        log.close()


def test_a_detector_is_refused_with_the_way_back(tmp_path):
    params = tmp_path / 'params.yaml'
    params.write_text('camera_stream_front_left:\n  ros__parameters:\n'
                      '    detector: rfdetr\n')
    out = subprocess.run(command('rust', params), env=environment(), capture_output=True,
                         text=True, timeout=60)
    assert out.returncode != 0
    assert 'camera_stream_impl:=python' in out.stdout + out.stderr
