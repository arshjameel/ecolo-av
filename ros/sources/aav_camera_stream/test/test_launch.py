"""The launch file's parameters: each camera's URL, frame and lens, with the right types."""

import importlib.util
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, '..', 'launch', 'camera_stream.launch.py')
try:
    import aav_description  # noqa: F401
except ImportError:             # tools/test.sh: the workspace isn't sourced
    sys.path.append(os.path.join(HERE, '..', '..', '..', 'bringup', 'aav_description'))
try:
    import launch  # noqa: F401
    import launch_ros  # noqa: F401
    HAVE_LAUNCH = True
except ImportError:
    HAVE_LAUNCH = False
pytestmark = pytest.mark.skipif(not HAVE_LAUNCH, reason='needs ROS launch')


def load():
    spec = importlib.util.spec_from_file_location('camera_stream_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_one_nodes_parameters():
    m = load()
    values = {name: default for name, default, _ in m.ARGS}
    values.update(host='192.168.1.108', labels='stop sign', decoder='gst-nvidia')
    p = m.node_parameters('front_left', {'frame_id': 'camera_left_link',
                                         'calibration': 'cal/left.yaml'}, values, '/repo')
    assert p['url'] == 'rtsp://192.168.1.108:8554/front_left'
    assert p['frame_id'] == 'camera_left_link'
    assert p['calibration'] == '/repo/cal/left.yaml'
    assert p['labels'] == 'stop sign' and p['decoder'] == 'gst-nvidia'
    assert p['width'] == 1280 and isinstance(p['width'], int)
    assert p['delay'] == pytest.approx(0.06) and isinstance(p['delay'], float)
    assert p['publish_image'] is False and p['time_code'] is False
    assert 'host' not in p and 'cameras' not in p


def test_no_calibration_means_no_undistortion():
    m = load()
    values = {name: default for name, default, _ in m.ARGS}
    p = m.node_parameters('front_right', {'frame_id': 'camera_right_link', 'calibration': ''},
                          values, '/repo')
    assert p['calibration'] == ''


def test_the_rust_node_by_default_the_python_one_with_a_model():
    m = load()
    values = {name: default for name, default, _ in m.ARGS}
    assert values['camera_stream_impl'] == 'rust'
    assert 'camera_stream_impl' not in m.node_parameters('front_left', {}, values, '/repo')
    python = m.node_command('python', 'camera_stream_front_left', '/tmp/p.yaml')
    assert python == ['python3', '-m', 'aav_camera_stream.camera_stream', '--ros-args',
                      '-r', '__node:=camera_stream_front_left', '--params-file', '/tmp/p.yaml']
    from ament_index_python.packages import PackageNotFoundError
    try:
        rust = m.node_command('rust', 'camera_stream_front_left', '/tmp/p.yaml')
    except PackageNotFoundError:
        return                  # not built and sourced (tools/test.sh)
    assert rust[0].endswith(os.path.join('lib', 'aav_camera_stream_rs', 'camera_stream'))
    assert rust[1:] == python[3:]
