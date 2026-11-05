"""
The camera Pi's MediaMTX settings (pi/mediamtx.yml), and MediaMTX reading them.

The decided stream, and that MediaMTX itself accepts the file.

MediaMTX refuses a file with a key it doesn't know (`json: unknown field`), so where
vendor/mediamtx is downloaded (pixi run get mediamtx) the real program loads it, on
private local ports. Off a Pi it can't open the cameras: that error is expected.
"""

import os
import socket
import subprocess
import time

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'pi', 'mediamtx.yml')
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
MEDIAMTX = os.path.join(ROOT, 'vendor', 'mediamtx', 'mediamtx')


def settings():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def test_the_decided_stream():
    """Ben, 23 Oct: H.264, 1280x720, 15 fps, about 2 Mbit/s, a key frame every 1 s."""
    d = settings()['pathDefaults']
    assert (d['rpiCameraWidth'], d['rpiCameraHeight'], d['rpiCameraFPS']) == (1280, 720, 15)
    assert d['rpiCameraCodec'] == 'softwareH264'          # the Pi 5 has no H.264 hardware
    assert d['rpiCameraH264Profile'] == 'baseline'        # no B-frames
    assert d['rpiCameraBitrate'] == 2_000_000
    assert d['rpiCameraIDRPeriod'] == d['rpiCameraFPS']   # one key frame a second


def test_both_front_cameras_on_rtsp_8554():
    s = settings()
    paths = s['paths']
    assert {name: p['source'] for name, p in paths.items()} == {
        'front_left': 'rpiCamera', 'front_right': 'rpiCamera'}
    assert sorted(p['rpiCameraCamID'] for p in paths.values()) == [0, 1]
    assert s['rtsp'] is True and s['rtspAddress'] == ':8554'
    assert 'tcp' in s['rtspTransports']


def test_only_the_pi_publishes():
    users = settings()['authInternalUsers']
    for user in users:
        actions = {p['action'] for p in user['permissions']}
        if 'publish' in actions or 'api' in actions:
            assert set(user['ips']) <= {'127.0.0.1', '::1'}


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.mark.skipif(not os.access(MEDIAMTX, os.X_OK),
                    reason='needs vendor/mediamtx (pixi run get mediamtx)')
def test_mediamtx_accepts_it(tmp_path):
    s = settings()
    # The same file on private ports, on this computer only: RTSP over TCP alone and no
    # WebRTC, so no UDP ports open (their keys are still read and checked).
    s.update(rtspAddress=f'127.0.0.1:{free_port()}', rtspTransports=['tcp'],
             apiAddress=f'127.0.0.1:{free_port()}', webrtc=False)
    path = tmp_path / 'mediamtx.yml'
    path.write_text(yaml.safe_dump(s))
    proc = subprocess.Popen([MEDIAMTX, str(path)], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        time.sleep(1.5)
        alive = proc.poll() is None
    finally:
        proc.terminate()
        out = proc.communicate(timeout=10)[0]
    assert alive, out
    assert 'configuration loaded' in out
    assert 'unknown field' not in out
