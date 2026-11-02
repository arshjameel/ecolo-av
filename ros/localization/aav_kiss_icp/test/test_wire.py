"""wire.py: the pipe's frames there and back; the worker with KISS-ICP, where it's installed."""

import io
import math
import os
import subprocess

from aav_kiss_icp import odometry, wire
import numpy as np
import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
PYTHON = odometry.find_python('', root=ROOT)
WORKER = os.path.join(os.path.dirname(os.path.abspath(wire.__file__)), 'worker.py')


def test_a_scan_with_times_goes_there_and_back():
    points = np.arange(12, dtype=float).reshape(4, 3)
    times = np.array([0.0, 0.25, 0.5, 1.0])
    got, got_times = wire.read_scan(io.BytesIO(wire.encode_scan(points, times)))
    assert got.tolist() == points.tolist()
    assert got_times.tolist() == times.tolist()


def test_a_scan_without_times_has_none():
    got, times = wire.read_scan(io.BytesIO(wire.encode_scan(np.ones((2, 3)))))
    assert got.shape == (2, 3) and times is None


def test_a_pose_goes_there_and_back():
    pose = np.arange(16, dtype=float).reshape(4, 4)
    got, ms = wire.read_pose(io.BytesIO(wire.encode_pose(pose, 12.5)))
    assert got.tolist() == pose.tolist() and ms == 12.5


def test_a_closed_pipe_says_so():
    with pytest.raises(EOFError):
        wire.read_pose(io.BytesIO(b'POSE' + b'\0' * 10))


def room(rng, count=20000):
    """Return points on the walls of a made-up lot: a 40 x 30 m box, two pillars, a slope."""
    out = []
    for _ in range(count):
        side, s, z = rng.integers(0, 6), rng.uniform(0, 1), rng.uniform(0.0, 3.0)
        if side == 0:
            out.append((-20 + 40 * s, -15, z))
        elif side == 1:
            out.append((-20 + 40 * s, 15 - 4 * s, z))
        elif side == 2:
            out.append((-20, -15 + 30 * s, z))
        elif side == 3:
            out.append((20, -15 + 26 * s, z))
        else:
            cx, cy = (5.0, 4.0) if side == 4 else (-8.0, -6.0)
            a = 2 * math.pi * s
            out.append((cx + math.cos(a), cy + math.sin(a), z))
    return np.array(out)


@pytest.mark.skipif(not os.path.isfile(PYTHON),
                    reason='needs the kiss environment: pixi install --locked -e kiss')
def test_the_worker_follows_a_car_driving_through_a_made_up_lot():
    walls = room(np.random.default_rng(1))
    worker = subprocess.Popen([PYTHON, '-I', WORKER], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, env={'PATH': os.environ.get('PATH', '')})
    try:
        worker.stdin.write(wire.encode_settings({
            'min_range': 0.0, 'max_range': 100.0, 'voxel_size': 0.5,
            'max_points_per_voxel': 20, 'threads': 1, 'deskew': True}))
        # 0.3 m a scan and 1 degree left, seen from the car: the walls move the other way.
        for k in range(10):
            yaw, x, y = math.radians(k), 0.3 * k * math.cos(math.radians(k / 2)), 0.0
            y = 0.3 * k * math.sin(math.radians(k / 2))
            car = np.eye(4)
            car[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
            car[:2, 3] = [x, y]
            seen = (walls - car[:3, 3]) @ car[:3, :3]
            worker.stdin.write(wire.encode_scan(seen))
            worker.stdin.flush()
            got, ms = wire.read_pose(worker.stdout)
            assert ms >= 0.0
            assert got[:2, 3] == pytest.approx([x, y], abs=0.02)
            assert odometry.yaw_of(got) == pytest.approx(yaw, abs=math.radians(0.2))
    finally:
        worker.stdin.close()
        worker.wait(timeout=10)
    assert worker.returncode == 0
