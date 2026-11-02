"""odometry.py: the cloud's bytes, the mount, the crop, the times, pose and twist. No ROS."""

import math

from aav_kiss_icp import odometry
import numpy as np
import pytest

F32, F64 = 7, 8


def cloud(rows, with_time=False):
    """Return (fields, data, point_step, count) for float32 x, y, z[, intensity, time]."""
    if with_time:
        fields = [('x', 0, F32), ('y', 4, F32), ('z', 8, F32), ('intensity', 12, F32),
                  ('ring', 16, 4), ('time', 18, F32)]
        dtype = np.dtype({'names': ['x', 'y', 'z', 'time'], 'formats': ['<f4'] * 4,
                          'offsets': [0, 4, 8, 18], 'itemsize': 22})
    else:
        fields = [('x', 0, F32), ('y', 4, F32), ('z', 8, F32)]
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                          'offsets': [0, 4, 8], 'itemsize': 16})
    data = np.zeros(len(rows), dtype=dtype)
    for i, row in enumerate(rows):
        for name, value in zip(dtype.names, row):
            data[name][i] = value
    return fields, data.tobytes(), dtype.itemsize, len(rows)


def test_xyz_are_read_and_the_rays_that_hit_nothing_left_out():
    points, times = odometry.cloud_points(*cloud([(1, 2, 3), (math.inf, 0, 0),
                                                  (4, 5, math.nan), (-1, 0.5, 0)]))
    assert points.tolist() == [[1, 2, 3], [-1, 0.5, 0]]
    assert times is None


def test_the_velodynes_time_field_comes_with_its_points():
    points, times = odometry.cloud_points(*cloud([(1, 0, 0, 0.0), (math.inf, 0, 0, 0.05),
                                                  (0, 1, 0, 0.09)], with_time=True))
    assert points.shape == (2, 3)
    assert times.tolist() == pytest.approx([0.0, 0.09])


def test_a_cloud_without_z_is_refused():
    with pytest.raises(ValueError, match='no z'):
        odometry.cloud_points([('x', 0, F32), ('y', 4, F32)], b'\0' * 8, 8, 1)


def test_the_mount_moves_the_scan_into_base_link():
    # A point 1 m ahead of a lidar 0.9 m ahead and 1.8 m up, turned 90 degrees left.
    out = odometry.to_base_link(np.array([[1.0, 0.0, 0.0]]), [0.9, 0.0, 1.8],
                                [0.0, 0.0, math.pi / 2])
    assert out[0].tolist() == pytest.approx([0.9, 1.0, 1.8])


def test_the_body_and_the_ranges_are_cut_out():
    points = np.array([[0.5, 0.0, 1.0],     # over the car: its roof
                       [5.0, 0.0, 1.8],     # 4.1 m from the lidar: kept
                       [0.9, 3.0, 1.8],     # 3 m to the side: kept
                       [200.0, 0.0, 1.8]])  # too far
    times = np.array([0.0, 0.1, 0.2, 0.3])
    kept, kept_times = odometry.crop(points, times, (-0.5, -0.7), (2.5, 0.7), 0.5, 100.0,
                                     (0.9, 0.0, 1.8))
    assert kept.tolist() == [[5.0, 0.0, 1.8], [0.9, 3.0, 1.8]]
    assert kept_times.tolist() == [0.1, 0.2]


def test_the_ground_is_cut_out_below_min_height():
    points = np.array([[8.0, 0.0, 0.02], [8.0, 1.0, 0.31], [9.0, 0.0, 2.0]])
    kept, _ = odometry.crop(points, None, (-0.5, -0.7), (2.5, 0.7), 0.5, 100.0,
                            (0.9, 0.0, 1.8), min_height=0.3)
    assert kept.tolist() == [[8.0, 1.0, 0.31], [9.0, 0.0, 2.0]]


def test_times_scale_to_one_turn_and_gazebos_single_instant_gives_none():
    assert odometry.sweep_times(np.array([0.02, 0.07, 0.12])).tolist() == pytest.approx(
        [0.0, 0.5, 1.0])
    assert odometry.sweep_times(np.zeros(5)) is None
    assert odometry.sweep_times(None) is None


def pose(x, y, yaw, z=0.0):
    m = np.eye(4)
    m[:3, :3] = odometry.rotation(0.0, 0.0, yaw)
    m[:3, 3] = [x, y, z]
    return m


def test_pose_of_gives_position_and_quaternion():
    (x, y, z), (qx, qy, qz, qw) = odometry.pose_of(pose(1.0, 2.0, 0.6, 0.1))
    assert (x, y, z) == pytest.approx((1.0, 2.0, 0.1))
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    assert odometry.yaw_of(pose(0, 0, -2.5)) == pytest.approx(-2.5)


@pytest.mark.parametrize('rpy', [(0.3, -0.2, 2.9), (3.0, 0.1, -1.0), (0.0, 0.0, math.pi)])
def test_quaternion_matches_the_matrix(rpy):
    r = odometry.rotation(*rpy)
    qx, qy, qz, qw = odometry.quaternion(r)
    assert qx * qx + qy * qy + qz * qz + qw * qw == pytest.approx(1.0)
    back = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    assert back == pytest.approx(r)


def test_the_twist_is_the_last_step_over_the_time():
    v, w = odometry.twist_of(pose(0.2, 0.01, 0.02), 0.1)
    assert v == pytest.approx((2.0, 0.1, 0.0))
    assert w == pytest.approx((0.0, 0.0, 0.2))
    assert odometry.twist_of(pose(1, 0, 0), 0.0) == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_clouds_dropped_come_from_the_stamps():
    assert odometry.dropped_between(None, 5.0, 0.1) == 0
    assert odometry.dropped_between(5.0, 5.1, 0.1) == 0
    assert odometry.dropped_between(5.0, 5.3, 0.1) == 2


def test_the_python_is_the_kiss_environments_unless_given(tmp_path):
    assert odometry.find_python('', root=str(tmp_path)) == str(
        tmp_path / '.pixi' / 'envs' / 'kiss' / 'bin' / 'python')
    assert odometry.find_python(str(tmp_path / 'py')) == str(tmp_path / 'py')
