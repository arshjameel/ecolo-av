"""ndt_map: the truth at a cloud's stamp, the points into the map, thinning, the files."""

import math
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ndt_map as m  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

MOUNT = ([0.9, 0.0, 1.8], [0.0, 0.0, 0.0])      # vehicle.yaml's lidar_link
BODY = m.body_box({'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2})


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_truth_is_interpolated_and_needs_samples_close_on_both_sides():
    truth = m.Truth([(1.00, (0.0, 0.0, 0.0), yaw_quat(0.0)),
                     (1.02, (0.2, 0.0, 0.0), yaw_quat(0.2)),
                     (1.10, (1.0, 0.0, 0.0), yaw_quat(0.2))])
    position, q = truth.at(1.01)
    assert position[0] == pytest.approx(0.1)
    assert 2.0 * math.atan2(q[2], q[3]) == pytest.approx(0.1)
    assert truth.at(1.06) is None       # 40 ms from each side
    assert truth.at(0.99) is None       # before the first
    assert truth.at(1.10)[0][0] == pytest.approx(1.0)


def test_slerp_takes_the_short_way_round():
    q = m.slerp(yaw_quat(math.radians(170)), yaw_quat(math.radians(-170)), 0.5)
    assert abs(2.0 * math.atan2(q[2], q[3])) == pytest.approx(math.pi)


def test_points_go_from_the_lidar_into_the_map_and_lose_ground_body_and_far():
    lidar = np.array([
        [4.1, 0.0, -0.8],           # a wall 5 m ahead of base_link, 1 m up: kept
        [4.1, 0.0, -1.75],          # the ground, 5 cm up
        [0.0, 0.0, -0.5],           # the car's own roof
        [50.0, 0.0, 0.0],           # past 45 m
        [np.inf, 0.0, 0.0],         # hit nothing
    ])
    # The car at (10, 20), facing north.
    pose = (np.array([10.0, 20.0, 0.0]), np.array(yaw_quat(math.pi / 2)))
    out = m.to_map(lidar, MOUNT, pose, BODY)
    assert out.shape == (1, 3)
    assert out[0] == pytest.approx([10.0, 25.0, 1.0])


def test_the_mount_turns_as_urdf_does():
    # A lidar turned to face left: its x is the car's y.
    out = m.to_map([[5.0, 0.0, 0.0]], ([0.0, 0.0, 1.0], [0.0, 0.0, math.pi / 2]),
                   (np.zeros(3), np.array(yaw_quat(0.0))))
    assert out[0] == pytest.approx([0.0, 5.0, 1.0])


def test_thinning_keeps_one_mean_point_per_cube():
    points = [[0.01, 0.01, 0.5], [0.03, 0.05, 0.5], [0.5, 0.5, 0.5]]
    out = m.thin(points, 0.1)
    assert len(out) == 2
    assert sorted(map(tuple, np.round(out, 6)))[0] == pytest.approx((0.02, 0.03, 0.5))


def test_a_cloud_is_read_from_its_bytes():
    # x, y, z float32, then intensity float32 and ring uint16: Gazebo's layout, padded.
    fields = [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1),
              ('ring', 16, 4, 1)]
    data = b''.join(struct.pack('<4fH2x', *p, 1.0, 3) for p in ([1, 2, 3], [4, 5, 6]))
    out = m.cloud_xyz(fields, 20, data, 2)
    assert out.tolist() == [[1, 2, 3], [4, 5, 6]]
    with pytest.raises(ValueError):
        m.cloud_xyz(fields[:2], 20, data, 2)


def test_the_pcd_round_trips(tmp_path):
    points = np.array([[1.0, -2.5, 0.75], [40.0, 3.0, 1.5]])
    path = tmp_path / 'map.pcd'
    m.write_pcd(path, points)
    assert m.read_pcd(path) == pytest.approx(points)
    assert path.read_bytes().startswith(b'# .PCD v0.7')


def test_a_build_skips_clouds_without_truth():
    truth = m.Truth([(t / 50.0, (t / 50.0, 0.0, 0.0), yaw_quat(0.0)) for t in range(51)])
    wall = np.array([[4.1, y, -0.8] for y in np.arange(-2, 2, 0.05)])
    points, notes = m.build([(0.5, wall), (5.0, wall)], truth, MOUNT, BODY)
    assert notes['clouds_used'] == 1 and notes['clouds_skipped_no_truth'] == 1
    assert points[:, 0] == pytest.approx(5.5)
    assert len(points) == 40


def test_the_route_is_not_a_scored_tour():
    tours = [(x, y) for x, y in [(15.0, 15.0), (35.0, 42.0), (20.0, 42.0), (30.0, 0.0)]]
    assert not any((x, y) in tours for _, x, y, _ in m.ROUTE)
