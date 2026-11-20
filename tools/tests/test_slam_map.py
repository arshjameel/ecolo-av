"""slam_map: poses flattened, GPS into the map, the fits, the 2D map, map errors, merging."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ndt_map  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import slam_map as sm  # noqa: E402

try:
    import scipy  # noqa: F401
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False
needs_scipy = pytest.mark.skipif(not HAVE_SCIPY, reason='scipy is not here')

DATUM = (45.3860, -75.6960)


def pose_matrix(x, y, z, yaw, pitch=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    m = np.eye(4)
    m[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    m[:3, 3] = (x, y, z)
    return m


def test_flatten_keeps_x_y_and_the_heading_of_a_pitched_pose():
    x, y, yaw = sm.flatten(pose_matrix(3.0, -2.0, -0.9, 1.2, pitch=0.05))
    assert (x, y) == pytest.approx((3.0, -2.0))
    assert yaw == pytest.approx(1.2)


def test_compose_and_wrap():
    assert sm.compose((1.0, 0.0, math.pi / 2), (2.0, 0.0, math.pi / 2)) == pytest.approx(
        (1.0, 2.0, -math.pi))
    assert sm.wrap(3 * math.pi) == pytest.approx(-math.pi)


def test_a_pose_between_two_takes_the_short_way_round():
    stamps = [0.0, 1.0]
    poses = [(0.0, 0.0, math.radians(170)), (2.0, 4.0, math.radians(-170))]
    x, y, yaw = sm.at(stamps, poses, 0.5)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert abs(yaw) == pytest.approx(math.pi)
    assert sm.at(stamps, poses, 1.0) == poses[1]
    assert sm.at(stamps, poses, 1.5) is None


def test_utm_matches_geographiclib():
    # GeoConvert's answers, as aav_hud's map-core earth.rs tests them.
    e, n, zone = sm.utm(*DATUM)
    assert zone == 18
    assert (e, n) == pytest.approx((445514.1385, 5026067.1450), abs=1e-3)
    assert math.degrees(sm.convergence(*DATUM)) == pytest.approx(-0.49546284270, abs=1e-6)


def test_gps_to_map_is_navsat_transforms():
    # earth.rs's Datum::to_map answers (navsat_transform's three steps).
    assert sm.gps_to_map(*DATUM, DATUM) == pytest.approx((0.0, 0.0), abs=1e-9)
    near = pytest.approx((-93.8754, 3944.0251), abs=2e-3)
    assert sm.gps_to_map(45.4215, -75.6972, DATUM) == near
    near = pytest.approx((-109.6068, -277.7464), abs=2e-3)
    assert sm.gps_to_map(45.3835, -75.6974, DATUM) == near
    x, _ = sm.gps_to_map(DATUM[0] + 0.01, DATUM[1], DATUM)
    assert abs(x) < 0.01            # due north is straight up the map


def test_good_fix_needs_a_fix_and_a_small_spread():
    assert sm.good_fix(0, [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.1])
    assert not sm.good_fix(-1, [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.1])
    assert not sm.good_fix(2, [4.0, 0, 0, 0, 4.0, 0, 0, 0, 9.0])     # 2 m
    assert not sm.good_fix(0, [0.0] * 9)                              # unknown
    assert not sm.good_fix(0, None)


def test_fit_2d_finds_the_turn_and_the_move():
    rng = np.random.default_rng(1)
    source = rng.uniform(-30, 30, (50, 2))
    truth = (12.0, -4.0, math.radians(33))
    x, y, yaw = sm.fit_2d(source, sm.apply_2d(truth, source))
    assert (x, y, yaw) == pytest.approx(truth)
    with pytest.raises(ValueError):
        sm.fit_2d(source[:1], source[:1])


def drive(n=100):
    """Return a path east then north, every 0.1 s: (stamps, poses)."""
    stamps = [i * 0.1 for i in range(n)]
    half = n // 2
    poses = [(i * 0.5, 0.0, 0.0) if i < half else (half * 0.5, (i - half) * 0.5, math.pi / 2)
             for i in range(n)]
    return stamps, poses


def test_georeference_by_gps_with_noise():
    stamps, poses = drive()
    truth = (100.0, 50.0, math.radians(-20))
    rng = np.random.default_rng(2)
    fixes = []
    for t, p in zip(stamps[::2], poses[::2]):
        gx, gy = sm.apply_2d(truth, [p[:2]])[0] + rng.normal(0, 0.3, 2)
        fixes.append((t, gx, gy))
    transform, how = sm.georeference(stamps, poses, fixes, start=(0, 0, 0))
    assert how['method'] == 'gps' and how['fixes_used'] == 50
    assert transform[:2] == pytest.approx(truth[:2], abs=0.2)
    assert transform[2] == pytest.approx(truth[2], abs=math.radians(0.5))
    assert how['residual_median_m'] < 0.5


def test_georeference_falls_back_to_the_start_pose_then_to_none():
    stamps, poses = drive()
    poses = [sm.compose((5.0, 1.0, 0.3), p) for p in poses]      # SLAM didn't start at 0
    start = (10.0, 20.0, math.pi / 2)
    few = [(stamps[i], 0.0, 0.0) for i in range(5)]
    transform, how = sm.georeference(stamps, poses, few, start)
    assert how['method'] == 'start pose' and '5 good GPS fixes' in how['why']
    assert sm.compose(transform, poses[0]) == pytest.approx(start)
    transform, how = sm.georeference(stamps, poses, [], None)
    assert transform == (0.0, 0.0, 0.0) and how['method'] == 'none: local'


def test_tum_round_trip(tmp_path):
    stamps, poses = drive(10)
    sm.write_tum(tmp_path / 'p.tum', stamps, poses)
    back = ndt_map.read_tum(tmp_path / 'p.tum')
    assert len(back) == 10
    t, (x, y, z), q = back[-1]
    assert (t, x, y, z) == pytest.approx((0.9, 2.5, 2.0, 0.0))
    assert 2 * math.atan2(q[2], q[3]) == pytest.approx(math.pi / 2, abs=1e-5)
    truth = ndt_map.Truth(back)
    assert truth.at(0.9)[0][1] == pytest.approx(2.0)


def wall_cloud():
    """Return a wall 10 m ahead (x = 10, 1 m tall), and the ground up to it, in base_link."""
    ys = np.linspace(-5, 5, 200)
    wall = np.stack([np.full_like(ys, 10.0), ys, np.full_like(ys, 1.0)], axis=1)
    gx, gy = np.meshgrid(np.linspace(1, 9.5, 40), np.linspace(-5, 5, 40))
    ground = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    return np.concatenate([wall, ground])


def cell_of(grid, origin, cell, x, y):
    return grid[int((y - origin[1]) // cell), int((x - origin[0]) // cell)]


def test_occupancy_marks_the_wall_and_frees_the_way_to_it(tmp_path):
    occ = sm.Occupancy(((-20, -20), (20, 20)), cell=0.25)
    for _ in range(3):
        occ.add(wall_cloud(), (2.0, 3.0, 0.0))
    whole = occ.grid()
    assert cell_of(whole, occ.origin, 0.25, 12.1, 3.0) == sm.PGM_OCCUPIED     # the wall
    assert cell_of(whole, occ.origin, 0.25, 7.0, 3.0) == sm.PGM_FREE          # on the way
    assert cell_of(whole, occ.origin, 0.25, 15.0, 3.0) == sm.PGM_UNKNOWN      # behind it
    grid, origin = occ.trimmed()                       # cut to what was seen, 1 m round
    assert grid.shape[0] < whole.shape[0] and grid.shape[1] < whole.shape[1]
    assert cell_of(grid, origin, 0.25, 12.1, 3.0) == sm.PGM_OCCUPIED
    assert origin == (round(origin[0]), round(origin[1]))                     # whole metres
    text = occ.write(tmp_path / 'm', note='test')
    assert 'mode: trinary' in text and 'resolution: 0.25' in text
    blob = (tmp_path / 'm.pgm').read_bytes()
    assert blob.startswith(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())


def test_occupancy_forgets_a_thing_seen_once():
    occ = sm.Occupancy(((-20, -20), (20, 20)), cell=0.25)
    occ.add(wall_cloud(), (0.0, 0.0, 0.0))
    for _ in range(5):
        cloud = wall_cloud()
        occ.add(cloud[200:], (0.0, 0.0, 0.0))           # the wall gone: only ground
    grid, origin = occ.trimmed()
    assert cell_of(grid, origin, 0.25, 10.1, 0.0) != sm.PGM_OCCUPIED


def test_occupancy_turns_with_the_car():
    occ = sm.Occupancy(((-20, -20), (20, 20)), cell=0.25)
    for _ in range(2):
        occ.add(wall_cloud(), (0.0, 0.0, math.pi / 2))   # facing north: the wall at y = 10
    grid, origin = occ.trimmed()
    assert cell_of(grid, origin, 0.25, 0.1, 10.1) == sm.PGM_OCCUPIED


@needs_scipy
def test_map_error_and_agreed():
    rng = np.random.default_rng(3)
    a = rng.uniform(0, 20, (2000, 3))
    error = sm.map_error(a + [0.05, 0.0, 0.0], a)
    assert error['median_m'] == pytest.approx(0.05, abs=0.01)
    parked = np.array([[50.0, 50.0, 1.0], [50.1, 50.0, 1.0]])
    both = sm.agreed(np.concatenate([a, parked]), a + 0.01)
    assert len(both) == 4000                               # the parked car dropped out
    assert both[:, 0].max() < 21


def test_grid_agreement_counts_cells_a_cell_apart_as_the_same():
    a = np.full((10, 10), sm.PGM_FREE, np.uint8)
    b = a.copy()
    a[5, 2:8] = sm.PGM_OCCUPIED              # a wall
    b[6, 2:8] = sm.PGM_OCCUPIED              # the same wall, one cell north
    b[0, 0] = sm.PGM_OCCUPIED                # something only b has
    assert sm.grid_agreement(a, b) == {'found': round(6 / 7, 3), 'right': 1.0}
    assert sm.grid_agreement(a, b, reach=0) == {'found': 0.0, 'right': 0.0}


def test_a_2d_map_written_reads_back(tmp_path):
    grid = np.full((5, 8), sm.PGM_UNKNOWN, np.uint8)
    grid[2:4, 3:6] = sm.PGM_FREE
    grid[0, 7] = sm.PGM_OCCUPIED                 # row 0: the south edge
    sm.write_map2d(tmp_path / 'map2d', grid, (-3.0, 4.0), 0.25, note='a test')
    back, origin, cell = sm.read_map2d(tmp_path / 'map2d.yaml')
    assert (back == grid).all() and origin == (-3.0, 4.0) and cell == 0.25


def test_trim_cuts_on_whole_metres():
    grid = np.full((40, 40), sm.PGM_UNKNOWN, np.uint8)
    grid[21, 22] = sm.PGM_OCCUPIED
    cut, origin = sm.trim(grid, (10.0, 20.0), 0.25, margin=0.5)
    assert origin[0] % 1.0 == 0.0 and origin[1] % 1.0 == 0.0
    assert (cut == sm.PGM_OCCUPIED).sum() == 1
