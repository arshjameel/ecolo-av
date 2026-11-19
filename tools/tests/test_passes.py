"""passes: occupancy across drives; a parked car that went is removed, walls stay."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import passes as ps  # noqa: E402
import pytest  # noqa: E402
import slam_map as sm  # noqa: E402

try:
    import scipy  # noqa: F401
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False
needs_scipy = pytest.mark.skipif(not HAVE_SCIPY, reason='scipy is not here')

RANGE = 15.0
# A yard 40 x 20 m: four walls 0.5 m thick (x0, y0, x1, y1).
WALLS = [(0.0, -10.5, 40.0, -10.0), (0.0, 10.0, 40.0, 10.5),
         (-0.5, -10.5, 0.0, 10.5), (40.0, -10.5, 40.5, 10.5)]
PARKED = (18.0, -6.0, 22.5, -4.2)        # a car 4.5 x 1.8 m, 4 m south of the route


def ray_box(ox, oy, dx, dy, box):
    """Return the distance along a ray to a box, or inf (the slab test)."""
    x0, y0, x1, y1 = box
    near, far = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        a, b = (lo - o) / d, (hi - o) / d
        near, far = max(near, min(a, b)), min(far, max(a, b))
    return near if 0 < near <= far else math.inf


def scan(x, y, boxes, rays=720):
    """Return a cloud in base_link (the car at x, y heading east): each ray's nearest box."""
    out = []
    for k in range(rays):
        a = -math.pi + (k + 0.5) * 2 * math.pi / rays
        d = min(ray_box(x, y, math.cos(a), math.sin(a), b) for b in boxes)
        if d <= RANGE:
            for z in (0.5, 1.0, 1.4):
                out.append((d * math.cos(a), d * math.sin(a), z))
        else:       # the ground, far off: the ray was free all the way
            out.append((RANGE * math.cos(a), RANGE * math.sin(a), 0.0))
    return np.array(out)


def drive(boxes, xs, y=0.0):
    """Return one pass's Occupancy: the car along y, a cloud each metre."""
    grid = sm.Occupancy(((-2.0, -12.0), (42.0, 12.0)), 0.25, max_range=RANGE)
    for x in xs:
        grid.add(scan(x, y, boxes), (x, y, 0.0))
    return grid


def points_of(boxes, step=0.1):
    """Return NDT-like points on each box's outline, 0.5-1.4 m up."""
    out = []
    for x0, y0, x1, y1 in boxes:
        for x in np.arange(x0, x1 + 1e-9, step):
            for yy in (y0, y1):
                out += [(x, yy, z) for z in (0.5, 1.0, 1.4)]
        for yy in np.arange(y0, y1 + 1e-9, step):
            for x in (x0, x1):
                out += [(x, yy, z) for z in (0.5, 1.0, 1.4)]
    return np.array(out)


def in_box(points, box, pad=0.3):
    x0, y0, x1, y1 = box
    p = np.asarray(points)
    return (p[:, 0] >= x0 - pad) & (p[:, 0] <= x1 + pad) & \
        (p[:, 1] >= y0 - pad) & (p[:, 1] <= y1 + pad)


@pytest.fixture(scope='module')
def three_passes():
    route = np.arange(4.0, 37.0, 1.0)
    grids = [drive(WALLS + [PARKED], route), drive(WALLS, route), drive(WALLS, route)]
    occ = np.stack([ps.occupied(g.hits, g.free) for g in grids])
    free = np.stack([ps.seen_free(g.hits, g.free) for g in grids])
    return grids, occ, free, ps.classify(occ, free)


def parked_cells(grid, occ):
    """Return the cells the first pass saw occupied on the parked car (its near side)."""
    rows, cols = np.indices(grid.hits.shape)
    centres = np.stack([grid.origin[0] + (cols + 0.5) * grid.cell,
                        grid.origin[1] + (rows + 0.5) * grid.cell], axis=-1).reshape(-1, 2)
    return occ[0] & in_box(centres, PARKED).reshape(grid.hits.shape)


def wall_cells(grid):
    mask = np.zeros(grid.hits.shape, bool)
    for x0, y0, x1, y1 in WALLS[:2]:     # the long walls the car drives beside
        row, col, inside = ps.cells_of(points_of([(x0 + 6, y0, x1 - 6, y1)]),
                                       grid.origin, grid.cell, mask.shape)
        mask[row[inside], col[inside]] = True
    return mask


def test_a_car_parked_in_one_pass_only_is_removed(three_passes):
    grids, occ, _, classes = three_passes
    car = parked_cells(grids[0], occ)
    assert car.sum() > 15
    assert classes['removed'][car].all()
    assert not (classes['kept'] & ps.near(car, 1)).any()
    assert classes['free'][car].all()              # the 2D map: free where it stood


def test_the_walls_every_pass_saw_are_kept(three_passes):
    grids, _, _, classes = three_passes
    walls = wall_cells(grids[0])
    seen = walls & (classes['occupied_in'] > 0)
    assert seen.sum() > 100
    assert classes['kept'][seen].all()
    assert not classes['removed'][walls].any()


def test_the_wall_behind_the_parked_car_stays(three_passes):
    grids, _, _, classes = three_passes
    g = grids[0]
    behind = np.zeros(g.hits.shape, bool)
    row, col, inside = ps.cells_of(points_of([(18.0, -10.5, 22.5, -10.0)]), g.origin,
                                   g.cell, behind.shape)
    behind[row[inside], col[inside]] = True
    assert classes['kept'][behind & (classes['occupied_in'] > 0)].all()
    assert not classes['removed'][behind].any()


def test_ndt_points_of_the_parked_car_are_dropped_and_the_walls_kept(three_passes):
    grids, occ, _, classes = three_passes
    g = grids[0]
    # The first pass's points: everything it hit, 0.5-1.4 m up (scan's points, map frame).
    cloud = np.concatenate([scan(x, 0.0, WALLS + [PARKED]) + (x, 0.0, 0.0)
                            for x in np.arange(4.0, 37.0, 1.0)])
    cloud = cloud[cloud[:, 2] > 0.3]
    car = in_box(cloud, PARKED, pad=0.05)
    kept, gone = ps.drop(cloud, classes, g.origin, g.cell)
    assert car.sum() > 100 and gone >= car.sum()
    assert in_box(kept, PARKED, pad=0.05).sum() == 0
    walls = points_of(WALLS[:2])
    assert len(ps.drop(walls, classes, g.origin, g.cell)[0]) == len(walls)


def test_drop_keeps_points_above_the_top_and_kept_cells():
    classes = {'removed': np.zeros((4, 4), bool), 'kept': np.zeros((4, 4), bool)}
    classes['removed'][0, 0] = True
    classes['kept'][0, 2] = True
    points = np.array([[0.5, 0.5, 1.0], [0.5, 0.5, 5.0], [1.5, 0.5, 1.0], [2.5, 0.5, 1.0],
                       [3.5, 3.5, 1.0]])
    kept, gone = ps.drop(points, classes, (0.0, 0.0), 1.0, top=3.0)
    # Gone: the removed cell's low point and its unkept neighbour's; a kept cell stays.
    assert gone == 2
    assert kept.tolist() == [[0.5, 0.5, 5.0], [2.5, 0.5, 1.0], [3.5, 3.5, 1.0]]


def test_something_only_one_pass_saw_is_kept_and_labelled_with_that_pass():
    route = np.arange(4.0, 37.0, 1.0)
    post = (38.0, 2.0, 38.6, 2.6)          # by the east wall: only the long pass gets near
    # The long pass drives to x 36; the short ones stop at x 15: 23 m off, out of range.
    grids = [drive(WALLS + [post], route), drive(WALLS + [post], np.arange(4.0, 15.0)),
             drive(WALLS + [post], np.arange(4.0, 15.0))]
    occ = np.stack([ps.occupied(g.hits, g.free) for g in grids])
    free = np.stack([ps.seen_free(g.hits, g.free) for g in grids])
    classes = ps.classify(occ, free)
    g = grids[0]
    # Its west face, the side the car saw.
    row, col, _ = ps.cells_of(np.array([[38.05, 2.3]]), g.origin, g.cell, g.hits.shape)
    assert classes['kept'][row[0], col[0]] and classes['once'][row[0], col[0]]
    labels = ps.seen_once_labels(classes, occ, ['long', 'a', 'b'], g.origin, g.cell, 2)
    mine = [lb for lb in labels if abs(lb['region']['centre_m'][0] - 38.1) < 1.0
            and abs(lb['region']['centre_m'][1] - 2.3) < 1.0]
    assert mine and mine[0]['seen_in'] == ['long'] and mine[0]['source'] == 'passes'


def test_classify_on_small_grids():
    occ = np.zeros((3, 1, 5), bool)
    free = np.zeros((3, 1, 5), bool)
    occ[:, 0, 0] = True                    # all three: static
    occ[0, 0, 2], free[1, 0, 2] = True, True     # hit once, free once: dynamic
    occ[0, 0, 3] = True                    # next to it, seen once: goes with it
    occ[1, 0, 4] = True                    # far off? no: two cells from the dynamic one
    free[2, 0, 1] = True
    c = ps.classify(occ, free, grow=1)
    assert c['kept'][0].tolist() == [True, False, False, False, True]
    assert c['removed'][0].tolist() == [False, False, True, True, False]
    assert c['once'][0].tolist() == [False, False, False, False, True]
    assert c['free'][0].tolist() == [False, True, True, True, False]
    want = [sm.PGM_OCCUPIED, sm.PGM_FREE, sm.PGM_FREE, sm.PGM_FREE, sm.PGM_OCCUPIED]
    assert ps.to_pgm(c)[0].tolist() == want


def test_free_needs_rays_and_keeps_off_its_own_occupied_cells():
    hits = np.array([[0, 0, 0, 5, 0]])
    free = np.array([[2, 9, 9, 1, 9]])
    want = [[False, True, False, False, False]]
    assert ps.seen_free(hits, free, tolerance=1).tolist() == want


def test_near_does_not_wrap_round():
    mask = np.zeros((3, 4), bool)
    mask[0, 0] = True
    grown = ps.near(mask, 1)
    assert grown[1, 1] and not grown[2, 3] and not grown[0, 3] and not grown[2, 0]


def test_resample_moves_a_map_onto_other_cells():
    grid = np.full((4, 4), sm.PGM_FREE, np.uint8)
    grid[0, 0] = sm.PGM_OCCUPIED                 # the cell (0-1, 0-1)
    out = ps.resample(grid, (0.0, 0.0), 1.0, (2.0, 1.0, 0.0), (0.0, 0.0), (6, 6), 1.0)
    assert out[1, 2] == sm.PGM_OCCUPIED and out[0, 0] == sm.PGM_UNKNOWN
    turned = ps.resample(grid, (0.0, 0.0), 1.0, (0.0, 0.0, math.pi / 2), (-4.0, 0.0),
                         (4, 4), 1.0)
    assert turned[0, 3] == sm.PGM_OCCUPIED       # (0.5, 0.5) turned to (-0.5, 0.5)


@needs_scipy
def test_agreed_by_keeps_what_most_maps_saw():
    wall = np.array([[x, 0.0, 1.0] for x in np.arange(0.0, 5.0, 0.1)])
    car = np.array([[2.0, 3.0, 1.0], [2.1, 3.0, 1.0]])
    maps = [np.concatenate([wall, car]), wall + 0.02, wall - 0.02]
    both = ps.agreed_by(maps, radius=0.1, min_maps=2)
    assert len(both) == 3 * len(wall)
    assert len(ps.agreed_by(maps[:2], 0.1, 2)) == len(sm.agreed(maps[0], maps[1], 0.1))


def test_clusters_and_their_boxes():
    mask = np.zeros((5, 6), bool)
    mask[1:3, 1:3] = True
    mask[4, 5] = True
    found = ps.clusters(mask, min_cells=2)
    assert len(found) == 1 and len(found[0]) == 4
    assert ps.box_of(found[0], (10.0, 20.0), 0.5) == pytest.approx((11.0, 21.0, 1.0, 1.0))
