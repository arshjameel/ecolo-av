#!/usr/bin/env python3
"""
A map from a recorded drive (TODO 250, phase D): KISS-SLAM, then NDT's map and a 2D map.

    pixi run map-drive bags/<drive>             the drive's maps into bags/<drive>/map/
    pixi run map-drive bags/<drive> --out maps/drives/lot
    pixi run map-drive --merge maps/a maps/b [maps/c...] --out maps/lot   drives' maps
                                                fitted onto the first: NDT keeps what two
                                                saw; the 2D maps merged (parked cars drop out)
    pixi run map-area <area> <drive> <drive> <drive>   the whole job, from the drives' own
                                                rays (tools/map_area.py, TODO 259/260)

Runs in the slam environment (pixi install -e slam, once): no ROS. The bag's MCAP files are
read with mcap-ros2-support. The page: docs/maps-from-drives.md.

1. SLAM: every lidar cloud, moved into base_link (vehicle.yaml's mount: the drive's own
   copy if it has one), the car's body and the ground (below 0.3 m) cut out, into
   KISS-SLAM (MIT, PRBonn): KISS-ICP's odometry in local maps of --split metres,
   MapClosures' loop closures, then g2o's pose graph. Its poses are flattened to x, y, yaw
   (the stack is 2D; on the flat sim lot a 3D path drifted 1 m down in 91 m: the ground
   is cut, so nothing holds the height).
2. Where in the map: the path fitted to the good GPS fixes (least squares, slam_map.
   georeference), else put on the EKF's start pose (/localization/odom/global), else left
   in SLAM's own frame (the map's origin where the drive began). The YAML says which.
3. The maps, from the same clouds on those poses: NDT's (tools/ndt_map.py's rules: ground
   below 0.3 m out, 45 m, 10 cm cubes), and a 2D map for the planner (points 0.3-2 m
   above the ground occupied, the lidar's rays free, 0.25 m cells, a map server's PGM).
   Both cut into 64 m chunks (ndt-chunks, chunk-map) when cargo is here.
4. A Ravensim drive (/vehicle/sim/truth recorded) is scored: the SLAM path and the EKF's
   against the truth, and their maps against the map laid on the truth.

Writes, in --out: poses.tum (map frame), poses_slam.tum (SLAM's own), closures.yaml,
ndt.pcd/.yaml, map2d.pgm/.yaml, chunks/ (chunks.yaml, fine/, coarse/, ndt/), report.md;
scored: truth.pcd, ekf.pcd.

The maths: tools/slam_map.py (tools/tests/test_slam_map.py).
"""

import argparse
import datetime
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros' / 'localization' / 'aav_kiss_icp'))
import ndt_map  # noqa: E402, I100
from aav_description import vehicle  # noqa: E402, I100  (ndt_map put it on the path)
from aav_kiss_icp import odometry as od  # noqa: E402
import slam_map as sm  # noqa: E402, I100

CLOUDS = '/sensors/lidar/points'
GPS = '/sensors/gps/fix'
EKF = '/localization/odom/global'
TRUTH = '/vehicle/sim/truth'
NAVSAT = ROOT / 'ros' / 'localization' / 'aav_localization' / 'config' / 'navsat.yaml'
VEHICLE = ROOT / 'ros' / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml'
SPLIT = 30.0            # m between KISS-SLAM's local maps (its default, 100, suits cities)
CELL = 0.25             # m: the 2D map (the global costmap's cell)
START_GAP = 0.2         # s: the EKF's start pose this close to the first cloud


# --- The bag, without ROS -----------------------------------------------------------------

def mcap_files(path):
    """Return the drive's MCAP files in order: path is the drive's folder or its bag/."""
    path = Path(path)
    folder = path / 'bag' if (path / 'bag').is_dir() else path
    files = sorted(folder.glob('*.mcap'),
                   key=lambda p: int(p.stem.rsplit('_', 1)[-1]) if p.stem[-1].isdigit() else 0)
    if not files:
        raise SystemExit(f'{path}: no .mcap files (a mapping drive is MCAP: '
                         'pixi run record-drive)')
    return files


def messages(files, topics):
    """Yield (topic, stamp s, message) for the topics, file after file."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    for name in files:
        with open(name, 'rb') as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, channel, _, msg in reader.iter_decoded_messages(topics=list(topics)):
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                yield channel.topic, stamp, msg


def cloud_of(msg):
    """Return (points N x 3 in lidar_link, per-point times or None) of a PointCloud2."""
    return od.cloud_points([(f.name, f.offset, f.datatype) for f in msg.fields], msg.data,
                           msg.point_step, msg.width * msg.height, msg.is_bigendian)


def odom_pose(msg):
    """Return (x, y, yaw) of an Odometry message."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


# --- SLAM ---------------------------------------------------------------------------------

def run_slam(files, car, split, min_height):
    """
    Return (stamps, poses (x, y, yaw) in SLAM's frame, closures, extras).

    extras: the GPS fixes, the EKF's and the truth's poses seen on the way, by time.
    """
    from kiss_icp.preprocess import get_preprocessor
    from kiss_slam.config import load_config
    from kiss_slam.slam import KissSLAM
    config = load_config(None)
    config.local_mapper.splitting_distance = split
    config.odometry.preprocessing.max_range = 100.0
    config.odometry.preprocessing.min_range = 0.5
    slam = KissSLAM(config)
    # A cloud without per-point times (Ravensim's) must skip the deskew (aav_kiss_icp's
    # worker: equal times give KISS-ICP NaN). Two preprocessors, picked per cloud.
    deskew = slam.odometry.preprocessor
    config.odometry.preprocessing.deskew = False
    straight = get_preprocessor(config.kiss_icp_config())
    stamps, extras = [], {'gps': [], 'ekf': [], 'truth': []}
    started = time.monotonic()
    for topic, stamp, msg in messages(files, (CLOUDS, GPS, EKF, TRUTH)):
        if topic == GPS:
            extras['gps'].append((stamp, msg.latitude, msg.longitude, msg.status.status,
                                  list(msg.position_covariance)))
        elif topic in (EKF, TRUTH):
            extras['ekf' if topic == EKF else 'truth'].append((stamp, odom_pose(msg)))
        else:
            points, times = cloud_of(msg)
            base = od.to_base_link(points, car['xyz'], car['rpy'])
            base, times = od.crop(base, times, car['body_min'], car['body_max'], 0.5,
                                  100.0, car['xyz'], min_height)
            times = od.sweep_times(times)
            slam.odometry.preprocessor = deskew if times is not None else straight
            slam.process_scan(base, times if times is not None else np.zeros(len(base)))
            stamps.append(stamp)
            if len(stamps) % 500 == 0:
                print(f'   {len(stamps)} clouds, {time.monotonic() - started:.0f} s', flush=True)
    if not stamps:
        raise SystemExit(f'no {CLOUDS} in the bag: nothing to map')
    slam.generate_new_node()
    slam.local_map_graph.erase_last_local_map()
    keyposes = [sm.flatten(k) for k in slam.get_keyposes()]
    closures = [(int(a), int(b)) for a, b in slam.closures]
    matrices, _ = slam.fine_grained_optimization()
    poses = [sm.flatten(m) for m in matrices]
    if len(poses) != len(stamps):
        raise SystemExit(f'KISS-SLAM gave {len(poses)} poses for {len(stamps)} clouds')
    return stamps, poses, (closures, keyposes), extras


def nearest(samples, t, gap):
    """Return the value of the (t, value) sample nearest t, if within gap s, else None."""
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s[0] - t))
    return best[1] if abs(best[0] - t) <= gap else None


# --- The maps -----------------------------------------------------------------------------

def build_maps(files, car, stamps, placed, others, cell, voxel):
    """
    Return (NDT points, Occupancy, {name: NDT points}, the truth's Occupancy or None).

    others: {name: ndt_map.Truth} also mapped, for scoring (the truth, the EKF); the
    truth's 2D map too, on the same cells.
    """
    xy = np.array([(x, y) for x, y, _ in placed])
    reach = ndt_map.MAX_RANGE
    bounds = (xy.min(axis=0) - reach, xy.max(axis=0) + reach)
    grid = sm.Occupancy(bounds, cell, max_range=reach)
    truth_grid = sm.Occupancy(bounds, cell, max_range=reach) if 'truth' in others else None
    by_stamp = dict(zip(stamps, placed))
    mount = (car['xyz'], car['rpy'])
    body = (tuple(car['body_x']), tuple(car['body_y']))
    parts = {'slam': [], **{name: [] for name in others}}
    for _, stamp, msg in messages(files, (CLOUDS,)):
        pose = by_stamp.get(stamp)
        if pose is None:
            continue
        points, _ = cloud_of(msg)
        x, y, yaw = pose
        parts['slam'].append(ndt_map.to_map(points, mount, ((x, y, 0.0), sm.yaw_quat(yaw)),
                                            body))
        base = od.to_base_link(points, car['xyz'], car['rpy'])
        base, _ = od.crop(base, None, car['body_min'], car['body_max'], 0.5,
                          ndt_map.MAX_RANGE, car['xyz'])
        grid.add(base, pose, sensor=car['xyz'][:2])
        for name, truth in others.items():
            there = truth.at(stamp)
            if there is not None:
                parts[name].append(ndt_map.to_map(points, mount, there, body))
                if name == 'truth':
                    (tx, ty, _), q = there
                    truth_grid.add(base, (tx, ty, 2 * np.arctan2(q[2], q[3])),
                                   sensor=car['xyz'][:2])
        for name, got in parts.items():       # thin as we go (ndt_map.build's way)
            if len(got) >= 50:
                parts[name] = [ndt_map.thin(np.concatenate(got), voxel)]
    clouds = {name: ndt_map.thin(np.concatenate(got), voxel) if got else np.zeros((0, 3))
              for name, got in parts.items()}
    return clouds.pop('slam'), grid, clouds, truth_grid


def chunks(out, origin):
    """Cut the 2D map and NDT's into 64 m chunks; return what was made."""
    if shutil.which('cargo') is None:
        return 'skipped: no cargo here (the whole maps are written)'
    folder = out / 'chunks'
    made = subprocess.run(
        ['cargo', 'run', '-q', '--release', '--locked', '--bin', 'chunk-map', '--',
         str(out / 'map2d.yaml'), str(folder)],
        cwd=ROOT / 'ros' / 'bringup' / 'aav_description' / 'map-chunks')
    made_ndt = subprocess.run(
        ['cargo', 'run', '-q', '--release', '--locked', '--bin', 'ndt-chunks', '--',
         str(out / 'ndt.pcd'), str(folder / 'ndt'), str(origin[0]), str(origin[1])],
        cwd=ROOT / 'ros' / 'localization' / 'aav_ndt')
    if made.returncode or made_ndt.returncode:
        return 'FAILED: see the cargo output above'
    return f'{folder}: chunks.yaml, fine/, coarse/, ndt/'


def path_error(stamps, poses, truth):
    """Return {'median_m', 'p95_m', 'max_m', 'yaw_median_deg'} of poses against truth."""
    off, turn = [], []
    for t, (x, y, yaw) in zip(stamps, poses):
        there = truth.at(t)
        if there is None:
            continue
        (tx, ty, _), q = there
        off.append(np.hypot(x - tx, y - ty))
        turn.append(abs(sm.wrap(yaw - 2 * np.arctan2(q[2], q[3]))))
    if not off:
        return None
    return {'median_m': round(float(np.median(off)), 3),
            'p95_m': round(float(np.percentile(off, 95)), 3),
            'max_m': round(float(np.max(off)), 3),
            'yaw_median_deg': round(float(np.degrees(np.median(turn))), 2)}


def as_truth(samples):
    """Return [(t, (x, y, yaw))] as an ndt_map.Truth (z 0)."""
    return ndt_map.Truth([(t, (x, y, 0.0), sm.yaw_quat(yaw)) for t, (x, y, yaw) in samples])


def lidar_car(path):
    """Return the lidar's mount and the car's box from a vehicle.yaml."""
    v = vehicle.load(path)
    sensor = v['sensors']['lidar_link']
    (x0, x1), (y0, y1) = ndt_map.body_box(v)
    gps = v['sensors'].get('gps_link', {}).get('xyz', [0.0, 0.0, 0.0])
    return {'xyz': [float(a) for a in sensor['xyz']], 'rpy': [float(a) for a in sensor['rpy']],
            'body_x': (x0, x1), 'body_y': (y0, y1), 'body_min': (x0, y0), 'body_max': (x1, y1),
            'antenna': (float(gps[0]), float(gps[1]))}


# --- The command --------------------------------------------------------------------------

def make(args):
    """Map one drive."""
    started = time.monotonic()
    drive = Path(args.drive)
    files = mcap_files(drive)
    own = drive / 'vehicle.yaml'
    car = lidar_car(own if own.exists() else VEHICLE)
    out = (Path(args.out) if args.out else drive / 'map').resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f'== SLAM: {len(files)} file(s) of {drive}', flush=True)
    stamps, poses, (closures, keyposes), extras = run_slam(files, car, args.split,
                                                           args.min_height)
    print(f'   {len(stamps)} clouds, {len(keyposes)} local maps, {len(closures)} loop '
          f'closure(s), {time.monotonic() - started:.0f} s', flush=True)

    datum = args.datum or yaml.safe_load(NAVSAT.read_text())[
        'navsat_transform']['ros__parameters']['datum'][:2]
    fixes = [] if args.no_gps else [
        (t, *sm.gps_to_map(lat, lon, datum)) for t, lat, lon, status, cov in extras['gps']
        if sm.good_fix(status, cov, args.gps_sd)]
    start = nearest(extras['ekf'], stamps[0], START_GAP)
    transform, how = sm.georeference(stamps, poses, fixes, start, car['antenna'])
    placed = [sm.compose(transform, p) for p in poses]
    print(f'   placed in the map by: {how["method"]}' +
          (f' ({how["why"]})' if 'why' in how else
           f' ({how["fixes_used"]} fixes, residual median {how["residual_median_m"]} m)'),
          flush=True)
    sm.write_tum(out / 'poses.tum', stamps, placed)
    sm.write_tum(out / 'poses_slam.tum', stamps, poses)
    (out / 'closures.yaml').write_text(yaml.safe_dump({
        'local_maps': len(keyposes), 'split_m': args.split,
        'closures': [{'local_maps': [a, b],
                      'at_m': [[round(v, 2) for v in sm.compose(transform, keyposes[i])[:2]]
                               for i in (a, b) if i < len(keyposes)]}
                     for a, b in closures]}, sort_keys=False))

    scored = bool(extras['truth'])
    others = {}
    if scored:
        others = {'truth': as_truth(extras['truth']), 'ekf': as_truth(extras['ekf'])}
    print('== The maps' + (" (and the truth's and the EKF's, to score)" if scored else ''),
          flush=True)
    points, grid, more, truth_grid = build_maps(files, car, stamps, placed, others,
                                                args.cell, args.voxel)
    ndt_map.write_pcd(out / 'ndt.pcd', points)
    place = {'method': how['method'], **{k: v for k, v in how.items() if k != 'method'},
             'slam_to_map': {'x_m': round(transform[0], 3), 'y_m': round(transform[1], 3),
                             'yaw_deg': round(float(np.degrees(transform[2])), 3)},
             'datum': [float(d) for d in datum]}
    lo, hi = points.min(axis=0), points.max(axis=0)
    (out / 'ndt.yaml').write_text(yaml.safe_dump({
        'frame': 'map' if how['method'] != 'none: local' else 'slam (local: not in the map)',
        'placed': place, 'poses': 'KISS-SLAM (poses.tum)', 'source_bag': str(drive),
        'git_commit': ndt_map.git_commit(), 'voxel_m': args.voxel, 'points': int(len(points)),
        'ground_removed_below_m': ndt_map.MIN_HEIGHT, 'max_range_m': ndt_map.MAX_RANGE,
        'bounds_m': {'min': [round(float(v), 2) for v in lo],
                     'max': [round(float(v), 2) for v in hi]},
        'clouds': len(stamps), 'loop_closures': len(closures),
        'made': datetime.datetime.now().isoformat(timespec='seconds')}, sort_keys=False))
    grid.write(out / 'map2d', note=(
        f'tools/map_drive.py from {drive.name}: occupied 0.3-2 m above the ground, the '
        f'lidar\'s rays free. Placed by {how["method"]}; slam_to_map {place["slam_to_map"]}'))
    with open(out / 'map2d.yaml', 'a') as f:
        f.write(yaml.safe_dump({'placed': place}, sort_keys=False))
    origin = yaml.safe_load((out / 'map2d.yaml').read_text())['origin'][:2]
    cut = 'skipped (--no-chunks)' if args.no_chunks else chunks(out, origin)

    rows = []
    if scored:
        for name, got in more.items():
            ndt_map.write_pcd(out / f'{name}.pcd', got)
        truth = others['truth']
        rows.append(('KISS-SLAM (this)', path_error(stamps, placed, truth),
                     sm.map_error(points, more['truth'])))
        ekf = [nearest(extras['ekf'], t, 0.05) for t in stamps]
        kept = [(t, p) for t, p in zip(stamps, ekf) if p is not None]
        rows.append((f'EKF ({EKF})', path_error([t for t, _ in kept], [p for _, p in kept],
                                                truth), sm.map_error(more['ekf'], more['truth'])))
        rows.append(('2D map', sm.grid_agreement(grid.grid(), truth_grid.grid()), None))
    report = page(drive, out, len(stamps), len(keyposes), closures, how, place, len(points),
                  grid, cut, rows, time.monotonic() - started)
    (out / 'report.md').write_text(report)
    print(report)
    return 0


def page(drive, out, clouds, local_maps, closures, how, place, points, grid, cut, rows,
         seconds):
    """Return report.md: what was made, how it was placed, and the scores if any."""
    lines = [f'# A map from {drive.name}', '',
             f'{clouds} clouds; {local_maps} local maps; **{len(closures)} loop closure(s)**; '
             f'{seconds:.0f} s.', '', '| File | What |', '|---|---|',
             f"| `{out}/ndt.pcd` | NDT's map: {points:,} points, 10 cm |",
             f'| `{out}/map2d.pgm` | the 2D map: {grid.trimmed()[0].shape[1]} x '
             f'{grid.trimmed()[0].shape[0]} cells of {grid.cell} m |',
             f'| `{out}/poses.tum` | the path, map frame (TUM) |',
             f'| chunks | {cut} |', '', '## Where in the map', '',
             f'By **{how["method"]}**: x {place["slam_to_map"]["x_m"]} m, '
             f'y {place["slam_to_map"]["y_m"]} m, yaw {place["slam_to_map"]["yaw_deg"]} deg.']
    if 'why' in how:
        lines.append(f'Not by GPS: {how["why"]}.')
    else:
        lines.append(f'{how["fixes_used"]} GPS fixes; off them after the fit: median '
                     f'{how["residual_median_m"]} m, 95 % {how["residual_95_m"]} m.')
    if rows:
        lines += ['', "## Against the simulator's truth", '',
                  "| Poses | Path off: median | 95 % | max | Map off the truth's map: "
                  'median | 95 % |', '|---|---|---|---|---|---|']
        for name, path, cloud in rows[:-1]:
            p = path or {'median_m': '-', 'p95_m': '-', 'max_m': '-'}
            lines.append(f'| {name} | {p["median_m"]} m | {p["p95_m"]} m | {p["max_m"]} m | '
                         f'{cloud["median_m"]} m | {cloud["p95_m"]} m |')
        agree = rows[-1][1]
        lines += ['', "The 2D map against one made the same way on the truth's poses: "
                  f'{agree["found"]:.0%} of the truth\'s occupied cells found (within a '
                  f'cell), {agree["right"]:.0%} of this map\'s occupied cells right.']
    return '\n'.join(lines) + '\n'


def icp_onto(reference, points, reach=1.0):
    """
    Return ((x, y, yaw), info): the move that puts points' map on reference's (ICP).

    Open3D's point-to-plane ICP, from no move. Both maps are flat (x, y, yaw poses, the
    ground at z 0): only the fit's x, y and yaw are kept, or a small tilt or lift from the
    walls would move every point. info: the fit's numbers, for the YAML.
    """
    import open3d as o3d
    clouds = []
    for cloud_points in (points, reference):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud_points))
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        clouds.append(cloud)
    fit = o3d.pipelines.registration.registration_icp(
        clouds[0], clouds[1], reach, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    m = fit.transformation
    x, y, yaw = sm.flatten(m)
    return (x, y, yaw), {'x_m': round(x, 3), 'y_m': round(y, 3),
                         'icp_z_m_dropped': round(float(m[2, 3]), 3),
                         'yaw_deg': round(float(np.degrees(yaw)), 3),
                         'icp_fitness': round(fit.fitness, 3),
                         'icp_rmse_m': round(fit.inlier_rmse, 3)}


def moved(points, transform):
    """Return N x 3 points turned and moved by (x, y, yaw); z kept."""
    return np.column_stack([sm.apply_2d(transform, points[:, :2]), points[:, 2]])


def merge(args):
    """
    Two or more drives' maps into one, fitted onto the first by ICP.

    NDT's map: the points at least --min-maps of the maps have nearby (default 2: for two
    drives, only what both saw). The 2D maps: each moved onto one grid, then the rule of
    occupancy across passes (passes.classify) on their cells: occupied in one map and free
    in another is gone (free); seen by one only, kept. map-area does it properly, from the
    drives' own rays (TODO 259); this works on map folders alone.
    """
    import passes as ps
    dirs = [Path(p) for p in args.merge]
    if len(dirs) < 2:
        raise SystemExit('--merge needs two map folders or more')
    maps = [ndt_map.read_pcd(d / 'ndt.pcd') for d in dirs]
    fits = [((0.0, 0.0, 0.0), {'reference': True})]
    for d, points in zip(dirs[1:], maps[1:]):
        fits.append(icp_onto(maps[0], points, args.icp_reach))
        print(f'   {d}: onto {dirs[0]}: {fits[-1][1]}', flush=True)
    placed = [moved(points, t) for points, (t, _) in zip(maps, fits)]
    both = ndt_map.thin(ps.agreed_by(placed, args.radius, args.min_maps), args.voxel)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ndt_map.write_pcd(out / 'ndt.pcd', both)

    grid_note = ''
    grids = [d / 'map2d.yaml' for d in dirs]
    if all(g.exists() for g in grids):
        read = [sm.read_map2d(g) for g in grids]
        cell = read[0][2]
        corners = []
        for (grid, origin, c), (t, _) in zip(read, fits):
            h, w = grid.shape
            x0, y0, x1, y1 = origin[0], origin[1], origin[0] + w * c, origin[1] + h * c
            box = np.array([[x0, y0], [x1, y0], [x0, y1], [x1, y1]])
            corners.append(sm.apply_2d(t, box))
        corners = np.concatenate(corners)
        lo = np.floor(corners.min(axis=0))
        shape = tuple((np.ceil((corners.max(axis=0) - lo) / cell)).astype(int)[::-1])
        on = [ps.resample(grid, origin, c, t, lo, shape, cell)
              for (grid, origin, c), (t, _) in zip(read, fits)]
        occupied = np.stack([g == sm.PGM_OCCUPIED for g in on])
        # Free kept a cell off each map's own walls, as map-area does (passes.seen_free).
        free = np.stack([(g == sm.PGM_FREE) & ~ps.near(o, ps.TOLERANCE)
                         for g, o in zip(on, occupied)])
        classes = ps.classify(occupied, free)
        grid, origin = sm.trim(ps.to_pgm(classes), tuple(lo), cell)
        sm.write_map2d(out / 'map2d', grid, origin, cell, note=(
            f"map_drive.py --merge of {len(dirs)} maps, onto {dirs[0].name}'s"))
        grid_note = ps.summary(classes, cell)
    info = {'merged': [str(d) for d in dirs],
            'onto_first': {str(d): f for d, (_, f) in zip(dirs[1:], fits[1:])},
            'kept_within_m': args.radius, 'min_maps': args.min_maps,
            'points': {**{d.name: len(m) for d, m in zip(dirs, maps)}, 'merged': len(both)},
            'map2d': grid_note or 'not made: a folder without map2d.yaml',
            'frame': f"{dirs[0].name}'s",
            'made': datetime.datetime.now().isoformat(timespec='seconds')}
    (out / 'ndt.yaml').write_text(yaml.safe_dump(info, sort_keys=False))
    print(yaml.safe_dump(info, sort_keys=False))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('drive', nargs='?', help="the drive's folder (bags/<date>_<name>)")
    parser.add_argument('--out', help='where the maps go (default <drive>/map)')
    parser.add_argument('--split', type=float, default=SPLIT,
                        help=f"m between KISS-SLAM's local maps (default {SPLIT:g})")
    parser.add_argument('--min-height', type=float, default=ndt_map.MIN_HEIGHT,
                        help='m above base_link: lower is ground, left out of SLAM (0.3)')
    parser.add_argument('--cell', type=float, default=CELL, help=f'2D map cell, m ({CELL})')
    parser.add_argument('--voxel', type=float, default=ndt_map.VOXEL,
                        help='NDT map thinning, m (0.1)')
    parser.add_argument('--datum', type=float, nargs=2, metavar=('LAT', 'LON'),
                        help="the map's datum (default navsat.yaml's)")
    parser.add_argument('--gps-sd', type=float, default=sm.GOOD_FIX_SD,
                        help=f"a good fix's largest standard deviation, m ({sm.GOOD_FIX_SD})")
    parser.add_argument('--no-gps', action='store_true', help='place by the start pose only')
    parser.add_argument('--no-chunks', action='store_true', help="don't cut chunks")
    parser.add_argument('--merge', nargs='+', metavar='MAP',
                        help='two map folders or more (each with ndt.pcd, map2d.yaml) into '
                             '--out, fitted onto the first')
    parser.add_argument('--min-maps', type=int, default=2,
                        help='merge: keep an NDT point at least this many maps saw (2)')
    parser.add_argument('--radius', type=float, default=0.25,
                        help='merge: a point is in both within this, m (0.25)')
    parser.add_argument('--icp-reach', type=float, default=1.0,
                        help="merge: ICP's largest match distance, m (1.0)")
    args = parser.parse_args()
    if args.merge:
        if not args.out:
            parser.error('--merge needs --out')
        return merge(args)
    if not args.drive:
        parser.error('which drive? pixi run map-drive bags/<drive>')
    return make(args)


if __name__ == '__main__':
    sys.exit(main())
