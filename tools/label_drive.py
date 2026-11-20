#!/usr/bin/env python3
"""
Label what's around after a drive (TODO 252, phase F): camera boxes onto lidar points.

    pixi run label-drive bags/<drive>                 labels into bags/<drive>/map/labels.yaml
    pixi run label-drive bags/<drive> --map maps/lot  into another map folder
    pixi run label-drive bags/<drive> --draw-sim WORLD.sdf   a Ravensim drive recorded
                                                      without pictures: draw them at its truth
    pixi run label-drive --sim-drive bags/lot-p4 --world campus --lot P4
                                                      make a drive from Ravensim alone: poses
                                                      round a campus lot, its pictures and
                                                      lidar at each, a map folder; then label

Runs in the ml environment (PyTorch on mps). The page: docs/labelling.md.

1. Frames: every --every seconds of the drive, its lidar cloud (base_link), the car's pose
   in the map (the map folder's poses.tum, else Ravensim's truth, else the global EKF) and
   each camera's picture nearest in time (JPEG or raw).
2. The detector: OWLv2 (Google, Apache-2.0 code and weights), open vocabulary: asked for
   label_core.WORDS (tree, building, car, pole, sign, person...). On the GPU (mps).
3. Each box onto the lidar points the camera saw (vehicle.yaml's mounts, camera_info's
   lens): the nearest group by depth inside the box takes the label (label_core.in_box).
4. Fused across frames in 0.5 m map cells, grown into clusters: one label each, a rotated
   box, a height band, a confidence, seen_in this drive (label_core.Fusion).
5. Merged into the map folder's labels.yaml (label_core.merge): a person's labels stay;
   pixi run map-edit shows them, a person fixes them. Then label-train learns from them.
6. Scored, where Ravensim's truth exists (the campus's objects, a world's boxes): each
   class's precision and recall, in <drive>/labels/report.md.

Writes <drive>/labels/: detections.json (the detector's boxes: run again with
--reuse to fuse without the detector), frames.npz (each frame's points and pose, for
label-train), report.md; and <map>/labels.yaml. The maths: tools/label_core.py
(tools/tests/test_label_core.py).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import label_core as lc  # noqa: E402, I100

VEHICLE = ROOT / 'ros' / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml'
CAMERAS_YAML = ROOT / 'ros' / 'sim' / 'aav_sim_cameras' / 'config' / 'cameras.yaml'
WORLDS = ROOT / 'ros' / 'sim' / 'aav_sim_models' / 'worlds'
MODELS_DIR = ROOT / 'ros' / 'sim' / 'aav_sim_models' / 'models'
CAMPUS = ROOT / 'maps' / 'campus'
SIM_CAMERAS = ROOT / 'ros' / 'sim' / 'aav_sim_cameras' / 'target' / 'release' / 'sim_cameras'
SIMULATOR = ROOT / 'ros' / 'sim' / 'aav_simulator'
SCAN_POSES = SIMULATOR / 'target' / 'release' / 'examples' / 'scan_poses'
DETECTOR = 'google/owlv2-base-patch16-ensemble'   # Apache-2.0 (docs/labelling.md)
CLOUDS = '/sensors/lidar/points'
TRUTH = '/vehicle/sim/truth'
EKF = '/localization/odom/global'
CAMERA_NAMES = ('front_left', 'front_right', 'rear')
# A world's box models by name, as truth (test_lot.sdf: building, parked_cars, walls).
SDF_NAMES = {'building': 'building', 'parked_car': 'parked_car', 'wall': 'other',
             'tree': 'tree', 'post': 'pole', 'stop_sign': 'pole'}


# --- The car's sensors --------------------------------------------------------------------

def mounts():
    """Return vehicle.yaml's sensor mounts: {frame: (xyz, rpy)}."""
    sensors = yaml.safe_load(VEHICLE.read_text())['sensors']
    return {k: (tuple(v['xyz']), tuple(v.get('rpy', (0, 0, 0)))) for k, v in sensors.items()
            if isinstance(v, dict) and 'xyz' in v}


def cameras(names, infos=None):
    """Return label_core.Cameras: the lens from camera_info if given, else cameras.yaml's."""
    where = mounts()
    out = {}
    for cam in yaml.safe_load(CAMERAS_YAML.read_text())['cameras']:
        if cam['name'] not in names:
            continue
        xyz, rpy = where[cam['mount']]
        info = (infos or {}).get(cam['name'])
        if info is not None and info.k[0] > 0:
            out[cam['name']] = lc.Camera(cam['name'], info.width, info.height, info.k[0],
                                         info.k[4], info.k[2], info.k[5], xyz, rpy)
        else:
            out[cam['name']] = lc.Camera.pinhole(cam['name'], cam['width'], cam['height'],
                                                 cam['focal_px'], xyz, rpy)
    return out


# --- A drive made from Ravensim alone -------------------------------------------------------

def world_path(world):
    """Return a world's SDF: campus, a name in aav_sim_models/worlds, or a path."""
    if world == 'campus':
        return CAMPUS / 'campus.sdf'
    path = Path(world)
    return path if path.suffix == '.sdf' else WORLDS / f'{world}.sdf'


def lot_route(name, step=2.0, inset=6.0):
    """Return poses (x, y, yaw) round a campus lot, `inset` m in from its edge."""
    import campus_world as cw
    datum = yaml.safe_load(cw.NAVSAT.read_text())['navsat_transform']['ros__parameters']['datum']
    from aav_carla.frames import geodetic_to_enu

    def project(lat, lon):
        return geodetic_to_enu(datum[0], datum[1], 0.0, lat, lon, 0.0)[:2]

    campus = cw.read_osm(cw.OSM, project)
    lots = [a for a in campus.areas if name in (a.tags.get('name'), a.tags.get('ref'))]
    if not lots:
        raise SystemExit(f'no lot called {name} in {cw.OSM}')
    ring = np.array(cw.open_ring(cw.simplify(lots[0].ring)), float)
    return route_inside(ring, step, inset)


def route_inside(ring, step=2.0, inset=6.0):
    """Return poses (x, y, yaw) every `step` m round a ring pulled `inset` m to its middle."""
    ring = np.asarray(ring, float)
    middle = ring.mean(0)
    pulled = []
    for p in ring:
        d = middle - p
        n = np.linalg.norm(d)
        pulled.append(p + d / n * min(inset, 0.45 * n) if n > 0 else p)
    pulled = np.array(pulled + [pulled[0]])
    poses = []
    for a, b in zip(pulled, pulled[1:]):
        length = float(np.linalg.norm(b - a))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for t in np.arange(0.0, length, step):
            p = a + (b - a) * t / max(length, 1e-9)
            poses.append((float(p[0]), float(p[1]), yaw))
    return poses


def make_sim_drive(out, world, poses, names):
    """
    Write a drive from Ravensim alone.

    poses.csv (t, x, y, yaw), pictures/ (sim_cameras --poses), scans.bin (scan_poses: the
    roof lidar, as Ravensim casts it), drive.yaml; and its map folder, map/: NDT's
    points, a 2D map, poses.tum (map_folder).
    """
    out = Path(out)
    (out / 'pictures').mkdir(parents=True, exist_ok=True)
    stamps = np.arange(len(poses)) * 1.0
    plain = out / 'poses_xy.csv'
    plain.write_text(''.join(f'{x:.3f},{y:.3f},{yaw:.4f}\n' for x, y, yaw in poses))
    (out / 'poses.csv').write_text('# t,x,y,yaw (s, m, m, rad; map frame)\n' + ''.join(
        f'{t:.1f},{x:.3f},{y:.3f},{yaw:.4f}\n' for t, (x, y, yaw) in zip(stamps, poses)))
    sdf = world_path(world)
    if not SIM_CAMERAS.exists():
        raise SystemExit(f'{SIM_CAMERAS}: missing (cd ros/sim/aav_sim_cameras && cargo build '
                         '--release)')
    subprocess.run([str(SIM_CAMERAS), '--world', str(sdf), '--models', str(MODELS_DIR),
                    '--cameras', ','.join(names), '--poses', str(plain),
                    '--save', str(out / 'pictures')],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not SCAN_POSES.exists():
        subprocess.run(['cargo', 'build', '--release', '--locked', '-p', 'sim-core',
                        '--example', 'scan_poses'], cwd=SIMULATOR, check=True)
    xyz, rpy = mounts()['lidar_link']
    subprocess.run([str(SCAN_POSES), str(sdf), str(plain), str(out / 'scans.bin'),
                    '--models', str(MODELS_DIR), '--mount', ','.join(map(str, xyz + rpy))],
                   check=True)
    (out / 'drive.yaml').write_text(yaml.safe_dump({
        'kind': 'posed: drawn and cast by Ravensim at each pose, no ROS (label_drive.py)',
        'world': str(world), 'cameras': list(names), 'poses': len(poses),
        'note': 'not driven: the car is put at each pose; the lidar is still, so no skew'},
        sort_keys=False))
    frames = list(read_posed(out, names, pictures=False))
    map_folder(out / 'map', frames, note=f'Ravensim {world}, posed')
    return out


def read_scans(path):
    """Return scan_poses's scans: a list of N x 4 arrays (x, y, z in lidar_link, ring)."""
    data = Path(path).read_bytes()
    scans, at = [], 0
    while at < len(data):
        n = int(np.frombuffer(data, '<u4', 1, at)[0])
        at += 4
        scans.append(np.frombuffer(data, '<f4', 4 * n, at).reshape(n, 4).astype(float))
        at += 16 * n
    return scans


def read_posed(drive, names, pictures=True):
    """Yield a posed drive's label_core.Frames."""
    drive = Path(drive)
    rows = np.loadtxt(drive / 'poses.csv', delimiter=',', comments='#', ndmin=2)
    xyz, rpy = mounts()['lidar_link']
    for k, (scan, (t, x, y, yaw)) in enumerate(zip(read_scans(drive / 'scans.bin'), rows)):
        pics = {n: drive / 'pictures' / f'{n}_{k:05d}.png' for n in names} if pictures else {}
        yield lc.Frame(float(t), (x, y, yaw), lc.mounted(scan[:, :3], xyz, rpy),
                       {n: p for n, p in pics.items() if p.exists()})


def map_folder(out, frames, note=''):
    """Write a map folder map-edit opens: ndt.pcd, ndt.yaml, map2d.pgm/.yaml, poses.tum."""
    import ndt_map
    import slam_map as sm
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    points = [lc.to_map(f.points[(f.points[:, 2] > lc.GROUND)
                                 & (np.hypot(f.points[:, 0], f.points[:, 1]) < lc.MAX_RANGE)],
                        f.pose) for f in frames]
    cloud = ndt_map.thin(np.concatenate(points), 0.1)
    ndt_map.write_pcd(out / 'ndt.pcd', cloud)
    lo, hi = cloud[:, :2].min(0) - 5, cloud[:, :2].max(0) + 5
    occupancy = sm.Occupancy(((lo[0], lo[1]), (hi[0], hi[1])))
    lidar = mounts()['lidar_link'][0]
    for f in frames:
        occupancy.add(f.points, f.pose, sensor=lidar[:2])
    occupancy.write(out / 'map2d', note=note)
    sm.write_tum(out / 'poses.tum', [f.stamp for f in frames], [f.pose for f in frames])
    (out / 'ndt.yaml').write_text(yaml.safe_dump({
        'kind': f'from label_drive.py: {note}', 'frame': 'map', 'points': len(cloud),
        'voxel_m': 0.1, 'placed': 'truth (Ravensim)'}, sort_keys=False))


# --- A recorded drive -----------------------------------------------------------------------

def bag_frames(drive, names, every, map_dir, draw_world=None):
    """Return a recorded drive's Frames (rosbag2_py: run with ROS's Python, the ml env)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import ndt_map
    import slam_map as sm

    bag = Path(drive) / 'bag' if (Path(drive) / 'bag').is_dir() else Path(drive)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {CLOUDS, TRUTH, EKF}
    for n in names:
        for suffix in ('/image_raw/compressed', '/image_raw', '/camera_info'):
            wanted.add(f'/sensors/camera/{n}{suffix}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in types]))
    clouds, poses, ekf, pictures, infos = [], [], [], {n: [] for n in names}, {}
    last = -1e9
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == CLOUDS and t - last >= every:
            last = t
            clouds.append((t, ndt_map.cloud_xyz(
                [(f.name, f.offset, f.datatype, f.count) for f in msg.fields], msg.point_step,
                msg.data, msg.width, msg.height, msg.row_step, msg.is_bigendian)))
        elif topic in (TRUTH, EKF):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            (poses if topic == TRUTH else ekf).append(
                (t, (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)))
        elif topic.endswith('/camera_info'):
            infos[topic.split('/')[3]] = msg
        elif '/image_raw' in topic:
            pictures[topic.split('/')[3]].append((t, msg))
    tum = Path(map_dir) / 'poses.tum'
    if tum.exists():
        rows = np.loadtxt(tum, ndmin=2)
        track = [(r[0], (r[1], r[2], 2 * math.atan2(r[6], r[7]))) for r in rows]
    else:
        track = poses or ekf
    if not track:
        raise SystemExit(f'{drive}: no poses (poses.tum, {TRUTH} or {EKF})')
    stamps = [t for t, _ in track]
    xyz, rpy = mounts()['lidar_link']
    frames = []
    for t, cloud in clouds:
        pose = sm.at(stamps, [p for _, p in track], t)
        if pose is None:
            continue
        pics = {}
        for n, seq in pictures.items():
            if seq:
                k = int(np.argmin([abs(s - t) for s, _ in seq]))
                if abs(seq[k][0] - t) < 0.06:
                    pics[n] = seq[k][1]
        frames.append(lc.Frame(t, tuple(pose), lc.mounted(cloud, xyz, rpy), pics))
    if draw_world and not any(f.pictures for f in frames):
        frames = draw_pictures(frames, names, draw_world, Path(drive) / 'labels' / 'pictures')
    return frames, infos


def draw_pictures(frames, names, world, folder):
    """Draw each frame's pictures with sim_cameras at its pose (a Ravensim drive's truth)."""
    folder.mkdir(parents=True, exist_ok=True)
    plain = folder / 'poses.csv'
    plain.write_text(''.join(f'{x:.3f},{y:.3f},{yaw:.4f}\n' for x, y, yaw in
                             (f.pose for f in frames)))
    subprocess.run([str(SIM_CAMERAS), '--world', str(world_path(world)), '--models',
                    str(MODELS_DIR), '--cameras', ','.join(names), '--poses', str(plain),
                    '--save', str(folder)], check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    for k, f in enumerate(frames):
        f.pictures = {n: folder / f'{n}_{k:05d}.png' for n in names}
    return frames


def rgb(picture):
    """Return a picture (a path, CompressedImage or Image) as a PIL RGB image."""
    from PIL import Image
    import io
    if isinstance(picture, Path):
        return Image.open(picture).convert('RGB')
    if hasattr(picture, 'format'):
        return Image.open(io.BytesIO(bytes(picture.data))).convert('RGB')
    array = np.frombuffer(bytes(picture.data), np.uint8).reshape(
        picture.height, picture.step)[:, :picture.width * 3].reshape(
        picture.height, picture.width, 3)
    if picture.encoding.startswith('bgr'):
        array = array[:, :, ::-1]
    return Image.fromarray(np.ascontiguousarray(array))


# --- The detector -------------------------------------------------------------------------

class Detector:
    """OWLv2 on PyTorch: the words of label_core.WORDS, boxes in picture pixels."""

    def __init__(self, device, threshold):
        """Load the model (downloaded once into Hugging Face's cache)."""
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch = torch
        self.device = device
        self.threshold = threshold
        self.processor = Owlv2Processor.from_pretrained(DETECTOR)
        self.model = Owlv2ForObjectDetection.from_pretrained(DETECTOR).to(device).eval()
        self.words = list(lc.WORDS)
        self.queries = [[f'a photo of a {w}' for w in self.words]]

    def __call__(self, image):
        """Return [(word, score, (u0, v0, u1, v1))] for one PIL picture."""
        inputs = self.processor(text=self.queries, images=image, return_tensors='pt').to(
            self.device)
        with self.torch.no_grad():
            out = self.model(**inputs)
        side = max(image.size)          # OWLv2 pads the picture square, bottom and right
        found = self.processor.post_process_grounded_object_detection(
            out, threshold=self.threshold,
            target_sizes=self.torch.tensor([[side, side]]))[0]
        return [(self.words[int(k)], float(s), tuple(float(v) for v in b))
                for s, k, b in zip(found['scores'].cpu(), found['labels'].cpu(),
                                   found['boxes'].cpu())]


def detect(frames, names, device, threshold):
    """Return {frame index: {camera: [(word, score, box)]}}, and seconds a picture."""
    detector = Detector(device, threshold)
    out, n, t0 = {}, 0, time.time()
    for k, f in enumerate(frames):
        out[k] = {}
        for name in names:
            if name in f.pictures:
                out[k][name] = detector(rgb(f.pictures[name]))
                n += 1
        if k % 25 == 0:
            print(f'  detector: frame {k + 1}/{len(frames)}', flush=True)
    return out, (time.time() - t0) / max(n, 1)


# --- Onto the points -----------------------------------------------------------------------

def fuse(frames, found, cams, drive):
    """Return (Fusion, the map points the cameras saw, counts): boxes onto lidar points."""
    fusion = lc.Fusion()
    pictured, counts = [], {'boxes': 0, 'on_points': 0, 'by_word': {}}
    for k, f in enumerate(frames):
        flat = np.hypot(f.points[:, 0], f.points[:, 1])
        keep = (f.points[:, 2] > lc.FLOOR) & (flat < lc.MAX_RANGE)
        points = f.points[keep]
        in_map = lc.to_map(points, f.pose)
        for name, boxes in found.get(k, {}).items():
            cam = cams[name]
            u, v, depth = cam.project(points)
            seen = cam.sees(u, v)
            pictured.append(in_map[seen])
            for word, score, box in boxes:
                counts['boxes'] += 1
                counts['by_word'][word] = counts['by_word'].get(word, 0) + 1
                cls = lc.WORDS.get(word)
                index = lc.in_box(u, v, depth, box)
                if cls is None or len(index) == 0:
                    continue
                counts['on_points'] += 1
                fusion.add(in_map[index], cls, score, k)
    pictured = np.concatenate(pictured) if pictured else np.zeros((0, 3))
    return fusion, pictured, counts


# --- Truth --------------------------------------------------------------------------------

def truth(world):
    """Return Ravensim's objects in a world as label_core.Things, or [] if none known."""
    if world is None:
        return []
    if str(world) == 'campus':
        objects = CAMPUS / 'objects.yaml'
        if objects.exists():
            items = yaml.safe_load(objects.read_text())['objects']
        else:                         # a campus made before objects.yaml: the same build again
            import campus_world as cw
            from aav_carla.frames import geodetic_to_enu
            datum = yaml.safe_load(cw.NAVSAT.read_text())[
                'navsat_transform']['ros__parameters']['datum']
            campus = cw.read_osm(cw.OSM, lambda la, lo: geodetic_to_enu(
                datum[0], datum[1], 0.0, la, lo, 0.0)[:2])
            _, _, _, walls, trunks, _, _, _ = cw.build(campus, cw.box_round())
            items = cw.objects(walls, trunks)
        return [lc.Thing(o['class'], np.array(o['ring'], float), o['height']) for o in items]
    return lc.sdf_things(world_path(world).read_text(), SDF_NAMES)


# --- The command ---------------------------------------------------------------------------

def main():
    """Label one drive; or make a drive from Ravensim, then label it."""
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('drive', type=Path, nargs='?', help='bags/<drive>')
    p.add_argument('--map', type=Path, help='the map folder (default: <drive>/map)')
    p.add_argument('--every', type=float, default=1.0, help='s between frames (default 1)')
    p.add_argument('--cameras', default=','.join(CAMERA_NAMES))
    p.add_argument('--threshold', type=float, default=0.2, help="the detector's (0.2)")
    p.add_argument('--device', default='mps', help='mps, cuda or cpu')
    p.add_argument('--draw-sim', metavar='WORLD',
                   help='a Ravensim drive without pictures: draw them at its poses')
    p.add_argument('--truth', metavar='WORLD', help='score against this world (campus, '
                   'a world name or .sdf); a Ravensim drive scores against its own')
    p.add_argument('--reuse', action='store_true', help='the detections from the last run')
    p.add_argument('--sim-drive', type=Path, metavar='OUT', help='make a drive from Ravensim')
    p.add_argument('--world', default='campus', help='--sim-drive: the world (campus)')
    p.add_argument('--lot', default='P4', help='--sim-drive: round this campus lot (P4)')
    p.add_argument('--step', type=float, default=2.0, help='--sim-drive: m between poses')
    p.add_argument('--sim-cameras', type=Path, default=SIM_CAMERAS,
                   help='the sim_cameras program (cargo build --release in aav_sim_cameras)')
    args = p.parse_args()
    globals()['SIM_CAMERAS'] = args.sim_cameras
    names = [n for n in args.cameras.split(',') if n]

    if args.sim_drive:
        t0 = time.time()
        poses = lot_route(args.lot, args.step)
        make_sim_drive(args.sim_drive, args.world, poses, names)
        print(f'{args.sim_drive}: {len(poses)} poses round {args.lot}, '
              f'{time.time() - t0:.0f} s')
        args.drive = args.sim_drive
    if args.drive is None:
        p.error('a drive, or --sim-drive OUT')
    drive = args.drive
    map_dir = args.map or drive / 'map'
    work = drive / 'labels'
    work.mkdir(parents=True, exist_ok=True)
    info = yaml.safe_load((drive / 'drive.yaml').read_text()) if (
        drive / 'drive.yaml').exists() else {}
    posed = (drive / 'scans.bin').exists()
    t0 = time.time()
    if posed:
        frames, infos = list(read_posed(drive, names)), {}
    else:
        frames, infos = bag_frames(drive, names, args.every, map_dir, args.draw_sim)
    if not frames:
        raise SystemExit(f'{drive}: no frames')
    if not map_dir.exists():
        map_folder(map_dir, frames, note=f'{drive.name}: no map-drive map; truth poses')
    cams = cameras(names, infos)
    print(f'{drive}: {len(frames)} frames, {sum(len(f.pictures) for f in frames)} pictures')

    cache = work / 'detections.json'
    if args.reuse and cache.exists():
        found = {int(k): v for k, v in json.loads(cache.read_text())['found'].items()}
        per_picture = json.loads(cache.read_text())['s_per_picture']
    else:
        found, per_picture = detect(frames, names, args.device, args.threshold)
        cache.write_text(json.dumps({'model': DETECTOR, 'threshold': args.threshold,
                                     's_per_picture': per_picture, 'found': found}))
    fusion, pictured, counts = fuse(frames, found, cams, drive.name)
    new = fusion.labels(drive.name)
    labels_path = map_dir / 'labels.yaml'
    merged, merge_counts = lc.merge(lc.read_labels(labels_path), new, drive.name)
    lc.write_labels(labels_path, merged)
    np.savez_compressed(work / 'frames.npz', **{
        'stamps': np.array([f.stamp for f in frames]),
        'poses': np.array([f.pose for f in frames]),
        'sizes': np.array([len(f.points) for f in frames]),
        'points': np.concatenate([f.points for f in frames]).astype(np.float32)})

    world = args.truth or args.draw_sim or (info.get('world') if posed else None)
    things = truth(world)
    lines = [f'# Labels: {drive.name}', '',
             f'{len(frames)} frames, {sum(len(f.pictures) for f in frames)} pictures; '
             f'{DETECTOR} at {args.threshold}, {per_picture * 1000:.0f} ms a picture on '
             f'{args.device}; {time.time() - t0:.0f} s in all.', '',
             f"Boxes: {counts['boxes']} ({counts['on_points']} on lidar points); by word: "
             + ', '.join(f'{w} {n}' for w, n in sorted(counts['by_word'].items())), '',
             f'Labels from this drive: {len(new)}; into {labels_path}: {merge_counts}.']
    if things:
        seen = lc.visible(things, pictured)
        shown = [t for t, s in zip(things, seen) if s]
        scores = lc.score(new, shown)
        lines += ['', f"## Against Ravensim's truth ({world}: {len(shown)} of "
                  f'{len(things)} objects in the pictures and on the lidar)', '',
                  lc.score_table(scores)]
        (work / 'scores.json').write_text(json.dumps(scores, indent=1))
    (work / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
