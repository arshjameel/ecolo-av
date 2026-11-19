#!/usr/bin/env python3
"""
Write lane-core's agreement vectors from aav_lane_finder's Python modules (no ROS).

The Rust (crates/lane-core, tests/vectors.rs) must give the same answers on the same
pictures:

  - made-up lanes (test/scenes.py): straight, offset, turned, curved both ways, dashed,
    one side missing, none, noise, night-dark, a tilted camera, distorted lenses, other
    settings, and a run of frames that teaches the finder a narrower lane;
  - with --recorded DIR, every *.png in it (and DIR/lens.json: the CameraInfo and the
    camera's mount): pictures recorded in Ravensim (test/capture_frames.py writes them).

Each picture is written to crates/lane-core/tests/pictures/ as a PNG; noise is added after
reading it, by a generator both languages share (`noisy`), so the noisy pictures stay
small. For each frame it records the paint mask's size, the bird's-eye view's grey sum,
the lane (every LaneGeometry field), and the finder's remembered width; and for each lens
where points on the ground land (birds_eye.project).

  pixi run python ros/perception/aav_lane_finder_rs/gen_vectors.py [--recorded DIR]
"""

import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
FINDER = HERE.parent / 'aav_lane_finder'
sys.path[:0] = [str(FINDER), str(FINDER / 'test')]
from aav_lane_finder.birds_eye import BirdsEye, Grid, Lens, Mount, project  # noqa: E402, I100
from aav_lane_finder.lane_lines import LaneFinder, Settings  # noqa: E402
from scenes import BENCH_CAMERA, picture  # noqa: E402

CORE = HERE / 'crates' / 'lane-core' / 'tests'
OUT = CORE / 'vectors.json'
PICTURES = CORE / 'pictures'
BENCH_LENS = Lens.pinhole(640, 360, 343.0)


def noisy(image, seed, amplitude=20):
    """
    Return the picture with noise added: the generator tests/vectors.rs repeats.

    A 31-bit linear congruential generator (glibc's constants), one step per byte, in the
    picture's memory order (rows, columns, then blue, green, red).
    """
    flat = image.reshape(-1).astype(np.int32)
    x = seed
    span = 2 * amplitude + 1
    noise = np.empty(flat.size, np.int32)
    for i in range(flat.size):
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF
        noise[i] = (x >> 16) % span - amplitude
    return np.clip(flat + noise, 0, 255).astype(np.uint8).reshape(image.shape)


def dark(image, factor):
    return (image.astype(float) * factor).astype(np.uint8)


def lens_json(lens):
    return {'width': lens.width, 'height': lens.height, 'fx': lens.fx, 'fy': lens.fy,
            'cx': lens.cx, 'cy': lens.cy, 'distortion': list(lens.distortion)}


def settings_json(s):
    return {k: getattr(s, k) for k in s.__dataclass_fields__}


def grid_json(g):
    return {k: getattr(g, k) for k in g.__dataclass_fields__}


def line_json(line):
    return None if line is None else {'a': line.a, 'b': line.b, 'c': line.c,
                                      'confidence': line.confidence}


def lane_json(lane):
    return {'left': line_json(lane.left), 'right': line_json(lane.right),
            'left_confidence': lane.left_confidence, 'right_confidence': lane.right_confidence,
            'offset': lane.offset, 'width': lane.width, 'left_radius': lane.left_radius,
            'right_radius': lane.right_radius, 'mean_radius': lane.mean_radius,
            'curve_direction': lane.curve_direction, 'steering_hint': lane.steering_hint,
            'left_paint': lane.left_paint, 'right_paint': lane.right_paint}


def save(name, image):
    PICTURES.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(PICTURES / name), image, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    # Read back, as the Rust will: what goes in is what's on disk.
    return cv2.imread(str(PICTURES / name), cv2.IMREAD_COLOR)


def case(name, frames, lens=BENCH_LENS, mount=Mount(BENCH_CAMERA), grid=Grid(),
         settings=Settings()):
    """Run one finder over the frames: [(picture file, image, noise seed)]."""
    view = BirdsEye(lens, mount, grid)
    finder = LaneFinder(view, settings)
    out = []
    for file, image, seed in frames:
        if seed:
            image = noisy(image, seed)
        lane = finder.find(image)
        out.append({'picture': file, 'noise': seed, 'mask': int(finder.last.mask.sum()),
                    'bev_sum': int(finder.last.bev.max(axis=2).astype(np.int64).sum()),
                    'lane': lane_json(lane), 'width_after': finder.width})
    nearest = view.nearest_seen()
    return {'name': name, 'lens': lens_json(lens),
            'mount': {'xyz': list(mount.xyz), 'rpy': list(mount.rpy)},
            'grid': grid_json(grid), 'settings': settings_json(settings),
            'nearest_seen': nearest if math.isfinite(nearest) else None, 'frames': out}


def made_up():
    """Return the made-up cases."""
    pics = {
        'straight.png': picture(),
        'offset_left.png': picture(offset=0.5),
        'turned.png': picture(offset=-0.4, heading=0.05),
        'curve_left.png': picture(radius=60.0),
        'curve_right.png': picture(radius=-60.0),
        'curve_tight.png': picture(radius=30.0, offset=0.2),
        'dashed.png': picture(offset=0.3, dash=(9.0, 3.0)),
        'dashed_curve.png': picture(radius=-80.0, dash=(6.0, 2.0)),
        'right_only.png': picture(offset=0.3, lines=('right',)),
        'left_only.png': picture(offset=-0.2, lines=('left',)),
        'no_lines.png': picture(lines=()),
        'night.png': dark(picture(offset=0.2), 0.32),
        'night_too_dark.png': dark(picture(offset=0.2), 0.2),
        'narrow.png': picture(lane_width=3.0),
        'narrow_right_only.png': picture(offset=0.2, lane_width=3.0, lines=('right',)),
        'wide_offset.png': picture(lane_width=4.2, offset=-0.6, heading=-0.04),
        'yellow_left.png': picture(offset=0.2, yellow=('left',)),
        'yellow_right_dashed.png': picture(offset=-0.3, dash=(6.0, 3.0), yellow=('right',)),
    }
    tilted = (1.5, 0.0, 1.6)
    pics['tilted.png'] = picture(offset=0.3, camera=tilted, pitch=0.12)
    img = {name: save(name, image) for name, image in pics.items()}

    def frames(*names, seed=0):
        return [(n, img[n], seed) for n in names]

    cases = [case(n, frames(n)) for n in (
        'straight.png', 'offset_left.png', 'turned.png', 'curve_left.png', 'curve_right.png',
        'curve_tight.png', 'dashed.png', 'dashed_curve.png', 'right_only.png',
        'left_only.png', 'no_lines.png', 'night.png', 'night_too_dark.png',
        'wide_offset.png', 'yellow_left.png', 'yellow_right_dashed.png')]
    cases.append(case('noise on a straight lane', frames('offset_left.png', seed=7)))
    cases.append(case('noise on a curve', frames('curve_right.png', seed=12345)))
    cases.append(case('noise on a dashed lane', frames('dashed.png', seed=99)))
    cases.append(case('noise on a yellow line', frames('yellow_left.png', seed=31)))
    cases.append(case('a narrower lane, learnt, then one line',
                      frames(*['narrow.png'] * 20, 'narrow_right_only.png')))
    cases.append(case('a camera higher up, tilted down', frames('tilted.png'),
                      mount=Mount(tilted, (0.0, 0.12, 0.0))))
    cases.append(case('a camera turned and rolled a little', frames('straight.png'),
                      mount=Mount(BENCH_CAMERA, (0.01, 0.02, 0.03))))
    plumb = Lens(640, 360, 343.0, 340.0, 322.0, 178.0, (-0.08, 0.02, 0.001, -0.0005, 0.003))
    rational = Lens(640, 360, 343.0, 343.0, 320.0, 180.0,
                    (0.1, -0.02, 0.0, 0.0, 0.001, 0.12, -0.01, 0.002))
    cases.append(case('a plumb_bob lens', frames('straight.png', 'curve_left.png'),
                      lens=plumb))
    cases.append(case('a rational_polynomial lens', frames('turned.png'), lens=rational))
    cases.append(case('other settings', frames('curve_tight.png', 'dashed.png'),
                      grid=Grid(x_min=3.0, x_max=20.0, y_max=5.0, resolution=0.04),
                      settings=Settings(paint_contrast=30.0, paint_width=0.25, windows=9,
                                        window_margin=0.6, min_pixels=5, min_span=1.5,
                                        fit_scale=0.2, min_confidence=0.4, lane_width=3.2,
                                        lookahead=5.0, wheelbase=2.0)))
    return cases, [plumb, rational, BENCH_LENS]


def recorded(folder):
    """Return the case of the pictures recorded in Ravensim, in file order."""
    info = json.loads((folder / 'lens.json').read_text())
    lens = Lens(info['width'], info['height'], info['fx'], info['fy'], info['cx'],
                info['cy'], tuple(info.get('distortion', ())))
    mount = Mount(tuple(info['xyz']), tuple(info['rpy']))
    frames = []
    for path in sorted(folder.glob('*.png')):
        name = 'ravensim_' + path.name
        frames.append((name, save(name, cv2.imread(str(path), cv2.IMREAD_COLOR)), 0))
    return [case(f'Ravensim, {info.get("where", "recorded")}', frames, lens=lens,
                 mount=mount)], lens, mount


def rerun(old):
    """Return a recorded case worked out again from its saved pictures."""
    lens = old['lens']
    lens = Lens(lens['width'], lens['height'], lens['fx'], lens['fy'], lens['cx'], lens['cy'],
                tuple(lens['distortion']))
    mount = Mount(tuple(old['mount']['xyz']), tuple(old['mount']['rpy']))
    frames = [(f['picture'], cv2.imread(str(PICTURES / f['picture']), cv2.IMREAD_COLOR),
               f['noise']) for f in old['frames']]
    return case(old['name'], frames, lens=lens, mount=mount, grid=Grid(**old['grid']),
                settings=Settings(**{k: v for k, v in old['settings'].items()
                                     if k in Settings.__dataclass_fields__}))


def projections(lenses, mounts):
    """Return random ground and air points through each lens: where they land."""
    rng = np.random.default_rng(2026)
    points = np.column_stack([rng.uniform(-5.0, 40.0, 200), rng.uniform(-15.0, 15.0, 200),
                              rng.uniform(-0.5, 1.0, 200)])
    out = []
    for lens in lenses:
        for mount in mounts:
            uv, seen = project(points, lens, mount)
            out.append({'lens': lens_json(lens),
                        'mount': {'xyz': list(mount.xyz), 'rpy': list(mount.rpy)},
                        'points': points.tolist(),
                        'uv': [list(map(float, p)) if s else None for p, s in zip(uv, seen)]})
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--recorded', type=Path, help='a folder of Ravensim pictures')
    args = parser.parse_args()
    for old in PICTURES.glob('*.png'):
        if not args.recorded and old.name.startswith('ravensim_'):
            continue   # keep the recorded ones when only the made-up ones are rewritten
        old.unlink()
    cases, lenses = made_up()
    mounts = [Mount(BENCH_CAMERA), Mount((1.5, 0.0, 1.6), (0.0, 0.12, 0.0)),
              Mount(BENCH_CAMERA, (0.01, 0.02, 0.03))]
    if args.recorded:
        more, lens, mount = recorded(args.recorded)
        cases += more
        lenses.append(lens)
        mounts.append(mount)
    elif OUT.exists():
        # The recorded case from the last run, worked out again: its pictures are still
        # there, with its lens and mount.
        for c in json.loads(OUT.read_text())['cases']:
            if c['name'].startswith('Ravensim'):
                cases.append(rerun(c))
    OUT.write_text(json.dumps({'cases': cases, 'project': projections(lenses, mounts)},
                              indent=None, separators=(',', ':')) + '\n')
    frames = sum(len(c['frames']) for c in cases)
    print(f'{OUT}: {len(cases)} cases, {frames} frames')


if __name__ == '__main__':
    main()
