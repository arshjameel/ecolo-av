#!/usr/bin/env python3
"""
Pillow's crop and scale on a made-up picture, for ufld-core's `tests/pillow.rs`.

    pixi run -e ml python3 ros/perception/aav_lane_finder_learned/gen_pillow.py

The model was trained on pictures cut and scaled by Pillow (tools/lane_data.py `crop`); the
Rust node must give it the same numbers. The picture is a formula both languages share, so
nothing big goes in git: crates/ufld-core/tests/pillow.json holds every channel's sum and
some pixels.
"""

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / 'tools'))
import lane_data as L  # noqa: E402, I100


def picture(width=640, height=360):
    """r, g, b from x and y: edges, ramps and noise-like bits, so every weight matters."""
    y, x = np.mgrid[0:height, 0:width].astype(np.int64)
    r = (x * 7 + y * 3) % 256
    g = (x * x + y * 5) % 256
    b = (x ^ y) % 256
    return np.stack([r, g, b], axis=2).astype(np.uint8)


def main():
    from PIL import Image

    out = {}
    for w, h in [(640, 360), (1280, 720)]:
        p = picture(w, h)
        im = Image.fromarray(p)
        top = round(L.CROP_TOP * h / L.HEIGHT)
        small = np.asarray(im.crop((0, top, w, h)).resize((L.INPUT_W, L.INPUT_H),
                                                          Image.BILINEAR))
        rng = np.random.default_rng(1)
        ys, xs = rng.integers(0, L.INPUT_H, 64), rng.integers(0, L.INPUT_W, 64)
        out[f'{w}x{h}'] = {
            'sums': [int(small[:, :, c].sum()) for c in range(3)],
            'pixels': [[int(x), int(y), *map(int, small[y, x])] for x, y in zip(xs, ys)]}

    (HERE / 'crates' / 'ufld-core' / 'tests' / 'pillow.json').write_text(json.dumps(out))
    print('wrote pillow.json')


if __name__ == '__main__':
    main()
