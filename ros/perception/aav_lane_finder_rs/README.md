# aav_lane_finder_rs

## Overview

`lane_finder` in Rust (rclrs): a **drop-in** for `aav_lane_finder`'s Python node. The
same node name, topics, QoS, parameters and answers. Ben, 18 Nov: "the aav_lane_finder
might be nice to have since it helps the car stay on the correct side"; Rust where
possible, for trust and speed on the car's ARM computers.

```
/sensors/camera/front_left/image_raw ─► bird's-eye warp ─► paint mask ─► sliding windows
/sensors/camera/front_left/camera_info ─┘ (one look-up table          │
                                           per lens)                   ▼
                     /perception/lane/geometry ◄── the lane ◄── a fit per line
                     /perception/lane/image (debug_image) ◄─┘
```

**The default** wherever the lane finder runs (`lane:=true`, `lane_pose:=true`, the car's
`lane`, bench `lane:=true`); `lane_finder_impl:=python` runs the Python one instead.

No OpenCV: the warp, the mask and the windows are a few loops, and the fit is `nalgebra`'s
SVD. So it builds anywhere `cargo` does, the car kit included, with nothing in C++.

What it does, step by step, and every parameter: `aav_lane_finder`'s README.

**A learned finder, the challenger** (TODO 262b): [aav_lane_finder_learned](../aav_lane_finder_learned/README.md),
`lane_finder_impl:=learned`, our UFLD-v2-style model through ONNX, behind the same topic.
It uses this package's `lane-core` for the lens, the mount and the fits (`Picture::rgb`
and `Distortion::apply` are public for it).

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/lane-core` | the warp, the paint mask, the windows, the fits, the lane, the debug picture | no: `cargo test` on any laptop |
| `crates/lane-node` | the node, binary `lane_finder` | yes (rclrs 0.8, `ros-env` 0.3) |

### `lane-core`

| Module | Holds | From |
|---|---|---|
| `birds_eye` | `Lens`, `Distortion`, `Mount`, `Grid`, `BirdsEye` (`warp`, `nearest_seen`), `project`, `rotation` | `birds_eye.py`; OpenCV's `projectPoints` and fixed-point `remap` |
| `lines` | `Settings`, `Line`, `Paint`, `paint_of`, `Lane`, `Window`, `Mask`, `View`, `LaneFinder`, `paint_mask`, `starts`, `follow`, `fit_line`, `lane_from`, `centre_y` | `lane_lines.py`; numpy's `polyfit`, `convolve`, `linspace` |
| `draw` | `draw`, `Drawing`: the debug picture | `lane_lines.py`'s `draw` |
| `picture` | `Picture` (borrowed, any stride), `Encoding` (an enum), `Bev` | new |
| `font` | a 5 x 7 font: the characters the debug picture writes | new |

### Node `lane_finder`

The same as `aav_lane_finder`'s (its README): topics, QoS (pictures and camera_info
reliable, the newest only), parameters with the same names and defaults. It reads
`aav_lane_finder`'s `config/lane_finder.yaml`; `aav_lane_finder`'s launch file starts it.

Pictures in `rgb8`, `bgr8`, `rgba8`, `bgra8` or `mono8`. The debug picture is `bgr8`.

## How it matches the Python

`gen_vectors.py` runs the Python modules on the pictures in
`crates/lane-core/tests/pictures/` and writes `vectors.json`; `tests/vectors.rs` runs the
Rust on the same pictures. The noisy ones get their noise from one small generator that
both languages share, so the PNGs stay small.

| Pictures | What |
|---|---|
| made up (`aav_lane_finder`'s `test/scenes.py`) | straight, offset, turned, curved left, right and tight, dashed, dashed on a curve, one side missing (each side), a yellow line (each side), no lines, night (found) and too dark (not found), a wide lane, noise on three of them; a run of 20 that teaches a 3 m lane, then one line; a camera higher and tilted, one turned and rolled; a `plumb_bob` lens and a `rational_polynomial` one; other settings and grid |
| Ravensim | front left camera frames, University Drive (`test/capture_frames.py`) |

What must agree, and how closely: what matters for driving.

| What | Allowed | Largest difference seen |
|---|---|---|
| each line found or not, its paint, the curve's direction | the same | the same |
| offset, lane width, the width it remembers | 2 cm | 7e-15 m |
| each line's heading at the look-ahead, steering hint | 0.5° | 2e-15 rad |
| each line's curvature | 0.001 1/m | 3e-16 1/m |
| each line's confidence | 0.02 | 2e-16 |
| the paint mask's pixels, the view's grey sum | 0.5 % | 0 (the same pixels) |
| where a ground point lands in the picture | 1e-6 px | 1e-13 px |

The warp gives OpenCV's grey levels exactly: it uses OpenCV's own fixed point (places to
1/32 of a pixel, rounded half to even from a float32; weights in 2^15ths). So the masks are
the same pixels, and the rest agrees to rounding.

## Measured

Ravensim, the campus, University Drive (`measure lane`: lane keeping, 150 s, 219 m), cameras
on, on a MacBook (M-series). The Rust node ran as the default; the Python one beside it as
`lane_finder_py` on the same pictures (`/perception/lane/geometry_py`). CPU from `ps`
over 60 s, both at once, about 10 pictures a second.

| | Python | Rust |
|---|---|---|
| CPU | 17.4 % of a core | **3.7 %** |
| memory | 175 MB | **37 MB** |
| lanes on the same 1,535 pictures | | the same: lines found in all, both in 965; offset within 6e-14 m |

The drive itself (Rust steering): 1.9 cm from the lane's centre on average, 3.0 cm RMS,
the lane never lost.

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| The debug picture | OpenCV's anti-aliased lines, Hershey text | plain lines, a 5 x 7 font; the same layout, sizes and colours |
| A distortion with the wrong number of coefficients | OpenCV raises on every picture: none published | a warning; the lens without distortion |
| A tilted-sensor distortion (14 coefficients) | OpenCV's | refused as above (no camera of ours has one) |
| A parameter both in a file under the node's name and under `/**` (`-p`, or a launch dict with no namespace) | rclpy: the later one wins | rclrs: the node's name wins. `lane_finder.launch.py` gives the node `namespace='/'`, so its dict is written under `/lane_finder` and wins in both (`debug_image:=true` checked with each) |

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from them. Change one, change both, write the vectors again.
- The same node name and topics as the Python: only one of the two runs
  (`lane_finder_impl:=`).
- `lane-core` has no ROS and no OpenCV; `cargo fmt` and `cargo clippy -D warnings` clean.
- REP-103: x forward, y left, metres; the offset + when the car is left of the centre.

## Verification

```sh
cd ros/perception/aav_lane_finder_rs && pixi run cargo test -p lane-core
pixi run python ros/perception/aav_lane_finder_rs/gen_vectors.py   # after changing the Python
pixi run build && colcon test --packages-select aav_lane_finder_rs
```
