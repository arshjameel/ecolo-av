# Provenance

Ported for `ecolo-av` on 2026-11-18: a Rust rewrite of our own Python node and modules,
`aav_lane_finder` (`ros/perception/aav_lane_finder` at commit `b55880a1`). The Python
stays, selectable with `lane_finder_impl:=python`.

| Part | From |
|---|---|
| `crates/lane-core/src/birds_eye.rs` | `aav_lane_finder/birds_eye.py`: `Lens`, `Mount`, `Grid`, `rotation`, `project`, `BirdsEye`. The distortion is OpenCV's `projectPoints` model (k1-k6, p1, p2, s1-s4; Apache-2.0, the formula, not its code); the warp is OpenCV's `convertMaps` to `CV_16SC2` and `remap` `INTER_LINEAR` with a black border, in its own fixed point (1/32 px, weights in 2^15ths), checked against `cv2.remap` on 20,000 random places: the same grey levels |
| `crates/lane-core/src/lines.rs` | `aav_lane_finder/lane_lines.py`: `Settings`, `Line`, `Lane`, `Window`, `View`, `paint_mask`, `starts`, `follow`, `fit_line`, `radius`, `centre_y`, `lane_from`, `LaneFinder`. numpy's `polyfit` (columns scaled, then an SVD least-squares solve, here `nalgebra`'s), `convolve(..., 'same')` and `linspace` (BSD-3-Clause, the algorithms, not their code). `Mask` is new |
| `crates/lane-core/src/draw.rs` | `lane_lines.py`'s `draw`: the same layout and colours; plain lines and our own 5 x 7 font (`font.rs`, new) in place of OpenCV's anti-aliasing and Hershey text |
| `crates/lane-core/src/picture.rs` | new: `cv_bridge`'s encodings, read in place |
| `crates/lane-core/tests/vectors.rs`, `gen_vectors.py` | new: the Python modules' answers on the same pictures |
| `crates/lane-node` | `aav_lane_finder/lane_finder.py`: the same name, topics, QoS, parameters, logs. `params.rs` is `aav_things_rs`'s, with a three-number list |
| `test/test_lane_finder_node.py` | new |
| `test/capture_frames.py` | new: front camera frames from Ravensim, for the vectors |

Why no OpenCV crate: the `opencv` crate binds OpenCV's C++ through clang at build time,
which the car kit's offline ARM build can't count on. Every OpenCV call the Python makes
has a short pure-Rust form (above), and the vectors show the same answers.

## Where it differs, on purpose

The README's table: the debug picture's drawing, a wrong-length or tilted-sensor
distortion (a warning, not an error per picture), and `-p` against a params file.

## Changes

- 2026-11-18: written; the Python's answers on its pictures.
