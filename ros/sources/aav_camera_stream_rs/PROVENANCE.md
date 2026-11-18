# Provenance

Ported for `ecolo-av` on 2026-11-18: a Rust rewrite of our own Python node and modules,
`aav_camera_stream` (`ros/sources/aav_camera_stream` at commit `c6b40fe9`). The Python
stays, selectable with `camera_stream_impl:=python`, and runs whenever a detector or
`decoder:=opencv` is asked for.

| Part | From |
|---|---|
| `crates/stream-core/src/decoders.rs` | `decoders.py`: `check`, the GStreamer presets, `ffmpeg_command`, `gst_command`, `command`, `pixi_gst`, `gst_works`, `choose`, `child_env`. Python's `str.format` (four names) and `shlex.split` (POSIX mode) rewritten for the pipelines |
| `crates/stream-core/src/pipe.rs` | `decoders.py`'s `PipeSource`: the child in its own process group, stopped by `killpg` |
| `crates/stream-core/src/reader.rs` | `reader.py`'s `Reader` |
| `crates/stream-core/src/latest.rs` | `latest.py`: `Latest`, `Rate`, `Delays`, `stamp_ns`; `next_pause` is `reader.py`'s retry rule |
| `crates/stream-core/src/lens.rs` | `lens.py`: `Lens`, `parse_ini`, `parse_yaml`, `load`, `scaled`, `identity`, `projection`, `Undistorter`. OpenCV's `getOptimalNewCameraMatrix`, `undistortPoints` (5 rounds), `invert` (3x3 closed form), `initUndistortRectifyMap` to `CV_16SC2` and `remap` `INTER_LINEAR` with a black border (Apache-2.0; the formulas, not its code); the fixed-point remap is `aav_lane_finder_rs`'s `Tap` |
| `crates/stream-core/src/timecode.rs` | `timecode.py` |
| `crates/stream-core/src/models.rs` | `models.py`'s detector names only (no models) |
| `crates/stream-core/src/preview.rs` | `camera_stream.py`'s `preview`: OpenCV's `INTER_AREA` shrink as area averaging; JPEG by the `jpeg-encoder` crate |
| `crates/stream-node` | `camera_stream.py`: the same name, topics, QoS, parameters, diagnostics, logs. `params.rs` is `aav_lane_finder_rs`'s, with text |
| `gen_vectors.py`, `crates/stream-core/tests/vectors.rs` | new: the Python modules' answers |
| `test/test_camera_stream_node.py` | new |

Why no OpenCV, GStreamer or FFmpeg crate: the decoders stay child programs (JetPack's
`gst-launch-1.0` on the Orin), and the `opencv` crate binds C++ through clang at build
time, which the car kit's offline ARM build can't count on.

## Changes

- 2026-11-18: written; the Python's answers in the vectors, both nodes on the same test
  stream.
