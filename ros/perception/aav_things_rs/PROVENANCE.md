# Provenance

Ported for `ecolo-av` on 2026-11-17 (TODO 185): a Rust rewrite of our own Python node and
modules, `aav_things` (`ros/perception/aav_things` at commit `933cd46d`), with the pinhole
of `aav_detector`'s `ground.py`. No outside code: nothing new for
`docs/acknowledgements.md` but the rows for AB3DMOT (the idea) and NumPy's pairwise sum (below). The
Python stayed, selectable, until 20 Nov.

| Part | From the Python |
|---|---|
| `crates/things-core/src/groups.rs` | `aav_things/groups.py`: `GroupShape`, `Group`, `group`, `to_frame` |
| `crates/things-core/src/tracker.rs` | `aav_things/tracker.py`: `TrackerShape`, `Track`, `Tracker`, `predict`, `update`, `RadarCluster`, `ground_radial_speed`. The naming hook is `Tracker::rename` |
| `crates/things-core/src/radar.rs` | `aav_things/things_node.py`'s `use_clusters`, its maths |
| `crates/things-core/src/background.rs` | `aav_things/background.py`: `Background`, `new_only`, `ChunkBackground` (the PCD reader is `ndt-core`'s, the chunk grid `map-chunks`') |
| `crates/things-core/src/tf_wait.rs` | `aav_things/tf_wait.py`: `Step` (an enum), `WaitShape`, `decide`, `Waiting` |
| `crates/things-core/src/naming.rs` | `aav_things/naming.py`; `aav_detector/ground.py`'s `Lens`, `Mount` (`CameraMount`), `lens_from_k`, `is_optical` (`Axes`), `rotation` |
| `crates/things-core/src/looks.rs` | `aav_things/looks.py` |
| `crates/things-core/src/cloud.rs` | new: `sensor_msgs_py`'s `read_points`, as bytes |
| `crates/things-core/src/lib.rs` `pairwise_sum` | numpy's pairwise sum of float64s (`loops_utils.h.src`), so a radar cluster's mean is numpy's; BSD-3-Clause, the algorithm, not its code |
| `crates/things-core/src/tests.rs` | `test/test_things.py`, `test_background.py`, `test_naming.py`, `test_tf_wait.py`, `test_looks.py` (their random runs with our own generator) |
| `crates/things-node` | `aav_things/things_node.py`: the same name, topics, QoS, parameters, markers; `params.rs` is `aav_behaviour_rs`'s, with whole numbers |
| `gen_vectors.py`, `tests/vectors.rs` | new: the Python modules' answers |
| `test/test_things_node.py` | new |

The TF buffer is `aav_behaviour_rs`' `behaviour-core::tf`, given `newest()` for this
node (tf2's `lookup_transform(..., Time())` stamp).

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| `expire` while the last cloud's stamp is exactly 0.0 | `self.time or now`: 0.0 reads as no time | 0.0 is a time (only a test's clock starts there) |
| A cloud waiting for TF | `tf2_ros.Buffer.can_transform` | `behaviour-core`'s buffer: the same 10 s history, interpolated as tf2 |
| `background_chunks` with no `chunks.yaml` | an exception at start-up | the node refuses to start, saying why |
| A chunk's PCD that can't be read | an exception in the callback | a warning; the cubes kept, all of them |
| A `/map` whose cells don't fill width × height | an exception | a warning; no background |

## Changes

- 2026-11-17: written; the Python's answers on 180 clouds, 300 names and 20 background
  cases; `things_impl:=` in `navigation.launch.py`, `sim.launch.py` and `car.launch.py`.

**20 Nov:** the Python (`aav_things`) was removed (Ben, 16 Nov: "lets remove python were possible"; tag `save/python-things`). `gen_vectors.py` went with it; `vectors.json` is a golden fixture now.
- 2026-11-20 (TODO 176): `cloud.rs` now re-exports the shared `cloud-view` crate's `CloudView` and `read_columns` (`ros/bringup/aav_description/cloud-view`); the same values, all vectors pass.
- 2026-11-20 (TODO 195): ours: `background.rs` reads the 3 x 3 chunks and grows the walls on many cores (rayon); the walls grow row by row (each row the rows within reach), the same cells (a new cell-by-cell test). `examples/timing.rs` times it.
