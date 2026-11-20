# Provenance

Ported for `ecolo-av` on 2026-11-15 (TODO 185): a Rust rewrite of our own Python node,
`aav_voxels` (Ben's idea and code, 30 Oct; `ros/perception/aav_voxels` at commit
`5dbd51b2`, where `cubes.py` was last changed by `6ded423c`). No outside code: nothing to
add to `docs/acknowledgements.md`. The Python stays, as the default.

| Part | From the Python |
|---|---|
| `crates/voxels/src/lib.rs` | `aav_voxels/cubes.py`: `CubeShape`, `Ground` (now `Road`, an enum), `fit_ground`, `lowest_per_cell`, `nearest_per_cube`, `cube_centres`, `cubes` |
| `crates/voxels/src/mount.rs` | `aav_lidar/scan.py`: `rotation`, `to_base` |
| `crates/voxels/src/cloud.rs` | new: `sensor_msgs_py`'s `read_points_numpy` and `create_cloud_xyz32`, as bytes |
| `crates/voxels-node` | `aav_voxels/voxels_node.py`: the same name, topics, QoS, frames, status keys |
| `crates/voxels/src/tests.rs` | `aav_voxels/test/test_cubes.py`, the cases that need no lidar turn |
| `test/capture_python_vectors.py` | new: `cubes.py`'s answers on `aav_voxels/test/lot.py`'s clouds and others |

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| The plane | `numpy.linalg.lstsq` (SVD) | the normal equations about the points' mean: the same to 1e-15 on the vectors |
| Candidates exactly on one line | lstsq's least-norm plane, then the checks | no plane: the road taken as flat |
| `vehicle.yaml` | read by the node | read by `navigation.launch.py`, passed as `lidar_xyz`, `lidar_rpy`, `body` |
| A bad cloud (no x field, too short) | an exception in the callback | a warning, every 2 s at most; the cloud is skipped |
| A bad shape (size 0, min above max) | cuts nonsense | the node refuses to start |

## Changes

- 2026-11-15: written; the Python's answers on 9 recorded clouds and one whole turn;
  `voxels_impl:=` in `navigation.launch.py`, `sim.launch.py` and `car.launch.py`.
- 2026-11-15 (TODO 176): the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour.
- 2026-11-16 (TODO 119): ours, not ported: the kerbs' step finder (`steps.rs`), the node's
  `steps:=true` and `/perception/voxels/steps`. The Python has none; the cubes are unchanged.
- 2026-11-20 (TODO 189): ours: `kerbs_only` drops the steps at the foot and up the face
  of anything taller than a kerb before the memory keeps them (a walker's foot, under
  every beam, held the car for good in `sim-check --moving`). The cubes are unchanged.
- 2026-11-20 (TODO 176): `cloud.rs` reads through the shared `cloud-view` crate (`ros/bringup/aav_description/cloud-view`), the four copies of `CloudView` now one; the same points, all vectors pass.
