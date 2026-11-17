# Provenance

Ported for `ecolo-av` on 2026-11-17 (TODO 185): a Rust rewrite of our own Python node,
`aav_lidar` (`ros/perception/aav_lidar` at commit `3e1aa38e`). No outside code: nothing to
add to `docs/acknowledgements.md`. The Python stays, selectable with `lidar_impl:=python`.

| Part | From the Python |
|---|---|
| `crates/lidar-scan/src/lib.rs` | `aav_lidar/scan.py`: `ScanShape`, `body_box`, `flatten`, `SELF_MARGIN`, `FRONT_MARGIN` (`rotation` and `to_base` are `aav_voxels_rs`'s `Mount`, reused) |
| `crates/lidar-scan/src/ground.rs` | `aav_lidar/ground.py`: `ground_hit`, `worst_ground_height`, `max_safe_roll`, `VLP16_ELEVATIONS` |
| `crates/lidar-scan/src/tests.rs` | `test/test_scan.py`, `test/test_ground.py` |
| `crates/lidar-scan-node` | `aav_lidar/lidar_scan.py`: the same name, topics, QoS, frame and parameters |
| `gen_vectors.py`, `tests/vectors.rs` | new: `scan.py`'s ranges on 7 clouds |
| `test/test_lidar_scan_node.py` | new, after `aav_voxels_rs`' node test |

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| `vehicle.yaml` | read by the node | read by `navigation.launch.py`, passed as `lidar_xyz`, `lidar_rpy`, `body` |
| A bad cloud (no x field, too short) | an exception in the callback | a warning, every 2 s at most; the cloud is skipped |
| A bad shape (an increment of 0) | a division by zero | the node refuses to start |

## Changes

- 2026-11-17: written; `scan.py`'s ranges the same float32 on 7 clouds; `lidar_impl:=` in
  `navigation.launch.py`, `sim.launch.py` and `car.launch.py`.
