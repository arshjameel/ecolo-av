# Provenance

Ported for `ecolo-av` on 2026-11-17: a Rust rewrite of our own Python node and modules
(Ben's code): `ros/perception/aav_detector`'s `stop_sign_watcher.py`, `stop_sign.py`, and
`normal_label` (`boxes.py`) and `is_optical` (`ground.py`), at commit `c0fd92db`. No
outside code: nothing new for `docs/acknowledgements.md`. The Python stays, as the
fall-back (`stop_sign_impl:=python`).

| Part | From the Python |
|---|---|
| `crates/stop-sign-core` | `stop_sign.py`: `StopRule`, `StopSignWatcher` (its states as the enum `State`), `sign_distance_m`, `sign_point`, `biggest_sign`, `lidar_sign`; `normal_label`, `is_optical`; the node's `facing_forward` |
| `crates/stop-sign-node` | `stop_sign_watcher.py`: the same node name, topics, QoS and parameters; the TF buffer is `aav_behaviour_rs`'s, reused |
| `gen_vectors.py` | new: `stop_sign.py`'s answers, for `tests/vectors.rs` |
| `test/test_stop_sign_node.py` | new: the process, both languages |

## Where it differs

With `lidar_distance` (off by default; the simulator turns it on with things named), TF
comes from `aav_behaviour_rs`'s buffer, not tf2's: the same lookups at the picture's stamp.
