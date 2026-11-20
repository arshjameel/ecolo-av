# Provenance

Ported for `ecolo-av` on 2026-11-16 (TODO 185): a Rust rewrite of our own Python node and
modules (Ben's code, 5 Nov onward): `ros/drivers/aav_behaviour`,
`ros/bringup/aav_description` and `ros/safety/aav_safety` at commit `4cdecda3`. No outside
code: nothing new for `docs/acknowledgements.md` (CPython's `hypot` is ego-core's, reused;
its `sum()` is below). The Python stays, as the default.

| Part | From the Python |
|---|---|
| `crates/behaviour-core/src/machine.rs` | `aav_behaviour/behaviour.py`: `Behaviour`, `Settings`, `Stop`, `World`, `Decision`, `diagnostic`; `aav_description/stop_law.py`'s `nav2_cap`. The states, reasons and locks are enums |
| `crates/behaviour-core/src/paths.rs` | `behaviour.py`: `active_plan`, `path_left`, `along_until`, `lane_ahead`, `moving_ahead`, `Seen`, `Crossing` |
| `crates/behaviour-core/src/road.rs` | `aav_description/road_ahead.py`, the writer: `resample`, `build`, `Strip`, `StopOnLine` (the readers are ego-core's) |
| `crates/behaviour-core/src/lanes.rs` | `aav_description/lane_map.py`: `Road`, `Lane`, `LaneMap.lane_at`, `lane_offset`, `load`, `lane_line`, `with_edges` (not `roads_at` or `costmap_block`: the node doesn't use them) |
| `crates/behaviour-core/src/fence.rs` | `aav_safety/geofence.py`: `Polygon` (`contains`, `depth`) |
| `crates/behaviour-core/src/tf.rs` | new: tf2's buffer, the part the node uses (each link's history, interpolated as tf2's `TimeCache`, slerp as tf2's `Quaternion::slerp`) |
| `crates/behaviour-core/src/lib.rs` `py_sum` | CPython 3.12's `sum()` of floats (Neumaier's compensated sum, `Python/bltinmodule.c`), so the goal's distance gets Python's last bit. PSF License; the algorithm, not its code |
| `crates/behaviour-node` | `aav_behaviour/behaviour_node.py`: the same node name, topics, QoS, parameters; `params.rs` is `aav_ego_rs`'s, cut to what this node uses |
| `gen_vectors.py` | new: the Python modules' answers, for `tests/vectors.rs` |

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| A stop sign with no TF at its picture's stamp | `lookup_transform` waits up to 0.1 s (blocking the node), then drops it | dropped at once, with the same warning: the TF can't arrive while the callback runs anyway |
| A fence of 3+ corners under 1 m² | `Polygon` raises in the callback | logged, and no fence |

**20 Nov:** the Python (`aav_behaviour`) was removed (Ben, 16 Nov: "lets remove python were possible"; tag `save/python-behaviour`). `gen_vectors.py` went with it; `vectors.json` is a golden fixture now.
