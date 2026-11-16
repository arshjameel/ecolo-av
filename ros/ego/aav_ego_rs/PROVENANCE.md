# Provenance

Ported for `ecolo-av` on 2026-11-15 (TODO 185): a Rust rewrite of our own Python nodes and
modules, `aav_ego` (Ben's code, 10 Oct onward; `ros/ego/aav_ego` and
`ros/bringup/aav_description` at commit `bc5a1cba`). No outside code: nothing to add to
`docs/acknowledgements.md` but CPython's `hypot` (below). The Python stays, as the default.

| Part | From the Python |
|---|---|
| `crates/ego-core/src/ackermann.rs` | `aav_ego/ackermann.py`: `Limits`, `EgoModel`, `steering_angle`, `slew_speed`, `ease_speed`, `curvature` |
| `crates/ego-core/src/dynamics.rs` | `aav_ego/dynamics.py`: `Body` and its limits, `describe` |
| `crates/ego-core/src/pedal.rs` | `aav_ego/pedal.py` |
| `crates/ego-core/src/lag_lead.rs` | `aav_ego/lag_lead.py`: `LagLead`, `LeadTimes`, `LeadLimits`, `lead_wanted`, `LagAwareWatch`; the mode is an enum |
| `crates/ego-core/src/stop_law.rs` | `aav_description/stop_law.py`: `StopCurve`, `from_car_model`, `from_numbers`, `loop_lag` |
| `crates/ego-core/src/road_ahead.rs` | `aav_description/road_ahead.py` (the readers: `along_of`, `room_at`, `limit_at`) and `aav_ego/road_limit.py` |
| `crates/ego-core/src/trace.rs`, `stop_ahead.rs` | `aav_ego/trace_ahead.py`, `aav_ego/stop_ahead.py`, on the shared `car-model` crate |
| `crates/ego-core/src/look_ahead.rs` | `aav_ego/look_ahead.py`, and `look_ahead_node.py`'s `Frame2D`, `yaw_of`, `nearest_index` |
| `crates/ego-core/src/frames.rs` | new: the newest of each TF link, in place of tf2's buffer |
| `crates/ego-core/src/lib.rs` `hypot` | CPython 3.12's `vector_norm` (`Modules/mathmodule.c`), for 2 numbers: Python's own `math.hypot` and `math.dist` |
| `crates/ego-node` | `aav_ego/ego_node.py` and `look_ahead_node.py`: the same node names, topics, QoS, parameters |
| `gen_vectors.py` | new: the Python modules' answers, for `tests/vectors.rs` |

CPython is under the PSF License; the 15 lines of `hypot` follow its algorithm (not its
code), so that the Rust gives Python's last bit. `docs/acknowledgements.md` has its row.

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| Processes | two: `ego`, `look_ahead` | one, holding both nodes (the same names) |
| The trace's speed loop | `car_model.SpeedLoop` (f64) | spine-core's own (f32): within 4e-8 on the vectors |
| TF | tf2's buffer, the newest time the chain shares | the newest of each link |
| `vehicle.yaml`, the body outline | read by the nodes | ego reads `vehicle_yaml` (else the installed one); look_ahead gets `body` from `ego.launch.py` |
| A live parameter set out of range | refused, with the reason | refused, with the reason (rclrs' `validate`) |
| A start-only parameter set while running | refused: "restart ego" | refused: read only |
| `look_ahead:=false` | one node fewer | `--ego-only` |

## Changes

- 2026-11-15: written; `ego_impl:=` in `ego.launch.py` and `sim.launch.py`.
