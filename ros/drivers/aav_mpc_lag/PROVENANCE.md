# Provenance

The `mpc-lag` crate ports our own Python lag-aware MPC to Rust, in Rust's style: enums
for errors, `Result`, a sparse-row matrix for the QP's constraints. No outside code.

## Where it came from

| | |
|---|---|
| Repo | this one (`AAV-2025-2026/ecolo-av`) |
| Commit | `bc5a1cba` (15 Nov 2026) |
| Licence | ours |

| Ours (Rust) | Ported from (Python) |
|---|---|
| `src/path.rs` | `ros/drivers/aav_track/aav_track/path.py` (`Path`, `Foot`, `wrap`) |
| `src/model.rs` | `ros/drivers/aav_track/aav_track/lag_model.py` (`step`, `pedals`, `dead_ticks`, the car's variants) |
| `src/qp.rs` | `ros/drivers/aav_track/aav_track/qp.py` (OSQP's ADMM; the algorithm is OSQP's, `docs/acknowledgements.md`) |
| `src/tracker.rs` | `ros/drivers/aav_track/aav_track/trackers/mpc_lag.py`; `planned_deceleration` from `aav_drive_to_point/approach.py` |
| `crates/mpc-node/src/main.rs` | the shape of `aav_mppi_lag`'s node; its map correction and checks are used, not copied |

## Changes since

| Date | Change |
|---|---|
| 15 Nov | `qp.rs` keeps A by rows, only its non-zeros (the Python's is dense): the same iterations, less work each |
| 15 Nov | The node forgets the estimate when a path starts with none running (the car was held since the last), where the Python carries it on |
| 15 Nov | TODO 176: the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour |
| 20 Nov | TODO 213: reversing. `src/gears.rs` ports `aav_track/gears.py` (the cut at cusps, the mirror, `Gears`); `src/geared.rs` what `track_node.py` does around a tracker. Ours beyond the Python: the gear changes only standing (0.05 m/s, mppi_lag's, not 0.15), the set speed held at 0 within 0.3 m of a cusp (the MPC crept past it), the estimate's steering turned round at a cusp (`MpcLag::turn_round`), the MPC's cruise set to the reverse limit backwards, and the progress check started again at each stretch (mppi_lag's). A forward path is unchanged: the vectors still pass |
