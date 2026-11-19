# Provenance

Ported for `ecolo-av` on 2026-11-19: a Rust rewrite of the 2025-26 team's lane follower,
`control_interface`, as it is in this repo's `ros/perception/aav_lane_geometry`
(`scripts/control_interface.py`, with our changes since; the whole history is in that
package's `PROVENANCE.md`). It came from github.com/AAV-2025-2026/Lane_Detection_ROS2 @ main
(345285860e), `src/lane_geometry`, MIT (its `package.xml`). The Python stays, as the
fall-back (`lane_follower_impl:=python`).

The same day, the Python's logic moved, unchanged, into `aav_lane_geometry`'s
`scripts/lane_follow.py` (no ROS), so `gen_vectors.py` can run it on made-up times.

| Part | From |
|---|---|
| `crates/lane-follow-core` `follower.rs` | `lane_follow.py`: `LaneFollower` (the five states as the enum `State`), `Settings`; `lane_speed.py`'s `slow_down` and `stopping_speed` |
| `crates/lane-follow-core` `steering.rs`, `Pid` | the 2025-26 `PID` class |
| `crates/lane-follow-core` `steering.rs`, the rest | new: the lane centre from the fits (as `aav_lane_finder`'s `centre_y`), pure pursuit and Stanley with the car's lag |
| `crates/lane-follow-core` `pyfmt.rs` | new: numbers written as Python's f-strings write them |
| `crates/lane-follow-node` | `control_interface.py`: the same node name, topics, QoS and parameters |
| `gen_vectors.py`, `test/` | new |

The challengers' ideas: pure pursuit (Coulter 1992) and Stanley (Thrun et al. 2006), both
in `docs/acknowledgements.md`. No outside code was copied.

## Where it differs

Nothing the car would notice. Kept as it was, on purpose, to be a drop-in:

- **RESUMING doesn't ramp.** The 2025-26 code adds `accel_rate` x 0.1 to the speed, then
  the lane-keeping command overwrites it, so the first command after the lines come back
  is the full lane-keeping speed (ego still limits the acceleration). Both languages do this;
  fixing it is a change of behaviour, for its own commit.
- **The PID's clock is the wall clock**, the watchdog's the node's, as in the Python.
