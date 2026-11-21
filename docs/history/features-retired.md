# Retired features

Rows moved here, word for word, from [`docs/features.md`](../features.md) once the
feature was removed or archived (20 Nov), so the live table stays short. Each row says
when and why it went, and where its code and scores are kept.

## Perception

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Lane keeping (`aav_lane_geometry`) | Steers back to the lane's centre: on the bench, from 0.3 m off to within 5 cm. **Removed 19 Nov** (TODO 264; Ben: "drop there code all together"): lanes are perception, the costmap prices them, navigation drives. Git tag `save/lane-followers`; [its scores](lane-followers.md) | the 2025-26 team | 2026-10-10 | sim |
| [The lane follower in Rust](lane-followers.md) (`aav_lane_follower_rs` `control_interface`, `lane_follower_impl:=`), the default; the challengers pure pursuit and Stanley (`lane_follower:=`) | Ben, 19 Nov: "We also need to port the lane following code.or find a better version." Rust where possible. The 2025-26 PID and its five states ported as a drop-in: the Python's commands, status lines and states on 2,797 shared events (its logic moved, unchanged, into `lane_follow.py`). Beside it, two challengers on the camera's fitted lane centre, each steering from where the car will be after its lag (`vehicle.yaml`): pure pursuit with a speed-scaled look-ahead, and Stanley. `measure lane` now counts the steering's flips and the seconds with the lane lost. Scored, 3 runs each (TODO 262): the Rust PID as the Python; pure pursuit 2.4 cm RMS on Library Road against the PID's 3.9-4.1, 1 flip a minute against 187, but slower back from an offset, so the PID stays the default. **Removed 19 Nov** (TODO 264; Ben: "drop there code all together"): lanes are perception, the costmap prices them, navigation drives. Git tag `save/lane-followers`; [its scores](lane-followers.md) | Ben | 2026-11-19 | sim |

## Backends

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Gazebo backend (`aav_gazebo`) | The car in 3D with Ackermann steering, an IMU and GPS, to test Nav2 before the car. **Removed 16 Nov** (Ben: "lets do it"): Ravensim took its place; its nodes, worlds and sim models live on in `aav_sim_models` ([Gazebo, as it was](gazebo-as-it-was.md)) | Ben | 2026-10-09 | sim |

## HUD

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| HUD back end (Flask, `hud_server`): **archived 25 Oct** to `archive/2026-27-flutter-hud/` | Kept the 2025-26 touchscreen working, on our topics, until the raylib HUD replaced it (Q29) | the 2025-26 team, ported by Ben | 2026-10-10 | sim |
| Flutter HUD app: e-stop button, Cancel, camera: **archived 25 Oct** to `archive/2026-27-flutter-hud/` | The app could never reach its back end; then its e-stop stopped the bench car in 0.3 s | the 2025-26 team, changed by Ben | 2026-10-11 | sim |

