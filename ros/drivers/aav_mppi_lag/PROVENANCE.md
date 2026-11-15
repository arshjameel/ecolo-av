# Provenance

The `mppi-lag` crate ports Nav2's MPPI controller to Rust. It is rewritten in Rust's own
style, not transliterated: enums for the critics, a trait for the car model, no xtensor.

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Package | `nav2_mppi_controller` |
| Commit | `645abd95f2be02a13ca539b29c2ddc065db34f89` (Jazzy, `vendor/ros2-src.repos`) |
| Licence | Apache-2.0 (the package's `LICENSE.md`; Samsung Research America and the Nav2 authors) |
| Read 7 Nov 2026 | `src/optimizer.cpp`, `include/.../motion_models.hpp`, `include/.../tools/utils.hpp`, `src/path_handler.cpp`, `src/noise_generator.cpp`, `src/critics/*.cpp` |

## What was taken

| Ours | Nav2's |
|---|---|
| `optimizer.rs` | `Optimizer`: noise on the sequence, rollouts, critics, control cost (gamma / std²), softmax with temperature, retries on all-collide, shift |
| `smoothing.rs` | `utils::savitskyGolayFilter`, the same 9 taps and history |
| `critics.rs` | `ConstraintCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic` (its three modes), `PreferForwardCritic`; `findPathFurthestReachedPoint`, `findPathCosts`, `findClosestPathPt`; `posePointAngle` |
| `path.rs` | `PathHandler`: closest pose within `max_robot_pose_search_dist`, prune, `prune_distance` ahead, cut at the costmap's edge; `enforce_path_inversion` (`utils::findFirstPathInversion`, `isWithinInversionTolerances`) |
| `costmap.rs` | the costmap's cost codes; `FootprintCollisionChecker`'s edge cost; `InflationLayer::computeCost` |

## What changed, and why

| Change | Why |
|---|---|
| The car model: ego's ramps, the spine's speed loop, throttle and brake lag, steering rate, linkage lag, deadbands, dead times (`lag.rs`, `ego.rs`) instead of Nav2's velocity clamps | the point of the port: plan with the lag (0.3, Ben, 7 Nov) |
| Samples in (speed, steering angle), not (vx, wz) | the actuators take an angle; full lock is a plain clamp |
| Samples clipped only to the range and full lock; ego's ramps are in the model | Nav2's per-step clipping of noise drifts samples slower (nav2.yaml's ax_max 3.0 workaround) |
| Noise correlated step to step (AR(1), `noise_correlation`, 0 = Nav2's) | through a lagging steering, white noise averages away and MPPI lost the path on a slow car (`docs/mppi-lag.md`) |
| Sends element 0 of the sequence, not 1 | the model's state already holds every command sent (the dead times) |
| A reset starts from the last command, not zeros | taking over mid-drive shouldn't brake first |
| No `TwirlingCritic`, `ObstaclesCritic`, `VelocityDeadbandCritic` | nav2.yaml runs none of them |
| Our own random numbers (xoshiro256++) and `std::thread::scope` | no dependencies: the crate builds offline |
| The lag model follows the shared `car-model` crate's rules (exact first-order lags, the deadband on the motor's setpoint, dead times floor(t/dt + 0.5) steps, the bicycle with the new speed and steering at the mid-step heading) | one model of the car for every 0.3 algorithm; at the merge, that crate implements `CarModel` |
| `/drivers/nav/lag_aware`: true while a path runs with `model: lag` | ego's lead (0.3) turns off for a driver that already plans with the lag |
| Goal and progress checks (`follower.rs`) | controller_server's `SimpleGoalChecker` and `PoseProgressChecker`, as aav_track's `checks.py` |

## Changes since

| Change | Why |
|---|---|
| 14 Nov: a costmap older than `costmap_timeout` (0.8 s: controller_server's `costmap_update_timeout` 0.3 s plus the local costmap's 2 Hz publish period) publishes zero and fails the path with CONTROLLER_TIMED_OUT; a path gets that long for its first costmap (`age.rs`) | Nav2's controller waits for a current costmap, then does the same; ours went on with the last one for ever (the gap list, 14 Nov). Nav2 checks its layers in-process; we see only the published costmap, hence the publish period |
| 14 Nov: reversing. The lag model has the spine's gear (it changes only while stopped; the speed loop works along it, as spine-core's); ego's ramp speeds up and slows down either way and changes direction through a stop (ackermann.py `slew_speed`); `PathAngleCritic`'s modes 1 and 2; the path followed one stretch at a time at its cusps (Nav2's `enforce_path_inversion`). Ours differs at a cusp: the critics' goal is the cusp while one lies ahead (Nav2's is always the path's end), and the goal check waits until the last stretch | Ben, 14 Nov: "The car can reverse. I don't know why we keep setting it to no". The cusp as the goal makes the car stop there as at a goal; Nav2 stops only because the path ends |
| 14 Nov: `LaneCritic` (ours; Nav2 has none, `lane.rs`): the outline past the road ahead's soft edges (lane lines) costs, squared (100 per m²); past its hard edges (kerbs) far more (1000 per pose, plus per m), not a collision. Silent without a lane or where an edge is NaN. Runs only when the critics list names it | TODO 169: keep the car in its lane once the lane map gives the road ahead its edges (the whole-car design, "The road ahead") |
