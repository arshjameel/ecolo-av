# aav_pose_check

## Overview

**NDT leads, the GPS checks** (TODO 210; Ben, 18 Nov: "The gps is seeming to fail").
A challenger beside today's default (`pose_source:=gps`), switched on with
`pose_source:=ndt`. Our own code, in Rust; the idea is Autoware's (NDT the main pose,
the GNSS only to start and to check).

```
/localization/ndt/pose ───┐                  ┌─▶ /localization/pose_check/ndt_pose ─▶ ekf_global (pose0)
/localization/ndt/status ─┤                  ├─▶ /localization/pose_check/gps_odom ─▶ ekf_global (odom1)
/localization/gps/odom ───┼─▶ pose_check ────┼─▶ /localization/pose_check/fix ──────▶ geofence
/sensors/gps/fix ─────────┤  (pose-check-    ├─▶ /localization/pose_check/status ───▶ sensor watchdog
/localization/amcl/pose ──┤      core)       └─▶ /localization/ndt/initial_pose ────▶ ndt
/localization/odom/global ┘
```

| Source | When | ekf_global takes | Status |
|---|---|---|---|
| **NDT** | NDT tracks, a fit in the last second, not judged wrong | NDT's fits only | OK; WARN if the GPS is in doubt (NDT carries on) |
| **GPS** | NDT lost, stale or judged wrong; the GPS healthy | the GPS's fixes only | WARN |
| **HOLD** | neither | nothing: it runs on the wheels and IMU | ERROR: the sensor watchdog stops the car |

**Why not the GPS at a low weight, beside NDT?** A GPS that drifts slowly (metres over a
minute: no jump, never silent) would still pull the pose by its share, all the time.
Held out, it moves the pose not at all. It is still compared with NDT every fix.

### Who is wrong when they disagree

| Check | Over | Disagree when |
|---|---|---|
| the gap against both spreads, d'(Σ_ndt + Σ_gps)⁻¹d | 5 s of pairs (at least 5) | mean over 13.8 (99.9 %, 2 numbers) **and** mean gap over 1.5 m |

Then: NDT's median score per point over the window at `trust_score` or more → **the GPS
is wrong**; else, if the GPS's own checks pass → **NDT is wrong**. The verdict holds until
the two agree again (gap under 1.0 m), so a GPS judged wrong is never the fall-back.

The GPS's own checks: a fix (NavSatFix status 0 or more, no NaN), its spread at most
`gps_max_std`, fresh (`gps_max_age`), and no jump (faster than 15 m/s plus 5 m: held
until it agrees with NDT again).

### When NDT is lost: the seed

| First choice | Then | Else |
|---|---|---|
| AMCL's pose, if it has converged (fresh, 0.5 m and 10 degrees or better) | the GPS's position, with the fused heading, if the GPS is trusted | none: HOLD, the car stops |

One seed at once, then one every `reseed_every` (5 s) while NDT stays lost. NDT also
starts again from `ekf_global` by itself, which, with NDT out, the GPS now leads.
Every hand-over and every seed is logged.

## Interface

### Crates

| Crate | Holds |
|---|---|
| `crates/pose-check-core` | the logic, no ROS: `Checker`, `Settings`, the doubts, the verdict, the seed |
| `crates/pose-check-node` | the ROS node, binary `pose_check` |

### Node `pose_check`

| Direction | Topic | Type |
|---|---|---|
| sub | `/localization/ndt/pose`, `/localization/ndt/status` | PoseWithCovarianceStamped, DiagnosticStatus |
| sub | `/localization/gps/odom` (navsat_transform), `/sensors/gps/fix` | Odometry, NavSatFix |
| sub | `/localization/amcl/pose` (where AMCL runs), `/localization/odom/global` (the heading) | PoseWithCovarianceStamped, Odometry |
| pub | `/localization/pose_check/ndt_pose`, `/localization/pose_check/gps_odom` | the passed poses, as they came |
| pub | `/localization/pose_check/fix` | Odometry: every pose passed, of either |
| pub | `/localization/pose_check/status` (10 Hz) | DiagnosticStatus: `source`, `ndt`, `gps`, `gap_m`, `nees`, `score`, `handovers`, `seeds` |
| pub | `/localization/ndt/initial_pose` | PoseWithCovarianceStamped: the seed |

Parameters: `config/pose_check.yaml` (each number's reason is beside it).
`sim.launch.py` sets `trust_score` on the campus (`worlds.CAMPUS_POSE_TRUST`).

### Where it is wired (`pose_source:=ndt`)

| Piece | Change |
|---|---|
| `localization.launch.py` | starts `pose_check`; `ekf_global` reads `ekf_global_ndt.yaml` then `ekf_global_ndt_leads.yaml`: pose0 and odom1 from `pose_check` |
| `sim.launch.py` | starts NDT; the watchdog's `gps` input becomes `gps_check` (the same checks, warn only), and `pose_check` (its status, stop at ERROR) is added; the geofence's "a fix arrived" is `/localization/pose_check/fix` |
| `car.launch.py` | refuses it: no lidar map of the real lot yet |

## Invariants

- Only ever holds a pose back; never makes one up. Never TF.
- NDT and the GPS never both reach `ekf_global` at once.
- A GPS judged wrong, or jumped, is never the fall-back until it agrees with NDT again.
- HOLD is an ERROR status: the sensor watchdog's lock stops the car (the safety locks are
  the same ones; nothing here is a new lock).
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean; no ROS in `pose-check-core`.

## Side Effects

- Publishes the topics above; logs every hand-over and seed.

## Verification

```sh
cd ros/localization/aav_pose_check && cargo test --locked -p pose-check-core
pixi run build && colcon test --packages-select aav_pose_check && colcon test-result --verbose
pixi run measure controllers --ndt-leads --gps-fault drift
```

`tools/test.sh` runs the first. `colcon test` adds the node's own test
(`test/test_pose_check_node.py`: NDT leads, then NDT lost and the GPS takes over).
