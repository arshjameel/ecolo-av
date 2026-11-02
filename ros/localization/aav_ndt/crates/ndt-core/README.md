# ndt-core

## Overview

NDT's maths with no ROS: where the car is on a saved lidar map, in x, y and yaw. The map
becomes one bell curve per cube; each scan is slid and turned until its points sit best
on them (Biber 2003; Magnusson 2009). `ndt-node` only turns messages into calls here.
The design: `docs/designs/2026-10-29_ndt.md`.

```
lidar points ──Mount──▶ base_link ──ScanFilter──▶ ~500 points ──Matcher (4, 2, 1 m)──▶ pose
                                   ground, body, 45 m,     guess = last ∘ odom⁻¹ ∘ odom
                                   inf, thin to 1 m
```

## Interface

| Module | Item | Does |
|---|---|---|
| `pose` | `Pose2`, `wrap`, `Mount` | x, y, yaw: `compose`, `inverse`, `between`, `apply`, `lerp`; the lidar's mount from `xyz`/`rpy` |
| `cloud` | `ScanFilter`, `Ground`, `BodyBox`, `thin` | drop non-finite, far (45 m), the car's body, the ground (`Above(0.3)` or `AboveLowest` per cell); one mean point per 1 m cube |
| `cloud` | `read_xyz(CloudView) -> Result<Vec<Point>, CloudError>` | a `PointCloud2`'s x, y, z (float32 or float64) from its bytes |
| `map` | `CubeMap::build(points, MapSettings)` | a curve per cube with ≥ 6 points; spreads floored at 1 % of the widest and (1 cm)² |
| `map` | `CubeMap::neighbours(p)` | the point's cube and the 6 that share a face |
| `score` | `Gauss::new(side, outlier_ratio)`, `evaluate` | Magnusson's score, gradient and Hessian in (x, y, yaw) |
| `matcher` | `Matcher::new(map, cubes, settings)`, `align(scan, guess) -> Result<Match, MatchError>` | Newton: damping, a 0.1 m / 2° step cap, Armijo backtracking, coarse to fine |
| `guess` | `OdomBuffer`, `predict` | 1 s of the local EKF, interpolated at a cloud's stamp; the guess |
| `localizer` | `Localizer`, `LocalizerSettings`, `Outcome`, `Poor`, `Report`, `Tracking` | per cloud: the guess, the match, good or poor (low score, no convergence, or a jump from the guess while tracking: `max_correction`), LOST after 5 poor, restart from `ekf_global`; `covariance`, `raw_covariance` (-H⁻¹) |
| `pcd` | `read`, `write` | the map file (PCD v0.7, binary or ASCII) |

## Invariants

- No ROS, no unsafe, no `unwrap` outside tests. Metres, radians, seconds (REP-103).
- 3 numbers only (x, y, yaw, design N1): height, roll and pitch are the map's.
- A step never moves more than `step_m` or turns more than `step_rad`, and never lowers
  the score (the line search halves it until it doesn't).
- The same scan and guess always give the same answer (thinning is in a fixed order).
- Cube sizes are the matcher's, not the map file's.

## Verification

```sh
cd ros/localization/aav_ndt && cargo test --locked -p ndt-core
cargo test --release --locked -p ndt-core --test matching -- --nocapture   # times per match
```

| Test (`tests/`) | Checks |
|---|---|
| `matching.rs` | a known shift and turn within 1 cm and 0.1°; gradient and Hessian against finite differences (relative 1e-6); 1.5 m and 10° off on a lot like Gazebo's pulls in (needed the 4 m cubes) |
| `localizer.rs` | a drive on odometry drifting 5 % and 2°/s; LOST after five poor fits, then a restart from `ekf_global`; a jump while tracking is poor, not after a start |

The worlds are made up in `tests/common`: boxes on flat ground and a 16-beam lidar like
Gazebo's, ray-cast, 3 cm of noise.
