# debug-core

## Overview

The debug HUD's logic, with no ROS and no raylib, so it is tested on any laptop. The
`debug_hud` program (`debug-app`) feeds it the lidar's clouds, the localizers' poses,
`/tf` and every sensor's topic, and draws what it says: the cubes around the car, where
each localizer puts the car, and the panel. Design: `docs/designs/2026-10-29_debug-hud.md`.

```
cloud (lidar_link) --static tf--> base_link --local EKF at the cloud's stamp--> odom
                                                                  |
                                                       cubes (20 cm), a height band each
                                                                  |
                                     kept by cell, fading over 2 s (by stamps), nearest kept
map-frame layers (EKF global, AMCL, NDT, truth, plan, particles, NDT's map)
                                        --latest map -> odom from /tf--> odom
```

## Interface

| Module | Holds |
|---|---|
| `config` | `DebugConfig` from `config/debug_hud.yaml` (built in); `Topic`, `Frame`, `Number`, `NODE_NAME` = `debug_hud` |
| `frames` | `Stamp` (ns), `Transform3`, `StaticFrames::lookup` (follows a chain up to base_link), `MapToOdom` |
| `poses` | `PoseBuffer`: 1 s of local EKF poses; `at(stamp)` interpolates, or says `TooOld` / `TooNew` |
| `cloud` | `positions(&CloudView)`: x, y, z from a PointCloud2's bytes |
| `cubes` | `build(points, cloud_to_base, car, stamp, settings)` -> `CubeScan`; `Band` (ground ... higher) |
| `scan` | `place(...)`: a cloud start to finish, or a `ScanError` |
| `persist` | `KeptCubes`: each scan's cubes kept by cell; `Fade` (four steps over `keep_cubes_for`); `FadedCube` |
| `ndt` | `NdtStatus::from_message` (TRACKING / WAITING / LOST, score per point, ms per match); `MapFile::read` (a PCD, with ndt-core's reader) into `MapCells` (1 m, 3 points or more) and its `Footprint` from above (the points thinned to 15 cm, one `Ellipse` per column at 2 sigma, `covers`) |
| `keys` | `Layers` (L the layout: the seeing screen or the panel, K cubes kept or this scan's, M NDT's map, A particles, H the legend), `Layout`, `Action`, `KeyLine` for the on-screen legend |
| `topdown` | the seeing screen's top-downs: `View::fit` (the whole map) and `View::around` (round the car), map metres to window points, `ellipse` |
| `orbit` | `Orbit`: the 3D camera; `Mode` chase / follow / free |
| `track` | `TrackStatus::from_values`: aav_track's `/drivers/nav/track/status` |
| `swap` | keys 1-4: `Tracker::on_key`, `allowed(&Origins)` (SIM only), `Reply` |
| `state` | `DebugState`, `Update`, `Localizer` (EKF local, global, truth, AMCL, NDT), `Source` (the panel's rows); `drawn_cubes()`, `particles_in_odom()`; for the seeing screen `pose_in_map()`, `scan_in_map()` |

It builds on (path dependencies): [`hud-core`](../hud-core/README.md) (`health`,
`origin`, `vehicle`), [`map-core`](../map-core/README.md) (`geom`, `car`),
`aav_radar_clusters`' `radar-clustering` (its PointCloud2 reader, with its tests) and
`aav_ndt`'s `ndt-core` (its PCD reader; it brings nalgebra).

## Invariants

- No ROS and no raylib.
- odom frame, z up, metres, radians (REP-103/105). Heights are above base_link: the EKF
  is 2D, so a pitched car tilts the cubes.
- A cloud is placed with the local EKF's pose **at its stamp**, interpolated; outside
  the poses kept, it is dropped (`TooOld`) or retried (`TooNew`).
- Stamps are compared only with stamps, never with a clock (Gazebo's are sim time).
- Map-frame layers need `map -> odom`; without it they are not drawn.
- Over `max_cubes`, the nearest cubes stay and the scan counts the rest (`cut`).
- Rays that hit nothing (inf, NaN) are skipped and counted.
- Kept cubes age by the clouds' stamps; a clock that goes back (a restart, a looped bag)
  or a new cube size starts them again.

## Side Effects

- None, except `ndt::MapFile::read`, which reads the map file it is given.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p debug-core
```

`tools/test.sh` runs it (with `hud-core` and `map-core`).
