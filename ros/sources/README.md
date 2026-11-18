# ros/sources

## Overview

**Sensors become telemetry.** Drivers for the car's sensors, each publishing on
`/sensors/<device>/<thing>` (or `/vehicle/battery`) in REP-103 units and frames.

## Subdirectories & Boundaries

| Package | Sensor | Main output |
|---|---|---|
| [aav_sensors](./aav_sensors/README.md) | launch and config for all of them: IMU + Madgwick, GPS (u-blox or NMEA), radar, Velodyne lidar, battery, USB cameras | — |
| [aav_imu_yahboom](./aav_imu_yahboom/README.md) | Yahboom IMU, in Rust (rewritten from the 2025-26 C++) | `/sensors/imu/data_raw` |
| [aav_camera_stream](./aav_camera_stream/README.md) | camera Pis' RTSP streams, decoded, undistorted, detector in-process | `/perception/detections`, `camera_info` |
| [aav_camera_stream_rs](./aav_camera_stream_rs/README.md) | the same `camera_stream` in Rust, no model: the default with `detector:=none` | the same |
| [aav_radar_ti](./aav_radar_ti/README.md) | TI IWR6843AOP radar (2025-26 team) | `/sensors/radar/points` |
| [aav_battery_victron](./aav_battery_victron/README.md) | Victron VE.Direct battery monitor (2024-25 team) | `/vehicle/battery` |

## Invariants

- Same topic, type, frame and QoS as the bench mocks in `ros/sim/aav_bench`.
- Sensor frames and poses come from `aav_description` (`vehicle.yaml`).
- Device paths are the udev names in `tools/udev/` (`/dev/aav/*`), never `ttyUSB*`.
- Inherited packages record every change in their `PROVENANCE.md`.

## Side Effects

- Opens serial, USB, UDP and RTSP connections to the sensors.
- The radar driver writes its config to the radar; `ublox_gps` sends RAM-only settings.

## Verification

- Run: `pixi run test` (also runs `cargo test -p yahboom-protocol` and `-p stream-core`).
