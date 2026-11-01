# aav_bench

Bench mode's pretend sensors. Each one reads where the model car really is
(`/vehicle/bench/truth`, from `spine_sim` in `aav_spine_sim`) and publishes what the
real driver will: the same topic, message type and frame. So nothing downstream
changes between the bench and the car.

`aav_bringup`'s `bench.launch.py` starts them, with `config/bench.yaml`.

## Overview

| Pretend sensor | Stands in for (on the car) |
|---|---|
| `mock_imu` | `aav_imu_yahboom` |
| `mock_gps` | `ublox_gps` (`aav_sensors`) |
| `mock_battery` | `aav_battery_victron` |
| `mock_camera` | `usb_cam` (`aav_sensors`) or `aav_camera_stream` |
| `mock_radar` | `aav_radar_ti` |
| `mock_lidar` | the roof's VLP-16 itself: it sends UDP packets to the stock velodyne driver |

The bench world (`config/bench.yaml`) is in the map frame: x east, y north, metres from
the GPS datum. The car starts at (0, 0), facing east. It has one lane, a stop sign 25 m
ahead on the right, and (with `obstacle:=true`) a 1 m box in the lane 15 m ahead.

## Interface

### Nodes (all in `aav_bench/sensors.py`)

| Node | Subscribes | Publishes | Frame |
|---|---|---|---|
| `mock_imu` | `/vehicle/bench/truth` (Odometry) | `/sensors/imu/data_raw` (Imu), `/sensors/imu/mag` (MagneticField) | `imu_link` |
| `mock_gps` | `/vehicle/bench/truth` | `/sensors/gps/fix` (NavSatFix), 5 Hz | `gps_link` |
| `mock_battery` | none | `/vehicle/battery` (BatteryState), 1 Hz | none |
| `mock_camera` | `/vehicle/bench/truth` | `/sensors/camera/front_left/image_raw` (Image), `.../camera_info`, `.../image_raw/compressed` (JPEG, only while read), 10 Hz | `camera_left_link` |
| `mock_radar` | `/vehicle/bench/truth` | `/sensors/radar/points` (PointCloud2, best effort), 10 Hz | `radar_link` |
| `mock_lidar` | `/vehicle/bench/truth` | UDP VLP-16 packets to `127.0.0.1:2369`, 754 a second | `lidar_link` |

Every node also publishes, once and latched, a `DataSource` on `/system/source`
saying its topic is `BENCH`. The HUD shows a BENCH banner.

### Parameters

Most defaults are in `config/bench.yaml`. Three kinds are not:

- `datum` comes from `aav_localization`'s `navsat.yaml`, passed in by `bench.launch.py`;
- `seed`, `host`, `port` and `frame_id` have their defaults only in `sensors.py`;
- `frame_id` defaults to the frame in the nodes table above.

| Node | Parameters |
|---|---|
| `mock_imu` | `frame_id`, `gyro_stddev` (rad/s), `accel_stddev` (m/s²), `gyro_bias` (rad/s), `seed` |
| `mock_gps` | `frame_id`, `datum` (lat, lon, alt), `drift_stddev` (m), `drift_time` (s), `jitter_stddev` (m), `rate_hz`, `seed`: the error is `models.GpsError`, a slow drift plus jitter |
| `mock_battery` | `full_voltage`, `empty_voltage` (V), `hours` |
| `mock_camera` | `frame_id`, `stop_signs` (x, y pairs), `sign_height`, `sign_size` (m), `width`, `height`, `focal_px`, `rate_hz`; the road: `road_y`, `lane_width`, `line_width`, `shoulder`, `bend_x`, `bend_radius` (m) |
| `mock_radar` | `frame_id`, `obstacle`, `boxes` (x1, y1, x2, y2 each), `box_height`, `max_range` (m), `fov_deg`, `ray_step_deg`, `keep`, `range_noise`, `height_noise` (m), `rate_hz`, `seed` |
| `mock_lidar` | `host`, `port`, `obstacle`, `boxes`, `box_height`, `max_range` (m) |

`obstacle` and `boxes` are read on every frame, so a test can change them live:
`ros2 param set /mock_radar obstacle true`.

### Modules

| Module | Holds |
|---|---|
| `models.py` | what each sensor would measure: `Truth`, `imu_reading`, `geo_from_map`, `GpsError`, `magnetic_field`, `battery_voltage`, `Camera`, `sign_in_view`, `Road`, `Radar`, `radar_returns`, `Lidar`, `lidar_ranges` |
| `pictures.py` | the camera's picture: `road`, `scene`, `draw_stop_sign` (NumPy and OpenCV) |
| `vlp16.py` | the VLP-16's 1206-byte UDP packet: `packet`, `decode`, `Sender` (also used by `tools/fake_vlp16.py`) |
| `sensors.py` | the six ROS nodes and their entry points |

## Invariants

- `models.py`, `pictures.py` and `vlp16.py` have no ROS imports. Their tests run anywhere.
- Each pretend sensor uses the real driver's topic, message type, frame and QoS. The
  radar is best effort, as the TI driver is.
- Sensor poses come from `aav_description`'s `vehicle.yaml`, never from here.
- Nothing is published before the first `/vehicle/bench/truth` (battery excepted).
- `boxes` must hold four numbers per box; other lengths are refused.
- Noise is seeded (`seed`), so runs repeat.
- SI units: metres, radians (degrees only in `fov_deg` and `ray_step_deg`), seconds.

## Side Effects

- Publishes the topics above, and the latched `/system/source`.
- `mock_lidar` opens a UDP socket and sends to `127.0.0.1:2369` by default.
- Reads `vehicle.yaml` at start-up. No files written.

## Verification

```sh
pixi run build && colcon test --packages-select aav_bench && colcon test-result --verbose
```

The tests cover `models.py`, `pictures.py` and `vlp16.py`. `pixi run test` runs
everything. To see it running: `pixi run bench`. `pixi run bench-check` starts bench
mode and tests the whole car graph.
