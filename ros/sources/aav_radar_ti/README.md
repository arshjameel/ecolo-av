# aav_radar_ti

The driver for the TI IWR6843AOP mmWave radar. It sends the radar a `.cfg` over the
command port, reads its binary frames from the data port, and publishes a
`PointCloud2`. Inherited from the 2025-26 team's `AAV-2025-2026/Nav_Radar`: see
[PROVENANCE.md](./PROVENANCE.md) for the commit and every change since. Called
`ti_ros2_driver` until 26 Oct 2026.

On the car, `aav_sensors`' `sensors.launch.py radar:=true` starts `pcl_pub` as node
`radar`, with `cfg/base.cfg`, on `/sensors/radar/points`.

## Overview

```
cfg/<name>.cfg ─command port (115200)─> IWR6843AOP ─data port (921600)─> get_data (TI.py)
                                                               └─> pcl_pub ─> PointCloud2
```

| Mode | Picked by | `<topic>` fields | Axes |
|---|---|---|---|
| out-of-box | a cfg path with no "tracking" before its first `.` (`base.cfg`) | x, y, z, intensity, velocity | remapped to x forward, y left |
| 3D tracking | a cfg path with "tracking" before its first `.`: `3d_tracking_*.cfg`, and also `AOP_6m_tracking_default.cfg` | x, y, z, intensity, velocity (the same 5 columns) | left as TI's: x right, y forward |

The test is `'tracking' in cfg_path.split('.')[0]` (`TI.py`). A `.` in a folder name
cuts the path short, before the file name is checked.

In tracking mode, `<topic>_target` (with `publish_target_tracker`) carries the tracked
targets: x, y, z, velocity_x/y/z, acceleration_x/y/z.

## Interface

### Executables

| Executable | Node | Does |
|---|---|---|
| `pcl_pub` | `iwr6843aop` (renamed by launch) | the driver |
| `mock_radar_publisher` | `mock_radar_publisher` | a pretend radar cloud: `aav_radar_clusters`' `mock:=true` starts it |

### `pcl_pub`

| Publishes | Type | Notes |
|---|---|---|
| `<topic>` | `sensor_msgs/PointCloud2` | best effort |
| `<topic>_target` | `sensor_msgs/PointCloud2` | only with `publish_target_tracker` |

| Parameter | Default |
|---|---|
| `cfg_path` | the installed `cfg/base.cfg` |
| `command_port`, `data_port` | `/dev/aav/radar_cmd`, `/dev/aav/radar_data` |
| `topic` | `ti_mmwave_0` (`aav_sensors` sets `/sensors/radar/points`) |
| `frame_id` | `radar_link` |
| `roi` | '' or `xl_xu;yl_yu;zl_zu` (m) |
| `publish_target_tracker`, `output_RA_heatmap`, `output_RD_heatmap`, `debug_mode` | false |
| `debug_log_path` | `<cwd>/src/aav_radar_ti/debug/result.json` |

### `mock_radar_publisher`

Publishes a pretend cloud on `topic`. Parameters: `topic` (`/ti_radar_0/ti_mmwave_0`),
`frame_id` (`radar_link`), `publish_rate_hz` (10.0), `mode` (`moving`, or `static`).

### Modules

| Module | Holds |
|---|---|
| `TI.py` | `get_data`: opens the ports, loads the cfg, parses frames (`read`, `close`) |
| `pc_publisher.py` | `rospublisher`, the `pcl_pub` node: a producer thread reads, a consumer thread publishes |
| `mock_radar_publisher.py` | `MockRadarPublisher` |
| `plotheatmap.py` | `plotHM`: range-azimuth and range-Doppler heat maps (matplotlib), for `output_RA_heatmap`/`output_RD_heatmap` |
| `benchmark.py` | `Instrumentor`, `InstrumentationTimer`: timing for debug mode |

### Configs

No launch file of its own: `aav_sensors`' `sensors.launch.py radar:=true` starts it. The
2025-26 team's five demo launch files and its `obstacle_extractor` were removed on 26 Oct
(`aav_radar_clusters` clusters the points now).

- `cfg/*.cfg`: TI's radar configs. `base.cfg` gives 8.4 m at 10 Hz (indoor demo
  settings). `cfg/rules.txt`: TI's notes on the commands.

## Invariants

- Inherited code: every change goes in `PROVENANCE.md`.
- The driver never changes port permissions. A port it can't open raises an error that
  points at `tools/udev/99-aav.rules`.
- Only out-of-box mode gives REP-103 axes. Use it (`base.cfg`) for anything that feeds a
  costmap.
- The cloud is best effort: subscribers must match it.
- The radar's fields (x, y, z, intensity, velocity) are what `aav_bench`'s `mock_radar`,
  `aav_sim_models`'s `sim_radar` and `aav_carla` copy.

## Side Effects

- Opens two serial ports and writes the cfg to the radar. `close()` sends `sensorStop`.
- Two threads in `pcl_pub`. With a heat map on, a matplotlib window.
- With `debug_mode`, writes a timing JSON under the working directory.

## Verification

```sh
pixi run build && colcon test --packages-select aav_radar_ti && colcon test-result --verbose
```

The tests are the stock lint checks (copyright, flake8, pep257). `pixi run test` runs
everything. Without the radar:
`ros2 launch aav_radar_clusters radar_clusters.launch.py mock:=true minimum_points:=5`.

## Drift and known gaps

- `pcl_pub`'s and `mock_radar_publisher`'s default topics are still the old names
  (`ti_mmwave_0`, `/ti_radar_0/ti_mmwave_0`), not `/sensors/...` (`docs/naming.md`).
  Their launchers (`aav_sensors`, `aav_radar_clusters`) set `/sensors/radar/points`.
- No module docstrings listing topics, and the parsing sits in `TI.py` with no tests.
- The cfg files are indoor demos (`base.cfg`: 8.4 m range, 10 Hz).
