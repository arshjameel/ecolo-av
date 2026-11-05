# Names and code style

One naming system and one way to lay out code, for new code and inherited code alike.
Inherited code is ours to edit: bring it in line when you touch it, and note the change
in its `PROVENANCE.md`.

## Names

| Thing | Pattern | Examples |
|---|---|---|
| Our package | `aav_<what>` | `aav_ego`, `aav_spine`, `aav_teleop` |
| A device driver package | `aav_<kind>_<maker>`, so drivers of one kind sort together | `aav_imu_yahboom` |
| Folder under `ros/` | the area of the picture | `sources/`, `localization/`, `drivers/`, `command_mux/`, `ego/`, `can_bridge/`, `sim/` |
| Node and executable | what it is, in snake_case; both the same | `ego`, `spine_bridge`, `scripted_driver` |
| Logic module | a noun, no ROS imports | `ackermann.py`, `can_codec.py`, `vehicle.py` |
| Topic | absolute: `/<area>/<device or node>/<thing>` ([Topic areas](naming.md)) | `/drivers/joy/cmd_vel`, `/vehicle/odom`, `/ego/actuator_cmd` |
| Frame | `<thing>_link`, plus `map`, `odom`, `base_link` | `imu_link`, `gps_link` |
| Parameter | snake_case, SI units, the unit in a comment | `max_speed: 1.0  # m/s` |
| Config file | `config/<node>.yaml`; its top key is the node's name, or ROS ignores it | `config/ego.yaml` starts with `ego:` |
| Launch file | `launch/<what>.launch.py`; whole-system profiles live in `aav_bringup` | `ego.launch.py`, `turtle.launch.py` |
| Message, action | a CamelCase noun | `VehicleStatus`, `ScriptedDrive` |
| Rust crate | kebab-case | `spine-core` |
| Docs page | short kebab-case | `how-it-fits.md` |

## Renames

The inherited packages kept their old names until someone brought them in. Renaming
changes their launch files and executables.

| Was | Now | When |
|---|---|---|
| `maincar` (the HUD's web server, Flask) | `aav_hud` | done, 10 October; archived 25 October (`archive/2026-27-flutter-hud/aav_hud`) |
| `aav_hud_raylib` (the raylib HUD and map window) | `aav_hud` | done, 25 October, when the Flask one was archived |
| `lane_geometry`, `object_detection`, `yolo_ros2` | `aav_lane_geometry`, `aav_object_detection`, `aav_yolo` | done, 10 October, when perception came in. `aav_object_detection` became `aav_radar_clusters` (Rust), 15 October |
| `aav_yolo`, its node `yolo_detector` | `aav_detector`, `object_detector` | done, 24 October: it runs RF-DETR Nano by default, YOLO only with `detector:=yolo` |
| `yahboom_imu_cpp` | `aav_imu_yahboom` | done, 11 October: Ben's Rust rewrite replaced it |
| `aav_goto`, its node `goto_driver`, action `/goto` (`GoTo`), slot `goto` | `aav_drive_to_point`, `point_driver`, `/drive_to_point` (`DriveToPoint`), slot `point` (`/drivers/point/cmd_vel`) | done, 26 October: "goto" named a keyword, not the job. `measure atob` still reads bags with the old names |
| `aav_mux` | `aav_command_mux` | done, 26 October: there are three muxes; this one is twist_mux's, in `ros/command_mux` |
| `ti_ros2_driver` | `aav_radar_ti` | done, 26 October. The executables `pcl_pub` and `mock_radar_publisher` stay |
| `battery_monitor` | `aav_battery_victron` | done, 26 October. The executable `battery_monitor` stays |

## How a package is laid out

```text
ros/<area>/aav_<what>/
  package.xml              its dependencies
  setup.py, setup.cfg      its executables (Python packages)
  aav_<what>/
    <logic>.py             the maths: no ROS imports, unit-tested
    <node>.py              the ROS node: subscribe, call the logic, publish
  config/<node>.yaml       its parameters
  launch/<what>.launch.py  how to start it
  test/test_<logic>.py     tests for the logic
  PROVENANCE.md            inherited code only: where it came from, what changed
```

## Writing code here

1. **Purpose first.** Every file opens with a short docstring: what it's for. A node's
   docstring also lists what it subscribes, publishes, serves and calls.
2. **Comments say why**, not what. The code already says what.
3. **Keep the maths out of ROS.** Put it in a plain module with tests; the node stays
   thin.
4. **Prefer configuration to code**: stock packages first, then inherited code, then
   new code.
5. **SI units and REP-103 frames**: metres, radians, seconds; x forward, y left, z up;
   positive steering turns left.
6. **No pins in ROS.** Pins, I2C addresses and calibration live in the firmware.
7. **No secrets in git**: only where a secret lives and what kind it is.
8. **Small commits**, each passing `tools/test.sh`, each message saying why; the full
   `pixi run test` before you push ([CONTRIBUTING.md](../CONTRIBUTING.md)).
9. **Write each language its own way** (Ben, 10 Oct). Ported code is rewritten in the
   new language's style, not copied line by line.

| In Rust | Not C written in Rust |
|---|---|
| an `enum` for a state or a mode | number codes and `#define`-style constants |
| `Result<T, E>`, with an error type | -1, 0 or a flag for an error |
| iterators: `iter()`, `map`, `filter` | index loops over arrays |
| small types for units: `Metres(f32)` | bare `f32` everywhere |
| `?`, and `expect` with a reason, only where failure is a bug | `unwrap()` in library code |
| traits for a device (`embedded-hal`) | functions that take a bus handle and a pin number |
| `cargo fmt`, `cargo clippy` clean | a style of your own |

### A launch override that silently loses

ROS 2 quirk, found 15 Oct: if a launch file sets a **global** parameter (`SetParameter`,
as `sim.launch.py` does for `use_sim_time`), then a YAML file that **names the node**
(`ego:`) beats any dict passed **after** it. The override is ignored, with no warning.
`sim_speed:=3` left ego at 1.0 m/s that way, and (again, 15 Oct) `watchdog:=false` left
twist_mux's sensors lock at its 1 s timeout, so Nav2 never drove.

| Do | Don't |
|---|---|
| Load the YAML yourself and pass it as a dict (`ego.launch.py`), edit a copy of the file (`navigation.launch.py`'s `faster_copy`), or pass a second node-named YAML file after the first (`aav_command_mux`'s `no_sensor_lock.yaml`) | put a node-named YAML file first and trust a later dict to override it |
| Check the live value: `ros2 param get /<node> <name>` | trust the launch file |

## Style, and what checks it

| Language | Style | Checked by |
|---|---|---|
| Python | PEP 8 with 99-character lines, single quotes, docstrings that start with a summary line | `pixi run lint` (ROS's `ament_flake8` and `ament_pep257`) |
| C++ | the ROS 2 C++ style | `ament_uncrustify`, `ament_cpplint` (in `colcon test`) |
| Rust | rustfmt and clippy | `cargo fmt`, `cargo clippy` |
| Markdown docs | short pages, a purpose line, sections, tables | `pixi run docs`, and a look at the result |
