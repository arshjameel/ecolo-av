# aav_voxels_rs

## Overview

`voxels` in Rust (rclrs): a **drop-in** for `aav_voxels`' Python node. The same node
name, topics, QoS, frames, fields and status, and the same cubes. Ben, 15 Nov: the
heavy Python goes to Rust "to perform in the car" (TODO 185). The Python stays, as the
default, until this one has run on the car.

```
/sensors/lidar/points ──► voxels (Rust) ──► /perception/voxels/cubes
                             ├────────────► /perception/voxels/ground
                             └────────────► /perception/voxels/status
```

Pick it with `voxels_impl:=rust` (`navigation.launch.py`, `sim.launch.py`,
`car.launch.py`), beside `voxels:=true` or `screen_cubes:=true`.

How fast (Ben's MacBook, M4 Max, 15 Nov):

| Measured | Python (numpy 2.5) | Rust |
|---|---|---|
| one whole lidar turn (28,800 points): to the car's frame + the cubes | **4.29 ms** | **0.53 ms** (8 times faster) |
| the same, + reading and writing the cloud's bytes | not timed alone | + 0.17 ms |
| our simulator's road world, 10 Hz: the status's `ms` (the whole callback) | 6.8-7.5 ms | 1.4 ms |
| the same run: the node's CPU (CPU seconds a second) | 25 % of a core | 6 % of a core |

Most of the Rust node's 6 % is receiving the 0.9 MB cloud, not the cubes.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/voxels` | the road fit, the cubes, the lidar's mount, `PointCloud2`'s bytes | no: `cargo test` on any laptop |
| `crates/voxels-node` | the node, binary `voxels` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt`: `cargo build --release --locked`, then the
binary is installed as `lib/aav_voxels_rs/voxels`.

### Node `voxels`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `sensor_msgs/PointCloud2`, reliable, keep last 1 |
| out | `/perception/voxels/cubes` | `PointCloud2`, `base_link`, x y z float32, reliable, depth 1: each cube's point nearest `base_link` |
| out | `/perception/voxels/ground` | `PointCloud2`, `base_link`, the same: each flat square of road seen |
| out | `/perception/voxels/status` | `diagnostic_msgs/DiagnosticStatus`: `ms`, `cubes`, `ground_cubes`, `points`, `fitted`, `roll_deg`, `pitch_deg`, `clouds_dropped` |

| Parameter | From | Means |
|---|---|---|
| `size`, `min_height`, `max_height`, `range_max` (m) | `nav2.yaml`'s `voxels:` block, as the Python | the cube's edge; the band above the road; how far |
| `lidar_xyz` (m), `lidar_rpy` (rad) | `vehicle.yaml`'s `lidar_link`, by `navigation.launch.py` | where the lidar sits |
| `body` (m: x from, x to, half width) | `aav_lidar` `scan.body_box`, by `navigation.launch.py` | the car's own outline: its points never count |

The last three have no default: without them the node stops and says which is missing.
The Python reads `vehicle.yaml` itself; the Rust gets the same numbers from the launch file.

### `voxels` crate

| Item | Does |
|---|---|
| `Point`, `Body`, `CubeShape` (`checked()`, `ShapeError`) | a point; the car's outline; which points count |
| `Mount::new(xyz, rpy)`, `to_base` | the lidar's frame into `base_link` (`scan.rotation`) |
| `fit_ground` → `Road` (`Fitted { plane, candidates }` or `Flat`) | the road plane, with `roll()`, `pitch()`, `height()` |
| `cubes`, `cubes_on(road)` → `Cubes` | obstacles, ground squares, the road, the points that counted |
| `lowest_per_cell`, `nearest_per_cube`, `cube_centres`, `square_centres`, `percentile`, `fit_plane` | the steps, each tested |
| `cloud::read_xyz`, `cloud::write_xyz32` | `PointCloud2`'s bytes in and out |

## Invariants

- The same answers as `aav_voxels`' `cubes.py`: `tests/python_vectors.rs` checks 9
  recorded clouds (below). Change one, change both, and record the vectors again.
- `voxels` has no ROS. Logic changes go there, with tests.
- The same node name and topics as the Python, so nothing downstream can tell which runs.
  Only one of the two runs at a time (`voxels_impl:=`).
- Rust style: enums and `Result`, no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## How it matches the Python

`test/capture_python_vectors.py` plays clouds through `cubes.py` (no ROS) and writes
`crates/voxels/tests/python_vectors.txt`: the points (float32, as a lidar sends them)
and the Python's answers.

| Cloud | Fitted | Cubes |
|---|---|---|
| the lot, a post ahead | yes | 5 |
| leaning 3° and 2°, a post and a wall, 1 cm noise | yes | 97 |
| walls all round, leaning 2° | yes | 950 |
| parked cars and a box, 2 cm noise | yes | 673 |
| a 10° ramp (too steep: flat) | no | 760 |
| a crowned road | yes | 0 |
| odd points: NaN, inf, the car's own, far, overhead | no | 3 |
| empty | no | 0 |
| random points on a tilted mount | no | 1,392 |

| Check | Result |
|---|---|
| fitted, candidates, points that count | the same, all 9 |
| the road's plane | within 1.2e-15 (Rust: normal equations; numpy: SVD) |
| obstacle cubes, on the fitted road and on a flat one | the same points, all 9 |
| ground squares | the same, all 9 |
| one whole turn (`--bench`, 28,800 points) | the same |

`test/test_voxels_node.py` (in `colcon test`) runs the node: a whole lidar turn in, the
Python's cubes, ground and status out, and a clean exit on Ctrl-C.

## Side Effects

- Publishes the three topics above. Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).
- colcon builds into its own `target/` folder in the build directory.

## Verification

```sh
cd ros/perception/aav_voxels_rs && cargo test --locked -p voxels      # no ROS
pixi run build && colcon test --packages-select aav_voxels_rs && colcon test-result --verbose
```

Timing, both on one cloud:

```sh
python3 ros/perception/aav_voxels_rs/test/capture_python_vectors.py --bench /tmp/cloud.txt
cd ros/perception/aav_voxels_rs && cargo run --release -p voxels --example timing -- /tmp/cloud.txt
VOXELS_VECTORS=/tmp/cloud.txt cargo test -p voxels     # and the same answers on it
```

In the simulator: `pixi run sim world:=road voxels:=true voxels_impl:=rust`. Run on
15 Nov (headless, our simulator): the node started from the launch file's parameters,
published cubes and ground at 10 Hz, the road fitted, no clouds dropped, and it exited
cleanly on Ctrl-C. Not yet run on the car.
