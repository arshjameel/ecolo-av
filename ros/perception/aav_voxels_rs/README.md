# aav_voxels_rs

## Overview

`voxels` in Rust (rclrs): a **drop-in** for `aav_voxels`' Python node. The same node
name, topics, QoS, frames, fields and status, and the same cubes. Ben, 15 Nov: the
heavy Python goes to Rust "to perform in the car" (TODO 185). The Python stays, as the
default, until this one has run on the car.

```
/sensors/lidar/points ──► voxels (Rust) ──► /perception/voxels/cubes
                             ├────────────► /perception/voxels/ground
                             ├────────────► /perception/voxels/steps   (steps:=true: kerbs)
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
| out | `/perception/voxels/steps` | only with `steps:=true`: `PointCloud2`, `base_link`, the same QoS: the high side of every kerb (step in the ground), one point per 0.1 m square |
| out | `/perception/voxels/status` | `diagnostic_msgs/DiagnosticStatus`: `ms`, `cubes`, `ground_cubes`, `points`, `fitted`, `roll_deg`, `pitch_deg`, `clouds_dropped`; and `steps` with `steps:=true` |

| Parameter | From | Means |
|---|---|---|
| `size`, `min_height`, `max_height`, `range_max` (m) | `nav2.yaml`'s `voxels:` block, as the Python | the cube's edge; the band above the road; how far |
| `lidar_xyz` (m), `lidar_rpy` (rad) | `vehicle.yaml`'s `lidar_link`, by `navigation.launch.py` | where the lidar sits |
| `body` (m: x from, x to, half width) | `aav_lidar` `scan.body_box`, by `navigation.launch.py` | the car's own outline: its points never count |
| `steps` (false), `step_min` (0.10 m), `step_radius` (0.3 m), `step_range` (12 m) | `navigation.launch.py kerbs:=true` | the kerbs' step finder (TODO 119, [kerbs](../../../docs/kerbs.md)) |
| `ground` (`plane`), `patch_min_height` (0.15 m) | `navigation.launch.py ground:=patchwork` | `patchwork`: the ground a plane a patch, ring by ring (TODO 215, a challenger): the cubes from 0.15 m over each patch's own ground |

The three mount ones have no default: without them the node stops and says which is missing.
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
| `patchwork::cubes(points, CubeShape, PatchSettings)`, `patchwork::judge`, `PcaPlane`, `Ground` | the ground ring by ring (Patchwork++'s idea, TODO 215): concentric zones, rings and sectors; a PCA plane a patch from its lowest points; uprightness, elevation and flatness; a drop-in for `cubes`. Tests: a 0.3 m and a 0.25 m kerb are cubes (the champion misses the 0.25), a road bending up a hill is none (the champion's one plane makes cubes of it), a wall and a bumpy raised patch are not ground |
| `steps(points, StepShape)` (`StepShape::checked()`, `StepError`) | the kerbs: points 0.10-0.5 m above a point within 0.3 m of them, flat; 2 or more to a 0.1 m square |
| `kerbs_only(steps, points, road, StepShape, ceiling)` | the steps less the feet and faces of anything taller than a kerb (a point 0.5 m-`ceiling` over the road within 0.3 m): a person, a post, a wall are cubes, and may walk away, so the memory never keeps them (TODO 189: a walker's foot, kept under every beam, held the car for good) |
| `StepMemory` (`with_blind(Blind)`), `Blind::under(lidar_xyz, LOWEST_BEAM)` | the kerbs kept in `odom` `step_keep` s (10); one under every beam (within the lidar's height / tan 15° of it) doesn't age while it stays there, so a car standing before a kerb never forgets it (TODO 219) |

### The step finder (`steps.rs`, TODO 119)

The cubes start 0.3 m above the road, so a kerb is never a cube. The step finder looks
for a sudden rise instead:

```text
   ring k+1 ─────────╮
   ring k   ──────╮  │       ● within 0.3 m of ○, flat,
                  │  ●       and more than 0.10 m higher: a step
   road ──────────○──┘
```

One ring alone can't show a step: all its points lie on one cone, so a kerb looks like
a ramp as steep as the beam. Two rings can, when one lands on the road just before the
kerb and the next on its face or top.

| Cloud (cast as the VLP-16 casts it, 3 cm noise) | Steps |
|---|---|
| a 0.3 m kerb across the road | found up to 8.6 m past the bumper |
| a 0.15 m kerb across the road | found up to 4.7 m past the bumper |
| a 0.15 m kerb beside the road | found where the two lowest rings cross it (about 5 m ahead and behind) |
| a 0.10 m kerb | never (the rise must be **more** than 0.10 m) |
| flat road; slopes and ramps of 3°, 6°, 10°; a 2.5 % camber; a 7.5 cm speed bump; rough grass; a sign overhead | none |
| the nine recorded clouds: the ramp, the crowned road | none |

`cargo test -p voxels --release -- --ignored --nocapture steps_range` prints the table of
distances. On the recorded clouds it takes 0.14-0.66 ms, 2 to 7 times the cubes' time (`examples/timing.rs`).

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
Python's cubes, ground and status out, and a clean exit on Ctrl-C; and with `steps:=true`, a
0.15 m kerb out on `/perception/voxels/steps`.

## Side Effects

- Publishes the three topics above (four with `steps:=true`). Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).
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
