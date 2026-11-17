# aav_ghost

## Overview

Ghost obstacles: a pretend person or post that only Nav2 sees. It tests stopping on the
real car with nothing real to hit. The full guide is `docs/ghost-obstacles.md`.

- `ghost_obstacle` (node) publishes what a perfect 2D lidar at `base_link` would see of
  the ghost. The local costmap's `ghost_layer` and the stop zone's ghost source read it.
- `ghost` (command) places or removes it: `pixi run ghost ahead 10`.

`navigation.launch.py ghost:=true` starts the node; `sim.launch.py` and `car.launch.py`
pass `ghost:=` on. Off by default.

## Interface

### Node `ghost_obstacle`

| In / out | Topic | Type |
|---|---|---|
| out | `/safety/ghost/scan` | `sensor_msgs/LaserScan`, `base_link`, at `rate_hz`: 720 ranges, the ghost's near side, `inf` elsewhere; all `inf` with no ghost |
| out | `/safety/ghost/marker` | `visualization_msgs/Marker`: a purple cylinder in `frame`; DELETE when it goes |
| in | TF `frame → base_link` | where the car is |

| Parameter | Default | Means |
|---|---|---|
| `enabled` | false | no ghost until someone asks |
| `ahead` | 10.0 m | ≥ 0: near edge this far past the front bumper, where the car is when placed. < 0: at `x`, `y` |
| `x`, `y` | 0, 0 m | in `frame`, when `ahead` is negative |
| `frame` | `map` | the ghost stands still in this frame |
| `width`, `height` | 0.5 m, 1.7 m | a person; a post is 0.3 x 1.2 |
| `rate_hz` | 10 Hz | keep well inside the stop zone's 1 s `source_timeout` |

Setting `enabled`, `ahead`, `x`, `y` or `frame` places the ghost again. A new size
keeps it where it stands. Defaults are in `config/ghost_obstacle.yaml`.

### Command `ghost` (`pixi run ghost ...`)

| Words | Does |
|---|---|
| `ahead <m>` | a ghost `<m>` past the bumper |
| `at <x> <y>` | a ghost at map x, y |
| `size <width> <height>` | change its size (m) |
| `on`, `off` | the last ghost again; take it away |
| `--local ...` | Gazebo or the bench on this computer (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) |

It calls `/ghost_obstacle/set_parameters_atomically`, so every value of one command lands at once.

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `ghost.py` | yes | `Pose`, `Ghost`, `ScanShape`, `ahead_of`, `to_base`, `ranges`, `command`, `USAGE` |
| `ghost_obstacle.py` | no | the node |
| `ghost_cli.py` | no | the command |

## Invariants

- `ghost.py` has no ROS imports, and its own tests.
- The ghost has its own costmap layer and its own stop zone source, so it only
  ever adds an obstacle. It never clears or hides a real one.
- Silent for 1 s, the ghost source stops Nav2, like a silent radar.
- The scan is always published, even with no ghost (all `inf`), so the layer clears itself.
- With `base_link` inside the ghost, every range reads `range_min`.
- The bumper's distance comes from `vehicle.yaml` (`body.length - body.rear_overhang`).

## Side Effects

- Publishes the two topics above.
- `ghost` changes `ghost_obstacle`'s parameters over ROS. It waits up to 10 s for the node.

## Verification

```sh
cd ros/safety/aav_ghost && python3 -m pytest -q          # the maths and the command words
pixi run build && colcon test --packages-select aav_ghost && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. In Gazebo:
`pixi run sim-check --ghost`.
