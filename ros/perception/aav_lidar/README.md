# aav_lidar

## Overview

The flat lidar scan's maths, in Python with no ROS: `scan.py` and `ground.py`. The node
that runs is Rust: [`aav_lidar_rs`](../aav_lidar_rs/README.md)'s `lidar_scan` (its topics
and parameters are there).

The Python node here (`lidar_scan.py`) was removed on 22 Nov 2026 (Ben: "lets start
removing the python fall back after being tested enough times"; git tag
`save/python-fallbacks-1`). The package stays for what still imports it:

| Who | Imports |
|---|---|
| `aav_navigation`'s `navigation.launch.py` | `scan.body_box` (the car's own outline, for the Rust nodes) |
| `aav_voxels`, `aav_voxels_rs`' tests | `scan.to_base`, `scan.body_box`, `scan.rotation` |
| `aav_lidar_rs`' `gen_vectors.py` and node test | `scan.flatten`, `ScanShape`, `to_base` (the Rust's yardstick) |
| `tools/sim_check.py`, `tools/tests/test_lidar_roll.py` | `scan`, `ground` |

## Interface

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `scan.py` | yes | `ScanShape`, `rotation`, `to_base`, `flatten`; `body_box` (the car's own outline plus its margins, `SELF_MARGIN`, `FRONT_MARGIN`: `aav_voxels` uses it too) |
| `ground.py` | yes | body roll against the lowest beams: `ground_hit`, `worst_ground_height`, `max_safe_roll`, `VLP16_ELEVATIONS`. Sums only; the node doesn't use it (TODO 52e) |

There is no node, launch or config folder: `aav_navigation` starts `aav_lidar_rs`'s node.

## Invariants

- `scan.py` and `ground.py` have no ROS imports, and their own tests.
- Output is in `base_link` (REP-105): x forward, y left, on the road under the rear axle.
- Each range is the nearest point between `min_height` and `max_height`; `inf` where none.
- Points inside the car's own outline never count.
- The body is taken as level. In a hard turn the road on the outside can rise above
  `min_height` (see `ground.py`). The simulator has no suspension, so it can't show this.

## Side Effects

- None: sums only.

## Verification

```sh
cd ros/perception/aav_lidar && python3 -m pytest -q        # the logic, no ROS
pixi run build && colcon test --packages-select aav_lidar && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. In Ravensim:
`pixi run sim-check --lidar-obstacle`.
