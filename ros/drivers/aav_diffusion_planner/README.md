# aav_diffusion_planner

## Overview

**A trial planner: a model draws the path, a guard checks it, the lattice is the
fallback.** `planner:=diffusion` (sim and bench only; the car refuses it). Off by
default. The design: `docs/designs/2026-11-02_diffusion-planner.md`. Results:
`docs/trials.md`.

```text
 bt_navigator ─ compute_path_to_pose ─► diffusion_planner ─┬─► the costmap crop ─► model worker (ONNX)
                                              │            │      16 sample paths ─► the guard (G1-G8)
                                              │            └─► compute_path_to_pose_lattice ─► planner_server
                                              ▼                                                 (the lattice)
                     the cheapest sample that passed, in 150 ms; else the lattice's path ─► /plan ─► MPPI
```

The model only **proposes** a path. MPPI, collision_monitor, twist_mux, ego and the
spine are unchanged below it.

## Interface

| | Name | Type |
|---|---|---|
| Serves | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` (planner_server's own, renamed `compute_path_to_pose_lattice`) |
| Calls | `compute_path_to_pose_lattice` | the same, planner_server's lattice |
| Subscribes | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` |
| Publishes | `/plan` | `nav_msgs/Path`: what it returned (planner_server's goes to `/drivers/nav/diffusion/lattice_plan`) |
| Publishes | `/drivers/nav/diffusion/status` | `diagnostic_msgs/DiagnosticStatus`: each plan's source (model, lattice, none), why, ms, refusals by check, the share from the model |

Launch arguments: `planner:=diffusion` (`navigation.launch.py`, `sim.launch.py`); in
`navigation.launch.py` also
`diffusion_model:=` (empty: `models/diffusion/planner.onnx`), `diffusion_provider:=`
(ONNX Runtime's: `CPUExecutionProvider`, `CoreMLExecutionProvider`, `CUDAExecutionProvider`).

## Files

| File | What | ROS? |
|---|---|---|
| `aav_diffusion_planner/window.py` | the window round start and goal, the crop, the path codes | no |
| `aav_diffusion_planner/guard.py` | the checks G1-G8, the price, the choice, model or lattice | no |
| `aav_diffusion_planner/wire.py` | the pipe to the worker | no |
| `aav_diffusion_planner/worker.py` | the model, in the learn environment's Python (ONNX Runtime) | no |
| `aav_diffusion_planner/planner_node.py` | the node | yes |
| `training/model.py`, `training/train.py` | the network (PyTorch), training, its score, ONNX export | no |
| `tools/diffusion_data.py` (repo) | the teachers' paths on random maps | yes |

## Set up and run

```bash
pixi install --locked -e learn     # PyTorch, ONNX, ONNX Runtime: once
pixi run diffusion data            # the lattice's and Hybrid-A*'s paths (no Gazebo)
pixi run diffusion train           # the model, scored by the guard: models/diffusion/
pixi run sim world:=test_lot planner:=diffusion
pixi run measure controllers --planners lattice diffusion --reversing --tours lot
```

No learn environment or no model: the node logs it, and every plan is the lattice's.

## Invariants

- The guard's checks have no ROS and their own tests (`test/`). A sample fails if any
  check fails; no sample passing, or late, means the lattice's path.
- Off by default; not in `vehicle.CAR_PLANNERS`.
- Model files and data are not in git (`models/diffusion/`).

## Verification

`tools/test.sh` (the unit tests); `pixi run measure controllers --planners lattice
diffusion --reversing --tours lot` in Gazebo.
