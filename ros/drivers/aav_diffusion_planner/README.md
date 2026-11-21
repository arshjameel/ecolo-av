# aav_diffusion_planner

## Overview

**A trial planner: a model draws the path, a guard checks it, the lattice is the
fallback.** `planner:=diffusion` in sim and bench; on the car only as a trial,
`car.launch.py diffusion_trial:=true`, at 0.5 m/s at most (without the switch, or faster,
the car refuses it). Off by default. The design: `docs/designs/2026-11-02_diffusion-planner.md`. Results:
`docs/trials.md`.

```text
 navigator ──── compute_path_to_pose ─► diffusion_planner ─┬─► the costmap crop ─► model worker (ONNX)
                                              │            │      16 sample paths ─► the guard (G1-G8)
                                              │            └─► compute_path_to_pose_lattice ─► our planner
                                              ▼                                                 (the lattice)
                     the cheapest sample that passed, in 150 ms; else the lattice's path ─► /plan ─► MPPI
```

The model only **proposes** a path. MPPI, the stop zone, twist_mux, ego and the
spine are unchanged below it.

## Interface

| | Name | Type |
|---|---|---|
| Serves | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose`, the name the navigator calls |
| Calls | `compute_path_to_pose_lattice` | the same, remapped to our planner's (`aav_planner`) |
| Subscribes | `/drivers/nav/costmap/global` | `nav_msgs/OccupancyGrid` |
| Publishes | `/plan` | `nav_msgs/Path`: what it returned (our planner's goes to `/drivers/nav/diffusion/lattice_plan`) |
| Publishes | `/drivers/nav/diffusion/status` | `diagnostic_msgs/DiagnosticStatus`: each plan's source (model, lattice, none), why, ms, refusals by check, the share from the model |

Launch arguments: `planner:=diffusion` (`navigation.launch.py`, `sim.launch.py`), and:

| Argument | Default | Meaning |
|---|---|---|
| `diffusion_model` | empty: `models/diffusion/planner.onnx` | the model |
| `diffusion_provider` | `CPUExecutionProvider` | the engine (`engines.py`): `TensorrtExecutionProvider` (FP16, the Orin), `CUDAExecutionProvider`, `CoreMLExecutionProvider`; each falls back to the CPU |
| `diffusion_python` | empty: Pixi's learn environment | the worker's Python (on the Orin, one with JetPack's ONNX Runtime) |
| `diffusion_impl` | `python` | the model's worker: `python` (`worker.py`) or `rust` (`aav_diffusion_rs`'s `diffusion_worker`: the same pipe, ONNX Runtime from Rust; `diffusion_provider:=auto` picks Core ML on a Mac). [ONNX](../../../docs/onnx.md) |
| `diffusion_smoothing` | `gear` | each gear's run smoothed on its own, the cusps kept; `average`: 2 Nov's |
| `diffusion_record` | empty | a folder: each plan's inputs saved, for `pixi run diffusion replay` |

On the car (`car.launch.py`): `diffusion_trial:=true` (with `planner:=diffusion` or
none), `max_speed:=` empty or at most 0.5, and `diffusion_provider:=`, `diffusion_python:=`
([car day A12](../../../docs/car-day.md#a12-the-diffusion-planners-trial)).

## Files

| File | What | ROS? |
|---|---|---|
| `aav_diffusion_planner/window.py` | the window round start and goal, the crop, the path codes | no |
| `aav_diffusion_planner/guard.py` | the checks G1-G8, the price, the choice, model or lattice | no |
| `aav_diffusion_planner/wire.py` | the pipe to the worker | no |
| `aav_diffusion_planner/engines.py` | which engine runs the model, its fallbacks, the time budget | no |
| `aav_diffusion_planner/worker.py` | the model, in the learn environment's Python (ONNX Runtime) | no |
| `aav_diffusion_planner/planner_node.py` | the node | yes |
| `training/model.py`, `training/train.py` | the network (PyTorch), training, its score, ONNX export (FP16 too) | no |
| `training/replay.py` | a sim run's recorded plans through any model, offline: the guard's score by check | no |
| `training/measure.py` | the time a plan, part by part, against the budget (`pixi run measure diffusion`; on the Orin with any Python that has ONNX Runtime) | no |
| `tools/diffusion_data.py` (repo) | the teachers' paths on random maps: our costmap and planner (`aav_planner`'s `teach` example) | no |

## Set up and run

```bash
pixi install --locked -e learn     # PyTorch, ONNX, ONNX Runtime: once
pixi run diffusion data            # the lattice's and Hybrid-A*'s paths (no simulator)
pixi run diffusion data --first 800 --maps 400 --uturn 0.6 --out models/diffusion/data_uturn
pixi run diffusion train --data models/diffusion/data models/diffusion/data_uturn --suffixes 3
pixi run diffusion export --fp16   # planner_fp16.onnx too, for a GPU (16 samples fixed: Core ML runs it whole)
pixi run measure diffusion         # the time a plan, against the budget
pixi run sim world:=test_lot planner:=diffusion
pixi run measure controllers --planners own diffusion --reversing --tours lot \
    --diffusion-record log/diffusion-plans
pixi run diffusion replay log/diffusion-plans --model <another .onnx>
```

No learn environment or no model: the node logs it, and every plan is the lattice's.

## Invariants

- The guard's checks have no ROS and their own tests (`test/`). A sample fails if any
  check fails; no sample passing, or late, means the lattice's path.
- Off by default; not in `vehicle.CAR_PLANNERS`. On the car only with
  `diffusion_trial:=true`, and then ego, Nav2 and the geofence at 0.5 m/s at most
  (`vehicle.DIFFUSION_TRIAL_SPEED`); the guard and every lock below it as always.
- Model files and data are not in git (`models/diffusion/`).

## Verification

`tools/test.sh` (the unit tests); `pixi run measure controllers --planners own
diffusion --reversing --tours lot` in Ravensim.
