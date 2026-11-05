# aav_run_info

## Overview

Says which algorithms this run uses, in words, for the screens. The launch knows its
choices (`planner:=`, `tracker:=`, `voxels:=`, `ndt:=`...); nothing else published them.
The cockpit's **Algorithms** table shows them ([Foxglove](../../../docs/foxglove.md#cockpit)).

```
sim.launch.py / car.launch.py ──(parameters)──▶ run_info ──▶ /system/algorithms ──▶ cockpit
```

## Interface

### Node `run_info`

| In / out | Topic | Type |
|---|---|---|
| out | `/system/algorithms` | `diagnostic_msgs/DiagnosticStatus`, latched, again every 2 s |

The message: `name` algorithms, `level` OK, `message` one short line
(`lattice, mppi, voxels, NDT scored`), `hardware_id` the run (`sim world:=road`), and one
`values` row each:

| Row | Example |
|---|---|
| run | `sim world:=test_lot` |
| planner | `lattice (SmacPlannerLattice)` |
| path follower | `MPPI (Nav2 controller_server)`, or `lqr (aav_track, trial)` |
| obstacles | `lidar scan + radar`, or `voxels: the lidar as 0.25 m cubes (trial)` |
| local EKF, global EKF | their inputs: `wheels, IMU`, `+ KISS-ICP speed (trial)` |
| NDT, KISS-ICP, AMCL | `off`, `on, scored only`, or `on, into the global EKF` |

| Parameter | Default | Means |
|---|---|---|
| `profile` | `sim` | the run, in words |
| `nav` | true | Nav2 runs |
| `planner`, `controller`, `tracker` | lattice, mppi, none | as `vehicle.py` names them |
| `lidar`, `radar`, `voxels` | false | what Nav2 sees obstacles with |
| `amcl`, `ndt`, `ndt_ekf`, `kiss_icp`, `kiss_ekf` | false | the trial localizers |
| `period` | 2.0 s | how often it says it again |

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `algorithms.py` | yes | `Choices` (refuses unknown names), `rows()`, `summary()`, `use()` |
| `run_info.py` | no | the node |

## Invariants

- It says what the launch **asked for**, not what is alive. A live tracker swap in
  Gazebo shows on `/drivers/nav/track/status`.
- Every planner, controller and tracker `vehicle.py` knows has words here (a test checks).
- `sim.launch.py` always starts it; `car.launch.py` where Nav2 runs.

## Side Effects

- Publishes `/system/algorithms`. Nothing else.

## Verification

```sh
cd ros/bringup/aav_run_info && python3 -m pytest -q   # the words, no ROS (tools/test.sh)
ros2 topic echo --once /system/algorithms             # against a running sim
```
