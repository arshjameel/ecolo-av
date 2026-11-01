# Switches: what you can swap in the simulated car

Every part we're trying out is a **switch** you give when you start. You don't need to
remember them: **`pixi run pick`** asks, in plain words, and runs it.

```
pixi run pick          # a menu: what to start, the route, who steers, where-am-I, speed loop
pixi run pick last     # the same again tomorrow
pixi run pick list     # every switch, then stop
```

It prints the whole command too, for example:

```
pixi run debugdemo tracker:=mpc reversing:=false ndt:=true ndt_map:=maps/ndt/test_lot.pcd amcl:=false speed_loop:=lqi
```

`sim`, `demo` and `debugdemo` all take switches as `name:=value`. If a switch is given
twice, the last one wins.

## Who draws the route

| Switch | In plain words | Gazebo, 29 Oct |
|---|---|---|
| `planner:=hybrid` (default) | Hybrid-A*: only routes a car can drive | **4 of 4 legs** |
| `planner:=lattice` | State Lattice: strings our car's own moves together; fast | 2 of 4 (lost the U-turn) |
| `planner:=smac2d` | A* on the grid: **ignores the turning circle** | 2 of 4 |
| `planner:=navfn` | NavFn, the classic: ignores the turning circle | not tried |
| `planner:=theta_star` | Theta*: straight lines: ignores the turning circle | not tried |

The real car takes only `hybrid` or `lattice`. More: [Controllers, Planners](controllers.md).

## Who steers along the path

| Switch | In plain words | Gazebo, 29 Oct |
|---|---|---|
| `controller:=mppi` (default) | Nav2's: try 1,000 wiggles, keep the good ones | 8-10 of 12 legs |
| `controller:=rpp` | Nav2's: steer at a point ahead, slow in bends | 10 of 12 |
| `tracker:=pure_pursuit` | ours: steer at a point ahead | **12 of 12** |
| `tracker:=stanley` | ours: fix the distance off the path and the heading | **12 of 12** |
| `tracker:=lqr` | ours: a formula for the best steering each moment | 10 of 12 |
| `tracker:=mpc` | ours: try the next 1.5 s of steering, keep the best | **12 of 12** |
| `tracker:=mpc_full` | ours: MPC that plans the speed too | 10 of 12 |

Our trackers drive forward only: give them `reversing:=false` (the menu does).
In `debugdemo`, keys 1-4 swap the tracker live. More: [Controllers](controllers.md),
[Trials](trials.md).

## Where am I

These run **beside** the EKFs and are only scored for now. They never steer.

| Switch | In plain words | Off from the truth |
|---|---|---|
| (nothing) | the EKFs: wheels, IMU and GPS | ~17 cm |
| `ndt:=true ndt_map:=maps/ndt/test_lot.pcd` | the lidar slid onto a saved map | **1-3 cm** |
| `amcl:=true` | many guesses; keep the ones that match the walls | 6-11 cm |

NDT's map isn't in git: make it once with `pixi run ndt-map drive` (about 6 min).

## Speed

| Switch | In plain words |
|---|---|
| `speed_loop:=default` | today's PI loop |
| `speed_loop:=lqi` | no overshoot, settles in 4-5 s |
| `speed_loop:=smooth` | the PI with a smooth stop |

## The world and the rest

| Switch | What |
|---|---|
| `world:=test_lot` / `road` / `empty` | where the car is (`sim` only; the demos use the test lot) |
| `reversing:=false` | Nav2 drives forward only |
| `sim_speed:=3` | a top speed above the car's 1 m/s, in Gazebo only |
| `geofence:=false` | no geofence (it's on by default) |
| `lidar:=false`, `radar:=false` | sensors off |

**Tried, 29 Oct 23:05:** the menu's pick (debugdemo, `mpc`, NDT, LQI) in Gazebo on Ben's
Mac: the track node ran MPC, NDT tracked (2 ms a match), AMCL stayed off, the speed loop
said LQI, and a Nav2 goal 15 m ahead ended SUCCEEDED.

Every switch with its own words: `ros2 launch aav_bringup sim.launch.py --show-args`.
