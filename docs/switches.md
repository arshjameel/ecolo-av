# Switches: what you can swap in the simulated car

Every part we're trying out is a **switch** you give when you start. You don't need to
remember them: **`pixi run pick`** asks, in plain words, and runs it.

```
pixi run pick          # a menu: what to start, the route, who steers, obstacles, where-am-I, speed loop
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

| Switch | In plain words | Gazebo, the lot tour |
|---|---|---|
| `planner:=lattice` (default) | State Lattice: strings our car's own moves together; fast | 30 Oct, reversing: every leg, **184 s** a tour. Forward only (29 Oct): 2 of 4, lost the U-turn |
| `planner:=hybrid` (the car's default) | Hybrid-A*: only routes a car can drive | 30 Oct, reversing: every leg, 213 s a tour |
| `planner:=lattice lattice_headings:=32` | the lattice with 32 headings, not 16 | 30 Oct, reversing: every leg, 197 s; slower plans. No better |
| `planner:=smac2d` | A* on the grid: **ignores the turning circle** | 29 Oct, forward only: 2 of 4 |
| `planner:=navfn` | NavFn, the classic: ignores the turning circle | not tried |
| `planner:=theta_star` | Theta*: straight lines: ignores the turning circle | not tried |

The lattice is the default since 1 Nov (Ben). The real car takes only `hybrid` or
`lattice`, and keeps `hybrid`: forward only (until H5), the lattice lost the U-turn. More: [Controllers, Planners](controllers.md), [the runs](trials.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).

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

These run **beside** the EKFs and are only scored for now. They never steer, except
`ndt_ekf:=true`: then the global EKF, which Nav2 plans by, takes NDT's pose too; and
`kiss_ekf:=true`: then the local EKF, which Nav2 steers by, takes KISS-ICP's speed too.
In `pixi run pick`, `debugdemo` offers NDT + AMCL first (to see every finder); the
others offer the EKFs only first.

| Switch | In plain words | Off from the truth |
|---|---|---|
| (nothing) | the EKFs: wheels, IMU and GPS | ~17 cm |
| `ndt:=true ndt_map:=maps/ndt/test_lot.pcd` | the lidar slid onto a saved map | **1-3 cm** |
| `amcl:=true` | many guesses; keep the ones that match the walls | 6-11 cm |
| `ndt_ekf:=true ndt_map:=maps/ndt/test_lot.pcd` | NDT, and the global EKF **uses** it beside the GPS (a trial) | the EKF **1.6 cm** ([trials](trials.md#ndt-into-the-global-ekf-30-oct)) |
| `kiss_icp:=true` | how far the car moved, from the lidar alone (no map) | 0.02-0.04 m per 100 m driven |
| `kiss_ekf:=true` | KISS-ICP, and the local EKF **uses** its speed beside the wheels (a trial) | no clear win yet ([trials](trials.md#kiss-icp-into-the-local-ekf-1-nov)) |

NDT's map isn't in git: make it once with `pixi run ndt-map drive` (about 6 min).
KISS-ICP needs its own environment once: `pixi install --locked -e kiss`.

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
| `voxels:=true` | Nav2 sees the lidar as 0.25 m cubes, not a flat scan (a trial: [Trials](trials.md#voxels-30-oct-the-lidar-as-cubes)) |

**Tried, 29 Oct 23:05:** the menu's pick (debugdemo, `mpc`, NDT, LQI) in Gazebo on Ben's
Mac: the track node ran MPC, NDT tracked (2 ms a match), AMCL stayed off, the speed loop
said LQI, and a Nav2 goal 15 m ahead ended SUCCEEDED.

Every switch with its own words: `ros2 launch aav_bringup sim.launch.py --show-args`.
