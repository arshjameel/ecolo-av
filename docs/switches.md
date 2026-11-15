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
pixi run demo --debug tracker:=mpc reversing:=false ndt:=true ndt_map:=maps/ndt/test_lot.pcd amcl:=false speed_loop:=lqi
```

`sim`, `demo` and `demo --debug` all take switches as `name:=value`. If a switch is given
twice, the last one wins.

## Who draws the route

| Switch | In plain words | Gazebo, the lot tour |
|---|---|---|
| `planner:=lattice` (default) | State Lattice: strings our car's own moves together; fast | 30 Oct, reversing: every leg, **184 s** a tour. Forward only (29 Oct): 2 of 4, lost the U-turn |
| `planner:=hybrid` (forward only) | Hybrid-A*: only routes a car can drive | 30 Oct, reversing: every leg, 213 s a tour |
| `planner:=lattice lattice_headings:=32` | the lattice with 32 headings, not 16 | 30 Oct, reversing: every leg, 197 s; slower plans. No better |
| `planner:=smac2d` | A* on the grid: **ignores the turning circle** | 29 Oct, forward only: 2 of 4 |
| `planner:=navfn` | NavFn, the classic: ignores the turning circle | 3 Nov, one leg: finished (its tightest turn 128 m) |
| `planner:=theta_star` | Theta*: straight lines: ignores the turning circle | 3 Nov, one leg: finished |
| `planner:=diffusion` (a trial) | **a model draws the route**, a guard checks it, the lattice's route if it fails. Needs `pixi install --locked -e learn` and `pixi run diffusion train` | 3 Nov, reversing, 3 runs: every leg, 177-270 s (the lattice 183-184); **73 %** of routes from the model ([trials](trials.md#3-nov-everything-for-02)) |

The lattice is the default since 1 Nov (Ben), on the car too, now that it may reverse.
Forward only (`reversing:=false`, or no lidar), the car takes `hybrid`: the lattice lost
the U-turn. The real car takes only `hybrid` or `lattice` (it refuses `diffusion` too). More: [Controllers, Planners](controllers.md), [the runs](trials.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).

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
| `tracker:=mpc_lag` | ours: MPC that knows the car is late, and plans for it (7 Nov) | our simulator, 8 Nov: 6 of 12 (no U-turn by the building; [trials](trials.md#lag-aware-mpc-7-nov-mpc_lag)) |
| `tracker:=mppi_lag` | ours: Nav2's MPPI in Rust, knowing the steering and pedals lag ([page](mppi-lag.md)) | our simulator, 8 Nov, 3 runs: 11 of 12 (Nav2's MPPI 8; [trials](trials.md#results-our-simulator-8-nov-the-lot-tour-3-runs-each)). It passes a goal at 2.5 m/s and uses twice Nav2's MPPI's CPU ([our navigator](own-navigator.md#the-verdict)) |

`aav_track`'s trackers drive forward only: give them `reversing:=false` (the menu does).
`mppi_lag` reverses (14 Nov).
In `demo --debug`, keys 1-4 swap the tracker live. More: [Controllers](controllers.md),
[Trials](trials.md).

## Where am I

These run **beside** the EKFs and are only scored for now. They never steer, except
`ndt_ekf:=true`: then the global EKF, which Nav2 plans by, takes NDT's pose too; and
`kiss_ekf:=true`: then the local EKF, which Nav2 steers by, takes KISS-ICP's speed too.
In `pixi run pick`, `demo --debug` offers NDT + AMCL first (to see every finder); the
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

`tracker:=mpc_lag` predicts the speed loop it is told about: `default` or `smooth` (its
`mpc_lag.speed_loop` setting in `track.yaml`; it doesn't follow `speed_loop:=` on its own,
and doesn't model `lqi` yet).

## The world and the rest

| Switch | What |
|---|---|
| `world:=test_lot` / `road` / `empty` / `campus` | where the car is (`sim` only; the demos use the test lot; the default is `empty`). `campus` needs `pixi run campus` first ([page](campus.md)) |
| `simulator:=gazebo` | Gazebo plays the world. The default is `simulator:=own`, our simulator, since 8 Nov ([page](own-simulator.md)); it needs a GPU for its cameras |
| `reversing:=false` | Nav2 drives forward only |
| `sim_speed:=3` | a top speed above the car's 1 m/s (up to 5), in the simulator only; `world:=campus` sets 2.5 |
| `lag_scale:=2` | the car's throttle, brake and steering lags twice what `vehicle.yaml` says; every driver still reads `vehicle.yaml` (a robustness test; with `simulator:=gazebo` only) |
| `geofence:=false` | no geofence (it's on by default) |
| `lidar:=false`, `radar:=false` | sensors off |
| `voxels:=true` | Nav2 sees the lidar as 0.25 m cubes, not a flat scan (a trial: [Trials](trials.md#voxels-30-oct-the-lidar-as-cubes)) |
| `costmap:=own` | our own costmaps (Rust) beside Nav2's; `tracker:=mppi_lag` then reads ours. 8 Nov: the same grid as Nav2's, half the CPU; with its clear service, the obstacle legs 3 of 3 as Nav2's ([page](own-costmap.md)) |
| `planner:=own` | our own planner (Rust: Nav2's lattice ported) answers the behaviour trees, Nav2's lattice beside it; with `costmap:=own` it plans on our global costmap. 8 Nov: the test lot 12 of 12 as Nav2's, about 20 % slower on the campus's long legs ([page](own-planner.md)) |
| `navigator:=own` | our own navigator (Rust) in place of Nav2's bt_navigator and waypoint_follower; with `planner:=own costmap:=own tracker:=mppi_lag` no Nav2 server navigates (all ours). 8 Nov: test lot 7 of 12 legs (Nav2 8), campus 4 of 6 (Nav2 2) ([page](own-navigator.md)) |
| `smoother:=own` | our own velocity smoother (Rust) in Nav2's `velocity_smoother`'s place: Nav2's speed limits, and the turn rate from where the wheels can be (the steering's rate and full lock). 8 Nov, test lot: the same legs as Nav2's, ego clipped 0-0.1 % of the steering against 9-15 % ([page](own-smoother.md)) |

| `behaviour:=true` | the behaviour planner: a speed cap on Nav2 before a stop sign, an obstacle, the fence and the goal (a trial, off by default: [page](stopping.md#2-the-behaviour-planner)) |
| `lane_map:=world` | with `behaviour:=true`: the road ahead's line on the right lane's centre, with its edges, from the world's lane map (the road world's, or the campus's `maps/campus/lanes.yaml`); or a `lanes.yaml`'s path. Off by default: the line is `/plan`'s |
| `payload:=150`, `adaptive_brake:=true` | kg aboard the modelled car; the brake that learns it stops too slowly (off by default; not with `spine` or `esp32`: [page](stopping.md#4-a-heavier-car-the-adaptive-brake)) |
| `plan_around:=false` | the planner sees only the map, so the car stops for an obstacle (default `true`: it plans round) |
| `obstacle:=true`, `ghost:=true` | a 1 m box 15 m ahead in the lane; a pretend person in Nav2's view ([page](ghost-obstacles.md)) |

### Every other switch of `sim.launch.py`

`sim.launch.py` declares **59** switches (14 Nov); the tables above cover the ones that
swap an algorithm. The rest:

| Switches | What | Default |
|---|---|---|
| `nav`, `rviz`, `gui`, `chase_camera`, `arm`, `watchdog`, `actuator_model` | Nav2, RViz, the simulator's window, the chase camera, arming the spine, the sensor watchdog, the actuator model | `true` |
| `teleop`, `hud`, `map`, `foxglove` | a joystick here, the HUD, the map window, the Foxglove bridge | `false` |
| `perception`, `lane`, `camera_obstacles`, `faults` | the detector, lane keeping (`true` or `fake`), boxes into the costmap, the camera fault path (worlds with cameras) | `false` |
| `spine`, `esp32`, `esp32_loop`, `bridge` | the spine or today's ESP32 in the loop; `pi` or `open`; `rust` or `python` | `false`, `false`, `pi`, `rust` |
| `start_x`, `start_y`, `start_yaw` | where the car starts (m, rad) | `0.0` |
| `geofence_radius` | the fence's circle (m) | `200.0` |
| `watch_hud`, `watch_screens`, `screen_cubes` | screen heartbeats; the cubes for the developer window | `false`, empty, `false` |
| `diffusion_model`, `diffusion_smoothing`, `diffusion_record` | with `planner:=diffusion`: the model file, `gear` or `average`, a folder to record plans | empty, `gear`, empty |
| `ndt_map` | NDT's map file | `maps/ndt/test_lot.pcd` |

**Tried, 29 Oct 23:05:** the menu's pick (demo --debug, `mpc`, NDT, LQI) in Gazebo on Ben's
Mac: the track node ran MPC, NDT tracked (2 ms a match), AMCL stayed off, the speed loop
said LQI, and a Nav2 goal 15 m ahead ended SUCCEEDED.

## The defaults: what runs if you say nothing

Checked 3 Nov ([the runs](trials.md#3-nov-everything-for-02)): every switch that existed then
started and drove a leg in Gazebo. Checked against the launch files 14 Nov: 0.3 changed one
default (the simulator); our Rust pieces are all trials.

| System | Sim | Bench | Car | Why this default | On trial beside it |
|---|---|---|---|---|---|
| Who draws the route | `lattice` | `lattice` | `lattice` (`hybrid` if it can't reverse) | every leg reversing, the fastest: 183 s a tour (3 Nov, 3 runs) | `hybrid`, `lattice32`, `smac2d`, `navfn`, `theta_star`, `diffusion` (8 Nov: 88 % from the model, still a trial), `own` (8 Nov: 12 of 12 as Nav2's; about 20 % slower on the campus) |
| Who steers | MPPI | MPPI | MPPI | it reads the costmap, so it steers round what the lidar sees; it reverses | RPP; our pure pursuit, Stanley, LQR, MPC, `mpc_full`, `mpc_lag` (forward only), `mppi_lag` (reverses too); sim only: the bench and the car have no `tracker:=` |
| The costmaps | Nav2's | Nav2's | Nav2's (always) | stock; the car never takes ours | `costmap:=own` (8 Nov: the same grid, half the CPU; sim only) |
| Who runs the goal | Nav2's `bt_navigator` | the same | the same (always) | stock | `navigator:=own` (8 Nov: lot 7 of 12 legs, Nav2 8; campus 4 of 6, Nav2 2; sim only) |
| The velocity smoother | Nav2's | Nav2's | Nav2's (always) | stock | `smoother:=own` (8 Nov: the same legs, ego clips 0-0.1 % against 9-15 %; sim only) |
| Behaviour planner | off | off | none | a trial (7 Nov); in the sim it never slowed for the stop sign ([page](stopping.md#2-the-behaviour-planner)) | `behaviour:=true` (sim only) |
| May it reverse? | yes | yes | yes, unless nothing watches behind | the U-turn needs it | `reversing:=false` |
| What Nav2 sees | the flat lidar scan | the radar only (no lidar) | the flat scan | stops 0.16 m before the post, every time | `voxels:=true`: 16 s slower a tour, and 1 of 3 post runs drove through it |
| Where am I (map) | the global EKF: wheels, IMU, GPS | the same | the same | needs nothing but the sensors | `ndt_ekf:=true`: **17 → 1.8 cm**, every leg, but needs a saved map; `amcl:=true`, `ndt:=true` (scored only) |
| How far it moved | `ekf_local`: wheels, IMU | the same | the same | | `kiss_icp:=true` (scored), `kiss_ekf:=true` (no clear win) |
| Speed loop | `default` (PI) | `default` | `default` (no switch on the car) | today's PI loop; `sim-check` 9 of 9 with each of the three | `lqi`, `smooth` (sim and bench) |
| Geofence, sensor watchdog | on | on | on | they hold Nav2 when the pose or a sensor goes bad | `geofence:=false` (sim only) |
| Who plays the world | ours (`simulator:=own`, since 8 Nov) | - | - | every `sim-check` leg Gazebo passes, 3 times each; no GPU-less start-up crashes | `simulator:=gazebo` (and on a computer with no GPU) |

**Why the winners stay off:**

- **NDT into the EKF** wins clearly in the lot, but needs `maps/ndt/test_lot.pcd`, which
  isn't in git: a new laptop and the road world have none. A default that changes with the
  files on each computer would surprise people. Pick it: `pixi run pick`, `ndt_ekf`.
- **On the car**, NDT needs a recorded map of the real lot first; until then, today's
  defaults.
- **Voxels** isn't better on the tour, and the post run that drove through must be
  understood first.

Every switch with its own words: `ros2 launch aav_bringup sim.launch.py --show-args`.
