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

## The stack: ours by default

Since 14 Nov the simulator runs **our stack** unless told otherwise (Ben: "Ours default
now, then delete"). One switch sets every level at once ([the league](league.md)):

| Command | What runs |
|---|---|
| `pixi run sim` | ours: our planner, `mppi_lag`, our costmaps and navigator, the behaviour planner (`stack:=ours`) |
| `pixi run sim stack:=best` | every level's champion |
| `pixi run sim tracker:=lqr` | ours, but LQR steers: **a switch you give always wins** |

Nav2 went on 17 Nov (Ben, 16 Nov: "Now"; [Nav2, as it was](history/nav2-as-it-was.md)):
`stack:=stock` and Nav2's planners, controllers, costmaps, navigator and smoother went with
it. The real car (`pixi run car`) and the bench run ours too.

## Who draws the route

| Switch | In plain words | Gazebo, the lot tour |
|---|---|---|
| `planner:=own` (default) | ours: Nav2's lattice ported to Rust (more in the table below) | 8 Nov: the test lot 12 of 12 as Nav2's |
| `planner:=own planner_search:=hybrid` | ours running Hybrid-A* (`path:=hybrid_rs`, a challenger) | 14 Nov ([page](own-planner.md)) |
| `lattice_headings:=32` | the lattice with 32 headings, not 16 | 30 Oct (Nav2's lattice), reversing: every leg, 197 s; slower plans. No better |
| `planner:=diffusion` (a trial) | **a model draws the route**, a guard checks it, the lattice's route if it fails. Needs `pixi install --locked -e learn` and `pixi run diffusion train` | 3 Nov, reversing, 3 runs: every leg, 177-270 s (the lattice 183-184); **73 %** of routes from the model ([trials](history/trials-log.md#3-nov-everything-for-02)) |

Ours is the default everywhere since Nav2 went (17 Nov); the simulator ran it from 14 Nov.
Nav2's planners (`lattice`, `hybrid`, `smac2d`, `navfn`, `theta_star`) went with it; their
scores stay in [Controllers, Planners](controllers.md). The real car takes only `own` (it
refuses `diffusion` without `diffusion_trial`). More: [Controllers, Planners](controllers.md), [the runs](history/trials-log.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).

`path:=lane_jerk` (or `lane_pass_shape:=jerk` with `lane_route`; TODO 214, a challenger,
scored in stage 2): a parked obstacle on the lane is passed by a QP that holds the
**lateral jerk** at the car's top speed (Apollo's piecewise-jerk path), not the half cosine.

## Who steers along the path

| Switch | In plain words | Gazebo, 29 Oct |
|---|---|---|
| `tracker:=pure_pursuit` | ours: steer at a point ahead | **12 of 12** |
| `tracker:=stanley` | ours: fix the distance off the path and the heading | **12 of 12** |
| `tracker:=lqr` | ours: a formula for the best steering each moment | 10 of 12 |
| `tracker:=mpc` | ours: try the next 1.5 s of steering, keep the best | **12 of 12** |
| `tracker:=mpc_full` | ours: MPC that plans the speed too | 10 of 12 |
| `tracker:=mpc_lag` | ours: MPC that knows the car is late, and plans for it (7 Nov) | Ravensim, 8 Nov: 6 of 12 (no U-turn by the building; [trials](history/trials-log.md#lag-aware-mpc-7-nov-mpc_lag)) |
| `tracker:=mppi_lag` (default; Nav2's MPPI and RPP went on 17 Nov) | ours: Nav2's MPPI in Rust, knowing the steering and pedals lag ([page](mppi-lag.md)) | Ravensim, 8 Nov, 3 runs: 11 of 12 (Nav2's MPPI 8; [trials](history/trials-log.md#results-our-simulator-8-nov-the-lot-tour-3-runs-each)). It passes a goal at 2.5 m/s and uses twice Nav2's MPPI's CPU ([our navigator](own-navigator.md#the-verdict)) |

`mppi_lag` reverses (14 Nov); `aav_track`'s trackers too since 19 Nov (TODO 174: each
backwards stretch of the plan mirrored, `gears.py`); `mpc_lag_rs` since 20 Nov (TODO 213,
the same in Rust, `gears.rs`). Every tracker reverses.
`steer:=mppi_lag_1k` (or `mppi_lag_batch:=1000`): `mppi_lag` with 1000 samples a tick, not
2000: half the work, a challenger for the Orin (TODO 152).

`tracker:=mppi_lag mppi_lag_sampling:=smooth` (or `steer:=mppi_lag_smooth`; TODO 212, a
challenger, scored in stage 2): MPPI samples the steering's **change**, so its commands come
out smooth without a larger control cost.
In `demo --debug`, keys 1-4 swap the tracker live. More: [Controllers](controllers.md),
[Trials](trials.md).

## Where am I

These run **beside** the EKFs and are only scored for now. They never steer, except
`ndt_ekf:=true`: then the global EKF, which the planner plans by, takes NDT's pose too; and
`kiss_ekf:=true`: then the local EKF, which the follower steers by, takes KISS-ICP's speed too.
In `pixi run pick`, `demo --debug` offers NDT + AMCL first (to see every finder); the
others offer the EKFs only first.

| Switch | In plain words | Off from the truth |
|---|---|---|
| (nothing) | the EKFs: wheels, IMU and GPS | ~17 cm |
| `ndt:=true ndt_map:=maps/ndt/test_lot.pcd` | the lidar slid onto a saved map | **1-3 cm** |
| `amcl:=true` | many guesses; keep the ones that match the walls (ours in Rust; stock removed 20 Nov) | 6-11 cm (5-9 cm, 18 Nov) |
| `ndt_ekf:=true ndt_map:=maps/ndt/test_lot.pcd` | NDT, and the global EKF **uses** it beside the GPS (a trial) | the EKF **1.6 cm** ([trials](history/trials-log.md#ndt-into-the-global-ekf-30-oct)) |
| `pose_source:=ndt` | NDT **leads** the global EKF; the GPS only checks it, and stands in when NDT is lost (a challenger, TODO 210; `aav_pose_check`) | the EKF **1 cm**, and the same with the GPS stopped, jumping or drifting 5 m ([Maps and position](maps-and-position.md#8-ndt-leads-the-gps-checks)) |
| `kiss_icp:=true` | how far the car moved, from the lidar alone (no map) | 0.02-0.04 m per 100 m driven |
| `kiss_ekf:=true` | KISS-ICP, and the local EKF **uses** its speed beside the wheels (a trial) | no clear win yet ([trials](history/trials-log.md#kiss-icp-into-the-local-ekf-1-nov)) |

NDT's map isn't in git: make it once with `pixi run ndt-map drive` (about 6 min).
KISS-ICP needs its own environment once: `pixi install --locked -e kiss`.

## Speed

| Switch | In plain words |
|---|---|
| `speed_loop:=default` | today's PI loop |
| `speed_loop:=lqi` | no overshoot, settles in 4-5 s |
| `speed_loop:=smooth` | the PI with a smooth stop |
| `speed_loop:=firm` | `smooth` with a firmer brake: stops where it was told |
| `speed_loop_model:=smooth` | the car keeps `speed_loop:=`'s loop; every model of the car plans with this one. Empty (the default): the car's own |

Every model of the car follows `speed_loop:=` (TODO 163): the path follower's
(`mppi_lag`, `mpc_lag`, `mpc_lag_rs`), ego's trace and the behaviour planner's stopping
law, unless `speed_loop_model:=` names another (sim and bench). `tracker:=mpc_lag` and
`mpc_lag_rs` don't model `lqi` yet: the launch refuses it as their model.

## The world and the rest

| Switch | What |
|---|---|
| `world:=test_lot` / `road` / `empty` / `campus` | where the car is (`sim` only; the demos use the test lot; the default is `empty`). `campus` needs `pixi run campus` first ([page](campus.md)) |
| `camera_lockstep:=true` | Ravensim's clock waits for each picture (`auto`, the default: under `tools/software_gl.sh`, no GPU). `simulator:=ravensim` is the only simulator since Gazebo went (16 Nov); `own` still works for one release |
| `reversing:=false` | navigation drives forward only |
| `sim_speed:=3` | a top speed above the car's 1 m/s (up to 5), in the simulator only; `world:=campus` sets 2.5 |
| `lag_scale:=2` | the car's throttle, brake and steering lags twice what `vehicle.yaml` says; every driver still reads `vehicle.yaml` (a robustness test, in `sim_adapter`) |
| `geofence:=false` | no geofence (it's on by default) |
| `lidar:=false`, `radar:=false` | sensors off |
| `voxels:=true` | the costmap sees the lidar as 0.25 m cubes, not a flat scan (a trial: [Trials](history/trials-log.md#voxels-30-oct-the-lidar-as-cubes)) |
| `costmap:=own` (the only one since 17 Nov) | our own costmaps (Rust); `tracker:=mppi_lag` then reads ours. 8 Nov: the same grid as Nav2's, half the CPU; with its clear service, the obstacle legs 3 of 3 as Nav2's ([page](own-costmap.md)) |
| `planner:=own` (default) | our own planner (Rust: Nav2's lattice ported), on our global costmap. 8 Nov: the test lot 12 of 12 as Nav2's, about 20 % slower on the campus's long legs ([page](own-planner.md)) |
| `navigator:=own` (the only one since 17 Nov) | our own navigator (Rust) in place of Nav2's bt_navigator and waypoint_follower. 8 Nov: test lot 7 of 12 legs (Nav2 8), campus 4 of 6 (Nav2 2) ([page](own-navigator.md)) |
| `smoother:=own` (the only one since 17 Nov) | our own velocity smoother (Rust) in Nav2's `velocity_smoother`'s place: Nav2's speed limits, and the turn rate from where the wheels can be (the steering's rate and full lock). 8 Nov, test lot: the same legs as Nav2's, ego clipped 0-0.1 % of the steering against 9-15 % ([page](own-smoother.md)) |

| `behaviour:=false` | no behaviour planner. On by default in the sim (`stack:=ours`; off with `stack:=best`): a speed cap before a stop sign, an obstacle, the fence and the goal ([page](stopping.md#2-the-behaviour-planner)) |
| `lanes:=ramp` / `camera` / `both` | the lanes layer in our global costmap (`lane_costs:=true`), a challenger, off by default. `ramp`: the lane map's roads; `camera`: the same costs from the lane finder's lines, yellow or white (`lane_source:=camera`, TODO 264; it starts the lane finder): the kerb high, the centre line and oncoming lane medium, the right lane's centre low, never a wall; `both`: the lane map wins where it has a road ([page](own-costmap.md#the-lanes-from-the-camera-19-nov-todo-264)) |
| `lane_map:=world` | with `behaviour:=true`: the road ahead's line on the right lane's centre, with its edges, from the world's lane map (the road world's, or the campus's `maps/campus/lanes.yaml`); or a `lanes.yaml`'s path. Off by default: the line is `/plan`'s |
| `payload:=150`, `adaptive_brake:=true` | kg aboard the modelled car; the brake that learns it stops too slowly (off by default; not with `spine` or `esp32`: [page](stopping.md#4-a-heavier-car-the-adaptive-brake)) |
| `plan_around:=false` | the planner sees only the map, so the car stops for an obstacle (default `true`: it plans round) |
| `obstacle:=true`, `ghost:=true` | a 1 m box 15 m ahead in the lane; a pretend person only navigation sees ([page](ghost-obstacles.md)) |

### Every other switch of `sim.launch.py`

`sim.launch.py` declares **59** switches (14 Nov); the tables above cover the ones that
swap an algorithm. The rest:

| Switches | What | Default |
|---|---|---|
| `nav`, `rviz`, `gui`, `chase_camera`, `arm`, `watchdog`, `actuator_model` | navigation, RViz, the simulator's window, the chase camera, arming the spine, the sensor watchdog, the actuator model | `true` |
| `teleop`, `hud`, `map`, `foxglove` | a joystick here, the HUD, the map window, the Foxglove bridge | `false` |
| `perception`, `lane`, `camera_obstacles`, `faults` | the detector, the lane finder alone (`true`), boxes into the costmap, the camera fault path (worlds with cameras) | `false` |
| `lane_finder_impl` | with `lane` or `lanes:=camera`: the lane finder in `rust` or `python`. `lane_follower`, `lane_follower_impl` and `lane:=fake` went with the lane followers on 19 Nov ([their scores](history/lane-followers.md)) | `rust` |
| `geofence_impl`, `stop_hold_impl`, `goal_leash_impl` | the geofence, the stop hold and the goal leash in `rust` (`aav_safety_rs`, 19 Nov: the Python's answers to the bit; the Python removed 20 Nov); the car and the bench take them too | `rust` |
| `gps_heading_impl` | the GPS heading in `rust` (`aav_gps_heading_rs`, 19 Nov: `course.py`'s answers to the bit; the Python removed 20 Nov); `localization.launch.py`'s | `rust` |
| `spine`, `esp32`, `esp32_loop`, `bridge` | the spine or today's ESP32 in the loop; `pi` or `open`; `rust` or `python` | `false`, `false`, `pi`, `rust` |
| `start_x`, `start_y`, `start_yaw` | where the car starts (m, rad) | `0.0` |
| `geofence_radius` | the fence's circle (m) | `200.0` |
| `watch_hud`, `watch_screens`, `screen_cubes` | screen heartbeats; the cubes for the developer window | `false`, empty, `false` |
| `diffusion_model`, `diffusion_smoothing`, `diffusion_record` | with `planner:=diffusion`: the model file, `gear` or `average`, a folder to record plans | empty, `gear`, empty |
| `ndt_map` | NDT's map file | `maps/ndt/test_lot.pcd` |

**Tried, 29 Oct 23:05:** the menu's pick (demo --debug, `mpc`, NDT, LQI) in Gazebo on Ben's
Mac: the track node ran MPC, NDT tracked (2 ms a match), AMCL stayed off, the speed loop
said LQI, and a Nav2 goal 15 m ahead ended SUCCEEDED.

## Stage 1, batch 8: challengers built 19 Nov, scored in stage 2

Each is off by default, and a drop-in: the same topics as the champion.

| Switch (league option) | In plain words | TODO |
|---|---|---|
| `mppi_lag_sampling:=smooth` (`steer:=mppi_lag_smooth`) | MPPI samples the steering's change, so its commands come out smooth | 212 |
| `lane_pass_shape:=jerk` (`path:=lane_jerk`) | a parked obstacle passed within a lateral-jerk limit (a QP) | 214 |
| `ground:=patchwork` (`kerbs:=patchwork`) | the ground found patch by patch, ring by ring: a lower kerb is an obstacle, a hill isn't | 215 |
| `odom_lidar:=kinematic_icp` with `kiss_ekf:=true` (`pose:=kinematic_ekf`) | lidar odometry that knows the car can't slide sideways, the wheels its prior, in KISS-ICP's place | 216 |
| `things_tracker:=per_kind` with `things:=true` (`behaviour:=on_per_kind`) | the things tracked with a motion model per kind (a car can't slide, a person can turn on the spot); a thing found again re-fits its speed | 217 |

## The defaults: what runs if you say nothing

Checked 3 Nov ([the runs](history/trials-log.md#3-nov-everything-for-02)): every switch that existed then
started and drove a leg in Gazebo. Checked against the launch files 14 Nov: 0.3 changed one
default (the simulator). Later on 14 Nov the sim's stack became ours (`stack:=ours`): the
Sim column below. On 17 Nov Nav2 went: the bench and the car run ours too.

| System | Sim | Bench | Car | Why this default | On trial beside it |
|---|---|---|---|---|---|
| Who draws the route | `own` (ours) | `own` | `own` | Nav2's lattice ported; Nav2's went on 17 Nov | Hybrid-A* in ours (`planner_search:=hybrid`), `lattice32`, `diffusion` (8 Nov: 88 % from the model, still a trial) |
| Who steers | `mppi_lag` (ours) | `mppi_lag` | `mppi_lag` | it reads the costmap, so it steers round what the lidar sees; it reverses | our pure pursuit, Stanley, LQR, MPC, `mpc_full`, `mpc_lag` (all reverse since 19 Nov), `mpc_lag_rs` (reverses since 20 Nov); sim only: the bench and the car have no `tracker:=` |
| The costmaps | ours | ours | ours | Nav2's went on 17 Nov | |
| Who runs the goal | ours (`aav_navigator`) | ours | ours | Nav2's went on 17 Nov | |
| The velocity smoother | ours | ours | ours | Nav2's went on 17 Nov (a tie, 16 Nov) | |
| The stop zone | ours (`aav_stop_zone`) | ours | ours | it stopped every time Nav2's did, no later, in its shadow run (17 Nov) | |
| Behaviour planner | on | off | none | a trial (7 Nov); in the sim it never slowed for the stop sign ([page](stopping.md#2-the-behaviour-planner)) | `behaviour:=true` (sim only) |
| May it reverse? | yes | yes | yes, unless nothing watches behind | the U-turn needs it | `reversing:=false` |
| What the costmap sees | the flat lidar scan | the radar only (no lidar) | the flat scan | stops 0.16 m before the post, every time | `voxels:=true`: 16 s slower a tour, and 1 of 3 post runs drove through it |
| Where am I (map) | the global EKF: wheels, IMU, GPS | the same | the same | needs nothing but the sensors | `ndt_ekf:=true`: **17 → 1.8 cm**, every leg, but needs a saved map; `amcl:=true`, `ndt:=true` (scored only) |
| How far it moved | `ekf_local`: wheels, IMU | the same | the same | | `kiss_icp:=true` (scored), `kiss_ekf:=true` (no clear win) |
| Speed loop | `default` (PI) | `default` | `default` (no switch on the car) | today's PI loop; `sim-check` 9 of 9 with each of the three | `lqi`, `smooth` (sim and bench) |
| Geofence, sensor watchdog | on | on | on | they hold navigation when the pose or a sensor goes bad | `geofence:=false` (sim only) |
| Who plays the world | Ravensim (`simulator:=ravensim`, since 8 Nov; `own` until the next release) | - | - | every `sim-check` leg Gazebo passed, 3 times each; no GPU-less start-up crashes | none: Gazebo removed 16 Nov ([as it was](history/gazebo-as-it-was.md)) |

**Why the winners stay off:**

- **NDT into the EKF** wins clearly in the lot, but needs `maps/ndt/test_lot.pcd`, which
  isn't in git: a new laptop and the road world have none. A default that changes with the
  files on each computer would surprise people. Pick it: `pixi run pick`, `ndt_ekf`.
- **On the car**, NDT needs a recorded map of the real lot first; until then, today's
  defaults.
- **Voxels** isn't better on the tour, and the post run that drove through must be
  understood first.

Every switch with its own words: `ros2 launch aav_bringup sim.launch.py --show-args`.
