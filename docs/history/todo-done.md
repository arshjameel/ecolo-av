# TODO: what is done

The finished rows and the dated logs from `TODO.md`, moved here on 14 Nov (the whole car, step 8: "shrink the docs and TODO"). Each under the heading it had; nothing changed but the place.

## The whole car: every design merged (14 Nov)

Ben, 13-14 Nov: "a review of everything with the plan of merging everything into the best version possible ... what am I missing". The page: [the whole car](docs/designs/2026-11-14_the-whole-car.md). His words: `docs/history/messages-2026-11-14.md`.

| # | Task | Status |
|---|---|---|
| 153 | **The whole car** ([design](docs/designs/2026-11-14_the-whole-car.md)): one picture of the world (the grid, the things, the road ahead), one car model, one scorecard; what is missing; the order | ✅ 14 Nov proposed; Ben: build it, cut nothing |

## One shape, and lanes as soft costs (13 Nov)

Ben, 13 Nov: "How can we make all of this fit together better", and lane lines "it can cross but only if it needs to and to stay on the right side". The page: [one shape](docs/designs/2026-11-13_one-shape.md).

| # | Task | Status |
|---|---|---|
| 147 | **One shape** ([design](docs/designs/2026-11-13_one-shape.md)): nine levels, one world model (costmap, a list of things, one pose), one boss with a contract, 36 switches down to 8 | ✅ 13 Nov proposed; Fable reviewed (go, with changes; folded in). ⬜ Ben: D1-D7 |

## A heavier car: the adaptive brake (8 Nov)

Ben, 8 Nov: "if we brake without a person vs with a person ... we are going to need to dynamically realize that we aren't braking as fast as we expect and start applying more force. We don't want to overwhelm the brakes though, so we might want to know what the max values are like the steering system." The page: [a heavier car](docs/stopping.md#4-a-heavier-car-the-adaptive-brake).

| # | Task | Status |
|---|---|---|
| 145 | **The adaptive brake** (`aav_description` `adaptive_brake.py`; `adaptive_brake:=true` in the sim): measured against the empty car model's slowing, more brake for a car that stops too slowly; never above `brake: safe_max`, a rise rate, `max_hold_time` near it; the car model's `payload`; `sim-check --loaded-stop` | ✅ 8 Nov built and tested without ROS: from 2 m/s with 150 kg aboard the car model stops 0.58 m past the empty plan, +0.04 m with the adaptive brake (the page). ✅ 8 Nov `sim-check --loaded-stop-only`, our simulator, 5 of 5: 150 kg without it +0.87 m past the empty model; with it +0.21 m, the next stop +0.05 m; empty +0.04 / +0.05 m either way. ⬜ car day A7 rows 6-8: the brake's real limits into `vehicle.yaml` `brake:`. ⬜ on the car: in `legacy_uart_bridge`'s `pi` loop (Python, the same module), then spine-core in Rust (a twin with shared vectors); the wheel speed's noise first (8 spokes a turn) |

## Off Nav2, one piece at a time (8 Nov)

Ben, 8 Nov: our own versions beside Nav2, each until it wins; Nav2 stays the default and on the car. The order: the costmap, a Rust path planner, the navigator.

| # | Task | Status |
|---|---|---|
| 143 | **Our own costmap** ([page](docs/own-costmap.md), `aav_costmap`, `costmap:=own`): Nav2's layers in Rust (the map, the scan's marks with ray clearing, inflation round the car), local (odom, rolling) and global (the map), on `/drivers/nav/costmap/*`; `mppi_lag` reads ours | ✅ 8 Nov built, 30 tests without ROS; in our simulator 100 % of 40,000 cells the same as Nav2's (stopped at the post, 2 of 2); 4-5 % of a core for both, Nav2's local one 7-9 %. mppi_lag on ours, 3 runs each: plan around 3 of 3 (2.1-2.5 m clear; Nav2's 3 of 3, 2.4 m); lidar obstacle: stopped 0.20, 0.21 m (Nav2's 0.17-0.71), but once steered past the post and once never reached the goal after it went (Nav2's 3 of 3). ✅ 8 Nov: the trees' recovery clears ours too (`clear_entirely_local`, bt_navigator remapped): lidar obstacle again 3 of 3 to the goal (stops 0.64, 0.15, 0.18 m; 100 % of cells the same).; ⬜ radar and camera clouds in the sim; ⬜ the planner on `/drivers/nav/costmap/global_raw` (next piece) |
| 144 | **Our own planner** ([page](docs/own-planner.md), `aav_planner`, `planner:=own`): Nav2's State Lattice in Rust (the primitives file, nav2.yaml's penalties, the Reeds-Shepp finish, the smoother, the car's outline on the global costmap: Nav2's or ours); serves ComputePathToPose, bt_navigator remapped to it, Nav2's lattice beside it | ✅ 8 Nov built; 6 tests without ROS (the curves end on the goal; straight, U-turns, a parking-lot exit, a narrow gap, no path), all under 1.1 ms. Our simulator, 3 runs each with reversing: test lot **12 of 12** as Nav2's lattice (tours 182-189 s against 186-187), the same paths within 2.2 m, 3-37 ms a plan. Campus at 2.5 m/s: 4 of 9 against 3 of 9, both lost to MPPI's "Optimizer fail" at the start of the long leg; ours 168-195 ms there against 138-151 ms. ⬜ faster (cache each node's check where safe); ⬜ Hybrid-A* (forward only); ⬜ ComputePathThroughPoses; ⬜ TODO 140's P4 to Fieldhouse once MPPI gets past that leg; ⬜ the navigator step (next piece) |
| 145 | **Our own navigator** ([page](docs/own-navigator.md), `aav_navigator`, `navigator:=own`): bt_navigator's trees and waypoint_follower in Rust (plan, follow, replan, final approach, recoveries, through-poses, GPS waypoints by the datum), Nav2's actions under `/drivers/nav/navigator/`; goal senders switch by `navigator`; our planner also serves ComputePathThroughPoses; all ours = no Nav2 server navigates | ✅ 8 Nov built; 11 tests without ROS. Our simulator, all ours vs all Nav2, forward only: test lot 7 of 12 legs against 8 (3 runs; the U-turn 1 of 3 against 0); the box 3 of 3 both; sim-check all ours, 13 of 13 pass; campus at 2.5 m/s 4 of 6 against 2 (2 runs); P4 to the Fieldhouse 1 of 2 against 0. Why Nav2 gave up "along the road": lidar marks beside the lot exit where leg 1 ends make the car's own spot occupied ("Start occupied", MPPI "Optimizer fail"); ours clears and replans. ⬜ `mppi_lag` overshoots the campus's first goal at 2.5 m/s (under Nav2's navigator too); ⬜ its CPU (2x Nav2's MPPI); ⬜ not the sim default until those |
| 146 | **Our own velocity smoother** ([page](docs/own-smoother.md), `aav_smoother`, `smoother:=own`): Nav2's velocity_smoother's speed limits, timeout, deadband and feedback, with the turn rate from the steering actuator's rate and full lock (vehicle.yaml through the car model), under Nav2's node name; `measure controllers --smoothers`, and the new row "steering commands ego had to clip" | ✅ 8 Nov built; 16 tests without ROS. Our simulator, all ours, test lot, 3 runs each: 10 of 12 legs both; ego clipped 0-0.1 % against 9-15 %; off the path 0.12 m against 0.15 m (worst 0.65 against 0.92); tour 701 s against 890 s. ⬜ campus at 2.5 m/s; ⬜ `tracker:=mppi_lag` under Nav2's navigator; ⬜ lateral-acceleration cap (ego has it); ⬜ not the default until those |

## 0.3: diffusion on the car, as a trial (7-8 Nov)

Ben, 7 Nov: "We want diffusion." His choice: on the car in 0.3, a trial, low speed, behind the guard and the locks.

| # | Task | Status |
|---|---|---|
| 133 | **Diffusion for 0.3** ([design, As built 0.3](docs/designs/2026-11-02_diffusion-planner.md#as-built-03-7-8-nov), [trials](docs/trials.md#diffusion-7-8-nov-the-u-turn-the-orin-the-car)): why the guard refused (G5 turns, G6 at the U-turn's cusp); each gear smoothed on its own, a wider second try for G5/G6 only, U-turn data, the map encoded once; record and replay plans offline; TensorRT/CUDA/CPU with fallbacks, FP16 export, `pixi run measure diffusion`; the car gate `diffusion_trial:=true`, 0.5 m/s ([car day A12](docs/car-day.md#a12-the-diffusion-planners-trial), test 4.27) | ✅ 8 Nov, branch `worktree-agent-a0cbc7e904c345e4d`: offline 77 → 87 % of plans with a passing sample, G5 2,197 → 850. Simulator, 3 runs each: 77 → **88 %** of plans from the model (84, 89, 92 %; 92 % after the merge), the U-turn 40 → 72 %, 12 of 12 legs, 171-175 s against the lattice's 180-185 s. ⬜ the Orin's timing (TensorRT), the car trial (A12) |

## 0.3: directions on the campus (7 Nov)

Ben, 7 Nov: Tesla-like "higher level navigation for directions"; for 0.3, on the campus. The page: [On the campus](docs/routes.md#on-the-campus).

| # | Task | Status |
|---|---|---|
| 140 | **Directions on the campus**: a place's name (or lat/lon, or a map point) to a route on the campus's own road graph (`campus_graph.py`, from the `.osm` Gazebo's world comes from), cut into Nav2 GPS waypoints; `street_router router:=campus`, `/drivers/nav/route/place`; `pixi run campus-route`; 19 named places in the site file (`campus_site.yaml`, TODO 100's first piece) | ✅ 7 Nov built and tested without ROS; in Gazebo the route goes to Nav2. ❌ 8 Nov Gazebo, P4 to Fieldhouse (453 m): **did not arrive**; 310 m driven, stuck half-turned at a sharp bend (each bend is a stop-and-face goal). ⬜ bend waypoints before the corner, or `navigate_through_poses`; then the run again. ⬜ real drop-off points (`stop:`) measured on the day; a screen to pick a place (the screens' agent); ✅ 8 Nov our navigator (TODO 145) passes the waypoints as one through-poses goal: P4 to the Fieldhouse reached 1 of 2 (263 s; Nav2's waypoint_follower 0 of 2, stuck at the lot exit) |

## 0.3: ego knows the car (7 Nov)

Ben, 7 Nov: "we really want the ego to have a mathematical model for steering, braking and acceleration ... have the algorithms know about lag". The page: [one car model](docs/car-model.md).

| # | Task | Status |
|---|---|---|
| 127 | **One car model**, Python (`aav_description` `car_model.py`) and Rust (`car-model`), shared vectors; dead times; the bicycle; `step`, `predict` | ✅ 7 Nov. Gazebo unchanged |
| 128 | **ego's lag lead** (`lag_lead: auto`, sim only), off under a lag-aware driver (`/drivers/<slot>/lag_aware`) | ✅ 7 Nov, sim. ⬜ `on` for the car after tests 3.10-3.12, 4.1-4.5 |
| 129 | **ego's trace** (`/ego/trace`, 2 s, `base_link`) | ✅ 7 Nov; the HUD and the developer window draw it (136). ⬜ drivers match it to their plan |
| 136 | **The squiggly line** ([page](docs/hud.md#the-squiggly-line-ahead)): ego's trace (bold, green / amber / red by the gap to the plan), the plan (faint), the stop bar from `/ego/stop_point` (the car model, full brake now), the room to the goal or an obstacle; on the HUD (AHEAD) and the developer window (key T), pretend data too | ✅ 7 Nov built. ⬜ stop lines (no topic yet); the line drawn on the camera picture (Q22); the car's real numbers (tests 3.10-3.12, 4.1-4.5) |
| 130 | The lag-aware drivers publish `/drivers/nav/lag_aware`: `aav_track`'s MPC, the Rust MPPI | ✅ 7 Nov: both (rows 131, 132) |

## 0.3: how the car stops, and the behaviour planner (7 Nov)

Ben, 7 Nov: "We need navigation planning and lane following and we need an obstacle avoidance planner to avoid tricky situations ... we can use that to say if we have turned enough or slowing down enough". His picks for 0.3: the behaviour planner and a stop-point check. The page: [how the car stops](docs/stopping.md).

| # | Task | Status |
|---|---|---|
| 137 | **The behaviour planner** (`aav_behaviour`, `behaviour:=true`, a trial): DRIVE, APPROACH, STOPPED, WAIT, GO, ASK; a speed cap on Nav2 (`/drivers/nav/speed_limit`) from **one stopping law** (`aav_description` `stop_law.py`, TODO 110) for a stop sign, an obstacle in the lane, the fence and the goal; `/drivers/nav/behaviour/state` for the screens; the watcher's `/perception/stop_sign/distance` | ✅ 7 Nov built, every transition tested; the sim runs: see the page. ⬜ the `patient` progress checker, ASK's reply, the trackers reading the cap, the car (stage gates) |
| 138 | **The stop-point check** (`pixi run sim-check --stop-point`): drives at 1, 2, 3 m/s, slows at 1 m/s², and scores where the car stopped (truth and localization) against the car model's `stop_point`; `tools/stop_points.py` can score a car bag later, with a tape for truth | ✅ 7 Nov, numbers on the page. ⬜ on the car, after tests 4.1-4.5, with the real numbers |
| 139 | **Wiring**: the shared `car-model` crate behind `sim-core`'s and `mppi-lag`'s `CarModel` traits (feature `shared`) | ✅ 7 Nov, tested against their own copies. ⬜ the nodes switched over, after a quiet-machine sim-check |

## 0.3: Gazebo merged into Rust (7 Nov)

Ben, 7 Nov: "Does the bevy tool still use gazebo in the background. I would love to merge in everything from gazebo so its standalone in rust." His picks: everything in 0.3, all sensors.

| # | Task | Status |
|---|---|---|
| 134 | **Our own simulator's core** ([page](docs/own-simulator.md), `aav_simulator`, `sim simulator:=own`): Gazebo's worlds (test lot, road, campus), the car (a `CarModel` trait, the car-model crate's rules), the roof lidar and the radar stand-in by ray casting, IMU, GPS, wheels, truth, `/clock`, TF through `/joint_states`; the car stops at what it hits; spawn and delete services; `sim-check --simulator own` | ✅ 7 Nov built; on a loaded Mac: drop-in pass (no cameras), a Nav2 goal reached, `sim-check` pass but the box run's last leg (a late TF), `--lidar-obstacle` pass. ⬜ the cameras plug in (another branch); `car-model` behind the trait; a quiet-machine run beside Gazebo's spread; then ours the default |

## Saturday 7 November: release 0.3, our own simulator

Ben, 7 Nov: replace Gazebo with our own Rust simulator, every sensor. The core (world, car, lidar, IMU, GPS, wheels, clock, TF) and the cameras are built side by side.

| # | Task | Status |
|---|---|---|
| 135 | **The simulator's cameras** ([page](docs/own-simulator-cameras.md), `aav_sim_cameras`): Bevy offscreen, Gazebo's four camera topics, `CameraRenderer` for the core. Left: feed it the core's world and truth; read Gazebo's SDF worlds; a no-GPU path for CI; compare with Gazebo's pictures; the detector on its pictures | ✅ built, test world only. ⬜ merge with the core |
| 141 | **The cameras joined to our simulator** ([page](docs/own-simulator-cameras.md)): the core's worlds (test lot, road, campus: boxes, cylinders, planes, OBJ meshes, their pictures, the sky and sun) drawn by `sim_cameras`; `simulator:=own` starts them, so perception, lane and faults run on ours. Also the ground under each wheel (slopes tilt the car, a 0.3 m kerb stops it) | ✅ 8 Nov: `sim-check --simulator own --lane` all pass; drop-in: every camera at 10 Hz. Head to head on a quiet Mac (`sim-check` x 3, `--obstacle`, `--lidar-obstacle`, `--speed-steps`, `--lane-only`): ours passes all, Gazebo the same but one start-up. **The default since 8 Nov:** `--perception`, `--faults`, `--drive-to-point-only` 3 of 3 each on ours; placed models drawn in the pictures. Left: the one stop-sign miss (1 of 8, cause unknown); computers with no GPU use `simulator:=gazebo` |
| 142 | **Lane following for 0.3** ([page](docs/perception.md#in-the-simulators-8-nov)): `measure lane` scores a long drive by the truth (road world, campus roads); the campus's painted roads get edge lines | ✅ 8 Nov on ours: Library Road 171 m at 1.4 m/s, error mean 4 cm, 95 % 13-15 cm, lane lost in 0-1 % of pictures. Gazebo on the campus: lane lost in 68 % (not diagnosed). Left: unpainted service roads; real pictures |

## Saturday 7 November: 0.3, the algorithms know about lag (MPPI)

Ben, 7 Nov: "have the algorithms know about lag". MPPI out of Nav2 into our own Rust driver, its model with the actuators' lag; Nav2 keeps planning.

| # | Task | Status |
|---|---|---|
| 131 | **MPPI that knows the lag** (`aav_mppi_lag`, `tracker:=mppi_lag`, [page](docs/mppi-lag.md)): Nav2's MPPI and its eight critics in Rust, on a lag model behind a `CarModel` trait (the shared car model's rules); `/drivers/nav/lag_aware` true while it drives, for ego's lead | ✅ 7 Nov built, tested without ROS. ⬜ Gazebo against Nav2's MPPI on a quiet computer (7 Nov's runs: the odometry stalled under load and it gave up; the page). ⬜ at merge: the shared `car-model` crate behind `CarModel` (its rules matched already). ⬜ the car's real lags (tests 3.10-3.12, 4.1-4.5), then the same runs again; the Orin's tick time (estimated, not measured) |

## Wednesday 4 November: the weak links, for Thursday's pick

Ben, 4 Nov: "think about the areas of this car and what is the weakest ... ideas on how to improve each section of the driving experience", then "save this full list". The list: [the weak links](docs/designs/2026-11-04_weak-links.md) (every area, the evidence, the ideas, the papers). Words: `docs/history/messages-2026-11-04.md`.

| # | Task | Status |
|---|---|---|
| 99 | **The weak-links list**: every area, what's weakest, ideas tagged eng / res; the three research proposals (A: identification + a CBF safety filter; B: the lidar teaches the camera; C: degeneracy-aware NDT); the site file; the Nav2 verdict; the Drive screen | ✅ 4 Nov: `docs/designs/2026-11-04_weak-links.md`. ⬜ Thursday: Ben picks |
| 105 | **Every research idea judged, and the new ones** (Ben, 4 Nov: "look at all the research ideas we have ... what improvements or edge can we bring"): the 20 ideas we had, each kept, folded or dropped; 12 new ones (shadow mode, the winter lot, the blind-zone memory, the radar's own tracker, the tuner on the harness, the VLM that may only say STOP, the brake-health monitor, speed by trust, shielded learning, pole landmarks, the truth rig, stop-and-stare); three threads (knows itself, the winter lot, learns safely) and a month-by-month plan | ✅ 4 Nov, corrected 5 Nov after the EGM review: a backlog, not a plan; "drop" is now "not pursued; stays available". ⬜ Ben: its proposals |
| 106 | **One great car** (Ben, 4 Nov: "taking the best of everything we know ... Simplify down to the best parts of each algorithm and put them all together into the best of all worlds"): the direction after 0.3; a convergence table, one winner per area, in [one car](docs/designs/2026-11-04_one-car.md) | ✅ 4 Nov written (one car, section 4). **Corrected by Ben, 5 Nov:** one design is for down the road; now we build new algorithms of our own from papers, known algorithms and current implementations. Ben, 5 Nov: keep every algorithm, a new best hybrid per area, **how the car stops first**, stock stays the default until ours wins. ⬜ the stopping design, after the EGM stages |
| 109 | **The EGM review of the 4-5 Nov designs** ([report](docs/designs/2026-11-05_egm-review.md)): none passed; all eight corrected after it, and all but `one-car` are now far too long for "short pages" | ✅ 5 Nov, uncommitted. ⬜ cut them down: the stopping design takes Part 3 of `knows-itself` and the requirements in `behaviour-and-speed`; `one-window` can split off SWAP |
| 110 | **How the car stops** ([design](docs/designs/2026-11-05_how-the-car-stops.md)): Ben's first hybrid, one stopping law from `approach.py`, `dynamics.py`, `speed.py` and `geofence.py` | ✅ 7 Nov `stop_law.py` (row 137), with the speed loop's lag in ρ; the trackers use it, the point driver with `stop_law:=hybrid`. ⬜ the point driver in Gazebo, 10 runs against `approach`; the CBF filter |
| 117 | **One RViz** (Ben, 5 Nov: "two nav screens"): `demo_all.sh` says `rviz:=false` | ✅ edited 5 Nov; not launched yet |
| 118 | **Everything is a drop-in** (Ben, 5 Nov: "just swap out and test"): one conformance check against `docs/interfaces.yaml` for any sensor source or algorithm, and find what still depends on Gazebo by name | ✅ 5 Nov: `pixi run drop-in`, the contract's `drop_in` rules, `/vehicle/sim/` and `/control/sim/`, target `sim`. Against real sensors: not yet |
| 123 | **MPPI held the car near 1 m/s** (found 5 Nov): its 3 m of path was shorter than its futures at any higher speed. The path now grows with `sim_speed`; the campus defaults to 2.5 m/s (mean 2.2) | ✅ Gazebo only |

## Tuesday 3 November: 0.2 polished for the meeting (4 Nov), then the car

Ben, 3 Nov: "Tomorrow is meeting day and we want to have 0.2 fully polished and documented ready to go." The plan: [the brief](docs/designs/2026-11-03_release-0.2-brief.md), "Recommendations for today". Words: `docs/history/messages-2026-11-03.md`.

| # | Task | Status |
|---|---|---|
| 85 | **The 0.2 guide**: every feature, what it does, how to run it, tips, how solid; the state machine; the two EKFs; which parts are Python | ✅ 3 Nov, `guide-0.2`: `docs/release-0.2.md` is now "The 0.2 guide" (same file and PDF): Your first hour, the defaults, every feature as What / How to run / Tips / How solid, the state machine's diagram and its gap, the two EKFs (they meet only in TF), the Python table. ⬜ its defaults table after TODO 88; its cockpit block after TODO 87 |
| 86 | **Everything tested in the simulator** (Ben: "I would like to see diffusion tested in simulation. Acutally everything"): sim-check, bench-check, every switch launched once, diffusion against the lattice 3 runs each | ✅ 3 Nov, `sim-all`: sim-check 9/9, bench-check 11/11, **23 of 23 switches** finished a leg, every sim-check option passes (`--lane` after a fix). Diffusion 12/12 legs, 73 % of plans from the model (2 Nov: 63 %), tours 177, 179 and 270 s against the lattice's 183.4 ± 0.7 ([trials](docs/trials.md)) |
| 87 | **The cockpit as the one driving screen**: the localization views and the running algorithms; screenshots from live Gazebo | ✅ 3 Nov, `cockpit-drive`: an **Algorithms** table from the new `aav_run_info` (`/system/algorithms`, filled from the launch arguments); NDT's and AMCL's poses, `/map`, KISS-ICP's speed; tabs for the localizers, the tracker swap, the switches; screenshot live in the test lot (`docs/img/cockpit-drive.jpg`). ⬜ not showing: the EKFs' arrows and NDT's scan (two layout scripts, 6 alerts to read at the Mac); NDT's map and KISS-ICP's cloud need publishers; frame labels clutter the car; the bench doesn't start `run_info` |
| 88 | **The best defaults**: one table, default where and why; sim winners switched only with their numbers | ✅ 3 Nov: no switch. NDT-EKF 0.018 m against 0.174 m, but needs the lot's map (not in git); the cubes slower and once drove into a post. Tables: `docs/switches.md`, the guide |
| 89 | **The ESP32 in QEMU** on Ben's Mac (`esp tools`, `esp qemu-check`): the heartbeat brake proven before A8 | ✅ 3 Nov: **PASS** at load 17. 6 of 6 cycles (silence, frozen, bad CRC): full brake and DAC 0 in 112-119 ms, latched until a stop; uplink 50 frames a second, 0 lost; RC via ROS, RC direct and a lost RC, 6 of 6. Found and fixed: the checker crashed on a log line QEMU cut short (a busy Mac); such lines are now counted and skipped |

## Monday 2 November: Ben's asks for 0.2

Ben, 2 Nov: "I would like the hud changes in, same with for the map. Something nicer then catpucine. We also want diffused planning, and joystick control ... We want to develop a list of tests." Words: `docs/history/messages-2026-11-02.md`.

| # | Task | Status |
|---|---|---|
| 80 | **The Initial D HUD in** (`hud-initial-d`) | ✅ 2 Nov: merged into `drive-plan` |
| 81 | **Initial D red and black on every screen**: HUD, map window, debug screen, hardware panel (Ben: in place of Catppuccin) | ✅ 2 Nov, `theme-initial-d`: the default on all four; its own map, grid and 3D colours, sharp corners; contrast tests for every mark, the e-stop 5.2 : 1. Screenshots in `docs/hud.md`. Seen with demo data only. ⬜ tests F.7, F.8 on the car's monitors, day and night |
| 82 | **The joystick on the Motor Pi alone** (TODO 74a): the joystick-alive check, so a dead pad stops the car | ✅ 2 Nov, `joy-alive`: `joy_alive` between `joy_node` and `teleop_twist_joy`; SILENT (0.5 s), FROZEN (2 s, deadman held, stick bit-identical), latched until the deadman is released; `/drivers/joy/alive`. bench-check 11 of 11. ⬜ car day: the real pad (A9b 6, 6b, 7; tests 3.J1-3.J3), tune `frozen_after`; (b) the throttle cap, (d) the button and axis numbers |
| 83 | **The diffusion planner in 0.2** (TODO 78; Ben, 2 Nov: "Put it in 0.2"), a trial beside the lattice, off by default, with the guard | ✅ 2 Nov, `diffusion-build`: `planner:=diffusion` (the car refuses it); 45,092 teacher paths, 3.9 M weights, ONNX, 75 ms on the CPU; guard G1-G8. Gazebo lot tour, 2 runs: 8 of 8 legs, 178-182 s (lattice 186-188 s), **63 % of plans from the model** (design: 90 %), the U-turn weakest. 0.3 (TODO 133, 8 Nov): the U-turn and G5/G6 fixed offline, the Orin ready to measure, the car gated. ⬜ the Orin, the car |
| 84 | **The list of tests**: `docs/tests.md` gets the new parts' tests (theme, joystick-alive, diffusion) | ✅ 2 Nov: 1.16-1.17 the screens, 1.18-1.19 diffusion, 3.J1-3.J3 the joystick, F.7-F.8 the monitors |

## Sunday 1 November: the ideas before the second PR

Ben, 1 Nov: "do all the ideas. I like them all but car day. Keep that in a list fo things todo".

| # | Task | Status |
|---|---|---|
| 55 | **The cubes in RViz** (`voxels:=true`) | ✅ 1 Nov, `ideas-1nov`: a "Cubes" display (0.25 m boxes) in `road.rviz` and `drive.rviz` |
| 56 | **The cubes and Betsi's health checks on the debug screen** | ✅ 1 Nov, `hud-cubes-health`: Nav2's cubes as red wireframes (key V), a "Nav2's cubes" section (road fit, count, ms) and "Localization health" (OK, or WARN and each check) on the seeing screen; rows in the sensor panel. Seen in `--demo`; untested live |
| 57 | **Health in `pixi run check`** | ✅ 1 Nov: one line from `/localization/health` at the end; a WARN never fails the check |
| 58 | **KISS-ICP into `ekf_local`** (`kiss_ekf:=true`, a trial) | ✅ 1 Nov, `kiss-ekf`: speed and turn rate only, covariance measured against Gazebo's truth. Not a clear win yet: ekf_local 0.30-1.27 m per 100 m on, 0.44-1.91 off, one run each on a busy Mac. Stays off. Next: 3 runs each on a quiet machine, then ekf_local's process noise; refit on a car bag |
| 59 | **The lattice as the default planner** (54b) | ✅ 1 Nov (Ben): everywhere that may reverse, the car too (TODO 62); Hybrid-A* forward only (the lattice lost the U-turn) |
| 62 | **Reversing and the U-turn on the car** | ✅ 1 Nov (Ben: "reverse and uturn should be in"): `reversing:=true` on the car again where the lidar sees behind; its planner the lattice (Hybrid-A* forward only). Tests: stands 3.2b, ground 4.23-4.26 (tests.md 4f). ⬜ car day: run them. Until H5, the speed's sign is the command's |
| 63 | **The ground plan: maps and scores from the first drive** | ✅ 1 Nov: [tests.md 4e](docs/tests.md#4e-record-the-lot-the-maps-and-the-scores) (record with the lidar, the wheel diameter, NDT's map, score the trials on replay) and the stack for every ground test, pinned by `test_car_launch.py`. `tools/record.sh --lidar`; `ndt-map build --poses` for a real drive. No `best:=true`: the best of the trials *is* the defaults |
| 66 | **The lab's Ubuntu desktop** (stays in the lab: CARLA, the Orin's full image) | ✅ 1 Nov: [car day session 11](docs/car-day.md#session-11-the-labs-ubuntu-desktop); Ben's MacBook is the ground station at the drive. ⬜ fill in: CARLA's version, Ubuntu's, the GPU; 11a the week before |

## Friday 30 October: the lattice, fairly

Ben, 30 Oct: "Reversing is always true i think".

| # | Task | Status |
|---|---|---|
| 54 | **Rerun the planners with reversing on** (hybrid, lattice, lattice32), 3 runs each | ✅ 30 Oct, branch `lattice-rerun`: every leg for all three; the lattice fastest, 184 s a tour against Hybrid's 213 s; 32 headings no better ([trials](docs/trials.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32)) |
| 54b | **Ben: make the lattice the default planner?** | ✅ 1 Nov, Ben: yes (TODO 59). Was: Ben decides. Before: the bench, and forward only (the car's `reversing:=false` until H5), where the lattice still loses the U-turn |

## Thursday 29 October: control (Ben, 27 Oct)

Ben: "investigate LQR and maybe collapse all these various controllers into a single
controller ... closed loop". The plan: `docs/designs/2026-10-27_control-plan.md`; the ideas: "Ideas to keep" below.

| # | Task | Needs |
|---|---|---|
| 1 | **A page that explains it all:** PID, LQR, MPC and MPPI, with pictures; which loop runs where (ESP32, Jrk, Orin); what each knows. The next chapter of `docs/learn-control.md` ("Control from zero") | ✅ 29 Oct: [Controllers](docs/controllers.md), after Learn: control. The old RPP-vs-MPPI page is now `docs/path-followers.md` |
| 3 | **LQR for speed and brake (LQI):** one signed acceleration, in `spine-core`; scored against today's PI and P-only brake | ✅ 29 Oct: built (`speed_loop:=lqi`, gains from `pixi run python3 tools/vehicle_model.py --lqi`); the PI stays the default. No overshoot in Gazebo's steps (today's 10-16 %), settles in 4-5 s (11-13 s); at 3 m/s it stops 0.2-0.4 m past B (the PI 0-0.4): [Controllers, 2a](docs/controllers.md#2a-lqi-against-the-pi-results-29-oct). Also the bridge's smooth stop (Autoware's design): the jolt at standstill halves (2b). Left: a ramp feed-forward for the LQI's stop; nothing tried on the car |

Every gain still comes from guessed numbers until the car is measured (`docs/measure-the-car.md`).

## 24 October, night: the car as a linear model

Ben, 24 Oct: "Do we have a mathematical model for the car now for the ego? ... a matrix
control thing ... is that something we should consider? ... Maybe second order system".

| # | Task | Status |
|---|---|---|
| 52 | **The car's linear model**, in SYSC 3600's terms ([The car as a linear model](docs/vehicle-model.md)) | ✅ 24 Oct, `wt/vehicle-model`. `aav_ego` `vehicle_model.py` (no ROS; numpy, scipy.signal): speed K/(τs+1) with the throttle lag and the spine's PI (poles −0.28 ± 0.28j: settles in 13 s); the steering as a second-order system (ωn 12.6 rad/s, ζ 1.03); the bicycle linearised, x = [e_y, e_θ, δ], with linearised pure pursuit (ζ 0.707 at every speed); Bode margins with the delays (lateral, 1 m/s: 52° none, 45° typical 84 ms, 31° worst 259 ms); the discrete loops at 100 Hz and 20 Hz. 27 + 3 tests. `pixi run python3 tools/vehicle_model.py`: the tables, `docs/img/vehicle-model-*.png`. Found: the guesses disagree (drag 0.1 says full throttle holds 15 m/s; the open loop's `FULL_THROTTLE_SPEED` says 3); Gazebo's `steer_p_gain` 10 adds a steering lag after the actuator model: measured in Gazebo (headless, `steer-rate --sim` and `/joint_states`), its wheels follow the model's angle with a 0.110 s first-order lag (0.5 mrad rms) |
| 52c | Make Gazebo's steering match `vehicle.yaml` | ✅ 25 Oct, `wt/goals-steering`. Gazebo's wheels now follow the actuator model's angle with a **0.022 s** lag (was **0.111 s**; 2 runs each, 0.4-0.5 mrad rms left; a third on a busy Mac 0.063 s: the adapter's ticks stalled). Found: raising `steer_p_gain` alone made it worse (0.07-0.16 s): the URDF's steering joints had a 1.0 rad/s speed limit, which Gazebo enforces (the wheels turned at 0.63 rad/s, under the model's 0.8). Fix: the joints' limit 1.0 → 5.0 rad/s, `steer_p_gain` 10 → 100. The lag left is 1/gain + about 12 ms from the adapter's 50 Hz hold. Chose gain 100 over "the adapter's lag less 0.1 s" (11 ms off): `/vehicle/steering` stays the wheels' angle, and a new `steer_lag` needs no change in the xacro. Lateral phase margin in Gazebo 43° at 1 m/s (was 37°; the model 45°). `tools/gazebo_steering.py` measures it (with `actuator-test steer-rate --sim`); `test_vehicle_model_tool.py` keeps the extra lag under a quarter of `steer_lag`. `sim-check`: the scripted drive came up short (2.3-3.8 m of 6.5) with main's settings as with these, on a Mac at load 24 (other simulators): not this change. Next: the same step on the car (T2.7) |
| 52d | **The state-space primer** ([Learn: state space](docs/learn-state-space.md)), for Ben's SYSC 3600 refresher | ✅ 25 Oct, `wt/learn-state-space`. The 12 steps (why, free-body diagrams, Newton, states and A B C D, simulating, transfer functions, poles = eigenvalues, Bode and delay, what it means at 1 vs 3 m/s, shocks and roll, LQR and Kalman, a cheat sheet). `pixi run python3 tools/state_space_demo.py` (`tools/state_space_demo.py`, 16 tests): Euler, RK4 and exact side by side; a quarter-car and a roll model with GUESSED springs. Roll at 3 m/s on full lock: 2°, 46 % of the inside wheels' weight moves out, 0.32 g against a 0.70 g rollover limit: not a control problem; matters for the lidar's tilt. The kinematic bicycle holds to 3 m/s (the tyres' poles 4× faster than the steering's) |
| 52e | **Body roll tilts the lidar** (found by the roll model, [Learn: state space](docs/learn-state-space.md) section 10): at 3 m/s on full lock about 2° of roll, so the ground about 0.7 m off at 20 m, and could show as an obstacle in hard turns. Nothing to do at today's 1 m/s cap. Before raising it: check Nav2's obstacle layer's height filter against a rolled scan (Gazebo has no suspension, so it can't show this: a replayed bag from the car), or tilt-correct the cloud with the IMU's roll | ✅ 25 Oct, `wt/ekf-heading`: **safe up to 4 m/s, no change needed; today's 1 m/s untouched.** The ground filter is `lidar_scan`'s 0.3 m above the road (level body), marked out to 12 m (`lidar_layer` `obstacle_max_range`); collision_monitor's boxes are within 1 m of the bumpers, too close for the road to rise. `aav_lidar` `ground.py` (no ROS, 9 tests): where each VLP-16 beam meets a rolled road. The road shows above 0.3 m within 12 m from **1.46°** of roll. ego's 2 m/s² sideways limit holds the roll to **1.33°** at most (the springs' overshoot included, a sudden full lock from 1-4 m/s through ego's own model): the road at most 0.23 m up, 10 m out. At 5 m/s a sudden full lock overshoots (3.9 m/s², 2.4°): ego brakes 2 m/s² but the wheels turn at 0.6 rad/s. `tools/tests/test_lidar_roll.py` ties vehicle.yaml, ego.yaml, nav2.yaml and the roll model together. Not done: levelling the cloud with the IMU's roll, a trap: madgwick reads 2 m/s² sideways as 11.5° of tilt ([Costmaps](docs/costmaps.md#body-roll-and-the-lidar-todo-52e)). Left: springs and CG height are guesses: softer than ~7400 N/m a corner and the margin goes (measure day: the spring rates and the CG height, M7b) |

## 24 October, evening: the car-day kit

Ben, 24 Oct: "Lets get everything prepared so the only thing left it to get into the car."

| # | Task | Status |
|---|---|---|
| 51 | **The car-day kit**: one runbook, one preflight | ✅ 24 Oct, `wt/car-day`. [docs/car-day.md](docs/car-day.md) (in the docs as "Car day"): sessions 0-10, each with who, how long, what to bring, the commands, what you should see, what to do if not, and 🔒 where the hardware lead must answer first. `pixi run car-day-build` builds the ESP32 firmware (version 2 and `link-v1`) into `car-day/esp32/` (out of git), runs `esp qemu-check`, builds the C++ way back and the docs PDF; `pixi run car-day-check` (ROS-free, 35 tests) says OK / MISSING with the fix, or REMIND. Built on Ben's Mac: both ELFs, QEMU PASS (111-119 ms), the C++, the PDF, and `models/rf-detr-nano.onnx` (108 MB: the Orin then needs only TensorRT's `trtexec`). Found: on JetPack 6, RF-DETR can't run inside the stream node under Pixi (Python 3.12 vs JetPack 6's 3.10 TensorRT): the streams run, the detector waits for the reflash (Q24). Also fixed: every `!!!` box in the HTML docs showed its text as code (four-space bodies with `tab_length` 2). Seen once: `esp qemu-check` in `tools/test.sh` failed at load 18 (QEMU starved: the firmware's clock ran 0.82 s per real second, timeouts of 2.3 s); it passed alone a minute later and in the next full run. A flake under load, not the firmware: QEMU has no `-icount` |
| 53 | **The software on 64-bit ARM Linux, the car's computers, before the car day** (PLAN T9.2) | ✅ 25 Oct, `wt/ci-arm`: two CI jobs on GitHub's free ARM runner (`ubuntu-24.04-arm`, Ubuntu 24.04, 4 cores), on every push to `main` and every pull request. **`arm`** (11-12 min): `pixi run test` passes (1056 tests); the HUD and the map window (`aav_hud`) are built as aarch64 programs, and each draws its demo once on a virtual screen (125 KB and 106 KB pictures; the artifact `arm-screens`). **`car-kit`** (6-7 min): `pixi run car-kit`, then `car_install.sh --all` with no network at all (a network namespace of its own): every package built offline from the kit, the HUD too, in 4.5 min. Fixed on the way, CI only: under `sudo`, the runner's `PSModulePath` came back and colcon chose PowerShell. Still untested: the Orin's and the Pis' own Ubuntu 22.04 and JetPack |

**Ready for the car** (the steps are written; each still needs the car):

|---|---|---|

## 23 October: what we can do without the car

Ben, 23 Oct: "yes lets use cyclone and lets add this to a list of stuff we did and the
justifications for it", and the list below. Words: `docs/history/messages-2026-10-23.md`.
The reasons for each choice: [decisions](docs/decisions.md#decided-on-22-and-23-october).

| # | Task | Status |
|---|---|---|
| 28 | Cyclone DDS the default on every computer | ✅ `pixi.toml`, `tools/ros.sh`, `pixi run doctor`, the docs. To check at the car: it uses one network card |
| 28b | Cyclone's limit of 32 ROS programs per computer (found 23 Oct on Linux: Gazebo's 33rd, the sensor watchdog, failed) | ✅ `tools/cyclonedds.xml` raises it to 120. oirp-spss: 45 test programs, 13 refused before, 0 after; `sim-check --obstacle`, `--spine`, `--plan-around` pass, `--faults` 24 of 25 (`odom freeze`, the known flake: fixed, task 40); colcon's 4 failures there are the HUD it can't build (no libclang) |
| 29 | The GPS antenna over the rear axle in `vehicle.yaml` (x = 0) | ✅ its height still to measure (S1) |
| 30 | The wheel sensors: an Arduino sends CAN frames, the Orin decodes them (PLAN D2) | ✅ decided by the hardware lead (passed on by Ben, 23 Oct); the frames are `aav.dbc` 0x200, 0x201 |
| 31 | Fix the nightly Gazebo test (failed on its first night, 23 Oct) | ✅ merged: software graphics on GitHub, and the scripted drive keeps the simulator's time (task 23). First GitHub run: the night of 23 Oct |
| 32 | The 10-minute timing test (task 11), Fast DDS against Cyclone | ✅ run 23 Oct, 21:00, a quiet Mac. **Cyclone: clean** (worst IMU gap 0.06-0.08 s, nothing over 0.5 s late, 0 stalls, 0 watchdog stops). **Fast DDS:** IMU gaps up to 1 s, camera pictures up to 4.5 s late, 490 moments parked with 3 or more key topics silent at once (the longest 2.4 s), the watchdog stopped Nav2 5 times. The decision holds. Two tool bugs, ✅ fixed 23 Oct: stalls chained overlaps into one (a "236 s stall"): now only topics silent at the same moment count; and the drive, forward only, ended facing a wall after the U-turn and re-sent goals thousands of times: now it reverses, as the car may, and stops after 4 failures in a row. The driving half isn't rerun (Ben agreed: not needed for the decision). Reports: `log/timing-test/` |
| 33 | The HUD heartbeat to the safety watchdog (PLAN T9.8) | ✅ merged: `/hud/heartbeat` at 10 Hz while the screen draws; silent 1 s, Nav2 and the AI stop (not latched); on the car by default (`watch_hud:=false` without a HUD). Questions for the team: latch it? the Flutter app and map window? (the Flutter app: archived 25 Oct) Found: `sim-check --faults` `odom freeze` flaky (fixed: task 40) |
| 34 | The sensor mux and output mux (PLAN T1.5, D3, S-4) | ✅ merged: `aav_io_mux`; backends read only `/control/<target>/actuator_cmd`, filled only while every source is REAL ([sources and targets](docs/sources-and-targets.md)). Questions for the team: "check, not copy" (D3); a bench with a real spine and pretend sensors; the HUD showing the gate; `spine_bridge` refusing to arm while it's shut |
| 35 | The CAN bridge (`aav_spine`) in Rust (PLAN T1.8) | ✅ merged: `aav_spine_bridge`, the default (`bridge:=python` for the old one); reads the output mux's topic, and the guard test runs against both. Untested: the Orin with real SocketCAN |
| 36 | A bandwidth budget: every sensor's rate and size, each link's load, the CAN bus load (PLAN T1.7) | ✅ merged: [bandwidth](docs/bandwidth.md), `pixi run measure bandwidth`, `pixi run python3 tools/can_load.py`. Raw cameras are the only risk; CAN 8.5% |
| 37 | **Compress the cameras** (Ben, 23 Oct: "Lets compress it"; [bandwidth](docs/bandwidth.md)) | ✅ **software done, merged 24 Oct, as 37b**; at the car: car-day session 5. (MediaMTX H.264 on the Pi, decoded on the Orin; [camera pipeline](docs/camera-pipeline.md)); on the car: untested. Before: Today: the camera Pis (Pi 5) run **MediaMTX**, RTSP on 8554: cam1 the Pi camera, cam2 `ffmpeg` from the USB camera (640x480, 30 fps); not ROS (audit 21 Oct). Plan: keep H.264 on the Pis (the Pi 5 has no hardware encoder: x264, low-latency settings), keep MediaMTX as the server; on the Orin, GStreamer with NVIDIA's hardware decoder (`nvv4l2decoder`) turns RTSP into ROS images on the Orin only, so raw pictures never cross the network; the HUD and laptops watch MediaMTX's stream (WebRTC/RTSP) directly. In Gazebo: `image_transport` compressed. Measure latency and size with `pixi run measure bandwidth`. **Ben, 23 Oct, evening:** both front cameras are **ribbon, on one Pi 5** (Ben, 23 Oct: "we have 2 ribbin cameras hanging off one pi"; the Pi 5 has two camera ports; one `camera_ros` per camera; the same clock stamps both, good for the stereo pair; check the audit's CAM2-PI with its USB camera) (CSI: libcamera, `camera_ros`; no MJPEG from the camera, so the Pi 5 encodes in software: measure its CPU); the Orin is **probably an AGX** (hardware encode and decode; to confirm); a **second, lower-end Orin** is coming (PLAN T1.9). **Suggested pipeline (23 Oct, to write up as `docs/camera-pipeline.md`, Proposed):** for the computers, ROS compressed JPEG from `camera_ros` on each Pi (capture time stamped, bag-recordable, the same in Gazebo) → on the Orin, one process per camera: decode (nvJPEG hardware if the CPU is short) → undistort (OpenCV) → YOLO in TensorRT → small detection messages. **Revised again (Ben: ROS cameras, or just stream both to the Orin with MediaMTX?): lean to the stream.** The camera Pi runs Debian 13 with no ROS, and MediaMTX works there today: it owns both cameras (H.264); on the Orin, one node per camera pulls the RTSP stream, decodes it in hardware (GStreamer `nvv4l2decoder`), stamps it, and feeds YOLO in the same process; people watch the same stream in a browser (WebRTC). **Three cameras** (Ben, 23 Oct: "2 cameras on one and one camerea on qnx"): the front pair on one Pi, one camera on the QNX Pi (probably the audit's CAM2-PI, .118, USB 640x480, Debian 13 now: check; likely the rear camera). Ben, later: "There is 2 ribbin cables on the pi that is linux. the qnx doesnt' work": the front pair is ribbon, on the Linux Pi; the QNX Pi and its camera don't work today, so the car has no rear camera for now (reversing already counts only the lidar as seeing behind). Ribbon or USB is not certain (Ben: "it could be on a usb cable. who knows"): at the car, `rpicam-hello --list-cameras` (ribbon) and `v4l2-ctl --list-devices` then `v4l2-ctl -d /dev/videoN --list-formats-ext` (USB: if it offers MJPG or H264, the camera compresses and the Pi does nothing). Measure first: the Pi's CPU for two H.264 streams, the delay end to end, and the time stamp's error. Earlier idea: only one program can open a ribbon camera, so one owner per Pi: `camera_ros`, everything in ROS. People get a small preview topic (lower size and rate, or H.264 through `ffmpeg_image_transport`, which Foxglove/Lichtblick show) instead of MediaMTX; keep MediaMTX only if someone needs a browser view without ROS. GStreamer only where the hardware needs it; Isaac ROS (NITROS, zero copy) as the upgrade. Not OpenCV's DNN module for detection |
| 43 | **The control board's bridge refuses to arm while the safety gate is shut** (Ben, 24 Oct: yes) | ✅ built 24 Oct, `wt/arm-gate`. `/spine/arm` refuses unless `/safety/output_mux/state` said `OPEN` for the bridge's target (car, bench, gazebo) within `gate_timeout` (1.5 s), with the reason; both bridges (`spine-bridge` `gate.rs`, `aav_spine/output_gate.py`). The bench and Gazebo's `arm:=true` now start `arm_when_open`, which asks each second until OPEN. `legacy_uart_bridge` has no arm step. Also fixed: `output_mux`'s clear logged at two severities from one line, which rclpy refuses ([Arming waits for OPEN](docs/sources-and-targets.md#arming-waits-for-open)) |
| 43b | The guard test's SIM case let 13 DRIVE_CMD frames through once on main (load 29): cross-talk or a start-up race? | ✅ 24 Oct, `wt/arm-gate`: **cross-talk**. Our `sim-check --spine` (domain 193) was latched that day by another run's REAL `spine_bridge`; the SIM case passed 16 of 16 at load 24-42 and the whole guard file 44 of 44 at load 10-48, with 24 busy processes; the announcement and the graph agree within 1 ms in one process. Fixed: the guard test, `sim_check.py` and `bench_check.py` take a domain nobody uses (`aav_io_mux/domains.py`), and the test fails naming any other run's nodes. The race's small window (across processes the graph up to 30 ms before the message) closed too: a node on `/system/source` not heard yet holds the gate, and an announcement counts before its node shows in the graph. New tests: Gazebo starting late (frames stop within 1 s), and an announcer seen before its announcement |
| 39 | GitHub's Linux tests failed twice on 23 Oct (`6f801dfc`, `4bed1125`), then passed on the next push with the same code: flaky, **in the build**, not a test (no failing test case; the log's tail only "Starting >>>") | ✅ not seen again: 0 failed pushes to main since `4bed1125` (40 runs to 25 Oct). `ci_annotate.py` now names the failed package and its error lines on the run's page. Next failure: read it there, then fix the cause (a guess: a download during the build, such as crates.io for the Rust packages) |
| 42 | The Mac nightly hung in `pixi run test` on 24 Oct (started 11:50 UTC, cancelled at the job's 90 min limit, nothing to read; 23 Oct's took 12 min). Linux CI and the Gazebo job passed on the same commit (`1ff7f587`); on this Mac all tests finish in 41 s, with local and with subnet discovery | ✅ 25 Oct, `wt/ci-arm`. **Cyclone DDS at a test's end, on the network**: aav_routes' tests all passed, then freeing the last ROS node (`dds_delete`) waited for ever to join Cyclone's multicast reader, which stops only on a packet the computer sends itself; on GitHub's Mac that packet never comes. Found by a probe (native stacks with lldb, Python's with faulthandler, as annotations; commit `82d9eda4`); every GitHub Mac run hung so since Cyclone became the default (23 Oct): the 24 Oct nightly, then 3 of 3 on this branch. pytest's `faulthandler_timeout` never fired: the tests had ended. Fixed: `pixi run test` and `tools/test.sh` keep every test on this computer (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: unicast on the loopback, no multicast reader), and the street router test takes a domain of its own. That showed cross-talk (two packages' tests side by side on one domain heard each other), so colcon tests one package at a time: 1 min 55 s, not 1 min 41. Then the heartbeat chain test wanted 30 of 50 frames in a second and that slow VM wrote 23: it wants 10 now. The Mac job then passed: 18 min, its tests 15.5 |
| 40 | `sim-check --faults`' `odom freeze` failed most runs since 22 Oct (Mac and oirp-spss) | ✅ 24 Oct, `wt/odom-freeze`. The cause: **MPPI** (Nav2's default since 22 Oct) plans from the wheels' speed, so frozen wheels keep the car only 0.1-0.2 m/s faster, under the GPS part's 0.3 m/s (with `controller:=rpp`: 4 of 4 caught). The watchdog's GPS part now fits 10 s, within 0.15 m/s (normal driving's worst: 0.04-0.08); the check freezes the wheels at 0.2 m/s and counts them wrong by the watchdog's limits. `--faults`: 25 of 25, three runs out of three ([frozen wheels and MPPI](docs/simulation.md#frozen-wheels-and-mppi)). Still to do on the car: the GPS's real noise over 10 s |
| 41 | `aav_camera_stream`'s `test_reader_restarts_an_ended_stream` is flaky (found 24 Oct: 1 in 5 alone on the Mac, and once in `pixi run test`): `os.killpg` in `decoders.py` `stop()` gets `PermissionError`, so the reader starts only twice | ✅ fixed 24 Oct: on macOS `os.killpg` says EPERM, not ESRCH, for a group whose last process has exited but not been reaped; the decoder's stop now treats both as "gone". 10 of 10 passes |
| 37b | **The camera stream, concretely** — ✅ **decided by Ben, 23 Oct**: MediaMTX on the Pi, compressed, to the Orin; OpenCV and detection there ([decisions](docs/decisions.md#decided-on-22-and-23-october)). OpenCV: which version JetPack 7.2 ships, and whether with CUDA and GStreamer, is **not checked** (JetPack 6 shipped 4.x without CUDA). **Ben, 23 Oct: "if htat is opencv 4 then thats what we use"**: JetPack's own OpenCV, whatever version; no own build. OpenCV only does picture work, so CUDA in it matters little. **The detector: RF-DETR Nano** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one"; [newer models](docs/model-comparison.md): the stop sign from 20 m, YOLO11n 10 m; Apache-2.0, not AGPL), in TensorRT, not OpenCV's DNN module. ✅ RF-DETR Nano is the default since 24 Oct (`aav_detector/detectors.py`, YOLO with `detector:=yolo`; Mac: Core ML 10 ms, `sim-check --perception` stopped 4.4 m before the sign). Left: build the TensorRT engine on the Orin (`tools/build_engine.py`) and time it (Ben: "we just need to figure out how to stream and encode it over the network and then what to do with it on the other side") | ✅ **software done, merged 24 Oct; GStreamer on Macs 25 Oct (27 ms)**; at the car: car-day session 5. Was: built 24 Oct, measured on the Mac; untested at the car. ([camera pipeline](docs/camera-pipeline.md)). `aav_camera_stream`: the Pi's `pi/mediamtx.yml` (MediaMTX itself loads it in a test), one `camera_stream` node per camera (decoder as a child program: GStreamer `nvv4l2decoder` on the Orin, ffmpeg or GStreamer on a Mac; OpenCV undistortion; `aav_detector`'s RF-DETR Nano in the same process; detections, a 4 kB preview a second, `/diagnostics`, `/system/source`). `pixi run -e ml measure stream` (Gazebo → OpenH264 → local MediaMTX → node): **26 ms** camera-to-detections with GStreamer, 160 ms with ffmpeg (its decoder holds 2 frames); 2.0 Mbit/s for a noisy picture; stop sign seen from 25 m in Gazebo. To do at the car: install the Pi file, the Pi's CPU for two streams, the delay and stamp error (a stopwatch), `delay` from it; calibrate the ribbon cameras; a `car.launch.py` part for the Orin. The plan as it was: **Pi:** MediaMTX owns each camera: ribbon via its `rpiCamera` source (software H.264 on a Pi 5), USB via `ffmpeg` (pass MJPG/H.264 through if the camera has it); 1280x720, 15 fps, about 2 Mbit/s, a key frame every second, no B-frames, low-latency settings. Config in the repo. **Network:** RTSP, port 8554, wired. **Orin:** one node per camera: GStreamer `rtspsrc ! rtph264depay ! h264parse ! nvv4l2decoder` (NVIDIA hardware; `avdec_h264` or `vtdec` on a Mac), into OpenCV (undistort) and YOLO in TensorRT **in the same process**; publishes detections, plus a small preview now and then. Stamp = arrival minus a measured delay (chrony on the Pi too). NVIDIA's GStreamer plugins are JetPack's own, not Pixi's: the node uses the system's GStreamer on the Orin. **Test without the car:** Gazebo's camera → `ffmpeg` → a local MediaMTX → the node → detections, on a Mac; measure the delay |
| 37b.1 | **RF-DETR Nano is the detector, not YOLO** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one") | ✅ merged 24 Oct. `aav_detector` runs RF-DETR Nano by default (since 20 Oct), YOLO with `detector:=yolo`. New: the models as plain Python objects, no ROS (`aav_detector/detectors.py`: `load_detector()`, `detect(bgr)`), for the camera-stream program; RF-DETR's **Core ML** export (`pixi run -e ml python3 tools/export_model.py --format coreml`: 10 ms a picture on Ben's Mac, 16 on Metal) and its **TensorRT engine** (`python3 tools/build_engine.py` on the Orin: `.pth` → ONNX → FP16 `.engine`, then a test run); `orin.yaml` loads `models/rf-detr-nano.engine`. `sim-check --perception` passed with RF-DETR (first seen at x = 5.9 m, the sign at 25 m; stopped with the bumper 4.4 m before it). **Untested: everything on the Orin** (needs the reflash, Q24): build the engine, time it |
| 38 | **Match the plan to the car's wiring** (Ben, 23 Oct, evening; to confirm with the hardware lead) | ✅ merged 24 Oct; the wiring is **as Ben understands it, the hardware lead confirms** ([today's wiring](docs/can-plan.md#todays-wiring-23-october)). **Actuators** (throttle, steering, brake), maybe the steering-angle and brake-position sensors, on the **ESP32**, through the **Motor Pi** (probably UART); the **CAN bus straight to the Orin**, with the wheel sensors' Arduino and a few others. Built: `spine_bridge role:=sensors` (Rust and Python): no `/can/tx`, no `/spine/arm`, no e-stop path, ignores the spine's frames; `/vehicle/odom` from the two wheels (no yaw rate) and `/vehicle/wheels`; announces only odometry and wheels. `car.launch.py actuators:=uart` (the default): `legacy_uart_bridge` on the Motor Pi drives, the Orin's `spine` part reads CAN for sensors (no CAN sender); `actuators:=can` keeps the CAN spine. `orin.yaml` and `all.yaml` turn the spine part on; the 2025-26 UDP encoder is off (two `/vehicle/odom` would stop the output mux). Tested: the guard test (both languages; no frame on `/can/tx`, the mux opens), launch tests, the Rust node fed wheel frames (1.0 and 1.2 m/s gave odom 1.1). Untested: SocketCAN on the Orin, the Arduino's frames. The Rust ESP32 firmware's 100 ms timeout is on the UART link (Gate 2). Questions for the hardware lead: [audit questions H1-H8](docs/audit-questions.md#8-the-cars-wiring-for-the-hardware-lead-23-oct) |
| 43a | **Foxglove and Lichtblick built in** (Ben, 11 Oct: "at some point"; go-ahead 24 Oct) | ✅ 24 Oct, branch `wt/foxglove` ([Watch from a laptop](docs/foxglove.md)). One bridge launch (`foxglove.launch.py`) for sim, bench, CARLA and the car: stand-ins on 127.0.0.1, the car on its network and watch only; a WiFi topic list without raw cameras, full lidar or costmap layers; the camera preview on the stand-ins too. Three layouts in `tools/foxglove/` (driving, sensors health, map); the sensor watchdog's diagnostics now carry each sensor's rate and gaps. Tested live: road sim and bench, every layout topic served, 1.4 Mbit/s for all three, a goal sent through the bridge drove the car. To do: open the layouts in the app by hand; at the car, `pixi run python3 tools/layout_check.py ws://<Orin>:8765 --data`; offline map tiles for the map panel |
| 44 | **Fix the HUD** (Ben, 24 Oct: "fix the hud?", "go"): (a) the HUD shows the safety gate; (b) a HUD that keeps crashing holds Nav2 until a person clears it (PLAN T9.5, T9.8) | ✅ merged 24 Oct (`wt/hud-gate`). (a) A **GO / STOP panel** under the speed: green GO, amber WAIT, red STOP or AUTO STOP, grey NO GATE, an icon, and the one main reason (E-STOP ON; the output mux's, such as "GPS is SIM"; the watchdog's first input that stops, such as "GPS STALE") ([HUD](docs/hud.md#may-the-car-move-the-go-stop-panel)); logic in hud-core `gate`, tested; **not yet seen on screen** (the Mac's lid was shut: no window). (b) **Latched**: one dropout within 5 s of the HUD's first beat is forgiven, any other is `hud LOST` until `pixi run clear watchdog` ([decisions](docs/decisions.md#decided-on-24-october)); new tasks `clear watchdog`, `clear gate`. `sim-check --faults` 25 of 25 (24 Oct, Mac): `hud stop` stood still 2.4 s after it, "hud LOST", still held 5 s after the beats came back, drove again once cleared; its first run found `watchdog hang` latched the HUD (Gazebo's clock catches up in steps after a stall): fixed, stalls are now seen on a steady clock |

## 24 October, afternoon (Ben's message: `docs/history/messages-2026-10-23.md`)

| # | Task | Status |
|---|---|---|
| 45 | **The ESP32 brakes on a lost heartbeat** (Ben: "if it doesn't get a heartbeat then it applys the breaks and stops the acceleration"). Ben confirms the wiring: Orin → network → Motor Pi → serial → ESP32 → throttle and the steering and brake motor controllers; CAN only the wheel encoders and the few listed; IMU, GPS, lidar on the network | ✅ **software done, merged 24 Oct, proven in QEMU**; at the car: car-day session 6. This is the Rust ESP32 firmware's 100 ms command timeout (`spine-core`, the UART build; `docs/esp32-bench.md`): no fresh command from the Motor Pi within 100 ms → full brake, throttle 0, latched until re-armed. Today's ESP32 C++ firmware has none (audit 16 Oct). **Proven in QEMU, 24 Oct** (`pixi run esp qemu-check`; [the emulator](docs/esp32-bench.md#before-the-board-the-emulator)): the real firmware, fed `aav_legacy`'s own frames, brakes 111-120 ms after the last one (brake Jrk 3500, DAC 0, fault bit 1) and stays braked until a stop frame; the Jrks and DAC are stand-ins there. It also found the firmware never turned the FPU on (fixed). Next: flash it on a spare ESP32 on the bench, the same check with the real chip (Gate 2, 27 Nov), then on the car with the hardware lead. Every link in the chain then fails safe: the Orin's watchdog, the Motor Pi's bridge, the serial cable, the ESP32 |
| 47 | The watchdog checks the camera program's health, not raw pictures (Ben: "that seems like a good idea") | ✅ already done 24 Oct (TODO 37c): the watchdog watches each stream's detections (sent for every picture, ~15 Hz, even with no boxes): quiet 1 s → Nav2 stops. Detections, not `/diagnostics`: those come once a second, averaged over 5 s, so a lost camera would be caught seconds later |
| 48 | **The RC remote and the joystick through ROS, with direct drive kept for testing** (Ben, 24 Oct: "allow for both ... eventually we will move that to ros network") | ✅ **built and merged 24 Oct, Proposed** (off until H9 and H10); at the car: car-day session 6e. Was: Proposed and built 24 Oct (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct); a Proposed row in `docs/decisions.md`): with `--features uplink`, CH6 reads three positions: low computer, middle (1301-1700) **RC via ROS**, high **RC direct**. RC via ROS: the ESP32 sends the sticks up and drives only from the Pi's frames; `aav_rc`'s `rc_driver` turns `/vehicle/rc` into `/drivers/rc/cmd_vel` (twist_mux priority 95). Losing the RC in either RC position brakes (fault 256) and latches until a stop. QEMU (fake iBUS on UART2): all six RC steps pass; a ROS test: uplink sticks → `/drivers/rc/cmd_vel`. Left for the car: H10 (is CH6 a three-position switch? the receiver's failsafe), [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design, then the hardware lead. Today: the RC transmitter → iBUS → the ESP32 drives directly (switch channel 5); the USB joystick → the CORTEX Pi → ROS already (`joy_node`; ours: `aav_teleop` → twist_mux). Plan: a transmitter switch with three positions: **computer** (ROS drives), **direct** (the ESP32 drives from the RC, no computer: for testing, and the fallback), **RC via ROS** (the ESP32 sends the sticks up the serial link; a ROS driver `/drivers/rc/cmd_vel` at people's priority in twist_mux, through ego's limits). Needs an uplink frame ESP32 → Pi (the status the Pi doesn't get today). Direct stays until RC-via-ROS has been proven on stands |
| 50 | **The safety code lives on the ESP32; the serial link triggers it** (Ben, 24 Oct: "thats the place to put the safety code ... some kind of communication system that triggers it") | ✅ **built and merged 24 Oct**; at the car: car-day session 6 (branch `wt/orin-heartbeat`; [the heartbeat](docs/rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)): the output mux's counter on `/control/<target>/heartbeat`, relayed by `legacy_uart_bridge` into a version 2 frame (17 bytes, CRC-8/SAE-J1850), and the Rust ESP32 ignores any frame whose counter didn't change. QEMU: brakes 111-120 ms after the counter freezes (the Pi still sending at 50 Hz) or the CRCs go bad, latched until a stop. The 13-byte frame stays (`link-v1`; the car's `legacy.yaml` says `frame_version: 1` until the Rust is flashed). **The uplink, built 24 Oct** (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct)): a 35-byte frame ESP32 → Pi on TX1 (GPIO17), 50 Hz, the same CRC-8: mode, mode switch, fault bits, the echo of the last Orin counter, the Jrks' feedback positions (unknown when a Jrk has no feedback set), the outputs, and the RC sticks; shared Rust/Python vectors. `legacy_uart_bridge` (`uplink: true`) publishes `/vehicle/status`, `/vehicle/steering`, `/vehicle/brake` and `/vehicle/rc`; with `uplink: false` (the default) nothing changes. Behind the firmware feature `uplink` until the hardware lead confirms TX1's wire (H9). QEMU: 50-51 Hz, the echo right in all 6 cycles, 0 frames lost; the check now passes on a loaded computer too. Untested on a real ESP32 and Pi: [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design agreed in principle (it's the spine design: `docs/rust-firmware.md`). **Ben, 24 Oct: the heartbeat must come from the Orin and travel to the ESP32; the Pi could be off the network and still be sending.** So: the Orin's output mux (the last Orin node, which is shut whenever the safety layer says so) puts a counter in every command; the Pi only relays it, never makes one; the ESP32 brakes if the counter hasn't changed for 100 ms, however many frames the Pi sends. Today's 13-byte frame can't carry it well: a sum checksum, no counter (a stuck Pi resending the same frame would look alive), no arm or e-stop bit, nothing back up. **A version 2 of the serial frame**, as the CAN plan's DRIVE_CMD already has: a counter that must change, armed and e-stop bits, a CRC-8; and an uplink frame ESP32 → Pi (mode, faults, brake and steering positions from the Jrks, RC sticks for TODO 48). The ESP32 enforces its own limits whatever the Pi sends (speed cap, steering rate, braked at boot). Keep the 13-byte frame as a fallback until v2 is proven on the bench |

## Where we stopped (15 Oct, morning)

Everything from the night of 14-15 Oct is on `main` and pushed. GitHub's Linux CI is green
again. The three branches that waited for Ben are merged (Ben, 15 Oct: "These are
important"), on the branch `merge-three-branches`, for him to bring into `main`:

| Merged 15 Oct | From | Still open |
|---|---|---|
| CAN v0.2.0: wheel frames 0x200/0x201, 100 ms timeout; `wheel-core`; wheel speed in `/vehicle/odom`, per wheel on `/vehicle/wheels` | `can-v2-work` (`wt/can-v2` was an older copy) | ~~the wheel node boards (`boards/esp32s3-wheel`)~~ *dropped 23 Oct: the hardware lead's Arduino sends 0x200/0x201* |
| The ESP32 control board's firmware in Rust (today's UART link, CAN optional) | `wt/rust-esp32` | never flashed: `docs/esp32-bench.md` first, with Ben's go-ahead |
| One reverse speed (0.5 m/s, `vehicle.yaml`); Nav2 may reverse with `reversing:=true` (Gazebo, the bench), never on the car by default | `wt/ackermann` (its unfinished part finished) | a goal behind the car, tried in Gazebo. *Since 22 Oct the car may reverse where the lidar sees behind it (task 24)* |

**Found overnight, for Friday's audit:**
- Today's ESP32 firmware has **no command timeout, no e-stop input and no watchdog**, and an
  unprotected web update page.
- SparkFun's ESP32 MicroMod schematic shows its **CAN pads not connected**: check that the
  control board's CAN chip (U7) can reach the ESP32 before ordering the KEFC CAN parts.
- The front cameras are ribbon (CSI); our driver `usb_cam` is USB-only: `camera_ros` instead.
- A lidar is on the roof; we have a simulated one (`lidar:=true`) but no driver. *Since 20 Oct: the stock velodyne driver.*

**Also landed on 15 Oct (after the summary above):**
- the two Pico 2 boards and the radar clustering in Rust; no C++ left in `src/` (now `ros/`);
- Friday's plan, `docs/history/friday-2026-10-16.md`, and every hardware question, `docs/audit-questions.md`;
- the offline install kit for the car: `pixi run car-kit` (1.8 GB, untested on ARM);
- `max_speed:=` caps ego at launch, for the first drive on stands;
- plain-English driving with a local model (`pixi run -e llm talk`, Qwen3-4B), Gazebo only:
  `docs/plain-english.md`;
- better open-source models, including NVIDIA's per-pixel CitySemSegFormer, in
  `docs/perception-and-hud-map.md`.

**Checked by Ben on his Mac, 15 Oct:** the raylib HUD opens (`hud --demo`), and with the turtle it shows speed, throttle and steering live while driving from the keyboard. The car kit is made (1.8 GB, `car-kit/`).

**Next, in order:** Thursday evening, `pixi run car-kit`; Friday's audit and our stack on
the car on stands (`docs/history/friday-2026-10-16.md`); the A-to-B
drive (`docs/a-to-b.md`); the HUD's map panel; Python to Rust, step 1 (spine bridge, ego).

## Where we stopped (11 Oct, 12:30)

Ben, 12:27: "lets try to put everything to a reasonable state and we can audit the
hardware on friday". `main` is pushed and its tests pass. The three pieces in progress are
saved on their branches, pushed to GitHub too. Each branch's unfinished part is in its last
commit, "Work in progress".

| Branch | Done there (committed, checked) | Unfinished (its last commit) |
|---|---|---|
| `wt/sim-full` | the road world in Gazebo: lane lines, a stop sign, an obstacle, the car's camera and radar; the car's whole graph in `pixi run sim`, checked by `sim-check` | ~~the Simulation page, and a picture~~ *Done 15 Oct, with a lidar; merged into `main`* |
| `wt/ackermann` | one reverse speed for the whole car: 0.5 m/s, from `vehicle.yaml` | reversing in Nav2: behaviour trees, settings, a test |
| `wt/rust-ports` | the Rust IMU driver is the default, and the C++ one is gone | ~~the radar clustering as a Rust node (`aav_radar_clusters`)~~ *Done 15 Oct: the C++ is gone too; merged into `main`* |

**Next time, in order:**

1. **Friday 16 Oct: the hardware audit,** read-only first (`docs/work.md`, steps 1-3):
   backups, then the inventory and the measurements.
2. **Drive with W A S D,** the car or the turtle (Ben, 11 Oct). `teleop_twist_keyboard`'s
   keys are fixed (i, j, l and so on), so this is a small keyboard driver of our own, in
   the keyboard's place in the mux.
3. **Finish and merge the three branches:** each one's own checks, then `pixi run test`.
   `wt/sim-full` is merged (15 Oct).
4. **Then** the board drivers in Rust, and the raylib HUD.

## 15 Oct: all the C++ to Rust

Ben, 15 Oct: "We should port all cpp to rust", and remove the old.

- [x] **The wheel encoder and the status display Pico 2s, in Rust and Embassy**
      (`wt/rust-boards`): `boards/wheel-encoder-pico2` (same UDP packet, checked against
      the same examples as `aav_legacy`) and `boards/sentinel-pico2` (same I2C packet),
      with their logic in the crates `wheel-encoder`, `sentinel` and `hd44780`. The C++
      folders are deleted; their source stays on GitHub. *Done 15 Oct: builds, never
      flashed.*
- [x] **The radar clustering in Rust, and the IMU's C++ driver gone** (`wt/rust-ports`):
      `aav_radar_clusters` replaces `aav_object_detection` in the car, bench and Gazebo
      launch files, with the C++ node's answers for 94 clouds kept as tests. *Done 15 Oct:
      no C++ left in `src/` (now `ros/`).*

## 14 Oct: the car in photos, the KEFC CAN plan

Ben sent 32 photos and the KEFC request (CAN bus, wheel speed, steering feedback, new GPS,
a second Orin). Pages: [car photos](docs/car-photos.md), [CAN plan](docs/can-plan.md),
[topic areas](docs/naming.md).

- [x] **The raylib HUD** (Ben: "today maybe"). *Started 14 Oct, on `wt/hud-raylib`. Built: `aav_hud_raylib` (now `aav_hud`), works on a Mac.*
- [x] **Topic names by area** (`docs/naming.md`). *Done 15 Oct: Decided by Ben, renamed
      everywhere (`wt/topic-areas`), checked by `tools/tests/test_topic_areas.py`.*
- [x] **DBC v0.2.0** for the KEFC plan: the ESP32 board as the spine, wheel frames 0x200 and
      0x201, a 100 ms timeout (`docs/can-plan.md`, Proposed) [[needs-you]]. *Merged 15 Oct.*
- [x] **A lidar driver:** a lidar is on the roof; our code has none. Model at the audit.
      *A generic 16-beam lidar runs in Gazebo's road world since 15 Oct
      (`lidar:=true`), on `/sensors/lidar/points`: the real driver should publish the same.
      Nav2 reads it since 15 Oct (`aav_lidar` `lidar_scan`); then `car.launch.py lidar:=true`. Done 20 Oct: the stock velodyne driver for the VLP-16, on in `orin.yaml`; untested with the real lidar.*
- [x] **Gazebo's road world on `main`** (`wt/sim-full`): the camera, radar and a roof
      lidar, with the new topic names. *Done 15 Oct: `sim-check` passes on the Mac.*

## 11 Oct: what we can try at home

Ben, 03:00: "We can only do so much from home so lets focus on stuff that makes sence to
work on." His order, with the work under way:

1. [x] **The full graph, all set up:** every driver, ego, the simulators. In Gazebo: a road
      with lane lines, a stop sign and an obstacle, a camera and a radar on the car, and
      perception, lane keeping, collision_monitor and the HUD in `pixi run sim`. *Done
      15 Oct: merged from `wt/sim-full`, with a lidar too ([Simulation](docs/simulation.md)).*
      - [x] **The full simulation** (Ben, 15 Oct: "We want to fully simulate the car inside
            gazebo with all the sensors and build the full pipeline";
            `docs/full-simulation.md`). *Steps 1-3 built 15 Oct on `wt/full-sim-1`:
            `/system/source` and the HUD's SIMULATED banner, the per-actuator topics, the
            actuator model with guessed numbers. Steps 5 and 6 built 15 Oct: the lidar into
            Nav2, sensor failures on purpose and the sensor watchdog (`aav_safety`). Next:
            step 4, the spine in the loop. Done 15 Oct: `pixi run sim spine:=true`.*
      - [x] **The sensor watchdog's to-dos** (`docs/simulation.md`, "Still to do"): a lock
            that expires if the watchdog dies, frozen wheel odometry, the EKF's own guard
            against a GPS jump, the verdict on the map window, the same faults on the car.
            *Done 15 Oct, but for the car: a fail-safe lock, the wheels cross-checked with
            the IMU and the GPS, ekf_global's GPS gate (holds while the car is stopped),
            the map window's banner. Left: the cross check's limits and the faults on the
            car.*
2. [x] **Our C++ to Rust:** Ben's Rust IMU driver becomes the default and the C++ one goes;
      the radar clustering becomes a Rust ROS node. Python to Rust later (Ben). *Done
      15 Oct (Ben: "We should port all cpp to rust"): `aav_imu_yahboom` and
      `aav_radar_clusters`; no C++ left in `src/` (now `ros/`). Only the ESP32's firmware is C++ (and the hardware lead's Arduino).*
3. [ ] **The Mac on the network, with ML on it:** the laptop's detector (Core ML) on the
      car's compressed pictures. *On in `laptop.yaml` since 11 Oct; to try against the
      bench as the car.*
4. [ ] **Nav2:**
      - [x] the obstacle stop: a box as wide as the lane, and a re-arm after a latch, in
            `bench-check --obstacle`. *Passes on `main`, 11 Oct;*
5. [x] **The board drivers in Rust, ready to try on a board:** the wheel encoder and display
      Picos (Embassy); the spine's Pico 2 program is written. *Done 15 Oct
      (`wt/rust-boards`): all three build; none flashed.*
6. [x] **The HUD in raylib** (Ben, 11 Oct: yes): one Rust program, raylib and rclrs, that is
      also the ROS node. The driving screen first (speed, mode, the e-stop button, the
      camera); the Flutter app stays until it matches. *Started 15 Oct (branch
      `wt/hud-raylib`): the driving screen, the e-stop, GPS and topic health run on a Mac
      (`pixi run hud --demo`, `pixi run hud`). Next: the bench, a Pi, the map page.*
      *25 Oct: it matched, so the Flutter app and its back end were archived
      (`archive/2026-27-flutter-hud/`, Q29); this package took the name `aav_hud`.*

## Overnight (10-10 → 10-11): all the code, then Ackermann, Rust, native style

Ben, 21:45: "Lets try to get a fully functionin gthing going on." The order of work is
Decided ([Decisions](docs/decisions.md)):

1. **The high-level pieces**, by five helpers in parallel, each on its own branch:
   - [x] **The HUD app:** an e-stop button, Cancel, the camera page, the car's status, the
         OpenStreetMap credit. Flutter 3.35.7 in `vendor/` (`pixi run get-flutter`,
         `hud-test`). *Fixed: the app could never connect (Socket.IO tried polling
         first); now WebSocket only. The back end has 98 tests.* *Archived 25 Oct with the
         app (`archive/2026-27-flutter-hud/`): `get-flutter` and `hud-test` are gone.*
   - [x] **The HUD's place database:** `pixi run get places` (51 MB, checksum pinned).
   - [x] **The car's launch:** perception as parts of `car.launch.py` (camera, detector,
         stop_signs, radar_clusters), the GPS driver (`gps_driver:=ublox` by default),
         `gps:=false` fixed. *21 launch tests.*
   - [x] **The lane finder:** `aav_lane_finder`, classic OpenCV, 0.78 ms a picture.
         *`bench-check --lane`: from 0.3 m off the centre line back to within 5 cm in
         about 6 s.*
   - [x] **The spine's board program:** `boards/pico2` (Embassy), the `mcp2515` CAN driver,
         a 100 ms hardware watchdog, the e-stop's spare contact, and `pixi run pico spine build`,
         `pico spine tools`, `pico spine flash`. *Builds; never flashed.*
   - [x] **Obstacles in the costmap:** the radar in Nav2's local costmap, collision_monitor,
         the camera's detections as points on the road, compressed images,
         `bench-check --obstacle`. *Merged 11 Oct.*
   - [x] **Fix the obstacle stop:** after the stop-sign drive, Nav2's recovery cleared its
         costmap and the car drove on past the box; alone, the car stopped but the spine
         latched and it never went on. *Fixed in the check, 11 Oct: the car weaves (N7), so
         the box now spans the lane; a latch from a busy computer is armed again.*
   - [x] **The `lane` part** in `car.launch.py`, off on every machine: it drives whenever
         it sees a lane.
   - [x] **The laptop's detector** (Core ML), once it reads compressed images. *On, 11 Oct.*
   - [x] **`bench-check --perception` again on a quiet Mac:** it failed 5 of 5 on a busy one.
         *Passed on `main`, 11 Oct: the car locked 4 m before the sign and went on.*
2. [ ] **Ackermann done properly:** reversing (Reeds-Shepp in Nav2), both front wheels' real
      angles in the model and RViz, and the steering motor's real curve from the
      `Ackermann.m` linkage (Q2). *Reversing done 22 Oct (task 24); the rest still open.*
3. [x] **All our C++ to Rust:** the IMU (Ben's twin becomes the default, then the C++ goes),
      the radar clustering as a Rust ROS node, the wheel encoder and display Picos (Embassy).
      *The two Picos, the IMU and the radar clustering: done 15 Oct. The Picos build, never
      flashed.*
4. [ ] **Every file in its language's own style:** `cargo fmt --check` and
      `cargo clippy -D warnings` in `pixi run test`.
5. [ ] **The whole car working, end to end.**

### Needs Ben or the team

- [x] Make the repo public? Checked 11 Oct: no secrets in any of its 167 commits. *Waits on Ben's
      yes, and on what to tidy first (the 11 October email). Public since 15 Oct (Ben); now `AAV-2025-2026/ecolo-av`.*
- [x] The HUD on the car's Pi: Flutter has no Linux ARM download; the tile server and the
      OSRM route data. *No longer needed: the Flutter app was archived 25 Oct; the raylib HUD
      builds on ARM (CI's `arm` job), and street routes come from Valhalla.*

## On Ben's Mac (10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-10.md`. "Our version" is
`aav/aav` (`~/Code/aav/aav` on the Mac; retired 25 Oct: now `~/Code/ecolo-av`); this work was done in a clone at `~/aav` while that
folder downloaded, and moves into it as a fast-forward.

- [x] **Build and test on the Mac.** *`pixi run test` green after the `coreutils` fix. The
      turtle, Gazebo headless, a Nav2 goal, the Rust listener, the docs, the doctor and
      recording all run. `record.sh` now goes through `tools/ros.sh` (SIP again).*
- [x] **Email everything we talk about, in detail, with sections** (a standing request).
      *Sent at 12:16, 12:32, 12:39, and with the docs PDF.*
- [x] **Docs for a new team member:** first day, starting points, packages, work,
      decisions, infrastructure, build a driver, the costmap exercise, the older code.
      *The driver page and the costmap exercise were run as written, in a scratch copy.*
- [x] **`.gitignore` in `aav/aav`:** restored at 12:35. *The app's per-message snapshots
      were taking in the Linux `.pixi` files as scp copied them.*
- [x] **A `.gitignore` for every kind of code.** *Applied in 593db7f.*
- [x] **A `vendor/` folder** for downloaded source (Ben, 17:03): `vendor/ros2-src.repos`
      (121 repos) and `vendor/ros2-rust.repos` (9), at exact commits, in git; the 559 MB
      out of it. `pixi run get vendor` downloads; `tools/test.sh` checks `COLCON_IGNORE`.
- [x] **Move today's commits from `~/aav` into `aav/aav`** once the download finishes.
      *Done 16:13: the scp was still copying the server's Linux build output after six hours,
      so it was stopped and rsync brought the rest. The copy matches the server, Ben's
      uncommitted work is intact, and 55 commits fast-forwarded in. The Linux `.pixi`,
      `install` and Rust `target` went to `~/Code/aav/aav-linux-build-output`, not deleted.
      Built (25 packages, Ben's `aav_imu_yahboom` too) and tested green on the Mac.*
- [x] **Gazebo's window on the Mac:** Ben saw the screenshots (16:34).
- [x] **The firmware requirements in `vendor/`, built out of the box** (Ben, 20:54).
      *A `firmware` environment with PlatformIO; its downloads go in `vendor/platformio/`.
      `pixi run -e firmware legacy-esp32` built today's ESP32 firmware from a clean
      checkout on the Mac (48 s). The tools to flash an ARM chip (`probe-rs`, a UF2
      converter) aren't on conda-forge: they come with the first board program.*
      *Done 11 Oct: `tools/elf2uf2.py`, and probe-rs in `vendor/cargo-tools`.*
- [x] **CARLA: 4 or 5, and do we need it?** (20:54) *Parked this year: no Jazzy
      support, no Mac version. Gazebo and bench mode cover the car graph (Q31).*
      *Reopened 15 Oct (Ben: "I think we want carla 5 not 4"): research, `carla_adapter`
      through CARLA's Python client, and Carleton's OpenStreetMap download done
      (docs/carla.md). Left: run it on the NVIDIA computer.*

## The afternoon of 10-10: "merge in all of this", then "stub in the rest"

- [x] **Perception in:** `aav_yolo`, now `aav_detector` (YOLO, detections, the stop-sign watcher and its mux
      lock), `aav_object_detection` (radar clustering), `aav_lane_geometry` (lane keeping
      on `/drivers/ai/cmd_vel`), the camera calibrations in `cameras.launch.py`, the `ml` environment.
- [x] **The graph down to the sensors:** bench mode (`pixi run bench`): spine_sim runs
      spine-core on the CAN frames and drives a model car; pretend IMU, GPS, camera,
      battery and radar. `pixi run bench-check`: 7 of 7. `gps_nmea.launch.py`,
      `perception.launch.py`, udev placeholders.
- [x] **Rust drivers for the spine's devices:** jrk-g2, ad5311, pca9555, ibus, spine-board
      (98 tests); all seven firmware crates build no_std for the Pico 2 in `pixi run test`.
- [x] **The HUD's back end on our topics:** `aav_hud`, 86 tests. *(The Flask one: archived 25 Oct; `aav_hud` is now the raylib HUD.)*
- [x] **Docs:** Bench mode and Perception pages; every page brought up to date.
- [x] **Ben: commit the server work** (`aav_imu_yahboom` and its three edited files). Then
      perception and the bench become parts in `car.launch.py`'s machine files.
      *Done: the server's files are byte for byte in `dfaec15`; perception is parts
      since 11 Oct.*
- [x] **The board program for the spine's chip** (implements `spine-board`'s `Board`): waits
      on the chip (Q10-Q13). *Done 11 Oct for a Pico 2, to try on the bench.*
- [x] **The HUD app's small changes** (docs/hud.md). *Done 11 Oct. Archived 25 Oct.*
- [x] **The first-day steps in `README.md`** (Ben, 17:03).
- [x] **`~/Code/aav/CLAUDE.md`:** every old folder, and what replaced it in `aav` (Ben, 17:03). *That workspace was retired 25 Oct.*
- [x] **An MCP server** (Ben, 17:03): read-only, over stdio. `tools/mcp_server.py`, `.mcp.json`
      and `docs/mcp.md`: 9 tools. Its smoke test reads a live topic.
- [x] **On GitHub** (Ben): `git@github.com:zander1976/aav.git`, private (public since 15 Oct; since 25 Oct `AAV-2025-2026/ecolo-av`). CI: Linux on every push, a
      Mac every night. Commit messages give the intention, with no attribution lines; the
      README's Credits section covers who did what.
- [x] **Ben's Rust IMU driver** (`aav_imu_yahboom`, then `imu_driver:=rust`): reviewed and
      committed; its tests now run in `pixi run test`. *The only IMU driver since 15 Oct.*
- [x] **Cleaned up** (Ben approved): `~/aav`, the Linux build output and the old bundle.

## The first night (10-09 → 10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-09.md`.

- [x] **Build on this machine** with Pixi + RoboStack Jazzy (no admin, no Homebrew).
      `colcon build`, `colcon test`, then run `turtle.launch.py` headless and drive it.
      If something can't build, write down why.
      *Done: all 19 packages build in 11 s; our tests pass; the turtle drives through
      mux → ego; check_graph: 0 failures. The inherited packages' lint failures were
      fixed on 10-10.*
- [x] **Mac tomorrow.** The same `pixi.toml` for macOS (Apple Silicon). Check that every
      package resolves for `osx-arm64`. Write the steps: no Homebrew.
      *Done 10-10 on Ben's Mac (Apple Silicon, macOS 27): build 18 s, `pixi run test` all
      green after one fix (GNU `coreutils`: SIP drops `DYLD_LIBRARY_PATH`). See below.*
- [x] **Port everything to Jazzy as is.** Apply the fixes the audits found (configs,
      inherited code, missing dependencies). Reports: `docs/history/audit-2026-10-10-*.md`.
      - ~~Spine safety, before any firmware is written~~ *Done 10-10: B1 arming always
        sends the 0→1 edge; B2 the spine boots latched; W1 `cmd_timeout` 0.1; W2 driver
        faults stop and latch; W3 `max_decel` 2.5. Each has a test (Rust 47, bridge 4).*
      - ~~Legacy path~~ *Done 10-10: `to_legacy` evens out the steering gain; `encoder_udp`
        goes quiet after 0.3 s without packets; it runs on the Motor Pi, where the
        Pico sends.*
      - ~~pixi.toml deps; inherited packages~~ *Done 10-10: flask, flask-socketio, scipy,
        topic_tools added for all 3 platforms. Radar, battery and IMU code restyled to
        ROS's lint rules and fixed (no `sudo chmod`, udev port names, clean shutdown);
        maincar's NameErrors and start-up fixed; CMake 3.10; `.warning(`. `pixi run test`
        is all green (colcon test: 77, 0 failures).*
      - ~~Live check of sim, car and replay~~ *Done 10-10: every config key is declared
        by its Jazzy node. Fixed: Nav2 on sim time, the sim GPS noise (degrees, not
        metres), a map on the car, steering speed and inflation in the sim, a replay
        profile. Nav2 goals now succeed in Gazebo. Left for the team: see below.*
- [x] **The graph.** Every topic has the right publisher, subscriber, type and QoS.
      Check it live with `tools/check_graph.py` on the turtle run.
      *Done 10-10: turtle, sim and replay have 0 failures. The car, with no hardware,
      fails only on `/sensors/imu/data_raw` and `/sensors/gps/fix`. No QoS mismatches.*
- [x] **Docs as HTML (and PDF)**, short pages, sections, red and white, a picture of how
      it fits together. Markdown stays as the source.
      *Done: `docs/html/index.html` and `pixi run docs` (PDF). Four pictures; pages for
      each point below.*
      - What ROS 2 is, and how we extend it (stock modules vs our nodes)
      - How Python, C++ and Rust fit into Jazzy, and what we give up with Rust
      - How the dual EKF works ("local injects, global reads local + GPS")
      - Run it: turtle, keyboard, Gazebo
      - src vs firmware; one interface, several drivers (the IMU example)
- [x] **Email progress** to cmail, red and white, every few hours and when something lands.
      *Sent at 22:49, 03:50, 04:07 (the morning summary, with the PDF) and 04:17 (the car
      must run Jazzy before a Mac can join it).*
- [x] **Naming and code style** for all code. Inherited code is ours to edit now.
      *Done: `docs/conventions.md`, `pixi run lint` passes. Renaming the inherited
      packages is proposed and waits for the team.*
- [x] **Rust:** pull `rclrs` and the message crates through cargo. Build one example in
      the Pixi environment. Write down what a Rust rewrite would take (ESP32, Pi).
      *Done: `examples/rust-listener` (rclrs 0.8 + ros-env 0.3 from crates.io; the
      message code ships with Jazzy) builds in 18 s and printed ego's commands live.
      `pixi run rust-listener`. The rewrite trade-offs are in docs/languages.md. Removed
      26 Oct: four real rclrs nodes show the same.*
- [x] **Design doc:** fix the ten "last year's", and update the tree, profiles and decisions.
      *Done 10-10 (doc rev 58): all ten now name 2024-25; the tree, the four profiles,
      the mux priorities, the tests, Gazebo before CARLA; Jazzy, one repo and Languages
      marked Decided; new rows: Macs (Decided), bare metal waits (Tentative), spine
      arming (Proposed).*
- [x] **CLAUDE.md:** record tonight's decisions, so no session starts from scratch.
      *Done: docs for a dyslexic reader, Macs first-class, `pixi run test` green, test
      isolation, the pytest skip trap, Rust nodes later, where the TODO and audits live.*

## Decided on the first night

- Teleop (keyboard, joystick, remote) is a **driver**: it feeds the mux, then ego, then
  the turtle or the car.
- **Bare metal: wait.** Only the spine firmware is bare metal.
- **Port to Jazzy as is first**, then consider rewriting in Rust.
- **Simulator:** start with a Gazebo test world.
- **Docs:** short, sectioned, HTML or PDF. Ben is dyslexic, so use tables and pictures
  over walls of text.
