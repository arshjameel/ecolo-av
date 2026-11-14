# One developer window: Gazebo, Nav2's map and the cubes on one screen

* **Author:** Ben (the ask), written up 5 Nov 2026
* **Date:** 2026-11-05
* **Status:** the scope is Ben's. Everything marked "proposed" is **proposed; awaiting Ben**.
  Revised after the EGM review. Partly built (checked 13 Nov), in Bevy and not raylib as this page first proposed: `ros/hud/aav_dev_window` (TODO 114), live on the simulators, with click-to-go. Not built: the algorithm list, SWAP, cancel a goal. TODO 107.
  Replaces TODO 73's "merge the cockpit and the debug HUD".

## Changed by Ben, the evening of 5 Nov

**Where this page and the evening disagree, the evening wins.** The plan from here:
[our own simulator](2026-11-05_own-simulator.md).

| Ben decided | So this page's |
|---|---|
| Left: the scene over a console. Right: what the lidar sees from the car with NDT, one map you click, keys | window drawing is replaced: `ros/hud/aav_dev_window` |
| Bevy on laptops; raylib only for the HUD and street map, on the Pi | "raylib first" is dropped. D1 waits only for a slower laptop's result |
| The window draws the cubes the car makes, never the point cloud | cube source changes to `/perception/voxels/cubes` |
| The Orin runs no graphics and no simulator | every "on the Orin's screen" line is wrong, and `debug_hud` retires once this window works |
| Gazebo goes, once our own simulator wins | Gazebo's picture is only for the time until then |

Still right: the empty panels' wording, the status strip, asked and alive, SWAP's stages, the
three gates on goals, and two presses for a goal.

| Phase 0 | Done 5 Nov: `demo_all.sh` says `rviz:=false`, so one RViz opens. Not yet launched |
|---|---|

## What Ben asked

| When | His words |
|---|---|
| 4 Nov | "Right now I get tons of windows and I even get the nav stuff twice." |
| 5 Nov | "Now when I run gazeboo, i get the gazebbo 3d display, a 2d pop up of the nav2 map so I can select were to go, and ndt/icp stuff and i want a single screen just for the developers and a list of algo so you can swap." |
| 5 Nov, on the technology | "If we can find a way to merge gazebo and nav2 and cube window great, if not lets think about using bevi. We have the code from gazebo . The car hud and street map should stay raylib." |

That last note came with his pick of the option "Try Bevy first". This design proposes
raylib first, because the merge looks possible. It is his call: see D1.

| Decided by Ben | Proposed; awaiting Ben |
|---|---|
| One screen, just for developers | the window is `debug_hud`, grown, in raylib |
| Gazebo's 3D view, the Nav2 map you click, NDT and ICP, a swap list | Gazebo's view comes as the chase camera's picture |
| The car's HUD and street map stay raylib, unchanged | Bevy only if the test below passes |
| Every algorithm stays in the list; our hybrids join as they're built | SWAP restarts the whole simulator at first |

## The idea in one line (proposed)

`pixi run demo` opens one window: `debug_hud` in a new developer mode, showing Gazebo's real
picture beside the Nav2 map, the raylib cubes, NDT, ICP and the algorithm list. Gazebo runs
without its own window.

## Goals

| # | Goal | Done means |
|---|---|---|
| G1 | One window for developers | plain `demo` opens no Gazebo window, no RViz, no HUD, no map window |
| G2 | See the simulated world | Gazebo's picture shows, spawned posts included |
| G3 | Send the car from a click | click, aim, confirm; cancel works; heading is yours |
| G4 | See where the car thinks it is | NDT, ICP and every localizer, with clear "off" states |
| G5 | Swap any algorithm from a list | the list shows what is asked for and what is alive |
| G6 | Still know why the car stopped | e-stop, watchdog, locks and source always on screen |

## Non-goals

| Not in this design | Why |
|---|---|
| Changing the HUD or the map window | Ben: they "should stay raylib" |
| Anything on the car | the car's screens stay; `debug_hud` on the Orin stays listen-only |
| A sender of goals on the car | shadow mode needs one; it isn't this window |
| Compiling Gazebo's renderer into our program | C++ API, breaks "no C++ of our own" |
| Free look through Gazebo's world, first version | later, only if the fixed view is too limiting |
| Our own drawing of the world, first version | the picture already shows it; the reader moves out |
| Changing `pixi run sim` | it keeps Gazebo's window and RViz, for anyone who wants them |

## Can we merge Gazebo, Nav2 and the cubes?

From reading Gazebo Harmonic's source (`vendor/gazebo`). No runs; efforts are estimates.

| Route | Verdict | Loses |
|---|---|---|
| Gazebo's own GUI as the window | works with limits | clean click-to-goal, side-by-side map, algorithm list |
| **Our Rust window + the chase camera's picture** (proposed) | works | free look; cubes can't sit behind Gazebo's objects |
| The same with free look: a separate camera model moved by `set_pose` | works with limits | smoothness, sim speed |
| Lichtblick | works with limits | free look; reverses Ben's 4 Nov call for a Rust window |
| RViz | works with limits | algorithm list, swap |
| gz-rendering through FFI | no | breaks no-C++ |

The evidence, route by route:

| Route | Evidence |
|---|---|
| Gazebo's GUI | `/marker` works from Python (`gz-gui/.../MarkerManager.cc:162-182`) |
| | no stock plugin reports a click; a plugin must be a compiled library (`gz-gui/src/Application.cc:553,568`) |
| | one 3D scene only (`MinimalScene.cc:745`); no text on ogre2 |
| | 20,000 cubes and a 200x200 costmap as markers look impractical |
| Chase camera | the bridge exists (`ros/sim/aav_gazebo/config/bridge_chase.yaml`) |
| | `hud-core` `picture.rs` reads rgb8; `hud-app` already subscribes to `Image`; `debug-app` already uses `hud-core` |
| | measured with it: 72 % to 83-88 % of a core, RTF 0.985 to 0.96-0.98, road world, parked (`docs/foxglove.md:129-130`) |
| | the car's own cameras render headless on Ben's Mac at 10 Hz, RTF 0.98-0.99, road world, before the chase camera existed (`docs/simulation.md:380-382`) |
| Free look | `set_pose` works on top-level models only (`gz-sim/.../Physics.cc:2418-2431`), so not the camera on the car |
| | all rendering shares one thread, and the sim waits for the last render (`Sensors.cc:395,528,968-972`), so slow rendering lowers RTF |
| | triggered cameras exist (`sdformat/sdf/1.10/camera.sdf:8,16`); latency guessed at 50-150 ms, unmeasured |
| Lichtblick | the cockpit and a goal tool exist; two script layers don't work; NDT's map has no topic (`docs/foxglove.md:89-95`) |
| RViz | stock goal tool; Mac untested (`docs/mac.md:294`); a custom panel means a C++ plugin |
| FFI | C++ API only, and it doesn't load the world |

### Why not Bevy now

- Gazebo's renderer is C++ on Ogre. So "we have the code from gazebo" gives Bevy only data:
  SDF files, meshes, poses. raylib can use the same data.
- The car's screens stay raylib, so Bevy means two engines in one package.
- raylib already holds up: 358 fps at 20,000 cubes with pretend data, 58 fps live against
  Gazebo (`docs/hud.md:746-747`).

### The test for moving to Bevy (proposed)

Move only when **all three** hold:

| # | Condition | How we'd know |
|---|---|---|
| 1 | Ben wants smooth 60 fps free look through a lit 3D world | he says so, after using the fixed view |
| 2 | Gazebo's picture is too slow or too costly | run S1 (and S2 for free look) misses its marks |
| 3 | raylib fails on his Mac | under 30 fps at 20,000 cubes, or it can't draw the world acceptably |

### Three runs that settle it

Nobody has run these. The marks are proposed.

| Run | What | Measure | Pass |
|---|---|---|---|
| S1 | the chase camera at 10, 15 and 30 Hz, Ben's Mac, `test_lot`, driving | RTF, Gazebo's CPU, the window's fps, frame age | RTF 0.95 or more, frame age 0.2 s or less, at 10 Hz or more |
| S2 | a free camera model moved by `set_pose` | click-to-frame latency, RTF | only if the fixed view is too limiting |
| S3 | a short `/marker_array` script in Gazebo's GUI | does it draw, does it show in the lidar and cameras | only if the GUI route stays alive |

| Note on S1 | Detail |
|---|---|
| Frame age | latest local odometry's stamp minus the picture's stamp: stamps compare only with stamps (`debug-core` README) |
| Rate | fixed at 10 Hz in `aav.sensors.gazebo.xacro:155`, so the run needs a temporary change |
| Must be settled first | `docs/simulation.md:195,364` say `test_lot` renders nothing; `test_lot.sdf:8,25` loads Sensors |

The source points one way on that last row: `sim.launch.py` goes by `worlds.has_sensors`, and
`docs/simulation.md:82` itself lists the chase camera in `test_lot`. S1 settles it by running.

**Don't draw overlays with `/sensors/marker`:** they may appear in the lidar and cameras
(inferred; test first).

## What merges, and what doesn't

| Window | Program | Shows | Becomes (proposed) |
|---|---|---|---|
| Gazebo | `gz sim` GUI | the world in 3D | headless; its picture is a panel. `--gazebo` brings the window back |
| RViz (opened twice) | `rviz2` | Nav2's map; click to set a goal | off, once click-to-go works. `--rviz` brings it back |
| The debug HUD | `debug_hud` (raylib) | cubes in 3D, NDT's map and match, every localizer's car | **grows into the window** |
| The HUD | `hud` (raylib) | speed, state, GO / STOP, e-stop, camera | unchanged; only opens with `--car-screens` |
| The map window | `map` (raylib) | street map, place search, DRIVE HERE / CONFIRM | unchanged; only opens with `--car-screens` |
| The hardware panel | `hw_panel` | actuators and sensors | unchanged, separate; `demo` doesn't open it today |
| Lichtblick | cockpit, over the bridge | panels of topics | unchanged |

The cockpit is watch-only **on the car**. In the simulator it can send a goal, drive and
e-stop (`docs/foxglove.md:148-151`).

## The window

```text
┌─ STATUS: GO / STOP and why · e-stop · watchdog · source · [E-STOP] ───────────┐
├───────────────────────────┬──────────────────────────┬────────────────────────┤
│ NAV2 MAP, from above      │ GAZEBO'S PICTURE         │ ALGORITHMS             │
│ map, costmap, plan, goal, │ the chase camera         │ row · asked · seen     │
│ every localizer's car     ├──────────────────────────┤ planner      ▾         │
│                           │ CUBES, 3D (raylib)       │ follower     ▾         │
│ click, drag to aim        │ lidar's and Nav2's,      │ where am I   ▾         │
│ [DRIVE HERE] [CONFIRM]    │ plan, cars               │ obstacles    ▾         │
│ [CANCEL GOAL]             ├──────────────────────────┤ speed loop   ▾         │
│                           │ NDT · ICP: score, ms,    │ [ SWAP ]               │
│                           │ LOST                     │                        │
└───────────────────────────┴──────────────────────────┴────────────────────────┘
```

| Panel | Reuses | New |
|---|---|---|
| Nav2 map | `map-core` `grid.rs`, `view.rs` (`to_map` for clicks); `map-app` draws it today, so the drawing moves or is copied | the panel in `debug-app` |
| The cars from above | `debug-core` `topdown.rs`; `state.rs` `pose_in_map()` (`poses.rs` only buffers the local EKF) | none |
| The plan | `state.rs` already holds `/plan` | a 2D line |
| Costmap | `Grid` reads any OccupancyGrid; `nav2.yaml:343,394` already send full costmaps | subscriptions, and colours for cost |
| Gazebo's picture | `hud-core` `picture.rs`; `hud-app`'s way of showing it | the panel; subscribes to `/vehicle/gazebo/chase_camera/image_raw` (6.9 MB/s, this computer only) |
| Cubes, 3D | `cubes.rs`, `voxels.rs`, `persist.rs`, `orbit.rs` | a smaller panel |
| NDT | `ndt.rs` | none |
| ICP | `/localization/kiss_icp/status` and `/odom` | status row and a trail. KISS-ICP has no map and its cloud has no topic, so no match view (D6) |
| List | `pick.py`'s rows, `aav_run_info` | everything else |

### The status strip

D7 would hide the HUD, and with it GO / STOP and the e-stop. So the strip is always on.

| Shows | From |
|---|---|
| GO / WAIT / STOP, one reason, what to do | `hud-core` `gate.rs` `verdict` |
| E-stop | `/vehicle/status` |
| Output gate | `/safety/output_mux/state` (new subscription) |
| Watchdog | `/safety/sensor_watchdog/state` (new subscription) |
| Geofence and collision-monitor locks | `/safety/geofence/lock`, `/safety/stop_hold/lock` (new; `verdict` doesn't cover them) |
| Source | `/system/source` |
| [E-STOP] button | `hud-core` `estop.rs`: sends only `true`. Clearing stays `pixi run clear estop` |

Closing the HUD is safe for Nav2 in `demo`: the watchdog only watches its heartbeat with
`hud:=true` or `watch_hud:=true` (`sim.launch.py`), and `demo` sets neither.

### Empty panels

Plain `demo` runs no NDT, AMCL, ICP or trial tracker (`demo_all.sh:40-53`). A panel never looks
broken:

| Panel | Says |
|---|---|
| NDT, not running | "NDT off: pick it in the list" |
| NDT, no map file | "no map: `pixi run ndt-map drive`" (as today) |
| ICP, not running | "KISS-ICP off: pick it in the list" |
| Tracker | "Nav2 steers (MPPI)" |
| Picture, none | "no picture: `chase_camera:=false`, or this world renders nothing" |
| Map, none yet | "waiting for Nav2's map" |

## What `demo` starts (proposed)

| Command | Starts |
|---|---|
| `pixi run demo` | Gazebo headless, the stack, the one window |
| `demo --debug` | the same, with the switches `--debug` sets today: AMCL, a trial tracker, NDT if its map exists. It no longer changes which windows open |
| `demo --car-screens` | adds the HUD and the map window |
| `demo --rviz` | adds RViz (`drive.rviz`) |
| `demo --gazebo` | Gazebo's own window too (`gui:=true`) |
| `demo --fullscreen` | the window full screen; the car's screens too, if open |
| `pixi run sim`, `pixi run debug-hud` | as today |

`--debug` stays because `pick.py`, its tests and many docs use it.

### RViz opens twice today

Checked: `demo_all.sh:86` starts `drive.rviz`, and `sim.launch.py:345-349,362` starts `road.rviz`
by default, because `demo` never says `rviz:=false`.

| Step | Change | Why this order |
|---|---|---|
| Now | `demo_all.sh` adds `rviz:=false` to the launch | one RViz, and `drive.rviz:118` keeps the goal tool. Cures "the nav stuff twice" |
| After click-to-go | line 86 moves behind `--rviz` | until then RViz is plain `demo`'s only click-to-go: `map-app/main.rs:417-439` handles clicks on the search box only |

## Click to go

`destination.rs` is the search box's flow: it takes a place, not a click. So this is new, in a
new `debug-core` module, using what exists.

| Step | You do | Uses |
|---|---|---|
| 1 | click the map | `view.rs` `to_map` |
| 2 | hold and drag to aim; no drag: faces from the car, as the search does | `destination.rs` `goal_for` for the map and car checks |
| 3 | see the arrow, or the refusal in words | `Refusal`; new: refuses an occupied cell, from `Grid::cell` |
| 4 | press DRIVE HERE, then CONFIRM | the same two presses as the map window |
| 5 | goal goes out on `/goal_pose`, map frame | stock Nav2 |
| 6 | CANCEL GOAL, or Esc before CONFIRM | below |

| Detail | Proposed |
|---|---|
| Drag in the map panel | left picks; right-drag pans; wheel zooms, as the 3D view |
| Cancel | call `/navigate_to_pose/_action/cancel_goal` with an empty goal ID: cancels all. No window of ours can cancel a Nav2 goal today |
| Why that way | `/goal_pose` gives no goal handle. rclrs 0.8 has action clients, but none is used here yet. The call is untried from rclrs |
| After cancel | `map-core` `trip.rs` already says "Nav2 stopped: cancelled or failed?" |
| The car's position | `/localization/odom/global`, as the map window; `debug_hud` already hears it |

### Who may send a goal

`tools/screen.sh` runs `debug-hud` **not** local, so it can see a real car. Three gates:

| Gate | Holds |
|---|---|
| `developer:=true` launch parameter, default false | no publisher or cancel client exists without it |
| Discovery is local | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, read at start (`ros.sh:39` lets `AAV_DISCOVERY` change it, so check the value, not the flag) |
| `/system/source` says SIM | `debug-core` `swap.rs` `allowed`, checked at each press. It also refuses BENCH and "not heard yet". The wording becomes general |

Only `demo_all.sh` passes `developer:=true`.

### `debug_hud`'s promise changes, in the contract first

| Today | Proposed |
|---|---|
| "publishes nothing"; one call, SIM only (`docs/hud.md:670`, `aav_hud` README) | **without `developer:=true`, exactly that** |
| | with it: also `/goal_pose` (after CONFIRM) and `/safety/estop` (`true` only), and the cancel call |

| Edit first | Then |
|---|---|
| `docs/interfaces.yaml`: `debug_hud` under `/goal_pose` and `/safety/estop` pub, the chase image, status and costmap subs; the cancel call | `tools/gen_interfaces_md.py > docs/interfaces.md` |
| The READMEs' invariants | the code |

**The Orin:** `docs/hud.md:729-730` says the debug HUD runs on the Orin's own screen. That run
never passes `developer:=true`, so it opens the same two layouts as today, and listens only.

**The window never drives:** a goal is a request to Nav2. twist_mux, ego and the locks stay in
charge.

## The algorithm list

| Topic | Proposed |
|---|---|
| Data | `pick.py`'s rows, one source: new `pick.py list --json`; the window reads it. `debug-core`'s `serde_yaml_ng` can read JSON, so no new dependency |
| What each row gives | the name, plain words, the switches, and `blocked`: why it can't run here |
| Every algorithm stays | rows missing today are added: `lattice_headings:=32`, `kiss_icp:=true` alone. New hybrids get a row |
| Where-am-I | one list of pick's combinations at first: `ekf`, `ndt`, `amcl`, `both`, `ndt_ekf`, `kiss_ekf`, plus `kiss`. They aren't truly one choice; ticks can come later |

### Dependencies

| Row | Needs, or forces | Source |
|---|---|---|
| any tracker | forces `reversing:=false` | `vehicle.check_tracker`; pick's rows |
| `mppi`, `rpp` | `tracker:=none` | pick's rows |
| `lattice` with a tracker | warns: forward only, it lost the U-turn; `hybrid` is the car's choice there | `docs/switches.md:34` |
| `diffusion` | `pixi install --locked -e learn`, `pixi run diffusion train` | `docs/switches.md:31` |
| `ndt`, `both`, `ndt_ekf` | `maps/ndt/test_lot.pcd`: `pixi run ndt-map drive` | `sim.launch.py` refuses |
| `kiss`, `kiss_ekf` | `pixi install --locked -e kiss` | `docs/switches.md:71` |
| `amcl`, `ndt`, `voxels`, `kiss` | the lidar; `demo` has it | `sim.launch.py` refuses |

A blocked row stays in the list, greyed, with the command that unblocks it. Switches are rebuilt
from the chosen rows each time, never added to the last run's: otherwise a tracker's
`reversing:=false` would stick after going back to MPPI.

### Asked, and alive

`/system/algorithms` says what the launch **asked for**, not what is alive
(`aav_run_info/README.md`). So the list has two columns.

| Row | Seen from |
|---|---|
| Tracker | `/drivers/nav/track/status`: it also shows a live swap |
| NDT | `/localization/ndt/status` |
| KISS-ICP | `/localization/kiss_icp/status` |
| AMCL | `/localization/amcl/pose`, `/particles` (only after it updates: check) |
| Cubes | `/perception/voxels/status` |
| Diffusion | `/drivers/nav/diffusion/status` |
| Planner, Nav2's controller | `planner_server`'s `GridBased.plugin`, `controller_server`'s `FollowPath.plugin`, read by parameter call (new; untried) |
| Speed loop | nothing yet: says "asked only" |

| State | Means |
|---|---|
| ALIVE | answered within `stale_after` |
| NOT SEEN | asked for, and silent: red |
| ASKED ONLY | nothing can prove it |
| OFF | not asked for |

## SWAP

### What each swap needs

| Switch | Needs | Today |
|---|---|---|
| One trial tracker to another | nothing: live | keys 1-4 cover four; `swap.rs` lacks `mpc_full` |
| `tracker:=none` to or from a tracker | Nav2 restarted | launch only |
| `planner`, `controller`, `voxels`, `reversing`, `amcl`, `sim_speed` | Nav2 restarted | launch only |
| `ndt`, `kiss_icp`, scored only | that one node | launch only |
| `ndt_ekf`, `kiss_ekf` | the EKFs too | launch only |
| `speed_loop`, `actuator_model`, `lidar`, `radar`, `chase_camera`, `world` | Gazebo's launch | launch only |

### Two stages (proposed)

| Stage | What | For | Against |
|---|---|---|---|
| **A, first** | SWAP stops the whole `sim.launch.py`, Gazebo too, and starts it with the new switches. The window stays open | no launch split; no pose seeds; every switch works; a clean start | the car goes back to the start; slower (`demo_all.sh:83` alone waits 20 s; total unmeasured) |
| B, only if A is too slow | split into a Gazebo launch and a stack launch; Gazebo keeps running | the car stays where it is | seeds, clock and TF problems below; `speed_loop` and friends still need A |

Partial restarts have gone wrong here before: on 21 Oct, restarting single nodes on Ben's Mac
made a half-mute sim worse, and a full stop and start fixed it. Another reason to start with A.

Tracker swaps stay live in both stages, through `swap.rs`. No restart, no stop needed.

### Stage A, in detail

| Question | Proposed |
|---|---|
| Who owns restarts | `tools/demo_all.sh`, not the window. It already owns the processes |
| How the window asks | writes the switches to a request file in a per-run temp folder given by `--swap-dir`; `demo_all.sh` watches it |
| Why not a ROS service | a file exists on this computer only; a service would be a restart button on the graph |
| `debug-hud` started by hand | no `--swap-dir`, so SWAP is off and says "started by hand: use `pixi run pick`". Live tracker swap still works |
| Gates | developer mode, local, SIM (as for goals) |
| "Stopped" | `/vehicle/odom` under 0.05 m/s for 1 s (numbers proposed). Stale odometry counts as not stopped |
| Moving | refused, with "stop first" and the CANCEL GOAL button |
| Stopping the old run | by its process group, never `pkill` by name; then wait for it to end |
| The window during it | "restarting: planner:=hybrid"; clears its kept state; sim time goes back to 0, which kept cubes already handle (`debug-core` README). The rest needs a test |
| Done | the launch is up and the list's asked rows are ALIVE |
| One at a time | a second SWAP is refused until the first ends |
| Printing | the full command goes to the terminal, to learn or paste, as `pick` does |

### When a restart fails

| Case | What happens |
|---|---|
| The launch refuses (no NDT map, bad mix) | `demo_all.sh` writes the last lines to a status file; the window shows them; no retry |
| Not ALIVE after a time limit (90 s proposed; measure first) | the window says which rows are NOT SEEN; the run stays up to look at |
| Either | [BACK TO LAST] restarts with the last good switches. Never by itself |
| Gazebo won't die | after a time limit, stop the process group harder; say so |
| Stale shared memory, sim goes mute | the window shows NOT SEEN; `fastdds shm clean` is the known fix. Check it applies under Cyclone DDS, today's default |

### Stage B's extra problems, if it comes to that

| Problem | Today | Needs |
|---|---|---|
| AMCL's seed | `amcl_x=0.0`, `amcl_y=start_y`, `amcl_yaw=start_yaw` (`sim.launch.py`) | the car's real pose, or AMCL starts wrong |
| NDT's seed | `ndt_x=0.0`, `ndt_y`, `ndt_yaw`, same | same; it also has `/localization/ndt/initial_pose` |
| EKFs restarted | `ndt_ekf`, `kiss_ekf` | `map -> odom` jumps |
| Geofence | centred at `start_y` | keeps its place; check |
| Live planner swap, stock | Nav2's `PlannerSelector`, `ControllerSelector` BT nodes exist (`vendor/ros2-src/.../nav2_behavior_tree`) | all plugins loaded in one server; our trees changed. Worth a look before B |

### BENCH and the car

| Where | SWAP |
|---|---|
| BENCH | off: `swap.rs` refuses it, and no `demo_all.sh` owns the run. `run_info` isn't started there either (TODO 87) |
| The real car | off, three ways: no developer mode, not local, source not SIM |

## Checks

macOS blocks clicks from scripts (`docs/hud.md:188`). So each check says how it runs.

| Label | Means |
|---|---|
| **test** | `cargo test` or pytest, no window |
| **hook** | a scripted option through the same logic, as `map --send-goal` and `hw_panel --self-test` do |
| **by hand** | a person, a real mouse; written down with the date |

`--click-goal`, `--swap-to` and `--bench-window` below are new options, to be built with their phases.

| Check | How | Pass |
|---|---|---|
| Click to map, aim, refusals, gates, two presses | test | off the map, no car, occupied, not SIM, not local: all refused in words |
| Without `developer:=true` | test, with ROS | no publisher on `/goal_pose` or `/safety/estop`; today's layouts |
| Goal in Gazebo: `--click-goal x,y,yaw --self-test` | hook | Nav2 plans and drives; plan shows on map and in 3D |
| Cancel | hook | Nav2's goal ends; the car stops |
| The real click, drag and buttons | by hand | same result |
| Status strip for each cause | test, then `--demo` screenshots | right reason, e-stop first |
| `--demo` window | hook | every panel filled; a screenshot needs an unlocked Mac |
| Empty panels on plain `demo` | hook | every "off" line shows |
| A post spawned while running | by hand | visible in the picture |
| S1 | hook | its marks above |
| `pick.py list --json` | test | every row, matches `test_pick.py`'s rules |
| Asked and alive | hook | NOT SEEN when a node is killed by its process group |
| SWAP `lattice` to `hybrid`: `--swap-to` | hook | `/system/algorithms` and the plugin parameter both say hybrid |
| SWAP while moving, not local, by hand-started | test | refused, with the reason |
| SWAP to a broken choice | hook | error shown; BACK TO LAST recovers |
| Plain `demo` | by hand | one window; no RViz; `--rviz`, `--gazebo`, `--car-screens` each add theirs |
| `pixi run test`, `tools/test.sh` | test | green; `demo` stays local |

**Speed:** 30 fps or more, 20,000 cubes and the picture, on Ben's Mac **and** on a slower team
laptop (which one: ask the team), with Gazebo running, RTF noted. Ben's Mac alone isn't a fair
bar. Add a `--bench-window` beside `--bench-cubes`.

## Order

All sizes are estimates. Nobody has built these parts. One person, working days.

| Phase | Delivers | Size |
|---|---|---|
| 0 | RViz once | an hour |
| 1 | developer mode: Nav2 map, costmap, click-to-go, cancel, status strip; RViz off | 1-2 weeks |
| 2 | S1, then Gazebo's picture; `gui:=false` | half a day, then 1-2 days |
| 3 | the list, read-only: asked and alive; ICP panel; live tracker from the list | 3-5 days |
| 4 | SWAP, stage A | about a week |
| Later | stage B, free look, world reader, Bevy: each only if needed | not sized |

Phase 3 could ship before phase 2. SWAP can wait as a later phase without blocking the rest.

### Files per phase

Paths under `ros/hud/aav_hud/` unless they start `tools/`, `docs/` or `ros/`.

| Phase | Files |
|---|---|
| 0 | `tools/demo_all.sh` |
| 1, contract | `docs/interfaces.yaml`, `docs/interfaces.md`, `README.md`, `../README.md`, `crates/debug-core/README.md`, `docs/hud.md` |
| 1, logic | `crates/debug-core/src/click_goal.rs` (new), `lib.rs`, `config.rs`, `state.rs`, `swap.rs` (wording) |
| 1, window | `crates/debug-app/src/ros.rs`, `ui.rs`, `main.rs`, `demo.rs`; `config/debug_hud.yaml`; `launch/debug_hud.launch.py` |
| 1, start | `tools/demo_all.sh`, `tools/pick.py` (words), `pixi.toml` (description), `docs/switches.md`, `docs/features.md` |
| 2 | `crates/debug-core/src/config.rs`, `crates/debug-app/src/ros.rs`, `ui.rs`; `docs/interfaces.yaml`, `docs/interfaces.md`; `ros/bringup/aav_description/urdf/aav.sensors.gazebo.xacro` and `ros/sim/aav_gazebo/config/bridge_chase.yaml` (comments: "only the cockpit reads it"); `tools/demo_all.sh`; `docs/simulation.md:195,364` |
| 3 | `tools/pick.py`, `tools/tests/test_pick.py`, `crates/debug-core/src/algorithms.rs` (new), `swap.rs` (`mpc_full`), `crates/debug-app/src/ros.rs`, `ui.rs`; `docs/switches.md` |
| 4 | `tools/demo_all.sh`, a new test in `tools/tests/`, `crates/debug-core/src/swap_request.rs` (new), `crates/debug-app/src/main.rs`, `ui.rs`; `docs/hud.md`, `docs/features.md` |

## Alternatives

The merge table above covers how to show the world. Also weighed:

| Alternative | Verdict | Why |
|---|---|---|
| A new app, not `debug_hud` | no | `debug_hud` already has the 3D, the plan and NDT |
| Our own world reader | later | draws the file, not the simulation, so spawned posts would be missing; the picture closes that gap |
| Only the map and cubes, no picture | possible | cheapest; Ben asked for Gazebo's 3D view |
| A compressed picture | no | a JPEG must be decoded; raw is local and `picture.rs` reads it |
| Keep `--debug` as the only way to the window | no | Ben's single screen should be the default |
| Remove `--debug` | no | `pick.py`, tests and docs use it; its switches are still useful |
| SWAP by ROS service | no | a restart button on the graph |
| SWAP inside each algorithm | no | a new mechanism in each one |
| Goals from a separate node | no | more parts; same contract change |

## Risks

| Risk | How bad | What we do |
|---|---|---|
| The window can send goals on a real network | high | three gates; off by default; `developer:=true` only from `demo_all.sh` |
| The picture slows the simulation: rendering shares a thread, and the lidar with it | medium | S1 before building; `chase_camera:=false` |
| `test_lot` doesn't render as the source says | medium | S1 first; the fallback is the road world |
| A fixed view isn't what Ben meant by Gazebo's 3D view | medium | show him early; `--gazebo` stays; S2 |
| A window that hides why the car stopped | medium | the strip is always on |
| `debug_hud` grows too big: `ui.rs` is already about 1,500 lines | medium | panels in their own files; logic in `debug-core` |
| The Orin's run changes by accident | medium | developer off keeps today's layouts; a test says so |
| Slow restarts make SWAP unpleasant | low | measure; stage B only then |
| A restart leaves stray processes | medium | process groups; a time limit; say so on screen |
| Cancel through the hidden service fails from rclrs | low | test early; else an action client |
| Reversing Ben's "Try Bevy first" | low | D1 asks him directly |
| Two views of the truth: list versus launch | low | switches come only from `pick.py`; asked and alive are shown |

## Tools Ben pointed at (5 Nov)

Read from their GitHub pages only; not built or run.

| Repo | What it is | Licence, activity | Use here |
|---|---|---|---|
| [ros2_bevy](https://github.com/rydb/ros2_bevy) | an experimental simulator on Bevy and Rapier | MIT; last push Sep 2023; 15 stars; main features still TODO | not usable as it stands |
| [ros2-impulse-examples](https://github.com/open-rmf/ros2-impulse-examples) | ROS workflow examples with `bevy_impulse` and rclrs | no licence shown, so no code can be copied; last push Jun 2025 | not usable as it stands; not a window |

## Questions for Ben (proposed; awaiting Ben)

| # | Was | Question | Proposed |
|---|---|---|---|
| D1 | D6 | Raylib first, and Bevy only on the test above? You picked "Try Bevy first" | raylib first: the merge looks possible |
| D2 | D3 | Gazebo's 3D view as the chase camera's fixed picture? | yes; free look later if it limits you |
| D3 | D2 | RViz off by default, once click-to-go works? | yes; `--rviz` brings it back |
| D4 | D7 | `demo` opens the HUD and map only when asked? | yes, `--car-screens`; the status strip covers what they showed |
| D5 | D5 | The task's name | `pixi run demo`, no new task |
| D6 | new | What should the ICP panel show? | status and a trail; KISS-ICP has no map |
| D7 | new | SWAP restarts the whole simulator at first, and the car goes back to the start? | yes |
| D8 | new | Keep two presses for a goal in the simulator? | yes, as the map window |

## Related

[One car](2026-11-04_one-car.md), [the weak links](2026-11-04_weak-links.md),
[the EGM review](2026-11-05_egm-review.md), [the debug HUD's design](2026-10-29_debug-hud.md),
`docs/hud.md`, `docs/switches.md`, `docs/foxglove.md`, `tools/demo_all.sh`, `tools/pick.py`,
TODO 73, 87, 90, 107.
