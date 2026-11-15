# TODO

The current work list, newest first. Tick items off as they land; each tick = a commit.
Ben's messages, word for word: `docs/history/messages-*.md`. What is finished:
[todo-done](docs/history/todo-done.md).

## What is next

Ben, 14 Nov night: "chiping througth the list 166-173 and then the 155, 156, 157, etc".

| Order | Next step | Row |
|---|---|---|
| 2 | Ours scores as well as stock: 3 runs each. 14 Nov: ours 4/4 on the lot after the cusp fix; the campus waits on the costmap fix | 167 |
| 3 | The wiggle: each follower's own swing; the EKF change on the car | 168 |
| 4 | The lane line for `aav_track`'s trackers and Nav2's MPPI (`mppi_lag` done) | 169 |
| 5 | The things: a moving thing in the sim, then Gazebo | 170 |
| 6 | The last gaps on the audit list | 171 |
| 7 | The edges smoothed: one start command per job, errors, docs, a start page | 172 |
| 8 | On the car, with the hardware lead | 173 |
| 9 | Reverse everywhere; the costmap in tiles (design first) | 174, 175 |
| then | The lane finder as a pose sensor; the things' camera and radar; record and score a car drive | 155, 156, 157 |
| last | The Rust made more Rust-like, once feature complete | 176 |

## Release candidate 0.4-rc1 (Ben, 14 Nov night)

Ben: "We are still in beta stages for sure but we want to start thinking about refinement like 0.4rc1"; "refining the project towards a release candidate. Something that is fully function and complete and having the edges smoothed out." The candidate's stack is ours (`stack:=ours`, Rust from planner to follower), with Nav2 one switch away. It passes when every row is ✅:

| # | Gate | Status |
|---|---|---|
| 166 | **Every stack runs**: `sim-check` passes on `stack:=stock` and `stack:=ours`, on our simulator and Gazebo | ✅ 14 Nov: `sim-check --obstacle --stack` stock and ours, both simulators, every check passes (ours stops 0.91 m short of the box, stock 0.23-0.34 m) |
| 167 | **Ours scores at least as well as stock** on the lot and campus tours, 3 runs each (`measure controllers --stacks`; TODO 164) | 🔧 14 Nov, 1 run each, reversing on, a busy Mac: the lot, ours 3/4 (missed "turn back": stuck at the cusp), Nav2 4/4; past the box, both; forward legs alike (ours 0.09-0.12 m off the path, Nav2 0.07-0.76). Campus: ours 0/3, every time `mppi_lag` stopped for a costmap 0.84 s old (our costmap does the 41-million-cell global map on the local one's thread); Nav2 2/3. Fieldhouse (18 GPS waypoints): Nav2 2/18 (stuck from a standstill after a sharp waypoint turn), ours 0/18 (the costmap). After the cusp fix (mppi_lag's control cost for its correlated noise; 14 Nov night): ours 4/4 on the lot, turn back in 29 s and 0.15 m off (Nav2 59 s, 0.76 m), home 59 s; past the box 1/1. Next: the costmap's threads and bounds, the route's waypoints, then 3 runs on a quiet machine |
| 168 | **No oscillation** (Ben: "don't forget reverse and no occelation"): the wiggle under 5 cm on the campus road, both stacks (TODO 165: the GPS noise in the pose, our navigator keeps its path) | 🔧 14 Nov: the global EKF trusts each GPS fix less (x, y process noise 0.05 to 0.005): steering changes side 0.39 to 0.30 a second, worst EKF error 0.53 to 0.32 m, the swing itself unchanged at 0.089 m (RPP, our sim, 2 runs each). Left: each follower's own swing; check the EKF change on the car (its odometry is 17 % off loaded) |
| 169 | **The followers steer along the road ahead's line** (the lane centre), not only its speed | 🔧 `mppi_lag` done 14 Nov: with a lane map its `LaneCritic` keeps the outline inside the lane (our simulator, a goal 1.5 m off centre: 0.05-0.13 m over the line, never near the kerb; 0.95 m over and onto the kerb without). Left: `aav_track`'s trackers, Nav2's MPPI (docs/mppi-lag.md, The lane) |
| 170 | **The things in the behaviour planner**: wait for what moves, go round what's parked (decides TODO 162). Built (`things:=true`); 14 Nov, our sim: a parked box driven round 2.0 m clear, no APPROACH(obstacle), stop zone quiet (`sim-check --behaviour-things-only`). Left: a moving thing, Gazebo, the car | 🔧 |
| 171 | **No switch leaves a gap**: reversing everywhere, BackUp, IsPathValid, respawn, age checks, `cameras:=false` in nightly CI | 🔧 most done 14 Nov |
| 172 | **The edges smoothed**: one start command per job, clear errors, the docs pages shrunk, a "start here" page for the release | ⬜ |
| 173 | **On the car** (with the hardware lead): the seven blockers (158), then ours driven, recorded and scored | ⬜ |
| 174 | **Reverse everywhere** (Ben: "reverse is important"): every stack reverses where the plan says, through cusps, on the lot and the campus; `reversing:=true` the sim's default | 🔧 14 Nov: `mppi_lag` through cusps fixed (its control cost); ours drives the lot's turn back in 29 s (Nav2 59 s) |
| 175 | **Three levels, fixed chunks** (Ben, 14 Nov: "I like having hte 3 levels with fixed size map chunks"; "Zoomed out is routes, middle is how we get from cube to cube and finally close up is obstical avoidance"): the **route planner** (zoomed out: the street graph, A*; Lanelet2 later), the **path planner** (middle: 64 m chunks at 1 m and 0.25 m, ~100 m ahead along the route), the **local planner** (close up: the 0.1 m costmap from the lidar's cubes, obstacle avoidance), the **behaviour planner** across all three. The cubes also become the NDT map, in the same chunks, moving things left out. Ben, 14 Nov: "NDT can give us the stationary things ... then all we need is occupancy graph to track poepole": the saved map is the background; live cubes not on it are the things to track (with a cube of slack for pose error); the map also replaces OSM as the middle level's map. Safety never relies on the subtraction: the stop zone and close-up costmap see every point. Order: plan ~100 m ahead; chunk the map; cubes into the close-up costmap; NDT in chunks | 🔧 14 Nov: each costmap on its own thread, only the changed area redone (campus global 500 ms to 0; ours now drives the campus roads). Next: the design (EGM), for Ben |
| 176 | **The Rust made more Rust-like** (Ben: "once we are confident that we are feature complete"): a pass over every crate against the Rust API Guidelines | ⬜ after the rows above |

## The whole car: every design merged (14 Nov)

Ben, 13-14 Nov: "a review of everything with the plan of merging everything into the best version possible ... what am I missing". The page: [the whole car](docs/designs/2026-11-14_the-whole-car.md). His words: `docs/history/messages-2026-11-14.md`.

| # | Task | Status |
|---|---|---|
| 154 | **The road ahead** (`/world/road_ahead`): the line to follow (the right lane's centre), hard and soft edges, the speed limit and stop points from `stop_law.py`, and an end the car may not pass; the boss (`aav_behaviour`) is its only writer; Nav2's followers and ours read it; in the sim first | 🔧 14 Nov: the message, the writer, ego holds Nav2 (and so our followers) to it; sim-check passed sign and goal with it on. The point driver (R5) too: its line A -> B makes the strip. The screens show it: the dev window, and the in-car HUD ("ROAD 0.6 m/s, SIGN IN 12 m", through look_ahead). The lane line, 14 Nov: with `lane_map:=world` the line is the right lane's centre and has edges (`lane_map.py`, a `lanes.yaml`: the road world's and the campus's; Lanelet2 doesn't import on a Mac). Seen in the road world: on the centre while /plan swung -0.6..1.9 m. Next: the followers (`mppi_lag`) reading the line and its edges; the campus untested live |
| 155 | **The lane finder as a pose sensor**: its offset in the lane, against the lane map, into the EKF as a sideways correction; the lane driver stays as a challenger | ⬜ after W4, and the wiggle's tests (149) |
| 156 | **The things** (`/world/things`): camera-to-lidar calibration; the cubes grouped; a small tracker (a stock one first, licence checked); names from the camera's boxes; the radar's speed per cluster; the list remembers what the lidar saw on the way in (TODO 93 and 103's Part B) | ⬜ |
| 157 | **Record everything, score something**: `tools/record.sh` gains pictures (compressed), cubes, radar and the locks; one car drive scored; shadow mode's stage 1 (TODO 108) | 🔧 14 Nov: `record.sh` keeps what the stack saw and decided (the front camera as JPEG, boxes, sign, lanes, radar, cubes, locks, the road ahead, behaviour, ego, NDT); 28 s in the sim = 11 MB, 53 topics. `tools/score_drive.py` scores any bag (`scorecard.py`, tested): a 25 m point drive in the sim scored clean but for TODO 163. Next: score a car drive |
| 158 | **The car's seven blockers** in one place (weak links, section 1): the firmware timeout (45), the brake follows a wish, wheel speed seen (60), clocks checked (a `car-day.md` step), the steering's centre (67), a truth for "1 m" | ⬜ with the hardware lead |
| 159 | **A safety case**: one page per hazard: what stops it, the number, who measured it; actuator faults on purpose in the sim | 🔧 14 Nov: [the safety case](docs/safety-case.md), 12 hazards, every number from the sim, bench or QEMU (none from the car yet). ⬜ actuator faults on purpose |
| 161 | **Our stack up to Nav2's and Gazebo's standard** (Ben, 14 Nov: "so we can drop their work and have our own complete stack"): the gap list and its order, [audit](docs/history/audit-2026-11-14-own-stack-gaps.md). First: clippy and fmt checked; a CI job with our simulator; age checks on `/tf`, the costmap and odometry in every Rust node; time limits in the navigator; a watchdog on our nodes | 🔧 14 Nov: clippy and fmt in `tools/test.sh`; costmap age in mppi_lag and our planner; odometry and map -> odom age in our costmap; `/tf` age in our planner and navigator; time limits in the navigator; our nodes respawn; `cameras:=false` and `sim-check --no-cameras` (the CI job itself next, once run here) |
| 162 | **The behaviour obstacle leg fails** (`sim-check --behaviour`, 8 Nov and 14 Nov, Gazebo and ours): with `plan_around:=false` the car still steers round the box, so neither APPROACH(obstacle) nor the stop zone fires. Likely the local costmap still sees it (unproven). The design wants a parked box passed and a person waited for: decide what the leg should expect | ⬜ |
| 163 | **The simulated car runs 14 % over its command** (found by the scorecard, 14 Nov): ego asked at most 1.0 m/s, the sim's truth reached 1.145 m/s (the speed loop's overshoot, `speed_loop` default). The road ahead's limits assume the car goes no faster than told: tune the loop, or give the stopping law the overshoot; check the spine's loop on the car the same way. Scored again with `speed_loop:=smooth`: top speed 1.00 m/s, no overshoot; but over the last 1.5 m before B the car slowed at about 0.3 m/s² while the road ahead's limit fell faster (2.7 s over it, never past the end, stopped 0.22 m from B). Likely both loops brake only weakly past their 0.2 m/s coast band (`brake_gain` 0.5): unproven | ⬜ 14 Nov, the car model alone (`car_model.py`, ego ramping 0.5 m/s² up and 1 m/s² down): `default` tops 1.144 m/s for 1.0 (+14 %) and 2.70 for 2.5 (+8 %), 95 % in 4.3 s, stops in 1.27 m; `smooth` 0.985 and 2.44, 5.7 s, 1.17 m; `lqi` 0.998 and 2.48, 5.0 s, 1.21 m. So the overshoot is the PI loop's, not the sim's. Proposed: `smooth` (or `lqi`) as the default in the sim and the spine, after a car test on stands; until then the stopping law should allow for +14 % |
| 164 | **Score "all ours" again** (`navigator:=own planner:=own costmap:=own tracker:=mppi_lag`): until 14 Nov `mppi_lag.yaml` set `costmap_topic`, which beat the launch file's choice, so with `costmap:=own` mppi_lag read Nav2's costmap topic, which nobody published, and drove blind to its costmap. The 8 Nov all-ours scores (lot 7 of 12, campus 4 of 6) were with that. Fixed and tested; sim-check all ours with `--plan-around` now passes (2.21 m clear of the box) | ⬜ re-score |
| 165 | **The wiggle, what's left** (149's tests): the GPS's jitter through `ekf_global` (0.09 m sideways every 0.2 s in the sim; with the truth as the pose RPP flips half as often): follow in `odom` and apply the map's corrections slowly, or trust each fix less; `mppi_lag`'s flip a second (its sampling); our navigator keeps a valid path as the trees now do | ⬜ |
| 160 | **Shrink the docs and this file**: pages merged or moved to `docs/history/`; TODO's finished sections moved out; a short "what is next" at the top | 🔧 14 Nov: this file's finished rows and dated logs moved to [todo-done](docs/history/todo-done.md) (979 lines to about 560), "What is next" at the top; the pages still to do |

## One shape, and lanes as soft costs (13 Nov)

Ben, 13 Nov: "How can we make all of this fit together better", and lane lines "it can cross but only if it needs to and to stay on the right side". The page: [one shape](docs/designs/2026-11-13_one-shape.md).

| # | Task | Status |
|---|---|---|
| 148 | **The lane layer**: drawn per route as a ramp (lowest at the right lane's centre) into a mask for Nav2's keepout filter on the global costmap, so the planner keeps right and MPPI follows; compared with a lane-centre planner (`path:=lane`); then a `lanes` layer in `aav_costmap` with the camera's lines | ⬜ after D2 |
| 149 | **The wiggle, alone** (TODO 124): truth pose; one plan, no replanning (the single-goal tree replans at 1 Hz); the command before and after the smoother; score the swing in metres. **14 Nov:** the replanning moved RPP and `mppi_lag` and made loops; the GPS's jitter halves RPP's flips; the smoother nothing; MPPI's 6-8 cm is its own. The trees keep a valid path: RPP 0.12-0.13 → 0.09 m, `mppi_lag` 0.14-0.17 → 0.10 m, no loops ([trials](docs/trials.md#the-wiggle-14-nov-todo-124-149)). Left: the slow map correction (165) | ✅ |
| 150 | **The league** (`stacks.yaml`, `stack:=best/car/stock/ours`, `pixi run measure league`): one champion per level, challengers scored on the same routes and recordings; a swap needs a margin on 3+ runs and a person's yes | 🔧 14 Nov ([page](docs/league.md)): `config/stacks.yaml` (aav_description), `stack:=` on `sim.launch.py` (a switch given by hand still wins), its tests; checked offline. Next: a sim run of each stack, `stack:=` on `car.launch.py`, `pixi run measure league` |
| 151 | **Placeholder cameras** (`sim_cameras --placeholder`) so GitHub's nightly drive check runs our simulator, not Gazebo; then real pictures on Mesa's software renderer | ⬜ after D7 14 Nov: `cameras:=false` and `sim-check --no-cameras` (no pictures at all) as the first step |
| 152 | **Our planner faster** (cache each square's check per heading, outlines per heading at start-up, several cores: 180 ms against Nav2's 150 ms on long campus plans) and **MPPI with lag** (slow earlier near the goal with `stop_law.py`; fewer futures; timed on the Orin) | ⬜ |

## Saturday 7 November: 0.3, the algorithms know about lag (MPPI)

Ben, 7 Nov: "have the algorithms know about lag". MPPI out of Nav2 into our own Rust driver, its model with the actuators' lag; Nav2 keeps planning.

| # | Task | Status |
|---|---|---|
| 132 | **The lag-aware MPC** (`tracker:=mpc_lag`, [results](docs/trials.md#lag-aware-mpc-7-nov-mpc_lag)): steering and set speed planned through the spine's speed loop, throttle, brake, the steering motor's rate and linkage, dead times and the commands already sent; one QP a tick, on the shared car model (`car_model`: its numbers, speed loops and estimate; `lag_model.step` is its maths on arrays, held to it within 1e-9). Says `/drivers/nav/lag_aware` (latched) so ego's lead stays off. `lag_scale:=2` (Gazebo) for the robustness test | 🟡 built and scored. Pretend car: no speed overshoot; lags x2, the 2 m/s U-turn 0.17 m off (MPC 0.36). Our simulator, lot tour, 3 runs: **6 of 12 legs** (MPPI 8, RPP 9, `mppi_lag` 11): the quickest and closest on the legs it drives, 5 ms a tick, but 0 of 3 on the U-turn by the building (collision_monitor stops it on the planner's path; no aav_track tracker reads the costmap: mpc and pure pursuit fail it too). Gazebo, 1 run: lags as told **4 of 4** (MPPI 2, RPP 2); doubled 2 of 4 (MPPI 3, RPP 3). Left: keep off obstacles (costmap cost in the QP, or slow near them, as RPP); move to Rust before the car (worst tick near 50 ms on the Orin, estimated); model the LQI and the ESP32's open loop |

## Saturday 7 November: the hardware debug plan

Ben, 7 Nov: the Rust replaces the C++, WiFi pages too, with JSON instead of streaming text; graphs of steering, brake and throttle in `hw_panel`; debug one part of the car at a time.

| # | Task | Status |
|---|---|---|
| 126 | **The hardware debug plan** ([brief](docs/designs/2026-11-07_esp32-debug-link_brief.md), "The plan now"; [second review](docs/designs/2026-11-07_esp32-debug-link_review-2.md)). Steps: today's log; USB cable and Pololu's tools; two small fixes and paper sheets; the Pi ready; Rust with no radio; record the joystick path, PASS or FAIL; return wire and graphs in `hw_panel`; the steering page; timing test and WiFi pages. Replaces the 6 Nov [bench link](docs/designs/2026-11-06_bench-link.md) | ⬜ proposed, order chosen by Ben. Nothing built. 🔒 the hardware people: a USB cable, the current limits, the return wire, where the steering and brake sensors connect |

## Wednesday 4 November: the weak links, for Thursday's pick

Ben, 4 Nov: "think about the areas of this car and what is the weakest ... ideas on how to improve each section of the driving experience", then "save this full list". The list: [the weak links](docs/designs/2026-11-04_weak-links.md) (every area, the evidence, the ideas, the papers). Words: `docs/history/messages-2026-11-04.md`.

| # | Task | Status |
|---|---|---|
| 100 | **The clocks, walk the fence, the site file** (keep-in, keep-out, speed zones, named places, missions; stock Nav2 filters) | ⬜ **picked** 4 Nov (Ben: "The clock yes"); the fence and the site file go with the first lot visit |
| 101 | **The brake follows a wish** (control plan step 1: feed-forward + PI for throttle and brake) | ⬜ **picked** 4 Nov. Needs a driven-wheel speed (TODO 49b), then `pi` and test 4.3 **on the ground**: never `pi` on stands without that speed. 5 Nov: the geofence holds programs in every mode until that mode's stop is measured (`pi_loop_decel`) |
| 102 | **A behaviour state machine + a speed profile on the path** ([design](docs/designs/2026-11-05_behaviour-and-speed.md)) (DRIVE → APPROACH → STOPPED → WAIT → GO; PERSON_AHEAD; "stop at X" for the sign, the fence and the goal; jerk-limited speed along the path) | ⬜ **picked** 4 Nov; corrected 5 Nov: a speed cap on Nav2, not one profile for everything; a small node (proposed); no radar targets. The stopping law moves to [how the car stops](docs/designs/2026-11-05_how-the-car-stops.md), which comes first |
| 103 | **The tracker** ([design](docs/designs/2026-11-05_tracks-and-memory.md)): an ID, a speed and a class for each radar cluster and cube group; a person means stop and wait | ⏸ **paused** (Ben, 5 Nov): after the stopping design, and once the radar is on the car. What must be true to resume is in the design; the radar's mode is not a switch |
| 104 | **Research A** ([design](docs/designs/2026-11-05_knows-itself.md)): the car identifies itself (delay, lag, braking, with error bars) and a CBF safety filter in ego, fed by the site file's fence | ⬜ **picked** 4 Nov. Staged: the measured numbers (TODO 52b), the closed loop, the monitor, then the filter beside the locks. Where it goes: proposed before twist_mux, not in ego (K1, Ben's) |
| 107 | **One developer window** ([design](docs/designs/2026-11-05_one-window.md)) (Ben, 5 Nov: "a single screen just for the developers and a list of algo so you can swap"): `demo` opens Gazebo headless and **one** Rust window: 3D, the Nav2 map you click on, NDT and ICP, the algorithm list with SWAP. RViz off unless `--rviz`. The street map and the HUD stay in the car. Not Gazebo's renderer compiled into Rust | ⬜ 0.3; the design corrected after the EGM review. Proposed: raylib, with Gazebo's real picture as a panel; Bevy only on a written test (D1, Ben's). Replaces TODO 73's merge into the cockpit |
| 108 | **Shadow mode** (research thread 3, step 1): while a person drives, Nav2 and the other automatic drivers keep publishing under a SHADOW lock (twist_mux 60), the bag records what each would have done, `shadow_score.py` scores the disagreement | ⬜ **not picked**: proposed, in stages ([design](docs/designs/2026-11-05_shadow-mode.md)): recording and a score on real bags first; a live lock only if that shows a need (S1, Ben's) |
| 111 | **The radar driver can die on a short frame**, in today's default mode too: `unpack` returns `None`, the generator raises, and the publisher reuses the dead generator (`TI.py:117, 125`; `pc_publisher.py:110, 143`). Read, not run | ⬜ |
| 112 | **`vehicle.yaml` has three duplicate keys** under `vehicle:` (`centre_of_mass`, `tyre_grip`, `encoder_counts_per_rev`). The values match today, so harmless until someone edits only one: delete the copies | ⬜ |
| 113 | **The stop-sign watcher lets rear-camera boxes trip the lock** (`stop_sign_watcher.py:34`). Reviewer's word, not checked | ⬜ |
| 114 | **The developer window in Bevy** ([design](docs/designs/2026-11-05_own-simulator.md)) (Ben, 5 Nov): his layout, in `ros/hud/aav_dev_window` | 🔧 live on Gazebo, with goals through its gates (5 Nov). Next: a slower laptop, cancel a goal, the algorithm list, SWAP |
| 115 | **Our own simulator in Rust and Bevy** ([design](docs/designs/2026-11-05_own-simulator.md)) (Ben, 5 Nov: "we want to replace gazebo"): every sensor Gazebo gives, beside Gazebo until it wins on the same routes | ▶ 7 Nov: the core is built (row 134); the cameras on their own branch |
| 116 | **Does the car stop if Ben's MacBook drops off?** The LLM on the Mac sends Nav2 goals; a goal may carry on with the Mac gone. Check, then a heartbeat if so Screen heartbeats now exist for tests (`watch_screens:=`, `docs/simulation.md`), refused on the real car; what the car does about a lost Mac is still to decide | ⬜ before the LLM drives the real car |
| 119 | **The foot-tall kerbs may be invisible to Nav2** (Ben, 5 Nov: the lots have kerbs about a foot tall; `nav2.yaml:366` sets the cubes' floor at 0.3 m, and `docs/learn-lidar.md:141` ignores kerbs). Read, not tested. Check the flat scan's floor, then test a 0.3 m kerb in the simulator. The map may have to hold them | ⬜ before autonomy near a kerb |
| 120 | **The campus world and the one map** (Ben, 5 Nov): OpenStreetMap plus his measurements gives surfaces, kerbs and buildings, anchored by lat/lon to `navsat.yaml`'s datum, each cell saying where it came from. Generates Gazebo's world and NDT's practice map; Bevy draws it with pavement, concrete and grass. Real drives replace it. No RTK (Ben, 5 Nov): the rough map guides the stitching of a real drive, and NDT against the finished map replaces GPS's offset. Tape two or three long distances as the check. A drive only suggests map changes and a person accepts them (Ben, 5 Nov): the window shows them on the map to approve, and that review doubles as the winter record | 🟡 world, one costmap and NDT's guess built and driven in Gazebo (`docs/campus.md`); drive fitting, the GPS offset and approving changes not built; which lot and his measurements: Ben |
| 121 | **NDT's score floor on the campus** (found 5 Nov): 1.5 there, 2.3 in the test lot. Measure how high a wrong pose scores before `ndt_ekf` on the campus | ⬜ |
| 122 | **Reversing on long drives** (found 5 Nov): with `reversing:=true`, Nav2 backed the car a long way, at 0.5 m/s, to a goal behind it. Should reversing cost more, or stay short? | ⬜ decision |
| 124 | **Why the car wiggles** (Ben, 5 Nov; N7): the turn rate flips about once a second on MPPI and RPP. Not the steering lag model (the same without it). NDT in place of GPS's jitter: no clear change. **14 Nov (149):** a new plan every second (fixed: the trees keep a valid path) and the GPS's jitter (165); MPPI's own 6-8 cm swing stays | ✅ |
| 125 | **Read their Humble bags** (after the drive as is, 6 Nov): rename `/velodyne_points`, `/imu/data_raw`, `/aav/gps/fix` to ours, so `ndt-map build` and replay can use them ([tonight](docs/car-evening.md)) | ⬜ after the drive |

## Tuesday 3 November: 0.2 polished for the meeting (4 Nov), then the car

Ben, 3 Nov: "Tomorrow is meeting day and we want to have 0.2 fully polished and documented ready to go." The plan: [the brief](docs/designs/2026-11-03_release-0.2-brief.md), "Recommendations for today". Words: `docs/history/messages-2026-11-03.md`.

| # | Task | Status |
|---|---|---|
| 93 | **The cubes let the car into a post once** (`sim-check --lidar-obstacle --voxels`: 1 of 3 runs, 1.30 m in; 30 Oct it stopped 0.44 m before). Suspect: the lidar's new place (1 Nov, x 0.90 → 0.49 m, z 1.80 → 1.52 m) | ⬜ before `voxels:=true` goes anywhere near the car |
| 94 | **Drive-to-point stops 0.61 m past B** by the car's own EKF, once a run (limit 0.5 m; by Gazebo's truth within 0.46 m). GPS noise or the GPS's new place? | ⬜ |
| 96 | **A model builds our own algorithms** (Ben, 3 Nov: "let a fable running on max build our own algorothms with everythign we have done and the data we collect"). What it needs from us: the recordings (session 10: lidar, cameras, wheels, IMU, GPS, every command), a ground truth (RTK GPS, the tape), and the trials' scoring (`measure controllers`, `measure atob`, the localizer scores), so each algorithm it writes is a trial beside the default, scored on the same routes and the same bags | ⬜ after car day: the data first |
| 95 | **NDT-EKF the default of the test-lot demos** (`demo`, `pick`), which already need its map | ⬜ 0.3 |
| 90 | **One screen per concept** (Ben, 3 Nov: "the gazeboo with nav2 selection with cubes and stuff all as a single screen and then hardware as a seperate screen. Just the stuff we control"): **Drive** (Gazebo or the camera, Nav2 and its choices, the cubes, localization, the algorithms) and **Hardware** (only what we control: the actuators, the sensors, calibration). The debug screen's parts move into Drive; the HUD and the map stay in the car | 0.3: today's step is 87 |
| 97 | **The 0.2 release notes, as an advertisement** (Ben, 3 Nov: "a release note like an advertisement of what is done, what is going to be done. How to use the stuff we have"): `release-notes.md`'s clean look; a short feature list; the names spelled out (NDT, MPPI, ICP...); each screen's picture, its command, what it means and how to use it (the hardware test in front of the car); the cockpit explained; what's left | 🔧 3 Nov |
| 98 | **The faint car behind every screen** (Ben: "there was an outline of the car in the background before. I liked that"): today only the HUD draws it; the map, the debug screen and the hardware panel too, and new screenshots | 🔧 3 Nov, `faint-car`: drawn on all four in Initial D, one painter (`hud_app::paint::faint_car`), the contrast test extended (385 cargo tests). ⬜ the four screenshots: the Mac's screen was locked |
| 91 | **PR #21**: close #20, open #21 from the fork's `main`, after the full tests | ⬜ last |
| 92 | **Ben, tonight**: the people named; the hardware lead's OK for A8 and H1; the restore test's spare SD card | ⬜ Ben |

## Sunday 1 November, night: toward 0.2

Ben, 1 Nov: "Lets maybe aim at this being 0.2 version ... I think we are trying to get to 0.2 so maybe we merge in all the code. Do some testing and then extend the whats new document to include everyhting?"

**0.2 = sim-complete, car-ready. 0.3 = it drove.** After the review ([the road to 0.2](docs/designs/2026-11-01_road-to-0.2.md)), Ben, 1 Nov: split car day; a **small** 0.2; keep every idea for 0.3 and later.

### Done in 0.2

| # | What |
|---|---|
| 59, 62 | Reversing and the U-turn: the lattice is the default where the car may reverse; Hybrid-A* forward only |
| 63 | The drive-test plan: maps, recording with the lidar, scoring the trials (tests.md 4e, 4f) |
| 67, 70 | The steering calibration below the screen (link v3: limp, jog; raw counts; `SteerCal`), and the hardware panel's page with the lock-to-lock check that stops on resistance. Untested on the car |
| 69 | Fewer pixi commands: 89 → 45, no aliases |
| 72 | The car tests, written: [Know the car](docs/know-the-car.md), one page of order and stop rules |
| 73 | The Lichtblick cockpit, built (screenshots wait for an unlocked Mac) |
| 75 | `tools/car_setup.sh <role>` and [Car setup](docs/car-setup.md) (one computer); STOPs for a pending reboot, a wrong hostname, an upgrade too early. Untested on a car computer |
| | The screens in Catppuccin: HUD, map, debug screen, hardware panel |
| | [Car day](docs/car-day.md) split: **Day A** destroys nothing, **Day B** upgrades; the people, the vault, sign-offs, the brake on stands, the joystick-off test |

### Left for 0.2 (five rows)

| # | Task | Status |
|---|---|---|
| 68 | **The car-ready checklist**, every box ticked ([car-day.md](docs/car-day.md#ready-for-car-day-the-02-checklist)): `pixi run test`, `car-day-check` 0 missing, `esp qemu-check`, the `car_setup.sh` dry runs, the merges | 🔧 1 Nov: 1513 tests, 0 failures, before the review's fixes |
| 74 | **The first moves' gaps** (the joystick, on the Motor Pi alone, `machine:=motor_pi_alone`; [A9b](docs/car-day.md#a9b-the-joystick-on-stands)). (a) **build a joystick-alive check**: the joystick's messages stop changing, or the pad is gone → stop (`joy_node` repeats the last state while the receiver is in); (b) a **throttle cap** in the firmware for the first moves (the ESP32 can't see speed: today only DAC 700 of 1023), Ben and the hardware lead decide the number; (c) **the Rust firmware only** for the first moves (the C++ has no timeout); (d) the joystick's button and axis numbers. **The Kar-Tech stop is the main stop**, held by the e-stop person | 🔧 hold-to-drive, the stale-command brake, the 100 ms heartbeat brake (Rust) and `max_speed` exist. ⬜ (a) to build; (b), (d) on car day |
| | **Questions to the hardware lead, before the day**: H1 (is CORTEX the Motor Pi?), C1 (CAM1 at .108 or .102?), `.211`'s job, who owns the router (fixed addresses) | ⬜ ([session 3](docs/car-day.md#session-3-questions-for-the-hardware-lead)) |
| | **The people named**: hardware lead, e-stop person, safety driver, laptop person, approver, with phones; and the shared password vault (Q34) | ⬜ ([the people](docs/car-day.md#the-people)) |
| 77 | **The 0.2 release notes**: a "Release 0.2" section in [release-notes.md](docs/release-notes.md) (the stub is there); `whats-new.md` stays as history | ⬜ after 68 |

### 0.3 and later: every idea kept

Ben, 1 Nov: "I do like the 23 ideas? lets keep them around like the diffusion path finding. I want that in."

| # | Idea | When |
|---|---|---|
| | **Car day itself**: Day A, then the car tests and the map (0.3 = it drove) | after 0.2 |
| | Day B: **the Pis to Ubuntu 24.04** (the Motor Pi after the Rust ESP32; LOCALIZATION after its machine file) | after a restore is proven |
| | Day B: **the Orin's reflash** to JetPack 7.2 (once 7.2 is confirmed for AGX Orin), the lab desktop after it | after the Orin's full image |
| 75 | A machine file for the localization Pi (where are the IMU and GPS?) | before its upgrade |
| 78 | **The diffusion planner**: in 0.2 as a trial (TODO 83); for 0.3, more plans from the model (63 % → 90 %), the U-turn, the Orin (TensorRT) | 0.3: TODO 133 (8 Nov) |
| 78 | **The talking model**: a vision-language model that explains, never drives | design first |
| | **The Initial D HUD**: the arcade gauge in Carleton red and black | ✅ in 0.2, on every screen (TODO 80, 81) |
| | **The little robot** (a tribute to WALL-E): the map's car, eyes peeking on the panel and the debug screen | ✅ 2 Nov; ⬜ the HUD's eyes, after the Initial D HUD |
| 73 | The cockpit's **algorithm switch**; then **merge the cockpit and the debug HUD** into one (the hardware panel stays apart: it drives) | 0.3 |
| 71 | **The hardware panel's pages**: actuators off, measure, direct control, type in measurements, joystick, speed and distance; the car tests from the panel (72); the calibration page's open bits (70: wheel angles typed in; a pixi task for the panel on the car's network) | 0.3 |
| 76 | **Python speed** on the Orin: measure (`timing-test`), the cubes to Rust first if they miss their budget | car day, then 0.3 |
| 79 | **Brake and throttle setting**: measure, then set, like the steering | after the steering |
| | "Two drive paths": the bridge refuses when a second mux or heartbeat appears | 0.3 |
| | Way B on the car: link version 3 with the uplink (H9), the panel's calibration page | after the Rust with the uplink |

## Sunday 1 November: the ideas before the second PR

Ben, 1 Nov: "do all the ideas. I like them all but car day. Keep that in a list fo things todo".

| # | Task | Status |
|---|---|---|
| 60 | **Car day: measure the wheel diameter by rolling** (Betsi's 0.36 m is PROVISIONAL; loaded it read 0.42 m, a 17 % speed error) | ⬜ car day: roll the car 10 m by tape, count the encoder ticks, `vehicle.yaml` `wheel_diameter` |
| 61 | **Car day: the track, front and rear** (0.65 m and 0.85 m, today one average, 0.75 m) | ⬜ car day: check both; decide whether `vehicle.yaml` needs a front and a rear track |
| 64 | **Set the datum to the test lot** (`navsat.yaml`, a placeholder near Carleton) | ⬜ once the lot is chosen: before the first recording |
| 65 | **The 2024-25 / 2025-26 drive recording** (Humble): find it; if it has the lidar, NDT's map of that lot | ⬜ Ben: who has it |
| 67 | **The steering pulls right: find the steering actuator's ends and centre** (calibration.rs `TODO(Q1)`, `TODO(Q2)`) | 🔧 1 Nov, `steer-cal`: below the screen done: serial frame version 3 (limp, jog), the uplink's raw counts, the firmware's rules (`SteerCal`), `/drivers/buttons/steering_calibration`, `/vehicle/steering_actuator/raw`, `steering_limits`, [the procedure and sheet](docs/actuator-tests.md#find-the-steering-actuators-ends-and-centre). ⬜ the panel's page (another helper). ⬜ car day: way A with Pololu's utility (test 3.0); way B once the Rust firmware and the uplink wire are in. Untested on the car |

## Thursday 29 October: control (Ben, 27 Oct)

Ben: "investigate LQR and maybe collapse all these various controllers into a single
controller ... closed loop". The plan: `docs/designs/2026-10-27_control-plan.md`; the ideas: "Ideas to keep" below.

| # | Task | Needs |
|---|---|---|
| 0 | **The plan's step 1: the bridge uses ego's acceleration**, feed-forward + PI for throttle and brake (`docs/designs/2026-10-27_control-plan.md`); scored with `measure atob` | Gazebo, bench |
| 0b | **The spine's state machine (Arsh's #15), follow-up:** the board for today's car (`esp32-umod`) has no wheel speed (`SpeedFeedback::None`), so the test mode's "leave above 0.5 m/s" can't fire there. Refusing throttle tests without a wheel speed: done (`spine.rs` turns them into Hold; found by the two-screens review, 29 Oct). (Test requests have no transport yet, so nothing can trigger it today.) Also: the self-test is one tick with no fault | Arsh |
| 2 | **LQR for steering:** from `vehicle_model.lateral()`, scheduled by speed; follows the planner's path (Smac), in place of pure pursuit; scored with `measure atob` and `measure controllers` | 🔧 29 Oct, branch `trackers-tuning`: rebuilt about the path's bend (gains by speed and bend), the command's rate in the design, a 2 s preview of the bends ahead. Fake car's full-lock U-turn: 0.80 → 0.29 m off. Gazebo: 10 of 12 legs, U-turn 2 of 3 (04:18, was 8 and 1 of 3); on a busy Mac (15:23) 6 of 12, one run starved, one **weaved**. Left: rerun on a quiet Mac with the status topic logged ([trials](docs/trials.md#results-29-october-afternoon-lqr-rebuilt-and-mpc_full)) |
| 4 | **MPC for the whole A-to-B:** speed and steering together, with the brake's and steering's limits; scored against 2 and 3, and MPPI | 🔧 29 Oct, branch `trackers-tuning`: `tracker:=mpc_full`, one QP a tick over the next 9 m (aav_track `qp.py`, OSQP's method in numpy), within full lock, steering rate, throttle, brake, and the policy's limits. Gazebo (busy Mac): 10 of 12 legs (MPC 12, MPPI 10), the same times as MPC, 2.6 ms a tick. At 1 m/s it hardly slows for the U-turn (its model sees no need); at 2 m/s on the fake car it slows to swing the steering. Not scored against 3 (LQI) yet |
| 5 | Unfinished from 26 Oct: the QEMU and NaN fixes (`small-fixes`), the bench fixes (`bench-fixes`). The F6 geofence: ✅ 29 Oct, [The geofence](docs/geofence.md) (branch `geofence`). Left for the team: the test lot's polygon once its datum is surveyed (W3); test 4.3's stop with the legacy speed loop `open` (`vehicle.yaml` `open_loop_decel`), until then programs stay held (COASTING); a `sim-check --geofence` in Gazebo | goal circling ✅ 29 Oct, branch `goal-circling-fix` (TODO 17); the rest open |

Every gain still comes from guessed numbers until the car is measured (`docs/measure-the-car.md`).

## 24 October, night: the car as a linear model

Ben, 24 Oct: "Do we have a mathematical model for the car now for the ego? ... a matrix
control thing ... is that something we should consider? ... Maybe second order system".

| # | Task | Status |
|---|---|---|
| 52b | **Refit the model from the actuator tests at the car** | ⬜ car day: the tests print the numbers; `pixi run python3 tools/vehicle_model.py --set throttle_lag=... --set drag=...`; then the speed loop's gains by pole placement (`place_speed_gains`), not before |

## 24 October, evening: the car-day kit

Ben, 24 Oct: "Lets get everything prepared so the only thing left it to get into the car."

| # | Task | Status |
|---|---|---|

**Ready for the car** (the steps are written; each still needs the car):

| TODO | What | Runbook session |
|---|---|---|
| 14 | weigh and measure the car (Gate 1, 30 Oct) | 2 |
| 28 | Cyclone's one network card: the laptop's and the Orin's file | 0, 4c |
| 37b.1, 37c | RF-DETR's engine (`trtexec` from the ONNX), the Pi file, which camera, CPU, delay, calibration pictures | 4d, 5 |
| 38, 46, 49, 50 | the hardware lead's questions: H1-H8, the Jrk wires, TX1, CORTEX, the camera Pi's address | 3 |
| 43a | Foxglove at the car (their bridge holds 8765: ours on 8766) | 8 |
| 45 | the ESP32: backup, the Rust flash, checks 1-14 and 9a-9c, roll back | 6 |
| 30, 38 | CAN read-only: the Arduino's 0x200, 0x201 | 7 |
| T2.7 | bags for the sim-versus-real report | 10 |

## 23 October: what we can do without the car

Ben, 23 Oct: "yes lets use cyclone and lets add this to a list of stuff we did and the
justifications for it", and the list below. Words: `docs/history/messages-2026-10-23.md`.
The reasons for each choice: [decisions](docs/decisions.md#decided-on-22-and-23-october).

| # | Task | Status |
|---|---|---|
| 37c | The camera stream's follow-ups (merged 24 Oct, `aav_camera_stream`, [camera pipeline](docs/camera-pipeline.md); `pixi run -e ml measure stream`, VideoToolbox by default: 27 ms camera to detections, 15 fps, 0.13 Mbit/s for Gazebo's plain scenery) | 🔧 (1) ✅ 24 Oct, branch `wt/camera-mac`: a Mac decodes with GStreamer by default (`decoder:=auto`: `vtdec`, VideoToolbox, else `avdec_h264`, else ffmpeg with a warning), from a new small `gst` environment (`pixi install -e gst`; no re-solve: `pixi lock` only added it and the `calib` one, every other environment's packages unchanged). stream-check: 4 ms decoded, 27 ms to detections (ffmpeg: 160). (2) ✅ 24 Oct, branch `wt/camera-car`: the sensor watchdog watches each stream through its detections (kind `detections`, by `frame_id`: `front_left_stream`, `front_right_stream`; quiet 1 s or NaN stops Nav2), not `/diagnostics` (1 Hz, its rate over 5 s: slower). `measure stream` checks it: OK for 60 s, STALE 1.1 s after the stream stopped. The lane finder gets `image_raw` from the stream (`publish_image`) when the lane part is on. (3) ✅ 24 Oct, same branch: `car.launch.py`'s `camera_stream` part, on in `orin.yaml` (CAM1-PI `192.168.1.108`, `gst-nvidia`, RF-DETR's engine); it replaces the detector part for streamed cameras, feeds the stop-sign watcher and camera_obstacles, and is skipped (with a warning) until the engine is built. Tried with a stand-in machine file on the Mac (two local streams); untested on the Orin. (4) ✅ 24 Oct, `wt/camera-mac`: the stop-sign watcher stops at a distance (4.7 m, the old 55 px with Gazebo's lens) from each camera's focal length (its `CameraInfo`), so it carries over to the 1280x720 ribbon cameras once calibrated. (5) ✅ 24 Oct, same branch: the calibration is ready for the car: `pixi run calibrate-camera` (the board to print, stock `camera_calibration` on the stream from a `calib` environment, the result filed in `config/calibrations/`; [how](docs/camera-calibration.md)); tried with a made-up board stream, the buttons untested. At the car: install the Pi file, the Pi's CPU for two streams, the real delay, calibrate the cameras (`pixi run calibrate-camera`), check the stop at 4.7 m |

## 24 October, afternoon (Ben's message: `docs/history/messages-2026-10-23.md`)

| # | Task | Status |
|---|---|---|
| 46 | The steering-angle and brake-position sensors are probably on the ESP32 (Ben) | ⬜ hardware lead, question H2 (`docs/audit-questions.md`): which pins, which sensors. **Hint from the old code (24 Oct):** steering and brake each run a Pololu Jrk G2 (I2C 0x05 and 0x04), which closes its own position loop from a feedback input; the 2025-26 ESP32 never reads it back, but the 2024-25 `FinalRev` firmware read the steering position from the Jrk over I2C. So the angle and brake position are likely one I2C read each from the Jrks; then the Rust firmware reads them and sends them up the serial link (Gate 3, closed loop) |
| 49b | **On stands, nothing measures the driven wheels** (found 29 Oct, EGM review of the preflight): the wheel sensors are on the **front** wheels (the Arduino, CAN 0x200/0x201), the motor drives the **rear**. On stands `/vehicle/odom` reads ~0 while the rear wheels spin, so no speed cap or stop that reads the wheels can fire there, and the bridge's new speed loop (`speed_loop.mode pi`) would wind the throttle up. Today's stands tests cap the **command** and the car stays on `mode: open`, so they hold; nothing closed-loop may run on stands until this is answered | ⬜ hardware lead: is there a speed sensor on the rear wheels or the motor (the motor controller's tach output, a Hall sensor)? If not, add one before closed-loop stands tests |
| 49 | **Is I2C good enough for the Jrks?** (Ben, 24 Oct) | ⬜ hardware lead: how long are the wires from the ESP32 to the two Jrks, and do they run beside the motor cables? The load is tiny (50 updates a second, a few bytes each), so speed is fine; the risk is noise on long wires (I2C has no checksum and can lock up). The Rust firmware already drives both Jrks over I2C, flags a Jrk that stops answering (fault bits 8, 16) and brakes. If the wires are long or noisy: the Jrk G2 also takes TTL serial with a CRC (Pololu's protocol), a small change in the firmware |

## Working list, from 22 October (Claude works from this; Ben checks it in the morning)

Everything Ben asked for since 21 Oct, in one place. Progress is updated as each step lands,
and each step is committed. Ben's words: `docs/history/messages-2026-10-22.md`. The
proposal's full list: [docs/plan.md](docs/plan.md).

**The night in one table** (22 Oct, 06:10; all in Gazebo, all pushed):

| | Result |
|---|---|
| MPPI | the default; reaches every goal of the lot tour, U-turn included when it may reverse |
| Obstacles | the planner steers round what the radar and lidar see |
| Heading | the global EKF's heading: worst per leg 7-16° (was up to 76°) |
| ego | a jerk limit; the weight model checked at 3 m/s |
| Gazebo | steering within 3% of the model; one node per sensor; `sim-check` nightly in CI (on trial) |
| ✅ Answered | Cyclone DDS: yes (23 Oct). The GPS antenna: already over the rear axle (23 Oct). Reversing: yes (22 Oct) |

| # | Task | Status | Progress |
|---|---|---|---|
| 1 | MPPI as a choice (`controller:=mppi`), and a script that compares it with Regulated Pure Pursuit | ✅ done | `f22dc4b8`; results in `docs/path-followers.md` |
| 2 | ego knows the car's weight (181 kg) and its body | ✅ done | `7e76850f`; the centre of mass is a guess until the car is weighed (task 14) |
| 3 | The proposal as a to-do list with % done | ✅ done | `docs/plan.md`, `b70300d7` |
| 4 | The AAV colour: the email's blue | ✅ done | `da411333` |
| 5 | Email Ben as work lands | 🔧 ongoing | part 1 ~01:30; part 2 ~02:50 (every goal reached; the DDS finding); part 3 ~03:20 (MPPI default; Cyclone); part 4 ~04:06 (round the box; the U-turn; the early push); part 5 04:38 (three-point turns); part 6 05:14 (jerk limit, steering check, nightly Gazebo; the part 4 time corrected); part 7 06:08 (the EKF heading; the antenna mount; part 6 corrected); part 8 06:41 (good morning: the forward U-turn still a coin toss) |
| 6 | MPPI loops at goals | ✅ fixed (`7697778f`), confirming in 8 | it now turns to the goal's heading from 2.5 m out: left turn 28 s, 21.4 m (was 58 s, 47.5 m with a loop); round the building reached. Confirm in the full comparison (task 8) |
| 7 | The corner problem (docs/plan.md M8): the car stops at the building's corner | ✅ fixed | the planner's path passed 0.86 m from the corner. Now the planner's costmap inflates 3.0 m, gently (1.0), so paths keep to the middle of a lane; and it plans turns 1.25 x wider than full lock (`vehicle.PLAN_TURN_MARGIN`), so the controller has steering to spare. Lot run, 02:00: both kept clear of the building, both reached the first goal in 29 s, each reached 3 of 4 goals (were 1 and 2) |
| 17 | The car still sometimes circles a goal it just missed (a different goal each run) | ✅ fixed 29 Oct, branch `goal-circling-fix` | **Why:** the behaviour tree planned a new path every second, also 1-4 m from the goal. There the car runs a little behind on the last bend, and Smac's only path to the goal's heading at its 3.6 m radius is a 21-25 m loop; and each new path resets controller_server's goal checker. Not the tolerances, nor the EKF. **Fix:** `navigate_to_pose_car*.xml` keep the path within 8 m of the goal (`vehicle.final_approach`: 2 planned radii), while it ends at this goal, `IsPathValid` and under 20 s old; 1 Hz elsewhere. Stock Nav2 nodes. The map window waits 25 s there before NO NEW PLAN. **Runs** (Gazebo lot, first leg, 3 each of RPP, MPPI, pure_pursuit, lqr, `--plans`): before, loop plans in 10 of 12 runs and 3 driven (RPP 59 s, 49.3 m; MPPI 64-65 s, 49.5-50.1 m, for 21); after, no loop plan, all 12 at 0.94-0.98 of the route, 23-30 s. Whole tours after: reversing 8 of 8 goals; forward only RPP 4 of 4, MPPI's U-turn failed as before; the box tour passes ([Path followers](docs/path-followers.md#circling-a-goal-found-and-fixed-29-october-todo-17)). Not done: the through-poses trees (Nav2's waypoints go through to-pose) |
| 8 | Run the full comparison again with 6 and 7; update `docs/path-followers.md` | ✅ done | 6 runs overnight (b-f); the page has the fixes, a table of runs and a picture of the best. Pushed 03:18 (815 tests pass) |
| 9 | Make MPPI the default, if it now does as well as Pure Pursuit (Ben: "I thought we decided to go mppi") | ✅ done | in every lot run MPPI did at least as well; it reached the first goal cleanly in all 4, Pure Pursuit looped there in 2 of 3 (it aims only at a point, so it arrives facing wrong). `vehicle.DEFAULT_CONTROLLER`; `controller:=rpp` for the old one. `sim-check --obstacle` passes with it. Untested on the Orin's CPU |
| 10 | **Split `gazebo_adapter` into one node per sensor**, as on the car, so each runs on its own core (Ben, 22 Oct). Rust for any part still late | ✅ done | `gazebo_imu`, `gazebo_gps`, `gazebo_radar`, `gazebo_camera` (one process each); `gazebo_adapter` keeps the car (actuators, wheels, odometry). The IMU works out the car's acceleration from Gazebo's odometry. Each sensor node takes its own `fault.<sensor>`. `sim-check` passes; `sim-check --faults`: 14 of 16, every fault reaches its node; the 2 fails were a whole-Mac stall (task 11). Not on the car: there each sensor already has its own driver |
| 11 | A long timing test (10 min, parked and driving) to catch the rare late data, before and after 10 | ✅ run 23 Oct (task 32) | a 1 min test was clean: real time 0.99, every topic under 80 ms late. Seen since: the IMU, wheels and GPS silent 0.7-2.1 s together in MPPI's lot run (one adapter, before 10); and after 10, in `--faults`, a stall of everything at once (the global EKF missed its rate, the controller ran at 8 Hz, `map → odom` 1.4 s old). So not only the adapter: find what the whole Mac waits on. 02:13 run: IMU, wheels, both EKFs silent 0.5-0.7 s together, with the Mac 77% idle and nothing else busy: so not a CPU shortage. **Found: it's the messaging (Fast DDS), not macOS.** A bare 50 Hz publisher and subscriber beside a sim run, 25 min: the publisher's timer never slipped more than 0.03 s, a no-ROS loop 0.06 s, but the subscriber waited up to 0.48 s (9 times over 0.1 s). **Cyclone DDS: worst 0.05 s, none over 0.1 s** (25 min beside a sim run), and `AAV_RMW=cyclone pixi run sim-check --obstacle` passes everything (the rear camera at 10 Hz, 7-8 on Fast DDS). Opt-in now: `AAV_RMW=cyclone` (tools/ros.sh). ✅ **Ben, 23 Oct: "yes lets do it"**: Cyclone is the default everywhere (`pixi.toml`); `AAV_RMW=fastdds` goes back | Also seen: Spotlight (`corespotlightd`) up to 75% of a core now and then |
| 18 | Stuck facing a wall: after a loop or a wide turn the collision monitor stops the car facing a wall, and forward only it can't get out | ✅ way out on | not a false stop: at the building the car had turned toward it before passing it, and a box that bends with the steering (`VelocityPolygon`) would have bent further in. So not built. The way out is reversing (Nav2's back-up), off on the car until the team decides. With reversing, both controllers now get round every goal of the lot (run n, task 19). ⚠️ **Team: may the car reverse?** It needs something that sees behind it. **22 Oct, morning: Ben said yes, the car may reverse** (task 24) |
| 20 | Goals: a miss of heading still cost a loop | ✅ done | heading tolerance 0.5 → 0.75 rad (the EKF's heading is up to 25° off; both controllers arrived 35° off by cutting the plan's last bend). Run j, 04:03: the first two goals clean for both (29-32 s, 21-22 m; 53 s, 42 m). The U-turn failed for both: its plan is a light-bulb turn using the whole 14 m lane |
| 19 | MPPI's U-turn: forward in the 14 m lane it went wide or round the building (3 of 4 runs); with reversing, the planner drew a three-point turn but MPPI never reversed: it drove on to the far wall, stopped, and the back-up recovery failed | ✅ done with reversing | three causes, with reversing on: (1) `PreferForwardCritic` kept MPPI from backing up at all (runs h, k: 0 reversals), so it is dropped when reversing: the planner's `reverse_penalty` already decides when to reverse; (2) `PathAngleCritic` mode 2 and inversion tolerances 0.5 m / 0.6 rad, so it follows the path's own direction; (3) the progress checker cancelled the turn 10 times (rocking back and forth moves < 0.5 m): now `PoseProgressChecker`, turning 0.5 rad counts too. Run m, 04:25: turn back reached, 69 s, 2 reversals. **Run n, 04:35, whole tour: all 8 goals, both controllers.** MPPI's turn back: 71 s, 2 reversals, wide; Pure Pursuit's: 74 s, tighter, but 10 reversals and 4 stops. Forward only (run o, 04:46): the U-turn still fails for both, as before the changes: so they did no harm, and only reversing fixes it. Left: MPPI drives on a few metres before it backs up (its own choice: it asks 0.5 m/s forward, easing to 0 over 13 s). Forward only, after the EKF fix: run s (06:16) all 8 goals, the U-turn as a loop; but run t (06:26) MPPI's loop met the north wall (2 of 4), Pure Pursuit looped twice at the first goal. So forward only it's still a coin toss; reversing is the reliable way. Tried 06:35, not kept: MPPI's `az_max` 1.0 → 0.4 (to match ego's steering rate): runs u, v made the U-turn once in two, the same as before. `docs/path-followers.md` |
| 16 | `sim-check`'s scripted drive came up short once (4.43 m, needs 4.5), then 5.99 m twice | 👀 watch | probably the same stalls as 11 |
| 12 | Steer round the box in the lane: sensors in the planner's map (docs/plan.md M5) | ✅ done | the planner's costmap gets the radar and lidar layers (`plan_around:=true`, the default). Both controllers drove round the box 2.6-3.8 m clear. New `sim-check --plan-around` passes (3.9 m clear, no stop); the stop tests run with `plan_around:=false` and still pass (`sim-check --obstacle --lidar-obstacle`, `bench-check --obstacle`) |
| 13 | The global EKF's heading jumps at walking pace (docs/plan.md M9, T6.3) | ✅ 25 Oct, `wt/ekf-heading`: **a heading source, from the GPS receiver's velocity.** `gps_heading` (`aav_gps_heading`, `course.py` with 20 tests): the velocity's direction (Doppler, 0.05 m/s), turned round when reversing, the antenna's swing taken out, only above 0.3 m/s with the GPS's and the wheels' speeds agreeing; into `ekf_global` as imu1, yaw only. Gazebo's `gazebo_gps` now sends `/sensors/gps/fix_velocity` too (the truth + 0.05 m/s). `pixi run tools/ros.sh --local python3 tools/heading_check.py` (3.5 min, 0.5-1 m/s, forward and reversing, empty world), off → on: **5 Hz GPS, 4 runs each**: forward median 2.2-2.8° → 1.6-1.8°, worst 12.7-27.1° → 10.4-13.6°; reversing median 2.3-2.7° → 1.7-2.1°, worst 11.4-18.0° → 8.5-13.1°. **1 Hz (the receiver's factory rate), 3 / 2 runs**: forward median 2.8-5.4° → 2.7-2.9°, reversing 3.1-3.5° → 2.2-3.1°; worst about the same (10-21°). **Gyro biased 0.6°/s, 2 runs each**: median 3.3-4.3° → 1.5-1.9°, worst 12.3-22.3° → 9.4-16.9°. **Started 30° wrong, 2 runs each**: under 5° after 7.8-10.0 s → 5.4-6.1 s (3 s of it parked). Not taken: the magnetometer (raw counts, the motors), lidar matching (not installed; no north), `use_odometry_yaw` / `yaw_offset` / `differential` (no effect with a datum). Left, the hardware lead decides: 5 fixes a second (a receiver setting), RTK (fixes 25× better), two antennas (~0.4°, standing still too). On the car: `gps_driver:=ublox` for the velocity; check with the first bag |  needs a heading source; lower noise helped a little. 22 Oct, 05:30: at 0.5 m/s its heading wanders up to ±25°, forward and back alike (probe: the wheel speed and turn rates are right in reverse, so no sign bug). Over the night's runs, reversing: median 11° off, worst 76°; forward at the same speed: median 4°. **Cause of the reversing part, 05:35:** the antenna is 0.8 m ahead of the rear axle, and `navsat_transform` removes that offset with the EKF's own heading, so a heading error feeds back. Antenna over the rear axle (Gazebo, two runs each): reversing median 4.6-6.6° (was 7.5-12.8°), worst 26-31° (was 39-54°). ✅ **it is over the rear axle already** (Ben, 23 Oct); `vehicle.yaml` now says x = 0. **And in software, 05:55:** the global EKF's yaw process noise 0.06 → 0.01 (`a588a993`): heading median 2.6° off, worst 14° (was 6.2-6.7°, 39-54°); still right within 10 s of a start 29° wrong; better with a biased gyro too. Check with the real IMU's bias. Lot tour, run r (06:04): all 8 goals, worst heading per leg 7-16° (runs n, q: 14-76°) |
| 14 | Weigh the car, front and back, then with the rear lifted (M7, M7b) | ⬜ needs the team | `docs/measure-the-car.md` |
| 24 | **The car may reverse** (Ben, 22 Oct morning: "Yes the car can reverse"), safely | ✅ done | `52d58380`: `reversing:=true` on the car, but only where the lidar sees behind it (`vehicle.blind_behind`; else forward only, with a warning). collision_monitor's stop zone turns round with the command (`velocity_polygon`): reversing, a box 1 m behind the rear bumper. `sim-check --rear-obstacle`: stopped 0.75 m short of a post, then backed on to the goal. `sim-check --obstacle` still passes (0.46 m). Seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5 (its speed loop overshoots) |
| 25 | The rear camera as a sensor behind the car | ✅ wired, ⬜ untested with a real picture | the detector reads more cameras with one model (`extra_images`); camera_obstacles publishes every camera's newest points together (`aav_detector/recent.py`), so the rear camera's don't flicker; collision_monitor stops for them (source `camera`) only with `camera_guard`, which the car sets when the rear camera watches behind: elsewhere (bench, Gazebo) no people detector feeds it, and a silent source would stop Nav2. It counts as seeing behind only when the machine's detector reads `/sensors/camera/rear/image_raw` and keeps people (`car.launch.py watches_behind`; the car's detector keeps stop signs only today). Not tried in Gazebo: it has no person to detect. Needs the cameras calibrated (Q22). Also fixed: `blind_behind` had counted camera_obstacles alone, which reads only the front camera |
| 26 | Check the nightly Gazebo job (task 23) after its first run, 03:00 on 23 Oct | ✅ 25 Oct | nightly `sim`: 21, 22 and 24 Oct passed; 23 Oct failed (no GL: fixed that day); 25 Oct failed on Gazebo's own segmentation fault at start (software rendering on the runner), and every node after it. sim-check now starts once more when Gazebo itself crashed while starting (`tools/gz_crash.py`; a second crash fails). Task 23 keeps watching |
| 27 | The speed loop overshoots its target by about 15% (seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5) | ✅ 25 Oct, branch `wt/speed-loop`: a profile, **not the default** (Ben: "lets do them all"; the firmware keeps today's gains until the stands, TODO 52b) | The cause is the PI's zero (−ki/kp = −0.33, slower than the poles), not windup: clamping alone changes nothing. Fix: a **setpoint weight** (the P term sees b × target), which moves the zero and leaves the poles and margins alone. spine-core `SpeedLoopConfig` gets `setpoint_weight` (1.0) and `anti_windup` (`Bounds`), both today's loop to the bit; `SpeedLoopProfile` names `default` and **`smooth`** (kp 1.0, ki 0.25, b 0.75, integral limit 1.0, clamping). Gazebo's port has the same, and both check `firmware/spine/speed_loop_vectors.txt`. Model (`pixi run python3 tools/vehicle_model.py --speed-loop`, 6 runs per cell, the lag doubled and 110 ms late included): overshoot 17 → 0 % at K, 0 → 0 % at K/5 by drag (today 29 % short of 2 m/s), 17 → 8 % at K/5 by a weak throttle (today 29 % short), 15 → 1 % at 2K; settles (5 %) 13.7 → 5.7 s; phase margin ≥ 42°. Rejected: clamping alone, feed-forward (88 % at 2K), pole placement (22 % at K/5, 31°). Gazebo, `sim-check --speed-steps-only --speed-loop`, 3 runs each: today 15.9 / 15.2 / 10.1 % at −0.5 / 1 / 2 m/s, within 5 % after 11-13 s; `smooth` 0 / 0 / 0 %, 4.2-5.3 s (and the same with the spine in the loop). `sim-check --speed-loop smooth` (road world): all 10 checks pass; `bench-check` (default) and `esp qemu-check` pass. Car day: [where to try it](docs/actuator-tests.md#the-speed-loops-profile-todo-27) (needs the spine's closed loop: today's ESP32 runs open loop) |
| 23 | `sim-check` in the nightly CI (PLAN T2.6) | 👀 watch | 25 Oct: a Gazebo crash at start, now retried once (task 26). Job `sim` in `test.yml`, Linux, `sim-check --obstacle`; first run 03:00 on 23 Oct **failed**: Gazebo crashed, as EGL had no driver (no GPU, no Mesa). Fixed 23 Oct: `tools/software_gl.sh` (Mesa's llvmpipe), and the scripted drive on the simulator's time (a slow simulator cut it short); passes on oirp-spss with 4 cores, Cyclone and Fast DDS. A failure now leaves annotations and the artifact `sim-logs`. On trial (`continue-on-error`): once it passes a few nights, drop `continue-on-error` |
| 22 | Jerk limit in ego (PLAN T5.3) | ✅ done | `max_jerk` 2.0 m/s³ (`ego.yaml`): speeding up, the push builds over 0.5 s and eases off before the set speed (0 to 1 m/s in 1.5 s, was 1 s). Braking is never softened: a stop starts at once. `sim-check --obstacle` passes (stopped 0.38 m before the box) |
| 21 | Gazebo check: a 10° steer gives the predicted radius within 5% (PLAN T2.3) | ✅ done | 22 Oct, 04:55: 9.05-9.14 m for 8.90 m (+2-3%); 20° and full lock also within 3%. `docs/simulation.md` |
| 15 | Gazebo check: a tight turn at `sim_speed:=3` slows to ego's curve speed | ✅ done | asked 3 m/s at full lock: ego allowed 2.40 m/s (the computed limit), the car held 2.37-2.43 m/s with 1.9-2.0 m/s² sideways (limit 2.0); straight, 3.1 m/s. Probe: `/tmp/aav-probe/curve_probe.py` (not in git) |

## 15 Oct: all the C++ to Rust

Ben, 15 Oct: "We should port all cpp to rust", and remove the old.

- [ ] **Flash them** on spare Pico 2s, then on the car: `pixi run pico wheel build`,
      `pico sentinel build` ([Rust on the hardware](docs/rust-firmware.md)) [[needs-you]].
- [ ] **Something to feed the sentinel:** its 2025-26 sender isn't in any repo. A small
      ROS node that writes the packet over I2C (`sentinel::Telemetry::to_packet` shows the
      bytes), once we know the display is on the car (the audit).

## 14 Oct: the car in photos, the KEFC CAN plan

Ben sent 32 photos and the KEFC request (CAN bus, wheel speed, steering feedback, new GPS,
a second Orin). Pages: [car photos](docs/car-photos.md), [CAN plan](docs/can-plan.md),
[topic areas](docs/naming.md).

- [ ] **The simulated lidar on by default:** off for now, as on the Mac it made the full
      graph with YOLO flaky (DDS over loopback UDP). Try Fast DDS's shared memory, or a
      smaller cloud (`docs/simulation.md`).
- [ ] **Friday's audit:** the checklist at the end of `docs/car-photos.md`.

## 11 Oct: what we can try at home

Ben, 03:00: "We can only do so much from home so lets focus on stuff that makes sence to
work on." His order, with the work under way:

1. [x] **The full graph, all set up:** every driver, ego, the simulators. In Gazebo: a road
      with lane lines, a stop sign and an obstacle, a camera and a radar on the car, and
      perception, lane keeping, collision_monitor and the HUD in `pixi run sim`. *Done
      15 Oct: merged from `wt/sim-full`, with a lidar too ([Simulation](docs/simulation.md)).*
      - [ ] **Ackermann done properly**, testable in Gazebo and on the bench: reversing
            (Reeds-Shepp), both front wheels' real angles, the steering curve from
            `Ackermann.m` (Q2). *Paused 11 Oct, on `wt/ackermann`. Reversing done 22 Oct (task 24); both wheels' angles and the steering curve still open.*
2. [x] **Our C++ to Rust:** Ben's Rust IMU driver becomes the default and the C++ one goes;
      the radar clustering becomes a Rust ROS node. Python to Rust later (Ben). *Done
      15 Oct (Ben: "We should port all cpp to rust"): `aav_imu_yahboom` and
      `aav_radar_clusters`; no C++ left in `src/` (now `ros/`). Only the ESP32's firmware is C++ (and the hardware lead's Arduino).*
3. [ ] **The Mac on the network, with ML on it:** the laptop's detector (Core ML) on the
      car's compressed pictures. *On in `laptop.yaml` since 11 Oct; to try against the
      bench as the car.*
4. [ ] **Nav2:**
      - [ ] N7, the weave: the bench's pretend GPS adds fresh noise to every fix, unlike a
            real receiver, whose error drifts slowly. Model it as a slow drift, then tune
            `ekf_global` for the real receiver (Q18). *29 Oct (branch `stop-hold`): the
            drift is modelled (`aav_bench` `models.GpsError`); `bench-check --weave`
            measures the weave. Tuning `ekf_global` is left;*
      - [ ] collision_monitor flickers between stop and go at the zone's edge. *29 Oct
            (branch `stop-hold`): `aav_safety`'s `stop_hold` holds each stop until the
            zone stays clear 1 s (twist_mux `stop_zone`, priority 21); `bench-check
            --zone-edge` tests it.*
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

## Ideas to keep

- [ ] **Your Mac as the operator station** (Ben, 11 Oct): the HUD on the Mac's screen,
      driving the car from the Mac, the Core ML AI driver on the Mac, maybe all of
      perception; the Orin doing the processing when you sit at the car. *Saved, not
      started. What runs where: `docs/history/conversation.md`, 11 October.* It waits on:
      - the car on Jazzy (Q24): a Jazzy Mac can't join the car's Humble;
      - the HUD on a Mac: raylib's runs there as it is; the Flutter app needs CocoaPods (archived 25 Oct);
      - the car's WiFi, measured: how late the pictures arrive, and how often it drops.
- [ ] **A hardware workbench: every sensor and actuator, one at a time** (Ben, 28 Oct,
      word for word in `docs/history/messages-2026-10-28.md`). Just for us. One screen on
      the laptop (the HUD), with the RC or the joystick as well, to isolate the hardware:
      - **each sensor, live:** is it there, how fast, its values (GPS, IMU, lidar,
        cameras, wheel speed, steering pot, battery);
      - **each actuator, by hand:** jog the steering, throttle and brake in small capped
        steps, and see what came back;
      - **on stands, find the limits:** steer slowly until the motor meets resistance:
        the Jrk G2 reports its current and position, so a stall shows as current rising
        while the position stops. Back off a margin: that's the steering limit;
      - **work out the numbers:** acceleration, braking and steering rate, into
        `vehicle.yaml`; then check them by driving in the lot.
      Builds on what we have: `pixi run actuator-test` (its `--stands` tests already measure
      the steering sweep and rate), the spine's hardware test mode (#15, capped at
      `TEST_MAX_THROTTLE`), bench mode and the raylib HUD. New: the live sensor page, the
      jog controls, and the stall search. Check the 2025-26 team's clamps first
      (`JRK_STEER_MIN`/`MAX`, `DAC_MAX_CAP` in `hardware_configs.h`). Steering and braking
      limits on the real car are the hardware lead's to sign off.
- [ ] **Tesla-style cubes on our HUD, then on the camera picture** (Ben, 28 Oct: "I want
      to see the occupancy network", "put it on a hud like we have", "merge the voxels with
      the camera feed to figure out what is where"). No new install: not OctoMap, which
      would bring PCL back (tried 28 Oct, undone). Gazebo first, where every pose is exact:
      1. **cubes from the lidar:** a voxel grid in Rust in `hud-core` (no ROS, own tests):
         each `/sensors/lidar/points` point into its 20 cm cube; cubes not seen for a while fade;
      2. **draw them:** raylib draws 3D cubes; colour by height, like Tesla's view;
      3. **onto the camera:** project each cube into the HUD's camera picture (`picture.rs`),
         from where the camera sits relative to the lidar; colour by distance;
      4. **what is where:** each RF-DETR box gets the distance of the cubes inside it
         ("stop sign, 14 m").
      On the car, 3 and 4 need camera-to-lidar calibration (research pick 4). Nav2's own
      `VoxelLayer` is a separate choice (it changes driving): only if 2D misses something.
- [ ] **Cubes for Nav2** (Ben, 30 Oct: "this is a car not a needle"): built as a trial,
      `voxels:=true` (`aav_voxels`, `docs/designs/2026-10-30_voxels.md`); in Gazebo it stops
      for the post and drives the lot tour like the scan (`docs/trials.md`). Left:
      `--rear-obstacle` and `--plan-around` with `--voxels`; a thin pole in the stop zone
      (`min_points: 3` on cubes); 3 runs of the lot tour; the Orin's ms a cloud; the car,
      leaning in a turn; then Nav2's 3D `VoxelLayer` on the same cubes; then make it the
      default if it wins.
- [ ] **The debug HUD** (Ben, 29 Oct, word for word in `docs/history/messages-2026-10-29.md`):
      the hardware workbench above, as its own HUD screen. It reads each sensor's and
      actuator's own node directly (`/sensors/...`, `/vehicle/...`, the spine's status),
      not what comes out of the muxes, so a fault shows where it starts:
      - **sensors:** shake the car and watch the IMU; turn a wheel and watch the encoder;
        GPS, lidar, cameras, battery, the steering pot;
      - **actuators:** watch the wheels turn, speed up and brake, in the spine's hardware
        test mode (#15): its caps and the e-stop still apply. Never around the spine;
      - **what the car thinks:** the lidar's cubes and the NDT match (where the scan sits
        on the saved map), drawn on the same screen.
- [ ] **measure controllers picks a random ROS domain without checking it's free** (found 29 Oct:
      it picked 191, the same as a Gazebo already running, and the two runs mixed). Until
      fixed, give it one: `AAV_SIM_DOMAIN=213 pixi run measure controllers ...`.
- [ ] **Two screens, not one** (Ben, 29 Oct evening, word for word in `docs/history/messages-2026-10-29.md`):
      1. **The hardware screen**: every sensor live, and **very simple buttons** for the
         actuators: speed +/-, brake +/-, steering left/right, **centre**. Through the
         spine's hardware test mode (#15) and its caps, never around them; the e-stop stays
         the stop. TODO 49b limits the throttle on stands until a rear-wheel speed exists;
      2. **The seeing screen** (today's debug HUD, split): left, big, the 3D cubes; right
         top, a top-down of the lidar map (ICP/KISS-ICP, or the NDT map today) with the
         car on it; right bottom, a top-down of NDT's match (the scan on the map's cells).
      Design: `docs/designs/2026-10-29_two-screens.md` (EGM-reviewed): `hw_panel`, its own
      program, twist_mux slot 45, hold-to-drive; `debug_hud` stays read-only.
      **Steps 1-3 built 29 Oct** (branch `two-screens`): `debug_hud` opens as the seeing
      screen (key L: the old panel); `hw_panel` (`pixi run hw-panel`, `hw-panel --demo`)
      with every sensor and the buttons through `/drivers/buttons/cmd_vel` (slot 45),
      speed on SIM and BENCH only (docs/hud.md, "The hardware panel"). Left: the window
      itself against Gazebo and both screenshots from it (the Mac's screen was locked:
      demo screenshots only); read the legacy bridge's `speed_loop.mode` before stands;
      the bench check (a person's joystick beats the buttons); step 4, option B (brake and
      steering through the spine's test mode, once it has a transport, TODO 0b)
- [ ] **A preflight on stands, before the ground** (Ben, 29 Oct): one scripted run, wheels
      off the ground: speed up, hold, slow down, brake; steer left, right, centre; and
      every sensor checked live (rate, values in range) at the same time. PASS or FAIL per
      line, as `car-day-check` does. Builds on `actuator-test --stands` and bench-check.
      **Built 29 Oct** (branch `preflight`; [the design](docs/designs/2026-10-29_preflight.md),
      revised after its EGM review): `pixi run actuator-test preflight --stands` (`--sim`, `--bench`, `--no-drive`, `--skip`). On the car the drive is
      refused until a rear-wheel speed exists (49b): `--no-drive` checks the sensors only.
      Every actuator test now also stops on 4 s of driving with no measured speed, and on
      a speed rising while the command falls. Left: the car itself (car-day session 9, 3.P)
- [ ] **Cubes into objects, with names** (Ben, 29 Oct: "merge cubes that related like a
      stop sign or a car"): join touching filled cubes into one object (as
      `radar_clusters` does for the radar); then name it: the RF-DETR box it falls in
      (needs the camera-to-lidar calibration, research pick 4), or later a small model
      that reads the cubes' camera crop. Each object: a name, a size, a distance. Shown on
      the debug HUD first.
- [ ] **numpy in the `ml` environment** (Ben, 28 Oct: "do we need to upgrade?"): not now.
      The pin (`<=2.3.5`) keeps `python3 tools/export_model.py --format coreml` working; the everyday environment already
      runs 2.5. Try again when coremltools passes 9.0: newer numpy, run `python3 tools/export_model.py --format coreml`.
- [ ] **Four research picks** (Ben, 28 Oct; details: `docs/designs/2026-10-28_research-picks.md`):
      1. **Autoware**: borrow its designs (PID + feed-forward speed with a smooth stop,
         MPC steering, NDT + EKF localization), not the whole stack. **Our own versions,
         in Rust** (Ben, 28 Oct: "take ideas from that and rebuild them in rust", "lets
         write our own version of it all"): the speed loop with a smooth stop
         (`spine-core`), NDT matching, MPC steering, the occupancy cubes. Cite what we read
         (Autoware is Apache-2.0; the NDT papers) in `PROVENANCE.md` and the report: the
         feed-forward + PI was our plan (27 Oct) before we read Autoware; MPC, NDT and the
         smooth stop came from reading it. Crates, checked 29 Oct: see the research picks;
      2. **lidar localization**: map the lot with KISS-ICP from a VLP-16 bag, then locate
         by NDT or ICP; no RTK on this car, so this is where centimetres come from;
      3. **a safety argument**: Simplex (we have most parts) and RSS (stopping distance
         from measured braking), for the final report;
      4. **seeing from above**: inverse perspective mapping (an AprilTag or a mark at B
         for the last metre) and camera-to-lidar calibration (distances for RF-DETR's boxes).
- [ ] **AMCL: a particle filter against a lidar map** (Ben, 28 Oct; not sure yet if we
      need it): Nav2's stock Adaptive Monte Carlo Localization. Thousands of guesses of
      where the car is; each lidar scan is matched against a map, and unlikely guesses
      die off as the car moves. ~5-10 cm where the lidar sees features (curbs, posts,
      walls), against GPS's 1-3 m: it could matter most for the last metres to B. Needs
      a map first (`slam_toolbox`), and a test site with things to see: a bare lot gives
      it little. Would sit beside the GPS + EKF, not replace them.
- [ ] **One model-based controller instead of many** (Ben, 27 Oct): investigate LQR, and
      collapse today's controllers (pure pursuit, the stopping law in
      `aav_drive_to_point`, the spine's PI and P-only brake) into one controller built
      from `vehicle_model.py`, closed loop, maybe running on the ESP32. Questions:
      - **speed:** throttle and brake as one signed acceleration, LQR with integral action
        (LQI), on the ESP32 where wheel speed and the actuators are;
      - **steering:** a lateral LQR on the path errors (sideways, heading, wheel angle),
        scheduled by speed; it needs the car's pose and path, which live on the Orin;
      - **stop on B:** distance-to-go as a state; the ESP32 could count the last metres on
        wheel odometry, independent of GPS lag (a CAN contract change: D4);
      - **MPC** if the limits matter (brake, steering rate): LQR doesn't know them;
      - scored with `measure atob` against pure pursuit and MPPI; needs the measured car first.
      *Saved, not started (`docs/vehicle-model.md`, "After state space").*
- [ ] **Learned state-space models, Mamba-style** (Ben, 27 Oct): Mamba (and S4 before it)
      runs the same `x' = Ax + Bu` as `vehicle_model.py`, but learns `A`, `B`, `C` and lets
      them depend on the input. Its diagonal `A` is a bank of first-order lags (τ = -1/a),
      like our steering lag. Worth a closer look once hours of car logs exist: a learned
      model of what the linear one misses (drag, slope, the brake's play), checked against
      the fits from the actuator tests. *Saved, not started; needs car data first.*
- [ ] **micro-ROS, again** (Ben, 11 Oct): "lets rethink thinks later after we see the
      hardware". No micro-ROS for now (Tentative).

## Overnight (10-10 → 10-11): all the code, then Ackermann, Rust, native style

Ben, 21:45: "Lets try to get a fully functionin gthing going on." The order of work is
Decided ([Decisions](docs/decisions.md)):

1. **The high-level pieces**, by five helpers in parallel, each on its own branch:
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

- [ ] A spare Pico 2 and a **3.3 V** MCP2515 CAN module (for example with an SN65HVD230),
      to try the spine on a desk. The common 5 V module (TJA1050) doesn't suit a Pico 2.
- [ ] The e-stop: a photo, and where its wires go (Q6).
- [ ] The brake after boot: `IDLE_BRAKE = 0.0` lets it go; the 2025-26 firmware kept it on.
- [ ] A fresh clone on oirp-spss before anyone works there.

## On Ben's Mac (10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-10.md`. "Our version" is
`aav/aav` (`~/Code/aav/aav` on the Mac; retired 25 Oct: now `~/Code/ecolo-av`); this work was done in a clone at `~/aav` while that
folder downloaded, and moves into it as a fast-forward.

- [ ] **Prune the snapshots from `aav/aav/.git`** (149 MB down to about 1 MB)? *Ben to
      decide: it removes the app's undo points.*
- [ ] **Look at RViz and rqt_graph on the Mac**, and say what they show.
- [ ] **The small fixes** listed at the end of `docs/work.md`.

## The afternoon of 10-10: "merge in all of this", then "stub in the rest"

- [ ] **Team decisions:** the steering on an e-stop; the brake after boot; a brake on the RC
      gear switch (docs/firmware.md). *Since 11 Oct, one constant each in `spine-core`.*
- [ ] **Find `best.pt`**, the 2024-25 stop-sign model. (The 2025-26 lane finder isn't
      needed: we wrote our own, 11 Oct.)
      `best.pt` is probably on the team's Linux machine with the NVIDIA card (Ben, 10 Oct).
      No SSH to it today: later, once all the code is in place (Ben, 21:14).
- [ ] **Tune the speed loop on the car:** on the bench it trails a ramp by about a second.
- [ ] **Slim per-machine Pixi environments** (Orin, Motor Pi, laptop): at Phase 0.
- [ ] **An audit on the car's network** (Ben, 16:42: "at some point"): a read-only audit of
      the hardware. *Ready: the stand-ins stay on their own computer (`tools/ros.sh
      --local`, checked by `tools/test.sh`), so nothing pretend can reach the car.*

## Questions Ben asked

*Answered in the morning email and in the docs: `run.md`, `languages.md`, `mac.md`.*

- Is `aav_turtlesim` different from the regular turtlesim?
- What languages are used now, and should everything be Rust?
- Can it compile on a Mac without Homebrew?
- What does the claude.ai chat (fe09d020…) say? It can't be opened from here, so Ben
  needs to paste the key parts.

## For the team to decide

- **`ROS_DOMAIN_ID`** for the car and every laptop (0–101: safe on Linux; Fast DDS breaks above 232; one that no one nearby uses).
- **Spine arming rules** (design doc, Proposed): boot latched, arm on a 0→1 edge.
- **Move the car to Jazzy** before a laptop can join it (Humble and Jazzy don't mix):
  try Pixi on today's Ubuntu 22.04 first, then reflash the Orin to JetPack 7.2 after a
  backup (`docs/platforms.md`).
- **GPS driver** (B4): `ublox_gps` (in Pixi) is the default since 11 Oct, Tentative.
  `gps_driver:=ublox_dgnss` (apt, the car only) stays an option.
- **N2:** twist_mux hears `/safety/estop` best-effort, and the joystick sends it once per press.
  The backends still brake. Send it repeatedly, or make the mux reliable?
- **N3:** no `respawn` on device nodes, and Nav2 gives up without TF. Add respawn?
- **N7:** Nav2's turn command flips sign about once a second in the sim. Try a longer
  lookahead.

## Later

- Rewrite parts in Rust: now step 3 of the order of work, at the top.
- Measure the car and replace every guessed number in `vehicle.yaml`.
- Rotate the leaked secrets, and back up the Orin and Pis before any reflash.
- Answer Q1–Q34 with the team.

## For the 2027-28 team

Learned models: worth knowing, but each needs training data and GPUs we don't have this
year (Ben, 28 Oct):

- [ ] **Learned BEV** (BEVFusion, BEVFormer): all cameras and the lidar into one top-down view.
- [ ] **One network for perception, prediction and planning** (UniAD).
- [ ] **World models** (Wayve's GAIA-1) and **vision-language driving** (Waymo's EMMA).
- [ ] **Predicting others' paths** (MTR, Wayformer); at 1 m/s, constant velocity is enough today.
- [ ] **Learned planners** (diffusion planners).
- [ ] **State-space models** (Mamba): see "Ideas to keep".
