# Our own stack against Nav2 and Gazebo: the gaps (14 Nov 2026)

Ben, 14 Nov: "get all our rust code up to the same standard as the nav2 and gazebo code so
we can drop their work and have our own complete stack". Nothing is cut until the car runs,
and stock stays the default until ours wins ([the whole car](../designs/2026-11-14_the-whole-car.md)).

## Status, 14 Nov night

Checked item by item against the code on `next` (gate 171). Laptop only: unit and cargo
tests, no simulator run for this check.

### Navigation

| Item | State | Where |
|---|---|---|
| `/tf` age checks | done | costmap (odometry, map -> odom), planner (TF_ERROR), navigator (1 s), mppi_lag (map -> odom) |
| Respawn | done | `navigation.launch.py` (1 s), `test_respawn.py` |
| A watchdog on our nodes' status | **done 18 Nov**, warn only | `sensor_watchdog` `kind: status`: FAULT when a node says WARN/ERROR, STALE when silent; `status_*` inputs; `stop: true` is the switch |
| Costmap age checks | done | mppi_lag `costmap_timeout`, planner; `costmap/src/age.rs` |
| clippy and fmt | done | `tools/test.sh` (clippy on the logic crates; the rclrs nodes need ROS's messages) |
| Tests of the node binaries | left (M) | only `test_respawn.py`. A harness would: start the binary on its own `ROS_DOMAIN_ID` with `LOCALHOST`, feed its topics, call its services, read its answers, stop it by process group (as `aav_watchdog_rs`'s `test_watchdog_rs_node.py`). Meanwhile the logic moves out of the binaries (Ravensim's `node.rs`, 18 Nov) |
| Run on the car | left | gate 173 |
| Costmap: says its age, stops on stale odometry | done | status topic; `age.rs` |
| Costmap: threads and bounds | done | each costmap on its own thread; only the changed area |
| Costmap: radar, cubes, camera clouds | left (L) | TODO 156, 175 |
| Planner: `IsPathValid` | done | served by the planner |
| Planner: Hybrid-A* | **for Ben** | see below |
| Planner: 20 % slower on long plans | left | needs a timing run on the campus |
| Navigator: time limits | done | `plan_timeout`, `follow_timeout`, `recovery_timeout` |
| Navigator: `IsPathValid` on the final approach | done | once a second |
| Navigator: BackUp with no behaviour server | done | FollowPath on `path::straight_back` |
| Navigator: goals in other frames | **done tonight** | `odom`, `base_link` by the latest /tf (`Pose::compose`) |
| Navigator: error codes as enums, no `expect` | done | `PlanError`, `FollowError`, `Abort` |
| Navigator: lot score | sim | gate 167: 4/4 after the cusp fix |
| Smoother: a stale speed used for ever | **done tonight** | odometry over `velocity_timeout` old not used (`Reading::fresh`) |
| Smoother: campus at 2.5 m/s | sim | gate 167 |
| mppi_lag: costmap age | done | `costmap_timeout` |
| mppi_lag: an odometry gap | done | the path ends after `odom_timeout` |
| mppi_lag: forward only, cusps | done | reverses; cusp fix (gate 174) |
| mppi_lag: the goal at 2.5 m/s; CPU on the Orin | sim, car | gate 167, 173 |
| Reversing everywhere | left | TODO 174: `aav_track`'s trackers are forward only |

### Our simulator

| Item | State | Where |
|---|---|---|
| M1 cameras on software Vulkan | sim | needs a try without a GPU |
| M2 `cameras:=false`, a CI job | done | nightly `sim` job in `test.yml` (not yet seen green there) |
| M3 the stop sign's size | sim | |
| M4 Gazebo's helpers and names | left (L) | |
| M5 placing the car | done | `vehicle/sim/set_entity_state` |
| S1 repeatable runs | **part done tonight** | lidar noise the same on any `threads`; Python noise seeded. Left: wall-clock pace |
| S2 the shared car model | left (M) | sim to check |
| S3 the IMU | **done 18 Nov** | three turn rates, the mean acceleration between readings (it read 10 or 0 m/s²); `sim_imu` passes it on, as `imu_node` gives it (`docs/own-simulator.md`, The IMU). Left: 50 Hz against the car's 200 |
| S4 perfect wheel odometry | **done** (15 Nov, TODO 179; values and why 18 Nov) | `wheel_noise:=0.05`, `wheel_slip:=0.02`, seeded, off by default (`docs/own-simulator.md`, Noisy wheels). Left: the car's own numbers from a bag |
| S5 tests of `sim-node`, `scene.rs` | **done 18 Nov**, but the binary | `scene.rs`: 9 tests; the node's decisions in `sim-core` `node.rs`: 8. Found: a ray exactly through a shared edge can slip between two triangles |
| S6, S7, N1-N4 | sim | |

### For Ben

Ben, 18 Nov: take the recommendations. Done: the watchdog (warn only) and S4; Hybrid-A*
waits for after the release (TODO 178).

| Question | Recommendation |
|---|---|
| A watchdog on our nodes' status? | Warn only (`stop: false`): respawn, mppi_lag's costmap age and twist_mux's timeout already stop the car when a costmap or the follower dies; a lock would also hold the point driver and the AI slot |
| Hybrid-A* in our planner? | After the release: the lattice ties Nav2's on the lot |
| Noise and slip on the simulated wheels (S4)? | Yes, seeded and off by default: Gazebo's wheels are perfect too, so it isn't a gap against Gazebo, only against the car |

**How this was made:** two read-only reviews of the code and docs. Nothing was built or run
for them, and clippy was not run. Sizes: **S** under a day, **M** 2-4 days, **L** a week or more.

## Part 1: navigation (our five Rust packages against Nav2)

### Shared by all five: fix first

| Gap | Evidence | Rank | Size |
|---|---|---|---|
| Nobody notices when `/tf` stops: costmap, planner and navigator keep the last pose | `aav_navigator` navigator-node `main.rs:533`, `aav_planner` planner-node `main.rs:492`, `aav_costmap` costmap-node `main.rs:455` | must | M; **done 14 Nov**: the costmap stops on stale odometry or map -> odom; the planner fails a goal (TF_ERROR) and the navigator has no car pose when map -> odom -> base_link is over 1 s old |
| No lifecycle, bond or respawn: a dead node goes unnoticed | `navigation.launch.py:425-520` | must | M; **respawn done 14 Nov** (each of ours, 1 s; `test_respawn.py`). Its status on a watchdog still to do |
| No costmap age check anywhere (Nav2 waits 0.3 s in the controller, 1 s in the planner) | `nav2.yaml:71, 407`; `mppi-node main.rs:343` | must | S |
| None has run on the car | each page's "What's missing" | must | L |
| No node binary has a test; only the maths crates | `tools/test.sh:53-62` | should | M |
| clippy and `cargo fmt` are never checked | not in `tools/test.sh` or CI | should | S |

### Each package

| Package (Nav2 twin) | Must | Should | State |
|---|---|---|---|
| `aav_costmap` (costmap_2d) | say its age; stop placing scans when odometry stops; try radar, cubes, camera clouds | — | 100 % the same cells as Nav2; half the CPU |
| `aav_planner` (Smac lattice) | Hybrid-A* (the car's forward-only planner) | costmap age (done 14 Nov); `IsPathValid` (served 14 Nov); 20 % slower on long plans | ties on the lot |
| `aav_navigator` (bt_navigator) | time limits on planner and follower calls (a crash mid-goal hangs it); lot 7 of 12 against 8 | `IsPathValid` on the final approach; BackUp still Nav2's; goals in other frames; error codes as enums; 8 `expect` in library code (`machine.rs`) | 11 mission tests |
| `aav_smoother` (velocity_smoother) | — | a stale speed reading used for ever in closed loop; campus at 2.5 m/s not run | the closest to Nav2; better steering |
| `aav_mppi_lag` (MPPI) | passes the goal at 2.5 m/s; twice the CPU, never timed on the Orin; costmap age | forward only; what to do on an odometry gap | the best follower (11 of 12) |

**What Nav2 does that we hadn't noticed:** it waits for a fresh costmap; a crashed server
stops the whole stack (lifecycle bond), where ours leaves collision_monitor as the only
guard; it turns goals from other frames into map goals; it has tests that kill a node
partway through a run (`system_failure`, `updown`).

## Part 2: our simulator against Gazebo

Ours matches Gazebo's lidar, GPS and radar stand-in, and passes every `sim-check` leg.

| # | Gap | Evidence | Rank | Size |
|---|---|---|---|---|
| M1 | Our cameras need a GPU. Try Bevy on Mesa's software Vulkan; **not sure it works** | `own-simulator.md` | must | M |
| M2 | A `cameras:=false` switch for ours, then a CI job with ours | `worlds.py:83-86`; CI runs only Gazebo (`test.yml:297-308`) | must | S; **switch done 14 Nov**: `cameras:=false`, `sim-check --no-cameras`; the CI job next |
| M3 | Our stop sign is drawn 52 px against Gazebo's 68; an early `--perception` failure is unexplained | `own-simulator-cameras.md` | must | M |
| M4 | Ours still runs Gazebo's Python helpers and world files (`gazebo_adapter`, `gazebo_imu`, `gazebo_gps`, `worlds/`, `models/`); `/gazebo/*` topic names | `aav_gazebo` | must | L |
| M5 | No service to place the car (`tools/sim_frames.py` needs Gazebo's) | `sim_frames.py:66` | must | S; **done 14 Nov**: `vehicle/sim/set_entity_state` (simulation_interfaces/SetEntityState) places the car; `sim_frames.py --simulator own` |
| S1 | Runs aren't repeatable: wall-clock pace, unseeded Python noise, lidar noise depends on `threads` | `main.rs:277-307`, `lidar.rs:115-118` | should | M |
| S2 | The node drives a plain bicycle model; the shared car model (`SharedCar`) isn't wired in; no slopes | `car.rs:116-166` | should | M |
| S3 | IMU: only the z turn rate; `gazebo_imu` throws away our slope-correct acceleration | `sim.rs:565`, `gazebo_imu.py:58-59` | should | S |
| S4 | Wheel odometry is perfect: no slip, no noise | `sim.rs:527-557` | should | S |
| S5 | `sim-node` (748 lines) and `scene.rs` have no tests | — | should | M |
| S6, S7 | Speed above real time unmeasured; `spine:=true`, `esp32:=true` never run with ours | — | should | S |
| N1-N4 | Moving people and cars; spheres, hills; lidar intensity; chase camera at 30 Hz | — | nice | S-M |

## The order

| # | Do | Why first |
|---|---|---|
| 1 | clippy and fmt checked; `cameras:=false` and a CI job with our simulator | cheap, and they keep the rest honest |
| 2 | A shared Rust transform crate with time stamps; age checks on `/tf`, the costmap and odometry in every node | the biggest safety gap in "all ours" |
| 3 | Time limits in the navigator; respawn and a watchdog on each node's status | a dead node must stop the car |
| 4 | `mppi_lag`: the goal at speed, its CPU, its time on the Orin | the follower we'd pick |
| 5 | The navigator's lot score, `IsPathValid` | to beat Nav2, not tie |
| 6 | The simulator's musts (M1, M3, M4, M5), then repeatable runs (S1) | so scores can be trusted |
| 7 | Hybrid-A*; costmap with radar and cameras; then the car | the rest |

**Unsure:** whether rclrs ends an action wait when the server dies (if so, the navigator's
hang drops to "should"); whether Bevy runs on software Vulkan; clippy's real state.
