# Audit, 18 Nov: the docs and TODO.md

Ben asked for a docs and TODO audit on 18 Nov. This page covers it, with TODO 172
(shorter pages), 160 (shrink the docs and TODO.md) and the docs part of 232 (Nav2's
leftovers).

## What was checked

| Check | How | Found |
|---|---|---|
| Every page in `docs/*.md` (115) | read the reading list, the learn pages and every line naming Nav2 or Gazebo without a date or "until", "ported", "was" | most were already right after the 17 Nov sweep; the slips are below |
| Links between pages | every `](file.md)` and `](file.md#anchor)` against the files and their headings | 1 heading lost (trials' NDT section, 18 Nov), 1 anchor gone (simulation's Gazebo table) |
| Switches named in the pages (`name:=`) | against every `DeclareLaunchArgument` in `ros/` | all exist; `stack:=stock`, `controller:=rpp`, `simulator:=gazebo` only as history |
| `pixi run <task>` in the pages | against `pixi.toml`'s tasks | all exist, but `car-test` (the testing plan's phase A, not built yet: TODO 246) |
| Topics in the pages | against `docs/interfaces.yaml` and the code | `/safety/collision_monitor/state` is now `/safety/stop_zone/state` |
| Packages | `packages.md`, `modules.md` against every `package.xml` (72) | 9 missing from packages.md, 18 from modules.md |
| `TODO.md` | every ✅ row; the open rows against the code | 29 finished rows, and 5 rows out of date |

## What changed, page by page

| Page | Change |
|---|---|
| `trials.md` | the "NDT into the global EKF (30 Oct)" heading put back (the 18 Nov AMCL commit had replaced it); then 14 dated result sections (1319 lines) moved to [trials-log](trials-log.md); the page keeps the guide and an index (1470 → 158 lines) |
| `hud.md` | 8 "Tested" sections moved to [hud-log](hud-log.md) (1167 → 1077) |
| `mppi-lag.md` | the Gazebo run, the campus junction, the two sway hunts moved to [mppi-lag-log](mppi-lag-log.md), with a two-line summary of each fix left (716 → 461) |
| `controllers.md` | the 29 Oct speed-loop results moved to [controllers-log](controllers-log.md) (757 → 662) |
| `car-day.md` | the 24 Oct `car-day-check` run moved to [car-day-log](car-day-log.md) (1138 → 1097) |
| `status.md` | as of 18 Nov: the car's upgrade day, a "Since 16 November" table, the package count, AMCL in Rust, the behaviour planner's Rust default; "Next" from `TODO.md` and the testing plan. The 15 Nov "Next" moved to [status-log](status-log.md) |
| `packages.md` | 9 packages added (the Rust twins of `camera_stream`, the point driver, ego, `joy_alive`, the stop-sign watcher, the lane finder; `aav_amcl_rs`, `aav_lane_pose`, `aav_mpc_lag`); `aav_track` is in `mppi_lag`'s place, not `controller_server`'s; a note that "Gazebo" in a status is a pre-16 Nov run; 72 packages |
| `modules.md` | 18 packages added to "More packages"; the stop zone's state topic; `camera_obstacles` feeds the costmap, not Nav2; the Gazebo note |
| `interfaces.yaml` (and `interfaces.md`) | the Rust ego and behaviour are the defaults, not challengers; navigation, not Nav2, holds the slot; no Nav2 costmap, planner or controller runs beside ours |
| `learn-lidar.md` | the cubes, the things, NDT, AMCL, the debug HUD and the lot's map are built, not "planned"; section 11 is as of 18 Nov |
| `release-0.4.md` | the Rust defaults as of 18 Nov; four 18 Nov rows in "What's new" |
| `release-0.4-rc1.md` | six Rust nodes on 17 Nov, 16 switches by 18 Nov |
| `release-0.4-guide.md` | the 17 Nov queue with an 18 Nov column (AMCL ported, six Rust ports done) |
| `languages.md` | rows for `joy_alive`, the stop-sign watcher, the lane finder, the camera stream and AMCL in Rust |
| `overview.md`, `controllers.md`, `switches.md`, `perception.md` | AMCL is ours in Rust; the stop zone, not `collision_monitor` |
| `features.md` | the league's row: `stack:=stock` until 17 Nov |
| `platforms.md`, `infrastructure.md` | the Orin to JetPack 7.2 or newer, the hardware lead's call (Ben, 18 Nov); our navigation, not Nav2, on the Orin and in `sim` |
| `start-up.md` | the stand-in backends: Ravensim, not Gazebo |
| `ghost-obstacles.md`, `learn-journey.md`, `glossary.md`, `naming.md` | `/safety/stop_zone/state` |
| `plan.md` | its link to Ravensim's sensor table |
| `ros/drivers/README.md` | the behaviour planner writes the road ahead, not `/drivers/nav/speed_limit` |
| `ros/safety/aav_stop_zone/package.xml` | the lock since 17 Nov, not a shadow |
| Links into moved sections | 20 files now point at the logs (`TODO.md`, `features.md`, `switches.md`...) |
| `TODO.md` | 29 finished rows and "Closed on 15 Nov" moved word for word to [todo-done](todo-done.md) (285 → 234 lines, 100 → 79 kB); "What is next" has an 18 Nov column; 181 done (`stack:=stock` went with Nav2), 184 (230 done), 163 in progress, 167 can't run as written (no stock stack left), 232's docs part done |

## Found, and left

| What | Why left |
|---|---|
| TODO 167, "ours at least as well as stock": no stock stack is left to run | a gate: Ben decides its new yardstick |
| TODO 168 is ✅ "sway at Nav2's" (15 Nov), but `release-0.4.md` says 🔧 "about 2 times Nav2" | two scorings on different days; which one is the gate is Ben's call |
| `simulation.md`, `own-*.md`, `sim-and-car.md`, `full-simulation.md`: many Gazebo and Nav2 numbers | dated, and each page has a note saying so; the record stays |
| `release-0.2.md`, `release-notes.md`, `whats-new.md`, `proposal.md`, `plan.md`, `team-todo.md`, `audit-friday.md` | history pages: their Nav2 and Gazebo are as they were |
| `nav2.yaml`'s name, the one-option switches (`costmap:=`, `navigator:=`, `smoother:=`, `controller:=`) | code, not docs (TODO 232) |
| `todo-done.md`'s links written from the root (`docs/...`) | copied word for word from `TODO.md`; they break only inside that page |
| `simulation.md` (726 lines), `rust-firmware.md` (778), `release-0.2.md` (852) | mostly guide or a release record, not dated logs: no clean cut |
| `car-test` in the testing plan | the plan for TODO 246, not built yet |
