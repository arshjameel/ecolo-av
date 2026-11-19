# 0.4-rc1: start here

The first release candidate of 0.4. **Beta, not 1.0** (Ben, 14 Nov: "We are still in beta
stages"). It is about refining what we have, not adding more.

**Going to the car (Wed 18 Nov):** [0.4-rc1 on the car](release-0.4-rc1.md): install,
first checks, what is untested there, the seven blockers.

## What 0.4 means

| Must-have | Where it stands |
|---|---|
| **Our own stack** drives by default (`stack:=ours`) | ✅ since 14 Nov: our costmap, planner, navigator, follower, behaviour, all in Rust where it is heavy |
| **Reverse** everywhere the plan says | ✅ on the lot and the campus; a three-point turn in a 6.5 m street works, with 4-6 changes of direction where 3 are planned (TODO 174) |
| **No oscillation** (no swaying) | ✅ in the sim: at the last side-by-side (15 Nov afternoon, 2 runs each) ours swayed as Nav2 on the lot (0.021 against 0.018 m/s²) and less on the campus (0.099 against 0.131); that morning it was about 2 times Nav2's. On mapped road, `path:=lane` sways least of all. Untested on the car (TODO 167, 168) |
| Nav2 and Gazebo only as yardsticks | ✅ both deleted early: Nav2 17 Nov ([as it was](history/nav2-as-it-was.md)), Gazebo 16 Nov ([as it was](history/gazebo-as-it-was.md)); TODO 184. Ravensim, ours, is the only simulator |

## One command per job

| I want to | Run |
|---|---|
| set up a new laptop | [Getting started](getting-started.md) (30 minutes) |
| drive the simulated car | `pixi run sim`, then `pixi run dev-window --live` |
| see everything at once | `pixi run demo` (`--debug` adds the seeing screen) |
| check it all works | `pixi run sim-check` (2 min) |
| compare options of our stack | `pixi run measure controllers --stacks own own_calm --reversing` ([the league](league.md)) |
| drive Carleton's campus | `pixi run campus`, then `pixi run sim world:=campus` ([Campus](campus.md)) |
| run every test | `pixi run test` |
| go to the car | [Car day](car-day.md) |

## How ours compares: the league's final (16 Nov)

Nav2's last score before it was deleted, on the same three tours in Ravensim:

| | Ours | Nav2 |
|---|---|---|
| Legs reached (lot, campus, University Drive) | **30 of 30** (3 runs) | 14 of 20 (2 runs) |
| Off the path, campus tour | **0.17 m** | 0.83 m |
| Time over the centre line, University Drive | **0 %** | 13 % |

The final configuration: **no champion changed**; `pose:=ndt_ekf` is the one to try
next ([the league, 16 Nov](league-results-2026-11-16.md#final-configuration)).

On University Drive (16 Nov, 3 runs each), ours with `path:=lane` (now its default):
0 % of the time over the centre line (61 % before), 0.11 m off the lane centre, sway 0.055 m/s².

## What's new since 0.3

| Area | New |
|---|---|
| Driving | our stack is the default; our navigator serves Nav2's action names, so no goal sender knows which runs |
| Planning | Hybrid-A* in our planner (`planner_search:=hybrid`), a challenger; smooth lattice moves tried (no win) |
| Behaviour | a new feature type: the behaviour planner, the road ahead, the stopping law ([Features](features.md#behaviour)) |
| Safety | the watchdog warns when one of our own nodes stops (never stops the car for it) |
| Simulator | `/clock` rate (`clock_hz:=`), seeded wheel noise and slip (`wheel_noise:=`, `wheel_slip:=`), the campus widens its fence itself |
| Credit | [Acknowledgements](acknowledgements.md): every project and paper we built on |
| Lanes (16 Nov) | `path:=lane`: the plan follows the right lane's centre on mapped road (the lot falls back to the lattice); now `ours`' path |
| Map chunks (16 Nov) | NDT on the campus loads 64 m chunks (ready in 0.02 s, not 2.5 s); the things list drops low cubes on the map's walls (9.9 things to 1.7, every person still found); the dev window draws the chunks (key H) |
| Safety (16 Nov) | the watchdog latches OVERSPEED (a stuck throttle) as it does STUCK steering |
| Rust (17-18 Nov) | the Python nodes the car runs alone, ported: `joy_alive`, the stop-sign watcher, the lane finder, the point driver, `arm_when_open`, `camera_stream`; each Python one falls back with `*_impl:=python` |
| AMCL (18 Nov) | ours in Rust (`aav_amcl_rs`), the default; a tie with stock `nav2_amcl` on the lot (TODO 231) |
| Navigator (18 Nov) | a follower's feedback counts from the moment a path is sent, so a lost answer no longer aborts a drive (TODO 240) |
| Speed loop (18 Nov) | `speed_loop_model:=`: the car runs one loop while the planners model another (TODO 163) |

## Not in 0.4-rc1

| Left | Why |
|---|---|
| The real car | needs the hardware lead's go-ahead and the seven blockers ([on the car](release-0.4-rc1.md); TODO 158, 173) |
| The middle level planning far goals in chunks | NDT and the things use chunks; the planner's rolling window loses far campus goals, so it stays a challenger (TODO 175) |
| The `firm` speed loop | stops on the line, but sways more on the lot: a challenger (TODO 163) |
| More heavy Python nodes in Rust | by 18 Nov the defaults are Rust for ego, behaviour, the watchdog, localization_health, lidar_scan, things, the muxes, the Motor Pi's bridge and RC, `joy_alive`, the stop-sign watcher, the lane finder, the point driver, `arm_when_open`, `camera_stream` and AMCL ([Languages](languages.md)); the rest in TODO 185 |

The whole list: `TODO.md`. What works today: [Status](status.md).
