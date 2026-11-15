# 0.4-rc1: start here

The first release candidate of 0.4. **Beta, not 1.0** (Ben, 14 Nov: "We are still in beta
stages"). It is about refining what we have, not adding more.

## What 0.4 means

| Must-have | Where it stands |
|---|---|
| **Our own stack** drives by default (`stack:=ours`) | ✅ since 14 Nov: our costmap, planner, navigator, follower, behaviour, all in Rust where it is heavy |
| **Reverse** everywhere the plan says | ✅ on the lot and the campus; a three-point turn in a narrow street still to test (TODO 174) |
| **No oscillation** (no swaying) | 🔧 ours sways 3-5 times more than Nav2: being fixed (TODO 168) |
| Nav2 and Gazebo only as yardsticks | ✅ `stack:=stock`, `simulator:=gazebo`; deleted after 0.4-rc1 is scored (TODO 184) |

## One command per job

| I want to | Run |
|---|---|
| set up a new laptop | [Getting started](getting-started.md) (30 minutes) |
| drive the simulated car | `pixi run sim`, then `pixi run dev-window --live` |
| see everything at once | `pixi run demo` (`--debug` adds the seeing screen) |
| check it all works | `pixi run sim-check` (2 min) |
| compare our stack with Nav2 | `pixi run measure controllers --stacks nav2 own --reversing` |
| drive Carleton's campus | `pixi run campus`, then `pixi run sim world:=campus` ([Campus](campus.md)) |
| run every test | `pixi run test` |
| go to the car | [Car day](car-day.md) |

## How ours compares (14 Nov, 3 runs each, reversing on)

| | Ours | Nav2 |
|---|---|---|
| Legs reached, lot + campus | **21 of 21** | 16 of 21 |
| Off the path, on average | **0.12 m** | 0.31-0.44 m |
| Sway (side-to-side), campus | 0.66 m/s² | **0.12 m/s²** |

Ours wins on reaching goals and tracking; Nav2 still drives more smoothly. The details:
`TODO.md` rows 167 and 168.

## What's new since 0.3

| Area | New |
|---|---|
| Driving | our stack is the default; our navigator serves Nav2's action names, so no goal sender knows which runs |
| Planning | Hybrid-A* in our planner (`planner_search:=hybrid`), a challenger; smooth lattice moves tried (no win) |
| Behaviour | a new feature type: the behaviour planner, the road ahead, the stopping law ([Features](features.md#behaviour)) |
| Safety | the watchdog warns when one of our own nodes stops (never stops the car for it) |
| Simulator | `/clock` rate (`clock_hz:=`), seeded wheel noise and slip (`wheel_noise:=`, `wheel_slip:=`), the campus widens its fence itself |
| Credit | [Acknowledgements](acknowledgements.md): every project and paper we built on |

## Not in 0.4-rc1

| Left | Why |
|---|---|
| The real car | needs the hardware lead's go-ahead (TODO 173) |
| Lanelet2 maps and the three map levels | designed, not built (TODO 175) |
| Heavy Python nodes in Rust | voxels first (TODO 185) |

The whole list: `TODO.md`. What works today: [Status](status.md).
