# Wrapping up: the plan

**The capstone has built more than it needs. What is left is to prove it.** This page is the
plan Ben set on 23 November 2026: one test regime with rungs, in the virtual world first
and then on the car; the commands and screens made simple; a final report from the
results. Nothing new is built unless a row here asks for it.

> **The short version**
> 1. **Ten rungs**, named for what the car can do. Each has one question, one graph, one
>    pass mark, a result in the simulator and a slot for the car's.
> 2. **Virtual first.** The laptop runs the car's own software against Ravensim, the same
>    way the car runs it.
> 3. **Then the car**, the same rungs, with the simulator's curve beside the car's. Where
>    they differ, the model's numbers are fixed.
> 4. **Fewer commands, four screens, one editor.**
> 5. **The final report** is written from the rungs' graphs, one chapter a layer.

## 1. The rungs

A rung opens when the ones under it have passed. S runs beside every rung. The list the
tools read is `tools/ladder/rungs.yaml`; a rung not built is shown as "not built yet".

| # | Rung | The question | Its graph | Passes when (proposed) | Today |
|---|---|---|---|---|---|
| 1 | On stands: accelerate, brake, turn | do the actuators follow a command, the right way? | wheel speed and steering, asked against measured | each follows within its lag; every fault stops the car | **a rung, in the simulator** (23 Nov): `pixi run ladder run 1`, or **Run** on the tests screen; its answer is a verdict line, three graphs with their numbers and the faults table, saved as one page ([Proving it](proving-it.md#rung-1-on-stands-accelerate-brake-turn)). The car's side is a slot: no car run yet |
| 2 | On the ground: speed up, slow down | how fast does it speed up and stop? | speed against time; stopping distance against speed | the model's speed within 10 %, its stop within 0.2 m | built (`speed-up`, `brake`) |
| 3 | A straight line: speed up, hold, slow down | does it hold a speed, and drive straight? | speed held; drift to the side | wanders under 0.1 m/s; drifts under 0.2 m in 10 m | built in part (hallway session; a hold test is TODO 266) |
| 4 | Navigate, an empty costmap | does the whole stack drive A to B with nothing in the way? | the path driven over the plan; metres off it | reaches the goal; under 0.3 m off the plan | the stack does it; no rung |
| 5 | Navigate, a hand-painted obstacle | does it go round what we drew? | the same, with the obstacle | never touches; reaches the goal | a ghost object only; painting is to build |
| 6 | Navigate, the lidar's costmap | does it go round what it sees? | the same, with what the lidar marked | never touches; reaches the goal | runs in the simulator; no rung |
| 7 | Things around the car, moving ones too | does it find them and follow them? | each thing's track against the truth | found within 1 s; position within 0.5 m | things are tracked; nothing predicts |
| 8 | What is it? (the cameras) | does it name what it sees? | right and wrong names, by kind | proposed with the perception lead | a detector exists; no rung |
| 9 | Rules of the road (behaviour) | does it do the right thing: stop at a stop sign, slow for the road ahead, stop at the goal, stay inside the fence? | speed against distance to each stop, with the speed cap | stops within 0.5 m of each mark; never over the cap | the behaviour planner runs in the simulator; no rung |
| S | Sensors: IMU, GPS, cameras, lidar, radar | is each one alive, on time, and believable? | rate and noise for each | each at its documented rate; still when the car is still | checked inside rung 1; no rung of its own |

On top, the same rungs run with one part swapped, to compare algorithms:

| Compare | On rungs | Scored by |
|---|---|---|
| Path followers: MPPI, MPC, pure pursuit | 4 to 6 | metres off the plan, sway, time |
| Planners: lattice, diffusion | 4 to 6 | the path's length and turns, how often the guard refuses |
| With and without the barrier and the filter (outside the capstone's scope: Ben's MASc research) | 5, 6 | least gap, who stopped the car |

## 2. One software, two worlds, and both at once

| Mode | What runs | State |
|---|---|---|
| Virtual | the car's software on the laptop, against Ravensim | works |
| Car | the same software on the car | being installed |
| Both (the twin) | the laptop listens to the car, and the virtual world shows the car where it really is (its GPS and pose), with the tests' graphs filling live | to build |

One rule for the twin: **the laptop only listens.** Exactly one backend drives the car, and
a simulator must never publish onto the car's network (the sensor mux refuses it today, and
that stays). The virtual world takes the car's data in; nothing goes back.

## 3. The commands: 71 today, four for every day

Ben, 25 Nov: "I want to be able to do a full sweet of tests on a virtual car using the full
ros messaging system."

| Want to | Today | The one command |
|---|---|---|
| Start the virtual car | `sim`, `demo`, `dry-run`, `ravensim`, `sim-cameras` | **`pixi run sim`**: Ravensim as the car, every topic the car has (step 1) |
| Start the real car | `car` | **`pixi run car <computer>`** |
| Open the screens | `hud`, `map`, `debug-hud`, `hw-panel`, `dev-window`, `nav-view`, `ladder-window`, `rviz` | **`pixi run screens`**: windows 1, 2 and 4 below, for whichever car runs |
| Run the tests | `ladder`, `car-test`, `actuator-test`, `sim-check`, `bench-check`, `measure` | **`pixi run ladder`**: window 3; `ladder all --sim` is the full suite on the virtual car, one report |

The rest stay as what a rung runs, or as tools (maps, firmware, set-up). The old names keep
working until the new ones have been used for a while; then they go. `sim.launch.py`'s 127
switches become a short list of named set-ups (as `stacks.yaml` does for algorithms).

## 4. The screens: eight become four

Ben, 25 Nov: the HUD and the map are the car's two screens; the hardware panel goes on the
map's screen; the tests and the hardware belong together; Ravensim, not RViz, shows the car
moving.

**The rule:** every screen only talks ROS. None knows whether the car is real or Ravensim.

```
 Ravensim ─┐                                     ┌─ 1 HUD            (car screen 1)
           ├── the same ROS topics, both ways ───┼─ 2 Map + Hardware (car screen 2)
 the car ──┘   (/sensors /vehicle /safety ...)    ├─ 3 Test bench     (laptop)
                                                  └─ 4 World view     (laptop)
```

| # | Window | Where | What is in it | Replaces |
|---|---|---|---|---|
| 1 | **HUD** | car screen 1 (raylib, Pi) | the driver's screen | `hud` |
| 2 | **Map + Hardware** | car screen 2 (raylib, Pi) | the street map; one key flips to the hardware page: every number, commanded against measured as graphs | `map`, `hw-panel` |
| 3 | **Test bench** | laptop (raylib) | the ladder: automated tests, Run, results, reports; live graphs; small steps by hand (speed +0.1 m/s, steering +2°) | `ladder-window`, `hw-panel`'s drive keys |
| 4 | **World view** | laptop (Bevy) | Ravensim's 3D picture (the real car: the same from its data), with the occupancy, costmaps, plan and lidar cubes on top | `dev-window`, `nav-view`, `debug-hud`, `rviz` |

On a laptop, windows 1 and 2 are the very programs the car runs: in the simulator you see
what the car's screens would show. Windows 2 and 3 share their graphs, in raylib, so the
test bench can run on the car's screen too. The ESP32's own read-only page (`wifi-status`)
stays for the board's direct numbers, on the bench.

| Editor | For | Today |
|---|---|---|
| The map editor | chunks, street map, NDT's map, labels | built (`map-edit`) |
| Painting a costmap by hand | rung 5 | to build, inside the map editor |

RViz opens only when asked for by name (since 23 Nov). Gazebo went on 16 Nov.

### The steps for the screens and the virtual car (Ben, 25 Nov: "yes")

| Step | What | Done when |
|---|---|---|
| 1 | **Ravensim as the car**: every topic the car has; `pixi run check virtual` fails if the simulator ever drifts from the car | the check passes |
| 2 | the hardware page into the map program | the page on the car's screen size |
| 3 | the test bench: the ladder, the graphs, the small steps | rung 1 run from it |
| 4 | the world view takes `debug-hud` and `nav-view`; the RViz layouts deleted | nothing opens RViz |
| 5 | the four commands; the old names become aliases | README's command list |
| 6 | rungs 2 to 9, and the full suite on the virtual car | `ladder all --sim`: one report |

**Step 4, where it is (25 Nov):** the world view has the debug screen's pages (SENSORS,
WHERE AM I, WHAT IT SEES, PLAN), each localizer's car and AMCL's particles, from
`debug-core`'s own state ([the table](hud.md#the-debug-hud-into-the-world-view)). Left: the
lidar's own kept cubes (not ported: the car's cubes stand for them), NDT's match as
ellipses. `debug-hud`, `nav-view` and the `.rviz` layouts stay until step 5.

## 5. The final report

One chapter a layer, the same five parts each, filled from the rungs' results:

| Chapter | Its rungs |
|---|---|
| The car and its model | 1, 2, 3 |
| Sensors | S |
| Localization and mapping | S, 4 |
| Perception | 7, 8 |
| Planning | 4 to 6, lattice against diffusion |
| Following | 4 to 6, the followers compared |
| Behaviour | 9 |
| Safety | 1 (the faults), 5, 6 |

Each chapter: what the layer does; the algorithms we tried; how each works; what we
measured, with its graph; which we chose and why.

## 6. The order

| Step | What | Done when |
|---|---|---|
| 1 | The ladder rebuilt round the ten rungs, virtual only; rungs 1 to 3 first | each of rungs 1 to 3 runs from the tests screen and saves a graph and a report. **23 Nov: the ladder is the ten rungs, and rung 1 is done** (run by the tool's own command beside the open simulator; its button was pressed once, on 24 Nov, over another run: that found two faults, both fixed). Rungs 2 and 3: not built |
| 2 | Rungs 4 to 6, and S | the same |
| 3 | The commands and screens of sections 3 and 4 | a new person starts the virtual car and its screens with two commands |
| 4 | Rungs 7 to 9; the algorithm comparisons | the same |
| 5 | The twin, and the simulator beside the car on one graph | the car's rung 1 beside the simulator's |
| 6 | The car: the rungs in order | each rung's car result recorded |
| 7 | The final report | every chapter has its graphs |

A model of what other cars and people will do next is not in this plan: it is research
for after the capstone.
