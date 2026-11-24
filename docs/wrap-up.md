# Wrapping up: the plan

**The capstone has built more than it needs. What is left is to prove it.** This page is the
plan Ben set on 23 November 2026: one test regime with rungs, in the virtual world first
and then on the car; the commands and screens made simple; a final report from the
results. Nothing new is built unless a row here asks for it.

> **The short version**
> 1. **Nine rungs**, named for what the car can do. Each has one question, one graph, one
>    pass mark, a result in the simulator and a slot for the car's.
> 2. **Virtual first.** The laptop runs the car's own software against Ravensim, the same
>    way the car runs it.
> 3. **Then the car**, the same rungs, with the simulator's curve beside the car's. Where
>    they differ, the model's numbers are fixed.
> 4. **Fewer commands, four screens, one editor.**
> 5. **The final report** is written from the rungs' graphs, one chapter a layer.

## 1. The rungs

A rung opens when the ones under it have passed. S runs beside every rung.

| # | Rung | The question | Its graph | Passes when (proposed) | Today |
|---|---|---|---|---|---|
| 1 | On stands: accelerate, brake, turn | do the actuators follow a command, the right way? | wheel speed and steering, asked against measured | each follows within its lag; every fault stops the car | built (`car-test --stands`, on jacks in the simulator) |
| 2 | On the ground: speed up, slow down | how fast does it speed up and stop? | speed against time; stopping distance against speed | the model's speed within 10 %, its stop within 0.2 m | built (`speed-up`, `brake`) |
| 3 | A straight line: speed up, hold, slow down | does it hold a speed, and drive straight? | speed held; drift to the side | wanders under 0.1 m/s; drifts under 0.2 m in 10 m | built in part (hallway session; a hold test is TODO 266) |
| 4 | Navigate, an empty costmap | does the whole stack drive A to B with nothing in the way? | the path driven over the plan; metres off it | reaches the goal; under 0.3 m off the plan | the stack does it; no rung |
| 5 | Navigate, a hand-painted obstacle | does it go round what we drew? | the same, with the obstacle | never touches; reaches the goal | a ghost object only; painting is to build |
| 6 | Navigate, the lidar's costmap | does it go round what it sees? | the same, with what the lidar marked | never touches; reaches the goal | runs in the simulator; no rung |
| 7 | Things around the car, moving ones too | does it find them and follow them? | each thing's track against the truth | found within 1 s; position within 0.5 m | things are tracked; nothing predicts |
| 8 | What is it? (the cameras) | does it name what it sees? | right and wrong names, by kind | proposed with the perception lead | a detector exists; no rung |
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

## 3. The commands: 68 today

| Want to | Today | Proposed |
|---|---|---|
| Start the virtual car | `sim`, `demo`, `dry-run`, `ravensim`, `sim-cameras` | **`pixi run sim`** (the car, virtual); screens are their own command |
| Start the real car | `car` | **`pixi run car`** |
| Open the screens | `hud`, `map`, `debug-hud`, `hw-panel`, `dev-window`, `nav-view`, `ladder-window`, `rviz` | **`pixi run screens`**: picks the same four for either car |
| Run the tests | `ladder`, `car-test`, `actuator-test`, `sim-check`, `bench-check`, `measure`, `barrier-watch` | **`pixi run ladder`** is the front door; the rest stay as what a rung runs |
| Watch the car from the laptop | `view`, `dev-window --watch` | **`pixi run twin`** |

The old names keep working until the new ones have been used for a while; then they go.
`sim.launch.py` has 127 switches: the plan is a short list of named set-ups (as
`stacks.yaml` does for algorithms), not fewer abilities.

## 4. The screens and editors

| Screen | Shows | Today | Plan |
|---|---|---|---|
| World | the car in its world, 3D and top-down: the map, the plan, the costmap, what it sees | the developer window and `nav-view` | keep; it replaced RViz and Gazebo's picture |
| Debug | what each layer believes: the localizers, the cubes, who is driving | `debug-hud`, and a debug page in the map window | **needs attention**: one debug screen, its pages named for the layers |
| Hardware | every sensor's number, the actuators, calibration | `hw-panel` | keep; add "on stands" |
| Tests | the rungs, run, result, graph, simulator beside car | `ladder-window` | rebuild round the nine rungs |
| In the car | the driver's HUD and street map | `hud`, `map` (raylib, on the Pi) | keep as they are |

| Editor | For | Today |
|---|---|---|
| The map editor | chunks, street map, NDT's map, labels | built (`map-edit`) |
| Painting a costmap by hand | rung 5 | to build, inside the map editor |

RViz opens only when asked for by name. Gazebo went on 16 Nov.

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
| Safety | 1 (the faults), 5, 6 |

Each chapter: what the layer does; the algorithms we tried; how each works; what we
measured, with its graph; which we chose and why.

## 6. The order

| Step | What | Done when |
|---|---|---|
| 1 | The ladder rebuilt round the nine rungs, virtual only; rungs 1 to 3 first | each of rungs 1 to 3 runs from the tests screen and saves a graph and a report |
| 2 | Rungs 4 to 6, and S | the same |
| 3 | The commands and screens of sections 3 and 4 | a new person starts the virtual car and its screens with two commands |
| 4 | Rungs 7 and 8; the algorithm comparisons | the same |
| 5 | The twin, and the simulator beside the car on one graph | the car's rung 1 beside the simulator's |
| 6 | The car: the rungs in order | each rung's car result recorded |
| 7 | The final report | every chapter has its graphs |

A model of what other cars and people will do next is not in this plan: it is research
for after the capstone.
