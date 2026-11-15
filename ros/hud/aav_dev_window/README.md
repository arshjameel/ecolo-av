# aav_dev_window

## Overview

The developer window, in Bevy and egui: one screen for laptops. Pretend data and live ROS
are drop-ins for each other, behind one `Feed`. SWAP isn't built yet. The plan:
[our own simulator](../../../docs/designs/2026-11-05_own-simulator.md).

```text
┌─ GO / STOP and why · e-stop · source ─────────────┐
├────────────────────────────┬──────────────────────┤
│ the scene: the car driving │ debug: what the      │
│                            │ lidar sees, and NDT  │
│                            ├──────────────────────┤
├────────────────────────────┤ the map: click to go │
│ console                    ├──────────────────────┤
│                            │ keys | algorithms    │
└────────────────────────────┴──────────────────────┘
```

## Subdirectories & Boundaries

| File | What it is |
|---|---|
| `src/main.rs` | the app: options, the cubes, the 3D camera, the speed test |
| `src/panels.rs` | the layout and every panel, in egui |
| `src/feed.rs` | what the window is told, and the gates on commands: no Bevy, no ROS, tested |
| `src/pretend.rs` | pretend data, as a `Feed`: no Bevy, no ROS, tested |
| `src/ros.rs` | the node `dev_window`, as a `Feed` (the `ros` feature). Its docstring lists its topics |
| `config/dev_window.yaml` | every topic's name |

It shares logic with the raylib screens: `debug-core` (the cubes' bands, the camera),
`map-core` (map to screen and back), `hud-core` (a picture message into RGBA).

## Interface

```sh
pixi run dev-window                                # the window, pretend data
pixi run demo screen_cubes:=true                   # then, beside it:
pixi run dev-window --live                         # live: may send goals and the e-stop, SIM only
pixi run dev-window --watch                        # listens to the network it's on; no publishers
pixi run dev-window --all-layers --seconds 12      # the speed test: prints the frame rate
pixi run dev-window --cubes 60000 --seconds 8 --screenshot /tmp/window.png
pixi run dev-window --live --lanes maps/campus/campus.osm   # the lane map's lines (pixi run campus)
pixi run dev-window --live --chunks maps/campus/chunks     # the 3 x 3 chunks round the car
```

| Key | Does | Key | Does |
|---|---|---|---|
| G | road cubes: the lidar's waves | M | NDT's map |
| V | outline every live cube | S | live scan |
| N | NDT's cells, dark to light purple (off, the default: cubes by height) | P | plan |
| O | costmap | C, R | from behind it (the default) or the car; reset |
| T | the squiggly line (on) | | |

### The squiggly line (Ben, 7 Nov)

Like a Tesla's line in front of the car, in the 3D scene and on the map (`docs/hud.md`):

| Part | Looks | From |
|---|---|---|
| what the car WILL do, 2 s | bold line: green on the plan, amber off it, red far off; white with no plan | `/ego/trace` |
| what the planner WANTS | thin grey line | `/ego/plan_ahead` |
| where a full brake now stops | white bar across the way (two posts in 3D) | `/ego/stop_point` |
| the words | STOPS IN, ON / OFF PLAN, GOAL or OBSTACLE room: bottom left of the scene | `/ego/ahead` |
| a close-up from above | the same, 8 m ahead, a line every 2 m: right of the scene | all four |

All four are base_link, placed with the car's newest pose (odom in 3D, map on the map).

**The road ahead** (`/world/road_ahead`, map; with the plan, key P): the strip the car
drives by, coloured by its speed limit, green at the strip's top speed to red at a stop; a
red bar across its end, which the car may not pass; on the map, a white ring and the reason
at each stop ([the whole car](../../../docs/designs/2026-11-14_the-whole-car.md#the-road-ahead)).
Silent for 2 s: not drawn. The maths is `hud_core::ahead` and `aav_ego look_ahead`.
Pretend data plays `hud_core::ahead::pretend`'s 12 s loop.

![The squiggly line, pretend data](../../../docs/img/dev-window-squiggly.png)

Live (8 Nov, `--live` beside our own simulator, after its Nav2 goal had finished): parked,
so the stop bar sits at the car and the words say NO PLAN, STOPS IN 0.00 m.
[The picture](../../../docs/img/dev-window-squiggly-live.png). The drive itself was caught
by the HUD (`docs/hud.md`).

**A goal sent elsewhere** (sim-check, the tools, RViz, the map window): while plans keep
coming on `/plan` (within 5 s), the map marks the plan's end in gold and says "driving
to a goal sent elsewhere". Nav2's navigator and ours both replan while they drive.

Mouse on the cubes: drag turns the head and the wheel zooms. From behind, right-drag also moves. On the map: click picks a goal,
right-drag moves, wheel zooms.

## Invariants

- **Its own Cargo workspace, and no `package.xml`**: colcon and the car's builds never compile
  Bevy. The in-car screens stay raylib, on the Pi (Ben, 5 Nov).
- It runs on laptops and desktops, never the Orin, which runs no graphics (Ben, 5 Nov).
- The world keeps ROS's axes (z up), so nothing is converted between frames.
- Logic goes in crates with no engine, so the raylib screens can share it.

## Side Effects

- `--live` also publishes `/hud/dev_window/heartbeat`: frames drawn, 10 Hz. Only sim's
  `watch_screens:=` reads it.
- Opens a window. `--live` publishes `/goal_pose` and `/safety/estop` (only ever true), after
  every gate: developer mode, ROS kept on this computer, every sensor SIM. `--watch` and pretend
  data have no publishers. Cancelling a Nav2 goal isn't built yet.

## Verification

```sh
cd ros/hud/aav_dev_window && pixi run cargo test --release   # the pretend data's tests
pixi run dev-window --all-layers --seconds 12                # needs a screen
pixi run dev-window --live --click-goal 14,0 --seconds 60    # a click and two presses, scripted
```

`tools/test.sh` skips this folder, so Bevy doesn't slow every test run.
