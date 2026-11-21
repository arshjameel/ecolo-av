# aav_dev_window

## Overview

The developer window, in Bevy and egui: one screen for laptops. Pretend data and live ROS
are drop-ins for each other, behind one `Feed`. SWAP (15 Nov) swaps `aav_track`'s follower live in the
simulator; the other choices are made at launch. The plan:
[our own simulator](../../../docs/designs/2026-11-05_own-simulator.md).

```text
┌─ GO / STOP and why · DRIVING: who · e-stop · source ┐
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
| `src/map_nav.rs` | how the map panel moves: follow the car, the whole map, or free; drag, wheel, pinch, F: no Bevy, no egui, tested |
| `src/chunks.rs` | the chunks loaded (`map_chunks::wanted`), their outlines, labels and saved maps: no Bevy, no egui, tested |
| `src/lanes.rs` | the lane map's lines and their looks (`lanelet-map`) |
| `src/ros.rs` | the node `dev_window`, as a `Feed` (the `ros` feature). Its docstring lists its topics |
| `config/dev_window.yaml` | every topic's name |
| `src/map_edit/main.rs` | the map editor (`pixi run map-edit`, TODO 251): its own binary, `map_edit`; no ROS. The edits are the `map-edit` crate's ([docs/map-editor.md](../../../docs/map-editor.md)) |
| `src/map_edit/tool.rs` | its tools: a drag into a box, an align drag or key: no Bevy, no egui, tested |
| `src/map_edit/streets.rs` | its street map: the tiles round the map, read once (`map-core`) |

It shares logic with the raylib screens: `debug-core` (the cubes' bands, the camera),
`map-core` (map to screen and back), `hud-core` (a picture message into RGBA).

## Interface

```sh
pixi run dev-window                                # the window, pretend data
pixi run demo screen_cubes:=true                   # then, beside it:
pixi run dev-window --live                         # live: may send goals and the e-stop, SIM only
pixi run dev-window --watch                        # listens to the network it's on; no publishers
pixi run dev-window --all-layers --seconds 12      # the speed test: prints the frame rate
pixi run dev-window --live --map-whole             # the map panel starts on the whole map
pixi run dev-window --cubes 60000 --seconds 8 --screenshot /tmp/window.png
pixi run dev-window --live --lanes maps/campus/campus.osm   # the lane map's lines (pixi run campus)
pixi run dev-window --live --chunks maps/campus/chunks     # the chunks loaded, their saved map
pixi run map-edit bags/<drive>/map                         # the map editor: no ROS (docs/map-editor.md)
```

The status strip says who is driving (19 Nov, TODO 247): **DRIVING: keyboard (90)**, from
`/control/command_mux/state` (`aav_mux_state`), grey `DRIVING: ?` when it is quiet. The
map window's debug page (key D) shows the whole command path.

| Key | Does | Key | Does |
|---|---|---|---|
| G | the ground: road, and the lidar's rings on raised grass (off) | M | NDT's map |
| V | outline every live cube | S | live scan |
| N | NDT's cells, dark to light purple (off, the default: cubes by height) | P | plan |
| O | costmap | C, R | from behind it (the default) or the car; reset |
| T | the squiggly line (on) | K | the things, with their names (on) |
| H | the chunks loaded, their saved map, and the route (on) | L | the lane map's lines (on) |
| F | the map: follow the car, or the whole map (also its buttons) | | |

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

**The things** (`/world/things`, odom; key K, TODO 192): each thing near the car as a box
in its kind's colour, in the 3D scene and on the map, with its name over it.

| Kind | Colour |
|---|---|
| person | yellow |
| car, truck, bus, motorcycle | blue |
| stop sign | red |
| any other name | green |
| unknown (no camera named it) | orange, no name |

The colours are `hud_core::things`' (the HUD's and the RViz markers' too). Silent for 2 s:
not drawn. Pretend data: a person crossing ahead, a stop sign, a car, an unnamed box.

**The three levels** (TODO 175, step 7; [the design](../../../docs/designs/2026-11-15_three-levels.md)):
on the map, with `--chunks` and `--lanes`; key H.

| Level | Drawn as | From |
|---|---|---|
| zoomed out: the route | thick violet line | `/drivers/nav/route/path` (street_router), latched |
| middle: the chunks loaded | each chunk's square, labelled `(i, j) fine` (blue; the car's own light blue) or `(i, j) coarse` (amber) | `chunks::loaded`: `map_chunks::wanted` on the car and the route (the plan with no route), 100 m ahead |
| middle: the saved static map | each loaded chunk's `.pgm`, at its detail (0.25 m or 1 m) | `--chunks` folder (`chunk-map`), read once per chunk |
| the lanes | white lines, grey dashes, amber kerbs, red stop lines | `--lanes` (a Lanelet2 `.osm`) |

With `--chunks`, the map fits the chunks loaded until you move it. Pretend data has a
route out of its lot (`pretend::route`). Seen 16 Nov only off screen (the screen was
locked): the same `chunks.rs` and the campus files drawn into a picture.

**A goal sent elsewhere** (sim-check, the tools, RViz, the map window): while plans keep
coming on `/plan` (within 5 s), the map marks the plan's end in gold and says "driving
to a goal sent elsewhere". Our navigator replans while it drives (as Nav2's did).

Mouse on the cubes: drag turns the head and the wheel zooms. From behind, right-drag also moves.

On the map (Ben, 17 Nov: "The map can't be scrolled"; `src/map_nav.rs`, map-core's view):

| Do | To |
|---|---|
| click (no drag) | pick a goal |
| drag, either button | move the map: it stops following the car |
| wheel, or pinch | zoom at the pointer |
| F, or **Follow car** | back to the car (F again: the whole map) |
| **Whole map** | the whole campus; `--map-whole` starts so |

It starts on the car, sized to the chunks loaded (the three levels), or 160 m on a big map
with no chunks; a small map is shown whole. Untested on screen yet (the screen was locked
on 17 Nov): its tests drive it.

### The ground, hidden (17 Nov)

Ben, 17 Nov: "lets get rid of the waves the lidar made ... those circles don't help
anymore". Each lidar ring that hits flat ground makes a circle of cubes. Two causes:

| Rings | Why they showed | Now |
|---|---|---|
| on the road (`/perception/voxels/ground`) | banded from 0 m above base_link: road a centimetre up was "< 0.75 m" | always ground |
| on the campus's grass and pavements, a 0.30 m kerb up (`tools/campus_world.py`) | the voxels node rightly calls them obstacles (`/perception/voxels/cubes`, 0.28-0.33 m) | flat raised ground: ground |

`debug_core::flat` decides: a cube under 0.45 m is ground unless the road or a taller
cube is within 0.5 m of it (seen from above), so the kerb's edge and a person's feet stay.
Key G shows the ground again, grey. The cubes on the map (key S) are as they were. The debug HUD does the same.

![Before and after, a campus scan](../../../docs/img/dev-window-ground-hidden.png)

Drawn off screen from a real campus scan (Ravensim, 17 Nov), through `debug_core::flat`,
from behind the car. The live window [before](../../../docs/img/dev-window-rings-before.png).

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
  every gate: developer mode, ROS kept on this computer, every sensor SIM. Behind the same
  gates it calls `/navigate_to_pose/_action/cancel_goal` (CANCEL THE GOAL: every goal, the navigator's or
  our navigator's) and `/track/set_parameters` (SWAP: `tracker`, which the track node takes only
  with `live_swap:=true`); the answers go to the console. `--watch` and pretend data have no
  publishers and call nothing.

## Verification

```sh
cd ros/hud/aav_dev_window && pixi run cargo test --release   # the pretend data's tests
pixi run dev-window --all-layers --seconds 12                # needs a screen
pixi run dev-window --live --click-goal 14,0 --seconds 60    # a click and two presses, scripted
```

`tools/test.sh` skips this folder, so Bevy doesn't slow every test run.
