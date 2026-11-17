# Kerbs

Ben, 15 Nov: "we don't want to hit a curb. we need to know if the lidar will see them."
TODO 119.

## The short answer

| Question | Answer |
|---|---|
| Does the flat scan see a kerb? | **No.** It keeps only what stands 0.3 m over the road |
| Does the step finder see a 0.3 m kerb? | **Yes**, from about 8.5 m past the bumper (sim) |
| Does it see a 0.15 m kerb? | **Yes**, but only from about 4.7 m past the bumper (cast clouds) |
| A 0.10 m kerb? | **No**: a step must rise more than 0.10 m |
| Does the car stop before it? | **Yes** in sim: the stop zone stopped it 0.80-0.82 m before a 0.3 m kerb |
| Is it on by default? | DEFAULT_RESULT |
| On the real car? | **Not tried yet** |

## In the simulator

`pixi run sim-check --kerb-only --with kerbs:=true` (our simulator, the road world, a
10 m x 0.3 m x 0.3 m kerb across the road, the planner blind to it):

| Run | Seen from (past the bumper) | Stopped | By |
|---|---|---|---|
| 15 Nov, no step finder, lidar | never | **drove into it** | nothing |
| 16 Nov, step finder, no memory, lidar | 8.6 m | 0.12 m before it | the controller |
| 16 Nov, step finder, no memory, lidar + radar | 8.5 m | **0.24 m into it** | nothing in time |
| 16 Nov, with the memory, lidar | 8.4 m | **0.82 m before it** | the stop zone |
| 16 Nov, with the memory, lidar + radar | 8.4 m | **0.80 m before it** | the stop zone |

Two more runs never started driving: the computer was loaded by other runs and the
simulator's odometry went stale, so the goal was aborted before the car moved (not the
kerbs). Results: `/tmp/kerb/` on Ben's Mac.

## Why the flat scan misses it

```text
   lidar, 1.52 m up
      ╲  ╲  ╲
       ╲  ╲  ╲        everything under 0.3 m is "road" to lidar_scan
   ─────╲──╲──╲─────────▄▄▄  ← a 0.3 m kerb: under the line, so not there
```

The 0.3 m floor is there so a pitched car doesn't see far road as a wall.

## The step finder

A step is **two points close together, one much higher**:

| Rule | Number |
|---|---|
| how close, flat | within 0.3 m |
| how much higher | more than 0.10 m, at most 0.5 m |
| how far out | 12 m from `base_link` |
| noise | 2 points in a 0.1 m square, or it doesn't count |

```text
   ring k+1 ─────────╮
   ring k   ──────╮  │       ● within 0.3 m of ○, flat,
                  │  ●       and more than 0.10 m higher: a step
   road ──────────○──┘
```

One ring alone can't see a step: all its points lie on one cone, so a kerb looks
like a ramp. It takes **two rings**, one on the road just before the kerb and the
next on its face or top.

It runs in the Rust voxels node (`aav_voxels_rs`, `steps.rs`): `kerbs:=true`.

## What it must not see

Tested on clouds cast the way the roof lidar casts them (16 rings, 3 cm noise):

| Ground | Steps found |
|---|---|
| flat road | none |
| slopes of 3°, 6°, 10°, up, down and sideways | none |
| a ramp starting 5 m ahead, up to 10° | none |
| a crowned road, 2.5 % each side | none |
| a speed bump, 7.5 cm high | none |
| rough grass beside the road | none |
| a sign hanging 2.2 m up | none |
| the nine recorded clouds: the ramp, the crowned road | none |

Walls and parked cars do give steps at their feet. That's fine: they are in the way too.

## The memory

The roof lidar's lowest ring meets the road 4.4 m past the bumper. A kerb nearer than
about 3.2 m (a 0.3 m kerb) or 3.8 m (a 0.15 m one) is **under every beam**.

```text
   bumper   1 m   3.2-3.8 m    8.5 m
     │ stop │ blind │ seen by   │
     │ zone │       │ the lidar │
```

So the node keeps every step point for **10 s**, fixed in `odom`, and sends back the
ones within 12 m with each cloud.

| Who reads `/perception/voxels/steps` | What it does |
|---|---|
| local costmap, `kerb_layer` | marks it; nothing clears it |
| stop zone (`collision_monitor`, source `steps`) | stops the car for it |

## Does the real lidar see a kerb?

The roof VLP-16: 16 rings, 2° apart, the lidar 1.52 m up (`vehicle.yaml`), 1.31 m
behind the front bumper.

| Ring | Meets the road, past the bumper | Gap to the next ring | Rings' height apart at that range |
|---|---|---|---|
| -15° | 4.4 m | 0.9 m | 0.21 m |
| -13° | 5.3 m | 1.2 m | 0.24 m |
| -11° | 6.5 m | 1.8 m | 0.28 m |
| -9° | 8.3 m | 2.8 m | 0.34 m |
| -7° | 11.1 m | 5.0 m | 0.44 m |

What that means:

| Kerb | Rings on its face at once | The step finder |
|---|---|---|
| 0.3 m | 1, sometimes 2, out to about 8.6 m | finds it from 8.6 m past the bumper |
| 0.15 m | at most 1; each ring only in a 0.6 m band | only the two lowest rings pair up: from 4.7 m |
| 0.10 m | at most 1 | never (the rule is **more** than 0.10 m) |

A real 0.15 m kerb is hit by enough rings only for the last 4.7 m. At 1 m/s the car
needs about 1 m to stop, so that's enough at our speeds. At 3 m/s it is not.

## In the map

The campus (`pixi run campus`, `tools/campus_world.py`) already had every kerb:

| Where | What |
|---|---|
| OpenStreetMap's tags | **none** on the campus: no `barrier=kerb`, no `kerb=*` |
| so | a 0.3 m kerb along **every** edge of every road and lot: 58 km |
| the static map (`campus.pgm`) and its chunks | everything off the pavement is occupied, the kerb line too: lethal |
| our simulator's campus | the kerbs are real 0.3 m walls the lidar hits |
| the lanes layer (a challenger) | the kerb costs 150, and 1 m past it |

Nothing to add there. If OpenStreetMap ever gets `barrier=kerb` lines, they are drawn
too (`campus_world.py` reads them).

## How to check it

```sh
cd ros/perception/aav_voxels_rs && cargo test -p voxels          # the cast clouds
cargo test -p voxels --release -- --ignored --nocapture steps_range   # the distances
pixi run sim-check --kerb-only --with kerbs:=true                 # the car and a kerb
```
