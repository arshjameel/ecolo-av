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
| Does the car slow for it first? | **Yes** since 19 Nov (TODO 219): the behaviour planner slows for a kerb on the road ahead as for an obstacle, so the stop zone isn't the first to act. Scored in stage 2 |
| Is it on by default? | **Yes**, in `stack:=ours` (the simulator's) since 16 Nov. Not on the car yet |
| On the real car? | **Not tried yet** |

## In the simulator

`pixi run sim-check --kerb-only --with kerbs:=true` (on by default now; our simulator, the road world, a
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

## No worse on the tours

`pixi run measure controllers --stacks own_calm_chunks own_calm_chunks_kerbs --tours lot
university --runs 2 --reversing` (16 Nov), our stack without and with the kerbs:

| Tour | Without | With the kerbs |
|---|---|---|
| University Drive: legs reached | 6/6 | 6/6 |
| University Drive: time, mean speed | 357 s, 2.07 m/s | 352 s, 2.10 m/s |
| University Drive: stop-zone stops | 0 | 0 |
| the lot: legs reached | 6/8 | 6/8 |
| the lot: stop-zone stops | 3 | 1 |
| the voxels node's time a cloud | - | 3.3-3.9 ms (University), 4.8-5.2 ms (the lot) |

Every leg missed, both ways, was the computer being too busy (the local costmap or the
odometry late), not a kerb. The campus's kerbs line every road there: no phantom stop.

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

Walls, parked cars and people give steps at their feet too, and up their faces (each
ring hits a post 0.1-0.5 m above the one below). Since 20 Nov (TODO 189) those are
dropped: a step within 0.3 m of a point more than 0.5 m over the road (and under 2 m)
is the foot of something taller than any kerb (`kerbs_only`). That thing is a cube, so
the costmap and the stop zone see it anyway; and it may walk away (below).

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
| behaviour planner (`kerbs:=true`, TODO 219) | slows for a kerb on the road ahead, as for an obstacle (`kerb_ahead`) |

### Standing before a kerb (19 Nov, TODO 219)

Before, a kerb was forgotten 10 s after it was last seen. A car that stood before a kerb
longer than that, under every beam, forgot it, and the stop zone let it on.

Now a point **under every beam** doesn't age: nearer the lidar than its height / tan 15°
(5.7 m for the roof lidar, 1.52 m up). The lidar can't see it again there, and can't see
it gone either. Once the car backs away and it is in view, it ages again: 10 s.

| Where the step point is | It is forgotten |
|---|---|
| in the lidar's view | 10 s after it was last seen |
| under every beam (within 5.7 m of the lidar) | never, while it stays there |
| further than 12 m | at once |

`cargo test -p voxels memory` (`a_kerb_under_every_beam_is_kept_while_the_car_stands_before_it`).

### A walker's foot (20 Nov, TODO 189)

`sim-check --moving` failed twice: the car waited for a person crossing 3 m ahead, then
stood in APPROACH(obstacle) for good. The person's foot made steps as they crossed, under
every beam, so the memory kept them for ever: a "kerb" across the lane. The behaviour
planner slowed for it (`left` -0.3 m) and the stop zone held the car.

| Step | Before | Since 20 Nov |
|---|---|---|
| a post's foot and face | steps, kept | dropped (`kerbs_only`): the cubes have it |
| a kerb, even beside a post | kept | kept (only within 0.3 m of the post is dropped) |

`cargo test -p voxels steps` (`a_walker_crossing_under_every_beam_leaves_no_kerb_behind`:
the old way keeps the walker's trail, the new one nothing).

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
needs about 1 m to stop, so that's enough at our speeds. At 3 m/s it is not. The
behaviour planner now slows from the first sight (TODO 219); whether that is enough at
3 m/s is stage 2's score. A 0.10 m kerb is still never seen.

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
pixi run sim-check --kerb-only                                   # the car and a kerb (ours: kerbs on)
```
