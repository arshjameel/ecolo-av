# The whole car: every design merged into one

* **Author:** Ben (the ask), written up 14 Nov 2026 from Fable's review of every design
* **Date:** 2026-11-14
* **Status:** Building (Ben, 14 Nov: "build the unified stack ... I would like to keep our code
  too"; nothing is cut). Built: the road ahead's message, its writer (the behaviour planner)
  and its first reader (ego holds Nav2 to it, R1's first form). TODO 153-160. W1-W7, R1-R6 open.
* **Builds on:** [one shape](2026-11-13_one-shape.md) (the nine levels, the league, the
  switches: all kept). This page changes how its lane layer works, and adds what it left out.

Ben, 13-14 Nov: "a review of everything with the plan of merging everything into the best
version possible", "what would improve navigation and object avoidance and perception",
"what am I missing", and "come up with some better ideas that merge stuff together into a
fully merged version". His words: `docs/history/messages-2026-11-14.md`.

## The idea in one line

Three things are shared by every level: **one picture of the world**, **one car model**, and
**one scorecard**. The picture's main part is new: **the road ahead**, one message that says
where the car may be, how fast, and where it must stop.

```text
                      ┌───────────── ONE SCORECARD (the league) ─────────────┐
                      │  the same scenarios in the sim · the same recordings │
                      │  · shadow runs on the car · one table of scores      │
                      └──────────────────────────────────────────────────────┘
 1 routing ─┐
 2 pose ────┤      ONE PICTURE OF THE WORLD                       ONE CAR MODEL
 3 sensing ─┴──►  ┌────────────────────────────────┐        ┌───────────────────────┐
                  │ the grid     what is solid     │        │ how it speeds up,     │
                  │ the things   who, where, speed │        │ steers and stops,     │
                  │ the road ahead  where I may be,│        │ with the lags and     │
                  │   how fast, where I must stop  │        │ the measured numbers  │
                  └───────────────┬────────────────┘        └──────────┬────────────┘
                                  ▼                                    ▼
          5 behaviour writes the road ahead ─► 6 path ─► 7 follow ─► 8 ego ─► 9 the locks
                                  every one of them reads the same two boxes
```

## Why: the designs so far solve one problem five ways

Each of these designs draws its own line round the car, with its own numbers:

| Design | Its line | Its output |
|---|---|---|
| [One shape](2026-11-13_one-shape.md): lanes | a cost ramp in a mask | costs on the global costmap |
| [Behaviour and speed](2026-11-05_behaviour-and-speed.md) | a stop point ahead | a speed cap on Nav2 only |
| [How the car stops](2026-11-05_how-the-car-stops.md): the filter | room to an edge | a trimmed command |
| [Weak links](2026-11-04_weak-links.md) 4.5: the keep-in polygon | the site's edge | a keepout mask, and the fence |
| [Tracks and memory](2026-11-05_tracks-and-memory.md): the person rule | a box ahead of the car | a lock |
| The lane driver (`aav_lane_geometry`) | the painted lines | its own commands, as a rival driver |

They are all the same question: **where may the car be, how fast, and where must it stop?**
Answer it once, in one message, and each design becomes a writer or a reader of it.

## The road ahead

A strip of road in front of the car, about 40 m long, re-sent ten times a second.

```text
        left edge (hard: kerb, fence, wall)
   ══════════════════════════════════════════════════╗
   ─ ─ ─ ─ ─ ─ soft edge (the centre line) ─ ─ ─ ─ ─ ║  ◄ the strip ENDS:
   ● ─────── the line to follow (lane centre) ─────► ║     the car may not pass
   ══════════════════════════════════════════════════╝     until more is granted
   car     speed limit along the way: 1.0 … 1.0 … 0.6 … 0.3 … 0
                                                 ▲ a stop point, with its reason
```

| Part | Means | Comes from |
|---|---|---|
| The line to follow | the right lane's centre, along the route | routing + the lane map |
| Hard edges | never cross | kerbs, the fence, walls |
| Soft edges | cross only when the boss allows it | the centre line; the boss opens it to pass a parked car |
| Speed limit along it | from the one stopping law (`stop_law.py`) | bends, a thing ahead, how sure the pose is |
| Stop points | a place, a reason, "wait" or "final" | a stop sign's line, a person, the fence, the goal |
| **The end** | the strip always ends in a stop | the boss |

**The end is the safety idea.** Railways call it a *movement authority*: a train may run only
as far as it has been told the track is clear. The boss grants the next 40 m and keeps
extending it. If the boss dies, nothing new arrives, and the car stops by itself at the end
of what it was last given. No timeout to tune, and silence can never mean "carry on".

### Who writes it, who reads it

| Level | Today | With the road ahead |
|---|---|---|
| 1 Routing | waypoints to Nav2 | gives the line to follow |
| 3 Lane finder | a rival driver on the AI slot | **a sensor**: corrects the car's sideways position (below) |
| 5 Behaviour (the boss) | a speed cap only Nav2's controller reads | **writes the road ahead**; the only writer |
| 6 Path (lattice, diffusion) | plans on the costmap | plans inside the edges; used to go round things and in lots |
| 7 Follow (MPPI, MPC, RPP) | follows the path; some read costs | follows the line, inside the edges, under the speed limit |
| The point driver (Gate 4's driver) | plans its own stop; the stop zone doesn't guard it | reads the same strip: a straight line to B, the same stop points. One shape left it out |
| 8 Ego | limits | adds: never faster than the strip allows (the filter's first form) |
| 9 Geofence, stop sign, stop zone | each its own sum | unchanged, and underneath: they still stop the car alone |
| Screens | several pictures | draw the one strip: the same picture the car drives by |

### What it fixes from one shape's review

| Fable's finding on the lane mask | With the road ahead |
|---|---|
| "right" depends on the way you drive | the strip is made for the route in hand |
| inflation hides the lane costs | no costs to hide: the line is the lane's centre |
| MPPI follows the path, not the costs | the line **is** the path |
| the boss is mute under `stack:=ours` | every follower reads the same message |
| silence must not mean "carry on" | the strip ends |
| re-sending a 6400 x 6400 mask to change a rule | a short message, 10 Hz |

The cost ramp stays as a challenger (`path:=lattice` with the mask). The lane-centre line
was one shape's step 1b; this page makes it step 1.

### Built so far (14 Nov)

| Part | Where | Seen |
|---|---|---|
| The message | `aav_interfaces/RoadAhead`, `StopPoint`; topic `/world/road_ahead` | — |
| The maths, one copy | `aav_description` `road_ahead.py` (beside the stopping law): `build` for the writer, `limit_at` for readers | 14 tests |
| The writer | `aav_behaviour`: `/plan`'s next 40 m, its stops, 10 Hz | alone: 10 Hz, the end at the plan's end |
| The first reader | ego (`road_ahead:=true`, set by `behaviour:=true`): Nav2's forward speed never above the strip's; people never trimmed; no strip or a stale pose allows 0. Our own followers (`mppi_lag`, the trial trackers) go through Nav2's slot, so it holds them too: the "mute boss under all ours" finding is closed for speed | alone: Nav2 asking 0.8 m/s, 0.5 m from the end, got 0.31; the joystick 0.8 |
| A whole drive | `pixi run -e ml sim-check --behaviour --perception`, our simulator, 14 Nov | contract 0 failures; sign PASS (APPROACH, stood 4.3 m before it); goal PASS; ego trimmed Nav2 9 times (0.43-0.97 m/s). The obstacle leg FAILED as on 8 Nov: the car steered round the box (TODO 162) |

| The point driver (R5) | it sends its line A -> B (`/drivers/point/plan`); the behaviour planner makes the strip from it while it runs; ego watches `point` too | road world, `behaviour:=true`, 20 m ahead: stopped 0.22 m from B in 24.7 s; states DRIVE > APPROACH(goal) > DRIVE |

| One scorecard | `tools/score_drive.py` (sums in `tools/scorecard.py`): any bag from `record.sh`, sim or car, gets the same table | a 25 m point drive: 0.13 m mean off the plan, never past the end; found the sim car 14 % over its command (TODO 163) |
| The screens | the developer window draws it, 3D and map: coloured by the speed limit, a red bar at the end, each stop and why | seen live beside the road world, 14 Nov |
| The lane-centre line (W3, W5) | `aav_description` `lane_map.py` + a `lanes.yaml` (the road world's in `aav_gazebo`, the campus's from `pixi run campus`); `lane_map:=world`. Lanelet2 tried first: its Python modules don't import on a Mac | road world, 14 Nov: the line on the lane's centre (y = 0) while /plan swung -0.6 to 1.9 m; edges 1.75 m soft, 2.25 m hard, none unknown |

Still to do in step 4: `mppi_lag` and the other followers reading the line and its edges; the campus's lane line live.

## Better ideas, area by area

Each row merges parts we already have with a known method. All proposed.

### Localization and the wiggle

| Idea | Merges | Why it should help |
|---|---|---|
| **The lane finder corrects the pose** | `aav_lane_finder` + the lane map + the EKF | GPS is weakest sideways (1-3 m), and NDT slips *along* a road, not across. The camera measures the car's offset in the lane to centimetres (4 cm mean in our simulator, 8 Nov). Three sensors, each strong where the others are weak |
| **Follow in `odom`, correct slowly** | the trackers already do it (`aav_track`: map to odom once per path) | a GPS step turns the map by up to 4° (11 Oct) and swings the whole path. Limit how fast the correction is applied. 14 Nov: the sim's EKF jumps 0.09 m sideways every 0.2 s; with the truth as the pose Pure Pursuit flips half as often. **Built 15 Nov** (TODO 186): each follower's own map -> odom, slewed at 0.1 m/s and 1°/s at the car; without ROS `mppi_lag`'s sway 0.10 → 0.04 m/s² ([mppi-lag](../mppi-lag.md#follow-in-odom-correct-slowly)); the sim run next |
| **Keep the plan unless a new one is clearly better** | weak links 4.5; **built 14 Nov** as Nav2's own consistent-replanning tree: keep a valid path up to 10 s | measured: Pure Pursuit's swing 0.12-0.13 → 0.09 m, `mppi_lag`'s 0.14-0.17 → 0.10 m, no loops at the goal ([trials](../history/trials-log.md#the-wiggle-14-nov-todo-124-149)) |
| **Trust NDT only where it is sure** | research C (Zhang 2016) | NDT slipped 0.4 m along a wall in Gazebo |
| **Stops reset the drift** | research idea 12 | a parked car's speed is exactly zero: free information |

The three wiggle tests (TODO 149) ran on 14 Nov: the replanning and the GPS's jitter move Pure Pursuit and `mppi_lag`; the smoother nothing; MPPI's 6-8 cm swing is its own sampling. The plan is now kept; the slow map correction is built (TODO 186), its sim run next.

### Seeing: things, not just points

Today nothing near the car has a name, a speed or a memory. The parts exist; they aren't joined:
the radar's clusters, the lane lines (beyond the lane driver) and every label but "stop sign"
are read by nobody ([Perception: who reads what](../perception.md)).

```text
 lidar cubes ─► groups ─────────┐
 radar clusters (speed along    ├─► THE THINGS: one list
   the ray; no firmware swap) ──┤     id · kind · place · speed · how sure · last seen
 camera boxes (RF-DETR) ─► name ┘        │
   (needs camera-to-lidar calibration)   ├─► the boss (wait for a person; pass a parked car)
                                         ├─► the grid (where each will be in 3 s)
                                         └─► the screens
```

| Step | What | From |
|---|---|---|
| 1 | Camera-to-lidar calibration | research pick 4 (Koide's tool); unlocks every row below, and `camera_obstacles` |
| 2 | Group the cubes into things | plain geometry; `aav_voxels` already fits the road |
| 3 | A small tracker: one Kalman filter per thing, constant speed | AB3DMOT's finding (Weng 2020): the simple tracker is a strong baseline. A stock one first, if its licence fits |
| 4 | Name a thing by the camera box its points fall in | the frustum idea (Qi 2018), without the network |
| 5 | Radar speed per cluster, the car's own speed taken off | tracks-and-memory's "no flash" way |
| 6 | The blind-zone memory | a thing in the list is remembered by the list: no second memory to build |

Step 6 matters: the roof lidar can't see a 0.5 m box within 2.5 m of the bumper. A list of
things that keeps what it saw on the way in is tracks-and-memory's Part B, for free.

### Avoiding: steer round what stands, wait for what moves

| Thing | The car | Through |
|---|---|---|
| Stands still, in my lane (a parked car, a cone) | the boss opens the soft edge; the path goes round and comes back | the road ahead's edges |
| Moves, or is a person | slows and waits; never swerves round a person | a stop point, by the stopping law |
| Moves along my lane, ahead | follows at a gap | the speed limit along the strip. ✅ 15 Nov: already so. The cap is the stopping law's v_cap(gap − stop_gap), so behind a lead moving away at v the car settles at stop_gap + its stopping distance from v: a constant-safety-distance follow (it stops in time even if the lead stops dead). A time-gap controller only pays above our 1 m/s |
| Unknown | treated as a person | the boss's contract |

This is the path-and-speed split of Werling (2010) and Apollo's EM planner (Fan 2018): decide
the line and the speed apart, in the road's own coordinates (metres along, metres across).
MPPI gets one new critic: the things' places over the next 3 s, not only where they are now.

### Safety: one sum

| Idea | Merges |
|---|---|
| Every "can I stop in time?" is the same sum: room to the next stop point, less the stopping distance, less a margin | the geofence's path, the stop zone, the speed cap and the filter (knows-itself, how-the-car-stops) |
| The filter's first form is one line in ego: speed never above what the strip allows | no solver; a person on the joystick is never trimmed |
| The locks keep their own margins | Simplex (Sha 2001): the simple layer never depends on the clever one |

## Where the designs disagree

Found by reading all 35 designs. Nothing in the repo says which side wins. Each needs one word from Ben.

| # | Question | One side | The other | Proposed |
|---|---|---|---|---|
| R1 | Where does the stopping filter go? | "in ego" (TODO 104) | before twist_mux (how the car stops, knows itself; K1) | its first form in ego, speed only, and only for the automatic drivers' commands; the full filter before twist_mux |
| R2 | What does the boss's silence mean? | "no crossing, slow down" (one shape) | today a cap of 0.0 means no limit, and a dead boss removes the cap | the strip ends: stop |
| R3 | Which follower is "the choice"? | MPC (the control plan, 27 Oct) | MPPI (one car, one shape) | MPPI is the champion; MPC a challenger; the league decides |
| R4 | Gate 4: 1 m, or 2 m without RTK? | README, one car | `docs/tests.md` 5.2 | ask the team (G1); score against a tape mark |
| R5 | "Safety is last, for every driver" | one shape's picture | the stop zone holds only Nav2 and the AI slot, not Gate 4's point driver | the point driver reads the road ahead, so one rule covers it |
| R6 | The developer window | raylib (TODO 107) | Bevy (one window; Ben, 5 Nov) | Bevy: fix the TODO row |

Also still waiting on the team, by the code's own comment: `IDLE_BRAKE` is 0.0 in
`spine-core` ("Proposed ... Change only with the team"), so the brake lets go once the
e-stop clears. The 2025-26 firmware held it full on. The first review raised it on 25 Oct.

## Cheap things never tried

| Thing | Cost | Proposed in |
|---|---|---|
| `observation_persistence` above 0 on the lidar layers: the stock blind-zone memory | one line in `nav2.yaml`, then the low-box test | tracks and memory; the EGM review |
| Keep the last plan unless the new one is clearly better | a behaviour-tree edit | weak links 4.5; bears on the wiggle |
| Shadow mode's stage 1: more topics in `record.sh`, and a score script | about a week | shadow mode |
| A stop sign seen stays on the map | small, once the things exist | weak links 4.3 |
| The chain stamped end to end: "it appears" to "the brake moves" | a measurement | weak links 4.3, 4.6 |

## What is missing

Not ideas. The stack is **wide and thin**: about 60 packages, a challenger at nearly every
level, and the real car has so far driven with only the localization update installed (13 Nov;
Ben, 14 Nov: "Localization run on the car without issue").

| # | Missing | Evidence | It blocks |
|---|---|---|---|
| 1 | **The car lags the software by months.** The firmware timeout, the brake that follows a wish, wheel speed seen, clocks checked, steering centred | weak links, section 1: seven blockers, written 4 Nov | Gate 4 (26 Feb) |
| 2 | **No list of things**: no tracker anywhere in `ros/` | checked 14 Nov; `aav_track` is path followers | passing, waiting, people |
| 3 | **No recording has ever been scored**; `record.sh` saves no pictures, cubes, radar or locks | `tools/record.sh:4`; weak links 4.8 | the league; any learning; shadow mode |
| 4 | **No ground truth on the car** | no RTK (28 Oct); TODO 96 | every score from the car |
| 5 | **Calibration**: camera to lidar, steering centre, wheel size (17 % off loaded) | Q22; TODO 60, 67 | the things; lane correction; odometry |
| 6 | **The near blind zone** has no sensor of its own, and the radar is off on the car (`orin.yaml`: "later") | tracks-and-memory: 2.5 m for a 0.5 m box | the hardware lead's call |
| 7 | **Clocks** never checked at the car | `car_setup.sh` installs chrony; `car-day.md` had no step for it until this audit added one (14 Nov) | fusing anything across computers |
| 8 | **No safety case**: one page per hazard, what stops it, the number, who measured it | weak links 4.7 | the report; taking a passenger |
| 9 | **The Orin's load** is unmeasured for our Python nodes | TODO 76 | every "works on a Mac" |
| 10 | **Winter.** Gate 4 is in February, in Ottawa. Nothing in `ros/` or `tools/` knows about snow | research idea 2 | Gate 4's day |
| 11 | **Too much to read**: 100 pages, 28,000 lines, a 958-line TODO | this audit | the team |
| 12 | **Actuator faults on purpose**: steering stuck, brake weak, throttle stuck | weak links 4.7 | the safety case |

The honest summary: the next level is not a new algorithm. It is **the champion stack, driven
on the car, recorded, and scored**. Every idea above is worth more after that.

## The order

| # | Do | Why now |
|---|---|---|
| 1 | **The car's seven blockers** (weak links, section 1), with the hardware lead | nothing else counts until programs may drive |
| 2 | **Record everything, score something**: pictures, cubes, radar and locks in `record.sh`; one drive scored | every later step needs recordings |
| 3 | **The wiggle's three tests** (TODO 149), and the cheap things above | cheap; picks the localization fix |
| 4 | **The road ahead, in the sim**: the message; the boss writes it; the lane-centre line; Nav2's followers and ours read it | the merge itself. **Started 14 Nov**: the message, the writer, ego's limit on Nav2 (below) |
| 5 | **The lane finder as a pose sensor** | the best new idea for the money: all its parts exist |
| 6 | **The things**: calibration, groups, the tracker, names | passing and waiting need it |
| 7 | **The league** (`stacks.yaml`, `stack:=`) | one answer to "what is the car?" |
| 8 | **Shrink the docs and TODO** | so the team can read them |
| Later | the filter beyond speed; winter data; a learned driver in shadow | after Gate 4 |

A freeze, proposed: **no new challenger at any level until the champion stack has driven on
the car.** Every existing one stays (Ben, 5 Nov).

## Not doing

- A camera-only learned driver now. Alpamayo 2 Edge (released 11 Nov) is a driving planner,
  not a detector: six cameras in, a 6.4 s path out, tested only on a data-centre GPU, aimed
  at NVIDIA's Thor computer. We have two front cameras and an Orin. Later it could be a
  level-6 challenger, in shadow, like diffusion. RF-DETR stays the detector.
- Autoware as a whole (decided 28 Oct, 4 Nov).
- Changing the locks. The road ahead sits above them.
- Throwing out the lane mask, the lane driver or any algorithm: they stay as challengers.

## Decisions for Ben

| # | Question | Proposed |
|---|---|---|
| W1 | The road ahead as the one message between behaviour and the drivers | yes, in the sim first |
| W2 | Its name and place: a new area `/world/` (`/world/road_ahead`, `/world/things`) | yes; `docs/naming.md` gains the area |
| W3 | The lane-centre line first, the cost ramp as the challenger (swaps one shape's steps 1 and 1b) | yes |
| W4 | The lane finder becomes a pose sensor; the lane driver stays as a challenger | yes |
| W5 | The lane map's format: Lanelet2 (on RoboStack for Jazzy, 1.2.1, BSD-3-Clause; not yet tried in our `pixi.lock`) | try it; else grow `lanes.yaml` |
| W6 | The freeze on new challengers until the champion drives on the car | yes. **Ben, 15 Nov: no freeze** (`docs/decisions.md`) |
| W7 | The order above | yes |

One shape's D1-D7 stand, except D2 (W3 replaces it).

## Where each earlier design lands

| Design | Now |
|---|---|
| [One shape](2026-11-13_one-shape.md) | the frame: levels, league, switches. Its lane layer becomes the road ahead |
| [One car](2026-11-04_one-car.md), [weak links](2026-11-04_weak-links.md) | still the list of blockers and options; the order above replaces their schedule |
| [How the car stops](2026-11-05_how-the-car-stops.md), [knows itself](2026-11-05_knows-itself.md) | the one car model and the one sum |
| [Behaviour and speed](2026-11-05_behaviour-and-speed.md) | the boss; its output grows from a cap to the road ahead |
| [Tracks and memory](2026-11-05_tracks-and-memory.md) | the things; Part B's memory folds into the list |
| [Shadow mode](2026-11-05_shadow-mode.md) | part of the scorecard; stage 1 goes with step 2 |
| [Research ideas](2026-11-04_research-ideas.md) | the backlog; ideas 1, 3, 12 and C are placed above |

## Papers and sources

| Source | For |
|---|---|
| Werling, Ziegler, Kammel, Thrun, *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*, ICRA 2010 | planning along and across the road |
| Fan et al., *Baidu Apollo EM Motion Planner*, 2018 | the path-and-speed split |
| Poggenhans et al., *Lanelet2*, ITSC 2018 | the lane map |
| Weng et al., *AB3DMOT: 3D Multi-Object Tracking: A Baseline and New Evaluation Metrics*, IROS 2020 | the simple tracker |
| Qi et al., *Frustum PointNets*, CVPR 2018 | naming a group of points by its camera box |
| Zhang, Kaess, Singh, ICRA 2016; Sha 2001; Ames et al. 2017 | as in [weak links](2026-11-04_weak-links.md) |
| The railway's movement authority (ERTMS/ETCS) | the strip that ends |
| [Alpamayo 2 Edge's model card](https://huggingface.co/nvidia/Alpamayo2-Edge), read 14 Nov | "Not doing" |
| [RoboStack's `ros-jazzy-lanelet2`](https://anaconda.org/robostack-jazzy/ros-jazzy-lanelet2), read 14 Nov | W5 |

None of this has been run. Every "should help" is a claim to test on the scorecard.
