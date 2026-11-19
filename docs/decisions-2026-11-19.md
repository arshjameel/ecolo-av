# What we decided, 18-19 November, and why

A page to explain the last two days to someone else: what we chose, why, and what is
still open. Ben's own words are in `docs/history/messages-2026-11-18.md` and
`messages-2026-11-19.md`. The short version of each decision is also in
[Decisions](decisions.md#decided-on-18-19-november).

> **The short version**
> 1. **Keep every commit.** The team's merge queue squashed Ben's work; the fork now protects the full history.
> 2. **Test in phases**, from the install to the campus twin, each one automated.
> 3. **NDT leads, the GPS checks**: a failing GPS stops mattering where we have a map.
> 4. **Maps come from our own drives** (KISS-SLAM), then a person cleans them in a **map editor**.
> 5. **Scores are tracked, not gates**: no Nav2 yardstick; a change may score worse for a while.
> 6. **One ONNX file per model**: Core ML on the Mac to test, TensorRT on the Orin.
> 7. **The TODO list regrouped by topic**; a checklist for after the install.

## 1. Git: keep every commit

### What happened

| | |
|---|---|
| The team's merge queue | **squashes** each pull request into one commit (the "Protect main" rule, since 25 Oct) |
| So | Ben's 1,062 commits from 27 Oct to 18 Nov became a handful on the team's `main` |
| GitHub counted | 616 of Ben's commits; the real number was **1,853** |
| Why it matters | the history shows how the work was done: for the report, and for a job |

### What we decided

| Decision | Why |
|---|---|
| **Never reset, rebase or force-push** Ben's fork | the fork's `main` is the only place the full history lives |
| **Protection rules on the fork**: no force-push and no delete, on every branch and on the save tags | a mistake can't wipe it |
| **Save tags** (`history-<date>`, `save/<branch>`) after every merge | a fixed point to come back to |
| Merge with **real merges** (`git merge --no-ff`), never squash | every commit stays |
| After each team PR: `git merge upstream/main` | stays in step without losing anything |
| **Arsh switches the queue to merge commits** | then one PR brings the ~1,130 original commits into the team's repo |

## 2. CI: the Linux timeout fixed

| Problem | Fix |
|---|---|
| The Linux job timed out at 60 min: the Rust cache missed, so the build alone took 27 min | the build is its own step (45 min), then the tests get their full 60 |
| The cache was saved only when the whole job passed, so one failure meant a cold build next time | the cache is saved **right after the build**, whatever happens later |
| A failed build was hard to read | GitHub annotations from `build.log` |

## 3. The testing plan

Ben's prof wanted "a full automated testing system". The plan:
[The testing plan](testing-plan.md) (`aav-testing-plan.pdf`).

```text
 0 after the install ─▶ A stands ─▶ B hallway ─▶ C recording ─▶ D maps ─▶ E editor ─▶ F labels ─▶ G campus twin
```

| Phase | What | Built? |
|---|---|---|
| **0** | [After the install](after-install.md): every laptop, each car computer, the car together | ✅ the checklist |
| **A** | On stands, one command (`car-test --stands`): every sensor, steering, throttle, **faults on purpose**, losing the Orin too | ✅ passes in Ravensim |
| **B** | **The transfer functions** in the winter hallway: speed up, coast, brake, steer | ✅ in Ravensim |
| **C** | Record a drive: every raw sensor at full rate (`record-drive`) | ✅ (needs ~**87 GB an hour**: a big SSD) |
| **D** | Maps from those drives (`map-drive`, KISS-SLAM) | ✅ 2.3 cm off the truth in Ravensim |
| **E** | The map editor | ⬜ next |
| **F** | Labels: trees, buildings, parked cars; a better model after each drive | ⬜ |
| **G** | The campus twin in Ravensim, from our maps | ⬜ |
| beside A, B | **The debug screen** (the prof's idea): who is driving, what each sends, the gate and why (key D on the map) | ✅ |

### The transfer functions, and the IMU

The car is modelled as a **delay plus a lag** for speeding up, braking and steering. In the
hallway the car does each one; the measured run is laid over the model's, with error bars.
The **IMU's acceleration is blended with the wheel speed** (a small Kalman filter): the
wheels say how fast, the IMU says how quickly that changes, so braking and acceleration
forces (F = m·a) come out cleaner than from either alone.

## 4. Where am I? NDT leads, the GPS checks

### The words

| Word | Means |
|---|---|
| **ICP** (Iterative Closest Point) | line up two lidar scans: each point finds its nearest partner, shift, repeat |
| **NDT** (Normal Distributions Transform) | the map stored as small 3D cubes, each with a "blob" of its points; a scan is lined up with the blobs. Smaller and smoother than ICP's raw points |
| **KISS-ICP** | ICP scan after scan: "how far did I move", no map needed, drifts slowly |
| **KISS-SLAM** | KISS-ICP plus "I've been here before" (loop closure): **builds the map**, after the drive |
| **AMCL** | a particle filter on a 2D map: thousands of guesses, kept if they match the scan. Our backup |
| **EKF** | blends wheels, IMU and a position source into one smooth pose |

### The maps

| Map | Made from | Used by |
|---|---|---|
| **NDT map** (3D) | our drives, by KISS-SLAM | NDT: "where am I", to centimetres |
| **2D map** (from above) | the same drives | AMCL and the route planner |
| **Lane map** | drawn or imported (Lanelet2's format) | the lane pose; driving in lane |
| **Street map** | OpenStreetMap | routes ("drive to the library"); the map window |
| **Costmap** | live lidar, **never saved** | the local planner: avoiding cars and people |

### The decision: NDT leads (TODO 210)

The GPS was failing. Before, the GPS gave the position and NDT only helped. Now, with
`pose_source:=ndt`, **NDT gives the position and the GPS only checks it**. If they
disagree, NDT's match score says which is wrong; a lost NDT restarts from AMCL, else the
GPS; with neither, the car stops.

Ravensim's lot, 3 runs each, legs reached and how far the EKF was off:

| GPS | Today (GPS leads) | NDT leads |
|---|---|---|
| clean | 12/12, 0.10 m | 12/12, **0.01 m** |
| stops | held after 3/12 (safe) | **12/12** |
| jumps 20 m | held after 3/12 (safe) | **12/12** |
| drifts slowly | **3.6 m off**, not noticed | 12/12, 0.01 m |

A challenger for now: it needs a saved map of wherever the car drives.

### Learned localization: studied, not built

| Idea | What | Why not now |
|---|---|---|
| IR-MCL (PRBonn, 2023) | AMCL with a neural network as the scan model | no licence stated: the idea only. Our AMCL's scan model is swappable, so it can be tried later |
| Transformer MCL | a transformer helps the particle filter | needs training data from real maps first |
| OverlapNet (MIT), Scan Context | "have I been here before?" from a scan | for loop closure and getting un-lost; after real maps |

## 5. Maps from drives, and the map editor

```text
 drive + record ─▶ KISS-SLAM (laptop) ─▶ NDT map + 2D map + 64 m chunks ─▶ map editor ─▶ the car
```

| Decision | Why |
|---|---|
| **Build maps from our own drives**, placed by the good GPS fixes | the EKF's poses smeared the map (33 cm); SLAM's are 2.3 cm |
| KISS-SLAM runs **on a laptop**, after the drive | offline is enough; no Orin build exists |
| Several drives **merged**; keep what every drive saw | a parked car seen once doesn't become a wall |
| **A map editor** (TODO 251): our map over OpenStreetMap; align, delete parked cars, label, approve, save | a person always has the last word on a map |
| The costmap stays **live only** | it is what the car sees right now, never a saved map |

## 6. Planning: what we keep from Nav2 and Lanelet2

Nav2 was deleted on 17 Nov, but its best parts live on, **ported to Rust**:

| From | Became | Its advantage |
|---|---|---|
| Nav2's costmap | `aav_costmap` | sensors into "how risky is each spot", in layers |
| Smac Lattice | `aav_planner` (**champion**) | only moves the car can really make, reversing too |
| Smac Hybrid-A\* | `planner_search:=hybrid` (challenger) | quick in open space |
| MPPI | `aav_mppi_lag` (**champion follower**) | thousands of futures a tick; we added the car's lag |
| Collision monitor | `aav_stop_zone` | a last-chance stop box |
| AMCL | `aav_amcl_rs` | the backup position finder |
| Lanelet2's map format and routing idea | `lanelet-map`, `path:=lane_lanelet2` | plans by lanes, like a driver; maps from other tools open in ours |

### MPPI and diffusion are not rivals

| Level | Job | Champion | Challengers |
|---|---|---|---|
| **Path** | the line from here to the goal | the lattice | Hybrid-A\*, lane, **diffusion** |
| **Motion** | follow that line, 20 times a second | `mppi_lag` | `mpc_lag` |

"1000 MPPI samples" (TODO 152) is about **speed**: half today's 2000 futures, so the
follower runs faster on the Orin. The diffusion planner is still a path challenger; it
must be scored again with today's stack (last scored on 8 Nov in our own simulator, the
lot: 12 of 12 legs; skipped in the 16 Nov league for time).

## 7. Scores, not gates

| Decision | Why |
|---|---|
| **No Nav2 yardstick** (TODO 167 closed) | Nav2 is gone; comparing with its old numbers holds us back |
| A change **may score worse for a while** | Ben: "We can get worse if we think there is room to improve" |
| Scores are still **tracked and reported honestly** | so we know where we are |
| **Safety checks stay pass/fail** | `sim-check --faults`, the stop zone, the watchdog: never traded away |

## 8. Machine learning: one ONNX file

```text
 PyTorch (train) ─▶ model.onnx ─▶ ONNX Runtime (Rust) ─┬▶ Core ML  (Mac: test here)
                                                       └▶ TensorRT (Orin, JetPack 7.2)
```

| Decision | Why |
|---|---|
| **The Orin runs JetPack 7.2 or newer** (Ubuntu 24.04) | the same Ubuntu and ROS as everything else |
| **One `.onnx` per model**, run from Rust | one file for every computer; no separate Mac and Orin exports |
| **Test with Core ML on the Mac now** (moved up, TODO 243) | the export, the node and the answers are proven before the car |
| **CPU as the reference**; report any layer that falls back to the CPU | if Core ML runs every layer, TensorRT very likely can too |
| Timing and the first TensorRT build are measured **on the Orin** | speed is the one thing a Mac can't tell us |

Done so far: the stop-sign detector (RF-DETR) as ONNX from Rust, **8 ms** a picture on
Core ML against PyTorch's 14.7. Next: the diffusion planner and the lane finder.

## 9. Housekeeping

| Decision | Why |
|---|---|
| **The TODO list regrouped by topic**, one line a row; the old one kept word for word in `docs/history/todo-done.md` | it had drifted from the stack we run |
| **One helper at a time** (two at most), each on its own branch, merged with the full test before each push | keeps the cost down and `main` always green |
| **A page for every algorithm** (`stack-algorithms.md`, being written): where it came from, strengths, weaknesses, still used? | to explain the full stack |

## Still open: Ben's or the team's to decide

| Question | Today |
|---|---|
| The runaway rule trips on 0.05 m/s wheel noise | unchanged: a safety lock, a person decides |
| The indoor geofence with `stands:=true` in the hallway | the hallway tests use their own distance fence |
| Close PRs #28 and #29 | comments drafted; Ben closes them |
| A big SSD for recording (~87 GB/h) | the hardware lead |
| Arsh's merge-queue switch | then the history PR |
