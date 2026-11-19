# The league, scored: 16 Nov

Ben, 16 Nov: "lets test and score everything and fingure out the best options and maybe even
the best hybrid options. Check the internet for other options too", then "What is our final
configuration ... score them individually and then score them together".

Every option was scored in our simulator with **everything else at `stack:=ours`**, reversing
on, on the same tours. The rule ([the league](league.md#how-a-challenger-wins)): a challenger
wins by a margin on 3 or more runs, with no column worse.

## Final configuration

**No champion changed.** Today's `stack:=ours` stays the best whole car. One challenger is
close: **NDT into the global EKF** (`pose:=ndt_ekf`). It wins most columns, but not all.

| Level | Chosen | Why |
|---|---|---|
| pose | `ekf` (GPS) | `ndt_ekf` is calmer on every tour, but stops once on University Drive (2 of 3 runs) and backs up once at the box: **recommended next** |
| costmap | `ours_chunks` | the same drive as the whole map, a third of the memory; Nav2's went over the centre line |
| navigator | `ours` | Nav2's bt_navigator drove the same (a tie): ours stays, Nav2 goes |
| path | `lane` (+ pass) | the only one that keeps right: every other path is over the centre line 45-74 % of University Drive |
| steer | `mppi_lag_calm` | the calmest that reverses; Nav2 MPPI and plain `mppi_lag` fail or sway, RPP swings |
| smoother | `nav2` | ours a tie (no column better) |
| lanes | `off` | the ramp is a tie once `path:=lane` keeps right |
| behaviour | `on` | off: a little more sway on the road (0.074 against 0.063) |
| speed loop (the car's) | `default` | `smooth` a tie; `firm` sways twice as much; `lqi` slower |

### The head-to-head: ours against ours + `ndt_ekf`, 3 runs each, every tour

| Tour | Option | Legs | Time (s) | Sway (m/s²) | Sway changes /100 m | Off the path (m) | Steering flips /s | Stops a run |
|---|---|---|---|---|---|---|---|---|
| lot | ours | 12/12 | 159 | 0.022 | 5.1 | 0.15 | **0.22** | 0 |
| lot | + ndt_ekf | 12/12 | 157 | **0.019** | **3.9** | **0.10** | 0.30 | 0 |
| campus | ours | 9/9 | 282 | 0.087 | 6.6 | 0.17 | 0.18 | **1.0** |
| campus | + ndt_ekf | 9/9 | 283 | **0.083** | **6.0** | **0.13** | **0.14** | 1.3 |
| university | ours | 9/9 | 171 | 0.064 | 6.4 | 0.10 | 0.21 | **0** |
| university | + ndt_ekf | 9/9 | 170 | **0.046** | **1.0** | **0.03** | **0.06** | 0.7 |
| university_box | ours | 3/3 | **42** | 0.063 | 8.2 | 0.07 | 0.26 | 0 |
| university_box | + ndt_ekf | 3/3 | 48 | **0.053** | **4.3** | 0.07 | **0.14** | 0 (1 backed 3.4 m) |

On University Drive, `ndt_ekf` keeps the lane's centre twice as well (0.05 m off against
0.12). It loses on: the lot's steering flips, a stop on the last leg in 2 of 3 runs, one
reverse at the box (6 s slower). **Not a swap by the rule.** What it needs first (TODO 210):
find the stop, the box's reverse, and a fallback to `ekf` in worlds with no NDT map (the
launch refuses `ndt_ekf` there today, so it can't be `ours` as is). On the car it needs a
lidar map of the real lot.

## How it was run

| Item | Value |
|---|---|
| Tool | `pixi run measure controllers --stacks own_calm_chunks --reversing --launch behaviour:=true lane_route:=true <option>` (= `stack:=ours`) |
| Tours | `lot` (4 legs, 135 m), `university` (3 legs, 290 m of University Drive), `university_box` (a 1 m box on the lane), `campus` (finals only) |
| Runs | 2 each; the champion 3; the finals 3 on every tour |
| Simulator | ours, headless, a ROS domain 120-155 each, this computer only |
| Load | each job started under 8; the median while running 7.9 (other sessions were building) |
| Results | `/tmp/r5/<option>/results.json` and `results.md`; lanes by `tools/lane_score.py` |

How much is noise: on the lot `ours_chunks` and the whole map are the same thing (the lot has no
chunks), yet sway changes /100 m read 8.1 and 5.2. So only differences much larger than that
count.

## Each level alone

Columns as above. "Off centre" and "over the line" are the lanes scorecard on University Drive.

### Steer (the follower)

| Option | Lot legs | Lot sway | Lot off path (m) | Uni legs | Uni sway | Uni steering flips /s | Verdict |
|---|---|---|---|---|---|---|---|
| **mppi_lag_calm** (champion) | 12/12 | 0.022 | 0.16 | 9/9 | 0.063 | 0.20 | stays |
| mppi_lag (plain) | 7/8 | 0.027 | 0.20 | 6/6 | 0.144 | 0.21 | worse: the calm swap of 15 Nov holds |
| Nav2 MPPI | 8/8 | 0.019 | 0.32 (worst 5.2) | **3/6** | 0.068 | 0.29 | worse: stuck "the car doesn't fit at the start" |
| Nav2 RPP | 8/8 | 0.054 | 0.12 | 6/6 | 0.091 | 0.44 | worse: 42 sway changes /100 m on the lot |
| mpc_lag_rs | - | - | - | - | - | - | **can't reverse**: the launch refuses it |

Forward only (reversing off), 2 runs:

| Option | Lot sway | Lot off path (m) | Uni sway | Uni off path (m) | Uni steering flips /s | Tick (ms) |
|---|---|---|---|---|---|---|
| mppi_lag_calm | 0.022 | 0.14 | 0.068 | 0.10 | 0.20 | 3.4-5.3 |
| mpc_lag_rs | 0.047 | **0.07** | 0.160 | **0.07** | 0.70 | **0.7-0.9** |
| mpc_lag_rs + ndt_ekf | 0.043 | **0.04** | 0.133 | **0.01** | 0.24 | 0.7-0.9 |

`mpc_lag_rs` follows the path 2-7 times closer at a fifth of the time, but swings: its sway
is its own steering, not the GPS (NDT doesn't calm it). It needs a steering-rate cost and
reverse. The Python `mpc_lag` was skipped (the same commands as the Rust, to 1e-3).

### Path

| Option | Lot sway | Uni legs | Uni sway | Off centre (m) | Over the line | Verdict |
|---|---|---|---|---|---|---|
| **lane** (champion) | 0.022 | 9/9 | 0.063 | **0.10** | **0 %** | stays |
| lattice_rs_strong | (= lane on the lot) | 6/6 | 0.083 | 1.44 | 66 % | leaves the lane |
| lattice_rs | 0.025 | 6/6 | 0.093 | 1.43 | 58 % | leaves the lane |
| hybrid_rs | 0.020 | 6/6 | **0.035** | 1.51 | 74 % | calmest, wrong lane |
| Nav2 lattice | 0.021 | 6/6 | 0.100 | 1.35 | 45 % | leaves the lane |
| Nav2 Hybrid-A* | 0.021 | 6/6 | **0.033** | 1.43 | 59 % | calmest, wrong lane |

A parked box on the lane (`university_box`, 3 runs against 2):

| Option | Over the centre line | Back in lane |
|---|---|---|
| **lane** (pass) | 5-5.5 s, 12-13 m | 8.6-9.6 m after the box |
| lattice_rs_strong | 34-36 s, 82-84 m | never |

Found: the Hybrid-A* paths sway half as much on University Drive (steering flips 0.05-0.06 /s
against 0.20). The lane's corners are rounded with circle arcs (`lane.rs`
`round_corners`), whose curvature jumps; Hybrid-A* goes through the smoother. Worth a
challenger: the lane's corners as clothoids (TODO 211).

### Costmap, navigator, smoother, lanes, behaviour

| Level | Option | Lot sway | Uni sway | Uni off centre (m) | Uni over the line | Verdict |
|---|---|---|---|---|---|---|
| costmap | **ours_chunks** | 0.022 | 0.063 | 0.10 | 0 % | stays |
| costmap | ours (whole map) | 0.024 | 0.067 | 0.13 | 0.9 % | a tie, 3 times the memory |
| costmap | Nav2's | 0.024 | 0.062 | 0.31 | 6.5 % | worse: over the line, 12 s slower |
| navigator | Nav2's bt_navigator | 0.023 | 0.063 | 0.11 | 0 % | a tie (a stop a run on the road) |
| smoother | ours | 0.022 | 0.067 | 0.10 | 0 % | a tie |
| lanes | ramp | - | 0.066 | 0.10 | 0 % | a tie: `lane` already keeps right |
| behaviour | off | 0.022 | 0.074 | 0.13 | 0 % | a little worse |

### Pose

| Option | Lot sway | Lot off path (m) | Uni sway | Uni sway changes /100 m | Uni off centre (m) | Verdict |
|---|---|---|---|---|---|---|
| **ekf** (GPS) | 0.022 | 0.16 | 0.063 | 6.2 | 0.10 | stays |
| ndt_ekf | 0.021 | 0.12 | **0.042** | **0.8** | **0.05** | close: the head-to-head above |

`kiss_ekf` was skipped (it needs the `kiss` environment, and only feeds the local EKF).

### The car's speed loop (`speed_loop:=`)

| Option | Lot time (s) | Lot sway | Uni sway | Uni steering flips /s | Verdict |
|---|---|---|---|---|---|
| **default** | 157 | 0.022 | 0.063 | 0.20 | stays |
| smooth | 158 | 0.019 | 0.067 | 0.22 | a tie |
| lqi | 173 | 0.019 | 0.066 | 0.16 | slower |
| firm | 153 | 0.034 | 0.088 | 0.43 | sways |

#### The car on one loop, the models on another (18 Nov, TODO 163)

`speed_loop:=` is the car's loop; `speed_loop_model:=` is what every model of the car
assumes (ego's trace, `mppi_lag`, the behaviour planner). Ravensim on Ben's Mac, GPU,
headless, `own_calm`, reversing.

**The lot tour** (`measure controllers --stacks own_calm --tours lot --runs 3 --reversing`):

| Car / models | Runs | Legs | Left-right changes /100 m | Sway (m/s²) | Off the path, worst (m) | Time a tour (s) |
|---|---|---|---|---|---|---|
| **default / default** | 6 | 24/24 | 7.3, 7.5 | 0.021, 0.023 | 0.44-0.53 | 155-167 |
| firm / firm | 3 | 12/12 | **20.8** | **0.034** | 0.49-0.75 | 153-154 |
| firm / smooth | 6 | 24/24 | 9.6, 8.6 | 0.024, 0.023 | 0.46-**1.05** | 159-169 |

Two numbers in a cell: two batches of 3 runs. Load average 4-11 (other sessions).

**The 25 m stop** (`drive-to-point --sim --ahead 25`, recorded, `tools/score_drive.py`):

| Car / models | Runs | Top speed (m/s) | Over the road ahead's limit (s) | Stopped from B (m) |
|---|---|---|---|---|
| default / default | 2 | 1.14 | 6.7, 6.7 | 0.13, 0.13 |
| firm / firm | 2 | 1.00 | 0.2, 0.2 | 0.06, 0.08 |
| **firm / smooth** | 2 | 1.00 | **0.0, 0.0** | 0.07, 0.08 |

**Verdict: firm / smooth stays a challenger.** It stops as well as firm, and sways far
less than firm / firm. But on the tour it is still a little worse than default: 23 % more
left-right changes, and 3 of 6 runs went 0.8-1.05 m off the path (default: never past
0.53). The worst moments are on the legs that back up ("turn back", "home").

## Hybrids

| Hybrid | Lot sway | Uni sway | Uni off path (m) | Uni over the line | Verdict |
|---|---|---|---|---|---|
| lane + Hybrid-A* search (`path:=lane planner_search:=hybrid`) | 0.024 | 0.055 | 0.18 (worst 2.1) | 6 % | worse: leaves the lane at the joins |
| ndt_ekf + the newest map correction | 0.021 | 0.058 | 0.07 | 0 % | worse than ndt_ekf alone (0.042) |
| ndt_ekf + mpc_lag_rs (forward) | 0.043 | 0.133 | **0.01** | 0 % | still swings; one lot run's legs all refused (planner error 200) |
| **ours + ndt_ekf** | 0.019 | 0.046 | 0.03 | 0 % | the best combination: [the head-to-head](#the-head-to-head-ours-against-ours-ndt_ekf-3-runs-each-every-tour) |

## Nav2's last yardstick

Ben decided on 16 Nov to delete Nav2 after the stop zone's shadow swap. These are its last
scores, `--stacks nav2` (every level Nav2's: bt_navigator, its lattice, its costmaps, MPPI;
no road ahead), reversing on, 2 runs each, beside `ours`:

| Tour | Stack | Legs | Time (s) | Sway | Sway changes /100 m | Off the path (m), mean / worst | Off centre (m) |
|---|---|---|---|---|---|---|---|
| lot | Nav2 | 8/8 | 183 | 0.020 | 14.1 | 0.27 / 3.7 | - |
| lot | ours | 12/12 | 159 | 0.022 | 5.1 | 0.15 / 0.7 | - |
| campus | Nav2 | **4/6** | 374 | 0.093 | 11.1 | 0.83 / 7.0 | - |
| campus | ours | 9/9 | 282 | 0.087 | 6.6 | 0.17 / 1.7 | - |
| university | Nav2 | **2/6** | - | 0.072 | 5.1 | 0.18 / 0.5 | 0.77 (13 % over the line) |
| university | ours | 9/9 | 171 | 0.064 | 6.4 | 0.10 / 1.2 | 0.13 (0 %) |

Nav2 gave up (status 6) on "down University Drive" in both runs and "along the road" on the
campus once. Its pieces alone inside ours are in the level tables above (MPPI, the lattice,
Hybrid-A*, the costmaps, bt_navigator).

## The survey: open source we haven't tried

Licences read from each project's GitHub record on 16 Nov. Our rule: learn from it, write it
ourselves in Rust or Python; none installed.

### Follower

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Smooth MPPI / KMPPI** ([pytorch_mppi](https://github.com/UM-ARM-Lab/pytorch_mppi)) | MIT, Python | samples the steering's *change* (or a few knots), so the commands come out smooth | port into `mppi_lag`: our sway |
| **Autoware's MPC lateral controller** ([autoware_universe](https://github.com/autowarefoundation/autoware_universe)) | Apache-2.0, C++ | steering lag and dead time in the model, a steering-rate cost, reverse | port into `mpc_lag_rs`: what it lacks above |
| **Vector Pursuit** ([vector_pursuit_controller](https://github.com/BlackCoffeeRobotics/vector_pursuit_controller)) | Apache-2.0, C++ | pure pursuit that also aims the heading; reverses | a small yardstick beside RPP |

### Path

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Piecewise-jerk path** (Apollo's EM planner, [apollo](https://github.com/ApolloAuto/apollo)) | Apache-2.0, C++ | a QP along the lane: keeps the lane, bends round a parked thing under a jerk limit | port into `lane.rs` with Clarabel: smooth corners and passes |
| **Autoware static obstacle avoidance** (behavior_path_planner) | Apache-2.0, C++ | the pass's length from the speed and a lateral-jerk limit (0.3-2 m/s³) | port the formula into `pass.rs` |
| **Autoware path optimizer** (elastic band + MPT) | Apache-2.0, C++ | the path made drivable (bicycle model) before the follower sees it | idea for `plan_smoothing` |

### Costmap and ground

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Patchwork++** ([patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus)) | BSD-2-Clause, C++ | finds the ground ring by ring, so a 0.3 m kerb stands out | port: the kerbs we miss (TODO 119) |
| **grid_map** ([grid_map](https://github.com/ANYbotics/grid_map)) | BSD-3-Clause, C++ | many layers on one grid | idea for our costmap's layers |
| **nvblox** ([nvblox](https://github.com/nvidia-isaac/nvblox)) | Apache-2.0, C++/CUDA | a distance field on the GPU (the Orin has one) | a yardstick only: a big stack |

### Lanes and routing

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Lanelet2's routing graph** ([Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)) | BSD-3-Clause, C++ | lane changes as edges in the route | add to `lane.rs`'s Dijkstra (we read Lanelet2 in Rust) |
| **CommonRoad route planner** ([commonroad-route-planner](https://github.com/CommonRoad/commonroad-route-planner)) | BSD-3-Clause, Python | the reference path and where to change lane | idea for `lane.rs` |
| Maliput (a Rust API over C++) | not checked | OpenDRIVE roads with routing | only if we take OpenDRIVE maps |

### Things (what moves)

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Fast-Poly / Poly-MOT** ([FastPoly](https://github.com/lixiaoyu2000/FastPoly)) | MIT, Python | a motion model per kind (bicycle for a car, constant speed for a person) | port into `aav_things` (one constant-velocity filter today) |
| **OC-SORT** ([OC_SORT](https://github.com/noahcao/OC_SORT)) | MIT, Python | a thing lost then found re-fits its speed from its last sightings | port into our tracker |
| **MapMOS** ([MapMOS](https://github.com/PRBonn/MapMOS)) | MIT, Python (a network) | moving points against the map | idea for `things_background`, later |

### Where the car is (lidar)

| Option | Licence, language | What it does better | Fit |
|---|---|---|---|
| **Kinematic-ICP** ([kinematic-icp](https://github.com/PRBonn/kinematic-icp)) | MIT, C++ | KISS-ICP with the wheels and a car that can't slide sideways | the next `kiss_ekf` challenger, ported |
| **small_gicp** ([small_gicp](https://github.com/koide3/small_gicp)) | MIT, C++ | fast GICP on many cores | a yardstick for NDT |
| **GenZ-ICP** ([genz-icp](https://github.com/cocel-postech/genz-icp)) | MIT, C++ | mixes plane and point matching by the scene: long straight roads | idea for NDT on University Drive |

## Skipped

| What | Why |
|---|---|
| aav_track's trackers (pure_pursuit, stanley, lqr, mpc) | forward only: reversing is an rc1 must-have |
| `mpc_lag` (Python) | the Rust gives the same commands; scored the Rust |
| `diffusion` | a trial for the lot, not the road; skipped for time |
| `kiss_ekf` | needs the `kiss` environment; the local EKF only |
| ego and behaviour in Python, `things:=true` | drop-ins of the same behaviour, not choices of how to drive |
| the campus tour for each level | time: the finals ran it, 3 runs each |
