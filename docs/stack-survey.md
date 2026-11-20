# The self-driving stack: a survey, and what we found

Ben, 19 Nov: "a document that is like a survey of the self driving car space and what we
found testing each algo". One page to explain the full stack: each algorithm we built or
studied, where it came from, what it adds, what testing showed, and which ones we still use.

> **The short version**
>
> - Self-driving cars **sense, localize, perceive, predict, plan and control**. Most open
>   stacks (Autoware, Apollo) build each step as its own part. Some companies now train one
>   network for the whole job (end-to-end).
> - **We are modular**, like Autoware: each part can be tested alone, scored the same way,
>   and swapped (Ben, 5 Nov: "just swap out and test").
> - **Today's defaults** (`stack:=ours`, the simulator's default):
>   - the EKF on GPS, helped by the lane finder;
>   - RF-DETR for objects, the lidar's cubes for kerbs and things;
>   - our costmap in 64 m chunks; the lane's centre as the path; MPPI that knows the lag;
>   - the behaviour planner, ego, and the safety locks that are never swapped.
> - **Next to swap in:** NDT leading, with the GPS only checking it (TODO 210).
> - **Almost nothing here has run on the real car yet.** The numbers come from Ravensim
>   (our simulator), Gazebo (until 16 Nov), bench mode and unit tests.

## How to read this page

Every algorithm has the same card:

| Field | Means |
|---|---|
| **What it is** | in a sentence or two |
| **Main goal** | the one job it does |
| **What it adds** | why our project has it |
| **From** | the paper or project, and its licence ([Acknowledgements](acknowledgements.md)) |
| **What we found** | real numbers from the repo, with where and when; or "not tested yet" |
| **Strengths**, **Weaknesses** | short |
| **Status** | one of the words below |

| Status | Means |
|---|---|
| **Default** | runs in `stack:=ours`, the simulator's default: the league's best whole car (16 Nov) |
| **Car champion** | `stacks.yaml`'s champion: what `car.launch.py` runs today. Named only where it differs from the default |
| **In use** | always runs; not a league level (ego, the EKFs, the safety locks) |
| **Challenger** | built, scored beside the default; swaps in only when it wins ([the league](league.md)) |
| **Retired** | no longer runs; why is in the card |
| **Studied** | read, not built |

The numbers come from these pages: [the league's scores](league-results-2026-11-16.md),
[the trials](history/trials-log.md), [Features](features.md), and the area pages linked in
each section.

## 1. The self-driving field in one page

### The usual pipeline

```text
  sense ──▶ localize ──▶ perceive ──▶ predict ──▶ plan ──▶ control ──▶ the car
  cameras,   where am I?   what is      where will   which way?  steering,
  lidar,                   round me?    it go?                  throttle,
  radar, GPS,                                                   brake
  IMU, wheels
```

| Step | The question | Usual tools |
|---|---|---|
| Sense | what do the sensors say? | drivers for cameras, lidar, radar, GPS, IMU, wheel speed |
| Localize | where am I, to the centimetre? | Kalman filters, lidar matching to a map (NDT, ICP), particle filters |
| Perceive | what is around me? | detectors (neural networks), lidar clustering, lane finding, trackers |
| Predict | where will each thing go? | constant speed at first; learned predictors later |
| Plan | which way, and how fast? | routes on a road graph, path search, sampling or optimising controllers |
| Control | how do I turn that into pedals and steering? | PID loops, MPC, a model of the car |

### Two schools

| | Modular | End-to-end (learned) |
|---|---|---|
| The idea | one part per step, each with its own inputs and outputs | one large network from pictures to steering |
| Who | Autoware, Apollo, most research cars, **us** | Tesla FSD v12 and later, Wayve; research networks such as UniAD |
| Strengths | each part is tested and explained alone; safety rules are written down | learns what is hard to write as rules; one model to improve |
| Weaknesses | parts can disagree; hand-tuned rules miss rare cases | needs huge amounts of driving data and GPUs; hard to explain or test part by part |
| For us | fits a student team: one person can own and prove one part | Ben, 4 Nov: "we can't use end to end like tesla because we are a school project" |

### The big open projects

| Project | Who, licence | What it is for | What we took |
|---|---|---|---|
| **Autoware** | the Autoware Foundation, Apache-2.0 | a full open stack for road cars, on ROS 2 | ideas only: the map levels, the lane-centre path, its MPC design, its smooth stop, its camera-lidar matching, YabLoc. Set aside as a whole on 4 Nov: too big, ROS 2 Humble, C++, needs an HD map |
| **Baidu Apollo** | Baidu, Apache-2.0 | a full open stack for road cars, not on ROS | ideas only: routing on a lane graph, map tiles, the piecewise-jerk path (studied) |
| **Nav2** | S. Macenski and the Nav2 authors, Apache-2.0 | navigation for mobile robots, on ROS 2 | ported to Rust: the costmap, the lattice and Hybrid-A*, MPPI, the stop zone, AMCL. Nav2 itself ran until 17 Nov ([as it was](history/nav2-as-it-was.md)) |
| **openpilot** | comma.ai, MIT | driver assistance on production cars, with a learned driving model | nothing: set aside on 4 Nov (highways, its own hardware) |

### Where we sit, and why

| Choice | Why |
|---|---|
| **Modular**, each part a drop-in | every part is scored on the same routes, and the winner swaps in ([the league](league.md)) |
| **A small car at low speed** | ego's top speed is 1.0 m/s to start; the campus tours run at 2.5 m/s. Simple models are enough at that speed |
| **Our own code, mostly Rust** | Ben, 14 Nov: "We want to run everythign ourselves in rust and/or python"; 16 Nov: "I trust rust more then python" |
| **Ideas from the big projects, not their code** | we learn their designs and credit them ([Acknowledgements](acknowledgements.md)) |
| **Learned parts only behind a guard** | the diffusion planner's paths pass eight checks, or the lattice's path is used |

## 2. Our stack in one picture

The **default** in each box (`stack:=ours`). Challengers in brackets.

```text
 SENSORS     wheels (CAN) · IMU · GPS · roof lidar (VLP-16) · radar · front cameras
    │
    ▼
 LOCALIZATION   ekf_local (wheels + IMU) ─▶ ekf_global (+ GPS, + lane pose)
                [NDT leads · NDT into the EKF · KISS-ICP · AMCL]
    │
    ▼
 PERCEPTION     RF-DETR (objects, stop signs) · lane finder · lidar flat scan
                kerb step finder · radar clusters · things tracker
                [ONNX from Rust · YOLO · lidar cubes for the costmap]
    │
    ▼
 WORLD MODEL    costmaps from 64 m map chunks · the road ahead · the things
                [lanes as soft costs]
    │
    ▼
 PLANNING       route: Valhalla / campus road graph ─▶ path: the lane's centre
                (the lattice off the lane map) ─▶ motion: mppi_lag, calm
                [Hybrid-A* · smooth moves · Lanelet2 routing · diffusion · mpc_lag]
    │
    ▼
 BEHAVIOUR      the behaviour planner: slow, stop, wait, go (the stopping law)
    │
    ▼
 EGO / CONTROL  twist_mux ─▶ ego (bicycle model, limits, timeout) ─▶ speed loop (PI)
    │
    ▼
 SAFETY LOCKS   e-stop · watchdog · stop zone · geofence · stop sign · output gate
    │           (never swapped)
    ▼
 THE CAR        ESP32 actuators today (UART); the CAN spine later
```

How each box talks to the next: [How it fits](how-it-fits.md).

## 3. Localization: where am I?

The full story: [Maps and position](maps-and-position.md), [Learn: localization](learn-localization.md),
[Maps from drives](maps-from-drives.md), [the trials](history/trials-log.md).

```text
 wheels + IMU ─────────────▶ ekf_local  ─▶ odom → base_link   (smooth, drifts)
 wheels + IMU + GPS ───────▶ ekf_global ─▶ map → odom         (true, may hop)
            + lane pose (default), + NDT (challenger)
```

### The two EKFs (robot_localization)

| | |
|---|---|
| **What it is** | two extended Kalman filters. `ekf_local` blends wheels and IMU; `ekf_global` adds the GPS |
| **Main goal** | one smooth pose to steer by, and one true pose to plan by |
| **What it adds** | every other part reads its pose; it carries on when one sensor stops |
| **From** | T. Moore, D. Stouch, IAS-13, 2014 (robot_localization, Apache-2.0); R. E. Kalman, 1960 |
| **What we found** | Gazebo's lot: `ekf_global` 0.17 m off the truth (mean), 0.58 m worst. Ravensim's lot (19 Nov): 0.10 m mean, 0.29 m worst. GPS stopped (Gazebo): the EKF, on wheels and gyro alone, drifted to 1.2 m by the tour's end |
| **Strengths** | small and proven; reads any sensor with a covariance; refuses a reading 5 sigma off |
| **Weaknesses** | only as true as the GPS; a slow GPS drift goes unnoticed (below) |
| **Status** | **In use**. Kept stock: small and proven (`CLAUDE.md`) |

### GPS (u-blox, `navsat_transform`)

| | |
|---|---|
| **What it is** | latitude and longitude turned into map x and y, fed to `ekf_global` |
| **Main goal** | pin the car to the earth |
| **What it adds** | a position anywhere outdoors, with no map |
| **From** | the stock `ublox_gps` driver; `navsat_transform` in robot_localization |
| **What we found** | Ravensim's lot, 19 Nov, GPS faults on purpose: stopped or jumped 20 m, the car **held safely** after 3 of 12 legs; drifting 5 m in a minute, it drove every leg **3.6 m off** (5.2 m worst) and nothing noticed. Ben, 18 Nov: "The gps is seeming to fail" |
| **Strengths** | no map needed; works anywhere with sky |
| **Weaknesses** | tens of centimetres of jitter; slow drift is invisible to the EKF |
| **Status** | **Default** (`pose:=ekf`). Becomes the checker, not the source, once NDT leads |

### NDT on a saved lidar map (ours, Rust: `aav_ndt`)

| | |
|---|---|
| **What it is** | the map stored as small 3D cubes, each with a "blob" of its points; each lidar scan is lined up with the blobs |
| **Main goal** | the car's pose to centimetres, with no RTK GPS |
| **What it adds** | a position that doesn't drift and doesn't need the GPS |
| **From** | P. Biber, W. Straßer, IROS 2003; M. Magnusson, PhD thesis, 2009 (papers). Autoware uses NDT the same way. Our own code |
| **What we found** | Gazebo's lot, 29 Oct: 0.9-3.0 cm off a leg (mean), 2 ms a match on an M4 Max. On the campus, 64 m chunks: ready in 0.02 s, not 2.5 s, just as close |
| **Strengths** | centimetres; fast; tells us how good each match is (its score) |
| **Weaknesses** | needs a map of every place it drives. On a lane with only parallel walls it slipped 0.25-0.4 m along the lane (about 2 % of fits) |
| **Status** | the matcher behind the next two cards |

### NDT into the global EKF (`pose:=ndt_ekf`)

| | |
|---|---|
| **What it is** | NDT's pose fed to `ekf_global` beside the GPS |
| **Main goal** | keep the planning pose to centimetres, and hold it when the GPS drops |
| **What it adds** | a calmer, truer pose for every planner and follower |
| **From** | our trial (30 Oct); NDT's covariance fitted to its errors (NEES) |
| **What we found** | Gazebo, 3 runs each: the EKF's error **17 cm → 1.6 cm**. GPS off: 1.2 m drift without NDT; 1-3 cm with it. League, 16 Nov: University Drive sway 0.046 against 0.064 m/s², off the path 0.03 against 0.10 m. But a stop on the last leg in 2 of 3 runs, and one reverse at the box |
| **Strengths** | calmer on every tour; the best combination in the league |
| **Weaknesses** | not better on every column; needs an NDT map; the GPS still leads, so a drift still pulls |
| **Status** | **Challenger**: "close", not a swap by the rule ([league](league-results-2026-11-16.md#final-configuration)) |

### NDT leads, the GPS checks (`pose_source:=ndt`, `aav_pose_check`)

| | |
|---|---|
| **What it is** | `ekf_global` takes NDT's pose only; the GPS is compared, never used, while NDT is trusted. If they disagree, NDT's score says which is wrong. A lost NDT restarts from AMCL, else the GPS; with neither, the car stops |
| **Main goal** | a failing GPS stops mattering where we have a map |
| **What it adds** | the answer to Ben's 18 Nov problem: the GPS failing |
| **From** | our design (19 Nov, TODO 210) |
| **What we found** | Ravensim's lot, 3 runs each, every GPS fault: **12 of 12 legs**, the EKF **0.01 m** off (mean). Clean GPS today: 0.10 m. Drift: 3.6 m today, 0.01 m with NDT leading |
| **Strengths** | drives through a GPS that stops, jumps or drifts; names the faulty one |
| **Weaknesses** | needs a lidar map of each place; `car.launch.py` refuses it until the real lot has one |
| **Status** | **Challenger**, being merged (TODO 210). Untested on the car |

### AMCL: stock (`nav2_amcl`) and ours (`aav_amcl_rs`)

| | |
|---|---|
| **What it is** | a particle filter on a 2D map: thousands of guesses of the pose, kept if they match the scan |
| **Main goal** | find the car on a flat map, and find it again when lost |
| **What it adds** | NDT's recovery when NDT leads; a second opinion beside the EKF |
| **From** | Nav2's `nav2_amcl` (from Player's amcl: A. Howard, B. Gerkey, K. Stoy), LGPL-2.1-or-later. D. Fox et al., AAAI 1999; *Probabilistic Robotics*, 2005. Ours is a Rust port, same licence |
| **What we found** | 29 Oct, default settings: lost the car (21 m worst). Tuned, with its own 0.05 m map: 0.06-0.11 m. 18 Nov, Ravensim, 3 runs: ours 0.05-0.09 m a leg, stock 0.05-0.10 m: **a tie**. One earlier run of ours started badly (0.94 m worst) |
| **Strengths** | needs only a 2D map; can find the car from far off |
| **Weaknesses** | lost to NDT on 29 Oct; a wall-less open lot gives it little to match; tuning matters a lot |
| **Status** | **Challenger** (`amcl:=true`, off by default). Ours only: stock `nav2_amcl` left on 20 Nov (TODO 232, tag `save/nav2-amcl`) |

### KISS-ICP (lidar odometry)

| | |
|---|---|
| **What it is** | lines up each lidar scan with the last few: "how far did I move", with no map |
| **Main goal** | a third measure of motion beside the wheels and the IMU |
| **What it adds** | catches slipping wheels; its twist (never its pose) can feed `ekf_local` (`pose:=kiss_ekf`) |
| **From** | I. Vizzo et al., RA-L 2023 (PRBonn), MIT. We run it as it is |
| **What we found** | Gazebo's lot, 30 Oct: 0.04 m drift per 100 m, against the wheels and IMU's 0.33; about 1 ms a scan. Into `ekf_local`: **no clear win** (0.30-1.27 against 0.44-1.91 m per 100 m; the Mac's load moved it more) |
| **Strengths** | cheap; no map; little drift |
| **Weaknesses** | Gazebo flatters it (perfect wheels, walls everywhere); needs its own Python environment |
| **Status** | **Challenger** (`kiss_ekf`); skipped in the 16 Nov league |

### KISS-SLAM: maps from drives (`pixi run map-drive`)

| | |
|---|---|
| **What it is** | KISS-ICP plus loop closures and a pose graph: it places every scan of a recorded drive, then builds the map |
| **Main goal** | NDT's 3D map and a 2D map from our own drives |
| **What it adds** | maps of real places, not guessed ones; drives can be merged |
| **From** | T. Guadagnino et al., IROS 2025 (PRBonn), MIT. Its wheel holds g2o (BSD) and CHOLMOD (LGPL-2.1+): an offline tool, never on the car |
| **What we found** | Ravensim's lot, two drives, 19 Nov: the map **2.3 cm** off the truth (median), against 27-33 cm from the EKF's poses. NDT on that map: 3.5-4.7 cm a leg. No loop closure found on the plain lot |
| **Strengths** | maps about ten times sharper; runs on a laptop with no ROS |
| **Weaknesses** | no build for the Orin; loop closures need features; untested on a real drive |
| **Status** | **In use** as an offline tool (testing plan phase D). Untested on the car |

### The lane pose (`aav_lane_pose`, `pose:=ekf_lane`)

| | |
|---|---|
| **What it is** | the lane finder's painted lines plus the lane map give the car's place across the lane; `ekf_global` takes that sideways only |
| **Main goal** | take the GPS's jitter out of the pose, across the road |
| **What it adds** | the car keeps the lane's centre better |
| **From** | ideas: Autoware's YabLoc (Apache-2.0); LaneLoc (D. Schreiber et al., IEEE IV 2013). Much simpler: one offset, no particle filter |
| **What we found** | Ravensim, 17 Nov: University Drive sideways error 9.2-9.9 → 6.9-7.8 cm RMS; its box tour 7.1-8.2 → 1.9-2.1 cm; the campus tour no change (little paint) |
| **Strengths** | cheap; a camera we already have; gated so bad lines never reach the filter |
| **Weaknesses** | sideways only; needs paint and a lane map |
| **Status** | **Default** in `stack:=ours` since 17 Nov. Car champion: `ekf` |

### Studied only: localization

| Idea | What it is | Licence | Why not yet |
|---|---|---|---|
| **IR-MCL** (PRBonn, 2023) | AMCL with a neural network as its scan model | no licence stated: the idea only | our AMCL's scan model is a swappable part, so it can be tried later |
| **Transformer MCL** | a transformer helps the particle filter | not checked | needs training data from real maps first |
| **OverlapNet** | "have I been here before?" from a scan | MIT | for loop closure and getting un-lost; after real maps |
| **Scan Context** | a place's fingerprint from one scan | not checked | the same |
| **Kinematic-ICP** (PRBonn, 2025) | KISS-ICP that knows the car can't slide sideways, with the wheels | MIT | the next `kiss_ekf` challenger (TODO 216) |
| **small_gicp**; **GenZ-ICP** | fast GICP on many cores; ICP that mixes planes and points on long roads | MIT | yardsticks for NDT |

## 4. Perception: what is around me?

The full story: [Perception](perception.md), [Model comparison](model-comparison.md),
[ONNX](onnx.md), [Kerbs](kerbs.md).

### RF-DETR Nano (the object and stop-sign detector)

| | |
|---|---|
| **What it is** | a transformer-based object detector: boxes and labels for people, cars, stop signs |
| **Main goal** | see stop signs and people in the front cameras |
| **What it adds** | the stop-sign lock and the things' names depend on it |
| **From** | Roboflow, Apache-2.0 |
| **What we found** | 16 Oct, Ben's Mac: Gazebo's stop sign seen from **20 m** (YOLO11n: 10 m); people in 4 of 5 photos (YOLO11n: 3); no false finds. 16-17 ms a picture on the Mac's GPU. Ravensim: the car stood still 4.3 m before the sign |
| **Strengths** | sees further; a licence with no strings |
| **Weaknesses** | 12 times YOLO11n's weights (30 M); 2.5 times slower on a CPU alone; untested on the Orin and the car's cameras |
| **Status** | **Default** (`detector:=rfdetr`) since 23 Oct (Ben: "I thought we were skipping yolo") |

### YOLO (Ultralytics)

| | |
|---|---|
| **What it is** | a fast one-stage detector; the 2024-25 team's first stop-sign detector was YOLO |
| **Main goal** | the same as RF-DETR |
| **What it adds** | a fast fallback |
| **From** | Ultralytics, **AGPL-3.0** |
| **What we found** | YOLO11n: sign from 10 m, 3 of 5 people, 12 ms on the Mac's GPU. YOLO26n scored the same as YOLO11n, picture for picture |
| **Strengths** | small (2.6 M weights); fast |
| **Weaknesses** | sees signs later; AGPL-3.0 |
| **Status** | **Challenger** (`detector:=yolo`). YOLO26: **studied**, brings nothing here |

### ONNX from Rust (`aav_detector_rs`)

| | |
|---|---|
| **What it is** | the same RF-DETR exported once to ONNX, run from Rust with ONNX Runtime: Core ML on a Mac, TensorRT on the Orin |
| **Main goal** | one model file for every computer, in a Rust node |
| **What it adds** | speed, and no 2 GB Python environment on the car |
| **From** | ONNX Runtime (Microsoft, MIT), the `ort` crate (MIT or Apache-2.0) |
| **What we found** | 17 Nov: the same boxes as PyTorch on 7 pictures (IoU 0.9999); **8.0 ms** a picture on Core ML, against PyTorch's 14.7. Ravensim's stop sign: passes |
| **Strengths** | faster; one file; Rust |
| **Weaknesses** | TensorRT compiles but has never run; Core ML's first start takes about 30 s |
| **Status** | **Challenger** (`detector_impl:=rust`); the plan for every model (TODO 243) |

### The flat lidar scan (`lidar_scan`, Rust)

| | |
|---|---|
| **What it is** | the 3D lidar cut into a flat 2D scan, ignoring anything under 0.3 m and the car's own body |
| **Main goal** | the costmap's and the stop zone's "something is there" |
| **What it adds** | the simplest obstacle input |
| **From** | ours; the Rust twin of our Python node |
| **What we found** | stopped 0.36 m before a person-sized post (Gazebo). The Rust: the same ranges as the Python, 2.0 % of a core against 4.8 % |
| **Strengths** | cheap; simple |
| **Weaknesses** | **blind to kerbs** (everything under 0.3 m is "road") |
| **Status** | **Default**, Rust since 17 Nov |

### Lidar cubes over the fitted road (`aav_voxels`)

| | |
|---|---|
| **What it is** | the lidar read as 0.25 m cubes, each measured from the road fitted to that scan |
| **Main goal** | "This is a car not a needle" (Ben): a leaning car doesn't see the road as a wall |
| **What it adds** | 3D obstacles for the costmap; the base of the kerbs and the things |
| **From** | Ben's idea; ours |
| **What we found** | Gazebo, 30 Oct: stopped 0.44 m before the post (the flat scan: 0.36 m); the lot tour the same. The Rust: 0.53 ms a whole turn against the Python's 4.29 ms |
| **Strengths** | sees everything behind the nearest thing; handles a pitched car |
| **Weaknesses** | more work a scan; the leaning car untested (Gazebo can't lean) |
| **Status** | **Challenger** for the costmap (`voxels:=true`); its Rust node runs the kerbs and the things |

### The kerb step finder (`aav_voxels_rs` `steps.rs`)

| | |
|---|---|
| **What it is** | a point more than 0.10 m (at most 0.5 m) above another within 0.3 m is a step; kept 10 s, since the roof lidar loses a kerb near the bumper |
| **Main goal** | Ben, 15 Nov: "we don't want to hit a curb" |
| **What it adds** | kerbs in the local costmap and the stop zone |
| **From** | ours. Patchwork++ was read for it (below) |
| **What we found** | Ravensim: before, the car **drove into** a 0.3 m kerb; now it stops **0.80-0.82 m before** it. A 0.3 m kerb seen from 8.5 m; 0.15 m from 4.7 m; 0.10 m never. No phantom stops on the campus |
| **Strengths** | simple; local, so a sloped road isn't a step |
| **Weaknesses** | a 0.15 m kerb is seen too late at 3 m/s (TODO 219); not slopes or grass |
| **Status** | **Default** in `stack:=ours` since 16 Nov. Car champion: off |

### Radar clustering (`aav_radar_clusters`, Rust)

| | |
|---|---|
| **What it is** | DBSCAN groups the radar's points into objects with boxes |
| **Main goal** | objects from the radar |
| **What it adds** | a second sensor for what is ahead; the radar's speed feeds the things |
| **From** | DBSCAN: M. Ester et al., KDD 1996. A rewrite of the 2025-26 team's C++ node |
| **What we found** | the same clusters as their C++, bit for bit, on 94 recorded clouds |
| **Strengths** | simple; proven by the 2025-26 team |
| **Weaknesses** | a few points per object; no tracking by itself |
| **Status** | **In use** wherever the radar runs |

### The things tracker (`aav_things_rs`)

| | |
|---|---|
| **What it is** | the lidar's cubes grouped into things, one constant-velocity Kalman filter each, with the radar's speed and a memory. Each thing takes the detector's label it overlaps in the camera (person, car, stop sign) |
| **Main goal** | give each thing near the car an id, a speed and a memory |
| **What it adds** | the behaviour planner waits for what moves and passes what is still |
| **From** | AB3DMOT (X. Weng et al., IROS 2020): the simple tracker is a strong baseline. The naming: Autoware's `roi_cluster_fusion` idea |
| **What we found** | Ravensim: the map's walls as background cut the campus's things from 9.9 to 1.7, every walker still found. A row of thin posts: 33 false holds before the gates, 0 after. The Rust: 1.5 % of a core against 7.5 % |
| **Strengths** | the lidar says where, the camera says what; cheap |
| **Weaknesses** | one motion model for everything; the camera calibration is needed on the car (Q22) |
| **Status** | **Default** in `stack:=ours` (with the behaviour planner). Studied for it: Fast-Poly, OC-SORT (below) |

### The lane finder (`aav_lane_finder_rs`)

| | |
|---|---|
| **What it is** | classic computer vision: a bird's-eye warp, a paint mask, sliding windows, a polynomial fit per line |
| **Main goal** | where the lane's lines are |
| **What it adds** | the lane pose; the costmap's camera lanes (`lanes:=camera`); each line's paint, white or yellow (19 Nov) |
| **From** | OpenCV (Apache-2.0) in the Python; ported with no OpenCV in the Rust |
| **What we found** | 0.78 ms a picture (Python). The Rust: the same lanes on 50 pictures and 1,535 Ravensim frames, 3.7 % of a core against 17.4 % |
| **Strengths** | no model to train; fast |
| **Weaknesses** | needs clear paint; untested on the car's cameras |
| **Status** | **Default**, Rust since 18 Nov |

### Studied only: perception

| Idea | What it is | Licence | Fit |
|---|---|---|---|
| **Patchwork++** (S. Lee et al., IROS 2022) | finds the ground ring by ring | BSD-2-Clause | its lesson used (find the ground locally); a port is TODO 215 |
| **Fast-Poly / Poly-MOT** | a motion model per kind (bicycle for a car) | MIT | for the things tracker (TODO 217) |
| **OC-SORT** (CVPR 2023) | a thing lost then found re-fits its speed | MIT | the same |
| **MapMOS** (PRBonn) | moving points against the map, a network | MIT | later |

## 5. Planning: which way, and how fast?

Three levels (Ben, 14 Nov): "Zoomed out is routes, middle is how we get from cube to cube
and finally close up is obstical avoidance". The full story: [Costmaps](costmaps.md),
[Our own planner](own-planner.md), [Controllers](controllers.md), [MPPI with lag](mppi-lag.md),
[Routes](routes.md), [How the car stops](stopping.md).

```text
 route (streets, lanes) ─▶ path (the line to the goal) ─▶ motion (follow it, 20 times a second)
       Valhalla               lane centre / lattice          mppi_lag
```

### The costmap (`aav_costmap`, Rust)

| | |
|---|---|
| **What it is** | a grid on the ground; each cell a cost, from layers: the map, the sensors, the kerbs, inflation round things |
| **Main goal** | "where may I go, and how risky is each spot" |
| **What it adds** | what the planner and MPPI search |
| **From** | Nav2's `nav2_costmap_2d` (Apache-2.0), from ROS 1's `costmap_2d` (BSD), ported |
| **What we found** | the same as Nav2's on every one of 40,000 cells (5 runs). In 64 m chunks: every leg as before, memory 474 → 145 MB, CPU 12 → 4 %. League: Nav2's costmap went over the centre line 6.5 % of University Drive; ours 0 % |
| **Strengths** | proven layers; each costmap on its own thread; chunks keep big maps cheap |
| **Weaknesses** | a grid: a lane is only what the layers draw |
| **Status** | **Default** (`ours_all_chunks`). Studied: grid_map, nvblox |

### Lanes as soft costs (`lanes:=ramp`)

| | |
|---|---|
| **What it is** | the lane map drawn into the costmap as a ramp, lowest at the right lane's centre |
| **Main goal** | make the planner keep right |
| **From** | the idea of Nav2's Costmap Filters (A. Merzlyakov, S. Macenski), Apache-2.0 |
| **What we found** | over the centre line 2-27 % of a leg, against 75 % with none. Once `path:=lane` kept right: a tie |
| **Strengths** | any planner keeps right |
| **Weaknesses** | a nudge, not a rule |
| **Status** | **Challenger** |
| **From the camera** (`lanes:=camera`, `both`; 19 Nov, TODO 264) | the same costs from the lane finder's lines where there is no lane map: the yellow line and the oncoming lane medium, the kerb high. Built, not yet scored |

### Routes: Valhalla, OSRM and the campus road graph (`aav_routes`)

| | |
|---|---|
| **What it is** | the way to a destination along real streets, cut into waypoints for the navigator. On the campus, by a building's name on its own road graph (Dijkstra) |
| **Main goal** | the zoomed-out level: "drive to the library" |
| **From** | Valhalla (MIT), picked; OSRM (BSD-2), an option; OpenStreetMap (ODbL); Dijkstra, 1959 |
| **What we found** | Carleton to Lansdowne: Valhalla 1986.9 m, OSRM 1986.4 m, the same shape. Valhalla installs through Pixi everywhere, the Jetson too |
| **Strengths** | offline; seconds to build Ottawa's graph |
| **Weaknesses** | streets, not lanes; only a suggestion: the navigator still plans every step |
| **Status** | **In use**, on request. Untested on the car |

### The State Lattice (`aav_planner`, `path:=lattice_rs`)

| | |
|---|---|
| **What it is** | joins short car moves from a ready-made list (primitives), searched with A* |
| **Main goal** | a path the car can really drive, reversing too |
| **From** | M. Pivtoraiko, R. Knepper, A. Kelly, J. Field Robotics 2009; Nav2's Smac Lattice (Apache-2.0), ported |
| **What we found** | Gazebo, 30 Oct, reversing on: every leg, **29 s quicker** a lot tour than Hybrid-A*. Ours: 12 of 12 legs, 3-5 ms a plan. League: on University Drive it leaves the lane (over the centre line 58 %) |
| **Strengths** | fast; never tighter than full lock |
| **Weaknesses** | knows no lanes; only as good as its primitives |
| **Status** | **Car champion**. In `stack:=ours` it plans off the lane map, with its smoother pulling harder (`lattice_rs_strong`) |

### Hybrid-A* (`path:=hybrid_rs`)

| | |
|---|---|
| **What it is** | A* with real car moves (straight, full left, full right), Reeds-Shepp curves to finish |
| **Main goal** | the same as the lattice |
| **From** | D. Dolgov et al., IJRR 2010; Nav2's Smac Hybrid, ported 14 Nov |
| **What we found** | the default until 1 Nov (the only one to finish the U-turn forward only). League: the **calmest** on University Drive (sway 0.035) but over the centre line 74 %. The lot (14 Nov): 7 of 8 legs, 585 s against the lattice's 393 |
| **Strengths** | smooth paths; good in open space |
| **Weaknesses** | slower; the wrong lane on roads |
| **Status** | **Challenger** (TODO 178) |

### Smooth lattice moves (`path:=lattice_rs_smooth`)

| | |
|---|---|
| **What it is** | lattice moves whose curvature ramps no faster than the steering turns (clothoids), straight at both ends |
| **Main goal** | no curvature jumps for the follower to guess |
| **From** | T. Fraichard, A. Scheuer, IEEE T-RO 2004; our generator (`tools/lattice_smooth.py`) |
| **What we found** | offline: the worst curvature change 0.38-0.54 → 0.27-0.30 1/m². Followed by `mppi_lag`: no better (off the path 0.077 against 0.067 m mean), 20 s slower |
| **Strengths** | paths the steering can follow exactly |
| **Weaknesses** | longer paths; no gain shown yet |
| **Status** | **Challenger**, tried with no win (TODO 177: score again) |

### The lane's centre (`path:=lane`, with passing)

| | |
|---|---|
| **What it is** | on the lane map, the right lane's centre by the road graph; the lattice only joins the car and the goal to it. Something parked on the lane is passed |
| **Main goal** | drive on the right side of the road |
| **From** | Autoware's route-then-lane-centre idea and Apollo's lane-graph routing (Apache-2.0); our code |
| **What we found** | University Drive, 3 clean runs: over the centre line **0 %** (61 % before); sway 0.055 against 0.087 m/s²; 0.10 m off the lane's centre. A parked box: over the line 5-5.5 s, against 34-36 s for the lattice |
| **Strengths** | the only path that keeps right |
| **Weaknesses** | needs a lane map; its corners are circle arcs, so curvature jumps (TODO 211: clothoids) |
| **Status** | **Default** in `stack:=ours` since 16 Nov. Car champion: `lattice_rs` |

### Lanelet2 routing (`path:=lane_lanelet2`)

| | |
|---|---|
| **What it is** | the lane-centre planner routed on a Lanelet2 map: lanes, their neighbours, one way, stop lines |
| **Main goal** | plan by lanes like a driver; open maps made in other tools |
| **From** | Lanelet2 (F. Poggenhans et al., ITSC 2018), BSD-3: its format and routing idea. Our own Rust reader (`lanelet-map`) |
| **What we found** | every leg of the campus and University Drive, but slower: campus 717 s against 533-562 s |
| **Strengths** | a standard map format; lane changes and stop lines |
| **Weaknesses** | slower routes today |
| **Status** | **Challenger** |

### The diffusion planner (`planner:=diffusion`)

| | |
|---|---|
| **What it is** | a small flow-matching model (3.9 M weights) draws 16 paths; a guard checks each against eight rules; the cheapest that passes is the plan, else the lattice's |
| **Main goal** | try a learned planner safely |
| **What it adds** | our first learned part, behind a guard |
| **From** | flow matching and diffusion planners (papers in `aav_diffusion_planner/PROVENANCE.md`); trained on our lattice's and Hybrid-A*'s paths |
| **What we found** | Ravensim's lot, 8 Nov, 3 runs: **88 %** of plans from the model; 171-175 s a tour against the lattice's 180-185; never tighter than full lock. A plan on a Mac CPU: 95 ms mean, 141 ms at the 95th percentile; the Orin's budget is 100 ms, so it needs the GPU |
| **Strengths** | quicker tours; the guard makes it safe to try |
| **Weaknesses** | copies the lattice, so at best equals it; needs a GPU; not scored in the 16 Nov league |
| **Status** | **Challenger**. On the car only behind `diffusion_trial:=true`, at 0.5 m/s |

### Retired planners: Smac 2D, NavFn, Theta* (Nav2)

They ignore the turning circle: in Gazebo (29 Oct) Smac 2D planned a U-turn as a straight
line back, as if the car could spin. The car refused them; **retired** with Nav2 on 17 Nov.

### MPPI that knows the lag (`aav_mppi_lag`, `steer:=mppi_lag_calm`)

| | |
|---|---|
| **What it is** | 2000 possible futures, 2.8 s long, through a model of the car; each scored against the path, the costmap and the goal; the best blended |
| **Main goal** | follow the path and avoid what the costmap sees |
| **What it adds** | the car's actuator lags in its model (Ben, 7 Nov: "have the algorithms know about lag") |
| **From** | G. Williams et al., J. Guidance, Control, and Dynamics 2017; Nav2's MPPI (Apache-2.0), ported to Rust with our lag model |
| **What we found** | predicts the car 70 times better (8 mm over 2.8 s, against 0.56 m). 1.7 ms a tick on an M4 Max. League: every leg of every tour. The calm version (slow map corrections, a steering cost) swayed 0.063 on University Drive; plain `mppi_lag` 0.144 |
| **Strengths** | avoids obstacles; reverses; knows the lag |
| **Weaknesses** | heavy: twice Nav2's MPPI's CPU; many settings; Orin timing not measured |
| **Status** | **Default** (`mppi_lag_1k` since 20 Nov: `mppi_lag_calm` with 1,000 samples, no column worse in stage 2, half the work). Car champion: `mppi_lag` |

### The lag-aware MPC (`mpc_lag`, Python; `mpc_lag_rs`, Rust)

| | |
|---|---|
| **What it is** | each tick, a quadratic program plans steering and speed through the car's lags, within its limits |
| **Main goal** | follow the path as closely as the car allows |
| **From** | Autoware's MPC lateral controller design (Apache-2.0, idea only); OSQP's ADMM algorithm (B. Stellato et al., 2020), our own code |
| **What we found** | Rust: 0.75 ms a tick against the Python's 5.3, the same commands. League, forward only: follows **2-7 times closer** than MPPI, but sways (0.160 against 0.068 on University Drive). It **can't reverse** |
| **Strengths** | closest to the path; fast; respects the steering's limits |
| **Weaknesses** | swings; no reverse; doesn't read the costmap |
| **Status** | **Challenger** (TODO 213: a steering-rate cost and reverse) |

### Our trackers: pure pursuit, Stanley, LQR, MPC (`aav_track`)

| Tracker | What it is | From | Gazebo, 29 Oct (legs of 12) |
|---|---|---|---|
| Pure pursuit | steer the arc to a point ahead | R. C. Coulter, CMU, 1992 | 12 |
| Stanley | heading error plus sideways error at the front axle | S. Thrun et al., J. Field Robotics 2006 | 12 |
| LQR with preview | optimal gains from the car's model, 2 s of bends ahead | textbook control | 6-10 |
| MPC, `mpc_full` | a QP each tick; `mpc_full` plans the speed too | textbook; Autoware's design | 12; 10 |

| | |
|---|---|
| **Main goal** | simple followers to score MPPI against |
| **Strengths** | light (0.2 ms a tick); easy to explain |
| **Weaknesses** | **forward only**, so skipped in the league (reversing is a must-have); they don't read the costmap |
| **Status** | **Challengers**. Pure pursuit is **in use** in drive-to-point (A to B) |

### Retired followers: Nav2's MPPI and Regulated Pure Pursuit

| | Nav2's MPPI | Regulated Pure Pursuit |
|---|---|---|
| **From** | Nav2 (Apache-2.0) | S. Macenski et al., Autonomous Robots 2023 |
| **What we found** (league, 16 Nov) | University Drive 3 of 6 legs: stuck, "the car doesn't fit at the start" | 42 sway changes per 100 m on the lot |
| **Why retired** | ported as `mppi_lag`; Nav2 deleted 17 Nov | swings; Nav2 deleted |

### Retired: the lane followers (19 Nov)

| | the 2025-26 PID (`control_interface`, Python and Rust) | pure pursuit | Stanley |
|---|---|---|---|
| **From** | the 2025-26 team's `Lane_Detection_ROS2` | R. C. Coulter, 1992 | S. Thrun et al., 2006 |
| **What we found** (19 Nov, 3 runs; Library Road RMS) | 3.9-4.1 cm, 187 flips a minute | **2.4 cm**, 1 flip a minute | 3.0 cm, 6 flips |
| **Why retired** | Ben, 19 Nov: "drop there code all together": lanes are perception, the costmap prices them, navigation drives (TODO 264). Git tag `save/lane-followers`; [their scores](history/lane-followers.md) | the same | the same |

### The navigator and the smoother (`aav_navigator`, `aav_smoother`)

| | |
|---|---|
| **What it is** | the navigator: plan, follow, replan, recover, as one state machine under Nav2's action names. The smoother: limits the command for a car, from the steering's rate and full lock |
| **From** | Nav2's `bt_navigator`, `waypoint_follower` and velocity smoother (Apache-2.0), rewritten in Rust |
| **What we found** | league: Nav2's `bt_navigator` drove the same (a tie); our smoother a tie |
| **Strengths** | no goal sender had to change; the smoother knows the car can't turn on the spot |
| **Status** | **Default** |

### The behaviour planner and the stopping law (`aav_behaviour_rs`)

| | |
|---|---|
| **What it is** | states DRIVE, APPROACH, STOPPED, WAIT, GO, ASK; a speed cap from one stopping law for a stop sign, an obstacle in the lane, the fence and the goal. One message, the road ahead, always ends in a stop |
| **Main goal** | slow before each stop, wait for what moves |
| **What it adds** | the rules of the road above the planners (Ben, 16 Nov: "if we see a car people or stuff moving around us we should likely just stop") |
| **From** | our design; the road ahead after a railway's movement authority |
| **What we found** | league: without it, a little more sway on the road (0.074 against 0.063). The Rust: the Python's answers bit for bit, 1.3 % of a core against 9.0 % |
| **Strengths** | one law for every planned stop; the locks still make every stop |
| **Weaknesses** | one person crossing flipped it 4 times (a 1 s hold added, TODO 189) |
| **Status** | **Default** in `stack:=ours`. Car champion: off |

### Studied only: planning

| Idea | Licence | Fit |
|---|---|---|
| **Smooth MPPI / KMPPI** (`pytorch_mppi`, UM ARM Lab) | MIT | sample the steering's change, so commands come out smooth (TODO 212) |
| **Autoware's MPC lateral controller** | Apache-2.0 | a steering-rate cost and reverse for `mpc_lag_rs` (TODO 213) |
| **Apollo's piecewise-jerk path** (Zhang et al., IV 2020) | Apache-2.0 | a QP along the lane: smooth corners and passes (TODO 211, 214) |
| **Autoware's static obstacle avoidance**; **path optimizer** | Apache-2.0 | the pass's length from a jerk limit; a drivable path before the follower |
| **Vector Pursuit** (Black Coffee Robotics) | Apache-2.0 | a follower yardstick |
| **CommonRoad route planner** (TU Munich) | BSD-3-Clause | where to change lane |
| **grid_map** (ANYbotics); **nvblox** (NVIDIA) | BSD-3-Clause; Apache-2.0 | the costmap's layers; a GPU distance field |

## 6. Control and the car

The full story: [Controllers](controllers.md), [Car model](car-model.md),
[Learn: control](learn-control.md), [The car as a linear model](vehicle-model.md).

### ego, the virtual car (`aav_ego_rs`)

| | |
|---|---|
| **What it is** | the only node that knows the car is a car: turns a speed and turn rate into an Ackermann command, 50 times a second |
| **Main goal** | one place for the car's limits |
| **What it adds** | every driver (joystick, navigation, AI) is the same to the car |
| **From** | the bicycle model: steering = atan(wheelbase × turn rate ÷ speed) |
| **What we found** | Rust: the Python's answers to 0; 2.4 % of a core against 17 % |
| **Strengths** | limits (1.0 m/s, ±0.5 rad, the steering rate) and a 0.3 s timeout in one place |
| **Weaknesses** | every car number is a guess until the car is measured |
| **Status** | **In use**, Rust by default |

### One car model, the lag lead and the trace

| | |
|---|---|
| **What it is** | one model of the actuators' lags, dead times, the steering motor and the bicycle, in Python and Rust. ego asks a little early for the lag; it publishes where the car goes in the next 2 s |
| **Main goal** | every part predicts the car the same way |
| **From** | ours (Ben, 7 Nov); the same vectors check both languages |
| **What we found** | it lets MPPI predict the car 70 times better (above) |
| **Weaknesses** | the dead times are 0 until measured |
| **Status** | **In use** |

### The speed loop (`speed_loop:=default`, `smooth`, `lqi`, `firm`)

| | |
|---|---|
| **What it is** | feed-forward plus PI on the wheel speed, out as throttle or brake, with a smooth stop |
| **Main goal** | hold the speed ego asks for, and stop where it planned |
| **From** | our plan (27 Oct); the smooth stop from Autoware's `pid_longitudinal_controller` design; LQI from textbook control |
| **What we found** | `default` overshoots 14 % (1.14 m/s for 1.0). League: `smooth` a tie, `lqi` slower, `firm` sways twice as much. A `firm` car planned with `smooth` models stops best (0.0 s over the limit) but goes 1 m off on the backing legs |
| **Strengths** | small enough for the ESP32 |
| **Weaknesses** | the overshoot; the brake is weak past its coast band |
| **Status** | `default`: **Default**. `smooth`, `lqi`, `firm`: **challengers** |

### The pedal (`pedal.rs`)

| | |
|---|---|
| **What it is** | a person's stick as accelerator and brake: let go slows at 1.0 m/s², pulled back brakes up to 2.0 m/s², then reverses after a full stop |
| **Main goal** | Ben, 15 Nov: the driver "had to swtich between acceleration, braking and reversing" |
| **What we found** | 8 laptop tests; a live ego with a pretend joystick did what was asked. Not yet felt in the sim or on the car |
| **Status** | **In use** for people only (joystick, RC, keyboard) |

### The transfer functions, with the IMU blended in (`car-test --hallway`)

| | |
|---|---|
| **What it is** | the car modelled as a delay plus a lag; in a hallway it speeds up, coasts, brakes and steers, and each run is fitted. The IMU's acceleration is blended with the wheel speed (a small Kalman filter) |
| **Main goal** | measure the car's real numbers, with error bars |
| **From** | Kalman, 1960; the bootstrap (B. Efron, 1979) for the error bars |
| **What we found** | in Ravensim and the car model, it found the known numbers within its error bars (with wheel noise and IMU bias). Untested on the car |
| **Status** | **In use** as a test tool (testing plan phase B) |

### The steering motor's PID (Jrk G2)

Underneath everything: the steering actuator's own position PID, about 1 kHz. **In use**;
tune it, don't replace it.

## 7. Safety: the locks that are never swapped

The full story: [How it fits, section 7](how-it-fits.md#7-safety-layers),
[The safety case](safety-case.md), [Geofence](geofence.md).

| Lock | What it does | What we found | Status |
|---|---|---|---|
| **The e-stop chain** | joystick deadman; `/safety/estop`; ego's 0.3 s timeout; the backend brakes when ego is quiet; the firmware's 100 ms watchdog; the hardware stop | layer by layer in the [safety case](safety-case.md); today's 2025-26 ESP32 firmware has no timeout, so programs are locked out | **In use** |
| **The command mux** (twist_mux) | the highest priority that spoke in 0.5 s wins; locks hold every driver below them | stock (Apache-2.0); `aav_mux_state` names who drives (226 of 227 checks agreed) | **In use** |
| **The output gate** (output and sensor muxes) | the real car gets commands only while every source is REAL | a simulator on the network can't drive the car | **In use** |
| **The sensor watchdog** | stops navigation when a sensor goes quiet, sends NaN, freezes or jumps; latches STUCK steering and OVERSPEED | `sim-check --faults` 15 of 15; stuck steering noticed 3.6 s after (before: never) | **In use** (Python); the Rust twin a **challenger**: a person's call |
| **The stop zone** (`aav_stop_zone`) | lidar or radar points in the box ahead (or behind) stop navigation | Nav2's `collision_monitor` ported; in a shadow run it stopped every time Nav2's did, no later. Nav2's stopped 0.3-0.4 m short of a box on the bench | **In use** since 17 Nov |
| **The geofence** | holds every program before its stopping path reaches the fence; never a person | bench, 29 Oct: stopped 1.06 m on, where it counts on 1.56 m | **In use** |
| **The stop-sign lock** | a 4 s stop for a close stop sign | Ravensim: stood still 4.3 m before the sign | **In use** |

## 8. Simulators

The full story: [Ravensim](own-simulator.md), [CARLA](carla.md),
[Gazebo, as it was](history/gazebo-as-it-was.md).

| Simulator | What it is | From, licence | What we found | Status |
|---|---|---|---|---|
| **Gazebo** | Open Robotics' 3D simulator | Apache-2.0 | ran our trials until Ravensim took over (8 Nov); on a loaded Mac its sensors went stale | **Retired** 16 Nov; its sensor numbers live on in Ravensim |
| **CARLA** | photo-real towns on Unreal Engine | MIT | the adapter is built and tested without a server; never run against one (no Mac version) | **Kept**, untested: needs the NVIDIA computer |
| **Ravensim** | our own, in Rust: the worlds, the car, lidar, radar, IMU, GPS, wheels, cameras (Bevy) | ours | the default since 8 Nov; a seed repeats every sensor; the shared car model inside; runs on a Mac or Linux | **Default**, the only simulator |
| Turtlesim, bench mode | stand-ins for the whole command path and the hardware | ROS's turtlesim; ours | the first check on any laptop | **In use** |

## 9. Learned, for later (studied only)

Each needs training data and GPUs we don't have this year (Ben, 28 Oct). For the 2027-28
team ([TODO](../TODO.md)).

| Idea | One line |
|---|---|
| **Learned BEV** (BEVFusion, BEVFormer) | all cameras and the lidar into one top-down view |
| **UniAD** | one network for perception, prediction and planning |
| **World models** (Wayve's GAIA-1); **vision-language driving** (Waymo's EMMA) | a model that imagines the next seconds of driving; a language model that drives |
| **MTR, Wayformer** | predict other road users' paths; at 1 m/s, constant speed is enough today |
| **Mamba** (state-space models) | learn what our linear car model misses, after hours of car logs |

## 10. What we use today, and why it won

| Area | Today (`stack:=ours`) | Why it won |
|---|---|---|
| Position | the EKFs on GPS, with the lane pose | the lane pose cut the sideways error on University Drive, no worse elsewhere. **Next:** NDT leads (every GPS fault driven through) |
| Maps | KISS-SLAM from our drives (offline) | 2.3 cm off the truth, against 27-33 cm from the EKF |
| Objects | RF-DETR Nano | saw the stop sign twice as far as YOLO11n; Apache-2.0 |
| Obstacles | the flat scan, the kerb step finder, the things tracker | the step finder stopped before a kerb the scan drove into |
| Lanes | the classic lane finder (Rust) | no training; the same lanes as the Python at a fifth of the CPU |
| Costmap | ours, in 64 m chunks | the same cells as Nav2's, a third of the memory |
| Route | Valhalla; the campus road graph | the only router that installs everywhere with no compiling |
| Path | the lane's centre (the lattice off the map) | the only path that keeps right: 0 % over the centre line |
| Motion | `mppi_lag`, calm | every leg of every tour, reversing; the calmest that reverses |
| Rules | the behaviour planner | slows before stops; less sway with it than without |
| Control | ego and the PI speed loop | the other loops tied, swayed or were slower |
| Safety | every lock above | never swapped (`CLAUDE.md`) |
| Simulator | Ravensim | ours, light, repeatable, on every laptop |

**Remember:** every number above is from a simulator, the bench or unit tests. The car is
being upgraded (18 Nov); the stands, the hallway and a mapping drive come next
([The testing plan](testing-plan.md)).
