# Acknowledgements: what we built on

Everything we copied, ported, or took an idea from, with its licence and where we used it.
Ben, 14 Nov: "give acknowlegement to stuff we have looked at or used as insperation".
Add a row in the same commit as the work (`CLAUDE.md`). Copied or ported code also keeps
a `PROVENANCE.md` beside it, with the repository, the commit and every change since.

| Kind | Means |
|---|---|
| **ported** | their code, rewritten by us (usually C++ to Rust); `PROVENANCE.md` beside ours |
| **copied** | their code, kept and changed; `PROVENANCE.md` beside it |
| **runs** | we run their program or library as it is |
| **idea** | we read it and built our own; no code taken |
| **data** | maps, models or numbers we use |

## The earlier Carleton AAV teams

| Who | What we took | Kind |
|---|---|---|
| The 2025-26 team (`AAV-2025-2026`) | the ESP32 actuator firmware, the wheel encoder and sentinel Picos, the IMU driver, the radar driver and its clustering, lane keeping, the HUD app, the steering geometry | copied, ported |
| The 2024-25 team (`Carleton-AAV-Capstone`) | the battery monitor, the camera calibrations, the first YOLO stop-sign detector | copied |

Each package's `PROVENANCE.md` says which repository and commit; `docs/older-code.md` has the whole list.

## Navigation

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **Nav2** (S. Macenski and the Nav2 authors; "The Marathon 2: A Navigation System", IROS 2020) | Apache-2.0 | ported, runs | our costmap (`aav_costmap`), planner (`aav_planner`), navigator (`aav_navigator`), smoother (`aav_smoother`) and follower (`aav_mppi_lag`) are Nav2's, in Rust, as are the stop zone (`aav_stop_zone`, from `collision_monitor`) and the map server (`aav_costmap` `map_server`, `map_server`'s job). Nav2 itself ran as the yardstick until 17 Nov, when it was deleted (TODO 184); `nav2_msgs` (the message types) and `nav2_amcl` (the AMCL trial) still run |
| ROS 1 `costmap_2d` (Willow Garage) | BSD | ported (through Nav2) | the costmap's layers |
| **Nav2's Costmap Filters** (keepout and speed masks; A. Merzlyakov, S. Macenski) | Apache-2.0 | idea | the lanes layer (`aav_costmap` `lanes.rs`, `lane_costs`, TODO 148): lanes as costs the planner pays, not walls. Ours is drawn from the lane map per route, since a mask can't know which way the car drives |
| **MPPI**: G. Williams, A. Aldrich, E. Theodorou, "Model Predictive Path Integral Control: From Theory to Parallel Computation", J. Guidance, Control, and Dynamics, 2017 | paper | idea | the local planner (`aav_mppi_lag`, and Nav2's) |
| **State Lattice**: M. Pivtoraiko, R. Knepper, A. Kelly, "Differentially Constrained Mobile Robot Motion Planning in State Lattices", J. Field Robotics, 2009 | paper | idea | the path planner (`aav_planner`, Nav2's Smac Lattice) |
| **Hybrid A\***: D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments", IJRR, 2010 | paper | idea, ported (through Nav2) | Nav2's Smac Hybrid; ours, built 14 Nov (TODO 178): Nav2's SmacPlannerHybrid ported into `aav_planner` (`planner_search:=hybrid`), sharing the lattice's search |
| **Continuous-curvature paths**: T. Fraichard, A. Scheuer, "From Reeds and Shepp's to Continuous-Curvature Paths", IEEE Trans. Robotics, 2004 | paper | idea | the lane line's corners as clothoid, arc, clothoid (`aav_planner` `lane.rs`, `lane_corners:=clothoid`, TODO 211) |
| **A\***: P. Hart, N. Nilsson, B. Raphael, IEEE Trans. Systems Science and Cybernetics, 1968 | paper | idea | the planners' search |
| **Dijkstra**: E. W. Dijkstra, "A note on two problems in connexion with graphs", Numerische Mathematik, 1959 | paper | idea | the campus route graph (`aav_routes`) |
| **Reeds-Shepp** curves: J. A. Reeds, L. A. Shepp, Pacific J. Mathematics, 1990 | paper | idea | reversing paths; through OMPL in Nav2's planner |
| **Continuous-curvature paths**: T. Fraichard, A. Scheuer, "From Reeds and Shepp's to Continuous-Curvature Paths", IEEE T-RO, 2004 | paper | idea | the steering-rate lattice moves (TODO 177) |
| OMPL (the Open Motion Planning Library) | BSD-3 | ported (through Nav2) | the curves inside the planner |
| **Pure Pursuit**: R. C. Coulter, CMU-RI-TR-92-01, 1992; **Regulated Pure Pursuit**: S. Macenski et al., Autonomous Robots, 2023 | paper | idea | `aav_track`, Nav2's RPP; `aav_lane_follower_rs`'s `lane_follower:=pure_pursuit` (removed 19 Nov, `save/lane-followers`) (on the camera's lane centre, with the car's lag) |
| **Stanley**: S. Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge", J. Field Robotics, 2006 | paper | idea | `aav_track`'s Stanley follower; `aav_lane_follower_rs`'s `lane_follower:=stanley` (removed 19 Nov, `save/lane-followers`) (heading and cross-track at the front axle, from the camera's lane centre) |
| LQR and MPC (textbook control) | - | idea | `aav_track`'s LQR, MPC and MPC with lag; `aav_mpc_lag`, the MPC with lag in Rust |
| **OSQP**'s ADMM: B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd, "OSQP: an operator splitting solver for quadratic programs", Math. Programming Computation, 2020 | Apache-2.0 (the solver; we took the algorithm, not its code) | idea | `aav_track`'s `qp.py` and its Rust port in `aav_mpc_lag` (`qp.rs`, 15 Nov) |
| **Clarabel** (P. Goulart, Y. Chen; Oxford) | Apache-2.0 | idea, runs | read and weighed for `aav_mpc_lag`'s QP (15 Nov); we kept our ADMM for its warm start and the same answers as the Python (`aav_mpc_lag` README). Runs as a crate for the jerk-limited pass's QP (`aav_planner` `jerk_pass.rs`, TODO 214) |
| **Flow matching** and the diffusion planner's other ideas | see `aav_diffusion_planner/PROVENANCE.md` | idea | `planner:=diffusion` |

## Where the car is

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **robot_localization** (T. Moore, D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System", IAS-13, 2014) | Apache-2.0 | runs | the two EKFs, `navsat_transform` |
| The Kalman filter: R. E. Kalman, 1960 | paper | idea | the EKFs; the things tracker (`aav_things_rs`); the hallway's IMU-with-wheels filter, speed and IMU bias (`aav_actuator_tests` `fusion.py`, TODO 248) |
| **The bootstrap**: B. Efron, "Bootstrap Methods: Another Look at the Jackknife", Annals of Statistics, 1979 | paper | idea | the hallway fits' 90 % intervals, resampling the passes (`aav_actuator_tests` `errorbars.py`) |
| **Complementary filter**: W. T. Higgins, "A Comparison of Complementary and Kalman Filtering", IEEE Trans. Aerospace and Electronic Systems, 1975 | paper | idea | follow in odom, correct slowly (TODO 186): the followers take the fast part of the pose from odometry and the slow part from the map, the map -> odom correction rate-limited (`aav_mppi_lag` `map_correction.rs`, `aav_track` `slow_frame.py`) |
| **NDT**: P. Biber, W. Straßer, IROS 2003; M. Magnusson, PhD thesis, Örebro, 2009 | paper | idea | `aav_ndt` (its `PROVENANCE.md` lists the rest) |
| **Autoware's localization** (the Autoware Foundation): NDT the main pose, the GNSS only for the first pose (`pose_initializer`); `localization_error_monitor` and `pose_instability_detector` | Apache-2.0 | idea | `aav_pose_check` (`pose_source:=ndt`, TODO 210): NDT leads `ekf_global`, the GPS only checks it, and a lost NDT starts again from AMCL or the GPS. Our own checks: the gap against both covariances, NDT's score, the GPS's fix, spread and jumps |
| **Normalised innovation squared, chi-square gating**: Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, 2001 | book | idea | `aav_pose_check`: NDT against the GPS, d'(Σ_ndt + Σ_gps)⁻¹d over a window, 99.9 % for 2 numbers |
| **KISS-ICP**: I. Vizzo et al., RA-L, 2023 | MIT | runs | `kiss_icp:=true`, scored beside the EKF |
| **KISS-SLAM**: T. Guadagnino, B. Mersch, S. Gupta, I. Vizzo, G. Grisetti, C. Stachniss, "KISS-SLAM: A Simple, Robust, and Accurate 3D LiDAR SLAM System With Enhanced Generalization Capabilities", IROS 2025 (PRBonn, `kiss-slam` 0.0.2 on PyPI) | MIT (its wheel builds in g2o, BSD, and SuiteSparse's CHOLMOD, LGPL-2.1+: an offline tool, never on the car) | runs | `pixi run map-drive`: a map from a recorded drive, with loop closures (TODO 250, [Maps from drives](maps-from-drives.md)) |
| **MapClosures**: S. Gupta, T. Guadagnino, B. Mersch, I. Vizzo, C. Stachniss, "Effectively Detecting Loop Closures using Point Cloud Density Maps", ICRA 2024 | MIT | runs | KISS-SLAM's loop closures, in `map-drive` |
| **g2o**: R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, W. Burgard, ICRA 2011 | BSD (core) | runs | KISS-SLAM's pose graph, in `map-drive` |
| **Open3D**: Q.-Y. Zhou, J. Park, V. Koltun, 2018 | MIT | runs | KISS-SLAM's closure check; `map-drive --merge`'s and `map-area`'s ICP between drives' maps |
| **Least-squares fit of two point sets**: S. Umeyama, IEEE TPAMI, 1991 (after W. Kabsch, 1976) | paper | idea | `map-drive`: the SLAM path turned and moved onto the good GPS fixes (`slam_map.fit_2d`) |
| **UTM by series**: J. P. Snyder, *Map Projections: A Working Manual*, USGS, 1987 | public domain | idea | `slam_map.utm`, checked against GeoConvert's numbers in `aav_hud`'s `earth.rs` |
| **Removert**: G. Kim, A. Kim, "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images", IROS 2020 (`irapkaist/removert`) | CC BY-NC-SA 4.0 | idea | `map-area`'s occupancy across passes (TODO 259, `tools/passes.py`): a map point another drive sees through is gone. No code read or copied |
| **ERASOR**: H. Lim, S. Hwang, H. Myung, "ERASOR: Egocentric Ratio of Pseudo Occupancy-Based Dynamic Object Removal for Static 3D Point Cloud Map Building", RA-L 2021 (`LimHyungTae/ERASOR`) | GPL-3.0 | idea | `tools/passes.py`: judge the column over each 2D cell, not single points; a car's points go up to a height (`--top`), what is above stays. No code read or copied |
| **DUFOMap**: D. Duberg, Q. Zhang, M. Jia, P. Jensfelt, "DUFOMap: Efficient Dynamic Awareness Mapping", RA-L 2024 (`KTH-RPL/dufomap`) | BSD-3-Clause | idea | `tools/passes.py`: space a drive saw empty is empty, kept a margin off what that drive saw occupied (the pose's and the sensor's error: `--tolerance`) |
| **Occupancy grids**: A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation", Computer, 1989 | paper | idea | `map-drive`'s 2D map: hits and free rays per cell |
| **TUM trajectory format**: J. Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012 | format | idea | `map-drive`'s `poses.tum`; `ndt-map build --poses-file` |
| AMCL (Nav2's `nav2_amcl`, from Player's amcl: A. Howard, B. Gerkey, K. Stoy) | LGPL-2.1-or-later | runs, ported | `amcl:=true`, a trial: stock until 20 Nov (TODO 232); ported to Rust as `aav_amcl_rs` (the default, TODO 231; same licence; its `PROVENANCE.md` lists the files) |
| **Monte Carlo localization and KLD sampling**: D. Fox, W. Burgard, F. Dellaert, S. Thrun, AAAI 1999; D. Fox, IJRR 2003; S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, 2005 | paper | idea | `aav_amcl_rs`: the filter, the odometry motion model, low-variance resampling, recovery's averages |
| **Distance transform**: P. Felzenszwalb, D. Huttenlocher, "Distance Transforms of Sampled Functions", Theory of Computing, 2012 | paper | idea | `aav_amcl_rs`: the likelihood field's distances to the walls |
| **Lane markings against a map**: Autoware's YabLoc (camera road markings matched to a vector map); D. Schreiber, B. Switkes, C. Stiller, "LaneLoc: Lane marking based localization using highly accurate maps", IEEE IV 2013 | Apache-2.0; paper | idea | the lane finder as a pose sensor (`aav_lane_pose`, TODO 155): the painted lines and the lane map give the car's place across the lane, a pose sure only sideways for `ekf_global`. Much simpler than either: one offset, no particle filter |
| imu_tools (Madgwick filter) | BSD / GPL-3 | runs | the IMU's orientation |

## Seeing

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **RF-DETR** (Roboflow) | Apache-2.0 | runs, data | the object and stop-sign detector |
| **Ultralytics YOLO** | AGPL-3.0 | runs | the earlier detector, compared and kept as an option |
| **DBSCAN**: M. Ester et al., KDD 1996 | paper | idea, ported | the radar's clusters (`aav_radar_clusters`) |
| **AB3DMOT**: X. Weng, J. Wang, D. Held, K. Kitani, "3D Multi-Object Tracking: A Baseline and New Evaluation Metrics", IROS 2020 | paper | idea | the things' tracker: a constant-velocity Kalman filter per thing, greedy association (`aav_things_rs`, and its Rust twin `aav_things_rs`) |
| **NumPy**'s pairwise summation (`loops_utils.h.src`) | BSD-3 | ported (the algorithm) | `things-core`'s `pairwise_sum`, so a radar cluster's mean is numpy's to the last bit (`aav_things_rs`) |
| **nalgebra** | Apache-2.0 | runs | the things' 4 × 4 Kalman matrices (`things-core`), NDT's (`ndt-core`); the Rust lane finder's least-squares fit (`lane-core`) |
| OpenCV | Apache-2.0 | runs; ported (the formulas) | lane finding (the Python `aav_lane_finder`); its `projectPoints` lens model and its fixed-point bilinear `remap` rewritten in Rust for `aav_lane_finder_rs` (`lane-core`), so the bird's-eye view has OpenCV's grey levels exactly |
| **NumPy**'s `polyfit`, `convolve` ('same') and `linspace` | BSD-3 | ported (the algorithms) | `lane-core`: the lane lines' fit, the line starts' histogram and the sliding windows' edges, as the Python lane finder computes them (`aav_lane_finder_rs`) |
| **png** (image-rs) | MIT OR Apache-2.0 | runs (tests) | reads the Rust lane finder's test pictures (`lane-core`'s vectors) |
| OpenCV's `getOptimalNewCameraMatrix`, `undistortPoints`, `initUndistortRectifyMap`, 3 × 3 `invert`, `INTER_AREA` | Apache-2.0 | ported (the formulas) | the Rust camera stream's undistortion and preview (`stream-core`, `aav_camera_stream_rs`): the same new lens and grey levels as the Python's OpenCV |
| **jpeg-encoder** crate (V. Ströbel), 0.7 | (MIT OR Apache-2.0) AND IJG: "this software is based in part on the work of the Independent JPEG Group" | runs | the Rust camera stream's preview JPEG (`stream-core`) |
| GStreamer (`gst-launch-1.0`), FFmpeg (`ffmpeg`), MediaMTX | LGPL-2.1; LGPL-2.1+ / GPL; MIT | runs, as separate programs | decoding the camera Pi's stream (both camera_stream nodes run them as child programs, linked to nothing); MediaMTX on the Pi and in `pixi run measure stream` |
| Python's `shlex.split` and `str.format` | PSF-2.0 | ported (the rules) | the Rust camera stream fills and splits a GStreamer pipeline as the Python does (`stream-core`'s `decoders`) |
| **libc** crate; **serde_yaml_ng** | MIT OR Apache-2.0 | runs | the Rust camera stream: stopping the decoder's process group (`killpg`); reading camera_info YAML calibrations |
| NVIDIA TensorRT; Apple Core ML Tools | NVIDIA licence; BSD-3 | runs | the detector on the Orin and on Macs |
| **ONNX Runtime** (Microsoft) | MIT | runs | the Rust detector (`aav_detector_rs`), the Rust diffusion worker (`aav_diffusion_rs`), `tools/onnx_check.py` and `onnx_measure.py`; its float16 converter makes the FP16 detector (`export_model.py --fp16`) ([ONNX](onnx.md)) |
| **`ort`** crate (pyke), 2.0.0-rc.15, and its prebuilt ONNX Runtime | MIT or Apache-2.0 | runs | ONNX Runtime from Rust: `onnx-session`, `detector-core`, `diffusion-core`, `ufld-core` |
| ONNX; onnxscript | Apache-2.0; MIT | runs | exporting RF-DETR to ONNX (`tools/export_model.py --format onnx`) |
| rfdetr's PostProcess and COCO class list | Apache-2.0 | ported | `detector-core`'s decode and labels, through our `detr.py` |
| **Ultra-Fast-Lane-Detection v2**: Z. Qin, P. Zhang, X. Li, "Ultra Fast Deep Lane Detection With Hybrid Anchor Driven Ordinal Classification", IEEE TPAMI 2022 (`cfzd/Ultra-Fast-Lane-Detection-v2`) | MIT | idea (our own code from the paper; no code or weights copied) | the learned lane finder (TODO 262b): row anchors, the column as a class with its expected value, the existence branch, the local expectation when decoding (`tools/lane_train.py`, `aav_lane_finder_learned`) |
| **torchvision**'s ResNet-18 (the architecture; K. He et al., CVPR 2016) | BSD-3 | runs (random start, no pretrained weights) | the learned lane finder's backbone |
| **Pillow**'s resampling (`precompute_coeffs`, its 8-bit fixed point) | MIT-CMU (HPND) | ported (the rules) | the learned lane finder's crop and scale in Rust, so the model gets the numbers it was trained on (`ufld-core` `preprocess.rs`) |
| CLRNet (T. Zheng et al., CVPR 2022), CLRerNet; LaneATT (L. Tabelini et al., CVPR 2021); PolyLaneNet (L. Tabelini et al., 2020); CondLaneNet (L. Liu et al., ICCV 2021); YOLOP, YOLOPv2, TwinLiteNet | Apache-2.0; Apache-2.0; MIT; MIT; Apache-2.0; MIT | idea (looked at) | the lane detector survey (`docs/perception.md`); their published weights are trained on CULane (non-commercial), BDD100K, TuSimple or CurveLanes (not clearly open), so not used |
| **OWLv2**: M. Minderer, A. Gritsenko, N. Houlsby, "Scaling Open-Vocabulary Object Detection", NeurIPS 2023 (`google/owlv2-base-patch16-ensemble`, from Google's `scenic`/`big_vision`) | Apache-2.0 (code and weights); trained on WebLI pseudo-labels, Objects365 and Visual Genome | runs | the labelling loop's open-vocabulary detector, offline (`tools/label_drive.py`, TODO 252; [Labelling](labelling.md)) |
| **Hugging Face transformers** | Apache-2.0 | runs | OWLv2 on PyTorch (mps), in `label-drive` |
| **Grounding DINO** (IDEA Research), **YOLO-World** (Tencent), **SAM 2** (Meta) | Apache-2.0; GPL-3.0; Apache-2.0 | looked at | the labelling loop's other candidates: Grounding DINO a later challenger, YOLO-World left out (copyleft), SAM 2's masks not needed yet ([Labelling](labelling.md#why-owlv2-the-licences)) |

## Maps and routes

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **OpenStreetMap** contributors | ODbL | data | the campus world, its roads and buildings (`pixi run campus`) |
| **OSRM**; **Valhalla** | BSD-2; MIT | runs | street routes (`aav_routes`) |
| Contraction Hierarchies: R. Geisberger et al., WEA 2008 | paper | idea | how OSRM answers fast; noted for the route planner |
| **Lanelet2** (F. Poggenhans et al., ITSC 2018) | BSD-3 | idea | the lane map's shape and its `.osm` format (lanelets, their left and right bounds, `traffic_sign` rules with a `ref_line` stop line); its routing graph's idea (a lanelet's followers, its neighbours over a dashed line, a lane change's cost); tried on RoboStack; our own Rust reader and router, `lanelet-map`, which the route level routes on with `lane_graph:=lanelet2` (TODO 175) |
| **Autoware** (the Autoware Foundation) | Apache-2.0 | idea | the three map levels, map chunks loaded near the car, its tracker as a reference (TODO 156, 175); its `roi_cluster_fusion` (a lidar cluster's box projected into the camera, matched by IoU) for the things' names (TODO 192); its route-then-lane-centre path (the mission planner routes on the lanelet map, the behaviour path planner follows the lane's centre and shifts only to go round) for our lane-centre planner (`aav_planner` `lane.rs`, `path:=lane`, TODO 148; Fable's review, change 7) |
| **Apollo** (Baidu) | Apache-2.0 | idea | routing on a lane graph, then planning along the road ahead; its HD map cut into fixed tiles (`map-chunks`) |
| **roxmltree** (Y. Kladov and contributors) | MIT or Apache-2.0 | dependency | reading Lanelet2's `.osm` XML in `lanelet-map` |
| **OctoMap** (A. Hornung et al., *Autonomous Robots*, 2013) | paper | idea | 3D occupancy; why our chunks hold a flat hash of voxels instead of a tree (TODO 175) |
| **Quadtrees** (R. Finkel, J. Bentley, *Acta Informatica*, 1974) | paper | idea | splitting a chunk only where it pays, the fallback in the three-levels design |

## Looked at for the league (16 Nov)

Read, not used yet: the survey in [the league's scores](league-results-2026-11-16.md#the-survey-open-source-we-havent-tried).
Each has a TODO row to port its idea.

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **Smooth MPPI / KMPPI** (`pytorch_mppi`, UM ARM Lab) | MIT | idea | `mppi_lag`'s smooth sampling: the steering's noise drawn on its change and summed (`aav_mppi_lag` `optimizer.rs`, `mppi_lag_sampling:=smooth`, TODO 212) |
| **Autoware's MPC lateral controller** (the Autoware Foundation) | Apache-2.0 | idea | a steering-rate cost and reverse for `mpc_lag_rs` (TODO 213) |
| **Vector Pursuit** (`vector_pursuit_controller`, Black Coffee Robotics) | Apache-2.0 | idea | looked at as a follower yardstick |
| **Apollo's piecewise-jerk path** (Baidu; Zhang et al., "Optimal Vehicle Path Planning Using Quadratic Optimization for Baidu Apollo Open Platform", IV 2020); Autoware's static obstacle avoidance | Apache-2.0; paper | idea | smooth lane corners in `lane.rs` (TODO 211); the jerk-limited pass, a QP along the lane (`aav_planner` `jerk_pass.rs`, `lane_pass_shape:=jerk`, TODO 214) |
| **Patchwork++** (S. Lee et al., IROS 2022; URL-KAIST) | BSD-2-Clause | idea | finding the ground so a kerb stands out: `aav_voxels_rs` `patchwork.rs` (`ground:=patchwork`, TODO 215): its concentric zones, rings and sectors, the lowest-point seeds, a PCA plane a patch and its uprightness, elevation and flatness checks, rewritten in Rust. Read for the kerbs' step finder (`aav_voxels_rs` `steps.rs`, TODO 119): its lesson that the ground must be found locally, not one plane for all; we kept a simpler local rule (a point 0.10 m above another within 0.3 m), since its zones fit a pavement's top as ground too. No code taken |
| **grid_map** (ANYbotics, ETH); **nvblox** (NVIDIA) | BSD-3-Clause; Apache-2.0 | idea | looked at for the costmap's layers |
| **CommonRoad route planner** (TU Munich) | BSD-3-Clause | idea | looked at for where `lane.rs` changes lane |
| **Fast-Poly / Poly-MOT** (X. Li et al., 2023-24); **OC-SORT** (J. Cao et al., CVPR 2023); **MapMOS** (PRBonn) | MIT | idea | motion models per kind and re-found tracks: `aav_things_rs` and `aav_things_rs` `tracker` (`things_tracker:=per_kind`, TODO 217): a vehicle's noise a bicycle's, a person's round, and the speed re-fitted from the last sightings (OC-SORT's observation-centric re-update) |
| **Kinematic-ICP** (T. Guadagnino et al., PRBonn, 2025); **small_gicp** (K. Koide); **GenZ-ICP** (D. Lee et al., 2024) | MIT | idea | Kinematic-ICP: `aav_kinematic_icp` (TODO 216, `odom_lidar:=kinematic_icp`): one arc a step and the wheels as a prior, rewritten in Rust, no code taken; small_gicp and GenZ-ICP: yardsticks for NDT |

## Taught in the guide (19 Nov)

Explained in [How a self-driving car works](how-self-driving-works.md), not built (or
not yet). The papers the guide names; the ones we built on are in the tables above.

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **ICP**: P. J. Besl, N. D. McKay, "A Method for Registration of 3-D Shapes", IEEE T-PAMI, 1992 | paper | idea | the guide's ICP; inside KISS-ICP and `map-drive --merge` |
| **Scan Context**: G. Kim, A. Kim, IROS 2018; **OverlapNet**: X. Chen et al., RSS 2020; **IR-MCL** (PRBonn, 2023) | paper | idea | place recognition and learned MCL: studied |
| **YOLO**: J. Redmon et al., CVPR 2016; **DETR**: N. Carion et al., ECCV 2020 | paper | idea | the guide's detectors (we run Ultralytics YOLO and RF-DETR, above) |
| **SORT**: A. Bewley et al., ICIP 2016 | paper | idea | the guide's tracking |
| **RRT**: S. M. LaValle, 1998; **RRT\***: S. Karaman, E. Frazzoli, IJRR 2011 | paper | idea | the guide's sampling planners: not built |
| **Dubins** curves: L. E. Dubins, American J. Mathematics, 1957 | paper | idea | the guide's curves (Reeds-Shepp, above, is the one we use) |
| **UniAD**: Y. Hu et al., "Planning-oriented Autonomous Driving", CVPR 2023; Wayve's GAIA-1 | paper | idea | the guide's end-to-end section: studied |
| **Statecharts**: D. Harel, "Statecharts: A Visual Formalism for Complex Systems", Science of Computer Programming, 1987 | paper | idea | the guide's hierarchical state machines: studied |
| **BehaviorTree.CPP** (Nav2's tree engine); Nav2's `navigate_to_pose_w_replanning_and_recovery.xml` | MIT; Apache-2.0 | idea | the guide's behaviour trees: the tree we ran for a car until Nav2 went (17 Nov) |
| **Apollo's scenarios and stages**; **Autoware's behaviour path and behaviour velocity planners** | Apache-2.0 | idea | the guide's traffic-rule modules, in general terms; our road ahead's stops are the same shape in small |
| **IDM**, the Intelligent Driver Model: M. Treiber, A. Hennecke, D. Helbing, Physical Review E, 2000 | paper | idea | the guide's car following: not built |
| **POMDPs**: L. P. Kaelbling, M. L. Littman, A. R. Cassandra, Artificial Intelligence, 1998; D. Sadigh et al., "Planning for Autonomous Cars that Leverage Effects on Human Actions", RSS 2016 | paper | idea | the guide's decisions under uncertainty and games: studied |
| **ALVINN**: D. Pomerleau, NIPS 1989; **ChauffeurNet**: M. Bansal et al., RSS 2019; **DAgger**: S. Ross, G. Gordon, D. Bagnell, AISTATS 2011 | paper | idea | the guide's learned behaviour: studied |
| matplotlib; NumPy | PSF-based; BSD-3 | runs | drawing the guide's pictures (`tools/teach_figures.py`) |

## Simulators and screens

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **Gazebo** (Open Robotics) | Apache-2.0 | ran (until 16 Nov), idea | the first simulator, deleted 16 Nov ([as it was](history/gazebo-as-it-was.md)); its source stays in `vendor/gazebo/` to read; its world files (SDF) and sensor models are what ours copies the numbers from |
| CARLA | MIT | runs | `pixi run carla-car` |
| **Bevy** | MIT / Apache-2.0 | runs | our simulator's cameras, the developer window |
| **Mesa** (llvmpipe, lavapipe), from conda-forge | MIT | runs | drawing with no GPU (`tools/software_gl.sh`): Gazebo's cameras through OpenGL (until 16 Nov), Ravensim's through Vulkan (lavapipe, 15 Nov) |
| Khronos Vulkan loader (`libvulkan-loader`, conda-forge) | Apache-2.0 | runs | finds lavapipe for our cameras with no GPU (`tools/software_gl.sh`) |
| **raylib** | zlib | runs | the HUD, the map window, the debug screen |
| egui | MIT / Apache-2.0 | runs | the developer window's panels |
| Foxglove; Lichtblick | MPL-2.0 | runs | the cockpit layouts |
| Catppuccin | MIT | data | the screens' Mocha and Latte themes |
| *Initial D* (S. Shigeno) | - | idea | the red-and-black look of the screens; no art copied |
| Tesla's driving display | - | idea | the "squiggly line" on the HUD and the developer window (Ben, 7 Nov) |

## Tools and the build

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **ROS 2 Jazzy** | Apache-2.0 | runs | everything |
| twist_mux; teleop_twist_joy and teleop_twist_keyboard | Apache-2.0; BSD | runs; ported (its rule) | the command mux and the locks; the joystick and keyboard. twist_mux's selection rule is rewritten in Rust in `aav_mux_state` (who is driving, TODO 247) |
| **rclrs** (ros2-rust) | Apache-2.0 | runs | our Rust ROS nodes |
| **rayon** | MIT / Apache-2.0 | runs | each map chunk loaded and built on its own core (`map-chunks` `Store::update`, TODO 195); our global costmap's 64 m tiles and chunks inflated each on its own core (`aav_costmap` `layered.rs`, TODO 175); the things' background: the 3 x 3 chunks read and the walls grown on many cores (`aav_things_rs` `background.rs`, TODO 195) |
| **proptest** | MIT / Apache-2.0 | tests only | the stop zone's, the map chunks' and the car model's rules checked on thousands of random inputs (`tests/props.rs`, TODO 204) |
| **rstar** (the georust project) | MIT / Apache-2.0 | runs | an R-tree of the Lanelet2 map's centre lines, for the nearest lane (`lanelet-map`, TODO 201) |
| **Rerun** (rerun.io) | MIT / Apache-2.0 | a trial viewer, laptops only | our MCAP recordings opened as they are (`pixi run -e rerun view-bag`, TODO 197) |
| **mcap** (Foxglove) | MIT | examples and tests only | MPPI's `follow_plan` reads a recording's plans (`examples/bag`, TODO 202) |
| **mcap**, **mcap-ros2-support** (Foxglove, Python) | MIT | runs, laptops only | `map-drive` reads a recording without ROS (TODO 250) |
| **rosbag2** (its MCAP storage, `zstd_fast` preset); image_transport's `republish` | Apache-2.0; BSD | runs | the mapping drive's recorder and its JPEG copies of the cameras (`pixi run record-drive`, TODO 249) |
| **chrony** (`chronyc tracking`) | GPL-2.0 | runs, a separate program | the recording kit's clock check on each car computer (TODO 249) |
| **criterion** (Brook Heisler, Jorge Aparicio) | MIT / Apache-2.0 | benchmarks only | MPPI's tick, the planner's legs, the costmap's updates and an NDT match timed with statistics (`benches/`, TODO 203) |
| **tracing**, **tracing-subscriber** (tokio-rs) | MIT | runs, only when built in | the MPPI tick's stages as spans, timed on stderr (`aav_mppi_lag` feature `trace`, TODO 196) |
| **Tracy** (Bartosz Taudul), through **tracing-tracy** | BSD-3-Clause; MIT / Apache-2.0 | runs, only when built in | the same spans, live in the Tracy viewer (feature `tracy`) |
| CPython's `math.hypot` (`vector_norm` in `Modules/mathmodule.c`, R. Hettinger) | PSF-2.0 | idea | `aav_ego_rs`'s `hypot`: the same algorithm, so the Rust ego gives the Python's last bit (TODO 185); `aav_safety_rs`'s `pyfmt::hypot`, so the Rust geofence's distances match `geofence.py`'s to the bit (batch 7) |
| **RoboStack**; **Pixi** | BSD-3 | runs | ROS on Macs and any Linux, without admin rights |
| Embassy; embedded-hal | MIT / Apache-2.0 | runs | the spine's firmware |
| PlatformIO | Apache-2.0 | runs | building the 2025-26 boards' firmware |
| The Elephant-Goldfish Model (EGM) | Apache-2.0 | runs | how we write and review designs |
