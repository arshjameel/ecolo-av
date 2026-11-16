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
| **Nav2** (S. Macenski and the Nav2 authors; "The Marathon 2: A Navigation System", IROS 2020) | Apache-2.0 | ported, runs | our costmap (`aav_costmap`), planner (`aav_planner`), navigator (`aav_navigator`), smoother (`aav_smoother`) and follower (`aav_mppi_lag`) are Nav2's, in Rust; Nav2 itself runs as the yardstick until 0.4-rc1, the stop zone's decision (`aav_stop_zone`, from `collision_monitor`) |
| ROS 1 `costmap_2d` (Willow Garage) | BSD | ported (through Nav2) | the costmap's layers |
| **Nav2's Costmap Filters** (keepout and speed masks; A. Merzlyakov, S. Macenski) | Apache-2.0 | idea | the lanes layer (`aav_costmap` `lanes.rs`, `lane_costs`, TODO 148): lanes as costs the planner pays, not walls. Ours is drawn from the lane map per route, since a mask can't know which way the car drives |
| **MPPI**: G. Williams, A. Aldrich, E. Theodorou, "Model Predictive Path Integral Control: From Theory to Parallel Computation", J. Guidance, Control, and Dynamics, 2017 | paper | idea | the local planner (`aav_mppi_lag`, and Nav2's) |
| **State Lattice**: M. Pivtoraiko, R. Knepper, A. Kelly, "Differentially Constrained Mobile Robot Motion Planning in State Lattices", J. Field Robotics, 2009 | paper | idea | the path planner (`aav_planner`, Nav2's Smac Lattice) |
| **Hybrid A\***: D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments", IJRR, 2010 | paper | idea, ported (through Nav2) | Nav2's Smac Hybrid; ours, built 14 Nov (TODO 178): Nav2's SmacPlannerHybrid ported into `aav_planner` (`planner_search:=hybrid`), sharing the lattice's search |
| **A\***: P. Hart, N. Nilsson, B. Raphael, IEEE Trans. Systems Science and Cybernetics, 1968 | paper | idea | the planners' search |
| **Dijkstra**: E. W. Dijkstra, "A note on two problems in connexion with graphs", Numerische Mathematik, 1959 | paper | idea | the campus route graph (`aav_routes`) |
| **Reeds-Shepp** curves: J. A. Reeds, L. A. Shepp, Pacific J. Mathematics, 1990 | paper | idea | reversing paths; through OMPL in Nav2's planner |
| **Continuous-curvature paths**: T. Fraichard, A. Scheuer, "From Reeds and Shepp's to Continuous-Curvature Paths", IEEE T-RO, 2004 | paper | idea | the steering-rate lattice moves (TODO 177) |
| OMPL (the Open Motion Planning Library) | BSD-3 | ported (through Nav2) | the curves inside the planner |
| **Pure Pursuit**: R. C. Coulter, CMU-RI-TR-92-01, 1992; **Regulated Pure Pursuit**: S. Macenski et al., Autonomous Robots, 2023 | paper | idea | `aav_track`, Nav2's RPP |
| **Stanley**: S. Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge", J. Field Robotics, 2006 | paper | idea | `aav_track`'s Stanley follower |
| LQR and MPC (textbook control) | - | idea | `aav_track`'s LQR, MPC and MPC with lag; `aav_mpc_lag`, the MPC with lag in Rust |
| **OSQP**'s ADMM: B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd, "OSQP: an operator splitting solver for quadratic programs", Math. Programming Computation, 2020 | Apache-2.0 (the solver; we took the algorithm, not its code) | idea | `aav_track`'s `qp.py` and its Rust port in `aav_mpc_lag` (`qp.rs`, 15 Nov) |
| **Clarabel** (P. Goulart, Y. Chen; Oxford) | Apache-2.0 | idea | read and weighed for `aav_mpc_lag`'s QP (15 Nov); we kept our ADMM for its warm start and the same answers as the Python (`aav_mpc_lag` README) |
| **Flow matching** and the diffusion planner's other ideas | see `aav_diffusion_planner/PROVENANCE.md` | idea | `planner:=diffusion` |

## Where the car is

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **robot_localization** (T. Moore, D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System", IAS-13, 2014) | Apache-2.0 | runs | the two EKFs, `navsat_transform` |
| The Kalman filter: R. E. Kalman, 1960 | paper | idea | the EKFs; the things tracker (`aav_things`) |
| **Complementary filter**: W. T. Higgins, "A Comparison of Complementary and Kalman Filtering", IEEE Trans. Aerospace and Electronic Systems, 1975 | paper | idea | follow in odom, correct slowly (TODO 186): the followers take the fast part of the pose from odometry and the slow part from the map, the map -> odom correction rate-limited (`aav_mppi_lag` `map_correction.rs`, `aav_track` `slow_frame.py`) |
| **NDT**: P. Biber, W. Straßer, IROS 2003; M. Magnusson, PhD thesis, Örebro, 2009 | paper | idea | `aav_ndt` (its `PROVENANCE.md` lists the rest) |
| **KISS-ICP**: I. Vizzo et al., RA-L, 2023 | MIT | runs | `kiss_icp:=true`, scored beside the EKF |
| AMCL (Nav2) | LGPL-2.1 | runs | `amcl:=true`, a trial |
| **Lane markings against a map**: Autoware's YabLoc (camera road markings matched to a vector map); D. Schreiber, B. Switkes, C. Stiller, "LaneLoc: Lane marking based localization using highly accurate maps", IEEE IV 2013 | Apache-2.0; paper | idea | the lane finder as a pose sensor (`aav_lane_pose`, TODO 155): the painted lines and the lane map give the car's place across the lane, a pose sure only sideways for `ekf_global`. Much simpler than either: one offset, no particle filter |
| imu_tools (Madgwick filter) | BSD / GPL-3 | runs | the IMU's orientation |

## Seeing

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **RF-DETR** (Roboflow) | Apache-2.0 | runs, data | the object and stop-sign detector |
| **Ultralytics YOLO** | AGPL-3.0 | runs | the earlier detector, compared and kept as an option |
| **DBSCAN**: M. Ester et al., KDD 1996 | paper | idea, ported | the radar's clusters (`aav_radar_clusters`) |
| OpenCV | Apache-2.0 | runs | lane finding |
| NVIDIA TensorRT; Apple Core ML Tools | NVIDIA licence; BSD-3 | runs | the detector on the Orin and on Macs |

## Maps and routes

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **OpenStreetMap** contributors | ODbL | data | the campus world, its roads and buildings (`pixi run campus`) |
| **OSRM**; **Valhalla** | BSD-2; MIT | runs | street routes (`aav_routes`) |
| Contraction Hierarchies: R. Geisberger et al., WEA 2008 | paper | idea | how OSRM answers fast; noted for the route planner |
| **Lanelet2** (F. Poggenhans et al., ITSC 2018) | BSD-3 | idea | the lane map's shape and its `.osm` format; tried on RoboStack; our own Rust reader, `lanelet-map` (TODO 175) |
| **Autoware** (the Autoware Foundation) | Apache-2.0 | idea | the three map levels, map chunks loaded near the car, its tracker as a reference (TODO 156, 175); its `roi_cluster_fusion` (a lidar cluster's box projected into the camera, matched by IoU) for the things' names (TODO 192) |
| **Apollo** (Baidu) | Apache-2.0 | idea | routing on a lane graph, then planning along the road ahead; its HD map cut into fixed tiles (`map-chunks`) |
| **roxmltree** (Y. Kladov and contributors) | MIT or Apache-2.0 | dependency | reading Lanelet2's `.osm` XML in `lanelet-map` |
| **OctoMap** (A. Hornung et al., *Autonomous Robots*, 2013) | paper | idea | 3D occupancy; why our chunks hold a flat hash of voxels instead of a tree (TODO 175) |
| **Quadtrees** (R. Finkel, J. Bentley, *Acta Informatica*, 1974) | paper | idea | splitting a chunk only where it pays, the fallback in the three-levels design |

## Simulators and screens

| Source | Licence | Kind | Where we used it |
|---|---|---|---|
| **Gazebo** (Open Robotics) | Apache-2.0 | runs, idea | the first simulator, now a yardstick; its world files (SDF) and sensor models are what ours copies the numbers from |
| CARLA | MIT | runs | `pixi run carla-car` |
| **Bevy** | MIT / Apache-2.0 | runs | our simulator's cameras, the developer window |
| **Mesa** (llvmpipe, lavapipe), from conda-forge | MIT | runs | drawing with no GPU (`tools/software_gl.sh`): Gazebo's cameras through OpenGL, ours through Vulkan (lavapipe, 15 Nov) |
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
| twist_mux; teleop_twist_joy and teleop_twist_keyboard | Apache-2.0; BSD | runs | the command mux and the locks; the joystick and keyboard |
| **rclrs** (ros2-rust) | Apache-2.0 | runs | our Rust ROS nodes |
| **rayon** | MIT / Apache-2.0 | runs | each map chunk loaded and built on its own core (`map-chunks` `Store::update`, TODO 195) |
| **proptest** | MIT / Apache-2.0 | tests only | the stop zone's, the map chunks' and the car model's rules checked on thousands of random inputs (`tests/props.rs`, TODO 204) |
| **rstar** (the georust project) | MIT / Apache-2.0 | runs | an R-tree of the Lanelet2 map's centre lines, for the nearest lane (`lanelet-map`, TODO 201) |
| **Rerun** (rerun.io) | MIT / Apache-2.0 | a trial viewer, laptops only | our MCAP recordings opened as they are (`pixi run -e rerun view-bag`, TODO 197) |
| **mcap** (Foxglove) | MIT | examples and tests only | MPPI's `follow_plan` reads a recording's plans (`examples/bag`, TODO 202) |
| **criterion** (Brook Heisler, Jorge Aparicio) | MIT / Apache-2.0 | benchmarks only | MPPI's tick, the planner's legs, the costmap's updates and an NDT match timed with statistics (`benches/`, TODO 203) |
| **tracing**, **tracing-subscriber** (tokio-rs) | MIT | runs, only when built in | the MPPI tick's stages as spans, timed on stderr (`aav_mppi_lag` feature `trace`, TODO 196) |
| **Tracy** (Bartosz Taudul), through **tracing-tracy** | BSD-3-Clause; MIT / Apache-2.0 | runs, only when built in | the same spans, live in the Tracy viewer (feature `tracy`) |
| **RoboStack**; **Pixi** | BSD-3 | runs | ROS on Macs and any Linux, without admin rights |
| Embassy; embedded-hal | MIT / Apache-2.0 | runs | the spine's firmware |
| PlatformIO | Apache-2.0 | runs | building the 2025-26 boards' firmware |
| The Elephant-Goldfish Model (EGM) | Apache-2.0 | runs | how we write and review designs |
