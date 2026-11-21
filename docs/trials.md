# Trials: new path followers and localizers beside the main ones

Ben, 29 Oct: "We want to make other nodes or plugins that we can test and then replace
the main ones if they work." Each trial runs **beside** today's default, chosen at launch,
and is **scored on the same routes**. It becomes the default only once it wins.

!!! note "Nav2 and Gazebo are gone"
    Most results (now in [the trials log](history/trials-log.md)) were scored in Gazebo, inside Nav2's chain, before both were deleted
    (Gazebo 16 Nov, Nav2 17 Nov: [Nav2, as it was](history/nav2-as-it-was.md),
    [Gazebo, as it was](history/gazebo-as-it-was.md)). They stay as the record. Today every
    trial runs in Ravensim, inside our stack (`stack:=ours`): the trackers in `mppi_lag`'s
    place, the planners in `aav_planner`'s. Nav2's own planners (Smac 2D, NavFn, Theta*)
    and followers (MPPI, RPP) went with it.

> **The short version**
> - **Trackers** (`aav_track`): pure pursuit, Stanley, LQR, MPC, and `mpc_full` (speed and steering together), in our chain, in `mppi_lag`'s place (in controller_server's, until 17 Nov). `tracker:=lqr`.
> - **AMCL** (ours in Rust, `aav_amcl_rs`, since 18 Nov; stock `nav2_amcl` removed 20 Nov, TODO 232): a particle filter on the map and the lidar, scored against the EKF, never steering. `amcl:=true`. [Ours against stock](history/trials-log.md#18-nov-amcl-in-rust-against-stock).
> - **NDT** (`aav_ndt`, ours in Rust): the lidar matched to a saved lidar map, scored against the EKF, never steering. `ndt:=true`.
> - **Score:** `pixi run measure controllers --trackers pure_pursuit lqr --runs 3`.
> - **Results (29 Oct, Gazebo, 3 runs):** pure pursuit, Stanley and MPC finished every leg; MPPI, RPP and LQR lost legs on the U-turn. AMCL lost itself; the EKF wins. [Results](history/trials-log.md#results-29-october).
> - **NDT**, 29 Oct: about 1-3 cm off on the lot tour, 2 ms a match (below).
> - **NDT into the global EKF** (`ndt_ekf:=true`), 30 Oct: the EKF **17 cm → 1.6 cm** off; with the GPS off, 1.2 m → 1.6 cm. [Results](history/trials-log.md#ndt-into-the-global-ekf-30-oct).
> - **Betsi's measurements** (1 Nov: wheelbase 1.561 m, track 0.75 m, lidar and GPS heights): the runs above used the old guesses. Rerun with hers: `sim-check` 9 of 9, `bench-check` 6 of 6, NDT's lot tour 4 of 4 legs, the EKF 1 cm off; the lattice's moves rebuilt (radius 3.572 m).
> - **1 Nov: the lattice is the default planner** (Ben), and the car reverses again; forward only, Hybrid-A*. [Planners](history/trials-log.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).
> - **Afternoon:** LQR rebuilt with a preview (U-turn 2 of 3 in a quiet run; it weaved on a busy Mac), `mpc_full` 10 of 12. [Results](history/trials-log.md#results-29-october-afternoon-lqr-rebuilt-and-mpc_full).
> - **Planners** (`planner:=`): Nav2's State Lattice (on our own primitives), Smac 2D, NavFn and Theta*, beside Hybrid-A*. 29 Oct, forward only: Hybrid-A* 4 of 4, the lattice 2 of 4. **30 Oct, reversing on, 3-4 runs: every leg for hybrid, lattice and lattice32; the lattice the fastest** (184 s a tour against 213 s). Ben chose the lattice on 1 Nov (above). [Results](history/trials-log.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).
> - **Voxels** (`voxels:=true`, 30 Oct): Nav2 reads the lidar as 0.25 m cubes. It stopped 0.44 m before the post (the scan 0.36 m), drove the lot tour 4 of 4, 6 ms a cloud. [Results](history/trials-log.md#voxels-30-oct-the-lidar-as-cubes).
> - **KISS-ICP** (`aav_kiss_icp`, stock, MIT): lidar odometry, scored against `ekf_local`, never in an EKF yet. `kiss_icp:=true`. 30 Oct, one run: **0.04 m per 100 m** over the lot tour, against ekf_local's 0.33; about 1 ms a scan; once the ground was cut out. [Results](history/trials-log.md#30-oct-kiss-icp-beside-ekf_local-one-run-mppi-the-lot-tour). Design: `docs/designs/2026-10-30_kiss-icp.md`.
> - **KISS-ICP into the local EKF** (`kiss_ekf:=true`), 1 Nov: its twist beside the wheels and IMU. Gazebo, 5 runs: **no clear win yet** (ekf_local 0.30-1.27 m per 100 m with it, 0.44-1.91 without; the spread is the Mac's load). Its covariance is now measured. [Results](history/trials-log.md#kiss-icp-into-the-local-ekf-1-nov).
> - **The diffusion planner** (`planner:=diffusion`, `aav_diffusion_planner`), 2 Nov: a model draws 16 paths, a guard checks them, the lattice's path if none passes or it's late. Gazebo, 2 runs: **every leg**, 178 and 182 s a tour (the lattice 186 and 188 s); **63 % of plans from the model**, 37 % the lattice's (17 % late, 20 % no sample passed). [Results](history/trials-log.md#diffusion-2-nov-a-model-draws-the-lattice-guards).
> - **Diffusion for 0.3**, 7-8 Nov: the guard refused the model mostly for turns (G5) and, at the U-turn's cusp, headings (G6). Each gear smoothed on its own, more U-turn data, the map encoded once: offline **77 → 87 %** of plans with a passing sample, G5 refusals **2,197 → 850**. In the simulator, 3 runs each: **77 → 88 %** of plans from the model (92 % in a fourth), the U-turn **40 → 72 %**, every leg, 171-175 s a tour against the lattice's 180-185 s. Allowed on the car as a gated trial (0.5 m/s). [Results](history/trials-log.md#diffusion-7-8-nov-the-u-turn-the-orin-the-car).
> - **3 Nov, everything for 0.2** (Gazebo, 3 runs each): every switch drives a leg; diffusion every leg, **73 %** of plans from the model; NDT into the EKF **17 → 1.8 cm**, every leg; voxels slower, and once drove through the post. **No default changed.** [Results](history/trials-log.md#3-nov-everything-for-02).
> - **7 Nov, the lag-aware MPC** (`tracker:=mpc_lag`): plans steering and speed through the car's own lags (the shared car model); tells ego to keep its lead off. Pretend car: no speed overshoot (others 14 %), and with the lags doubled the U-turn at 2 m/s off by **0.17 m** (MPC 0.36, LQR 0.27). **Our simulator, 3 runs: 6 of 12 legs** (MPPI 8, RPP 9, the Rust `mppi_lag` 11): best on the legs it drives, but it never made the U-turn by the building: it doesn't read the costmap. [Results](history/trials-log.md#lag-aware-mpc-7-nov-mpc_lag).
> - **7-8 Nov, our own Rust pieces** (each has its page; trials then, the simulator's default since 14 Nov): MPPI with lag (`tracker:=mppi_lag`, [page](mppi-lag.md)), the costmap ([page](own-costmap.md)), the planner ([page](own-planner.md)), the navigator ([page](own-navigator.md)), the velocity smoother ([page](own-smoother.md)); the behaviour planner and the adaptive brake ([how the car stops](stopping.md)). What is the default today: [Switches](switches.md#the-defaults-what-runs-if-you-say-nothing).
> - **14 Nov, the wiggle** (TODO 149): three tests on the campus road. A new plan every second moved Pure Pursuit and `mppi_lag`; the GPS's jitter moves Pure Pursuit; the smoother nothing; MPPI's 6-8 cm is its own. The trees now keep a valid path: RPP's swing **0.12-0.13 → 0.09 m**, `mppi_lag`'s **0.14-0.17 → 0.10 m**, no more loops at the goal. [Results](history/trials-log.md#the-wiggle-14-nov-todo-124-149).
> - **15 Nov, the lane finder as a pose sensor** (`lane_pose:=true`, TODO 155): the painted lines and the lane map correct the global EKF sideways. Our simulator: sideways error **9-10 → 2 cm** RMS on the straight road world, **9.9 → 5.4 cm** on a campus road; jitter a third to a half. The wiggle with it steering: not shown (little paint on that tour). Off by default. [Results](history/trials-log.md#the-lane-finder-as-a-pose-sensor-15-nov-todo-155).
> - The design: `docs/designs/2026-10-29_trials.md`.

## What is on trial

| Trial | Scored against | Switch |
|---|---|---|
| Stanley steering | our pure pursuit | `tracker:=stanley` |
| LQR steering, gains by speed and bend, with a preview | our pure pursuit | `tracker:=lqr` |
| MPC steering, within the steering's limits | our pure pursuit | `tracker:=mpc` |
| MPC speed and steering together (sets its own speed) | MPC, MPPI | `tracker:=mpc_full` |
| MPC through the car's lags (sets its own speed; lag-aware) | MPPI, RPP, MPC | `tracker:=mpc_lag` |
| The same MPC in Rust (`aav_mpc_lag`, 15 Nov; reverses since 20 Nov, TODO 213) | the Python `mpc_lag` | `tracker:=mpc_lag_rs` |
| our pure pursuit (the control) | `mppi_lag` (until 17 Nov, Nav2's RPP and MPPI) | `tracker:=pure_pursuit` |
| AMCL | the global EKF | `amcl:=true` |
| NDT on a saved lidar map | the global EKF, and AMCL | `ndt:=true` (after `pixi run ndt-map drive`) |
| NDT's pose into the global EKF | the global EKF on GPS alone | `ndt_ekf:=true` |
| KISS-ICP lidar odometry (how far the car moved, no map) | `ekf_local` (wheels and IMU) | `kiss_icp:=true` (after `pixi install --locked -e kiss`) |
| KISS-ICP's twist into the local EKF | `ekf_local` on the wheels and IMU alone | `kiss_ekf:=true` |
| State Lattice planner, our primitives | Hybrid-A* | `planner:=own` (Nav2's then, `planner:=lattice`): **won, the default since 1 Nov** (`lattice_headings:=32`: the 32-heading file, still a trial) |
| Smac 2D, NavFn, Theta* planners (Nav2's; ignored the turning circle) | the lattice | gone with Nav2 (17 Nov) |
| Smooth lattice moves: curvature no faster than the steering turns, the smoother off ([page](controllers.md#smooth-moves-a-trial-14-nov)) | Nav2's moves | `lattice_moves:=smooth` (1 m/s) or `smooth_fast` (2.5 m/s), with `planner:=lattice` or `own`; `measure controllers --planners lattice lattice_smooth own own_smooth` |
| The lidar as 0.25 m cubes over the fitted road (`aav_voxels`) | the flat scan (`lidar_scan`) | `voxels:=true` |
| A diffusion model draws the path, a guard checks it, the lattice behind it (`aav_diffusion_planner`; on the car only with `diffusion_trial:=true`, 0.5 m/s) | the lattice | `planner:=diffusion` (after `pixi install --locked -e learn`, `pixi run diffusion data`, `pixi run diffusion train`) |

## Where a tracker sits

```
goal ─► navigator ─► planner (aav_planner: the lattice, replans every 1 s)
              │ FollowPath ("follow_path")
              ▼
   tracker:=mppi_lag (default)      tracker:=stanley | lqr | mpc | mpc_full | mpc_lag | pure_pursuit
   aav_mppi_lag (Rust)              aav_track's track node
              └──────► cmd_vel_nav ◄──────┘
                            │
        velocity_smoother → stop zone → /drivers/nav/cmd_vel (twist_mux 20) → ego
```

Nothing after `cmd_vel_nav` changes: the same smoother, stop zone, mux slot, locks and
e-stops.

## Fair rules

| Rule | Why |
|---|---|
| One speed policy for every tracker: RPP's cruise speed, slower in bends, stop by distance | off-path error grows with speed |
| `mpc_full` plans its own speed, but within the same limits: cruise, bends, planned braking | it is on trial for the speed too |
| One goal and progress check (nav2.yaml: 1.0 m, 0.75 rad) | the same "done" for all |
| Track in `odom`; each path moved there once | the map pose jumps with GPS |
| Forward only (`reversing:=false`) in the runs until 19 Nov | `aav_track`'s trackers drove forwards only then; since 19 Nov they reverse (TODO 174, `gears.py`), scored in stage 2 |
| 3 runs each, mean and spread (`--runs 3`) | MPPI is random; one run names no winner |
| Loop time and missed ticks recorded | a slow MPC mustn't look like bad maths |

## Run it

```bash
pixi run sim world:=test_lot reversing:=false tracker:=mpc      # watch one
pixi run sim world:=test_lot amcl:=true                         # AMCL beside the EKF
pixi run measure controllers --trackers pure_pursuit stanley lqr mpc mpc_full --controllers mppi --runs 3
pixi run measure controllers --amcl                              # score AMCL too
pixi run ndt-map drive                                          # NDT's map, once (~6 min)
pixi run measure controllers --amcl --ndt                        # score NDT beside AMCL
pixi run measure controllers --ndt-ekf --runs 3 --tours lot      # NDT into the global EKF
pixi run sim-check --lidar-obstacle --voxels                     # the cubes stop for a post?
pixi run measure controllers --voxels --tours lot                 # the lot tour on cubes
pixi install --locked -e kiss                                   # KISS-ICP, once
pixi run measure controllers --kiss-icp --tours lot --controllers mppi   # its drift vs ekf_local
pixi run measure controllers --kiss-ekf --tours lot --controllers mppi   # ekf_local with its twist
pixi run sim-check --kiss-ekf                                   # the road world, kiss_ekf:=true
```

## Results, by date

The dated results moved to [the trials log](history/trials-log.md) on 18 Nov, word for word, so this page stays the guide:

| Result |
|---|
| [NDT and AMCL beside the EKF (29 Oct); AMCL in Rust against stock (18 Nov)](history/trials-log.md#results) |
| [NDT into the global EKF (30 Oct)](history/trials-log.md#ndt-into-the-global-ekf-30-oct) |
| [KISS-ICP into the local EKF (1 Nov)](history/trials-log.md#kiss-icp-into-the-local-ekf-1-nov) |
| [Results, 29 October](history/trials-log.md#results-29-october) |
| [Results, 29 October (afternoon): LQR rebuilt, and `mpc_full`](history/trials-log.md#results-29-october-afternoon-lqr-rebuilt-and-mpc_full) |
| [Lag-aware MPC, 7 Nov (`mpc_lag`)](history/trials-log.md#lag-aware-mpc-7-nov-mpc_lag) |
| [Planners, 29 Oct: hybrid, lattice and smac2d](history/trials-log.md#planners-29-oct-hybrid-lattice-and-smac2d) |
| [Planners, 30 Oct: reversing on, hybrid, lattice and lattice32](history/trials-log.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32) |
| [Diffusion, 2 Nov: a model draws, the lattice guards](history/trials-log.md#diffusion-2-nov-a-model-draws-the-lattice-guards) |
| [Diffusion, 7-8 Nov: the U-turn, the Orin, the car](history/trials-log.md#diffusion-7-8-nov-the-u-turn-the-orin-the-car) |
| [Voxels, 30 Oct: the lidar as cubes](history/trials-log.md#voxels-30-oct-the-lidar-as-cubes) |
| [3 Nov: everything, for 0.2](history/trials-log.md#3-nov-everything-for-02) |
| [The wiggle, 14 Nov (TODO 124, 149)](history/trials-log.md#the-wiggle-14-nov-todo-124-149) |
| [The lane finder as a pose sensor, 15 Nov (TODO 155)](history/trials-log.md#the-lane-finder-as-a-pose-sensor-15-nov-todo-155) |

## Scoring a localizer fairly

| Number | Why |
|---|---|
| off: mean, worst; heading worst | each pose against the truth at its own stamp |
| the same on one 10 Hz grid (each pose held until the next) | one that publishes more often isn't scored on more samples |
| time over 1 m off, or no pose yet; the longest stretch | lost, and how long it took to recover |
| NDT: share of clouds that gave a pose; each match's wall-clock time | simulated time can hide a slow match |

## Scoring an odometry fairly (KISS-ICP, ekf_local)

An odometry says only **how far the car moved**, in its own frame. So:

| Number | Why |
|---|---|
| lined up with the truth at its first pose (position and heading) | its frame starts anywhere |
| drift per 100 m driven: the last pose's gap, over the distance | the usual odometry number (KITTI's is a % of distance) |
| worst gap; heading off at the end | a drift that comes back still steered the car wrong |
| per leg, and over the whole tour | short legs flatter it |
| KISS-ICP's match time (wall clock) | simulated time hides a slow match |

Gazebo's lidar has **no sweep** (one instant per turn), so KISS-ICP's deskew does nothing
there; the car's VLP-16 has a `time` field, and its scans are smeared by the car's motion.

## Before a winner goes on the car

- Gazebo's map is perfect and its GPS noise simple: **AMCL feeds the EKF only after it
  also wins on a bag from the real lot.** On the car, slam_toolbox's map needs aligning
  to the GPS `map` frame first.
- AMCL publishes only after the car moves (`update_min_d`): parked, few samples.
- The trackers don't read the costmap: MPPI stays for the obstacle tours (design T3).
