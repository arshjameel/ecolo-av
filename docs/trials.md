# Trials: new path followers and localizers beside the main ones

Ben, 29 Oct: "We want to make other nodes or plugins that we can test and then replace
the main ones if they work." Each trial runs **beside** today's default, chosen at launch,
and is **scored on the same routes**. It becomes the default only once it wins.

> **The short version**
> - **Trackers** (`aav_track`): pure pursuit, Stanley, LQR, MPC, and `mpc_full` (speed and steering together), in Nav2's own chain, in controller_server's place. `tracker:=lqr`.
> - **AMCL** (stock `nav2_amcl`): a particle filter on the map and the lidar, scored against the EKF, never steering. `amcl:=true`.
> - **NDT** (`aav_ndt`, ours in Rust): the lidar matched to a saved lidar map, scored against the EKF, never steering. `ndt:=true`.
> - **Score:** `pixi run measure controllers --trackers pure_pursuit lqr --runs 3`.
> - **Results (29 Oct, Gazebo, 3 runs):** pure pursuit, Stanley and MPC finished every leg; MPPI, RPP and LQR lost legs on the U-turn. AMCL lost itself; the EKF wins. [Results](#results-29-october).
> - **NDT**, 29 Oct: about 1-3 cm off on the lot tour, 2 ms a match (below).
> - **NDT into the global EKF** (`ndt_ekf:=true`), 30 Oct: the EKF **17 cm → 1.6 cm** off; with the GPS off, 1.2 m → 1.6 cm. [Results](#ndt-into-the-global-ekf-30-oct).
> - **Betsi's measurements** (1 Nov: wheelbase 1.561 m, track 0.75 m, lidar and GPS heights): the runs above used the old guesses. Rerun with hers: `sim-check` 9 of 9, `bench-check` 6 of 6, NDT's lot tour 4 of 4 legs, the EKF 1 cm off; the lattice's moves rebuilt (radius 3.572 m).
> - **1 Nov: the lattice is the default planner** (Ben), and the car reverses again; forward only, Hybrid-A*. [Planners](#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).
> - **Afternoon:** LQR rebuilt with a preview (U-turn 2 of 3 in a quiet run; it weaved on a busy Mac), `mpc_full` 10 of 12. [Results](#results-29-october-afternoon-lqr-rebuilt-and-mpc_full).
> - **Planners** (`planner:=`): Nav2's State Lattice (on our own primitives), Smac 2D, NavFn and Theta*, beside Hybrid-A*. 29 Oct, forward only: Hybrid-A* 4 of 4, the lattice 2 of 4. **30 Oct, reversing on, 3-4 runs: every leg for hybrid, lattice and lattice32; the lattice the fastest** (184 s a tour against 213 s). Hybrid stays the default until Ben decides. [Results](#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32).
> - **Voxels** (`voxels:=true`, 30 Oct): Nav2 reads the lidar as 0.25 m cubes. It stopped 0.44 m before the post (the scan 0.36 m), drove the lot tour 4 of 4, 6 ms a cloud. [Results](#voxels-30-oct-the-lidar-as-cubes).
> - **KISS-ICP** (`aav_kiss_icp`, stock, MIT): lidar odometry, scored against `ekf_local`, never in an EKF yet. `kiss_icp:=true`. 30 Oct, one run: **0.04 m per 100 m** over the lot tour, against ekf_local's 0.33; about 1 ms a scan; once the ground was cut out. [Results](#30-oct-kiss-icp-beside-ekf_local-one-run-mppi-the-lot-tour). Design: `docs/designs/2026-10-30_kiss-icp.md`.
> - **KISS-ICP into the local EKF** (`kiss_ekf:=true`), 1 Nov: its twist beside the wheels and IMU. Gazebo, 5 runs: **no clear win yet** (ekf_local 0.30-1.27 m per 100 m with it, 0.44-1.91 without; the spread is the Mac's load). Its covariance is now measured. [Results](#kiss-icp-into-the-local-ekf-1-nov).
> - **The diffusion planner** (`planner:=diffusion`, `aav_diffusion_planner`), 2 Nov: a model draws 16 paths, a guard checks them, the lattice's path if none passes or it's late. Gazebo, 2 runs: **every leg**, 178 and 182 s a tour (the lattice 186 and 188 s); **63 % of plans from the model**, 37 % the lattice's (17 % late, 20 % no sample passed). [Results](#diffusion-2-nov-a-model-draws-the-lattice-guards).
> - **3 Nov, everything for 0.2** (Gazebo, 3 runs each): every switch drives a leg; diffusion every leg, **73 %** of plans from the model; NDT into the EKF **17 → 1.8 cm**, every leg; voxels slower, and once drove through the post. **No default changed.** [Results](#3-nov-everything-for-02).
> - The design: `docs/designs/2026-10-29_trials.md`.

## What is on trial

| Trial | Scored against | Switch |
|---|---|---|
| Stanley steering | our pure pursuit | `tracker:=stanley` |
| LQR steering, gains by speed and bend, with a preview | our pure pursuit | `tracker:=lqr` |
| MPC steering, within the steering's limits | our pure pursuit | `tracker:=mpc` |
| MPC speed and steering together (sets its own speed) | MPC, MPPI | `tracker:=mpc_full` |
| our pure pursuit (the control) | Nav2's RPP and MPPI | `tracker:=pure_pursuit` |
| AMCL | the global EKF | `amcl:=true` |
| NDT on a saved lidar map | the global EKF, and AMCL | `ndt:=true` (after `pixi run ndt-map drive`) |
| NDT's pose into the global EKF | the global EKF on GPS alone | `ndt_ekf:=true` |
| KISS-ICP lidar odometry (how far the car moved, no map) | `ekf_local` (wheels and IMU) | `kiss_icp:=true` (after `pixi install --locked -e kiss`) |
| KISS-ICP's twist into the local EKF | `ekf_local` on the wheels and IMU alone | `kiss_ekf:=true` |
| State Lattice planner, our primitives | Hybrid-A* | `planner:=lattice` (`lattice_headings:=32`: the 32-heading file) |
| Smac 2D, NavFn, Theta* planners (ignore the turning circle: sim and bench only) | Hybrid-A* | `planner:=smac2d`, `navfn`, `theta_star` |
| The lidar as 0.25 m cubes over the fitted road (`aav_voxels`) | the flat scan (`lidar_scan`) | `voxels:=true` |
| A diffusion model draws the path, a guard checks it, the lattice behind it (`aav_diffusion_planner`) | the lattice | `planner:=diffusion` (after `pixi install --locked -e learn`, `pixi run diffusion data`, `pixi run diffusion train`) |

## Where a tracker sits

```
goal ─► bt_navigator ─► planner_server (Smac Hybrid-A*, replans every 1 s)
              │ FollowPath ("follow_path")
              ▼
   tracker:=none (default)          tracker:=stanley | lqr | mpc | mpc_full | pure_pursuit
   controller_server (MPPI, RPP)    aav_track's track node; controller_server's own
                                    action is renamed follow_path_unused
              └──────► cmd_vel_nav ◄──────┘
                            │
        velocity_smoother → collision_monitor → /drivers/nav/cmd_vel (twist_mux 20) → ego
```

Nothing after `cmd_vel_nav` changes: the same smoother, stop zone, mux slot, locks and
e-stops.

## Fair rules

| Rule | Why |
|---|---|
| One speed policy for every tracker: RPP's cruise speed, slower in bends, stop by distance | off-path error grows with speed |
| `mpc_full` plans its own speed, but within the same limits: cruise, bends, planned braking | it is on trial for the speed too |
| Nav2's own goal and progress checks (nav2.yaml: 1.0 m, 0.75 rad) | the same "done" for all |
| Track in `odom`; each path moved there once | the map pose jumps with GPS |
| Forward only (`reversing:=false`: a tracker is refused otherwise) | the trackers drive forwards only |
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

## Results

Each run's results go here, with its date and its folder in `log/controller-compare/`.

### 29 Oct, 04:00: NDT and AMCL beside the EKF (one run, MPPI, the lot tour)

`measure controllers --tours lot --controllers mppi --amcl --ndt`, on the map
`ndt-map drive` made at 03:55 (169,121 points, 2,450 clouds). Off = from Gazebo's truth.

| Leg | EKF off, mean (m) | AMCL off, mean (m) | NDT off, mean / worst (m) | NDT: clouds with a pose | NDT match, mean / worst (ms) |
|---|---|---|---|---|---|
| a left turn | 0.17 | 0.66 | 0.009 / 0.02 | 100 % | 2.6 / 3.9 |
| round the building | 0.16 | 7.1 | 0.030 / 0.27 | 98 % | 1.7 / 4.7 |
| turn back | 0.18 | 17.7 | 0.017 / 0.37 | 98 % | 1.7 / 4.7 |
| home (Nav2 gave up) | 0.17 | 35.7 | 0.010 / 0.21 | 100 % | 2.2 / 14.0 |

- NDT was never over 1 m off; one LOST and restart from `ekf_global` (turn back).
  Heading off at most 1.0°.
- AMCL lost the car round the building and never found it again: a look at its
  settings is due before it is scored again.
- The last leg wasn't reached: MPPI and Nav2 gave up, with the EKF 0.17 m off, so that is
  not the localizers.
- Match times are this Mac's (M4 Max), not the Orin's. Gazebo's lidar has no sweep; the
  car's does. One run: no spread yet. `covariance_scale` (k) and the score floor are
  not fitted yet.

### 29 Oct, afternoon: AMCL tuned, and why it was lost

Each run: `measure controllers --tours lot --controllers mppi --amcl`, with a bag of the
scan, the truth and AMCL beside it (`ros2 bag record`, not in git). Off = from Gazebo's truth.

**What the bags showed:**

| Suspect | Finding |
|---|---|
| things in the world not in the map | **no**: at the truth, 3 of 83,000 beam ends were over 0.75 m from a wall in `example_lot` |
| the scan's 0.3-2.0 m band vs the 1.5 m walls | **no**: the lidar's beams (15° down to 15° up) cross the band on every wall from ~2.4 to 30 m |
| the odometry | **no**: `ekf_local` within 0.3 % of the distance and 0.5° per 4 m |
| the scan's stamp | small: it fits the truth ~0.07 s earlier (7 cm at 1 m/s) |
| the scan's directions | **a bug, fixed**: each point went to the direction below it, 0.25° clockwise |
| AMCL's motion noise (alpha 0.2) | **the main cause**: ~9° of heading noise per particle per 0.25 m driven |
| AMCL's laser model (z_hit 0.5, 60 beams) | **weak**: a perfect fit scored only ~6x a hopeless one |
| the 0.25 m solid map | **a bias**: a beam on a wall's face lands in the free cell before it; pushing beams into the building scored better than the truth |

**What changed** (each with its reason in `config/amcl.yaml`):

| Step | Change | Leg 1 / leg 2 off, mean (m) | Worst (m) |
|---|---|---|---|
| before | the defaults | 0.69 / 6.98 (lost, 95° off) | 21.1 |
| A | alpha 0.2 → 0.01-0.05; z_hit 0.9, z_rand 0.1; 180 beams; resample every 2nd scan; the scan's directions | 0.54 / 0.86 | 1.68 |
| B | A + sigma_hit 0.4 (as wide as the map's cells) | 0.40 / 0.27 | 0.79 (whole tour) |

B, the whole tour, one run (`log/controller-compare/20261029-110651/`):

| Leg | AMCL off, mean (m) | worst (m) |
|---|---|---|
| a left turn | 0.40 | 0.63 |
| round the building | 0.27 | 0.46 |
| turn back | 0.19 | 0.33 |
| home | 0.47 | 0.79 |

- Never lost. The worst is in the open (leg 1, home): only one wall within 30 m, so
  along the car's way nothing pins it, and the solid map's bias pulls it 0.2 m back.
- **AMCL's own map** (step C): the lot at 0.05 m with each obstacle only its outline
  (`maps/example_lot_amcl.yaml`, [why](../ros/drivers/aav_navigation/maps/README.md#why-amcl-has-a-map-of-its-own)).
  Offline, on these bags, its best-scoring pose is 0.07 m from the truth (0.25 m map:
  0.2-0.3 m). Its run: below.

**Step C: AMCL's own map, sigma_hit 0.2** (`log/controller-compare/20261029-213729/`,
one run, `--tours lot --controllers mppi --amcl`):

| Leg | AMCL off, mean (m) | worst (m) | B's mean / worst (m) | Lost? |
|---|---|---|---|---|
| a left turn | 0.11 | 0.15 | 0.40 / 0.63 | no |
| round the building | 0.06 | 0.12 | 0.27 / 0.46 | no |
| turn back | 0.06 | 0.12 | 0.19 / 0.33 | no |
| home | - | - | 0.47 / 0.79 | not scored: the car never moved |

- **C wins: 3-6x closer than B, and it's the default** for `amcl:=true` (sigma_hit 0.2
  in `config/amcl.yaml`; `sim.launch.py` always gives AMCL its own map).
- Heading off: 1° at worst (B: 6°). The first 10 s of leg 1 have no pose yet (AMCL waits
  for 0.25 m of driving), as in B.
- Nav2, not AMCL, failed the last two legs: it gave up on the turn back (status 6,
  collision_monitor stopped it once) and never started home. The Mac was at load ~25 then
  (another worktree's Gazebo). Run it again on a quiet computer for the home leg.
- **A bug found on the way:** until this run AMCL never read its own map. Under
  `sim.launch.py`'s global `use_sim_time`, `amcl.yaml`'s `map_topic: /map` beat the launch
  file's, so the runs at 11:20 and 19:31 (`20261029-112022/`, `-193140/`) were Nav2's 0.25 m
  map with sigma 0.2: 0.5-0.8 m mean, 1.8 m worst, worse than B. That shows 0.4 is right on
  the coarse map, 0.2 only on the fine one. `map_topic` left `amcl.yaml`; the launch test
  checks that nothing the launch file sets is in the file too.
- An earlier try at the same setup (`20261029-213056/`), beside another Gazebo: leg 1 0.08 m
  mean, then AMCL dropped 68 scans (its transforms came late) and published nothing more.
  With the computer this loaded, AMCL's numbers are only worth what its pose count says.

## NDT into the global EKF (30 Oct)

`ndt_ekf:=true`: `ekf_global` takes NDT's x, y and yaw beside the GPS. Off by default.

```
lidar ─► NDT + map ─► /localization/ndt/pose (good fits only) ─┐
GPS ─► navsat_transform ─► /localization/gps/odom ─────────────┼─► ekf_global ─► map → odom
wheels, IMU, GPS heading ──────────────────────────────────────┘    (refuses a pose 5 sigma off)
```

> **The short version**
> - The global EKF's error: **17 cm → 1.6 cm** on average (3 runs each).
> - GPS off: without NDT the EKF drifted to **1.2 m** off; with NDT it stayed at **1.6 cm**.
> - The worst moments stay: NDT slips **up to 0.4 m** along the lane north of the
>   building (only walls parallel to the drive), and the EKF follows it.

### The numbers (Gazebo, the lot tour, MPPI)

Off = the global EKF (`/localization/odom/global`) against Gazebo's truth. Runs:
`log/ndt-ekf/` (not in git): `base` and `ekf2` 3 runs each, `gpsoff-*2` one each.

| Leg | GPS, no NDT: mean / worst (m) | GPS + NDT: mean / worst (m) |
|---|---|---|
| a left turn | 0.17 / 0.50 | **0.008** / 0.02 |
| round the building | 0.18 / 0.58 | **0.031** / 0.25 |
| turn back | 0.17 / 0.45 | **0.013** / 0.38 |
| home | 0.17 / 0.52 | **0.011** / 0.25 |
| **all 12 legs** | **0.17 / 0.58** | **0.016 / 0.38** |

| | GPS, no NDT | GPS + NDT |
|---|---|---|
| Heading, worst (deg) | 13.5 | 2.4 |
| Off Nav2's path, mean (m, by the truth) | 0.11-0.14 | 0.03 (0.10 on one U-turn) |

**GPS off** (stopped after the first fix; `--gps-off`, one run each):

| Leg | No NDT: mean / worst (m) | NDT: mean / worst (m) |
|---|---|---|
| a left turn | 0.25 / 0.35 | **0.007** / 0.02 |
| round the building | 0.47 / 0.70 | **0.030** / 0.24 |
| turn back | 0.67 / 0.74 | **0.017** / 0.39 |
| home | 0.95 / **1.21** | **0.009** / 0.06 |

Without GPS the EKF has only the wheels and the gyro: it drifts, 1.2 m by the end.
With NDT it doesn't drift at all.

- Nav2 gave up on the U-turn ("turn back") in 1 of 3 runs without NDT and 2 of 3 with:
  MPPI's known U-turn trouble ([results](#results-29-october)), not the localizer (the
  EKF was 1-2 cm off then).
- NDT: 97-100 % of clouds gave a pose; no LOST, no restart; 2.0 ms a match (worst 6.3)
  on Ben's Mac (M4 Max), not the Orin.

### NDT's covariance: fitted (NEES)

NDT's spread is k·(−H⁻¹), with a floor. Fitted on the 3 `base` runs (5,852 good fits),
checked on the 3 `ekf2` runs (6,442):

| | Before | Now (`config/ndt.yaml`) | Why |
|---|---|---|---|
| k (`covariance_scale`) | 1 | **7.5** | the good fits' mean NEES is 3 (the honest value for 3 numbers); median 2.4 |
| floor, x and y | (2 cm)² | **(5 mm)²** | 2 cm hid k: a typical fit is 1 cm off |
| floor, yaw | (0.5°)² | **(0.1°)²** | the same |
| NEES on the check runs | - | **2.9** | k fitted again there: 7.2 |

NEES = e′P⁻¹e, e the error from the truth, P the covariance published. Around 3: the
spread matches the errors. `measure controllers --ndt` prints it, and the k that would
make it 3, after every run.

### Keeping bad matches out of the EKF

| Guard | Where | What it caught in Gazebo |
|---|---|---|
| Score per point under **2.3**: not published | `min_score_per_point` (was 2.0) | 4 fits 0.27-0.55 m off (scores 2.06-2.24). Every good fit scored 2.6 or more |
| Out of iterations, or a jump while tracking (over 0.5 m or 5° from the guess): not published | ndt-core; `max_correction_m`, `_deg` (new) | 43 more fits, the two not told apart (the status's message says which) |
| A pose more than 5 sigma from the EKF: refused | `pose0_rejection_threshold: 5` | not counted (robot_localization doesn't say) |
| The late pose put in at its own stamp | `smooth_lagged_data`, 1 s | each pose is 2-6 ms late, plus the lidar's own delay |

**When NDT is LOST** (5 poor fits in a row): it publishes nothing; `ekf_global` goes on
with the GPS (or the wheels and gyro alone, if the GPS is out too) and its doubt grows;
NDT starts again from `ekf_global`'s pose. The first fit after a restart may move far
(no jump check), so the EKF's 5-sigma gate is the guard then. Not seen in these runs:
**untested in Gazebo** (only `ndt-core`'s test).

### What it doesn't show yet

- **The lane north of the building** fools NDT along the lane (0.25-0.4 m, about 2 % of
  fits): only walls parallel to the car. Its spread there grows 4-10 times, not enough
  to cover it. Next: inflate the covariance along a weak direction (the Hessian's
  smallest eigenvalue), or refuse those fits.
- Gazebo's map is perfect, its lidar has no sweep and nothing moves: the score floor and
  k must be fitted again on the car (a bag of the real lot, KISS-ICP's map).
- The geofence holds the car when the GPS stops, even with NDT: GPS-off runs need
  `geofence:=false watchdog:=false`. If NDT wins, the geofence could take it as a
  position source.
- Before it goes on the car: the trials' rule ([below](#before-a-winner-goes-on-the-car)).

```bash
pixi run ndt-map drive                                     # the map, once
pixi run measure controllers --ndt --runs 3 --tours lot      # NDT beside: the EKF on GPS alone
pixi run measure controllers --ndt-ekf --runs 3 --tours lot  # NDT into the EKF
pixi run measure controllers --ndt-ekf --gps-off --tours lot # the GPS stops after its first fix
```

### 30 Oct: KISS-ICP beside ekf_local (one run, MPPI, the lot tour)

`pixi run measure controllers --kiss-icp --tours lot --controllers mppi`, headless, on Ben's
Mac (Apple silicon), another Gazebo run just finished. Folders `log/kiss-run1/` and
`log/kiss-run2/` (not in git).

**First try: the ground left in.** KISS-ICP read **12 m for 21 m** driven and 65° for an 85°
turn. Gazebo's ground is perfectly flat, so the lidar's 8 downward beams draw the same
circles round the car every scan: half the points, all saying "you didn't move".
**Fix:** points below 0.3 m above `base_link` left out (`min_height`), as NDT does.

**With the ground out**, the whole tour (178 m, 4 legs, all reached):

| | KISS-ICP | ekf_local (wheels + IMU) |
|---|---:|---:|
| Drift (m per 100 m driven) | **0.04** | 0.33 |
| Worst off (m, lined up at the start) | **0.11** | 0.59 |
| Heading off at the end (deg) | **0.63** | 1.38 |
| Match time (ms, wall clock, mean / worst) | 1.0-1.4 / 2.8 | - |
| Clouds dropped | 0 | - |

Leg by leg, drift per 100 m (KISS-ICP / ekf_local): a left turn 0.23 / 0.13; round the
building 0.21 / 0.18; turn back 0.08 / 0.21; home 0.23 / 1.65.

**What it shows**

- In Gazebo, KISS-ICP drifts **less than the wheels and IMU** over the tour, and its heading
  holds better. On a short leg the two are about level.
- It is cheap: about 1 ms a scan on the Mac, at 4 threads. The Orin is untested.
- **Gazebo flatters it**: no sweep to deskew, no people or cars moving, walls everywhere.
  The car's VLP-16 smears each scan by the car's motion (deskew on, from its `time`
  field); an open lot has fewer walls.
- One run only. Next: 3 runs, then a bag from the real lot, before its twist goes into
  `ekf_local`.

## KISS-ICP into the local EKF (1 Nov)

`kiss_ekf:=true`: `ekf_local` takes KISS-ICP's **twist** (vx, vyaw) beside the wheels
and the IMU. Off by default. Never its pose: it drifts, in its own frame.

```
lidar ─► KISS-ICP ─► /localization/kiss_icp/odom (twist) ─┐
wheels ─► /vehicle/odom ──────────────────────────────────┼─► ekf_local ─► odom → base_link
IMU ─► /sensors/imu/data ─────────────────────────────────┘   (refuses a twist 5 sigma off)
```

> **The short version**
> - **No clear win in Gazebo yet.** With it, ekf_local drifted 0.30-1.27 m per 100 m;
>   without, 0.44-1.91. The Mac's load moves the numbers more than the switch does.
> - **Its covariance was wrong, now measured**: the speed 3x too sure, the turn rate 3x too
>   doubtful. Too sure, it made ekf_local's speed 10x noisier.
> - **KISS-ICP stopping is safe**: killed mid-drive, ekf_local kept 30 Hz and the right speed.

### The covariance, against Gazebo's truth

Each KISS-ICP step against the truth over the same two stamps (5 runs, ~9,000 steps).

| | Its noise (1 sigma) | It said (30 Oct) | It says now |
|---|---:|---:|---:|
| Speed | **0.14-0.16 m/s** | 0.05 | 0.15 (`twist_linear_sd`) |
| Turn rate | **0.005-0.007 rad/s** | 0.02 | 0.01 (`twist_angular_sd`) |
| (the wheels, Gazebo) | 0.002 m/s, 0.003 rad/s | | |
| (the IMU) | 0.02 rad/s | | |

Why the speed is noisy while the pose holds: each scan's position wobbles about 1.5 cm,
and the next scan takes it back. Over 0.1 s, 1.5 cm is 0.15 m/s.

| ekf_local's speed vs the truth | sd (m/s) |
|---|---:|
| without KISS-ICP | 0.003 |
| with it, said 0.05 (too sure) | **0.035** |
| with it, says 0.15 | 0.008-0.009 |

### The drift (the lot tour, MPPI, one run each)

`measure controllers --kiss-icp` (off) and `--kiss-ekf` (on), headless on Ben's Mac, other
Gazebo and test runs beside it. Folders `log/kiss-ekf/` (not in git). MPPI lost the turn
back in every run, so "driven" differs.

| Run | ekf_local: drift (m per 100 m) | worst off (m) | heading at the end | driven | KISS-ICP alone |
|---|---:|---:|---:|---:|---:|
| off | 1.91 | 1.88 | 0.29° | ~90 m | 0.02 |
| off2 | 0.44 | 1.77 | 1.66° | 222 m | 0.04 |
| on (covariance 0.05, too sure) | 0.93 | 1.22 | 2.69° | 91 m | 0.02 |
| on2 (0.15) | 1.27 | 1.28 | 2.92° | 83 m | 0.03 |
| on3 (0.15, and lagged twists put back in time) | **0.30** | **0.46** | 0.80° | 109 m | 0.03 |

The first two legs, run by run (m per 100 m): off 1.19, 1.15; off2 1.76, 0.61; on2 0.48,
1.14; on3 0.51, 0.65. Mean: **1.18 off, 0.70 on**, but the spread overlaps.

### What it shows

- KISS-ICP **alone** holds 0.02-0.04 m per 100 m; `ekf_local` with it doesn't come
  close. Its turn rate (10 Hz) is outvoted by the IMU (50 Hz): ekf_local's turn-rate
  noise stayed 0.013 rad/s in every run. Next: tune its process noise.
- In Gazebo the wheels are nearly perfect, so there is little for the lidar to fix. The
  point is the real car, where wheels slip: score it on a bag from the lot.
- Lag: the twist arrives 17 ms after its cloud's stamp (median, Gazebo). `ekf_local`
  puts each in at its own time (`smooth_lagged_data`, 1 s of history), as `ekf_global`
  does with NDT; the car's VLP-16 may be later.
- Dropout (KISS-ICP killed mid-drive, test lot, 1 m/s): ekf_local stayed at 30 Hz,
  1.000 m/s against the truth's 1.000, for the next 10 s.
- `pixi run sim-check --kiss-ekf` (road world): all PASS; KISS-ICP 10 Hz, ekf_local 30 Hz,
  its speed 0.005 m/s from the truth (median).

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

## Results, 29 October

Gazebo's test lot, 4 legs, 3 runs per follower, forward only, on branch `thursday-29oct`
(6bfd0021), one after another on a Mac. Run `log/controller-compare/20261029-024252/` (not
in git). **Every car number is still a guess** (`docs/measure-the-car.md`), so this ranks
the followers in the sim, not on the car.

| Follower | Legs finished (of 12) | Off the path, mean | Tick, worst | CPU |
|---|---|---|---|---|
| **pure pursuit** (ours, the control) | **12** | 0.11-0.13 m | 0.8 ms | 21 % |
| **Stanley** | **12** | 0.10-0.13 m | 0.8 ms | 21 % |
| **MPC** | **12** | 0.11-0.12 m | 6.3 ms | 28 % |
| RPP (Nav2) | 10 | 0.11-0.14 m | - | 7 % |
| MPPI (Nav2, today's default) | 8 | 0.11-0.14 m | - | 22-30 % |
| LQR | 8 | 0.11-0.12 m | 0.6 ms | 21 % |

**What it shows:**

- On the straight and the easy bends, every follower stays about 0.12 m off the path: steering law isn't what matters there.
- **The U-turn** ("turn back", a 14 m lane, forward only) decides it. MPPI, RPP and LQR got collision_monitor stops near the walls and Nav2 gave up (status 6); the failed U-turn then left the next leg ("home") unreachable, so each U-turn failure counts twice.
- LQR is linearised about driving straight: at full lock it swings wide. A curvature-scheduled LQR or a tighter feed-forward is the next try.
- MPC never missed a 20 Hz tick (2 ms mean, 6 ms worst) in Python.
- `aav_track`'s 21 % CPU is mostly rclpy's executor and TF, not the trackers (0.2 ms a tick).

**AMCL** (`--amcl`, one MPPI run): 0.71 m mean off on the first leg (EKF: 0.16 m), then lost
on the second: 6.9 m mean, 21 m worst. Gazebo's GPS is easy (white noise), so the EKF has the
advantage here; AMCL's settings are untuned. Run `log/controller-compare/20261029-034402/`.

**Next:** tune LQR for the U-turn; AMCL's laser model and particle count; NDT beside both
(`docs/designs/2026-10-29_ndt.md`); then the same table on the car once it's measured.

## Results, 29 October (afternoon): LQR rebuilt, and `mpc_full`

Branch `trackers-tuning`. Two changes, then the same lot, 3 runs each, forward only.

| Change | What | Commit |
|---|---|---|
| **LQR** | linearised about the path's bend (gains by speed **and** bend), the input is the command's change per tick (so the steering rate is in the design), and a 2 s **preview** of the bends ahead (LQR with preview, Tomizuka) | 8b4dcf23 |
| **`mpc_full`** | plans speed **and** steering over the next 9 m, as one QP a tick, within full lock, the steering rate, the throttle, the brake and the policy's speed limits; it sets its own speed | 2bede5f6 |

**Why LQR swung wide.** The steering needs 1.7 s to go from one lock to the other. The
old LQR only had the bend's steering at the car as feed-forward, so it began each swing
when the bend had already arrived. On the fake car's U-turn at full lock:

| Fake car, U-turn at full lock, 1 m/s | Worst off the path |
|---|---|
| LQR before | 0.80 m |
| LQR with preview | **0.29 m** |
| MPC / pure pursuit | 0.31 / 0.24 m |
| `mpc_full` | **0.14 m** |

### Gazebo, all six (run `log/controller-compare/20261029-152311/`, not in git)

15:23-16:28. **The Mac was busy** (load average 24-30; other helpers' and Ben's runs):
Stanley's runs had sensor-watchdog stops, and LQR's run 1 had 125 planner-loop misses
(the others 1-13).

| Follower | Legs (of 12) | U-turns (of 3) | U-turn time | Tick mean / worst | CPU |
|---|---|---|---|---|---|
| pure pursuit | **12** | 3 | 42 s | 0.2 / 3 ms | 23 % |
| Stanley | **12** | 3 | 41 s | 0.3 / 24 ms | 23 % |
| MPC | **12** | 3 | 48 s | 3.5 / 16 ms | 32 % |
| `mpc_full` | 10 | 2 | 49 s | 2.6 / 13 ms | 30 % |
| MPPI (Nav2) | 10 | 2 | 80 s | - | 18 % |
| LQR | 6 | 1 | 45 s | 0.35 / 3 ms | 26 % |

Off the path, mean: 0.08-0.16 m for every follower.

### LQR and pure pursuit alone (run `log/controller-compare/20261029-041845/`)

04:18, load average 17-21 at the start. **LQR 10 of 12 legs, the U-turn 2 of 3** (was 8
and 1 of 3 at 02:42); pure pursuit 12 of 12 (its U-turn 3 of 3). LQR's failed
U-turn was the planner's clockwise loop, to the south wall.

### What it shows

- **The preview fixed what it was for**: on the fake car, and in the quiet run. In the
  busy run LQR lost legs it used to make: run 1 was starved, run 2 **weaved** (heading
  swinging ±0.5 rad round the building). The fake car with the car seen 0.2 s late
  doesn't weave (a new test), and the design keeps 15 ticks of delay margin at 1 m/s. So
  the cause is still open: see the questions.
- **`mpc_full` drives the whole A-to-B**: the same times as MPC, with its own speed. At
  1 m/s it hardly slows for the U-turn (0.92 m/s at least): over distance, at that speed,
  the steering can swing in time, so its model sees no gain in going slower. At 2 m/s on
  the fake car it slows to 1.4 m/s to swing the steering (the bend alone allows 1.7).
- Every U-turn, any follower, drives 2-5 m wider than the planner's first loop: the planner
  replans every second from where the car is, and each new loop starts a little further
  on. A planner setting, not a tracker one (open).
- MPC and `mpc_full` stay inside a 20 Hz tick in Python (worst 16 ms on a busy Mac).

**Open questions**

1. LQR's weave in Gazebo: rerun on a quiet Mac, then log `/drivers/nav/track/status`
   (its preview and feed-forward) to see if the path's jagged curvature drives it.
2. `mpc_full` slows only when its model says the steering can't keep up. Should it also
   value slowing for the U-turn at 1 m/s (a cost on the steering's rate per second)?
3. The planner's growing loops: replan less often in a U-turn, or keep the old path?

## Planners, 29 Oct: hybrid, lattice and smac2d

What each planner is: [Controllers, Planners](controllers.md#planners).

The run: Gazebo's test lot, the lot tour (4 legs), **one run each**, MPPI following,
forward only, 1 m/s. Branch `planner-switch` (74544111), 23:13, a quiet Mac (load 4).
Before each leg the planner alone was asked for the path, to time it and measure its
turns. `pixi run measure controllers --planners hybrid lattice smac2d --tours lot`;
results in `~/aav-runs/2026-10-29/planners-results/` on Ben's Mac (not in git).

### The whole tour

| | hybrid (default) | lattice | smac2d |
|---|---:|---:|---:|
| Legs finished | **4 of 4** | 2 of 4 | 2 of 4 |
| Time, all legs | 230 s | 141 s | 348 s |
| Planning time, mean / worst | 16 / 52 ms | 4 / 10 ms | 2 / 4 ms |
| Tightest turn in a plan (radius) | 3.03 m | 3.58 m | **2.65 m** |
| Plan tighter than full lock (2.87 m) | 0 m | 0 m | **0.75 m** |
| Off the path, mean | 0.12 m | 0.15 m | 0.22 m |

### Leg by leg

| Leg | hybrid | lattice | smac2d |
|---|---|---|---|
| a left turn | 30 s | 26 s | 27 s |
| round the building | 52 s | 46 s | 46 s |
| **turn back** (a U-turn in a 14 m lane) | 79 s, a 63 m loop | **lost**: its loop took the car into the north-east corner (45, 45), and from there it found no forward path | **lost**: planned 14 m straight back, as if the car could spin; MPPI looped, collision_monitor stopped it, cancelled at 116 s |
| home | 69 s | no path from the corner | its plan had 0.75 m tighter than full lock; the car never moved |

### What it shows

- **Hybrid-A* stays the default.** It was the only one to finish the U-turn.
- **The lattice is fast and drives well** where there's room: 4 ms a plan, and the
  smallest error on the first two legs. Every turn kept to 3.58 m, as made. But with
  only 16 headings its U-turn loop needs more room than Hybrid's 72; it ran the car into
  a corner it couldn't leave forwards. Done 30 Oct (below): with `--reversing` it backs
  out and finishes; 32 headings didn't help.
- **Smac 2D is the fastest planner and the worst for a car**: it ignores the goal's
  heading and the turning circle, so a U-turn is a straight line back. This is why the
  car refuses it.
- The leg after a lost U-turn starts from wherever the car stopped, so one failure costs
  two legs (as with the trackers).
- Planning times are the planner's own `planning_time`, on Gazebo's clock; the wall-clock
  call was within a few ms of it. One run each: **not a ranking yet**. `navfn` and
  `theta_star` weren't run.
- **Unfair to the lattice:** the run was forward only, and the sim's default (and Ben's,
  30 Oct: "Reversing is always true i think") is `reversing:=true`. Rerun below.

## Planners, 30 Oct: reversing on, hybrid, lattice and lattice32

The 29 Oct run again, fairly: **reversing on**, MPPI following (it follows backwards
too; our trackers can't), 1 m/s, the lot tour. `lattice32` is the lattice on its new
32-heading file (`lattice_headings:=32`). Branch `lattice-rerun`.

```bash
AAV_SIM_DOMAIN=224 pixi run measure controllers --planners lattice32 lattice hybrid \
    --reversing --tours lot --runs 3
```

Then one more run of each with every point of the plan kept (the cusp scoring, below).
Results in `~/aav-runs/2026-10-30/lattice-rerun*/` on Ben's Mac (not in git).

### The whole tour

Each planner's runs: hybrid 4, lattice 4, lattice32 3 (one more was spoiled, below).

| | hybrid (default) | lattice (16) | lattice32 |
|---|---:|---:|---:|
| Legs finished | 16 of 16 | 16 of 16 | 12 of 12 |
| Time, all legs: mean (each run) | 213 s (211-215) | **184 s** (182-185) | 197 s (187-216) |
| Driven in reverse, a tour | 9.1 m | 9.3 m | 9.0 m (16.8 once) |
| Planning time, mean / worst | 15 / 55 ms | **10 / 33 ms** | 18 / 68 ms |
| Tightest turn in a plan (radius) | 3.28 m | 3.59 m | 3.47 m |
| Plan tighter than full lock (2.87 m) | 0 m | 0 m | 0 m |
| Changes between forward and reverse (plans, a tour) | 2 | 4 | 4 |
| Off the path, mean | 0.11-0.13 m | 0.13-0.14 m | 0.13-0.24 m |

### Leg by leg (the range over the runs)

| Leg | hybrid | lattice | lattice32 |
|---|---|---|---|
| a left turn | 28-29 s | 26-27 s | 26-27 s |
| round the building | 53 s | 44-46 s | 45 s |
| **turn back** (the U-turn) | 64-67 s, backs up about 9 m | 56-60 s, backs up about 9 m | 56-59 s, backs up about 9 m |
| home | 65-66 s | 55-56 s | 57-63 s; **86 s** once (collision_monitor stopped it, it backed up 8 m more) |

### What it shows

- **With reversing on, the lattice finishes every leg**, and it is the fastest: about
  **29 s quicker** a tour than Hybrid-A* (14 %), with plans a third quicker. On 29 Oct
  (forward only) it lost the U-turn; backing up 9 m is what it needed.
- **32 headings don't help.** Same legs, slower plans, more spread. The 16-heading file
  stays the lattice's default.
- **1 Nov: the lattice is the default** (Ben), and the car reverses again (Ben: "reverse
  and uturn should be in"). Forward only, Hybrid-A* (`vehicle.default_planner`).
- Before, on 30 Oct: Hybrid-A* stayed the default, as the trials rule is that Ben decides a swap.
  The lattice has won this one test, in Gazebo. Before a swap: the bench, and a look at
  forward only on the car (its `reversing:=false` until H5), where the lattice still loses
  the U-turn.
- Every planner turned the U-turn the same way: forward, about 9 m back, forward.

### Notes

- **One lattice32 run was spoiled**: its car never moved. Another helper's ROS nodes on
  the same domain (226) told output_mux the actuators were REAL, so it stopped the car.
  Rerun on domain 221: 4 of 4, 187 s. It is left out of the table.
- **Cusps:** a reversing plan doubles back where the car changes gear. The first scoring
  read that corner as a 0.3-0.5 m "turn", so every planner looked as if it broke full
  lock. The scoring now cuts the plan at each cusp (a turn of more than 120°) and
  measures the turns between them. The tightest-turn row comes from the runs that kept
  every point of the plan (one each); the other runs kept one point in four.

## Diffusion, 2 Nov: a model draws, the lattice guards

The design: `docs/designs/2026-11-02_diffusion-planner.md` (its "As built" says what
changed). A small flow-matching model (3.9 M weights) copies the lattice and Hybrid-A*;
for each plan it draws 16 paths in 10 steps, the guard (G1-G8) checks each, and the
cheapest that passes within 150 ms is the plan. Else the lattice's, asked at the same time.
Branch `diffusion-build`.

```bash
pixi install --locked -e learn
pixi run diffusion data       # 45,092 paths on 791 maps, about 30 min (6 workers)
pixi run diffusion train      # 30 epochs, 19.5 min on the M4 Max (MPS)
AAV_SIM_DOMAIN=215 pixi run measure controllers --planners lattice diffusion diffusion_nomodel \
    --reversing --tours lot
```

Results in `log/controller-compare/20261102-213622/` and `.../20261102-215334/` (not in
git). One run each, twice; MPPI follows, reversing on, 1 m/s.

### Offline: the held-back maps (300 plans, 16 samples each)

| | 10 steps (the node's) | 5 steps |
|---|---:|---:|
| Samples that pass every check | **34.5 %** | 23.9 % |
| Plans with at least one that passes | **75 %** | 60 % |
| The model's time a plan (Mac CPU, ONNX Runtime) | 75 ms | 44 ms |
| Refusals (a sample may fail several): G5 turns, G4 collisions, G6 heading, G8 length | 2,220, 1,340, 899, 58 | 3,016, 1,334, 924, 52 |
| A passing path's length against the teacher's | 1.00 | 1.00 |

The design's target was 80 % of samples. Without `window.smooth` (its points evened, its
headings from the travel), 0.9 %.

### The whole tour (Gazebo)

| | lattice | diffusion | diffusion, no model |
|---|---:|---:|---:|
| Legs finished | 4/4, 4/4 | **4/4, 4/4** | 4/4 |
| Time, all legs | 186 s, 188 s | **178 s, 182 s** | 187 s |
| Driven in reverse | 9.5, 9.2 m | 8.2, 10.5 m | 9.6 m |
| Planning time, mean / worst (the planner alone, once a leg) | 11 / 38, 9 / 29 ms | 133 / 171, 140 / 161 ms | 14 / 42 ms |
| Plan tighter than full lock | 0 m | 0 m | 0 m |
| Off the path, mean | 0.15, 0.16 m | 0.14, 0.13 m | 0.13 m |

### The guard, plan by plan (both runs)

| | Run 1 | Run 2 | Both |
|---|---:|---:|---:|
| Plans | 135 | 132 | 267 |
| **From the model** | 92 (68 %) | 75 (57 %) | **167 (63 %)** |
| The lattice's: late (model and guard over 150 ms) | 18 | 28 | 46 (17 %) |
| The lattice's: no sample passed | 25 | 29 | 54 (20 %) |
| Samples that passed | 1,269 of 1,984 (64 %) | 1,168 of 1,984 (59 %) | 61 % |
| Refused by G5 (turns) / G6 (heading) / G4 (collisions) / G8 (length) | 630 / 220 / 61 / 2 | 722 / 329 / 56 / 8 | |
| Model and guard, mean | 126-133 ms | 134-145 ms | |

With no model (`diffusion_nomodel`): 156 of 156 plans the lattice's, every leg, 187 s.

### Leg by leg: plans from the model

| Leg | Run 1 | Run 2 |
|---|---:|---:|
| a left turn | 12 of 18 | 12 of 17 |
| round the building | 25 of 34 | 20 of 33 |
| **turn back** (the U-turn, reversing) | **18 of 39** | **13 of 40** |
| home | 37 of 44 | 30 of 42 |

### What it shows

- **It works, and the guard does its job.** Every leg in both runs; never a path tighter
  than full lock; the stack below it (MPPI, collision_monitor) unchanged. Where the model
  had nothing good, the car got the lattice's path and never noticed.
- **It is not a winner yet.** 63 % of plans from the model, against the design's 90 %.
  The **U-turn** is its weak leg (under half): G5 and G6, turns and headings round the
  reversing cusp. And it is **slow on a Mac beside Gazebo**: 130-145 ms of a 150 ms stop,
  so 17 % of plans were late.
- **178-182 s against 186-188 s** is 4-8 s quicker, but two runs each name no winner, and
  the turn back leg, where the model drew least, gave most of it. It copies the lattice:
  at best it equals it (the design's honest note).
- **Next** (design steps 4-5): train on the reversing cusps (more U-turn-shaped data),
  guidance away from obstacles, fewer steps on the Orin's GPU (TensorRT), then re-rank by
  how fast MPPI drove them.

## Voxels, 30 Oct: the lidar as cubes

`voxels:=true` (`aav_voxels`, [design](designs/2026-10-30_voxels.md)): Nav2's costmap
and stop zone read **0.25 m cubes** over the fitted road instead of the flat scan.
Gazebo on Ben's Mac, one run each, MPPI.

### Does it stop for the post? (`sim-check --lidar-obstacle`, radar off)

A person-sized post (0.5 × 0.5 × 1.8 m) 15 m ahead; the planner blind to it.

| | Flat scan (today) | Cubes |
|---|---|---|
| Stopped | ✅ | ✅ |
| Front bumper to the post | **0.36 m** | **0.44 m** |
| Top speed | 0.96 m/s | 0.97 m/s |
| Local costmap there | 100 of 100 | 100 of 100 |
| Post taken away: goal reached | ✅ | ✅ |

### What it costs (the node's own status, every cloud)

| | Flat scan | Cubes |
|---|---|---|
| One Gazebo cloud cut (the same cloud, both ways, in the check) | **0.18 ms** | **5.3 ms** |
| The node, read to publish (mean / 95th / worst) | not measured | **6.1 / 6.3 / 6.6 ms** (road world), **6.2 / 6.7 / 12.7 ms** (lot tour) |
| Clouds dropped | - | 0 |
| Road fitted | - | every cloud (Gazebo has no suspension: flat) |

Cubes per cloud: **4** in the road world (the stop sign), **15** with the post (45 at
most), **1,067-1,378** on the lot tour (walls, the building, parked cars).

### The lot tour (`measure controllers --tours lot --controllers mppi`, with and without `--voxels`)

| Leg | Flat scan | Cubes |
|---|---|---|
| a left turn | ✅ 29 s, 21.4 m | ✅ 29 s, 21.3 m |
| round the building | ✅ 52 s, 40.0 m | ✅ 53 s, 41.0 m |
| turn back (the U-turn) | ✅ 70 s, 55.5 m | ✅ 76 s, 60.9 m |
| home | ✅ 67 s, 52.6 m | ✅ 68 s, 53.1 m |
| Off the path, mean | 0.08-0.12 m | 0.10-0.11 m |
| Stops by collision_monitor | 0 | 0 |

Runs: `/tmp/voxbench/runs/` on Ben's Mac (not in git).

### What it shows

- **Cubes stop the car as well as the scan**: 0.44 m from the post against 0.36 m.
- **They cost more**, 6 ms a cloud against well under 1 ms, but that is 6 % of the
  100 ms between clouds: Python is enough. Rust only if the Orin is over 20 ms.
- **The lot tour drives the same**: all four legs; the U-turn took 6 s longer (one run:
  not a ranking).
- **Not shown yet**: the leaning car (Gazebo can't lean; the made-up lot in
  `test_cubes.py` does, 2-3°), a thin pole in the stop zone, backing up
  (`--rear-obstacle`), `--plan-around`, and the car itself.
- **Next**: Nav2's 3D `VoxelLayer` on the same cubes, which would keep a low box near
  the car marked (the 2D layer clears it, as the scan does).

## 3 Nov: everything, for 0.2

Ben, 3 Nov: "I would like to see diffusion tested in simulation. Actually everything."
Gazebo on Ben's Mac (M4 Max, 16 cores), headless, one Gazebo at a time. The load beside
each result is `uptime`'s 1-minute load at its start (other helpers were building).
Runs: `log/sim-all/` in the `sim-all` worktree (not in git).

> **The short version**
> - `sim-check` **9 of 9**, `bench-check` **11 of 11**.
> - **Every switch starts and drives a leg**: 23 of 23 on the lot (two needed a second try, on a busy Mac), 7 more in `sim-check`.
> - **Diffusion against the lattice**, 3 runs each: every leg (12 of 12, and 12 of 12);
>   **73 % of plans from the model** (2 Nov: 63 %); the U-turn 52 s against 56 s.
>   One slow home leg (149 s) makes its tour time spread wide. Still a trial.
> - **NDT into the global EKF**: the EKF **17 cm → 1.8 cm** off, the car **0.13 → 0.05 m**
>   off its path (2 of 3 runs), every leg. A clear win, but it stays off: it needs a map file.
> - **Voxels**: every leg, but **16 s slower** a tour, and **1 of 3 post runs drove
>   through the post**. Not a winner; a bug to find first.
> - `sim-check`'s other runs (obstacle, rear, plan-around, ghost, lane, spine, faults) all pass;
>   `--lane` only after a fix to the check itself. `--drive-to-point`: 1 of 9 drives ended
>   0.61 m past B by its own EKF (limit 0.5), 0.29-0.44 m by the truth.
> - **No default changed.** [Why](#the-defaults-what-to-do).

### Every switch, one leg

The first leg of the lot tour (a left turn, 22 m): `measure controllers --tours lot --legs 1`,
with the switch. MPPI unless the switch is a follower; reversing on unless forward only.

| Switch | Started? | Leg finished? | Time, off the path | Load | Notes |
|---|---|---|---|---|---|
| `planner:=lattice` (default) | ✅ | ✅ | 26 s, 0.17 m | 6-24 | |
| `planner:=hybrid` | ✅ | ✅ | 29 s, 0.09 m | 6-24 | reversing on |
| `planner:=hybrid reversing:=false` | ✅ | ✅ | 29 s, 0.14 m | 24 | the car's forward-only pick |
| `planner:=lattice lattice_headings:=32` | ✅ | ✅ (2nd try) | 26 s, 0.11 m | 10 | 1st try, load 24: the sensor watchdog stopped Nav2 (odom and IMU stale 0.9 s) |
| `planner:=smac2d` | ✅ (2nd try) | ✅ | 27 s, 0.10 m | 14 | 1st try, load 24: never came up (no `map` frame: navsat_transform got no fix) |
| `planner:=navfn` | ✅ | ✅ | 28 s, 0.16 m | 6-24 | first time in Gazebo; its tightest turn 128 m (it ignores the turning circle) |
| `planner:=theta_star` | ✅ | ✅ | 29 s, 0.18 m | 6-24 | first time in Gazebo; tightest turn 5.1 m |
| `planner:=diffusion` | ✅ | ✅ | 30 s, 0.08 m | 6-24 | 6 of 28 plans from the model: the Mac was busy (model and guard 180 ms) |
| `planner:=diffusion`, no model file | ✅ | ✅ | 27 s, 0.26 m | 6-24 | 25 of 25 plans the lattice's: the guard's fallback works |
| `controller:=mppi` (default) | ✅ | ✅ | 26 s, 0.17 m | 6-24 | |
| `controller:=rpp` | ✅ | ✅ | 26 s, 0.09 m | 21 | |
| `tracker:=pure_pursuit` | ✅ | ✅ | 25 s, 0.14 m | 17-27 | forward only, as all trackers |
| `tracker:=stanley` | ✅ | ✅ | 24 s, 0.13 m | 17-27 | |
| `tracker:=lqr` | ✅ | ✅ | 23 s, 0.11 m | 17-27 | |
| `tracker:=mpc` | ✅ | ✅ | 23 s, 0.11 m | 17-27 | |
| `tracker:=mpc_full` | ✅ | ✅ | 25 s, 0.18 m | 17-27 | |
| `amcl:=true` | ✅ | ✅ | 27 s, 0.17 m | 27 | AMCL 0.09 m off (worst 0.14) |
| `ndt:=true` | ✅ | ✅ | 27 s, 0.10 m | 25 | NDT 1.1 cm off (worst 2 cm) |
| `ndt_ekf:=true` | ✅ | ✅ | 26 s, 0.03 m | 16 | the EKF 3 cm off at worst |
| `kiss_icp:=true` | ✅ | ✅ | 27 s, 0.14 m | 11 | KISS-ICP 0.13 m per 100 m, ekf_local 0.30 |
| `kiss_ekf:=true` | ✅ | ✅ | 28 s, 0.16 m | 10 | KISS-ICP 0.30, ekf_local 1.22 m per 100 m (one short leg: no ranking) |
| `voxels:=true` | ✅ | ✅ | 26 s, 0.17 m | 10 | 6.2 ms a cloud, 669 cubes |
| `sim_speed:=3` | ✅ | ✅ | 28 s, 0.14 m | 13 | the left turn never gets fast |

`sim-check` with a switch (the road world: start-up, contract, sensors, a 6.5 m drive, the EKF):

| Switch | Checks passed | Notes | Load |
|---|---|---|---|
| (none) | 9 of 9 | the EKF 0.15 m off | 28 |
| `speed_loop:=lqi` | 9 of 9 | drove 5.82 m of 6.5 | 15 |
| `speed_loop:=smooth` | 9 of 9 | drove 5.93 m | 11 |
| `lidar:=false` | 7 of 7 | | 11 |
| `voxels:=true` | 9 of 9 | 6.1 ms a cloud, 6 cubes | 9 |
| `kiss_ekf:=true` | 10 of 10 | KISS-ICP 10 Hz, ekf_local 30 Hz | 9 |
| `--lidar-obstacle` (the flat scan) | 12 of 12 | stopped **0.16 m** before the post | 10 |
| `--lidar-obstacle --voxels` | 12 of 13, then 13 of 13 twice | **run 1: drove 1.30 m into the post** (the stop zone fired, too late); runs 2-3: stopped 0.25 and 0.29 m before it | 9 |
| `--obstacle` (the radar's box) | all PASS | stopped 0.23 m before it | 6 |
| `--rear-obstacle` (backing up) | all PASS | stopped 0.78 m before the post behind | 7 |
| `--plan-around` | all PASS | round the box, 2.70 m clear | 5 |
| `--ghost` | all PASS | stood still 0.51 m before it | 10 |
| `--lane` | 1 FAIL, then all PASS | the FAIL was the check's own (it asked for stop_hold, which needs Nav2): fixed | 8 |
| `--spine` | all PASS | full brake 202-205 ms after the commands stop; e-stops 2 ms | 9 |
| `--faults` | 25 of 25 | each of the 14 faults stopped the car in 1.4-7.9 s | 4 |
| `--drive-to-point` | 16 of 17, twice | every stop within 0.46 m of B by the truth; 1 drive a run ended 0.61 m past B by its EKF (the limit 0.5) | 4-14 |

### Diffusion against the lattice (3 runs each)

The whole lot tour, reversing on, MPPI, `--planners lattice` and `--planners diffusion`,
taken in turns (lattice, diffusion, NDT-EKF, voxels; three rounds) so the Mac's load
falls on each alike. Load 7-12 throughout.

| | lattice (today's default) | diffusion |
|---|---:|---:|
| Legs finished | 12 of 12 | **12 of 12** |
| Tour time, each run (s) | 184, 183, 184 | 270, 177, 179 |
| Tour time, mean ± spread (s) | **183.4 ± 0.7** | 208.5 ± 53.4 |
| Without the one bad leg (median) | 184 | **179** |
| Plan tighter than full lock | 0 m | 0 m |
| Planning time, mean (the planner alone) | 9-12 ms | 126-129 ms |
| The EKF off, mean | 0.17 m | 0.17 m |

The lattice's first run is left out: Gazebo's clock froze for up to 7 s on its first leg
(the controller's loop "inf Hz", every sensor stale at once), so no leg could finish.
A fourth lattice run took its place.

**The guard, plan by plan:**

| | Run 1 | Run 2 | Run 3 | All |
|---|---:|---:|---:|---:|
| Plans | 149 | 134 | 135 | 418 |
| **From the model** | 95 (64 %) | 105 (78 %) | 105 (78 %) | **305 (73 %)** |
| The lattice's: **late** (over 150 ms) | 20 | 5 | 10 | 35 (8 %) |
| The lattice's: **refused** (no sample passed) | 31 | 23 | 19 | 73 (17 %) |
| The lattice's: no model answer | 3 | 1 | 1 | 5 (1 %) |
| Model and guard, mean / worst (ms) | 126 / 186 | 119 / 163 | 119 / 173 | 121 |

**The U-turn ("turn back", reversing):**

| | lattice | diffusion |
|---|---:|---:|
| Finished | 3 of 3 | 3 of 3 |
| Time (s) | 57, 55, 57 | **52, 51, 52** |
| Driven (m) | 29.0-30.7 | 25.5-26.6 |
| Driven in reverse (m) | 8.5-9.0 | 8.2-8.7 |
| Plans from the model | - | 23 of 40, 18 of 41, 23 of 41 (**52 %**) |

**What it shows:**

- **It works, and better than on 2 Nov**: 73 % of plans from the model (63 % then), and
  fewer late (8 % against 17 %), on a less busy Mac.
- **The U-turn is still its weak leg**: half its plans there are the lattice's, refused
  for turns (G5) and headings (G6). Yet that leg is 5 s quicker: a shorter path (26 m
  against 30 m).
- **One bad home leg** (run 1: 149 s, 18 m in reverse, 9 route changes) is why the mean
  is slow and wide. Runs 2 and 3: 177 and 179 s, against the lattice's 183-184 s.
- **Not a winner**: below the design's 90 %, and 10 times slower to plan. It stays a trial.

### The defaults' evidence: NDT into the EKF, and voxels (3 runs each)

The same rounds as above, the same tour. Off = from Gazebo's truth. The EKF's mean is
worked out the same way for every run (`drive_metrics.localizer_row`, the whole leg).

| | today's default (lattice, MPPI, the scan, the EKFs) | `ndt_ekf:=true` | `voxels:=true` |
|---|---:|---:|---:|
| Legs finished | 12 of 12 | **12 of 12** | **12 of 12** |
| Tour time, each run (s) | 184, 183, 184 | 214, 185, 181 | 216, 193, 191 |
| Tour time, mean ± spread (s) | **183.4 ± 0.7** | 193.8 ± 18.1 | 199.9 ± 14.1 |
| The global EKF off, mean (m) | 0.176, 0.170, 0.177 | **0.016, 0.015, 0.024** | 0.177, 0.165, 0.182 |
| The global EKF off, worst (m) | 0.48-0.60 | **0.13-0.27** | 0.52-0.56 |
| The car off Nav2's path, mean (m) | 0.12-0.14 | **0.05-0.17** (0.05 in runs 2-3) | 0.15-0.21 |
| Stops by collision_monitor (the U-turn) | 1-2 a tour | 1 a tour | 1-2 a tour |
| Its own cost | - | NDT on the Mac | 6.2 ms a cloud, 670-970 cubes |

The first round's home leg was slow for all three trials (86 s for NDT-EKF and voxels,
149 s for diffusion, 56 s usually): MPPI backed and turned at the goal. In rounds 2-3,
NDT-EKF matched the default's time (185 and 181 s); voxels stayed slower (193 and 191 s:
the home leg 61-65 s).

### The defaults: what to do

| Trial | The numbers | Switch the sim default? | Why |
|---|---|---|---|
| `ndt_ekf:=true` | the EKF 10 times closer; the car 2-3 times closer to its path; every leg; time the same (rounds 2-3) | **No, not as a plain default** | it needs `maps/ndt/test_lot.pcd`, which isn't in git: a new laptop has none, and `sim.launch.py` stops without it. The road world (`sim-check`) has no map at all. A default that depends on a file outside git would act differently on each computer |
| `voxels:=true` | every leg, but 16 s slower a tour; the post test passed 2 of 3 | **No** | not better on the tour, and one run drove through the post: that must be understood before it steers anything |
| `planner:=diffusion` | 73 % from the model; 10 times slower to plan | **No** | still a trial (Ben, 3 Nov brief: no more model work today) |

So the sim keeps today's defaults. To drive with NDT in the EKF, pick it:
`pixi run pick` (where am I: `ndt_ekf`), or `pixi run sim world:=test_lot ndt_ekf:=true`.
**The car keeps today's defaults** too: NDT needs a recorded map of the real lot first.

**Next** (each its own trial):

- `ndt_ekf` as the default of the **test-lot demos** (`pixi run demo`, `pick`), which
  already need the map, with a plain message when it's missing.
- Find why the cubes let the car through the post once (`sim-check --lidar-obstacle
  --voxels`, run 1 of 3: `log/sim-all/check-lidar-obstacle-voxels.log`).
