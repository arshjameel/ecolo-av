# Trials: the dated results

Moved from [Trials](../trials.md) on 18 Nov (TODO 172: shorter pages), word for word, so that
page stays the guide: what is on trial, how to run it, the fair rules. Most of these were
scored in Gazebo, inside Nav2's chain, before both were deleted (16 and 17 Nov).
In the order they were on the page.

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
  (`maps/example_lot_amcl.yaml`, [why](../../ros/drivers/aav_navigation/maps/README.md#why-amcl-has-a-map-of-its-own)).
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

### 18 Nov: AMCL in Rust against stock

TODO 231: `nav2_amcl` was the last Nav2 node. Ours (`aav_amcl_rs`, `amcl_impl:=rust`, now
the default) reads the same `amcl.yaml`, topics and frames. Stock stays as
`amcl_impl:=stock`.

**How:** `measure controllers --amcl --tours lot --runs 3 --launch amcl_impl:=stock`, then
`amcl_impl:=rust`. Ravensim on a Mac (M4 Max, with a GPU), `mppi_lag`, 1 m/s, AMCL's own
0.05 m map. Results in `/tmp/amcl_cmp/` (not in git).

AMCL off the truth, mean of each run's mean (m), and the worst of all runs (m):

| Leg | Stock, 3 runs | Ours, 3 runs | EKF |
|---|---|---|---|
| a left turn | 0.103 (0.09-0.11), worst 0.16 | 0.089 (0.08-0.10), worst 0.14 | 0.10 |
| round the building | 0.051, worst 0.15 | 0.053, worst 0.11 | 0.11 |
| turn back | 0.067, worst 0.20 | 0.060, worst 0.15 | 0.09 |
| home | 0.075, worst 0.18 | 0.076, worst 0.13 | 0.10 |

- **A tie**, as a port should be: means within 0.015 m a leg, every leg reached, the same
  number of poses (each resample), heading within 1°.
- **One bad start**, in a single run before these (seed from the clock): ours was 0.27 m
  mean on leg 1, 0.94 m worst, over 1 m off or without a pose for 5.8 s, then 0.06-0.09 m
  on the other legs. Stock's single run then: 0.11 m, 0.15 worst. Not seen in the next 4
  runs of leg 1 (0.08-0.10 m). The first pose comes from the start's 0.5 m cloud after one
  scan; watch for it in more runs.
- Recovery (`recovery_alpha_*`) is still off, in both, as `amcl.yaml` had it.

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
- Before it goes on the car: the trials' rule ([below](../trials.md#before-a-winner-goes-on-the-car)).

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

## Lag-aware MPC, 7 Nov (`mpc_lag`)

Ben, 7 Nov: "have the algorithms know about lag". Two parts: ego gets a **lead** that
cancels the lag for every driver (another branch), and this tracker knows the lag
itself. Only one may cancel it, or the car gets twice the correction.

### What it knows

```
set speed ──> spine's PI ──> throttle, brake (lags) ──> speed (drag)
steering  ──> dead time ──> motor (0.8 rad/s) ──> linkage (lag) ──> wheels ──> the car
```

| It models | From |
|---|---|
| the spine's speed loop (`default` or `smooth`) | spine-core's gains (`car_model`'s `SpeedLoop`) |
| throttle and brake lags, drag, full brake | `vehicle.yaml` `actuators:` |
| steering motor's rate, deadband, linkage lag, offset | `vehicle.yaml` `actuators:` |
| dead times, and the commands already sent | `*_dead_time` (0 until measured), plus `latency` 0.1 s |
| the car | the kinematic bicycle, in the path's frame |

`latency` is the ROS chain's own delay (ego's 50 Hz ramp, the 20 Hz hold, the adapter).
Without it the pretend car's U-turn at 2 m/s was off by 0.14 m; with it, 0.09 m.

### How it plans (every 50 ms)

1. Estimate the state: pose and speed measured; throttle, brake, motor, wheels and the
   speed loop's integral from what it sent.
2. Run last tick's plan through the model; linearise it (one numpy call a step).
3. One QP (our OSQP-style solver), 1.5 s ahead, in steering **and** set speed.
4. Send the first steering and set speed.

**Hard limits:** full lock, the steering rate (the tighter of ego and the motor), the
set speed's rise and fall (the smoother's 1.0 and 2.0 m/s²), 0 to cruise. **Soft:** the
speed under the bend's and the goal's limit.

### It tells ego (the `lag_aware` contract)

`/drivers/nav/lag_aware` (`std_msgs/Bool`, reliable, transient local, depth 1): **true**
while `mpc_lag` is the tracker, **false** for every other tracker and when the node stops.
Nav2's MPPI and RPP publish nothing: ego keeps its lead on (`lag_lead: auto`). Ego's
contract (merged 7 Nov); the Rust MPPI (`mppi_lag`) says it on the same topic.

### Results: the pretend car (ego, then Gazebo's actuator model)

Worst off the path (m). "x2": the car's lags twice what the trackers were told.

| Drive | Lags | pure pursuit | LQR | MPC | `mpc_full` | **`mpc_lag`** |
|---|---|---:|---:|---:|---:|---:|
| U-turn, 2 m/s | x1 | 0.38 | **0.08** | 0.09 | 0.12 | 0.09 |
| U-turn, 2 m/s | x2 | 0.34 | 0.27 | 0.36 | **0.10** | 0.17 |
| U-turn, 2 m/s | x3 | 0.31 | 0.70 | 0.70 | **0.30** | 0.50 |
| U-turn, 1 m/s | x1 | 0.12 | 0.07 | **0.03** | 0.10 | 0.04 |
| U-turn, 1 m/s | x2 | 0.10 | 0.05 | 0.10 | 0.07 | 0.05 |
| 8 m bend, 3 m/s | x1 | 0.16 | 0.03 | 0.02 | 0.03 | **0.02** |
| 8 m bend, 3 m/s | x2 | 0.13 | 0.04 | 0.04 | 0.02 | **0.02** |
| Top speed (set: 1 m/s) | x1 | 1.14 | 1.14 | 1.14 | 1.15 | **1.00** |

What it shows:

- **Speed:** only `mpc_lag` sees the speed loop's 14 % overshoot coming, and eases
  the set speed in. No overshoot at any lag.
- **Steering, lags as told:** as good as MPC and LQR; no better.
- **Lags doubled:** better than MPC and LQR (0.17 against 0.36 and 0.27 m), but
  `mpc_full` is better still: it plans over distance and slows itself to swing.
- **Lags tripled:** it falls behind `mpc_full` and pure pursuit. A model it trusts
  that is wrong by 3x hurts: the car-day numbers (TODO 52b) matter.

### Time

| Where | Mean | Worst |
|---|---:|---:|
| Pretend car, Ben's Mac quiet | 7 ms | 16 ms |
| Our simulator, the lot tour (8 Nov, load ~12) | **5 ms** | 14-19 ms (60 once); 0-1 ticks missed a leg |
| Gazebo, Mac at load 30-50 (7 Nov) | 14-20 ms | 64-230 ms; up to 516 ticks missed a leg |
| Pretend car, Mac at load 39 | 34 ms | 250 ms |
| The Orin (estimate: 2-3 times slower in Python) | 15-20 ms | 40-50 ms |

It does about 4 times the old MPC's work a tick, so a busy computer hurts it more.

The tick is 50 ms. In Python it fits on average, but its worst ticks on the Orin
would be close to a whole tick. **It should move to Rust** before it drives the car
(CLAUDE.md: Rust where it pays off), with the shared car model's Rust twin: the same
QP there is well under 1 ms.

### Results: our simulator (8 Nov, the lot tour, 3 runs each)

Our Rust simulator (the default since 8 Nov) runs the shared car model, so the car
answers exactly as `mpc_lag` predicts. `log/own-nominal/`, not in git.

| | MPPI (Nav2) | RPP (Nav2) | `mppi_lag` (Rust) | **`mpc_lag`** |
|---|---:|---:|---:|---:|
| Legs reached | 8/12 | 9/12 | **11/12** | 6/12 |
| Off the path, mean (m) | 0.13 | **0.12** | 0.16 | 0.14 |
| Off the path, worst (m) | **0.60** | 0.66 | 1.63 | 0.66 |
| Sway (m/s², lower is calmer) | **0.038** | 0.086 | 0.054 | 0.097 |
| Left turn, time (s, mean of 3) | 27 | 28 | 34 | **25** |
| Left turn, worst off the path (m) | 0.40 | 0.44 | 0.64 | **0.33** |

Leg by leg, `mpc_lag`:

| Leg | Runs reached | Why not |
|---|---|---|
| a left turn | 3 of 3 | |
| round the building | 3 of 3 | |
| turn back (the U-turn in the 14 m lane) | **0 of 3** | it held the path (0.47 m worst, as MPPI's 0.49), but the path ran by the building: collision_monitor stopped the car, and the planner then refused to start ("Start occupied") |
| home | 0 of 3 | never started: the leg before had stranded the car |

What it shows:

- **Where it drives, it drives well:** the quickest and closest on the left turn, 5 ms
  a tick, no speed overshoot.
- **The U-turn is everyone's weak spot here** (MPPI 1 of 3, RPP 1 of 3, `mppi_lag` 2 of
  3), but `mpc_lag` lost all three. Our trackers follow the path and never read the
  costmap; MPPI keeps off obstacles, and RPP slows near them. `mpc_lag` needs the same
  before it can win the lot tour.

### Results: Gazebo, a loaded Mac (7 Nov, one run each)

Every follower lost legs to the sensor watchdog: at load 30-50 (other agents' builds
and simulators) the IMU, odometry and GPS went stale and stopped Nav2. These measure
the Mac more than the followers; kept for the record.

| Lags | MPPI | RPP | MPC | `mpc_lag` |
|---|---:|---:|---:|---:|
| as told: legs reached | 1/4 | 0/4 | 2/4 | 2/4 |
| as told: off the path, worst (m) | 0.50 | 1.65 | 2.16 | 1.88 |
| doubled (`lag_scale:=2`): legs reached | 0/4 | 2/4 | 3/4 | 2/4 |
| doubled: off the path, worst (m) | 3.39 | 1.66 | 0.50 | 0.48 |

### Results: Gazebo, a quiet Mac (8 Nov, one run each)

`simulator:=gazebo` (the doubled lags need Gazebo's adapter). `log/gz-nominal-c/`,
`log/gz-doubled-c/`, `log/gz-doubled-d/` (not in git).

| Lags | | MPPI (Nav2) | RPP (Nav2) | `mppi_lag` | **`mpc_lag`** |
|---|---|---:|---:|---:|---:|
| as told | legs reached | 2/4 | 2/4 | (never came up) | **4/4** |
| as told | off the path, worst (m) | 0.61 | 0.87 | - | **0.60** |
| doubled | legs reached | **3/4** | **3/4** | 2/4 | 2/4 |
| doubled | off the path, worst (m) | 1.17 | 0.76 | **0.58** | 0.60 |

- With the lags as told, `mpc_lag` was the only follower to drive the whole tour, the
  U-turn included.
- With them doubled it lost the U-turn (collision_monitor stopped it by the building)
  and then home; it stayed on the path (0.60 m worst).
- One run each: a hint, not a verdict. In our simulator the same U-turn beat every
  aav_track tracker (`mpc`, `pure_pursuit` and `mpc_lag` all 2 of 3 legs, 8 Nov): they
  follow the planner's path and don't read the costmap.

### In Rust, 15 Nov (`mpc_lag_rs`)

TODO 132: the Python plan was near 50 ms a tick on the Orin (estimated): the whole 20 Hz
tick. `aav_mpc_lag` is the same tracker in Rust, a challenger beside the Python one
(`tracker:=mpc_lag_rs`; the league's steer option `mpc_lag_rs`, in no stack).

```
the Python (aav_track)            the Rust (aav_mpc_lag, crate mpc-lag)
lag_model.py   ──── ported ────►  model.rs     the same states, to 4e-15
qp.py          ──── ported ────►  qp.rs        the same iterations (400 cold, 50 warm)
mpc_lag.py     ──── ported ────►  tracker.rs   the same commands, to 1e-3
car_model.py   ──── its twin ──►  car-model    the estimate
```

| On this Mac, the U-turn at 2 m/s | Python `mpc_lag` | Rust `mpc_lag_rs` |
|---|---|---|
| A tick, mean | 5.3 ms | **0.75 ms** |
| A tick, worst | 26 ms | **2.4 ms** |
| Worst off the path, vehicle.yaml's lags | under 0.12 m (its test) | 0.10 m |
| Worst off the path, the lags doubled | under 0.25 m (its test) | 0.18 m |

In our simulator, 15 Nov: the test lot's tour (4 legs), one run each, headless, forward
only, 1 m/s, `pixi run measure controllers --trackers mpc_lag_rs mpc_lag --tours lot`
(`log/mpc-lag-rs-compare/`, not in git):

| The lot tour | Python `mpc_lag` | Rust `mpc_lag_rs` |
|---|---|---|
| Legs reached | 4 of 4 | 4 of 4 |
| Time | 164 s | 160 s |
| Off the path, mean / worst | 0.12 / 0.84 m | 0.13 / 0.87 m |
| Sway (m/s² RMS) | 0.052 | 0.057 |
| A tick, mean (leg by leg) | 3.7-7.7 ms | **0.7-1.3 ms** |
| A tick, worst | 14.2 ms | **6.6 ms** |
| The follower's CPU | 28-35 % | **4-6 %** |
| Missed 20 Hz ticks | 0 | 0 |

They drive alike, as they should: the same maths. The Rust one swayed more on the first
leg (28 against 14 left-right changes per 100 m) and less on others: one run, a hint.

- **The QP:** our ADMM ported, not clarabel (TODO 199). It warm-starts from the last
  tick and gives the Python's answers; clarabel (an interior point method) does neither.
- **Shared vectors:** `gen_vectors.py` writes the Python's states and commands to
  `vectors.json`; `cargo test` checks the Rust against them.
- **Not in the Rust yet:** keeping each path inside the lane (TODO 169, `road_ahead_topic`)
  and the live swap.

### Not done yet

- The LQI speed loop (`speed_loop:=lqi`) isn't modelled; `mpc_lag` doesn't follow
  `speed_loop:=` on its own (set `mpc_lag.speed_loop`).
- Today's car runs the ESP32's open speed loop, not the spine's PI: not modelled.
- Dead times act on the set speed, not after the speed loop (an approximation; they are 0
  until measured).
- The plan runs `lag_model.py`, the shared car model's maths on arrays (for speed);
  a test keeps it equal to `car_model.step` to 1e-9. The estimate runs `car_model` itself.

## Planners, 29 Oct: hybrid, lattice and smac2d

What each planner is: [Controllers, Planners](../controllers.md#planners).

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

## Diffusion, 7-8 Nov: the U-turn, the Orin, the car

Ben, 7 Nov: "We want diffusion." What changed and why: the design's
[As built, 0.3](../designs/2026-11-02_diffusion-planner.md#as-built-03-7-8-nov). The guard's
checks did not change.

```bash
pixi run diffusion data --first 800 --maps 400 --uturn 0.6 --out models/diffusion/data_uturn
pixi run diffusion train --data models/diffusion/data models/diffusion/data_uturn --suffixes 3 --epochs 40
pixi run measure controllers --planners diffusion --reversing --tours lot \
    --diffusion-model <.onnx> --diffusion-smoothing <gear|average> --diffusion-record <folder>
pixi run diffusion replay <folder> --model <.onnx>
```

| | Before (the 0.2 recipe) | After (0.3) |
|---|---|---|
| Data | 44,644 teacher paths, 800 maps | the same + 21,420 on 400 maps, 60 % U-turns |
| Training | 30 epochs, 94,347 examples, 25 min (MPS) | 40 epochs, 196,124 examples, about 65 min (MPS, the Mac shared) |
| Smoothing | one average over the whole path | each gear's run on its own; a wider second try for G5/G6 only |
| Sampler | the map encoded 16 times | once |

### Offline: 300 held-back plans, 16 samples each

| | Before | After |
|---|---:|---:|
| Samples that pass | 38 % | **63 %** |
| Plans with a passing sample | 77 % | **87 %** |
| G5 turns | 2,197 | **850** |
| G4 collisions | 1,152 | 1,053 |
| G6 heading | 725 | **451** |
| G8 length | 96 | 111 |
| G1, G2, G3, G7 | 0-3 | 0-2 |

### In the simulator: the lot tour, reversing on, MPPI (8 Nov)

Our own simulator (`next-0.3`'s default), Ben's Mac, one simulator at a time
(`lockf /tmp/aav-sim.lock`), before and after taken in turns; load 7-15. Runs:
`log/controller-compare/agent106-*` (not in git).

| | Lattice (the default) | Before (0.2 recipe) | **After (0.3)** |
|---|---:|---:|---:|
| Legs finished | 12 of 12 | 12 of 12 | **12 of 12** (and 4 of 4 after the merge) |
| Tour time, each run (s) | 182, 185, 180 | 180, 175, 180 | **171, 175, 175** (175) |
| **Plans from the model** | - | 307 of 397 (**77 %**): 80, 73, 79 % | 343 of 389 (**88 %**): 84, 89, 92 % (92 %) |
| The U-turn ("turn back"), from the model | - | 47 of 117 (**40 %**) | 81 of 112 (**72 %**) (80 %) |
| The lattice's: refused (no sample passed) | - | 67 (17 %) | **25 (6 %)** |
| The lattice's: late (over 150 ms) | - | 14 (3.5 %) | 13 (3.3 %) |
| The lattice's: no model answer | - | 9 | 8 |
| Samples that passed | - | 57 % | **86 %** |
| Refused by G5 / G6 / G4 / G8 (samples) | - | 2,120 / 761 / 556 / 46 | **300 / 208 / 254 / 251** |
| Model and guard, mean | - | 108 ms | 104 ms |
| Tightest planned turn (full lock 2.86 m) | 3.27 m | 3.06 m | 3.06 m |

**The same plans, replayed** (`pixi run diffusion replay`, the 405 plans the "before"
runs recorded, through each model): plans with a passing sample **82 → 91 %**; the
U-turn's **43 → 71 %**; G5 2,204 → 435, G6 828 → 250, G4 637 → 388, **G8 75 → 401**.

### What it shows

- **88 % of plans from the model** over three runs (84, 89, 92 %), 92 % in a fourth
  after the merge: the 90 % target is reached on a quiet machine, not yet on average.
  What's left: 6 % refused, 3 % late, 2 % no answer.
- **The U-turn: 40 → 72 %** from the model, and its leg 44-51 s against the lattice's 51-56 s.
- **Faster than the default**: 171-175 s a tour against the lattice's 180-185 s, every leg,
  never a plan tighter than full lock.
- **G8 (length) rose** (46 → 251 samples): the U-turn data taught longer swings; the
  guard refuses them, and the next sample usually passes. Worth a look before more data.


### The time a plan (`pixi run measure diffusion`)

Ben's Mac (M4 Max), 8 Nov, the CPU, others' simulators and builds running (load 17),
30 made-up plans on a lot with boxes, the 0.3 model:

| Part | Mean | 95th | Worst | Budget (the Orin) |
|---|---:|---:|---:|---:|
| Crop | 0.7 | 1.1 | 1.1 | 2 |
| Model (16 samples, 10 steps) | 82.6 | 125.9 | 129.7 | 50 |
| Guard | 11.6 | 21.2 | 22.4 | 10 |
| Total | 94.9 | 141.1 | 142.8 | 100 (stop 150) |

The FP16 file on the CPU: the same (84.9 ms the model): FP16 pays only on a GPU. Core ML
loads the 0.3 export but can't run it: the worker drops to the CPU. **The Orin: not
measured** (car day A12 runs this script there, CPU then TensorRT).

### The guard and the time

- **The guard's refusals fell where the fault was**: G5 to 39 %, G6 to 62 % of before;
  G4 (collisions) barely moved: the model still cuts some corners, and the guard still
  refuses them.
- **No check was loosened**: every path that reaches MPPI passed G1-G8 as on 2 Nov.
- **Time** is the other half of the fallback: inside the 150 ms stop on a loaded Mac, but
  over the Orin's 100 ms budget on a CPU. The Orin needs its GPU (TensorRT).

## Voxels, 30 Oct: the lidar as cubes

`voxels:=true` (`aav_voxels`, [design](../designs/2026-10-30_voxels.md)): Nav2's costmap
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

## The wiggle, 14 Nov (TODO 124, 149)

**The question:** under Nav2 the car's turn flips from side to side. Why?

**The road:** the campus, lot P4 along the road, one goal, about 305 m
(`pixi run measure controllers --tours wiggle --series`). Our simulator, 2.5 m/s, 2 runs each.

**The numbers** (`drive_metrics.wiggle`, also on every scorecard):

| Number | Means |
|---|---|
| swing (m) | how far the car goes side to side about its own smoothed track (RMS) |
| flips a second | how often the steering, or the turn rate, changes side while moving |

### The three tests

Each switch takes one suspect away. Swing in metres, flips of ego's steering a second.

| Test | MPPI | Pure Pursuit (RPP) | What it says |
|---|---|---|---|
| Baseline | 0.054-0.065 m, 0.23-0.27 | 0.119-0.131 m, 0.54-0.56 | |
| 1. The simulator's truth as the pose (`truth_pose:=true`) | 0.062-0.063 m, 0.21-0.22 | 0.083-0.113 m, **0.29-0.36** | the GPS's jitter: half of RPP's flips; none of MPPI's |
| 2. One plan a goal (`replan_hz:=0`) | 0.062-0.074 m, 0.26 | **0.082-0.087 m**, 0.45 | the replanning: a third of RPP's swing; none of MPPI's |
| 3. Before and after the smoother | flips 0.24-0.27 before, the same after | 0.62-0.63 before, 0.60-0.62 after | the smoother adds nothing (`mppi_lag` too: 0.88-1.06 before, 0.79-0.87 after) |

The global EKF's sideways error jumps by **0.09 m RMS every 0.2 s** (the sim's GPS: 0.5 m
noise, 5 fixes a second), and its heading by about 1°. With the truth as the pose, both go.

**The replanning also made loops:** with a new plan every second the car drove 9-16 new
routes a goal and 40-130 m more than the route, mostly circling the goal. With one plan: 0.

### The causes

| Cause | Who it moves | Fixed? |
|---|---|---|
| **A new plan every second** (the to-pose tree's 1 Hz) | RPP, `mppi_lag`; the loops for all | **yes**: the tree keeps a valid path |
| **The GPS's jitter through the global EKF** | RPP (half its flips) | no: next, below |
| **MPPI's own sampling** | MPPI: 0.06-0.08 m, a flip every 4 s; `mppi_lag`: a flip a second | no: the follower itself |

### The fix: keep a valid path

The to-pose trees (`navigate_to_pose_car*.xml`) now plan again only when they must, as
Nav2's own `nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml`: the
path is kept while it still ends at this goal, nothing blocks it (`IsPathValid`) and it is
under 10 s old. The final approach keeps its 20 s (TODO 17). A config change: Nav2's
servers are the same.

| Follower | Swing before → after (m) | Steering flips a second | Time (s) | Driven (m) | New routes | Reached |
|---|---|---|---|---|---|---|
| MPPI | 0.054, 0.065 → 0.065, 0.080 | 0.27, 0.23 → 0.22, 0.26 | 189, 211 → **146, 142** | 343, 371 → **302, 304** | 9, 12 → **0** | 2/2 → 2/2 |
| RPP | 0.119, 0.131 → **0.089, 0.094** | 0.54, 0.56 → **0.42, 0.47** | 288, 145 → **125, 126** | 315, 350 → 302, 305 | 4, 4 → **0** | 1/2 → **2/2** |
| `mppi_lag` | 0.170, 0.137 → **0.100, 0.100** | 0.68, 0.71 → 1.04, 1.15 | 210, 210 → 137, 232 | 431, 93 → 301, 158 | 16, 0 → **0** | 1/2 → 1/2 |

The same, from a recorded drive (`tools/record.sh`, then `tools/score_drive.py`; RPP, one
run each, the old tree put back for "before"):

| Scorecard row | Before | After |
|---|---|---|
| wiggle: side to side, RMS / worst | 0.139 m / 0.50 m | **0.084 m / 0.22 m** |
| wiggle: steering changed side | 0.43 a second | 0.43 a second |
| distance, time | 524 m, 242 s | **305 m, 134 s** |
| off the plan, mean | 0.30 m | **0.17 m** |

**What it shows:** RPP's and `mppi_lag`'s swing is 25-40 % smaller, and no follower loops
at the end any more. MPPI's swing (6-8 cm) didn't change: it never came from the plan.
`mppi_lag`'s flips went up because it now spends the whole drive on the road at speed
(before, half of it was slow circling); its swing still fell.

### Next

- **The GPS's jitter:** follow in `odom` and apply the map's corrections slowly (the
  whole-car design's idea), or trust each fix less in `ekf_global`. Test 1 says it halves
  RPP's flips.
- **`mppi_lag`'s flip a second:** its own sampling; tune its noise and smoothing.
- **Our navigator** (`aav_navigator`) still plans every second: port `KeepAValidPath`.
- Not yet on the car, nor in Gazebo (our simulator only); 2 runs each.

Raw runs (not in git): `log/wiggle/a-base`, `a-truth`, `a-once`, `b-after`.

## The lane finder as a pose sensor, 15 Nov (TODO 155)

**The idea:** the GPS knows where the car is to a metre and jitters. The painted lines
know where it is **across** the road to a few centimetres. With the lane map, that is a
pose in the map: sure across the lane, nothing along it.

```text
 lane_finder ──lines──┐
 lane map ──centre────┼─► lane_pose ──► /localization/lane/pose ──► ekf_global (x, y)
 TF map → base_link ──┘    (gates)       0.15 m across, 50 m along
```

**Switch:** `lane_pose:=true` (`sim.launch.py`, `localization.launch.py`). Off by default:
the GPS-only EKF stays the champion. Package: `aav_lane_pose`; config:
`aav_localization/config/ekf_global_lane.yaml`.

### The gates

A picture becomes a pose only if all of these pass. The last two came from the campus.

| Gate | Refused when |
|---|---|
| a line | none seen, or the surest under 0.5 confidence |
| the car's angle | more than 0.3 rad from the lane |
| the map | off every road; another road or the road's end within 4 m behind, 12 m ahead |
| the width | both lines seen, gap off the map lane's by over 0.6 m |
| jumps | the offset moved over 0.4 m in under 1 s |
| the filter | the offset over 1.5 m from the filter's (the other lane) |
| **the angle again** | the lines' angle over 0.05 rad from the filter's |
| **the bend** | the lines' bend over 0.003 1/m from the map lane's |

**Why the last two:** the finder fits each line 4-16 m ahead and carries it back to the
car. A dashed centre line, or a junction's paint, bends the fit: the offset was then 0.2-0.4 m
wrong for seconds at a time. On a recorded campus drive (offline, the truth beside it):

| Pictures used | Offset vs the truth, RMS | 95 % | Worst |
|---|---|---|---|
| every one | 48 cm | 43 cm | 3.9 m |
| the angle and bend gates | **4.3 cm** | 9.9 cm | 19 cm |

### The numbers: the global EKF against the truth

`tools/lane_drive.py [--lane-pose]` (`pixi run measure lane`): the lane follower steers by
the camera; the EKF only watches, so this is the pose alone. Our simulator, 90 s each.

| Road | lane_pose | Sideways error, RMS | 95 % | Worst | Jitter (RMS) |
|---|---|---|---|---|---|
| the road world, straight (2 runs each) | off | 9.2, 9.7 cm | 21, 19 cm | 33, 26 cm | 3.2, 3.0 cm |
| | **on** | **1.9, 2.0, 2.1 cm** | 3.8-3.9 cm | 7-10 cm | **1.0-1.2 cm** |
| the campus, Library Road, 130 m with a bend | off (2 runs) | 9.9, 9.9 cm | 20, 20 cm | 30 cm | 3.3 cm |
| | on, before the last two gates | 10.5, 10.8 cm | 29 cm | 35 cm | 1.6 cm |
| | **on** (2 runs) | **5.4, 5.4 cm** | 13 cm | 20 cm | **2.0, 2.1 cm** |

The jitter is the sideways error less its own 2 s average: the quick swing a path
follower sees. Along the road nothing changes (6-8 cm RMS), as designed.

### The wiggle with the EKF steering: not shown

The wiggle's tour (`measure controllers --stacks own --tours wiggle --runs 2`, 2.5 m/s):

| lane_pose | Swing (m RMS) | Steering flips a second | Reached |
|---|---|---|---|
| off | 0.098, 0.086 | 0.34, 0.28 | 1/2 (run 2: the follower aborted) |
| on | 0.091, 0.092 | 0.28, 0.32 | 2/2 |

**No effect to read here:** most of that road has no paint the finder sees. Of about
1,400 pictures a run, 17-32 became poses. The rest: no line (most), a junction, or an
offset over 1.5 m from the filter's (probably the car out of the map's right-hand lane:
the planner doesn't keep right). A fair test
needs a long painted road with the car in its lane (our stack with `lane_map:=world`).

### What it shows

- Where the paint is good, the sideways error is **4-5 times smaller** and the jitter
  **a third**. On a real road with a bend, **half**.
- Bad lines would make it worse (the campus, before the last two gates): the gates matter
  more than the filter.
- **Not tested:** the car (a real camera, real paint, its EKF heading off by ~2°, which
  the angle gate leans on); the wiggle with it steering; reversing; night.
- 15 Nov: it stayed a challenger, off by default.

### With the EKF steering, ours' stack (17 Nov): now ours' default

`pixi run measure controllers --stacks own_calm_chunks --tours <tour> --runs 1 --reversing
--series --launch behaviour:=true lane_route:=true things:=true [lane_pose:=true]`, our
simulator with its cameras. Here the EKF steers, so the sway and the lane are its doing.
The sideways error from each run's 0.05 s series (`lane_score.pose_error`); off the lane's
centre by `lane_score.lane_error` on the campus's `lanes.yaml`.

| Tour | lane_pose | Runs | EKF sideways, RMS | Jitter | Sway (m/s²) | Off the lane's centre |
|---|---|---|---|---|---|---|
| University Drive, 367 m | off | 4 | 9.2-9.9 cm | 2.7-2.9 cm | 0.057-0.063 | 0.10-0.11 m |
| | **on** | 2 | **6.9, 7.8 cm** | **2.1 cm** | 0.056, 0.065 | 0.08, 0.10 m |
| University Drive, the box tour, 90 m | off | 5 | 7.1-8.2 cm | 2.7-3.1 cm | 0.054-0.078 | 0.26-0.43 m (it passes the box) |
| | **on** | 2 | **2.1, 1.9 cm** | **1.4, 1.2 cm** | 0.066, 0.060 | 0.33, 0.29 m |
| the campus tour, 534 m | off | 1 | 9.1 cm | 3.1 cm | 0.100 | 1.53 m (mostly off painted lanes) |
| | on | 1 | 9.4 cm | 3.0 cm | 0.097 | 1.34 m |

- On a painted road it cuts the sideways error by a fifth (University Drive) to three
  quarters (the box tour's straight), and the jitter by a quarter to a half.
- The campus tour: no change (little paint on its way, the finder found no line most of
  the time). Sway and the lane's centre: no worse, within one run to the next.
- **Ours' default since 17 Nov** (`stacks.yaml`, pose level `ekf_lane`): on where the
  world has a camera and a lane map, off elsewhere (the test lot, `cameras:=false`).
  `pose:=ekf` turns it off.
- Under a load of 15-20 (two simulators) the lane finder saw no line and the runs were
  lost to stalls (`sensor_watchdog`): those runs are left out.
