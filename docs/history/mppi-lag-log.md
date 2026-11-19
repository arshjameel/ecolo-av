# MPPI with lag: the investigations

Moved from [MPPI with lag](../mppi-lag.md) on 18 Nov (TODO 172: shorter pages), word for
word: the dated runs and the hunts for the sway, 7-15 Nov. The page keeps how it works,
its settings and how to run it.

## In Gazebo

The test lot's first leg ("a left turn", 22 m), `pixi run measure controllers --tours lot
--legs 1`, 7 Nov, on an M4 Max **shared with other jobs (load 20 to 45)**:

| Run | Follower | Reached | Driven | Off the path, mean | Tick, mean / worst |
|---|---|---|---|---|---|
| 1 | `mppi_lag` | no: Gazebo stalled 2.3 s, the geofence latched | 13.5 m | **0.13 m** | 2.5 / 13.3 ms |
| 2 | Nav2's MPPI | **yes, in 70 s** | 22.1 m | 0.20 m (worst 0.62) | (C++, not logged) |
| 3 | `mppi_lag` | no: Nav2 gave up | 2.3 m | 0.27 m | 10.3 / 41.1 ms |
| 4 | `mppi_lag` | no: Nav2 gave up | 7.4 m | **0.13 m** | 10.0 / 16.5 ms |

**Honestly: not a fair comparison yet.** Under that load the odometry stopped for 0.6 to
3.7 s at a time (the sensor watchdog's STALE lines, in every run, Nav2's too).

| On a gap in the odometry | Does |
|---|---|
| `mppi_lag` (as `aav_track`) | gives up the path after 1 s (`odom_timeout`); Nav2 retries, then gives up |
| Nav2's controller_server | reads the pose from `/tf` without an age check, and drives on |

While it drove, `mppi_lag` stayed closer to the path (0.13 m against 0.20 m). Run it again
on a quiet computer, the whole tour, both followers, before calling a winner.

## The campus junction: the geofence, not MPPI

**Found 14 Nov night:** the campus tour's "round the bend", all ours, reversing on.
After 92 m the car stood still at a wide junction, (224.25, -160.90), for 70 s:
4 short reverses, 11 x "no progress in time", then the leg gave up.

```
  start ··· road east ───────●╮    ● the car stopped: 196.9 m from the start
                          :   │    the path turns right (r ≈ 4.5 m), south,
                          :   │    out to 216 m from the start
                          :   ╰─╮
                          :     ← the sim's fence, 200 m round the start
```

| Evidence (the saved run) | Says |
|---|---|
| the car's spot | **196.9 m** from the start spot (49.2, -70.8) |
| the planned path | reaches **216 m** from it: 62 of its 246 points past 198.9 m |
| the launch log | `geofence NEAR: a stopping path ... comes 1.1 m from the edge`, at each stop |
| what MPPI asked | 2.4 m/s forward, or 0.4 m/s back: the car **did not move** either way |

**Cause:** the sim's geofence (`geofence_radius`, 200 m) holds every program, forward
and back, near its edge, by design. MPPI was never the one stopping.

**Fix:** the campus tours in `measure controllers` run with a 600 m fence
(`CAMPUS_FENCE`, as `docs/routes.md`), and a leg's notes now say "the geofence held
the car" when it did.

| Closed loop, `campus_junction_round_the_bend` | Result |
|---|---|
| the run's own path, 51 points from 75 m in; 2.5 m/s asked; rolling, 0.6 m off, 15° across | reached in 39.6 s |
| off the path | 0.011 m mean, 0.039 m worst |
| backing up | none |

**In the simulator:** not yet run with the 600 m fence.

## Oscillation: the cause

TODO 168. Ben: **"don't forget reverse and no occelation"**.

**Seen 14 Nov**, the campus tour, all ours (the saved runs, about 2.4 m/s):

| | All ours | All Nav2 |
|---|---|---|
| swing: side to side about its own track | **0.18-0.26 m** | 0.04-0.08 m |
| sway: the sideways push a passenger feels | **0.58-0.76 m/s²** | 0.04-0.16 m/s² |
| steering changes side | **0.7-1.1 a second** | 0.15-0.4 a second |

At 1 m/s on the test lot, all ours swung only 0.02 m. The swing is real: the true
track swings as much as the EKF's (0.22 against 0.23 m). The EKF's own error (0.08 m
RMS) changes side only 0.2 times a second.

### Rebuilt without ROS

`cargo run --release -p mppi-lag --example wiggle`. The closed-loop tests above saw only
nav2.yaml's 3 m of path; MPPI then holds the car near 1.2 m/s. The launch file shows it
2.8 s of path at the top speed (8.4 m at 2.5 m/s). With that, the model wiggles as the
sim did: on the sim's own campus path, with a sim-like pose, 0.18 m, 0.59 m/s², 0.76
changes a second (the sim: 0.20, 0.70, 0.77).

Each suspect, alone, on 100 m straight at 2.5 m/s (2 runs each):

| Suspect | Sway m/s² | Steering changes side /s | Says |
|---|---|---|---|
| as on 14 Nov, the true pose | 0.166 | 1.89 | wiggles with nothing wrong |
| + an EKF-like pose error (8 cm, 1.2°) | 0.209 | 1.45 | adds a little |
| + the pose 0.15 s old | 0.189 | 1.38 | adds a little |
| + 0.2 s steering dead time the model doesn't know | 0.194 | 1.53 | adds a little |
| no lag at all, and the model knows it | 0.229 | 2.31 | not the lag |
| no speed overshoot | 0.063 | 1.63 | less, but it drove slower (1.9 m/s) |
| 8000 samples, not 2000 | 0.092 | 1.31 | **less: it is sampling noise** |
| temperature 0.5, not 0.15 | 0.082 | 1.25 | less: more samples count |
| white noise (Nav2's) | 0.034 | 0.70 | less, but 6 tests of reversing and slow actuators fail |
| **after the fix** | **0.022** | **0.00** | |

### The cause

```
 steering noise 0.15 rad ──▶ a future's end moves v² x the angle
   1 m/s:   futures 2.8 s long spread ~0.2 m sideways ──▶ many samples count
   2.5 m/s: they spread ~1 m (6 x as wide) ──▶ the softmax rests on a handful
            each tick a different handful ──▶ the steering jumps ──▶ the wiggle
```

The noise was sized for 1 m/s (Nav2's `wz_std` 0.1 rad/s at 1 m/s). The critics
(PathFollow, PathAlign) score where a future ends, sharply, so at 2.5 m/s only the few
futures that end near the path count, and each tick's few are different ones. Pose
noise, delays and unknown lag only add to it.

### The fix

`steer_std_speed: 1.0` (`config/mppi_lag.yaml`, `Settings::steer_std_at`): above 1 m/s
the steering noise is 0.15 x (1 / speed)², 0.024 rad at 2.5 m/s, so the futures spread as
far sideways as at 1 m/s. It never goes under `steer_std_min`, 0.024 rad, so above
2.5 m/s the futures still spread. At 1 m/s and below (the lot, the cusps, backing up)
nothing changes. The recommended option of the ones tried: more samples cost CPU, a warmer
softmax blurs the path, white noise breaks reversing.

At 2.5 m/s, swing m / sway m/s² / steering changes a second, 2 runs each:

| Path | Before | After |
|---|---|---|
| 100 m straight, true pose | 0.010 / 0.166 / 1.89 | **0.003 / 0.022 / 0.00** |
| 100 m straight, sim-like pose (EKF-like, 0.15 s old) | 0.054 / 0.289 / 1.17 | **0.038 / 0.117 / 0.48** |
| bend r 30 m, true pose | sway 0.152, 0.49 a second | **sway 0.024, 0.00** |
| the sim's campus path ("along the road"), true pose | 0.141 / 0.409 / 0.86 | **0.090 / 0.194 / 0.30** |
| the same, sim-like pose | 0.182 / 0.590 / 0.76 | **0.112 / 0.262 / 0.37** |

The price: a little further off the path on tight bends (corner r 5 m: 0.014 to
0.049 m; S 2 x 30 m: 0.013 to 0.025 m). On the campus path, 0.026 to 0.090 m: it cuts
the plan's jogs (below) instead of following them.

Tests (`tests/closed_loop.rs`, as the launch file sets the node up): no wiggle on a
straight and a bend at 2.5 m/s, less with a sim-like pose, none backing up.

### What is left

| Left | Numbers |
|---|---|
| **The simulator** confirmed (15 Nov, all ours, 3 runs each, twice: drive-k and TODO 164's re-score) | sway, lot 0.055 → 0.030-0.031 m/s², campus 0.66 → 0.21-0.23; every leg reached in the re-score (lot 12/12, campus 9/9); off the path 0.21-0.22 m mean. Nav2: 0.017 and 0.12 |
| The target (drive-j, 15 Nov, before both fixes): Nav2's sway | campus at 2.5 m/s: ours 0.662, Nav2 0.123 m/s²; the lot: ours 0.055, Nav2 0.017 m/s². Keep ours' 100% legs and its off-path (0.13 m campus, 0.12 m lot) |
| The planner's jogs: 16 lattice headings make a road at an angle a staircase (±8° every ~10 m) | the plan itself swings 0.13 m at 2.4 m/s, both stacks; most of what is left |
| **Fixed too:** our navigator planned again every second; each new plan moved a little. Now it keeps a clear to-pose path up to 10 s, as the trees' KeepAValidPath (`keep_path_age`, [own navigator](../own-navigator.md)) | without ROS, the lot's left turn at 1 m/s: sway 0.075 (the sim's, ours) to 0.038 m/s²; a plan kept all the way, 0.025; Nav2 in the sim 0.017 |
| The lot at 1 m/s: still above Nav2's | less steering noise at 1 m/s helps the model (sway 0.022 to 0.013 with a sim-like pose) but fails the very slow actuators' S-curve test: not taken |
| Backing up at 0.5 m/s, the wheels dither about 1° | the car itself doesn't move sideways (0.3 mm) |

## The sway that was left

TODO 165, 167, 168, 186. 15 Nov, after the steering noise fix. Ben: **"no oscillation"**.

> **The short version**
> - **Where it starts:** the follower, `mppi_lag`. Swap it and the sway goes with it.
> - **Two causes:** the plan's **staircase** on roads at an angle, which `mppi_lag`
>   follows faithfully; and MPPI's own **sampling jitter**, at 1 m/s too.
> - **Two fixes**, both in the `ours` stack now: `plan_smoothing:=strong` (the
>   planner's smoother pulls harder) and `mppi_lag_steer_gamma:=0.05` (a steering
>   control cost), with `map_correction:=slow` (TODO 186).
> - **Result:** sway at or under Nav2's on both tours, closer to the path, every leg.

### Which stage? Swap one at a time

Our simulator, reversing on, 2 runs each, the same build. Sway (m/s²), the mean of the legs
reached:

| Stack | Lot | Campus | Campus speed (m/s) |
|---|---|---|---|
| all Nav2 | 0.018 | 0.131 | 1.60 |
| all ours | 0.032 | 0.231 | 2.18 |
| ours, **Nav2's MPPI** following | **0.020** | 0.099 (2 legs of 6) | 1.17 |
| Nav2's, **`mppi_lag`** following | **0.029** | **0.239** | 2.17 |

The sway follows the follower. Our navigator, planner and costmap add nothing.

```text
 navigator ─▶ planner ─▶ path ─▶ follower ─▶ smoother ─▶ ego ─▶ car
   (same)      (same)      │      mppi_lag      (same)    (same)
                           │         │
                staircase ─┘         └─ sampling jitter
```

The smoother and ego add no flips: the follower's turn changes side as often before the
smoother as after it (lot 0.27 a second, ours; 0.12, Nav2's).

### Cause 1: the plan's staircase

The lattice has 16 headings, 22.5° apart. A road at 8° comes out as steps: 0°, then
17°, then 0°, about every 9 m, 0.2 m high. Nav2's smoother weights (w_data 0.2,
w_smooth 0.3) only take out wiggles shorter than about 2 m, so the steps stay.

`mppi_lag` keeps to the path (0.13-0.23 m off; Nav2's MPPI 0.3-0.5 m), so it drives
the steps. At 2.2 m/s a 9 m step is a 0.25 Hz swing: just where ours had 3 times
Nav2's steering (0.3-0.5 Hz, the campus).

Without ROS (`cargo run --release -p lattice-planner --example staircase`), a 6.5 m
road, the plan's own swing at 2.2 m/s:

| Road at | Nav2's weights | w_data 0.02 |
|---|---|---|
| 5° | 0.148 m | 0.070 m |
| 8° | 0.160 m | 0.042 m |
| 11.25° | 0.110 m | 0.020 m |
| 15° | 0.128 m | 0.033 m |
| 30° | 0.084 m | 0.023 m |
| the lot's corners | tightest 3.33-4.10 m | 3.08-4.79 m (the car: 2.86 m) |

Fix: **`plan_smoothing:=strong`** writes w_data 0.02 into the planner block's
`smoother`. Nav2's lattice and ours both read it. Alone, in the sim, it took the
campus from 0.210 to 0.175 m/s² (with `map_correction:=slow`).

### Cause 2: the sampling jitter

Each tick MPPI averages 2000 random futures. Each tick a few different ones win, so
the steering jumps a little. Nav2's MPPI does the same, but its white noise averages
out; our correlated noise (needed for the lag) doesn't.

Without ROS (`examples/wiggle.rs`), a straight with a sim-like pose, 3 runs:

| Tried | 1 m/s sway | 2.5 m/s sway | Corner r 5 m off the path (2.5 m/s) |
|---|---|---|---|
| as it was | 0.023 | 0.120 | 0.085 m |
| 8000 samples | 0.019 | | |
| temperature 0.3 | 0.021 | | |
| correlation 0.6 or 0.9 | 0.021-0.022 | | |
| gamma 0.05 (speed and steering) | 0.014 | 0.069 | 0.138 m; slower |
| **steering gamma 0.05** | **0.015** | **0.068** | 0.160 m; speed kept |

The control cost (gamma) leans each sample toward a smaller command. On the speed
too it slowed the car on the campus (2.0 to 1.6 m/s) and dithered at a cusp (23
stops on one leg). On the steering alone the speed stays.

Fix: **`mppi_lag_steer_gamma:=0.05`** (the node's `steer_gamma`). Below 1 m/s
(`steer_gamma_speed`) it eases back to gamma, as speed², so the cusps steer as before:
at 0.05 everywhere, three cusp tests failed (0.61 m off the turn back); eased, all pass.

### The result

The simulator, reversing on, 2 runs each. All Nav2 and all ours: the same runs (15 Nov,
midday); ours today: the `ours` stack now (`measure controllers --stacks own_calm`).

| | Nav2 | Ours, before | **Ours, today** |
|---|---|---|---|
| **Lot** sway (m/s²) | 0.018 | 0.032 | **0.021** |
| Lot left-right changes per 100 m | 10.3 | 11.7 | 5.7 |
| Lot off the path, mean (m) | 0.25 | 0.19 | 0.15 |
| Lot legs | 8/8 | 8/8 | 8/8 |
| **Campus** sway (m/s²) | 0.131 | 0.231 | **0.099** |
| Campus left-right changes per 100 m | 11.3 | 11.8 | 8.3 |
| Campus off the path, mean (m) | 0.50 | 0.23 | 0.18 |
| Campus legs | 4/6 | 6/6 | 6/6 |
| Campus speed (m/s) | 1.60 | 2.18 | 1.98 |

The price: the campus's "along the road" takes 135-144 s, not 111-116: its
three-point turn at the start is slower (a stop, 2-4 reversals), and its worst moment
off the path is there (2.1 m, against 1.1). The lot's turn back is as quick as before
(30-33 s; Nav2's 53). Sway is speed x turn rate: ours drives the campus a quarter
faster than Nav2 (1.98 against 1.60 m/s) and still sways less.

| Stage | Share (sim) |
|---|---|
| `map_correction:=slow` alone | campus 0.231 to 0.213; off the path 0.23 to 0.13 m; the lot the same |
| + `plan_smoothing:=strong` | campus to 0.175; the lot 0.030 |
| + `mppi_lag_steer_gamma:=0.05` | campus to 0.099; the lot to 0.021 |

Every switch stays: `plan_smoothing:=nav2`, `map_correction:=newest` and
`mppi_lag_steer_gamma:=` (empty) give the old ours.

### Still open

| Open | Next |
|---|---|
| Ours' sway on the lot | at Nav2's per speed²; the lot's 3-point turn sways most |
| The car | none of this has been on the car: the GPS, the lags and the Orin's tick are the sim's |
| Nav2 with the strong smoother | not scored: it may help Nav2's MPPI too (`--stacks nav2` with `--launch plan_smoothing:=strong`) |
| A smoother that knows the costmap | Dolgov et al. 2010's (Nav2's Constrained Smoother) pulls away from obstacles too; ours only stops at them |

## 19 Nov: today's default against the yardstick

TODO 167. Nav2 went on 17 Nov, so the yardstick is two sets of recorded numbers:
Nav2's of 15 Nov (afternoon), and ours of the same afternoon ([the result](#the-result)).

> **The short version**
> - **The campus:** at or better than both, except 0.03 m more off the path than ours
>   was, and 2 % slower.
> - **The lot:** every leg, closer to the path than Nav2; sway 0.024 against ours' 0.021
>   and Nav2's 0.018. Ours was above Nav2's on 15 Nov too.
> - **No clear regression.** Each gap is inside the spread of one run to the next. The new
>   simulated IMU (TODO 171) is not the lot's sway.

### How it was run

Ravensim on Ben's Mac (GPU, headless), reversing on, 3 runs a tour, a ROS domain each
(161-173), this computer only. The build: this branch, from `main` at `15a4adeb`.

```sh
AAV_SIM_DOMAIN=161 pixi run measure controllers --stacks own_calm --tours lot \
    --runs 3 --reversing --launch stack:=ours --out /tmp/w167/lot
AAV_SIM_DOMAIN=171 pixi run measure controllers --stacks own_calm --tours campus \
    --runs 3 --reversing --launch stack:=ours --out /tmp/w167/campus
```

`--launch stack:=ours` comes after the tool's own `stack:=best`, so it wins: each launch
log says `stack:=ours - pose ekf_lane, costmap ours_all_chunks, navigator ours, path lane,
steer mppi_lag_calm, smoother ours, lanes off, behaviour on, kerbs on`. On the lot there
is no lane map, so `path lane` is the lattice alone and `ekf_lane` is the plain EKF.

| Batch | Load at the start | At the end |
|---|---|---|
| Lot | 0.7 | 6.1 |
| Campus | 4.9 | 7.2 |

### The results

Means of 3 runs, weighted by distance. "Top speed" is the truth's, over 0.5 s; the
command never asked above 1.0 m/s on the lot and 2.5 m/s on the campus (the 14 %
overshoot is the speed loop's: TODO 163).

| | Today | Nav2, 15 Nov | vs Nav2 | Ours, 15 Nov | vs ours |
|---|---|---|---|---|---|
| **Lot** legs | 12/12 | 8/8 | ✅ | 8/8 | ✅ |
| Lot sway (m/s²) | 0.024 | 0.018 | ❌ | 0.021 | ❌ (inside the spread) |
| Lot left-right changes per 100 m | 6.6 | 10.3 | ✅ | 5.7 | ❌ (inside the spread) |
| Lot off the path, mean (m) | 0.15 | 0.25 | ✅ | 0.15 | ✅ |
| Lot off the path, worst (m) | 0.70 | 4.5 (morning) | ✅ | not kept | - |
| Lot time a tour (s) | 158 | not kept | - | 159 (16 Nov) | ✅ |
| Lot top speed (m/s) | 1.12 | - | - | - | - |
| **Campus** legs | 9/9 | 4/6 | ✅ | 6/6 | ✅ |
| Campus sway (m/s²) | 0.098 | 0.131 | ✅ | 0.099 | ✅ |
| Campus left-right changes per 100 m | 8.2 | 11.3 | ✅ | 8.3 | ✅ |
| Campus off the path, mean (m) | 0.21 | 0.50 | ✅ | 0.18 | ❌ |
| Campus off the path, worst (m) | 1.70 | 6.2 (morning) | ✅ | 2.1 | ✅ |
| Campus mean speed (m/s) | 1.94 | 1.60 | ✅ | 1.98 | ❌ (2 %) |
| Campus time a tour (s) | 278 | not kept | - | 282 (16 Nov) | ✅ |
| Campus top speed (m/s) | 2.69 | - | - | - | - |

"Not kept": 15 Nov's raw results are gone; "(morning)" is drive-j's, before the sway
fixes; "16 Nov" is the league's final (`stack:=ours` then).

### Each run

| Tour | Run | Sway | Changes /100 m | Off, mean (m) | Off, worst (m) | Speed (m/s) | Time (s) |
|---|---|---|---|---|---|---|---|
| lot | 1 | 0.028 | 6.8 | 0.16 | 0.70 | 0.83 | 159 |
| lot | 2 | 0.019 | 4.6 | 0.13 | 0.52 | 0.84 | 157 |
| lot | 3 | 0.024 | 8.3 | 0.15 | 0.55 | 0.84 | 157 |
| campus | 1 | 0.097 | 7.4 | 0.20 | 1.69 | 1.95 | 277 |
| campus | 2 | 0.092 | 8.3 | 0.20 | 1.64 | 1.90 | 284 |
| campus | 3 | 0.105 | 9.0 | 0.21 | 1.70 | 1.96 | 271 |

The worst moment on the campus is "along the road" (1.64-1.70 m each run), as on 15 Nov:
its three-point start.

### What changed it? Two checks

Same commands, one thing different, 3 runs each:

| Run | Lot sway | Lot changes /100 m | Lot off, mean / worst (m) | Campus sway | Campus off, mean (m) | Campus speed (m/s) |
|---|---|---|---|---|---|---|
| **Today's default** | 0.024 | 6.6 | 0.15 / 0.70 | 0.098 | 0.21 | 1.94 |
| The IMU as before 18 Nov (`accel_from odometry`) | 0.024 | 6.3 | 0.14 / 0.51 | 0.095 | 0.18 | 1.97 |
| 15 Nov's shape (`stack:=best` + own_calm: no behaviour, kerbs, chunks, `ekf_lane`) | 0.022 | 6.0 | 0.13 / 0.54 | not run | | |

- **The IMU** (`sim_imu`'s `accel_from` has no launch switch: its default was changed in
  the source for this run only, then put back). The lot: the same. The campus: 0.03 m
  closer to the path with the old IMU, but its runs (0.16-0.21 m) overlap today's
  (0.20-0.21 m). Likely noise, unproven. The EKFs fuse only the IMU's yaw rate.
- **The parts added since 15 Nov** (behaviour, the kerbs, the chunks): the lot without
  them sways 0.022 (runs 0.020-0.026) against 0.024 (0.019-0.028). A hint, not a result.

Loads: the IMU batch started at 3.7 and ended at 9.1; the `best` batch 5.4 to 6.3.
Results: `/tmp/w167/` (not kept).

### Still open

| Open | Next |
|---|---|
| The lot's sway against Nav2's 0.018 | as on 15 Nov: the lot's three-point turn back sways most |
| The campus 0.03 m further off the path | 5 or more runs each way, to tell it from noise |
