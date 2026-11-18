# MPPI that knows the lag

Ben, 7 Nov (0.3): **"have the algorithms know about lag"**.

Nav2's MPPI plans as if the car obeys at once. It can't take our lag model without a C++
plugin, and we write no C++. So MPPI is **ported to Rust**, with the lag in its model.
It began as a trial beside Nav2's MPPI; since 14 Nov it is the follower in `stack:=ours`,
and since Nav2 went (17 Nov, [as it was](history/nav2-as-it-was.md)) the only MPPI.
The comparisons with Nav2's MPPI below are the record.

> **The short version**
> - **Switch:** `tracker:=mppi_lag`. Package `aav_mppi_lag`.
> - **Reverses** (14 Nov) as Nav2's MPPI does, with `reversing:=true`: it stops at each
>   cusp, then backs up ([below](#reversing)). The lot's turn back stalled at a cusp
>   until the control cost was fixed (14 Nov, [Cusps](#cusps)).
> - **Same MPPI:** Nav2's sampling, critics, softmax and smoothing, nav2.yaml's numbers.
> - **New model:** ego's ramps, the spine's speed loop, the pedals' and steering's lags.
> - **Without ROS:** it predicts the car **70 times better** (8 mm off over 2.8 s, against 0.56 m).
> - **On very slow actuators** it keeps the path (0.1 m) where the lag-free MPPI **loses it**.
> - **At vehicle.yaml's guesses** both stay within a few cm: the lag is small there.
> - **Fast:** 1.7 ms a tick on an M4 Max (2000 samples x 56 steps); about 10 ms on the Orin.
> - **Gazebo, 7 Nov:** not a fair test: on a loaded computer the odometry stalled and it gave up
>   ([below](#in-gazebo)); while driving it was closer to the path (0.13 m against 0.20 m).
> - **Our simulator, 8 Nov** (newer; the numbers live on other pages): the lot tour, 3 runs,
>   **11 of 12 legs** (Nav2's MPPI 8: [trials](trials.md#results-our-simulator-8-nov-the-lot-tour-3-runs-each)).
>   In an all-ours stack it passes the campus's first goal at 2.5 m/s and uses twice
>   Nav2's MPPI's CPU (33-39 % against 17-18 %: [our navigator](own-navigator.md#the-verdict)).
>   Open: TODO 152 (slow earlier near the goal, fewer futures, time it on the Orin).
> - **Costmap:** our local costmap ([page](own-costmap.md)); Nav2's until 17 Nov.
> - **No wiggle at speed** (15 Nov, TODO 168): at 2.5 m/s the steering noise sized for
>   1 m/s made the steering jump side to side; it now shrinks with the speed
>   ([Oscillation](#oscillation-the-cause)).
> - **Sways no more than Nav2** (15 Nov, the `ours` stack): the plan's staircase
>   smoothed out, a steering control cost, the map's corrections taken slowly. Lot
>   0.021 m/s² (Nav2 0.018), campus 0.099 (Nav2 0.131), every leg
>   ([The sway that was left](#the-sway-that-was-left)).

## How it fits

```
planner (ours) ──path──▶ navigator ──FollowPath──▶ mppi_lag ──cmd_vel_nav──▶ smoother
                                                     ▲          ──▶ stop zone
        odometry, /vehicle/steering, throttle, brake ┘          ──▶ twist_mux ──▶ ego
```

The same place as `aav_track`'s trackers: controller_server's `follow_path` is renamed
`follow_path_unused`. Everything after `cmd_vel_nav` is unchanged: the safety chain too.

## The model

Each sample is rolled out through this, step by step:

| Stage | What it does | From |
|---|---|---|
| ego | speed ramps at 1.0 up, 2.0 down m/s², either way; a change of direction passes through a stop; steering at 0.6 rad/s; holds the steering under 0.05 m/s | ego.yaml, `ackermann.py` |
| speed loop | the spine's PI along the gear: throttle from the speed error, brake past a coast band, a hold brake at a stop; the gear changes only while stopped (until then it brakes) | spine-core's `default` (as Gazebo's `SpeedLoop`) |
| dead times | each demand waits floor(t / dt + 0.5) steps | vehicle.yaml (0 until measured) |
| throttle, brake | first-order lag, exact: level += (u - level)(1 - e^(-dt/lag)) | `throttle_lag`, `brake_lag` |
| speed | max_accel x throttle (along the gear) - max_brake x brake - drag x v; the brake never reverses the car | `max_accel`, `max_brake`, `drag` |
| steering | deadband (on the motor's setpoint), motor at most `steer_rate`, linkage lag, + offset, full lock | `steer_*` |
| pose | bicycle with the new speed and steering, at the mid-step heading | `wheelbase` |

**Where each sample starts:** the pose and speed (odometry), ego's output, the measured
steering, throttle and brake (while fresh), and **the commands already sent that are
still in their dead time**. Nav2's starts from the pose and speed alone.

### The trait: where the shared car model plugs in

The shared crate, `car-model` (`ros/bringup/aav_description/car-model`), is merged (0.3).
`LagModel` follows its rules (the four rows above) and sits behind a trait
(`crates/mppi-lag/src/model.rs`). The shared crate implements the trait behind the cargo
feature `shared` (`src/shared.rs`, tested against `LagModel`); the node still runs
`LagModel` (TODO 139).

```rust
pub trait CarModel: Sync {
    type State: Clone + Send + Sync + Debug;
    fn at_rest(&self, pose: Pose, speed: f64) -> Self::State;
    fn step(&self, state: &mut Self::State, command: Command, dt: f64);
    fn observe(&self, state: &mut Self::State, seen: &Observation);
    fn pose(&self, state: &Self::State) -> Pose;
    fn speed(&self, state: &Self::State) -> f64;
    fn steering(&self, state: &Self::State) -> f64;
    fn wheelbase(&self) -> f64;
    fn lag_aware(&self) -> bool;
}
```

| Model | What | Use |
|---|---|---|
| `LagModel` | the spec above (the car behind ego) | `model: lag`, the default |
| `IdealModel` | obeys at once (Nav2's assumption) | `model: ideal`: the same MPPI without the lag, for comparison |
| `Driven<M>` | ego's ramps in front of any model | always |

## What differs from Nav2's MPPI

| | Nav2 | Ours | Why |
|---|---|---|---|
| Samples | (vx, wz), each step clipped to ax, az | (speed, steering), clipped to the range only | ego's ramps are in the model; Nav2's clipping drifts samples slower |
| Noise | white | **correlated**, AR(1) 0.8 | see below |
| Steering noise at speed | `wz_std`: the angle's shrinks as 1 / speed | 0.15 rad up to 1 m/s, then x (1 / speed)² | the same sideways spread at every speed ([Oscillation](#oscillation-the-cause)) |
| Control cost | gamma / std² x Σ u e | gamma x u' Σ⁻¹ e, Σ the AR(1) noise's covariance; Nav2's for white noise | Nav2's form on correlated noise stopped the car ([Cusps](#cusps)) |
| Sent | element 1 | element 0 | the dead times already hold what was sent |
| Critics | the eight in nav2.yaml | the same eight, same weights; PathAngle's three modes; and our `LaneCritic` with a lane map ([below](#the-lane)) | Nav2 has no critic for a lane |
| At a cusp (reversing) | the critics' goal is the path's end | the cusp, while one lies ahead; the goal check waits for the last stretch | the car stops there as at a goal; Nav2 stops only because the path ends |
| Cusp reached | within the inversion tolerances | the same, or stopped within the goal's tolerances; or stopped 3 s within 2 m (`cusp_reach`) | it may stop just outside them, or be held there by the stop zone ([Cusps](#cusps)) |

### Why the noise is correlated

Through a lagging steering, white noise averages away: every sample drives alike, and
MPPI learns nothing. Without ROS, on a slow car (steering 0.5 rad/s, 0.3 s lag, 0.2 s
dead time), an S-curve (2 x 16 m) at 1 m/s, mean distance off the path (measured
with the first, forward-Euler version of the model):

| Noise | Lag-aware MPPI | Lag-free MPPI |
|---|---|---|
| white (Nav2's) | **lost the path** (5.6 m off) | 0.033 m |
| correlated 0.8 | 0.036 m | 0.018 m |

So ours samples smooth wiggles (colored-noise MPPI). `noise_correlation: 0.0` gives Nav2's.
The control cost must then use the same noise's covariance ([Cusps](#cusps)).

## Results without ROS

A pretend car: ego and the lag model, stepped every 0.01 s. MPPI at 20 Hz, 1000 samples.
`cargo test --release -p mppi-lag --test closed_loop -- --nocapture`.

**Prediction** (2.8 s ahead, weaving):

| Car | Lag model | Lag-free |
|---|---|---|
| vehicle.yaml | 0.022 m | 0.446 m |
| slow | **0.008 m** | 0.559 m |
| very slow | **0.004 m** | 0.708 m |

**Tracking** (mean / worst distance off the path):

| Path, car | Lag-aware | Lag-free |
|---|---|---|
| circle r 5 m, vehicle.yaml, 1 m/s | 0.011 / 0.018 m | 0.009 / 0.021 m |
| S 2 x 16 m, vehicle.yaml, 2 m/s | 0.017 / 0.037 m | 0.019 / 0.049 m |
| S 2 x 16 m, slow car, 2 m/s | **0.015 / 0.030 m** | 0.042 / 0.094 m |
| corner r 4 m, **very slow** car, 2 m/s | **0.014 / 0.070 m** | **lost it** (2.3 / 6.0 m) |
| S 2 x 16 m, **very slow** car, 2 m/s | **0.030 / 0.063 m** | **lost it** (2.2 / 6.5 m) |
| S 2 x 16 m, **very slow** car, 3 m/s | **0.030 / 0.066 m** | **lost it** (5.2 / 8.5 m) |

The slow car: steering 0.5 rad/s, 0.3 s lag, 0.2 s dead time. The very slow car: steering 0.3 rad/s, 0.5 s linkage lag, 0.3 s steering and 0.2 s pedal
dead times. Numbers from 14 Nov, after the control cost fix ([Cusps](#cusps)); before
it, lag-aware was 1.5-2x further off everywhere.

**Honestly:** at vehicle.yaml's guesses (0.1 s lags, no dead times) the lag is small:
the two track within 1-2 cm of each other, the lag-free one 2-4 s slower over each
route. The lag model pays off when the actuators are slow. The real numbers come from
tests 3.10-3.12 and 4.1-4.5.

## Speed

`cargo run --release -p mppi-lag --example timing` (all eight critics, a costmap):

| Threads | 2000 x 56 | 1000 x 56 |
|---|---|---|
| 16 (M4 Max) | 1.7 ms | 0.9 ms |
| 4 (the default) | 3.1 ms | |
| 1 | 12.7 ms | 6.3 ms |

The tick is 50 ms. **Orin, estimated, not measured:** its A78AE cores are about 3 times
slower than an M4's, so about 10 ms on 4 threads, 40 ms on one.

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

## Reversing

Ben, 14 Nov: **"The car can reverse. I don't know why we keep setting it to no"**.

With `reversing:=true`, `navigation.launch.py` sets it up as it sets Nav2's MPPI:

| Setting | Forward only | Reversing |
|---|---|---|
| `vx_min` | 0 | -`max_reverse_speed` (-0.5 m/s) |
| `enforce_path_inversion` | false | true: stop within 0.5 m and 0.6 rad of each cusp, then the next stretch |
| `PathAngleCritic.mode` | 0: face the path nose first | 2: the way the path faces (tail first on a reversing stretch) |
| `PreferForwardCritic` | on | off: the planner already decides when to reverse |

```
  forward stretch            cusp            reversing stretch
 ───────────────────▶  ●  stop, gear  ◀──  ◀──  ◀──  (the car still faces ▶)
 the window ends here ─┘  changes        the next window starts here
```

What is new in the maths (`crates/mppi-lag`):

| Part | Change |
|---|---|
| `lag.rs` | the spine's gear: it changes only at a stop; the loop works along it |
| `ego.rs` | `slew_speed`, as ego's: speeding up or slowing down either way; through a stop |
| `critics.rs` | `PathAngleCritic`'s modes 1 and 2 (`PathAngleMode`) |
| `path.rs` | `with_inversion`: the window stops at the cusp; the next stretch starts there |

**Closed loop, no ROS** (ego + the lag model at 100 Hz, vehicle.yaml's lags):

| Path | Result |
|---|---|
| 8 m straight back | reached; 1 mm mean, 3 mm worst off the path; never forward |
| 6 m ahead, a cusp, a quarter circle (r 5 m) back | reached; forward to 1.06 m/s, a stop, back at 0.5 m/s; one change of direction; 0.008 m mean off |
| the same with r 3.6 m (the planner's) | reached; 0.033 m mean, 0.088 m worst (r 4 m failed before the [Cusps](#cusps) fix) |
| the test lot's turn back | reached in 25.9 s; from rolling and off the path, 27.6 s ([Cusps](#cusps)) |

**Our simulator, 14 Nov, all ours** (`--navigator own --planner own --costmap own
--tracker mppi_lag`, reversing on; a busy Mac, load 25-40), one run each:

| `pixi run sim-check ...` | Result |
|---|---|
| `--rear-obstacle` (the goal 15 m behind, a post 6 m behind) | PASS: backed up at up to 0.53 m/s, the stop zone stopped it 0.85 m from the post; post taken away, goal reached 14 m on. No `behavior_server` runs: `mppi_lag` did the reversing |
| `--plan-around` (a box in the lane) | PASS: goal 30 m on, 2.6 m clear of the box |
| plain (contract, sensors, a scripted drive) | PASS, every line |

## Cusps

**Found 14 Nov:** `measure controllers --stacks nav2 own --tours lot --reversing`. On
the "turn back" leg (35, 42) facing east to (20, 42) facing west, Nav2's MPPI got there
in 59 s; `mppi_lag` gave up 4 times ("no progress in time"), 22.6 m driven.

Our planner's path there: **back** round a 3.6 m bend, a cusp, then forward round to
face west. Rebuilt without ROS (`turn_back` in `tests/closed_loop.rs`):

| Step | What happened |
|---|---|
| 1 | backing up, MPPI steered only 0.1 rad of the 0.4 needed |
| 2 | 0.7 m outside the bend, it stopped 1.9 m beside the cusp |
| 3 | nothing moved it on: Stuck after 10 s, every time |

**Cause: the control cost.** MPPI adds gamma x u x noise / std² for each sample (Nav2's
form, made for white noise). Ours samples correlated noise, and kept that form:

| Noise | Spread of the control cost |
|---|---|
| white (Nav2) | 1 x |
| correlated 0.8, white form (ours before) | **3 x**: it outweighed the critics and pulled speed and steering to 0 |
| correlated 0.8, its own covariance (ours now) | 1/3 x |

**Fix 1:** the cost uses the noise's own covariance (`control_cost` in `optimizer.rs`).
With `noise_correlation: 0` it is Nav2's exactly.

**Fix 2:** starting rolling and 0.5 m off the path, the car stopped 0.6 m from the cusp:
outside the inversion tolerance (0.5 m), so still Stuck. Now a cusp also counts once the
car has **stopped within the goal's tolerances** (1 m, 0.75 rad), as the path would end
at its goal. The progress check starts again at each new stretch.

| Closed loop, the turn back | Before | After |
|---|---|---|
| from a stop on the path | Stuck at 18.9 s | reached in 25.9 s |
| rolling at 0.5 m/s, 0.5 m and 11° off | Stuck at 18.3 s | reached in 27.6 s (needs fix 2) |

The planner's own path (exported from `lattice-planner` on the lot's map), 8 seeds
each: 0 of 8 before, 8 of 8 after; also with a slow car, rolling at 0.8 m/s, or
started 0.2 rad off. **In the simulator: not yet run** with the fix.

### Held short of a cusp (16 Nov, TODO 174)

`sim-check --three-point` (a 6.5 m street): the plan had 2-3 changes of direction, the car
drove 4-8. A run's record (`AAV_THREE_POINT_SAVE=<file>`) showed why:

```text
 wall ═══════════════════════════════
        ▲ the stop zone (1 m) reaches the wall: collision_monitor holds the car
      [car]  stands 1.3 m short of the cusp ✕, out of the goal's 1 m
 the navigator plans again (20 s on, the final approach): new cusps from there
```

| Part | What | Now |
|---|---|---|
| The progress check | started again **every tick**: a new stretch was told by its cusp's pose, and `map_correction` moves the path a little most ticks. A held car never gave up | by the stretch's index: held anywhere, Stuck after 10 s, and the navigator plans again |
| Held short of a cusp | nothing moved the car on | stopped `cusp_wait` (3 s) within `cusp_reach` (2 m) of the cusp: the stretch is done, it backs on |

Both are in `tests/closed_loop.rs` (`held_short_of_a_cusp_the_car_backs_on`,
`the_progress_check_fires_on_a_moving_path`; each failed before).

`sim-check --three-point`, our simulator, the 6.5 m street:

| Version | Runs | Changes of direction (the plans') | Time |
|---|---|---|---|
| before | 1 | 5 (2, 2, 3, 1) | 64 s |
| both fixes | 2 | **4** (2, 3, 0) and **6** (2, 3, 1) | **43 s, 56 s** |

**Tried, and not kept** (each made it worse in the sim; the code in `/tmp/w152`):

| Try | What happened |
|---|---|
| The navigator keeps a path with a cusp up to 120 s, not 20 | without the follower's fixes the car stood held for 100 s (6-7 changes, 145 s) |
| The planner keeps the stop zone clear where it changes gear (`cusp_clearance` 1 m) | offline, 4 cusps in the street; in the sim it turned 14 m down the street, sat held, and once left the street (2 of 2 failed) |

Left: the cause itself, the plan's cusps nearer a wall than the stop zone allows.

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
| **Fixed too:** our navigator planned again every second; each new plan moved a little. Now it keeps a clear to-pose path up to 10 s, as the trees' KeepAValidPath (`keep_path_age`, [own navigator](own-navigator.md)) | without ROS, the lot's left turn at 1 m/s: sway 0.075 (the sim's, ours) to 0.038 m/s²; a plan kept all the way, 0.025; Nav2 in the sim 0.017 |
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
`mppi_lag_steer_gamma:=` (empty) give the old ours; `stack:=stock` is unchanged.

### Still open

| Open | Next |
|---|---|
| Ours' sway on the lot | at Nav2's per speed²; the lot's 3-point turn sways most |
| The car | none of this has been on the car: the GPS, the lags and the Orin's tick are the sim's |
| Nav2 with the strong smoother | not scored: it may help Nav2's MPPI too (`--stacks nav2` with `--launch plan_smoothing:=strong`) |
| A smoother that knows the costmap | Dolgov et al. 2010's (Nav2's Constrained Smoother) pulls away from obstacles too; ours only stops at them |

## Follow in odom, correct slowly

TODO 186. The GPS moves the map; the follower should not feel it.

### The problem

The follower drives in `odom`, on the smooth wheels and IMU. The path comes in `map`.
The global EKF's map -> odom jumps with each GPS fix: in the sim about **0.09 m
sideways, 5 times a second**. Moved by the newest map -> odom, each new path lands up
to 9 cm beside the car. The follower sees a cross-track error and steers at it.

### The fix

```text
  newest map -> odom ──(jumps 0.09 m at 5 Hz)──┐
                                               ▼
  held map -> odom ◄── at most 0.1 m/s and 1°/s, measured at the car
       │               taken at once past 2 m or 10° (a relocalisation, logged)
       ▼
  path (map) ──► path (odom) ──► MPPI; the running path moves as the held one moves
```

| Part | What |
|---|---|
| The held map -> odom | the follower's own; each new path comes into odom by it |
| The slew | toward the newest, at most `map_correction_speed` (0.1 m/s) and `map_correction_turn_rate` (1°/s) |
| Measured at the car | a 1° turn about odom's origin 100 m away is 1.7 m there, but 0 at the car |
| The snap | past `map_correction_snap_distance` (2 m) or `_snap_angle` (10°): taken at once |
| Off | `map_correction:=newest` (the launch's default): each new path by the newest. On: `map_correction:=slow` |
| Status | **the sim's `ours` stack takes it** (15 Nov, with the [two fixes above](#the-sway-that-was-left)); `navigation.launch.py`, the car and `stack:=stock` keep `newest`. The obstacle check's stall (TODO 193) happens with `newest` too |
| Watch it | `map_gap_m` on `/drivers/nav/track/status`: the held one's gap at the car |

Why 0.1 m/s: odometry drifts 1-2 % of the distance (0.05 m/s at 2.5 m/s), so the held
one keeps up with real drift, but a 9 cm jump takes about a second to come in.
A complementary filter (Higgins 1975): the fast part of the pose from odometry, the
slow part from the map. `aav_track` runs the same rule (`slow_frame.py`).

### Results without ROS

`tests/map_correction.rs`: the node's frames rebuilt (odometry in odom, the path in map
again every second, a global EKF jumping 5 times a second), at 2.5 m/s, 2 runs each.

| Case | Newest (old) | Held (new) |
|---|---|---|
| straight: off the path m / sway m/s² / steering changes side a second | 0.042 / 0.101 / 0.52 | **0.019 / 0.040 / 0.19** |
| bend r 30 m: off / sway | 0.042 / 0.102 | **0.015 / 0.041** |
| a real 0.5 m offset taken out | | 5.6 s (0.05 m/s: 10 s; 0.2 m/s: 3.5 s) |
| a 3 m relocalisation | | snapped, settled in 4.4 s |

`explore` (ignored) prints every case: also an EKF error that wanders over 2 s, and new
paths every 10 s (the straight, wandering, every 10 s: sway 0.111 to 0.050 m/s²).

**In the simulator** (15 Nov, all ours, reversing, 2 runs each, the same runs as newest):
the campus sway 0.231 to 0.213 m/s², off the path 0.23 to **0.13 m** (worst 1.11 to 0.76 m);
the lot sway the same (0.032), off the path 0.19 to 0.13 m; every leg either way. A small
win on sway, a clear one on keeping to the path.

## The lane

TODO 169. With a lane map, the road ahead (`/world/road_ahead`, the behaviour planner's)
has edges. `mppi_lag` reads them and keeps the car's outline inside its lane. Nav2 has no
critic for this; `LaneCritic` is ours.

```
  ══════════ hard edge (kerb): past it costs 1000 a pose ═══════
  ── ── ──── soft edge (line): past it costs 100 x m² ── ── ──
  ●───────── the line (the lane's centre): free ──────────────►
```

| | |
|---|---|
| On | `behaviour:=true` with `lane_map:=` (e.g. `world`) and `tracker:=mppi_lag`: the launch names the topic and adds `LaneCritic` to the critics (`vehicle.mppi_lag_params(..., lane=True)`) |
| Off | otherwise: Nav2's eight critics only, the topic not read, as before |
| Reads | `/world/road_ahead` (map), moved into odom by the latest map -> odom; dropped after 0.5 s (`road_ahead_timeout`); no lane: silent |
| Scores | every 2nd pose: each corner of the padded outline beside the lane; the furthest past a soft edge, squared, x 100; past a hard edge, (1 + m) x 1000; both averaged over the poses |
| Not | a collision: a car already over a kerb (a lane map off by a metre) still gets a plan back in |
| Status | `lane` (true / false) on `/drivers/nav/track/status` |

Inside the lines it costs nothing, so the path still decides where in the lane the car
drives; at a line it pushes back smoothly.

**Closed loop, no ROS** (vehicle.yaml's lags, 1 m/s): a path drawn 1 m left of a 2.6 m
lane's centre (lines 1.3 m, kerbs 1.8 m), so its outline would be 0.25 m over the line:

| | Outline, furthest left | Settles at |
|---|---|---|
| Without `LaneCritic` | 1.71 m: 0.41 over the line | y 1.00 (on the path) |
| With it | **1.36 m: 0.06 over** | y 0.78 |

**Our simulator, 14 Nov** (`simulator:=own world:=road behaviour:=true tracker:=mppi_lag
cameras:=false`, a busy Mac, load 9-11): one goal 40 m on, **1.5 m left of the lane's
centre**, so at the goal the body would be 0.25 m over the line. The road: lines 1.75 m,
kerbs 2.25 m either side. The body (2 x 1 m, unpadded) against the edges, all the way:

| Run | Reached | Car's y, widest | Body past the line, worst | Room to the kerb, least |
|---|---|---|---|---|
| no lane map (Nav2's critics only) | yes, 44 s | 1.98 m | **0.95 m** | **-0.45 m: over the kerb** |
| `lane_map:=world`, run 1 | yes, 44 s | 1.23 m | 0.05 m | 0.45 m |
| `lane_map:=world`, run 2 | yes, 45 s | 1.28 m | 0.13 m | 0.37 m |

The status said `lane true` on every tick but the first one or two. The soft edge gives a
little where the goal itself lies over it (GoalCritic pulls); the kerb never came close.

**Speed:** 4.3 ms a tick on 4 threads with a 40 m lane, against 3.3 ms without (M4 Max,
busy; `--example timing` prints both).

## lag_aware: so ego doesn't cancel the lag twice

Ego's lead (0.3) cancels lag for drivers that don't know about it. It must turn off while
a lag-aware driver drives.

Ego's contract: `/drivers/<slot>/lag_aware`, `std_msgs/Bool`, reliable, transient local,
depth 1. This drives through navigation's slot: `/drivers/nav/lag_aware`.

| When | Says |
|---|---|
| start, exit | `false` |
| a path starts (`model: lag`) | `true` |
| the path ends | `false` |
| `model: ideal` | always `false` |

## Run it

```sh
pixi run sim world:=test_lot tracker:=mppi_lag                   # reversing (the sim's default)
pixi run sim-check --navigator own --planner own --costmap own --tracker mppi_lag --rear-obstacle
pixi run measure controllers --trackers mppi_lag --controllers mppi --tours lot
cd ros/drivers/aav_mppi_lag && cargo test --release -p mppi-lag
```

## What is missing

| Missing | Next |
|---|---|
| The shared car-model crate | built behind the feature `shared`; switch the node over after a quiet-machine sim-check (TODO 139) |
| The car's real lags | tests 3.10-3.12, 4.1-4.5; then these runs again |
| The Orin's tick | measured on the Orin |
| Reversing, scored | `measure controllers --stacks own --reversing`: the turn back failed (14 Nov); run it again with the [Cusps](#cusps) fix |
| The forward legs with the new control cost | closer to the path without ROS; the simulator's tours again (forward only too) |
| ego's jerk limit, sideways slow-down | not modelled (off at our speeds) |
| The goal at speed, the CPU | passes a goal at 2.5 m/s; twice Nav2's MPPI's CPU (8 Nov; TODO 152) |
| A gap in odometry | gives up after 1 s, as `aav_track` (Nav2's MPPI drove on) |
| A live swap | it is its own node: `tracker:=` at launch only |
| The lane's soft edge, opened | ✅ 16 Nov: while our planner passes something parked (status `how: pass`), the road ahead follows the pass and its left soft edge opens to the kerb there; `LaneCritic` reads it as sent ([own-planner.md](own-planner.md#passing-something-parked-16-nov)). Not yet driven with `lane_map:=` on |

Code: [`ros/drivers/aav_mppi_lag`](../ros/drivers/aav_mppi_lag/README.md). Ported from
Nav2 (Apache-2.0): [PROVENANCE](../ros/drivers/aav_mppi_lag/PROVENANCE.md).
