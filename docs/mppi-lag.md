# MPPI that knows the lag

Ben, 7 Nov (0.3): **"have the algorithms know about lag"**.

Nav2's MPPI plans as if the car obeys at once. It can't take our lag model without a C++
plugin, and we write no C++. So MPPI is **ported to Rust**, with the lag in its model.
Nav2 still plans the path; ours follows it. It is a **trial**, beside Nav2's MPPI.

> **The short version**
> - **Switch:** `tracker:=mppi_lag`. Package `aav_mppi_lag`.
> - **Reverses** (14 Nov) as Nav2's MPPI does, with `reversing:=true`: it stops at each
>   cusp, then backs up ([below](#reversing)).
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
> - **Costmap:** Nav2's local costmap, or ours with `costmap:=own` ([page](own-costmap.md)).

## How it fits

```
planner (Nav2) ──path──▶ bt_navigator ──FollowPath──▶ mppi_lag ──cmd_vel_nav──▶ smoother
                                                         ▲          ──▶ collision_monitor
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
| Sent | element 1 | element 0 | the dead times already hold what was sent |
| Critics | the eight in nav2.yaml | the same eight, same weights; PathAngle's three modes | |
| At a cusp (reversing) | the critics' goal is the path's end | the cusp, while one lies ahead; the goal check waits for the last stretch | the car stops there as at a goal; Nav2 stops only because the path ends |

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
| circle r 5 m, vehicle.yaml, 1 m/s | 0.015 / 0.028 m | 0.015 / 0.031 m |
| S 2 x 16 m, vehicle.yaml, 2 m/s | 0.035 / 0.087 m | 0.024 / 0.052 m |
| S 2 x 16 m, slow car, 2 m/s | 0.041 / 0.097 m | 0.021 / 0.077 m |
| corner r 4 m, **very slow** car, 2 m/s | **0.023 / 0.101 m** | 0.248 / **1.467 m** |
| S 2 x 16 m, **very slow** car, 2 m/s | **0.049 / 0.103 m** | **lost it** (4.1 / 8.0 m) |
| S 2 x 16 m, **very slow** car, 3 m/s | **0.048 / 0.107 m** | **lost it** (3.5 / 7.3 m) |

The slow car: steering 0.5 rad/s, 0.3 s lag, 0.2 s dead time. The very slow car: steering 0.3 rad/s, 0.5 s linkage lag, 0.3 s steering and 0.2 s pedal
dead times.

**Honestly:** at vehicle.yaml's guesses (0.1 s lags, no dead times) the lag is small, and
the lag-free MPPI tracks as well or 1-2 cm better (on the slow car too), but 2-4 s
slower over each route. The lag model pays off
when the actuators are slow. The real numbers come from tests 3.10-3.12 and 4.1-4.5.

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
| 6 m ahead, a cusp, a quarter circle (r 5 m) back | reached; forward to 1.06 m/s, a stop, back at 0.5 m/s; one change of direction |
| the same with r 4 m | **not reached**: it fell 0.6 m outside the bend and stopped short |

Why r 4 m fails: the path bends at once after the cusp, and the car starts from a stop
with its wheels straight. The planner's radius is 3.6 m. In the stack the navigator
replans, which may save it; not measured yet.

**Our simulator, 14 Nov, all ours** (`--navigator own --planner own --costmap own
--tracker mppi_lag`, reversing on; a busy Mac, load 25-40), one run each:

| `pixi run sim-check ...` | Result |
|---|---|
| `--rear-obstacle` (the goal 15 m behind, a post 6 m behind) | PASS: backed up at up to 0.53 m/s, the stop zone stopped it 0.85 m from the post; post taken away, goal reached 14 m on. No `behavior_server` runs: `mppi_lag` did the reversing |
| `--plan-around` (a box in the lane) | PASS: goal 30 m on, 2.6 m clear of the box |
| plain (contract, sensors, a scripted drive) | PASS, every line |

Not yet seen in the simulator: a path with a cusp (a three-point turn). The lot tour
with `measure controllers --stacks own --reversing` would show it.

## lag_aware: so ego doesn't cancel the lag twice

Ego's lead (0.3) cancels lag for drivers that don't know about it. It must turn off while
a lag-aware driver drives.

Ego's contract: `/drivers/<slot>/lag_aware`, `std_msgs/Bool`, reliable, transient local,
depth 1. This drives through Nav2's slot: `/drivers/nav/lag_aware`.

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
| Reversing at a tight bend | after a cusp, a bend under ~1.1 x the planner's radius ([above](#reversing)) |
| Reversing, scored | `measure controllers --stacks own --reversing`: not yet run |
| ego's jerk limit, sideways slow-down | not modelled (off at our speeds) |
| The goal at speed, the CPU | passes a goal at 2.5 m/s; twice Nav2's MPPI's CPU (8 Nov; TODO 152) |
| A gap in odometry | gives up after 1 s, as `aav_track`; Nav2's MPPI drives on: decide which is right |
| A live swap | it is its own node: `tracker:=` at launch only |

Code: [`ros/drivers/aav_mppi_lag`](../ros/drivers/aav_mppi_lag/README.md). Ported from
Nav2 (Apache-2.0): [PROVENANCE](../ros/drivers/aav_mppi_lag/PROVENANCE.md).
