# Stop on the point: identify the car, then control it

* **Author:** Ben
* **Date:** 2026-10-26

## Overview

The task: drive from A to B by GPS. Speed up to a set speed, hold it, then brake and stop
on B.

The idea, before review:

1. **Identify the car.** Measure its steering, speeding up and braking from logged step
   runs, and fit the model (`vehicle_model.py`) to them.
2. **Control it with the model.** A predictive speed controller that knows where to start
   braking, and a path follower of our own (LQR or MPC), scored against MPPI.
3. **Keep the inner loops** (the Jrk G2's steering, spine-core's PI on speed), unless the
   data shows they hold the car back.

Six independent reviewers read it against the code. Their verdict, in one line:
**the plan is aimed at a car we don't drive yet, and half of it is already built.** The
gap is in what reaches the brake, and in what we can measure.

## Analysis

### 1. The car on the ground isn't the car in the plan (all six reviewers)

The car runs the 2025-26 C++ ESP32 through `legacy_uart_bridge`. No Rust board has been
flashed (`docs/status.md`).

| What the plan assumes | What the car does today | Where |
|---|---|---|
| a PI speed loop on wheel speed | throttle is **open loop**: speed ÷ 3.0 | `legacy_protocol.py:91` |
| braking we can command | the brake comes on **only below 0.05 m/s**, at a fixed 0.3 | `legacy_protocol.py:89-90` |
| ego's deceleration reaches the car | ego's acceleration field is **never sent on** | `legacy_protocol.py:17-18` |

So on an A-to-B stop, the car mostly **coasts** and brakes only in the last 5 cm.
A step run logged today measures coasting, not braking. Any braking plan made upstream
is lost below ego.

Even on the Rust spine, braking is weak:

- It is **P-only** on overspeed, with a 0.2 m/s coast band (`speed_loop.rs:202-239`).
- At 1 m/s it gives only 0.4 brake.
- DRIVE_CMD's `accel_limit` only limits speeding up (`spine.rs:504`).
- So the firmware's own rule, not a predictive controller, decides where the car stops.

**The fix comes first, before any controller:** a way for a wanted deceleration to reach
the brake. Two options:

- in `to_legacy`, today's car;
- a signed acceleration or brake field in `aav.dbc`, for the spine.

Both are team and hardware-lead decisions, and both need the CLAUDE.md steps for a
contract change.

### 2. Half of it is already built (zoom-out, codebase, structural, comprehension)

| The plan says "build" | Already in the repo |
|---|---|
| a predictive braking controller | `aav_goto/approach.py`: `v = √(2a(d − lead·v))`, corrected every tick from the distance left; a creep speed; a 5 cm stop band. In Gazebo, every stop was within 0.4 m of B, with no overshoot (`docs/a-to-b.md` §4) |
| our own path follower | `aav_goto/line.py`: pure pursuit, in Python. **A to B never uses MPPI.** |
| system identification | `pixi run actuator-test`: `steer-sweep`, `steer-rate`, `steer-centre`, `speed-up`, `coast`, `brake`. `analysis.py` fits them and prints the `vehicle.yaml` lines; `with_measured()` feeds the model |
| a speed loop design | `SpeedLoopConfig::smooth()` and `place_speed_gains` (pole placement), in `speed_loop.rs` and `vehicle-model.md` |
| an LQR | `vehicle_model.lateral()` gives A and B; scipy's `solve_continuous_are` does the rest (`vehicle-model.md`, "After state space") |

What is really missing:

- **measured numbers:** `max_brake`, `brake_lag`, `throttle_lag` and `drag` are all GUESS in `vehicle.yaml`;
- **a measured `brake_lead`:** it is set by hand to 0.8 s;
- **dead time in the model:** it has lags, but no pure delays.

### 3. We can't yet measure what we want to fit (codebase, readiness, design)

| To fit | Why we can't today | Who unblocks it |
|---|---|---|
| steering angle, lag, play | no angle sensor; the uplink (H9) and the Jrk's feedback (H2) aren't confirmed | the hardware lead |
| steering rate | `steer-rate` lifts ego's cap, but the spine still caps it at 0.6 rad/s, so the test measures the cap | us: a direct command |
| brake and throttle lag | the bridges publish the commands as NaN, so `fit_lag` returns nothing | us: log the commands |
| exact timing | the CSV is resampled at 50 Hz with no stamps of its own; `fit_lag` has no dead time | us: `--bag`, a dead-time term |
| slow stops | wheel speed comes from Hall pulses; the rotor's hole count is unknown. With 8 holes, a pulse every 0.12 m means 0.4 s between pulses at 0.3 m/s, slower than the lags | count the holes (S8b); add IMU `ax` |

**Plant boundary.** The tests drive through twist_mux, ego and the spine. A fitted
"lag" then also holds ego's and the spine's limits. Gazebo and `vehicle_model.py` would
count those limits twice. Each fit must say which signal it goes from and to: for
example, `/vehicle/throttle` to speed, not `cmd_vel` to speed.

### 4. GPS error may be bigger than control error (zoom-out, readiness, comprehension)

**The numbers:**

- **The receiver:** 1 fix a second from the factory, 1–3 m off without RTK.
- **The global EKF:** up to 0.7 m off in Gazebo, with 0.5 m of GPS noise.
- **Gate 4:** asks for a stop within 1 m.
- **The stop itself:** from 1 m/s it takes 0.33 m. A 0.1 s error in the brake lag costs about 0.1 m.

**So:** the GPS that drives the car can't also judge where it stopped. Measure each stop
**with a tape from a chalk mark at B**. Compare that with what `goto` reports as the
distance left. If the control error is the small part, the effort belongs in GPS
instead:

- a 5 Hz receiver setting;
- RTK (the hardware lead's call);
- `gps_heading`.

### 5. The scoring tool scores the wrong task (all six)

- `controller-compare` sends Nav2 goals on lot tours with turns, and knows only `mppi` and
  `rpp`. It can't run `aav_goto` or a driver of ours.
- Gazebo's car runs `vehicle.yaml`'s guessed numbers. A model-based controller scored
  there is checked against its own assumptions.
- Gazebo also steers 0.11 s later than `vehicle.yaml` says (TODO 52c). **The fix is on
  `wt/goals-steering`, which isn't merged.**

**Needed:**

- an A-to-B scorer on `drive_metrics.py`: stop error along the track, overshoot past B,
  speed error while holding, peak deceleration, jerk;
- the same scorer for recorded car runs (replay);
- at least 3 runs per variant (`path-followers.md` treats one-run gaps under 20% as noise).

### 6. Structure and upkeep (structural, design)

- **Three readers.** `vehicle.yaml`'s `actuators:` is read by three places: `actuator_model.py`, `vehicle_model.py` and `vehicle.ego_params`.
  - A new key (play, dead time) could reach only one of them.
  - Add a test that fails on any key a reader doesn't account for (the `NOT_LINEAR` pattern).
- **Gains go elsewhere.** `vehicle.yaml` holds measurements only; gains go in `config/<node>.yaml`. Each fitted value cites its date, its run and the fit error.
- **A rule to settle.** `vehicle_model.py` "never drives the car" (the `aav_ego` README). An LQR built from it at run time breaks that rule. Either move the maths to a package with no ROS imports, or change the rule on purpose.
- **Decision D4** (stop-at-distance in ego or in `aav_goto`, `plan.md`) is still open. Settle it, then extend `approach.py` in place.
- **Python and LQR first.** An MPC in Python at 50 Hz risks pauses on the Orin, and needs a QP solver in pixi and the car kit. Use Rust only if timing on the Orin calls for it.

### 7. Safety before any step run (readiness, comprehension)

- **No timeout on today's ESP32.** It has no command timeout and no e-stop input, so the last throttle stays on.
- **The e-stop:** what the Kar-Tech e-stop cuts is unknown (S1).
- **Sign-offs not started:** the session test card (S-1) and written permission for the lot (S-2).
- **Run every new step test inside `actuator-test`,** for its speed envelope, typed confirmation and logging. Write a one-page checklist: who holds the e-stop, stands or lot, test order, `--max-speed 0.5`.

### 8. Words that need pinning down (comprehension)

| Word | In the docs | To decide |
|---|---|---|
| play | `steer_deadband` | deadband, or backlash in the linkage? |
| lag | τ (time constant) or a pure delay | both: τ and dead time |
| throttle gain | `max_accel`, K = a_max/d, `throttle-map` | one name |
| set speed | `cruise_speed`, `v_set`, `target_speed` | one name |
| predictive | MPC's prediction, or predicting the braking point | the braking point (here) |
| LQR, MPC, MPPI | MPPI only as "2000 futures" | a short primer with a picture |

The "three loops" picture is really five (`learn-control.md` §3): path, speed, steering,
**brake**, plan. The stop depends most on the brake, which the idea never named.

## Goals vs. Non-Goals

**In scope:**

| Goal | How we'll know |
|---|---|
| Stop on B | ≤ 1 m from a taped mark, 5 runs, no takeover (Gate 4); never past B |
| Hold the speed | overshoot < 3 % (`actuator-tests.md`) |
| Stay on the line | mean off-path ≤ today's 0.12–0.18 m (Gazebo) |
| A wanted deceleration reaches the brake | a logged brake level that follows it |
| The car's numbers, measured | every `actuators:` GUESS replaced, with its run and fit error |
| An A-to-B scorer | sim and recorded runs, ≥ 3 runs per variant |
| LQR as an option in `line.py` | scored against pure pursuit, then against MPPI |

**Out of scope, for now:**

- rewriting the Jrk's steering loop;
- rewriting the spine's PI before the flash and the stands data;
- an MPC, until LQR is shown to be the limit;
- a Rust controller;
- replacing MPPI on the Nav2 lot tours;
- raising the 1 m/s speed cap.

## Key Decisions & Staging for Next Steps

**Decisions to take into `/egm-draft`:**

1. **Which car is the target?** Today's ESP32, or the Rust firmware once flashed (Gate 2, 27 Nov)? The fits only hold for one firmware. Tag each run with its firmware in `meta.yaml`.
2. **How a deceleration reaches the brake:** `to_legacy` today, a signed field in `aav.dbc` for the spine, or both. These are the team's and the hardware lead's calls.
3. **D4:** stop-at-distance stays in `aav_goto` (recommended), not ego.
4. **Plant boundary:** fit from the actuator commands to speed, from a bag, never through ego.
5. **Where the model maths lives,** if a driver uses it at run time.

**Stages** (each one ships on its own):

| Stage | Work | Needs |
|---|---|---|
| S0 | no code: H2/H9, count the rotor holes, S1, S-1, S-2, Gate 2 | the hardware lead |
| S1 | offline: merge `wt/goals-steering` (52c); the A-to-B scorer; log the commands; dead time in `fit_lag` and the model; a test that the fitting code survives play, coarse wheel speed and an IMU-only brake, using Gazebo with faults put in | nothing |
| S2 | on stands: steering tests with the uplink; raw CAN recorded | S0 |
| S3 | on the lot: `coast`, `speed-up`, `brake` at 0.5 and 1.0 m/s; one `vehicle.yaml` PR per test | S0, S2 |
| S4 | braking: measured `brake_lag` and `max_brake` in `approach.py`, then a model-based stopping distance; scored by the taped stop | S3 |
| S5 | LQR from `vehicle_model.lateral()` as a `line.py` option; scored against pure pursuit, then MPPI | S1 |
| S6 | the inner loops, only if S4 and S5's leftover error traces back to them | S4, S5 |

**Before S4, check the error split.** If the taped stops show GPS error is the larger
part, move the effort to GPS (5 Hz, RTK, heading) before the control work.
