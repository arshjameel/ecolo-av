# The control plan: speed up, hold, stop on B

* **Author:** Ben
* **Date:** 2026-10-27
* **Status:** the plan (Ben, 27 Oct: "Lets save this as our plan")

## The idea in one line

Keep **MPPI** for where to drive. Make the car **do what it's told** underneath it: one
speed-and-brake loop with **feed-forward plus feedback**, and the Jrk's own steering loop,
tuned. LQR and MPC are compared against this, not built first.

## The layers

```
where to drive   MPPI (Nav2), 20 Hz, Orin        ← position (GPS, IMU, EKF), the costmap
                 (A to B tests: pure pursuit today; LQR and MPC to compare)
      │  speed + acceleration + steering angle
      ▼
speed and brake  feed-forward + PI on wheel speed  ← wheel speed (Arduino CAN), IMU
                 now: the ROS bridge, 50 Hz; later: the ESP32 (Rust spine-core), 100 Hz
steering         the Jrk G2's position PID, ~1 kHz ← the steering motor's position
```

## Why

| Finding | So |
|---|---|
| MPPI assumes the car does what it's told: its model has no lags | the inner loops matter more than the choice of outer controller |
| Today's throttle is open loop (speed ÷ 3.0) | close it on wheel speed |
| Today's brake comes on only below 0.05 m/s, at a fixed 0.3 | the car coasts to B: braking has to follow what's wanted |
| ego sends the acceleration it wants; the bridge drops it | use it as the feed-forward (option 3b in `brake-path.md`) |
| The C++ ESP32 drives the throttle and brake values our bridge sends | we can start in ROS, before any firmware change |

## Speed and brake: two-part control

| Part | Does |
|---|---|
| **Feed-forward** | from the model: the throttle or brake that the wanted acceleration needs, sent at once (`vehicle_model.py`: `max_accel`, `drag`, `max_brake`) |
| **Feedback** | PI on the speed error, cleans up what the model got wrong (slope, wind, wear); anti-windup |
| **One signed command** | + throttle, − brake; never both: a small dead band between them; braking has its own gains |
| **Sensors** | wheel speed (count the rotor's holes; time between pulses at low speed); IMU acceleration to help while braking |

## Steps, in order

| # | Step | Where | Scored by |
|---|---|---|---|
| 1 | The bridge uses ego's acceleration: feed-forward + PI for throttle **and** brake | `aav_legacy` (`legacy_protocol.py`), Gazebo, bench | `atob-score` (stop error, overshoot, hold) |
| 2 | Measure the car: speeding up, braking, steering (the actuator tests); the real numbers into the feed-forward and into MPPI | stands, then the lot | the fits' errors |
| 3 | Move the loop into the ESP32 (Rust `spine-core`), 100 Hz; LQR picks the gains if it beats hand tuning | firmware: **the hardware lead's call** | bench, `atob-score` |
| 4 | Steering: confirm the Jrk has position feedback (H2), tune it with step tests, match MPPI's steering-rate limit | the Jrk | step response |
| 5 | Compare: LQR for steering, MPC for the whole A to B, against MPPI and pure pursuit | Gazebo, then the car | `atob-score`, `controller-compare` |

## Not in this plan

- Replacing MPPI on the obstacle tours.
- Rewriting the Jrk's loop.
- A learned (Mamba-style) model: after hours of car logs (`TODO.md`, "Ideas to keep").

## Related

`docs/designs/brake-path.md` (the options for braking), `docs/designs/concepts_brief.md`
(the stages S0-S6), `docs/vehicle-model.md` (the model), `docs/learn-control.md` (the basics).
