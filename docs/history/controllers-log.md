# Controllers: the dated results

Moved from [Controllers](../controllers.md) on 18 Nov (TODO 172: shorter pages), word for
word: the speed loops' runs of 29 Oct, in Gazebo. The page keeps what each controller is
and which one we use.

### 2a. LQI against the PI: results, 29 Oct

`speed_loop:=lqi` against today's PI (`default`) and TODO 27's `smooth`. Every number is
on guessed car numbers (`vehicle.yaml`), so it ranks the designs; it doesn't tune the car.

**On the model** (`pixi run python3 tools/vehicle_model.py --speed-loop`: the worst of 6 runs, 0.5 to
2 m/s, the lag doubled, up to 110 ms late):

| Plant | Today's PI: overshoot / within 5 % | `smooth` | `lqi` |
|---|---|---|---|
| K (`vehicle.yaml`'s guesses) | 17 % / 13.7 s | 0 % / 5.7 s | **0 % / 5.0 s** |
| K/5 by drag | 0 %, 29 % short | 0 % / 11.1 s | **0 % / 8.3 s** |
| K/5 by a weak throttle | 17 %, 29 % short | **8 %** / 15.3 s | 15 % / 15.0 s |
| 2K by a strong throttle | 15 % / 9.2 s | 1 % / 6.9 s | **0 % / 6.0 s** |
| Phase margin, worst | 54-95° | 42-75° | 44-73° |

**In Gazebo** (`sim-check --speed-steps-only --spine-steps --speed-loop <profile>`):
the steps, ramped at 1 m/s², held 20 s. Twice each: the actuator model's Python port,
then the spine in the loop (`spine:=true`: spine_sim runs spine-core's Rust). Both gave
the same numbers. Load average 5-10 (other helpers' runs on the Mac).

| Step | Today's PI: peak / within 5 % after | `lqi`: peak / within 5 % after |
|---|---|---|
| reverse 0.5 m/s | 0.579 (15.8 %) / 10.9 s | **0.500 (0 %) / 3.6 s** |
| forward 1 m/s | 1.151 (15.1 %) / 11.0 s | **1.000 (0 %) / 4.0 s** |
| forward 2 m/s | 2.203 (10.2 %) / 13.3 s | **2.000 (0 %) / 4.6 s** |

**On the bench's model car** (`cargo test -p bench-car scores -- --nocapture`: the spine
through the encoder): 1 m/s peaks at 1.125 (today), 1.000 (`smooth`), 1.000 (`lqi`); a
stop from 1 m/s rolls 0.65, 0.58 and 0.59 m.

**Drive to point, the spine in the loop** (`sim-check --spine-points-only --speed-loop
<profile>`: 4 drives at 1 m/s and 4 at 3 m/s each, scored by `measure atob` on Gazebo's
truth; load average 5-8):

| | Today's PI, 1 m/s | `lqi`, 1 m/s | Today's PI, 3 m/s | `lqi`, 3 m/s |
|---|---|---|---|---|
| Speed overshoot | 9-14 % | **0-0.5 %** | 0-6 % | **0-0.1 %** |
| Holding: mean error (+ fast) | +0.01 to +0.05 m/s | −0.03 to 0 m/s | +0.05 to +0.07 | −0.03 to −0.08 |
| Stop, past B | −0.13 to +0.29 m | **−0.03 to +0.16 m** | **+0.02 to +0.41 m** | +0.19 to +0.43 m |
| The last metre took | 1.6-1.9 s | 1.7-1.8 s | 2.0-5.1 s | **1.4-1.5 s** |
| sim-check | 4/4 pass | 4/4 pass | 3/4 (one +0.57 m past B by the EKF; the limit 0.5) | 3/4 (one +0.53 m) |

At 3 m/s the LQI stops a little further past B every time (+0.2 to +0.4 m): it brakes on
the speed error behind the spine's slowing ramp, where the PI brakes past its coast
band at once. A feed-forward of the ramp's slope (as the bridge's loop has) is the next
thing to try.

**What it shows:**

- The LQI is as calm as `smooth` and a little faster; both beat today's PI by far.
- Its weak spot: a throttle 5x weaker than the model. Its gains assume `max_accel`, so a
  weaker car turns every gain down (15 % overshoot there, `smooth` 8 %). Measure the
  throttle (test 4.1) and redo the gains: `pixi run python3 tools/vehicle_model.py --set max_accel=... --lqi`.
- It brakes with the same law as it speeds up (no 0.2 m/s coast band, no P-only brake).
- The PI stays the default. Which one goes on the car is a car-day choice, after the
  actuator tests.

### 2b. The smooth stop: results, 29 Oct

In the bridge's speed loop (`speed_control.py`), after Autoware's smooth stop. The car runs
behind ego (the actuators' lag), so it still rolls when ego reaches 0. Before, the loop
then dropped to `hold_brake`: a step in the slowing, a long crawl, then a jolt.

```text
 slowing ▲  before: plan ──┐ step down to hold_brake ── crawl ──┐ jolt (0.4-0.6 m/s²)
         │  after:  plan ─────────── fades with the speed ────────╲ (0.2-0.4 m/s²) ─ firm hold
         └──────────────────────────────────────────────────────────────────────► time
                          ego reaches 0                       stopped
```

| Part | Value (guesses) |
|---|---|
| Keep slowing as it was when ego reaches 0 | no step in the brake |
| Then fade: slowing at most `smooth_stop_rate` × speed | 1.5 /s |
| Never less than `weak_stop_decel`, so it can't creep | 0.2 m/s² |
| Still rolling after `weak_stop_time`: at least `strong_stop_decel` | 2 s, 1.0 m/s² |
| Stopped: the brake rises at 1.0 /s to `stopped_brake`, and holds | 0.5 (was 0.2) |
| Off | `speed_loop.smooth_stop_speed: 0` |

**In Gazebo** (`sim-check --drive-to-point-only --esp32 pi`: the bridge, the C++
firmware's logic, the actuator model; Gazebo's truth, scored by `measure atob` and the
slowing in the last 0.2 s before standstill). 9 drives each; load average 7-13:

| | Before (`smooth_stop_speed: 0`) | After |
|---|---|---|
| Every check | pass | pass |
| Stop, past B (truth) | −0.16 to +0.24 m | **−0.04 to +0.13 m** |
| Slowing in the last 0.2 s, 1 m/s drives (the jolt) | 0.43-0.63 m/s² | **0.22-0.35 m/s²** |
| The last metre took | 2.4-7.0 s | **1.9-4.0 s** |
| Crept in the last metre (under 0.15 m/s) | up to 0.30 m | up to 0.20 m |
| Hardest slowing (plan 1.5) | 0.83-1.64 m/s² | 0.86-1.61 m/s² |
| Peak jerk in the last 1.5 s (0.5 s smoothing) | 0.4-2.1 m/s³ | 0.4-2.9 m/s³: **not better** |

- The jolt at standstill halves, the stop lands closer to B, and the crawl is shorter.
- The peak jerk near the stop didn't fall: in 4 of 9 drives it rose to 2-2.9 m/s³,
  probably where the stop starts from the loop's last asked slowing, which can be harder
  than the car's. Next: start from the measured slowing (the IMU, or the wheel speed's
  slope). On the model chain (`test_speed_control.py`) the jolt halves too.
- Only on the bridge (`aav_legacy`, `mode: pi`), not yet in `spine-core`.
- Credit: the design is Autoware's `pid_longitudinal_controller` (the Autoware
  Foundation, Apache-2.0), read in their docs. Our own code and numbers.
