# Our own velocity smoother

Ben, 8 Nov: a velocity smoother that **knows it drives a car**, beside Nav2's, as the
challenger at first. The default since 17 Nov, when Nav2's went
([as it was](history/nav2-as-it-was.md)); the comparisons with Nav2's below are the record.

> **The short version**
> - **Switch:** `smoother:=own`. Package `aav_smoother` (Rust).
> - **What it is:** Nav2's `velocity_smoother` for the speed (limits, acceleration, the
>   timeout, the deadband, open or closed loop), and the car's steering for the turn.
> - **Drop-in:** the same node name, parameters and topics. Nothing downstream can tell.
> - **Test lot, 3 runs each (all ours):** the same legs (10 of 12); ego clipped
>   **0-0.1 %** of the steering against **9-15 %** behind Nav2's; off the path 0.12 m
>   against 0.15 m; the tour 701 s against 890 s. Campus not run yet.

## Why

Nav2's smoother treats `v` and `w` as two separate dials, like a robot that turns on the
spot. A car can't:

| A car's limit | Nav2's smoother | Ours |
|---|---|---|
| The tightest turn at this speed: `w ≤ v tan(full lock) / L` | doesn't know it | never asks for more |
| How fast the steering actuator turns the wheels | doesn't know it: `w` jumps | the steering angle moves at most that fast |
| Slowing down keeps the same path | scales `v` and `w` together | keeps the bend `w / v` |
| A turn too tight for `max_velocity[2]` | cuts `w`: the car bends less | slows the car |
| Speeding up and braking | nav2.yaml's numbers | nav2.yaml's, never above the car model's |

Every car number comes from `vehicle.yaml` through the car model; the steering rate is
the tighter of the steering actuator's (`actuators.steer_rate`) and ego's ramp.

## How it fits

```
follower (MPPI, mppi_lag, ...) ─▶ cmd_vel_nav ─▶ velocity_smoother ─▶ stop zone ─▶ /drivers/nav/cmd_vel
                                                  (Nav2's, or ours
                                                   under its name)
```

| Name | Type | For |
|---|---|---|
| `cmd_vel_nav` | TwistStamped | in: the follower's command |
| `/control/cmd_vel_smoothed` or `/drivers/nav/cmd_vel` | TwistStamped | out: to the stop zone, or to twist_mux |

**How the swap works.** `navigation.launch.py smoother:=own` starts `car_smoother` with
Nav2's node name, `velocity_smoother`, so it reads the same block of the tuned
`nav2.yaml` and gets the same remappings. The lifecycle manager no longer waits for it:
ours is not a lifecycle node.

## One tick (20 Hz)

| Step | What |
|---|---|
| 1 | No command for `velocity_timeout`: slow to a stop, then say nothing (Nav2's) |
| 2 | The wheels' target: `atan(L w / v)`, within full lock (held when `v` is about 0) |
| 3 | The speed: within `min/max_velocity`, slowed so the turn stays under `max_velocity[2]` |
| 4 | The speed moves at most `max_accel` faster, `max_decel` slower (Nav2's rule) |
| 5 | The wheels move at most `steering_rate` toward their target, once the car moves |
| 6 | Out: `(v, v tan(steer) / L)`; the deadband sends small values as 0 |

## Try it

| What | Command |
|---|---|
| See it | `pixi run sim smoother:=own` |
| Score it | `pixi run measure controllers --stacks own --smoothers nav2 own --tours lot --runs 3` |
| Its tests | `cd ros/drivers/aav_smoother && cargo test -p car-smoother` |

The score's new row, **"Steering commands ego had to clip"**: the share of ego's steering
commands more than 0.03 rad from the angle it was asked for (`atan(L w / v)` of
`/control/cmd_vel`). Fewer is better: the car was asked for what it can do.

## Scores (8 Nov, our simulator)

All ours (`navigator:=own planner:=own costmap:=own tracker:=mppi_lag`), forward only, the
test lot at 1 m/s, **3 runs each**, behind Nav2's smoother and behind ours.
`pixi run measure controllers --stacks own --smoothers nav2 own --tours lot --runs 3`.

| | Nav2's smoother | Ours |
|---|---|---|
| **legs reached** | 10 / 12 | 10 / 12 |
| **steering commands ego had to clip** (mean of the legs) | **8.6-14.8 %** | **0-0.1 %** |
| off the path, mean | 0.15 m | **0.12 m** |
| off the path, worst | 0.92 m | **0.65 m** |
| sway | 0.056 m/s² | 0.054 m/s² |
| the whole tour | 890 s | **701 s** |

| Leg | Nav2's: reached, time | Ours: reached, time |
|---|---|---|
| a left turn | 3 / 3, 24 s | 3 / 3, 24 s |
| round the building | 3 / 3, **44-171 s** | 3 / 3, **44 s every run** |
| turn back (the U-turn) | 1 / 3 | 1 / 3 |
| home | 3 / 3, 59-71 s | 3 / 3, 60-70 s |

- **Ego hardly ever clips ours.** Nav2's smoother passes on steering the wheels can't
  reach that fast; ours asks only for what the steering can do.
- **Steadier:** "round the building" took 44 s every time with ours; with Nav2's, two
  runs of three wandered (106 s, 171 s).
- **The U-turn is still the hard leg** (forward only, a 14 m lane): the follower and
  planner's problem, not the smoother's.
- **Not run yet** (cut short for car day, 9 Nov): the campus at 2.5 m/s, and
  `tracker:=mppi_lag` under Nav2's navigator. `--stacks own --smoothers nav2 own
  --tours campus --speed 2.5 --runs 2` and `--trackers mppi_lag --smoothers nav2 own`.
- Ours is the only smoother since 17 Nov, in the sim and on the car.

## Where it came from

Nav2's `nav2_velocity_smoother` (Apache-2.0): what was kept and what is ours is in
`ros/drivers/aav_smoother/PROVENANCE.md`.
