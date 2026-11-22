# Barriers in the MPC: keeping the car off things

!!! warning "Research, separate from the capstone"
    Ben's own work, not a team deliverable. Off by default: nothing in the stack turns
    it on, and no safety lock depends on it. Tested on a pretend car and compiled
    against ROS. **Not yet run in Ravensim, and never on the car.**

!!! note "The short version"
    - **Problem:** the path follower follows its path, even into something on it.
    - **Idea** (Ali, Shen, Hashim, 2024): a **control barrier function** in the MPC. The
      room to a keep-out circle may shrink only by a share each tick, so it never
      reaches zero.
    - **Built:** the barrier in `aav_mpc_lag`, a stop rule behind it, and the ghost as
      its first obstacle (`mpc_barriers:=true`).
    - **On a pretend car:** the plain tracker ends 0.5 m inside the object; with
      barriers, none of 36 runs touches it (smallest gap 0.22 m).

To learn the idea from nothing: [the walk-through](barriers-guide.md), then
[the survey](barriers-survey.md).

## 1. The rule

A **keep-out circle** round the object: its radius, the car's own (1.12 m from the
body's middle to a corner), and a margin (0.2 m).

```text
   H = (the car's middle to the circle's centre)² − radius²      H > 0: outside

   every tick of the plan:    H(next)  ≥  (1 − γ) · H(now)
```

| γ | Behaviour |
|---|---|
| 0.02 | turns away early and wide |
| 0.05 (the default) | later and closer |
| 1 | only "never inside": as late as a plain distance limit |

If H starts at 0 or more and the rule holds every tick, H is never negative (six lines
of induction: the walk-through, step 5).

## 2. The stop rule behind it

The rule above is kept by the **plan**. The car can break it (section 5, finding 1). So
each tick the tracker also asks:

> Holding my steering as it is, would I meet the circle within the distance I need to stop?

If yes, it asks a stop now.

| Part | From |
|---|---|
| The distance to stop, S(v) | the one stopping law (`ego_core::stop_law::StopCurve`): ego's, the behaviour planner's and the safety filter's |
| "Holding my steering" | the bend the car is on now, checked every 5 cm |
| The tracker's own wait | its dead time and two ticks |

## 3. Two forms of one idea

| | The safety filter (`aav_safety_filter_rs`) | The barriers here |
|---|---|---|
| Where | in front of twist_mux, on the computer drivers' commands | inside the path follower |
| Keeps | room to stop before the road's end; inside the lane | off a round object |
| Can it steer round? | no: it slows and trims | yes |
| How | a formula a step, no solver | rows of the MPC's problem |
| State | a node, in shadow | a node switch, off by default |

They share the stopping law. Both are on this branch.

## 4. How to run it

```sh
pixi run sim world:=test_lot tracker:=mpc_lag_rs ghost:=true mpc_barriers:=true
pixi run ghost --local ahead 10      # place the ghost 10 m ahead
```

| Piece | Where |
|---|---|
| H, its slope, the bend held, the room to stop | `ros/drivers/aav_mpc_lag/crates/mpc-lag/src/barrier.rs` |
| The rule in the plan; the stop rule | `tracker.rs` (`set_barriers`, `fence`) |
| Forward stretches only | `geared.rs` (`set_things`) |
| The ghost's marker to circles; the status keys | `crates/mpc-node/src/main.rs` |
| The tests | `tests/barrier.rs`, and `cargo test --release -p mpc-lag --test barrier -- --ignored --nocapture` for the whole table |

The status (`/drivers/nav/track/status`) gains `barrier_m`, `barrier_slack_m2` and
`barrier_stop`.

## 5. What happened on the pretend car

A straight 40 m path, a round object (0.5 m radius) at 15 m. `gap`: metres between the
object and the nearest point of the car's body.

| Object | Cruise | Plain tracker | Barriers, γ 0.05 | With 0.2 s dead time |
|---|---|---|---|---|
| 0.4 m left of the path | 1 m/s | **hit** (−0.50) | stops short, 0.28 | stops short, 0.24 |
| 0.4 m left | 3 m/s | **hit** (−0.50) | goes round, 0.90 | goes round, 0.98 |
| on the path | 1 m/s | **hit** (−0.50) | stops short, 0.34 | stops short, 0.32 |
| on the path | 3 m/s | **hit** (−0.50) | goes round, 0.85 | goes round, 0.72 |
| 1.5 m beside it | 1 m/s | passes at 0.50 | passes at 0.72 | passes at 0.75 |
| 1.5 m beside it | 3 m/s | passes at 0.50 | passes at 1.08 | passes at 1.24 |

γ, on the object 0.4 m left at 3 m/s:

| γ | Turns away at | Widest off the path | Gap |
|---|---|---|---|
| 0.02 | x = 6.1 m | 2.62 m | 1.76 m |
| 0.05 | x = 10.6 m | 1.54 m | 0.90 m |
| 1 | too late to steer: it stops | 0.07 m | 0.27 m |

| # | Finding | Why it matters |
|---|---|---|
| 1 | **The barrier alone crept through.** The first build slowed nicely, then crawled into the object at 0.15 m/s | the spine's speed loop only coasts through a small overspeed; a linear plan can't see its stop. The maths was right; the actuator broke its assumption. Hence the stop rule |
| 2 | **Slow is harder than fast.** At 1 m/s it stops instead of going round | the plan sees 1.5 s = 1.5 m; the keep-out circle is 3.6 m wide |
| 3 | **Safe, and stuck.** A stopped car stays stopped | a barrier only pushes away: choosing a side is the planner's job |
| 4 | **A straight look ahead stops a car that is turning away** | with the real stopping law it stood still mid-swerve; the rule now looks along the bend |
| 5 | **The solver works harder**: a tick 0.3 to 0.5 ms without, 3 to 15 ms with a barrier near; it often ends at its iteration limit | inside the 50 ms tick; a better solver setting is to do |

## 6. What it assumes

| Assumption | True on the car? |
|---|---|
| the model matches the car | roughly: the lags are guesses until measured |
| the plan's H is the real H | no: H is made linear each tick about the last plan |
| the car stops as S(v) says | S(v) is ego's law; its numbers are not measured on the car |
| the car holds its steering while it stops | roughly |
| the car knows where it is, and where the object is | as well as localization and the ghost do |
| the object is a circle that stands still | a first shape only |
| the path is straight beside the object | else the path's frame bends the circle |

So: shown on a pretend car, argued for the model. **Not proved for the car.**

## 7. Not done

| | |
|---|---|
| A run in Ravensim, and a score against `mppi_lag` | next |
| Real obstacles (the costmap, `things`) as circles | the ghost is the only source |
| The stability rule (a Lyapunov rule in the plan) | not started; the safety filter has one for the lane |
| A proof with the dead time | open |
| Moving objects | open |
| Backwards | no barriers on a backwards stretch |
