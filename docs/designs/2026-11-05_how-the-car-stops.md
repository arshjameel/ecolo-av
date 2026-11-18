# How the car stops: one curve for every stop

* **Author:** Ben (the pick), written up 5 Nov 2026
* **Date:** 2026-11-05
* **Status:** Partly built (0.3, 7 Nov; checked 13 Nov): the one stopping law is `aav_description` `stop_law.py` (TODO 110, 137, 138; `docs/stopping.md`), used by the behaviour planner, the trackers and the point driver (`stop_law:=hybrid`). Not built: the filter (the bumper, before twist_mux); nothing on the car. The first of our own hybrids.

Ben, 5 Nov: "lets create the best algorithms using papers, known algos, current implemntations,
merge learnings from each and create our own algo."

## The idea in one line

One curve, `S(v)`: how far the car needs to stop from speed `v`. Turned around, it is a speed
limit for any distance left. Used as a bumper, it is the CBF filter. **Same curve, same numbers.**

```text
   S(v)  =  v·τ  +  v²/(2a)           τ = ρ + a/(2j)
            └┬─┘    └──┬──┘
       delay + easing  braking

   the cap:     v_cap(d) = a·( √(τ² + 2d/a) − τ )     the speed that stops in d metres
   the bumper:  h = d − M − S(v)                      h above 0: room to stop
```

## Problem

The car has five answers to "how far do I need to stop?", and they disagree.

| Where | Law | At 1 m/s | Used by |
|---|---|---|---|
| `approach.py` | `√(2a(d − lead·v))`, a 1.5, lead 0.8 | 1.13 m | the point driver |
| `speed.py` | the same, lead 0 | 0.33 m | the trial trackers |
| `geofence.py` | RSS: speeds up for ρ, then brakes | 1.4 m | the fence's lock |
| `dynamics.py` | `v·0.4 + v²/2a` | ego's log | ego's log |
| `stop_sign.py` | none: a fixed 4.7 m range | | the sign's lock |

## Goals and non-goals

| Goals | Non-goals |
|---|---|
| One law, in one module with no ROS | Replacing or weakening any lock |
| The same stop or better, with less hard braking | Steering, reversing or moving targets, for now |
| Used by the point driver first: the Gate 4 driver | Driving the car in `open` mode: it coasts |
| Beside the default until it wins | A claim that this is new science: see "What's ours" |

## What each source teaches

| Source | What we keep | What we leave |
|---|---|---|
| `approach.py` | slowing by distance left, so a late car corrects itself; the lead | its slope is infinite at the stop, and the lead needs the measured speed |
| `speed.py` | looking ahead for bends | |
| `speed_control.py`, from Autoware's design | a stop that fades, with no jolt | it runs only in the bridge's `pi` mode |
| [Autoware's velocity smoother](https://tier4.github.io/autoware_universe/latest/planning/autoware_velocity_smoother) | limits on acceleration **and** jerk | a whole-path optimiser: more than we need at 1 m/s |
| Ego's `max_jerk` | the number, 2 m/s³ | it never softens braking |
| [RSS](https://arxiv.org/abs/1708.06374) in `geofence.py` | ρ and b named apart; worst-case thinking, for the lock | too cautious to drive by |
| [CBF](https://arxiv.org/abs/1903.11199) | trim as little as possible, and earlier when there's room | a solver: one unknown needs only a formula |
| Simplex, Sha 2001 (not read: seen only through others' citations) | the simple layer stays, and stays independent | |

## What's ours

The pieces are all known. The merge is new to us, not a new result; nobody has checked the
literature for it.

| Step | What it gives |
|---|---|
| Add jerk to the distance | a stop that eases in and out needs exactly `v²/(2a) + v·a/(2j)` |
| Solve for `v` in closed form | no measured speed inside the law, so no noisy feedback in it |
| The slope at the stop is `1/τ`, not infinite | near the stop the speed falls smoothly with distance: the smooth stop comes built in |
| The cap is the set `h = 0` | the cap and the CBF filter can't disagree, as they share `S(v)` |
| The most it asks for on the curve is `v/(τ + v/a)` | always below `a`: 0.63 m/s² at 1 m/s with these numbers |

### A first look (model only)

A throwaway script, not Gazebo: a car with a 0.15 s dead time and a 0.2 s lag, stopping on a
line. The numbers are guesses and I chose them, so this only shows the law isn't broken.

| Law | From | Stop error | Peak braking | Time |
|---|---|---|---|---|
| Point driver's (a 1.5, lead 0.8) | 1 m/s, 10 m | −0.01 m | 1.2 m/s² | 12.3 s |
| **One curve** (a 1, j 2, ρ 0.35) | 1 m/s, 10 m | +0.02 m | **0.73 m/s²** | 11.6 s |
| `speed.py`'s (a 1.5, lead 0) | 1 m/s, 10 m | +0.29 m | 1.83 m/s² | 11.5 s |
| Point driver's | 3 m/s, 20 m | −0.01 m | 1.2 m/s² | 10.5 s |
| **One curve** | 3 m/s, 20 m | +0.02 m | **0.89 m/s²** | 9.5 s |
| `speed.py`'s | 3 m/s, 20 m | +0.51 m | 2.0 m/s² | 8.6 s |

The one curve stops 2 cm past the mark here, where the point driver's stops 1 cm short.

## The numbers

| Symbol | Means | Start | From | Replaced by |
|---|---|---|---|---|
| `a` | planned braking | 1.0 m/s² | `geofence.braking`: half the softer of ego's 2.0 and `max_brake` | test 4.3 |
| `j` | jerk | 2.0 m/s³ | ego's `max_jerk` | Gazebo, then the car |
| `ρ` | the normal path's delay: input's age, ticks, `brake_lag` | 0.35 s | a guess | test 4.3 |
| `τ` | delay plus easing | 0.60 s | ρ + a/(2j) | |
| Floor | the lowest cap sent to Nav2 | 0.3 m/s | [behaviour and speed](2026-11-05_behaviour-and-speed.md) | Gazebo |

`a` 1.0, not the point driver's 1.5: the law then never asks for more than the lock assumes.

| Distance left | 0.5 m | 1 m | 1.1 m | 2 m | 5 m |
|---|---|---|---|---|---|
| `v_cap` | 0.57 m/s | 0.94 m/s | 1.00 m/s | 1.49 m/s | 2.62 m/s |

**ρ is not the geofence's ρ.** The lock's 0.9-1.0 s is a fault path, with timeouts. This law is the
normal path, with none. The lock keeps its own.

## Who uses it, in order

| # | User | How | Final stop |
|---|---|---|---|
| 1 | Point driver (25) | `Approach` asks the curve, not `braking_speed`; `stop_law:=hybrid`, default `approach` | its own command of 0, inside `stop_band` |
| 2 | Trial trackers | `SpeedPolicy` the same way | the same |
| 3 | Nav2's followers | the behaviour node sends `v_cap(d)` as a speed limit, floored, never 0 | a lock, or Nav2's goal check |
| 4 | The CBF filter (later) | below | a lock, or its own 0 |

Rules for the Nav2 cap, from Nav2's source:

| Rule | Why |
|---|---|
| Never publish 0 as a stop | 0.0 means no limit (`filter_values.hpp:59`, `SpeedLimit.msg:5`) |
| Clamp to the base speed | a higher limit raises MPPI's envelope (`optimizer.cpp:506-510`) |
| Fall no faster than the car can slow | `wz` is cut at once, while `vx` lags |
| **The sign: slow down, then the lock (B2, proposed)** | `stop_sign.py:86-91` can't be told the car stopped |

The sign's `d` is its range less `stop_distance_m`: the geometry is in [behaviour and
speed](2026-11-05_behaviour-and-speed.md), as are long waits and every state.

## The filter: the same curve as a bumper

**Later:** it needs the closed loop and measured numbers. Part 3 of the old [knows
itself](2026-11-05_knows-itself.md) is now here. The monitor's `r` and the σ come from there.

```text
   ρ̂ = ρ + k·σ_ρ      b̂ = r·a − k·σ_b      k = 2      S uses ρ̂ and b̂
   h = room − L(v) − M
   allowed slowing:  v̇ ≤ (α·h − c) / (τ̂ + v/b̂)        τ̂ = ρ̂ + b̂/(2j)
   v_out = sign(v)·min(|v|, |last| + v̇·dt)      w_out = w·v_out/v
```

| Term | Value |
|---|---|
| `c`, closing speed | `v` for fixed things |
| `α` | 1.0 1/s to start; 1/α ≥ 2·ρ̂ so the delay doesn't eat it. Easing starts at `h = v/α` |
| Only ever reduces | `|v_out| ≤ |v|`, no change of sign; `w` scales, so the curve is kept |
| `h` below 0 | `v_out = 0`; no latch |
| A stale barrier | its allowed speed can fall, not rise, until data is back: releasing would speed up because a sensor died |

### Where it goes (K1, proposed; awaiting Ben)

Ego sees only `/control/cmd_vel`, after the mux, and a `TwistStamped` doesn't say who sent it. A
filter there would trim the safety driver. So it goes before twist_mux.

```text
 Nav2, point, AI ─► /drivers/<d>/cmd_vel_raw ─► filter ─► /drivers/<d>/cmd_vel ─┐
 joystick, RC, keyboard, tests, scripted, buttons, replay ─────────────────────► twist_mux ─► ego
```

| Option | People safe? | Verdict |
|---|---|---|
| **A relay on the automatic drivers' topics** | yes, by construction | **proposed** |
| In ego, with a "who's driving" signal | only if the signal is right and on time | no |
| In ego, as first written | **no** | no |
| Stock `FootprintApproach` (`nav2.yaml:596`, unused) | yes; Nav2 and the stop zone only | **try first** |

### The fence, worked through

The fence's lock closes at its own stopping path + 1 m + σ. So `h` must include that, or the car
parks inside NEAR, which at a standstill only a person ends.

`M` = margin 1.0 + σ_pose 0.3. `L(v)` = `S_lock(0)` 0.81 + buffer 0.5 + `S(v)`.
At 1 m/s on the spine, with σ_ρ 0.05, σ_b 0.1: ρ̂ 0.45, b̂ 0.8, `S(1)` 1.275, `L(1)` 2.585.

| Room to the fence | h | Who acts |
|---|---|---|
| 6.00 m | 2.115 m | nobody: the command passes |
| 4.885 m | 1.000 m | the filter starts easing |
| 3.885 m | 0 | the filter, still easing |
| 2.70 m, at 1 m/s | −1.185 m | **NEAR closes** (1.4 + 1.0 + 0.3), if the car somehow still moves |
| 2.61 m, standing | 0 | where the filter parks the car |
| 2.11 m, standing | | NEAR closes at a standstill: the car is 0.5 m clear of it |

These are by hand, not a run. A test checks `L(v) ≥ S_lock(v)` for every speed up to the cap, on
each backend, reading the real configs.

## Where it lives

| Part | Where |
|---|---|
| `stop_law.py`: `StopCurve(a, j, rho)`, with `distance(v)`, `cap(d)`, `demand(v)`, `h(room, v, margin)` | new package `ros/drivers/aav_stop_law`: no ROS, no node, own tests |
| `Approach`, `SpeedPolicy` | a small optional hook, default off |
| `geofence.py` | **unchanged, and it doesn't import this.** The lock stays independent (Simplex). Only a test reads both |
| The behaviour node, the filter | their own designs and packages; they import the curve |

## Safety

| Rule | How |
|---|---|
| Every lock stays as it is | this law only asks for less speed; it can't open a lock |
| Off by default | `stop_law:=approach` until it wins |
| Refused on the car in `open` | the car can't follow a braking law there, and the geofence holds programs anyway |
| A bad number fails loudly | `a`, `j` must be above 0, `ρ` not negative; `cap` of NaN or a negative `d` gives 0 |

## Risks

| Risk | What happens | Answer |
|---|---|---|
| `a` too high, or `ρ` too low, for the real car | it overruns the mark | the locks; first runs at low speed; numbers from test 4.3 |
| Gazebo flatters | it wins there and loses on the car | a win in Gazebo only earns a try on the car; bags can't score this |
| Slow to arrive: the speed falls as the car nears | it takes long to reach the mark, or stops short | `creep` for the point driver; the floor for Nav2; a check on time |
| The filter and the cap act together | double slowing | they share `S(v)`, and each only lowers |
| One shared curve means one shared error | a wrong `a` weakens them all | the lock keeps its own numbers and doesn't read this |
| The point driver isn't held by the stop zone | `stop_zone` is priority 21, the point driver 25, so this law won't stop for an obstacle on the Gate 4 drive | not new, and not solved here: question G2 |

## Alternatives

| Option | Why not |
|---|---|
| Keep each module's own law | the disagreement is the problem |
| `approach.py` everywhere | has no jerk term, and its peak braking is higher in the model |
| Autoware's whole-path optimiser | much more machinery for 1 m/s |
| A solver in the filter (`aav_track/qp.py`) | one unknown needs a formula; keep for steering, later |
| Merge the lock's law in too | breaks the lock's independence |

## Checks (none run yet)

| Test | Pass |
|---|---|
| `stop_law` unit tests | `cap(distance(v)) == v`; demand below `a`; slope finite at 0; bad numbers refused |
| `L(v) ≥ S_lock(v)` | every speed up to the cap, every backend |
| Gazebo, `measure atob`, 10 runs each: `approach` vs `hybrid`, at 1 m/s | stop error no worse; peak jerk lower; time within 10 % |
| `pixi run sim-check`, `hybrid` on | still passes |
| On the car, after `pi` and test 4.3 | the same drive, low speed first, with the hardware lead |

Ten runs, as three clean runs can pass by luck. `measure atob` has a jerk column already.

## Order

1. `stop_law.py` and its tests. A day.
2. The point driver's hook; Gazebo against `approach`. Days. **The win or loss here decides the rest.**
3. The trackers' hook.
4. Nav2's cap, with the behaviour node.
5. The filter, after the closed loop and measured numbers.

## Questions for Ben (all proposed; awaiting Ben)

| # | Question | Proposed |
|---|---|---|
| C1 | Is this the hybrid you meant? One curve, with the point driver first | yes |
| C2 | `a` 1.0 m/s², below the point driver's 1.5, until test 4.3? | yes |
| K1 | The filter before twist_mux, not in ego? | yes |
| B2 | The sign: slow down, then the lock? | yes |
| G2 | Should something stop the point driver for an obstacle on the Gate 4 drive? | yes, as its own item: ask the team |

## Related

[One car](2026-11-04_one-car.md), [behaviour and speed](2026-11-05_behaviour-and-speed.md),
[knows itself](2026-11-05_knows-itself.md), [the EGM review](2026-11-05_egm-review.md),
`docs/geofence.md`, TODO 101, 102, 104, 110.
