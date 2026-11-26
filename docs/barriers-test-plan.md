# Barriers: the test plan

**Outside the capstone's scope (Ben's own research, toward his MASc thesis).** How we test the barriers in the MPC
([the page](barriers.md)) step by step, so a result is a number against a mark set
beforehand, not a feeling. Written 22 November 2026 by Ben Woodhead. The car's own ladder
(bench, stands, hallway, lot) is [the testing plan](testing-plan.md): this page hangs on it.

> **The short version**
> 1. **Before a run, write down:** the claim, the baseline, the pass mark, the repeats.
> 2. **First show the simulator is the car** (part 1). Until then a simulator result says
>    nothing certain about the car.
> 3. **Then climb the barrier's own rungs** (part 2). A rung opens only when the one below
>    has passed.
> 4. **Then try to break it** (part 3).
> 5. Every pass mark here is **proposed**: Ben sets them, with his professor's advice.

## The words

| Word | Here it means |
|---|---|
| Verify | the code does what the maths says (unit tests) |
| Validate | the model and the simulator behave as the car does |
| Claim | one sentence a run could show to be wrong |
| Baseline | the same run without the new thing: the plain tracker, and the distance limit alone (the "step") |
| Pass mark | the number that decides, fixed before the run |
| Repeat | the same run again, with a new seed, to see the spread |

## Part 1: is the simulator the car?

The same test on the car, in Ravensim and on the pretend car (the `car_model` crate). The
car's side is the testing plan's cards 2, 4 and 6.

| # | Test | Measure | Match mark (proposed) | Repeats | State |
|---|---|---|---|---|---|
| V1 | Speed step, 0 to 1 m/s | dead time; time to 63 % | each within 20 % | 5 | to do: the car's numbers are not measured |
| V2 | Brake from 1 and 2 m/s | stopping distance | within 10 %, and the car never longer than the model plus 0.2 m | 5 | to do |
| V3 | Steering step, 0 to 0.2 rad | dead time; rate | each within 20 % | 5 | to do |
| V4 | Coast from 2 m/s | distance to rest | within 20 % | 5 | to do |
| V5 | A circle, fixed steering, 1 m/s | radius | within 5 % | 3 | to do |
| V6 | The stop rule's curve S(v) | S(v) against V2's distances | S(v) at or above every measured stop | from V2 | to do |

When a test misses its mark: change the model's numbers (`vehicle.yaml`, the lags), run it
again, and write both results down. The barrier's rungs below are then run again too.

## Part 2: the barrier's rungs

| Rung | Where | The claim | Baseline | Pass mark (proposed) | Repeats | State |
|---|---|---|---|---|---|---|
| R0 | unit tests | the rule's rows and the stop rule do what `barrier.rs` says | none | all pass, every commit | every commit | **passes** |
| R1 | pretend car, a sweep | the body never meets the object, at any speed, place or dead time swept | plain; step | 0 touched; least gap 0.2 m or more | 1 a point (it has no noise) | **passes**: 0 of 210, least gap 0.21 m |
| R2 | pretend car, wrong on purpose | the same, with noise and the model's numbers off (part 3) | plain; step | 0 touched | 20 seeds a point | to build |
| R3 | Ravensim, whole stack, ghost ahead, 1 m/s | the stop rule acts before the stop zone, and the car stands clear | `mpc_lag_rs` alone; `mppi_lag` | 10 of 10: stood 0.5 m or more before the ghost, stop rule first | 10 each | **partly**: 3 runs, 0.76 to 0.88 m, stop rule first each time; and 2 more with `barrier-watch`, 0.75 and 0.80 m, stop rule first (measured from the body on the simulator's truth, not from the ghost's scan) |
| R4 | Ravensim, 3 m/s, ghost beside and on the path | it goes round and comes back to the path | the same two | 10 of 10 reach the goal; least gap 0.3 m or more | 10 each | **partly**: built (`pixi run barrier-watch barrier --speed 3`, [the page](barriers.md#watch-it-in-ravensim)). On the line: 11 of 11 went round and reached the goal, least gap 0.75 m. 0.4 m beside: 3 of 3, 0.71 m. 1.0 m beside: 2 of 2. Baselines, on the line and 0.4 m beside: plain touched 7 of 8; the step stopped short 6 of 7. Not yet 10 each in one set; `mppi_lag` not run |
| R5 | bench: the spine in the loop, no car (`sim spine:=true`) | the same as R3 through the real spine logic | `mpc_lag_rs` alone | as R3 | 10 | to do |
| R6 | the car on stands, a ghost | the wheels stop when the ghost is placed | none | the wheels stop within S(v)'s time, 5 of 5 | 5 | to do: needs part 1's V1 and V2 |
| R7 | the car in the lot, a ghost only, walking pace, a safety driver | as R3, on the ground: nothing real to hit | `mppi_lag` | 5 of 5: stood 0.5 m or more before the ghost | 5 | to do: needs R3, R5, R6, part 1, and the testing plan's card 6 |
| R8 | the car in the lot, a cardboard box | as R7, the box seen by the car's own sensors | `mppi_lag` | 5 of 5, no touch | 5 | later: needs obstacles from the costmap |

The car rungs follow the testing plan's safety rules, and the hardware lead decides what
the car is ready for.

## Part 3: trying to break it

Each row is one thing made wrong on purpose, on rung R2 first, then R3.

| # | What is wrong | How | The claim |
|---|---|---|---|
| B1 | The car is slower than the model says | lags and dead time up 30 % and 60 %, the tracker not told | 0 touched |
| B2 | The brakes are weaker | braking down 30 % | 0 touched, or the least speed at touch written down |
| B3 | The car is not where it thinks | 0.1 and 0.3 m of noise on the pose | 0 touched |
| B4 | The object is not where it seems | 0.1 and 0.3 m of noise on the object; a marker that comes late | 0 touched |
| B5 | A tick is missed | the tracker skips 1, 2 and 5 ticks | 0 touched |
| B6 | The solver runs out of turns | the limit cut to a half and a quarter | 0 touched: the stop rule must carry it |
| B7 | The object moves | across the path at 0.5 and 1.5 m/s | written down only: the rule assumes it stands still |
| B8 | Two objects, a gap between | gaps of 2, 3 and 4 m | 0 touched; which gaps it takes |

## Part 4: what every run keeps

| Thing | Where |
|---|---|
| The commit, the command, the seed, the settings | the first lines of the run's CSV |
| The result: touched, stopped, moving or reached; the least gap; the time | one line a run (`sweep.csv`'s form) |
| The whole run, for a simulator or car run | a bag |
| A failed run | kept and written into [the page](barriers.md) as a finding, as a pass is |

## What is known today, against this plan

| | |
|---|---|
| Passed | R0, R1 |
| Partly | R3 (5 runs of 10); R4 (11 runs on the line, 3 and 2 beside: all went round, none touched) |
| Against it | going round needs 2.5 m/s or more, and the car drives 1 m/s; the go-round brakes to 0.3 to 1 m/s first; the solver is unsettled on most ticks with a barrier near (B6 matters) |
| Not started | part 1, R2, R5 to R8, part 3 |
