# From "bad things can happen" to a proof: a survey

**Outside the capstone's scope.** Ben Woodhead's own research, toward his MASc thesis. Not a team deliverable.

A map of the whole problem behind [the barriers in the MPC](barriers.md), simplest first.

**How to use it.** Each topic is one small card:

- **What it is**, in a sentence.
- **Why you need it** here.
- **On our car**: a real example.
- **Watch**: words to search for a video.
- **You have it when** you can answer one question.

Go top to bottom. Each card leans on the ones above it. The hands-on companion is
[the walk-through](barriers-guide.md) (twelve steps with a pencil).

| Part | Cards | Answers |
|---|---|---|
| A | 1 to 3 | What can go wrong, and why testing isn't enough |
| B | 4 to 8 | How we describe a car with maths |
| C | 9 to 12 | "Stable": it settles |
| D | 13 to 16 | MPC: planning ahead with rules |
| E | 17 to 21 | "Safe": it never touches |
| F | 22 to 25 | "Never stuck", and the real world's mess |
| G | 26 to 27 | Where learning fits, and what is still open |

---

# Part A. The problem

## 1. Bad things that could happen

| Bad thing | Looks like | Usual cause |
|---|---|---|
| It sways | wider and wider round the path | correcting too hard, too late |
| It cuts a corner | over the kerb | the plan ignores the car's width |
| It drives into something | a parked car, a person | the path was wrong, or old |
| It reacts too late | brakes, still hits | it only looked at distance, not speed |
| It freezes | stopped for ever in a clear road | a rule with no way out |
| It has no command | the computer has no answer | rules that can't all be met |
| It trusts a bad reading | swerves at nothing | a position jump, a false detection |
| It asks the impossible | "turn now" at full lock | forgetting the machine's limits |

Every card below exists to stop one of these.

## 2. Three promises

| Promise | Stops which bad things | Maths word |
|---|---|---|
| **Stable** | swaying | Lyapunov stability |
| **Safe** | driving into things, cutting corners, late reactions | forward invariance |
| **Never stuck** | no command; freezing | recursive feasibility; liveness |

**You have it when** you can sort each row of card 1 under one promise.

## 3. Why testing isn't enough

- A test shows it worked **on the routes we drove**.
- A proof shows it works **on every route that fits the assumptions**.
- Neither replaces the other: a proof is only as good as its assumptions, and tests
  are how you check those.

**Watch:** "verification vs validation autonomous vehicles".
**You have it when** you can say why 1,000 good runs don't prove the 1,001st.

---

# Part B. Describing a car

## 4. State

**What:** the short list of numbers that says everything about the car right now.
**Why:** every proof is a statement about how these numbers move.
**On our car:** how far left of the path, heading error, speed, steering angle, and
six more for the lags. Ten numbers.
**Watch:** "state space representation control" (Brian Douglas; Steve Brunton's
"Control Bootcamp").
**You have it when** you can say why position alone isn't a state. (Two cars at the
same spot, one stopped and one at 5 m/s, have different futures.)

## 5. Model

**What:** the rule for the next state from this one and the command.
`next state = f(state, command)`.
**Why:** to plan ahead, or prove anything, you must be able to predict.
**On our car:** the bicycle model: two wheels' worth of geometry stands for four.
**Watch:** "kinematic bicycle model".
**You have it when** you can say what the model leaves out (tyre slip, slopes, wind).

## 6. Feedback

**What:** measure, compare with what you want, correct, repeat.
**Why:** it is the loop that can go unstable, and the loop we make promises about.
**Watch:** "open loop vs closed loop control".
**You have it when** you can explain why a thermostat is feedback and a timer isn't.

## 7. Limits

**What:** what the machine can't do: steer past full lock, steer faster than its
motor, brake harder than its brakes.
**Why:** most proofs in textbooks quietly assume no limits. Ours can't.
**On our car:** full lock 0.5 rad; steering rate 0.6 rad/s.
**Watch:** "actuator saturation", "anti-windup".
**You have it when** you can say what happens to a correction that asks for more than
full lock.

## 8. Delay: dead time and lag

**What:** dead time = nothing happens at first. Lag = it then catches up along a curve.
**Why:** delay is the usual reason a stable design sways on a real machine.
**On our car:** the steering's dead time and its motor's lag; both still to measure.
**Watch:** "time delay control systems", "dead time first order plus dead time".
**You have it when** you can explain why steering a boat is harder than steering a bike.

---

# Part C. Stable

## 9. Stability

**What:** after a push, the error comes back to zero and stays.
**Why:** the first promise.
**Picture:** a marble in a bowl (stable), on a table (neither), on a ball (unstable).
**Watch:** "stability of dynamical systems intuition".

## 10. Linear systems and poles

**What:** for simple (linear) models, a few numbers called poles tell you everything:
does the error die, ring, or grow.
**Why:** the quickest stability check there is, and how our trackers were tuned.
**Watch:** "poles and stability", "eigenvalues stability" (Steve Brunton).
**You have it when** you know which side of zero a pole must be on.

## 11. Lyapunov functions

**What:** one number V that is zero on the path, bigger the worse the error, and
**goes down every step**. If you can find one, the error must shrink.
**Why:** it proves stability without solving the equations, and it works for
non-simple models. Barrier functions (card 17) copy this idea.
**The line:** `V(next) < V(now)` whenever the error isn't zero.
**Watch:** "Lyapunov stability intuition" (MIT "Underactuated Robotics", Russ Tedrake).
**You have it when** you can explain why "energy always falling" means "it stops".

## 12. Why delay breaks it

**What:** a correction that arrives late pushes the wrong way.
**Why:** this is the gap between the papers and our car.
**Watch:** "phase margin time delay", "Smith predictor".
**You have it when** you can explain why turning the shower tap too eagerly burns you.

---

# Part D. MPC

## 13. Optimisation

**What:** choose the numbers that make a score as small as possible, while obeying
rules.
**Why:** MPC is an optimisation solved 20 times a second.
**Words:** *cost* = the score. *Constraint* = a rule. *Feasible* = obeys every rule.
**Watch:** "convex optimization intuition" (Stephen Boyd's lectures for depth).

## 14. Quadratic programs (QP)

**What:** the easy kind: a bowl-shaped score, straight-line rules. One best answer,
found fast.
**Why:** it is what our solver solves. A rule only helps us if it can be written as a
straight line in the plan.
**Watch:** "quadratic programming explained".
**You have it when** you know why a bowl has one lowest point and a crumpled sheet
doesn't.

## 15. MPC

**What:** each tick: imagine the next 1.5 s, find the best plan that obeys the rules,
do the first move, throw the rest away, repeat.
**Why:** it is the one controller that handles limits and looks ahead. Every promise
in this survey becomes a rule inside it.
**Watch:** "model predictive control" (MATLAB Tech Talks' "Understanding MPC").
**You have it when** you can say why it throws away 29 of its 30 planned moves.

## 16. Horizon and the end of the plan

**What:** the plan only sees 1.5 s. What happens after is invisible to it.
**Why:** a plan can obey every rule for 1.5 s and end in a hopeless spot. This is
where "never stuck" is won or lost (card 23).
**Watch:** "MPC terminal cost terminal constraint".
**You have it when** you can describe a plan that is fine for 1.5 s and doomed at 1.6.

---

# Part E. Safe

## 17. Safe sets and invariance

**What:** the safe set is everywhere the car is allowed to be. **Invariant** means:
once inside, always inside.
**Why:** "safe" in maths is exactly "the safe set is invariant".
**Watch:** "invariant sets control", "forward invariance".

## 18. Barrier functions

**What:** a number H: positive = safe, zero = the edge, negative = inside.
**Why:** it turns "stay out of that region" into arithmetic.
**On our car:** `H = (distance to the object's centre)² − radius²`.
**Watch:** "control barrier functions" (talks by Aaron Ames).

## 19. The barrier rule (CBF)

**What:** `H(next) ≥ (1 − γ) × H(now)`: each tick may use only the share γ of the room
left.
**Why:** this is the idea at the centre. Guide steps 3 to 6.
**The proof:** six lines of induction. Guide step 5.
**You have it when** you can do guide step 5 without looking.

## 20. Two ways to use it

| | A filter | Inside the MPC |
|---|---|---|
| How | check each command, change it as little as possible | add the rule to the plan |
| Sees ahead? | no | yes |
| Can steer round? | barely: it mostly slows | yes |
| Size | tiny | bigger |

**Watch:** "CBF QP safety filter", "MPC with control barrier functions".
**You have it when** you can say why the filter tends to get stuck and the MPC less so.

## 21. How many steps until the command matters

**What:** steering changes heading, heading changes position. The command reaches the
distance only after two steps. This is called **relative degree**.
**Why:** a barrier on position can't be held by this tick's steering alone. Either
plan ahead (MPC), or build the delay into the barrier.
**Watch:** "relative degree nonlinear control", "high order control barrier functions".

---

# Part F. Never stuck, and the mess

## 22. Feasibility

**What:** is there any plan that obeys all the rules?
**Why:** if not, the solver has no answer. Add limits (card 7) and a barrier (card
19), and they can ask opposite things.
**On our car:** "don't get closer" and "can't brake that hard" collided (guide step 7).

## 23. Recursive feasibility and the trusted fallback

**What:** if there was an answer this tick, there is one next tick.
**How it is proved:** make every plan end somewhere a simple, trusted controller can
take over and stay safe for ever. Then last tick's plan, shifted by one, plus the
trusted move, is an answer for this tick.
**On our car:** the trusted controller is "stop". Guide step 8.
**Watch:** "recursive feasibility MPC", "terminal invariant set".
**You have it when** you can explain why "can always stop in time" is a safe set.

## 24. Stuck in the other sense

**What:** the solver answers, the car is safe, and it sits there for ever.
**Why:** a barrier only pushes away. It can't choose left or right, or back out.
**On our car:** at 1 m/s, head-on, it stops short and stays.
**Watch:** "deadlock control barrier functions", "liveness".

## 25. When the assumptions are wrong

| Assumption | Wrong how | Field of study |
|---|---|---|
| the model is right | tyres, slopes, load | robust MPC, tube MPC |
| disturbances are small | wind, bumps | input-to-state stability (ISS) |
| the car knows where it is | error, jumps | estimation; safe control under uncertainty |
| we know where things are | missed or false detections | perception uncertainty |
| the command acts at once | dead time | predictor methods, delay-aware CBFs |

**Why:** each row turns "proved" into "proved, if". The research is in shrinking the ifs.
**Watch:** "robust MPC tube", "input to state stability intuition".

---

# Part G. Learning, and what is open

## 26. Where machine learning fits

| ML supplies | If it is wrong | Does the promise survive? |
|---|---|---|
| the goal or the path | the car safely follows a poor path | yes |
| where the objects are | the car avoids the wrong places | no |
| part of the model | predictions are off | only with a bound on the error |

The pattern: **the learned part proposes, the proved part guarantees.**
**Watch:** "safe learning in robotics", "safe reinforcement learning shielding".

## 27. What is still open

| Open question | Cards |
|---|---|
| The barrier rule with a real dead time | 8, 12, 19 |
| Choosing γ for a machine with limits | 7, 19, 22 |
| Stable **and** safe together, with delay | 11, 15, 19 |
| A fallback that works while turning, not just straight | 23 |
| Keeping the promise when the position is uncertain | 25 |
| Getting unstuck without breaking the promise | 24 |

---

# The whole thing in one picture

```text
   bad things (1)
        │
   three promises (2) ─────────────────────────────────────────┐
        │                                                       │
   describe the car: state, model, limits, delay (4-8)          │
        │                                                       │
   ┌────┴───────────┬───────────────────┐                       │
 stable (9-12)   plan ahead: MPC (13-16)   safe (17-21)         │
 Lyapunov: V↓    cost + rules, repeat      barrier: H ≥ 0       │
   └────────────────┴───────┬───────────┘                       │
                            │                                   │
            rules inside the MPC + a trusted fallback (22-23) ◄─┘
                            │
              the mess: delay, error, limits (24-25)
                            │
              learning on top (26)   ·   what's open (27)
```

# Three statements, in plain words

| # | Statement | Needs | Shown in |
|---|---|---|---|
| 1 | If the room is never negative at the start, and each tick uses at most a share of it, it is never negative | γ between 0 and 1; the rule really holds | guide step 5 |
| 2 | If the car can always stop in the room it has, and asks to stop the moment that is about to become false, it stops in time | a known worst-case braking and delay | guide step 8 |
| 3 | If every plan ends where the fallback can take over, there is always a next plan | the fallback keeps the car in the safe set | card 23 |

A first watching order, if you want one: cards 4, 6, 9, 11, 15, 18, 19, 23. That is
the spine; the rest hangs off it.
