# Keeping a car off things: a walk-through

**Outside the capstone's scope.** Ben Woodhead's own research, toward his MASc thesis. Not a team deliverable.

A study guide to [the barriers in the MPC](barriers.md): the idea worked through by hand.
The map of the whole area is [the survey](barriers-survey.md).

**How to use it.** Twelve steps. Each has three parts:

- **Read**: one idea, a few lines.
- **Try it**: something to do with a pencil. Do it before you look.
- **Check**: the answer.

You need multiplication, and nothing else, until step 9. Go slowly. One or two steps a
sitting is plenty.

| Steps | What you get |
|---|---|
| 1 to 3 | the problem, and why the obvious fix isn't enough |
| 4 to 6 | the barrier idea, and the proof |
| 7 to 8 | where it breaks on a real machine, and the backstop |
| 9 to 10 | from a line to a real car, and into the MPC |
| 11 to 12 | what the test runs showed, and what is still open |

---

## Step 1. The simplest version of the problem

**Read.** Forget steering. A car drives straight at a wall.

```text
   car ──►                          ┃ wall
        ◄────────── h ────────────►
```

- `h` is the room left, in metres.
- A **tick** is one decision. Here a tick is 1 second, to keep the numbers easy.
  (On our car it is 0.05 s.)
- Each tick the car moves `speed × 1 second`.

**Try it.** The car starts with h = 8 m and holds 2 m/s. Fill in h after each tick.

| Tick | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| h (m) | 8 | ? | ? | ? | ? | ? |

**Check.** 8, 6, 4, 2, 0, −2. At tick 4 it touches. At tick 5 it is 2 m inside the wall.

**The point.** A negative h means "inside". Our whole job is: **h never goes below 0.**

---

## Step 2. The obvious fix, and its flaw

**Read.** The obvious rule: "if h is under 2 m, brake." But brakes are not instant.
Say the car can lose at most 1 m/s each tick.

**Try it.** Start h = 8, speed 2 m/s. The rule fires when h < 2. Work out each tick.
Remember: brake first lowers the speed, then the car moves at the new speed.

| Tick | h before | Rule fires? | Speed this tick | h after |
|---|---|---|---|---|
| 0 | 8 | no | 2 | 6 |
| 1 | 6 | ? | ? | ? |
| 2 | ? | ? | ? | ? |
| 3 | ? | ? | ? | ? |
| 4 | ? | ? | ? | ? |

**Check.**

| Tick | h before | Rule fires? | Speed | h after |
|---|---|---|---|---|
| 1 | 6 | no | 2 | 4 |
| 2 | 4 | no | 2 | 2 |
| 3 | 2 | no (2 is not under 2) | 2 | 0 |
| 4 | 0 | yes | 1 | −1 |

It hit. The rule did nothing until the last moment, and then it was too late.

**The point.** A rule that only looks at **distance** reacts late. It ignores how fast
the room is being used up. Make the 2 m bigger and it works at 2 m/s, then fails at
4 m/s. You'd need a different number for every speed.

---

## Step 3. A better question

**Read.** Stop asking "how much room is left?" Ask:

> **How much of the room that is left am I allowed to use this tick?**

Pick a share. Call it **γ** (gamma). Say γ = ½.

**The rule:** each tick, use at most half of what is left.

**Try it.** Start h = 8. Use exactly half each tick. Fill in.

| Tick | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| h (m) | 8 | ? | ? | ? | ? | ? | ? |

**Check.** 8, 4, 2, 1, 0.5, 0.25, 0.125.

**The point.** It gets as close as you like. It never gets to zero. Half of something
positive is still positive.

**Try it again** with γ = ¼ (use at most a quarter). Start h = 8.

**Check.** 8, 6, 4.5, 3.375, 2.53... Slower approach. Same promise.

---

## Step 4. Write the rule down

**Read.** "Use at most the share γ" means "keep at least the share (1 − γ)".

```text
   h(next)  ≥  (1 − γ) × h(now)
```

That one line is the whole idea. The name for h is a **barrier function**. People
write it H. The name for the rule is a **control barrier function condition** (CBF).

| Word | Plain meaning |
|---|---|
| barrier function H | a number: positive = safe, zero = the edge, negative = inside |
| γ (gamma) | the share of the room one tick may use, between 0 and 1 |
| the safe set | all the places where H ≥ 0 |

**Try it.** H = 4 and γ = 0.05. What is the smallest H allowed next tick? And the tick
after?

**Check.** 0.95 × 4 = 3.8. Then 0.95 × 3.8 = 3.61.

**Try it.** What does the rule say when γ = 1?

**Check.** (1 − 1) × H = 0. So it says only "H(next) ≥ 0". That is the obvious fix from
step 2 again: allowed to use **all** the room in one tick. So γ = 1 is the late rule,
and small γ is the early, gentle one.

---

## Step 5. The proof

**Read.** We claim: *if H starts at 0 or more, and the rule holds every tick, H is
never negative.* The method is **induction**: show it is true at the start, then show
that "true now" forces "true next". Like dominoes.

**Try it.** Before reading on, try to say in your own words why it must be true. Use
the table from step 3.

**Check.** The proof, one line at a time.

1. **Start.** H(0) ≥ 0. We were given that.
2. **Suppose** H is ≥ 0 at some tick. Call it tick k.
3. γ is between 0 and 1. So (1 − γ) is between 0 and 1.
4. A number between 0 and 1, times a number ≥ 0, is ≥ 0. So (1 − γ) × H(k) ≥ 0.
5. The rule says H(k+1) is at least that. So H(k+1) ≥ 0.
6. So tick k being safe forces tick k+1 to be safe. Tick 0 is safe. So every tick is. ∎

**Try it.** Which line breaks if someone sets γ = 1.5?

**Check.** Line 3. (1 − 1.5) = −0.5. Then the rule would allow H(next) = −0.5 × H:
negative. Inside the wall, and the rule is happy.

**Try it.** Suppose the car is already inside: H = −2, γ = ½. What does the rule ask?

**Check.** H(next) ≥ ½ × (−2) = −1. So H must **rise** from −2 to at least −1. The same
rule that keeps you out also pushes you back out. Nobody had to add that.

---

## Step 6. What the rule asks of the car

**Read.** A rule about H is no use until we know what it means for speed.

In one tick the car uses up `speed × tick` of room. The rule allows `γ × H`. So:

```text
   speed × tick  ≤  γ × H        so        speed  ≤  γ × H / tick
```

**The allowed speed is proportional to the room left.** Far away: any speed. Close:
slow. At the edge: zero.

**Try it.** Our real numbers: tick = 0.05 s, γ = 0.05. What speed is allowed with
H = 3 m of room? With 1 m? With 0.2 m?

**Check.** γ / tick = 0.05 / 0.05 = 1. So speed ≤ H: 3 m/s, 1 m/s, 0.2 m/s.

**Try it.** The car cruises at 1 m/s. How far from the wall does the rule first start
to matter? Do it for γ = 0.05, then γ = 0.02.

**Check.** The rule bites when 1 = γ × H / 0.05.
γ = 0.05: H = 1 m. γ = 0.02: H = 2.5 m. **Smaller γ = it acts earlier.**

---

## Step 7. Where it breaks on a real machine

**Read.** The proof in step 5 has a hidden "if": *if the rule holds every tick.* The
rule tells the car to slow down. Can the car actually slow that fast?

Following the rule exactly, each tick the speed must drop by the share γ. So the
braking needed is:

```text
   braking needed  =  γ × speed / tick
```

**Try it.** tick = 0.05 s, speed = 1 m/s. What braking does γ = 0.05 need? γ = 0.02?
The car's gentle stop is about 0.8 m/s² (a guess, not measured).

**Check.** γ = 0.05: 0.05 × 1 / 0.05 = 1 m/s². More than 0.8: **the car can't keep the
rule.** γ = 0.02: 0.4 m/s². Fine.

**The point.** γ is not free. It has to fit what the brakes can do. Pick it too big
and the proof's "if" is false.

**What really happened.** In the first test run the car slowed nicely, then **crawled
into the object at about 0.15 m/s** and kept going. The reason was not in any paper:

| What the plan asked | What the car did |
|---|---|
| "slow down a little more each tick" | our speed controller only **coasts** when it is a little too fast |
| | it brakes properly only when it is a lot too fast, or told to stop |
| so the gentle request never became real braking | the car rolled on |

The maths was right. The machine broke the "if". Hold on to this one: it is the
difference between a proof on paper and a car.

---

## Step 8. The backstop

**Read.** When a clever rule can fail, put a simple one behind it. The simplest thing
a car can do is **stop**. So ask every tick:

> **If I asked for a full stop right now, would I still stop in time?**

The distance a stop takes has three parts:

```text
   stop distance  =  speed × wait   +   speed × (speed / ramp)   +   speed² / (2 × braking)
```

| Part | Why |
|---|---|
| speed × wait | nothing happens at first (dead time, and one tick) |
| speed × speed/ramp | the command itself is lowered gradually |
| speed² / (2 × braking) | the actual braking |

**Try it.** speed 1 m/s, wait 0.15 s, ramp 2 m/s², braking 0.8 m/s². Stop distance?

**Check.** 1 × 0.15 + 1 × 0.5 + 1 / 1.6 = 0.15 + 0.5 + 0.625 = **1.275 m**.

**Try it.** Now at 2 m/s.

**Check.** 2 × 0.15 + 2 × 1 + 4 / 1.6 = 0.3 + 2 + 2.5 = **4.8 m**. Twice the speed,
almost four times the distance. That is why speed matters so much.

**The rule.** If the object is nearer than the stop distance plus one tick of travel:
ask the stop now.

**Why it works** (try to say it first):

1. Last tick the rule did not fire. So the object was farther than "stop distance +
   one tick of travel".
2. One tick later the car has moved at most one tick of travel. So the object is
   still farther than the stop distance.
3. The rule fires now. The car needs at most the stop distance. It stops short. ∎

**Try it.** Which line is wrong if the car sped up during that tick?

**Check.** Line 2. A faster car has a longer stop distance, so "still farther than the
stop distance" may be false. The fix: use the fastest the car could be after one tick.

---

## Step 9. From a line to a real car

**Read.** A real car can steer. So the wall becomes a **keep-out circle**: the object,
plus the car's own size, plus a margin.

```text
              ╭───────╮
   car ──►   (  object )      d = distance from the car's middle to the centre
              ╰───────╯      R = the circle's radius
```

The barrier becomes:

```text
   H = d² − R²
```

| Where | H |
|---|---|
| outside | d > R, so H > 0 |
| on the edge | d = R, so H = 0 |
| inside | d < R, so H < 0 |

Same sign rule as h in step 1. The squares are there because they make the maths
smooth (no square roots). **The rule from step 4 does not change at all.**

**Try it.** R = 2 m. The car's middle is 5 m from the centre. H? Then 3 m. Then 2 m.

**Check.** 25 − 4 = 21. 9 − 4 = 5. 4 − 4 = 0.

**What is new:** there are now two ways to keep H up. **Slow down**, or **steer away**.
Steering away makes d grow. The rule does not say which. Something has to choose.

---

## Step 10. Putting it in the MPC

**Read.** MPC means "model predictive control". It is a loop:

| Each tick | |
|---|---|
| 1 | Look where the car is |
| 2 | Imagine the next 30 ticks (1.5 s), using a model of the car |
| 3 | Find the steering and speed for all 30 that are **best**, and that **obey the rules** |
| 4 | Do only the first one. Throw the rest away |
| 5 | Next tick, start again |

"Best" means close to the path and smooth. "The rules" were: steering within its
limits, speed within its limits.

**We add one more rule:** the step-4 barrier line, for each of the 30 ticks.

That answers step 9's question. The MPC chooses between braking and steering by
finding whichever keeps closest to the path while obeying the rule.

**One practical extra: the slack.** Sometimes no plan can obey every rule. A solver
with no answer is the worst case. So each barrier line gets a **slack**: it may be
broken, at an enormous cost. The solver always has an answer, and it tells us how much
slack it used. When slack is used, the step-8 backstop is what protects the car.

**Try it.** Why does adding the rule for 30 ticks ahead help, compared with step 2's
"brake under 2 m"?

**Check.** The plan sees the room shrinking 1.5 s ahead and starts to turn or slow
**now**, while small moves are enough.

---

## Step 11. What the test runs showed

**Read.** A pretend car (our model, not the real one). A straight 40 m path. A round
object at 15 m. "Gap" = metres between the object and the car's body.

| Object | Cruise | No barrier | With the barrier (γ 0.05) |
|---|---|---|---|
| on the path | 1 m/s | hit, 0.5 m inside | stopped short, gap 0.32 |
| on the path | 3 m/s | hit, 0.5 m inside | went round, gap 0.74 |
| 0.4 m to the left | 3 m/s | hit, 0.5 m inside | went round, gap 0.82 |
| 1.5 m beside | 1 m/s | passed at 0.50 | passed wider, 0.81 |

36 runs with the barrier in all. None touched. Smallest gap 0.20 m.

**Try it.** Before the table below: what should a smaller γ do? (Step 6.)

**Check.** Object 0.4 m left, 3 m/s:

| γ | Starts to turn at | Widest off the path | Gap |
|---|---|---|---|
| 0.02 | 6.2 m | 2.62 m | 1.76 m |
| 0.05 | 10.4 m | 1.45 m | 0.82 m |
| 1 | too late to steer: stops | 0.24 m | 0.37 m |

Earlier and wider, exactly as step 6 said.

**Try it.** Why did the slow car (1 m/s) stop, when the fast one went round?

**Check.** The plan looks 1.5 seconds ahead. At 1 m/s that is 1.5 m. The keep-out
circle is 3.6 m wide. There isn't room to steer round something you first notice 1.5 m
away. So it does the only safe thing left. It is **safe, and stuck**.

---

## Step 12. What is still open

**Read.** Three promises were on the table. Here is where each stands.

| Promise | Meaning | Where we are |
|---|---|---|
| **Safe** | never touches | steps 4 to 10. Shown on the pretend car. Proved for the model only |
| **Never stuck** | the solver always answers | the slack gives that. But the *car* can still sit stopped for ever |
| **Stable** | settles onto the path, doesn't sway | **not started** |

And every "if" we leaned on:

| The "if" | True on the real car? |
|---|---|
| the model matches the car | roughly: the lags are guesses until measured |
| the car brakes at 0.8 m/s² or more | not measured |
| the car stops in a straight line | no, it may be turning |
| the car knows where it is | only as well as localization does |
| the object is a circle that stands still | a first shape only |
| we know where the object is | someone has to detect it first |

Each row is a place where the paper proof and the real car can disagree. Step 7 was
one of them showing up.

### Words

| Word | Meaning |
|---|---|
| barrier function (H) | positive = safe, zero = edge, negative = inside |
| CBF | control barrier function: the rule H(next) ≥ (1 − γ) × H(now) |
| γ (gamma) | the share of the room one tick may use |
| induction | true at the start, and true-now forces true-next |
| MPC | plan a short way ahead, do the first move, repeat |
| horizon | how far ahead the plan looks (30 ticks, 1.5 s) |
| constraint | a rule the plan must obey |
| slack | permission to break a rule, at a high cost |
| feasible | at least one plan obeys every rule |
| recursive feasibility | feasible now means feasible next tick |
| dead time | the wait before anything moves at all |
| lag | the curve as it catches up |

### Where the idea comes from

A. M. Ali, C. Shen, H. A. Hashim, "A linear MPC with control barrier functions for
differential drive robots", IET Control Theory & Applications, 2024. Their robot has
two wheels and no delay; their proof is complete for it. Steps 7, 8 and 12 are about
what changes on a car.

### Open questions

1. In step 7, γ had to fit the brakes. How do you choose γ properly for a machine with
   limits?
2. The slack keeps the solver alive but weakens the promise. What do you use instead?
3. How does the proof change when the command takes 0.2 s to act?
4. What is the right way to add "stable" on top of "safe"?
5. Safe and stuck (step 11): whose job is it to get unstuck?
