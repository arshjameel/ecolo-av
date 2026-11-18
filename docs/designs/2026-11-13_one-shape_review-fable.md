# One shape: review

* **Reviewer:** Fable, 13 Nov 2026
* **Design:** [One shape](2026-11-13_one-shape.md)
* **How:** read the design, the code and Nav2 1.3.13's source (the version in `pixi.lock`). Nothing was launched: every number below is worked out from the source and `nav2.yaml`, not measured.

## Verdict: go, with changes

The shape is right: levels, one boss, fewer switches. Three parts need fixing before anything is built:

- the lane layer, as written, will not keep the car in the right lane;
- the lane layer is unlikely to cure the wiggle;
- D5 works against step 1.

## Changes, ranked

| # | Change | Why |
|---|---|---|
| 1 | Make the lane mask **from the route**, not once from `lanes.yaml` | "right side" depends on which way the car is going. One fixed mask can't hold both directions of a two-way road |
| 2 | Make the lane a **valley** (a ramp), on the **global** costmap | flat bands give nothing to centre on, and inflation hides the 40 |
| 3 | Take the wiggle out of "Done means" until three tests are run | the evidence points at the pose and the 1 Hz replan, not at lanes |
| 4 | Hold D5's costmap half until our costmap has the lane layer | our costmap has no filters, so the new default would have no lanes |
| 5 | Write the boss's contract before D1 | today it only caps speed, and only Nav2's controller listens |
| 6 | Keep `navigator:=` and `behaviour:=` apart; don't merge them into `boss:=` | they are two levels, not three options of one |
| 7 | Try a lane-centre planner (`path:=lane`) and Nav2's route server as challengers | simpler and stock; see the last section |

## 1. The nine levels

Good split. Ben's own five levels map onto it cleanly. Four fixes:

| Issue | Fix |
|---|---|
| Two numberings: 6 slots (the picture, "The slots today", TODO 147) and 9 levels | keep the 9; renumber the slot table |
| "Boss" mixes two jobs: the rules (stop, pass) and running one goal (plan, follow, recover) | level 5 = the rules. The navigator (Nav2's trees or ours) belongs with levels 6-7 |
| Routing (1) feeds only the boss in the picture | it must also feed the world model (change 1) |
| A costmap can't hold moving things, signs or stop lines | see section 2 |

## 2. One world model, one boss

**World model.** A grid has no speed and no meaning. The boss needs "a car 20 m ahead, in my lane, moving", "a stop line in 12 m". Say it as: one world model, **two views**: the grid (for path and steer) and a short list (lanes, tracked things, signs) for the boss. Both from the same sources. [Tracks and memory](2026-11-05_tracks-and-memory.md) is the list.

**Boss (D1).** Sound as a direction. Checked, what exists today:

| Fact | Where |
|---|---|
| The states are DRIVE, APPROACH, STOPPED, WAIT, GO, ASK. No PASS | `aav_behaviour/behaviour.py:38` |
| Its only output is a speed cap; "it isn't a driver" | `behaviour_node.py` docstring |
| Only Nav2's `controller_server` reads the cap | `navigation.launch.py:374` |
| No `speed_limit` reader in `aav_mppi_lag`, `aav_navigator`, `aav_smoother` | grep, no match |
| A cap of 0.0 means "no limit" | `behaviour_node.py` docstring |

What breaks:

- Under `stack:=ours` the boss is mute: nothing of ours listens to it.
- The trees still decide "what": back up, wait, clear the costmap. A back-up in the oncoming lane is a rule question. The boss must see the tree's state and be able to say no.
- If the boss dies, the cap goes away (fail-open). Fine for a cap that only slows. Not fine once it grants "may cross". Default to "no crossing" on silence.
- "Sets the goal": today the router and the goal senders do. Decide: the boss passes the goal on, or doesn't touch it.

## 3. Lanes as soft costs

**The filter does take in-between costs.** Checked:

| Behaviour | Source |
|---|---|
| Mask 0-100 becomes cost 0-254, linear. So 40 / 60 / 120 are mask values 16 / 24 / 47 | `costmap_filter.cpp:227-229` |
| The filter runs **after** every layer, inflation too | `layered_costmap.cpp:249-254` |
| It writes only where the mask is **higher** than what's there: a max, not a sum | `keepout_filter.cpp:365-369` |
| A new mask replaces the whole one, with a warning each time | `keepout_filter.cpp:146-151` |

**Problem A: which side is right?** On a two-way road the right lane going north is the left lane going south. A 2D cost can't depend on heading. So the mask must be painted for the route in hand. (86 of 129 roads in `lanes.yaml` are one-way: there the whole width is "right".)

**Problem B: inflation hides the bands (global costmap).** Global inflation is 3 m, factor 1.0 (`nav2.yaml:395-396`). On a 7 m road (the commonest width), if the kerbs are walls (`campus.md:49`: "only roads and lots are free"):

| Where | Inflation | Mask | Result (max) |
|---|---|---|---|
| Right lane centre (1.75 m from the kerb) | 56 | 0 | 56 |
| Just right of the centre line | 0-16 | 0 | 0-16, the cheapest place |
| Centre line | 0 | 60 | 60 |
| Left lane centre | 56 | 40 | 56, the same as the right lane |

So the planner would hug the centre line, left wheels on it. The left lane's 40 only shows in a 1.4 m strip. Fix: a ramp that is lowest at the right lane's centre and above inflation everywhere else, or less global inflation on roads. Test both.

**Problem C: MPPI won't hold the lane by cost.** Checked in the source:

| Critic | Size of its vote |
|---|---|
| CostCritic: 3.81 / 254 x mean cell cost (`cost_critic.cpp:28,37,213-215`) | whole future in the left lane at 40: about **0.6** |
| PathAlign: 10 x mean metres off the path (`path_align_critic.cpp:27`) | one lane (1.75 m) off: roughly **17** |

MPPI goes where the path goes. RPP and our trackers read no costmap at all. So the lane must act through the **planner**. The lattice does read it: a metre costs about (1 + 2.0 x cost / 252) (`node_lattice.cpp:301,323-324`, `nav2.yaml:455`): 40 is +32 %, 60 is +48 %, 120 is +95 %.

**Are 0 / 40 / 60 / 120 sensible?** As sizes, yes: crossing a line costs under a metre of extra path, so it will cross when blocked. As flat bands, no (B). The parked-car case should work, by inference: with `plan_around` the planner sees the lidar (`costmaps.md:24`).

**Smaller risks.** Any cost above about 2 makes MPPI and the lattice run the full outline check (`cost_critic.cpp:145-148`): more CPU, and campus plans already take 138-195 ms. "The boss raises the left lane's cost" means re-sending a 6400 x 6400 mask. Use two filters and the stock `toggle_filter` service (`costmap_filter.cpp:86-87`) instead.

## 4. 36 switches to 8

Good, and `stack:=` is the best part. Pitfalls:

| Pitfall | Suggestion |
|---|---|
| `boss:=nav2 / ours / behaviour` | two switches: `navigator:=`, and behaviour on in every stack |
| `pose:=` loses "scored beside" against "used" (`ndt` against `ndt_ekf`), and two at once | `pose:=ekf+ndt`; a separate `score:=` list |
| `steer:=` hides that our trackers and `mppi_lag` can't reverse | the launch refuses a bad mix, in plain words |
| A profile plus an override: who wins? | profile first, named switches after; print the final set |
| Nobody knows what ran | write the final set into each recording, so the scorecard can read it |
| `stack:=ours` rots | `sim-check` each stack in CI |

## 5. The wiggle (TODO 124)

Probably a separate problem. What points away from lanes:

- It happens on RPP too, which reads no costs (TODO 124).
- On 11 Oct it followed the GPS fixes: the map-to-odom heading stepped, and the path swung (`costmaps.md:244-262`). NDT gave "no clear change" since, so this is not settled.
- The tree replans once a second (`navigate_to_pose_car.xml:24`), and the flips are "about once a second". Each plan starts from a 0.25 m cell and one of 16 headings. Not tested, as far as I could find.
- "Sign flips a second" counts tiny changes round zero on a straight. Score the size: side-to-side RMS, as `bench-check --weave` does.

Test first, in this order, one change each:

| # | Test | If the wiggle goes |
|---|---|---|
| 1 | Pose from the simulator's truth | it's localization |
| 2 | One plan, no replanning | it's the replan |
| 3 | The command before and after the smoother and ego | it's downstream |

## 6. Replacing Gazebo and Nav2 today

No, and the project's own rule says not yet. What is left:

| | Genuinely left |
|---|---|
| Gazebo | computers with no GPU: CI and oirp-spss need it for the cameras (`own-simulator.md:15-16`); `lag_scale` is Gazebo-only (`switches.md`) |
| Nav2 | `map_server`, `collision_monitor`, the back-up recovery still run (`own-navigator.md:66,157`); ours can't reverse; all ours scores 7 of 12 against 8 (TODO 145); costmap filters not ported (`own-costmap.md:59`); the car has run none of it |

## 7. Open source worth a look

| Option | Take |
|---|---|
| Nav2's route server (`nav2_route`) | already installed (`pixi.lock:1934`); not run by me. A stock level 1, to score beside the campus router |
| A lane-centre planner (`path:=lane`) | the route's centre line, moved half a lane right, served as the path. This is Autoware's idea (follow the lane's centre, shift only to avoid). No cost tuning, direction is free. The lattice stays for lots and for going round |
| Lanelet2, as the map format | lanes, sides, stop lines in one file, with routing. Better than growing `lanes.yaml`. Licence and RoboStack build not checked |
| Autoware as a whole | no: too big to carry; take its ideas (already Ben's plan, 28 Oct) |
