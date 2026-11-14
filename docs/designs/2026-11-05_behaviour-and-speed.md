# The behaviour states and a speed cap on Nav2

* **Author:** Ben (the pick), written up 5 Nov 2026
* **Date:** 2026-11-05
* **Status:** Built as a trial (0.3, 7 Nov; checked 13 Nov): `ros/drivers/aav_behaviour`, `behaviour:=true`, sim only (TODO 102, 137). Not built: the `patient` progress checker, ASK's reply, the trackers reading the cap, people, the car. Its place in the whole: [one shape](2026-11-13_one-shape.md), level 5. Numbers from Nav2 come from reading its source.

Ben, 4 Nov: "we also considered state machines for speeding up and slowing down and
steering to make the car more spesent". His pick that day: "The behaviour state manche +
speed profil eon a the path. Yes".

## The idea in one line

A small state machine knows why the car should slow, and sends Nav2 a **speed cap**. The
cap slows the car. **It can't stop it**: a lock, or Nav2's goal check, makes the final stop.

## Decided by Ben, 5 Nov

| Decision | What it means here |
|---|---|
| "we want to keep all the algo but we want to create a new best hybrid for each" | every stop rule stays; this page adds beside them |
| Stock stays the default; ours challenges | `behaviour:=false` by default, until it wins |
| First algorithm: how the car stops | **a separate design, written after this one, owns the stopping law.** This page gives its requirements |
| Radar tracking paused | nothing here reads radar targets |
| `pi` mode comes first on the car | a stage gate, below |

Everything else on this page is **proposed; awaiting Ben**.

## Problem

| Today | Result |
|---|---|
| The stop sign's lock fires at cruise speed, with no slowing first | an abrupt stop |
| Nothing slows for a bend or a known stop under MPPI | the speed comes from MPPI alone |
| A long hold looks "stuck" to Nav2 after 10 s | recovery runs, and the goal can abort |
| Nothing says why the car is waiting | a person has to guess |

## Goals

- Reach the sign's lock at a crawl, not at cruise.
- Hold through long waits without Nav2 aborting.
- Say the state and the reason on one topic.
- Fail back to today's stack, or slower, if the node dies.

## Non-goals

- **The stopping law.** It moves to "how the car stops".
- **Steering.** Ben's message names it. Nothing here changes steering; a later design can.
- **Replacing a lock.** All of them stay.
- **Radar targets, or naming what an obstacle is.** See "People: later".
- **Driving the car in `open` mode.**

## The states

```text
 DRIVE ─► APPROACH ─► STOPPED ─► WAIT ─► GO ─► DRIVE
  (no cap)  (cap falls)  (lock on,  (lock on)  (cap rises)
                         still)        │
                                       └─► ASK (a person is told)
```

| State | The cap sent | Progress checker |
|---|---|---|
| DRIVE | the bend and goal terms; `0.0` (no limit) if they don't bind | `progress_checker` |
| APPROACH | falling, never under the floor | `progress_checker` |
| STOPPED | the floor | `progress_checker` |
| WAIT | the floor | `patient` if the hold isn't the sign's |
| GO | rising at `accel` from the floor | back to `progress_checker` |
| ASK | the floor | `patient` |

### Every event

| In state | Event | Goes to |
|---|---|---|
| DRIVE | a fresh forward sign point, with `left` under the start distance | APPROACH |
| DRIVE | the path comes within the fence's start distance | APPROACH |
| DRIVE | any lock on, car still | WAIT |
| APPROACH | the lock named by the reason goes on, speed under 0.05 m/s | STOPPED |
| APPROACH | another lock goes on, car still | WAIT |
| APPROACH | `left` is 1 m past 0 and no lock came | DRIVE, and a warning: "sign passed, no lock" |
| APPROACH | no sign point for 5 s | DRIVE, and a warning: "sign lost" |
| APPROACH | goal ended or cancelled | DRIVE |
| STOPPED | always, next tick | WAIT |
| WAIT | every lock off | GO |
| WAIT | a non-sign lock held over `ask_after` (30 s) | ASK |
| WAIT | geofence NEAR at a standstill | ASK at once: only a person can clear it |
| GO | cap reaches the base speed | DRIVE |
| GO | a lock goes on | WAIT |
| GO | a new sign point | APPROACH |
| ASK | every lock off | GO |
| ASK | the person answers | see "ASK's contract" |
| any | goal aborted | ASK |
| any | odometry stale over 0.5 s | stays; the cap goes to the floor; a warning |
| any | e-stop on | WAIT; the node never clears it |
| any | a person drives (`/control/cmd_vel` isn't Nav2's) | no change: people aren't capped |
| start | the node starts or restarts | DRIVE |

The sign's wait stays the watcher's: `hold_s` is 4 s, about 3.5 s of it standing still from
the floor. This node has no 3 s timer of its own.

## The speed cap, and a limit of 0

Stock input: controller_server's `speed_limit_topic` parameter, `nav2_msgs/SpeedLimit`.

| Rule | Why (source) |
|---|---|
| `0.0` means **no limit**, never stop | `filter_values.hpp:59`, `SpeedLimit.msg:5`, MPPI `optimizer.cpp:491`, RPP `:498` |
| So a cap is `0.0` (no limit) or at least the **floor** | at X the law gives 0, which would lift the limit |
| Clamp every cap to the base speed | MPPI's ratio is limit ÷ base, so a higher limit raises `vx_max` and `wz` (`optimizer.cpp:506-510`) |
| Absolute, in m/s | `percentage: false` |
| Sent at 20 Hz | a restarted controller_server keeps no limit, and the topic isn't latched |

**What makes the final stop:**

| Stopping for | The cap does | The final stop |
|---|---|---|
| Stop sign | slows to the floor | the `stop_sign` lock (priority 30) |
| The fence | slows to the floor | the geofence's NEAR lock (51) |
| The goal | slows to the floor | Nav2's goal check, then its zero command (`controller_server.cpp:619`) |
| An obstacle | nothing | collision_monitor, then the `stop_zone` lock (21) |

### The floor: 0.3 m/s (proposed; awaiting Ben)

| Test | At 0.1 m/s | At 0.3 m/s |
|---|---|---|
| Time to move 0.5 m, Nav2's progress radius | 5 s | 1.7 s |
| That, plus the sign's 4 s hold (limit 10 s) | 9 s: too close | 5.7 s |
| Roll after the lock: 0.5 s × v + v²/(2 × 2) | 0.05 m | 0.17 m |
| Above the car's deadbands (0.03, 0.05 m/s) | yes | yes |
| Will the car really move? | unknown: `creep` is 0.1 for a deadband | unknown: test 4.4 |

The review's "say 0.1 m/s" fails the progress line, so this page raises it.

| Note on the floor | Detail |
|---|---|
| Car day's `max_speed:=0.3` | the same as the floor, so the cap does nothing then |
| `pi`'s smooth stop | starts under 0.3 m/s, so the stop fades: roll about 0.29 m (0.09 + 0.3/1.5) |
| RPP | obeys it: the final clamp is to the cap, after its 0.25 m/s minimum (`:474`, `:482`) |
| MPPI | untested this slow: 2.8 s ahead is only 0.84 m, and `vx_std` 0.4 is above the cap |

### MPPI's steering under a cap

`wz`'s limit shrinks by the same ratio as `vx_max` (`optimizer.cpp:506-510`). So at the cap's
own speed the tightest radius doesn't change: it is base speed ÷ `wz_max`.

| Case | Tightest radius at the cap's speed |
|---|---|
| Base 1 m/s | 1.67 m by `wz`; the 2.86 m turning limit binds first: no harm |
| `sim_speed:=3` | **5 m**, and the planner plans 3.6 m: MPPI must go below the cap to follow it |
| A cap dropping faster than MPPI's `ax_min` lets speed fall | `wz` is cut at once while `vx` lags: from 1 m/s with a cap of 0.3, up to 5.6 m, briefly. By reading only |

So the cap may only fall as fast as the car can slow. A bend test at `sim_speed:=3` is in
"Checks". RPP has no such scaling.

## What the cap misses

This is a cap on Nav2's followers. It is **not** one speed profile for the whole stack.

| Driver | Slot | Gets the cap? | How it could later |
|---|---|---|---|
| MPPI, RPP | 20 | **yes**, as a ceiling: they may go slower | stock |
| Trial trackers | (20) | no | `track_node` reads the topic, passes `cap` to `SpeedPolicy.step` |
| `mpc_full` | (20) | no: it plans its own speed | give it the same `cap` |
| Point driver | 25 | no | `Approach.step` already takes a `limit` |
| AI slot | 10 | no; the locks hold it | when it exists |
| People, tests, scripts, buttons | 45-100 | no, on purpose | |

**A trap for the trackers:** `SpeedPolicy.step` treats a `cap` of 0 as stop, and Nav2 treats
0 as no limit. `track_node` must turn `0.0` into `None`.

## The stop sign: two choices (proposed; awaiting Ben)

Today's watcher can't be told the car already stopped: after CLEAR, a close sign always goes
to STOPPING (`stop_sign.py:86-91`). So stopping short and going on trips the lock anyway,
and `sim_check.py:736` **requires** the lock to fire.

| | A: slow down, then the lock | B: stop short, with a handshake |
|---|---|---|
| The final stop | the lock, on every run | a new lock from this node |
| Stops | once | once, if the handshake works |
| Watcher | one new topic | a new input: "this one is served", then PASSING |
| twist_mux | no change | a new lock, with its own silence rule |
| `sim_check.py:736` | stays, plus a speed check | must change |
| Can the stop be anywhere? | no: at the lock's range | yes |
| A lost handshake | can't happen | stops twice |
| Is the lock still a separate guard? | no: it is the stop | yes |

**Recommendation: A, proposed; awaiting Ben.** It is the least new code, and it needs no new
lock or handshake. "How the car stops" can move to B once its law is measured.

The rest of this page assumes A. So "the lock never fires" is gone: it fires every time, at
the floor.

## Where X is

**One reference point:** `base_link`. `left` means the metres the car has to go before the
final stop.

| For | `left` | Frame | Input |
|---|---|---|---|
| Sign | camera range to the sign − `stop_distance_m` | `odom` | the sign point, `/tf` |
| Goal | `Path.left(s)` on the newest plan | `odom` | `/plan`, moved with `path.moved` |
| Fence | path distance to where the car would trip NEAR | `map` | `/plan`, `/safety/geofence/fence` |

### The sign is a range, not a place on the path

The lock fires on camera range, so `left` is measured the same way. A straight line is never
longer than the path, so on a bend the car slows a little early, the safe way.

| Step | What happens |
|---|---|
| 1. Watcher | `sign_distance_m` on the biggest forward box, plus the bearing from the box's centre |
| 2. Publish | a `PointStamped`: the **picture's stamp**, the camera's `frame_id` |
| 3. Behaviour | looks up `odom ← camera` at that stamp in `/tf`, and keeps the sign in `odom` |
| 4. Each tick | range = from the camera's pose now to the kept point |

| Rule | Why |
|---|---|
| `odom`, not `map` | a GPS jump moves `map → odom`, and mustn't move the sign |
| The new type replaces `Float32` | `Float32` has no stamp or frame |
| Forward frames only (`forward_frames`) | the watcher reads the rear camera's boxes too |
| Published only while the watcher is CLEAR | PASSING needs no second state here |
| No TF at that stamp | drop the picture |
| Camera frames | both shapes `ground.py` already handles: `_link` and `_optical` |
| The cap never rises in APPROACH | a new picture can't make the car speed up |

### How good the range is

From `sign_distance_m`, Gazebo's lens (343 px): 1 px of box height changes the range by:

| Range | Box | 1 px is |
|---|---|---|
| 4.7 m | 55 px | 0.09 m |
| 10 m | 26 px | 0.39 m |
| 17.4 m | 15 px | 1.2 m |

Gazebo's first sighting (24 Oct) was about 17.4 m from the camera (x = 5.9 m, sign at 25 m,
camera 1.75 m ahead of `base_link`). A 950 px lens is about 3 times better.

### Where the car stops

| | Gazebo | Car (camera at `0.81`, a candidate) |
|---|---|---|
| Camera ahead of `base_link` | 1.75 m | 0.81 m |
| Bumper to sign as the lock fires | 4.65 m | 3.71 m |

`vehicle.yaml` still marks the camera's x as a TODO, so the line moves with it: the reason
for `/tf`. The roll sum gives 3.9 m from 1 m/s, and Gazebo measured 4.4 m on 24 Oct, so that
sum is a worst case. The first run sets the real band.

## The stopping law

`v_cap(d)` comes from [how the car stops](2026-11-05_how-the-car-stops.md), which holds its
numbers. This page adds the floor and the geometry of `d`.

## Long waits, and replans

### What Nav2 does on a hold

Nav2 doesn't know about locks, so it keeps "driving" a car that can't move.

| Fact (source) | Effect |
|---|---|
| 10 s under 0.5 m and 0.5 rad fails the check (`nav2.yaml:86-88`, `pose_progress_checker.cpp`) | FollowPath aborts `FAILED_TO_MAKE_PROGRESS` |
| A new path doesn't reset the checker (`controller_server.cpp:724-768`) | replans at 1 Hz don't help |
| The tree then retries, clears costmaps and waits 5 s, 6 times | by reading it, the goal aborts after about 155 s (7 × 20 s, 3 × 5 s) |

### The fix: a second stock checker (proposed; awaiting Ben)

| Part | What |
|---|---|
| `nav2.yaml` | add `patient`, `PoseProgressChecker`, `movement_time_allowance: 600.0` |
| All four trees | stock `ProgressCheckerSelector`, with `topic_name="/drivers/nav/progress_checker_selector"` |
| `FollowPath` | gets `progress_checker_id="{selected_progress_checker}"` |
| The behaviour node | selects `patient` for a non-sign hold; back to the default on leaving WAIT or ASK |
| Changing checker | resets it (`controller_server.cpp:751-758`), and FollowPath passes it on (`follow_path_action.cpp`) |
| QoS | the selector is transient local: send once, on each change |

| Other option | Why not |
|---|---|
| Raise the normal allowance | a truly stuck car then waits for minutes |
| Cancel the goal, resend later | a new sender of goals, and it can hit the goal-circling fault (TODO 17) |
| Let recovery run | it wipes costmaps during an ordinary hold |

The sign's own stop doesn't need this: 5.7 s is under 10 s. The trial trackers have their own
check (`track_node.py`) and ignore the selector, so they still abort after 10 s.

### Replans

| Plan changes while | What happens |
|---|---|
| APPROACH, for a sign | nothing: range doesn't use the path |
| DRIVE, for bends and the goal | use the newest plan, re-find the car on it |
| WAIT | ignored until GO |
| The plan goes round the sign | range still falls; the lock still fires, as today |
| No plan for 3 s | drop the bend and goal terms; keep the sign's |

## ASK's contract (proposed; awaiting Ben)

| Part | Rule |
|---|---|
| "Stuck" means | a non-sign lock over 30 s; or the goal aborted; or NEAR at a standstill |
| Hears the abort from | `/navigate_to_pose/_action/status` |
| Shows | the state topic's level and reason |
| New work | the HUD has no reply path today: it only sends the e-stop and a heartbeat |
| The reply | `/hud/ask/answer`, `std_msgs/String`: `GO` or `STOP` |
| `GO`, goal aborted | sends the plan's last pose to `/goal_pose` once |
| `GO`, goal alive | restarts the 30 s timer |
| `STOP` | cancels the goal |
| No reply | stays put. **Silence never makes the car move** |
| Leaves ASK | every lock off (to GO); or `GO` or `STOP` handled |
| Never | clears a lock, a latch or the e-stop; `GO` only asks Nav2 |

## The node and the package

### Subtree or node? A node (proposed; awaiting Ben)

Other pages say "subtree, not a new node". Reading Nav2's tree nodes, that can't do this job:

| Need | Stock tree? |
|---|---|
| Publish a speed limit | no such node |
| Read the sign's point, or a lock | no: conditions read goals, batteries, paths and time |
| A custom tree node | it is C++, and the repo has no C++ of our own |
| Pick a progress checker from a topic | **yes**: the one tree change |

So: a small node, plus one stock tree edit. If streets bring lane changes and right of way,
this is worth asking again.

### Where it lives

`aav_navigation` installs data only, and its README says "no node of our own".

| Part | Proposed |
|---|---|
| Package | new: `ros/drivers/aav_behaviour` (Python), with README and CLAUDE.md |
| Logic | `behaviour.py`, no ROS, with tests |
| Node | `behaviour_node.py` |
| Reuses | `Approach`, `SpeedPolicy`, `turn_speed_limit`, `Path`, `moved`, `sign_distance_m`; `geofence.braking`, `stopping_distance`, `depth` |
| The switch | `navigation.launch.py behaviour:=false` (default): the stock path, no topic wired |
| On | starts the node, sets `speed_limit_topic` |
| Restart | `respawn: true`, 1 s |

### Topics (contract first)

| Topic | Type | Dir |
|---|---|---|
| `/perception/stop_sign/distance` | `geometry_msgs/PointStamped` (new, from the watcher) | in |
| `/perception/stop_sign`, `/safety/stop_hold/lock`, `/safety/geofence/lock`, `/safety/sensor_watchdog/lock` | `std_msgs/Bool` | in |
| `/safety/geofence/state` | `std_msgs/String` | in: NEAR and why |
| `/safety/geofence/fence` | `geometry_msgs/PolygonStamped` | in |
| `/plan` | `nav_msgs/Path` | in |
| `/localization/odom/local` | `nav_msgs/Odometry` | in: `s` and speed |
| `/localization/odom/global` | `nav_msgs/Odometry` | in: the fence's pose and σ |
| `/tf`, `/tf_static` | | in |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | in |
| `/hud/ask/answer` | `std_msgs/String` | in |
| `/drivers/nav/speed_limit` | `nav2_msgs/SpeedLimit`, 20 Hz | out |
| `/drivers/nav/progress_checker_selector` | `std_msgs/String`, latched | out |
| `/drivers/nav/behaviour/state` | `diagnostic_msgs/DiagnosticStatus`, 5 Hz: state, `left`, cap, why | out |
| `/goal_pose` | `geometry_msgs/PoseStamped` | out: ASK's `GO` only |

It isn't a driver: no `cmd_vel`, no twist_mux slot.

KNOW's trust cap isn't an input yet: `/safety/model_health` doesn't exist. When it does, it
joins the `min()`.

### TODO 100's speed zones share the topic

Stock `SpeedFilter` publishes on its own `speed_limit_topic`, only when the value changes
(`speed_filter.cpp:243`). Two publishers on one topic means the last message wins.

| `behaviour:=` | The filter publishes to | controller_server reads |
|---|---|---|
| false | `/drivers/nav/speed_zone` | `/drivers/nav/speed_zone` |
| true | `/drivers/nav/speed_zone` | `/drivers/nav/speed_limit`: this node takes the lower |

The filter's 0 means no zone, and its message may be a percentage: this node handles both.

## When things fail

**Nothing clears a stale cap.** controller_server keeps the last one for ever.

| The node dies in | The car | Worse than today? |
|---|---|---|
| DRIVE (no limit) | today's stack | no |
| APPROACH | keeps the last cap, the lock still stops it | no: slower |
| WAIT, ASK | the floor once the lock lifts | no: slower; `patient` stays selected up to 600 s |
| GO | keeps a low cap | no: slower |

| Repair | How |
|---|---|
| Restart | launch's `respawn`: back in DRIVE, sending `0.0`, within about 2 s |
| Being seen | the state topic silent 1 s: the HUD's debug view and `pixi run check` say so |
| Clean exit | sends nothing: `0.0` mid-approach would jump to cruise |

| Input stale | After | Then |
|---|---|---|
| `/localization/odom/local` | 0.5 s | the floor; the watchdog's lock also holds |
| Sign point | 1 s | keep the point in `odom` |
| Sign point | 5 s | DRIVE, and a warning |
| `/plan` | 3 s | no bend or goal terms |
| `/safety/geofence/fence` | latched; none yet | no fence term; the lock still works |
| `/perception/stop_sign` | 1 s | a warning, and the floor while in APPROACH |

## Jerk

| Fact | Source | So |
|---|---|---|
| Ego's `max_jerk` covers speeding up only: "Braking: never softened" | `ego.yaml:12-13` | it doesn't help here |
| Stock velocity smoother | no jerk term | it doesn't either |
| A plain √ law steps from 0 to `a` | `approach.py` | a lurch where slowing starts |
| The spine's ramp doesn't reach today's UART car | `legacy_protocol.py`: the ESP32 ramps throttle only | a spine limit can't be the fix |

**So the ease-in is in the cap itself**, upstream of every backend. The spine's own limit
becomes separate, later work.

| Target (proposed; awaiting Ben) | Number |
|---|---|
| Peak braking jerk in APPROACH, by `peak_jerk` (0.5 s smoothing) | at most 2.0 m/s³ |
| The final stop from 0.3 m/s, by the same rule, estimated | about 1.2 m/s³ |
| Today's lock from 1 m/s, by the same rule, estimated | about 4 m/s³ |

`measure atob` has this number (`jerk_peak`). **`measure controllers` doesn't**: add
`peak_jerk` to `drive_metrics.summary` and its columns.

## People: later

Tracking is paused, so this page has no person state. An obstacle stays what it is today:
collision_monitor stops, `stop_hold` holds, and this node shows WAIT, reason "stop zone".

What a person rule will need, none of it from radar targets:

| Need | Note |
|---|---|
| A stamped distance | `ground.box_points`, `ROAD_USERS`, `recent.Recent` |
| The class reaching the node | the car keeps only `stop sign` boxes unless `camera_obstacles` is on (`car.launch.py`), off until Q22 |
| A clear rule | including someone standing still |
| WAIT's range longer than any lock's | else the lock fires first and WAIT never slows (the review's finding 15) |
| No false stops for ever | ASK covers it |

## Alternatives

| Option | Good | Why not, for this |
|---|---|---|
| Costmap `SpeedFilter` | stock | a fixed mask: can't follow a sign just seen. **Keep it for zones** (TODO 100) |
| A goal at X | stock; Nav2 really stops | loops near goals (TODO 17: 10 of 12 runs before the fix); needs every goal's owner; loses the real goal |
| Slow down, then the lock | little new code | **chosen, as A** (proposed) |
| Stop short, with a handshake | a true guard | new lock, new handshake: **B**, maybe later |
| The limit in ego | reaches every driver | ego can't tell who drives, so it would trim people (the review's blocker 2) |
| Extending `aav_track` | the policy is there | a trial, off by default, refused with `reversing:=true`; MPPI wouldn't get it. **Reuse its code, not its node** |
| A tree subtree | fits Nav2 | see "Subtree or node?" |

## Risks

| Risk | Effect | Answer |
|---|---|---|
| The sign is never detected | no slowing, no stop | the same as today; not fixed here |
| MPPI stalls or wanders at 0.3 m/s | progress fails, or a poor line | a bend test on both controllers; raise the floor |
| The cap falls too fast for MPPI's `wz` | it cuts the bend | limit the fall; a test |
| A dead node's stale cap | a slow car, no alarm but ours | respawn; the state topic; a kill test |
| Gazebo's numbers fool us | `a` and `lead` wrong on the car | stage gates; test 4.3 |
| `patient` hides a real fault | up to 600 s until Nav2 reacts | ASK at 30 s; only while a lock holds |
| A false sign | a slow-down; then a lock, as today | no worse; the warning |
| The rear camera's boxes still trip the lock | a stop with no sign ahead | today's fault; note in "Other docs" |
| Time lost at each stop | about 1.4 s more at the floor, at 1 m/s | the lot-tour check |

## The stage gates, to the car

In `open` the car coasts: from 1 m/s it runs about 10 m on drag (`geofence.py:197-198`). A
cap can't slow it. So the node **refuses to start with `behaviour:=true` on the UART backend
in `open`**, and the geofence's COASTING hold stays.

| Stage | Where | Needs first | Passes when |
|---|---|---|---|
| 1 | Gazebo | nothing | the Gazebo rows in "Checks" |
| 2 | Bench | stage 1 | `bench-check --perception` |
| 3 | Car, watching: a person drives, the cap goes nowhere | stage 2; the cameras calibrated | the node's `left` matches a tape to 0.25 m |
| 4 | Car, ground, in `pi`, `max_speed:=0.3` | a driven-wheel speed (TODO 49b); `pi` proven (TODO 101); `pi_loop_decel` from test 4.3; the no-timeout lock lifted; the hardware lead's OK | the lock stops it, with a person at the joystick |
| 5 | Car, ground, 0.5 then 1.0 m/s | stage 4; `a` and `lead` from test 4.3 | "Checks" numbers, by tape |

Never on stands: there `pi` is refused today (`safety.py`). Stage 3 changes nothing in how the
car drives. It wins the default only with numbers beating stock's, Ben's call.

## Checks (none run yet)

| Test | Pass |
|---|---|
| `behaviour.py` unit tests | never a cap in (0, floor); clamped to base; every row of the event table; `left` from stamped points; a person appearing changes nothing |
| `SpeedPolicy` with `0.0` from Nav2 | treated as no limit, in `track_node` when it gets the cap |
| Gazebo sign, `behaviour:=true`, 1 m/s | slowing starts 1.88 m ± 0.34 before the line; at most 0.35 m/s as the lock fires; **the lock fires**; `sim_check.py:736`'s rule passes; stops once; `peak_jerk` under 2.0 |
| The stop's position | `base_link` within 0.25 m of the first run's (0.17 roll + 0.09 for 1 px): a starting band |
| The same, 3 m/s | slowing starts 9.13 m ± 1.5 before the line |
| The same, `behaviour:=false` | today's result: the fallback still works |
| Kill the node mid-approach (by its process group) | the lock still stops it, before the sign; `check` reports it; respawned, the goal completes |
| A 30 s hold: `pixi run ghost ahead 10`, `plan_around:=false` | no recovery, no abort; ghost off, GO, goal reached |
| The same, `behaviour:=false` | recovery logged: shows what the trial changes |
| A hold of over 30 s | ASK, with the reason |
| Stale odometry (`sim-fault`) | the floor; the watchdog's lock; no full speed |
| A bend at the floor, MPPI and RPP; and at `sim_speed:=3` | stays on the path, or the limit is found and written down |
| Lot tour, `measure controllers`, no signs | time within 10 % of stock; the new jerk column |

Each run on a shared machine uses its own `ROS_DOMAIN_ID`.

## Order

1. `docs/interfaces.yaml`, then the watcher's `/perception/stop_sign/distance`.
2. "How the car stops" (after this page). Then `aav_behaviour`, and `behaviour.py`.
3. The node, the launch switch, the cap.
4. The sign test in Gazebo: stage 1.
5. The `patient` checker, in the four trees, and the hold test.
6. The jerk column in `measure controllers`.
7. The fence's term; ASK, with the HUD's reply.
8. Stages 2-5, as their needs arrive.
9. Trackers and the point driver get the cap, if they earn it. People, when tracking returns.

## Questions for Ben (all proposed; awaiting Ben)

| # | Question | Proposed |
|---|---|---|
| B1 | The stop line: today's 4.7 m from the camera, or at the sign? | 4.7 m, until the camera is calibrated |
| B2 | The sign: A (slow down, then the lock) or B (stop short, with a handshake)? | A |
| B3 | A node, or a subtree? | a node, in a new package, plus one tree edit |
| B4 | A floor of 0.3 m/s? | yes, then tested on MPPI |
| B5 | The stopping numbers | in [how the car stops](2026-11-05_how-the-car-stops.md) |
| B6 | Absolute limits, or percentages? | absolute (m/s) |
| B7 | Trial trackers too? | later; the mechanism is above |
| B8 | A second, patient progress checker? | yes, 600 s |
| B9 | ASK may resend a goal and cancel one? | yes, on a person's word only |

## Other docs

| Doc | Needs |
|---|---|
| One car, the weak links | "subtree, not a new node"; "one speed profile for the whole stack"; "plans a stop instead of locking" |
| TODO 102 | "jerk-limited", `PERSON_AHEAD` |
| Knows itself, Part 4 | its cap is a later input; zero means no limit |
| Tracks and memory | WAIT no longer reads its targets |
| `stop_sign_watcher` | rear-camera boxes trip the lock: an existing fault, for its own TODO |

## Related

[One car](2026-11-04_one-car.md), [the weak links](2026-11-04_weak-links.md) (4.4, 4.5),
[the review](2026-11-05_egm-review.md), `docs/geofence.md`, `docs/perception.md` (stop signs),
`docs/controllers.md`, `ros/drivers/aav_track/aav_track/speed.py`,
`ros/drivers/aav_drive_to_point/aav_drive_to_point/approach.py`. TODO 102.
