# How the car stops

One stopping law for every planned stop, a behaviour planner that uses it to slow navigation,
and a check of where the car really stops. Ben, 7 Nov: "we can use that to say if we have
turned enough or slowing down enough". The designs:
[how the car stops](designs/2026-11-05_how-the-car-stops.md),
[behaviour and speed](designs/2026-11-05_behaviour-and-speed.md).

> **Every number is a GUESS** until car tests 4.1-4.5 ([actuator tests](actuator-tests.md)).
> The locks don't use any of this: they keep their own laws.

## 1. The stopping law

```text
   S(v)  =  v·τ  +  v²/(2a)           τ = ρ + a/(2j)
            └┬─┘    └──┬──┘
       delay + easing  braking

   the cap:     v_cap(d) = a·( √(τ² + 2d/a) − τ )     the speed that stops in d metres
```

`aav_description` `stop_law.py` (no ROS): `StopCurve(a, j, rho)` with `distance(v)`,
`cap(d)`, `demand(v)`, `h(room, v, margin)`.

### Its numbers come from the car model

| Symbol | Means | From | Today |
|---|---|---|---|
| `a` | planned braking | half the softer of ego's `max_deceleration` and `max_brake` | 1.0 m/s² |
| `j` | jerk | ego's `max_jerk` | 2.0 m/s³ |
| `ρ` | every delay | input age 0.15 s + brake dead time + `brake_lag` + **the speed loop's lag** | 1.22 s |
| `τ` | delay + easing | ρ + a/(2j) | 1.47 s |

**The speed loop's lag** (new, not in the design): the spine's default PI coasts until the
car is 0.2 m/s too fast, then brakes 0.5 per m/s past that. To brake at 1 m/s² it must be
0.87 m/s too fast: 0.87 s behind. Without it, the law overran the point by 0.3-0.4 m on
the pretend car. `loop=None` gives the design's ρ 0.35.

| Distance left | 0.5 m | 1 m | 2 m | 5 m | 10 m |
|---|---|---|---|---|---|
| `v_cap` | 0.31 m/s | 0.57 m/s | 1.01 m/s | 2.02 m/s | 3.24 m/s |

### Who uses it

| User | Before | Now | Check |
|---|---|---|---|
| Trial trackers (`aav_track` `speed.py`) | `√(2·1.5·d)` | **the law** | never a higher speed, at every distance (`test_speed.py`) |
| Point driver (`aav_drive_to_point`) | `√(2a(d − 0.8v))` | the same; **`stop_law:=hybrid`** picks the law (a trial) | stops at B on the pretend car (`test_drive.py`) |
| ego's start-up log | `0.4v + v²/2a` | both lines: the hardest stop, and the planned one | the plan is never shorter (`test_dynamics.py`) |
| Behaviour planner | none | the law | below |
| The geofence's lock | RSS | **unchanged** (Simplex) | its worst case is never shorter than where the car model stops (`test_geofence.py`). Above about 0.7 m/s the law plans **more** room than the lock (1.97 against 1.40 m at 1 m/s on the spine), so the car slows before the lock closes |
| The stop sign's lock | 4.7 m | **unchanged** | |

### Point driver, pretend car (ego and the sim's actuator model)

| From | Law | Stop error | Peak braking | Time |
|---|---|---|---|---|
| 1 m/s, 10 m | approach | +0.06 m | 0.77 m/s² | 15.4 s |
| 1 m/s, 10 m | hybrid | +0.09 m | 0.77 m/s² | 15.3 s |
| 3 m/s, 30 m | approach | +0.04 m | 1.32 m/s² | 19.7 s |
| 3 m/s, 30 m | hybrid | +0.09 m | **0.78 m/s²** | 19.9 s |

Gentler at speed, 5 cm further past B. Not yet run in the simulator.

## 2. The behaviour planner

`aav_behaviour_rs` (a trial, **off by default**): `pixi run sim world:=road behaviour:=true`.
It knows why the car should slow, and sends navigation a speed cap. **It can't stop the car**:
a lock or the navigator's goal check makes every final stop.

```text
 DRIVE ─► APPROACH ─► STOPPED ─► WAIT ─► GO ─► DRIVE
  (no cap)  (cap falls)  (lock on,  (lock on)  (cap rises)
                         still)        │
                                       └─► ASK (a person is told)
```

| Stop for | `left` | The final stop |
|---|---|---|
| A stop sign | camera range to the sign − 4.7 m | the `stop_sign` lock |
| An obstacle in the lane | lidar: nearest point ahead − bumper − stop zone | collision_monitor, then the `stop_zone` lock |
| A kerb on the road ahead (`kerbs:=true`, TODO 219) | along the road ahead to the kerb − bumper − stop zone (an `obstacle` stop) | the stop zone (its `steps` source), then the `stop_zone` lock |
| A moving thing (`things:=true`) | to the thing, in the lane now **or walking into it** before the car gets there (below) | the `stop_zone` lock |
| The fence | along the plan to where the geofence would say NEAR | the geofence's lock |
| The goal | along the plan to its end | the navigator's goal check |

### Someone walking into the lane (TODO 194)

With `things:=true`, a thing moving at 0.3 m/s or more counts when:

| It is | It counts if |
|---|---|
| in the lane now | always |
| beside the lane, walking towards it (0.2 m/s or more across) | at its speed it is in the lane before the car reaches it + 2 s, and within 6 s |

Before 15 Nov only "in the lane now" counted. A person crossing was seen about 1 m from the
bumper: too late to slow, and only the stop zone stopped the car (`behaviour.py`
`moving_ahead`, `Crossing`).

### Anything moving near: stop and wait (17 Nov)

Ben, 16 Nov: "if we see a car people or stuff moving around us we should likely just
stop."

```text
 a thing moving 0.3 m/s or more  ──┬── within 10 m of the car ─────────────┐
 (/world/things, aav_things_rs)       └── on the road ahead within 3 s ───────┤
                                                                           ▼
                        WAIT (moving): the road ahead ends at the car, ego holds it at 0
                                                                           │
                        gone, or standing still, for 1 s (release_hold) ───► GO
```

| Rule | Number | Parameter |
|---|---|---|
| it moves | 0.3 m/s or more over the ground | `moving_speed` |
| and keeps moving | 1.5 s, the run started on a thing seen (not coasting) and not growing (a building coming into view); once sure, it stays sure while it coasts in the blind zone | `moving_settle` |
| near | 10 m from the car (its centre, less half its narrow side) | `moving_radius` |
| on the road ahead | within half the lane + half its narrow side of `/world/road_ahead`'s line, checked every 0.25 s for 3 s at its speed | `moving_within` |
| go again | 1 s after the last one has gone or stood still | `release_hold` |
| off | `moving_hold:=false` (the node's parameter) | `moving_hold` |

- **Only a still thing is passed** (the planner's lane pass, `pass.rs`). Anything moving is
  waited for: an oncoming car, a person on the pavement, cross traffic.
- **How it stops the car:** a stop called `moving` at 0 m on the road ahead, so its end is
  the car and ego allows 0 m/s. The cap goes to the floor at once, as in any WAIT.
- **Never over the e-stop or ASK:** the e-stop's WAIT stays the e-stop's; ASK and a lock's
  WAIT don't go to GO while something moves near.
- It needs the things (`things:=true`). `stack:=ours` turns them on since 17 Nov, where
  nav and the lidar run (`stacks.yaml`'s behaviour level).
- **Why 1.5 s, and not growing:** the things' tracker gives still things a speed for a
  while (`MovingFor`):

  | Seen | Its speed | For | Without the rule |
  |---|---|---|---|
  | the parked box, as the car passes | 0.5-0.9 m/s | 0.1-0.6 s | 8 holds passing it |
  | new tracks, coasting off | 0.5-1.8 m/s | seen 3-7 times | (in those 8) |
  | buildings coming into view at 2 m/s, growing 1 → 9 m | 1-2 m/s | 2-3 s | not growing: never |
  | the lot's thin posts, a low patch of grass | 1.5-11 m/s | 1-1.4 s | 3 holds in 7 runs at 1 s, none at 1.5 s; the posts gone since the tracker's gates (17 Nov, TODO 222): 0 holds in 3 lot tours |
  | a person, a car really moving | 0.8-3 m/s | 9-42 s | held, every time |
- **Known limit:** a person walking beside the car the same way, within 10 m, holds it
  as long as they walk. That is the rule as asked.

`behaviour.py` `moving_near`, `Nearby`, `MovingFor`; Rust `paths::moving_near`,
`MovingFor`. `mppi_lag` waits too: while the road ahead says WAIT (moving) its progress
check starts again, so a long hold is not "no progress" (it failed the path after 10 s).

**In our simulator** (17 Nov, the road world, 2 runs each, `pixi run sim-check --movers
--moving --behaviour-things`):

| Case | Held | Clear of it ahead of the moving car | Then |
|---|---|---|---|
| an oncoming car, 3 m to the left, 2 m/s | 11.6 s | 2.4, 3.5 m | reached the goal 2/2 |
| a person on the roadside, 5 m right, walking towards the car | 20.3 s | 6.6, 6.8 m | goal 1/2 |
| cross traffic, 22 m ahead, 3 m/s | 2.0 s | 19 m | goal 1/2 |
| a person crossing ahead (`--moving`) | APPROACH, then 17.7 s | 1.9 m | goal 1/2 |
| the parked box (`--behaviour-things`) | never | passed 1.0 m clear | goal 1/1 |

Every miss was the computer, not the rule: two simulators at a load of 15-20, the
odometry or the costmap late ("no costmap yet", the sensor watchdog's lock), the goal
aborted. Each held every time.

### The cap

| Rule | Why |
|---|---|
| `v_cap(left)` of the lowest stop | the stopping law, above |
| 0.0, or between 0.3 m/s and the top speed | 0.0 reads as **no limit** |
| Falls no faster than 2 m/s² | MPPI cuts its turn rate at once, its speed later |
| The sign's cap never rises in APPROACH | a new picture can't speed the car up |
| Rises at 0.5 m/s² in GO | |
| Odometry stale: the floor | |

### Topics

| Topic | Type | |
|---|---|---|
| `/world/road_ahead` | `aav_interfaces/RoadAhead`, 10 Hz | the cap is in its limits (none above it; TODO 233, 17 Nov): ego holds every follower to them. Before, `/drivers/nav/speed_limit`, read only by Nav2's controller |
| `/drivers/nav/behaviour/state` | `diagnostic_msgs/DiagnosticStatus`, 5 Hz | for the screens: state, reason, cap, left |
| `/perception/stop_sign/distance` | `geometry_msgs/PointStamped` | new, from the watcher: where the sign is |

### In Gazebo (8 Nov, `pixi run sim-check --behaviour`)

| Stop | Result |
|---|---|
| Obstacle | passed 2 of 4 runs: APPROACH, cap down to 0.3 m/s, the stop zone fired at 0.50 m/s, the bumper 0.8 m from the box |
| Obstacle, the other 2 | the car went round the box (the lidar is on, so MPPI may steer round it): no stop to score |
| Goal | APPROACH from 2 m out, lowest cap 0.48-0.68 m/s; reached, 0.9 m from the goal |
| Stop sign (`-e ml ... --perception`) | the lock stopped the car 3.5 m before the sign, but the behaviour **never entered APPROACH(sign)**: cause not found (see 14 Nov) |

### In Gazebo again (14 Nov, a Mac, `pixi run -e ml sim-check --simulator gazebo --behaviour-only --perception`)

| Stop | Result |
|---|---|
| Stop sign | **passed**: APPROACH(sign) 2.0 m out, the lock fired at 0.50 m/s, the bumper 4.0 m before the sign. One run |
| After the sign's hold | its old point came back as APPROACH(sign) at the floor, until "sign lost": **fixed**, a sign already stopped for is dropped (`test_a_sign_stopped_for_never_brings_the_approach_back`) |
| Obstacle | the car went round the box: no stop to score |
| Goal | passed: APPROACH, lowest cap 0.47 m/s |

The 8 Nov miss didn't come back. Ruled out: the sign's camera frame (`camera_left_link`
is in `/tf`), the lookup at the picture's stamp (220 of 220 found, parked), and the cap
(the node keeps the sign in `odom`, so `left` keeps falling between pictures). A dropped
picture now logs `stop sign dropped: <why>`.

### The obstacle leg: a wall across the road (15 Nov, TODO 162)

**Why it failed:** `plan_around:=false` hides the box only from the planner. The path
follower's own local costmap (`mppi_lag`; Nav2's MPPI then) still sees it, and
steers round a 1 m box when there is room. At 1 m/s the stopping law's cap binds only in
the last 2 m, so by then the car was already beside the box: no APPROACH(obstacle), no stop
zone. It went round 2 of 4 runs on 8 Nov, and on 14 Nov; it stopped on 15 Nov. A coin toss.

**What the leg expects now:** the design says a parked box is passed and a person is
waited for. Passing a parked box is `--behaviour-things`' leg. So `--behaviour` puts a
**wall across the road** (10 m wide): nothing to go round. The car must slow
(APPROACH(obstacle)), the stop zone must stop it, and it drives on once the wall is gone.

| Run (our simulator, 15 Nov) | Result |
|---|---|
| ours, the 1 m box (before) | passed: stop zone at 0.43 m/s, 0.91 m before it |
| ours, a box in a walled lane (tried) | stopped at the box, but then stopped again for a wall it faced at an angle after rocking: goal aborted. Dropped |
| ours, the wall across | **passed**: stop zone at 0.45 m/s, 0.94 m before it, goal reached |
| stock (Nav2, its last runs), the wall across | **passed**: stop zone at 0.33 m/s, 1.04 m before it, goal reached |

At 1 m/s the law's cap binds only in the last 2 m, so the slowing is short.

### Not built yet

| Part | Why it waits |
|---|---|
| The `patient` progress checker | holds over 10 s still look stuck to the navigator's progress check |
| ASK's reply (`/hud/ask/answer`) | the HUD has no reply path |
| The car | the stage gates in the design: `pi` mode first |

## 3. Where the car really stops

`pixi run sim-check --stop-point` (the empty world, the spine in the loop, about 4 minutes).

| Step | What happens |
|---|---|
| 1 | a scripted drive at 0.5, 0.75 and 1 m/s (the spine's top speed: 1 m/s) |
| 2 | steady for 1.5 s, then the e-stop: the spine brakes fully |
| 3 | the prediction: the first `/ego/stop_point` after the press (ego, the shared car model, full brake) |
| 4 | moved to the map by `/tf` at its stamp |
| 5 | scored against the simulator's truth and the global EKF |

Pass: within 0.25 m, or 10 % of the distance if more. A drive that never reached half
its speed fails. The maths is `tools/stop_points.py`: it can score a bag from the car
later, with a tape measure for the truth.

### Numbers (8 Nov)

| From | Predicted | Truth | Localization | Stop point after the press |
|---|---|---|---|---|
| 0.54 m/s | 0.11 m | 0.12 m | 0.37 m | 3 ms |
| 0.77 m/s | 0.19 m | 0.18 m | 0.17 m | 43 ms |
| 0.98 m/s | 0.30 m | 0.24 m | 0.33 m | 83 ms |

The stopping law plans far more (0.95, 1.42 and 1.93 m): it plans a gentle stop at
1 m/s², not a full brake.

## 4. A heavier car: the adaptive brake

Ben, 8 Nov: "if we brake without a person vs with a person ... we are going to need to
dynamically realize that we aren't braking as fast as we expect and start applying more
force. We don't want to overwhelm the brakes though."

The same brake stops a heavier car more slowly (a = F / m). With 150 kg aboard, the
181 kg car brakes at 55 % of its empty strength.

### Where it sits

```text
 ego ─▶ speed loop ─▶ brake asked ─▶ adaptive brake ─▶ brake sent ─▶ brake actuator
                                         ▲
                                   wheel speed

 e-stop, stale command, silent spine ─▶ full brake   (the locks: never through it)
```

Below ego and every safety lock: it only adds to what the speed loop asks.

### What it does, each tick

| Step | What |
|---|---|
| 1 | measures the slowing from the wheel speed |
| 2 | works out the slowing the **empty** car model expects for the brake sent |
| 3 | learns the **effectiveness**: measured ÷ expected (1 empty, 0.55 with 150 kg) |
| 4 | brakes harder by 1 ÷ effectiveness (at most 3 times) |
| 5 | the extra rises at most 2 (full brake) a second |
| 6 | **never above the safe max** |
| 7 | near the safe max for longer than **max hold time**: the extra comes off |

It keeps what it learnt from stop to stop: a person aboard stays aboard.

### The brake's limits (`vehicle.yaml`, `brake:`)

Like the steering's ends. **Placeholders** until car day A7, rows 6-8
([car day](car-day.md#a7-the-brake-on-stands)).

| Key | Placeholder | Means |
|---|---|---|
| `released` | 400 | the brake actuator's target with the brake let go |
| `touch` | 800 | where the pads start to grip |
| `full` | 3500 | full brake |
| `safe_max` | 3200 | the most the brake may ever be pushed |
| `max_hold_time` | 5 s | how long near `safe_max` |

`released` and `full` must match the firmware's (`calibration.rs`); a test checks.

### Try it

| Command | What |
|---|---|
| `pixi run sim payload:=150` | a heavier car, the adaptive brake off |
| `pixi run sim payload:=150 adaptive_brake:=true` | the same, with it on |
| `ros2 param set /sim_adapter payload 90` | change the load while it runs |
| `pixi run sim-check --loaded-stop-only` | the check below |

Not with `spine:=true` or `esp32:=true`: there the firmware decides the brake.

### On the car (TODO 258, 19 Nov)

**Built; untested on the car.** Off by default.

| Command | What |
|---|---|
| `pixi run car motor_pi adaptive_brake:=true` | the adaptive brake in `legacy_uart_bridge`, after its pi speed loop |

```text
 ego ─▶ output mux ─▶ legacy_uart_bridge: pi loop ─▶ adaptive brake ─▶ frame ─▶ ESP32 ─▶ brake
                                                         ▲
                                               /vehicle/odom (wheels)
 e-stop, stale command ─▶ legacy_uart_bridge: full stop brake       (never through it)
```

| | |
|---|---|
| Where | `legacy_uart_bridge` (Rust, and the Python twin): the last place a brake level exists before today's ESP32 |
| The same maths | `firmware/spine/crates/adaptive-brake` is `adaptive_brake.py` in Rust; both pass `firmware/spine/adaptive_brake_vectors.txt` |
| Needs | `actuators:=uart` and `legacy.yaml`'s `speed_loop.mode: pi` (open asks for no brake): refused otherwise, saying why |
| Not yet | `actuators:=can`: the spine's firmware runs its own speed loop; refused there |
| Its numbers | the car model from `vehicle.yaml` `actuators:`, the limits from `brake:` (placeholders until car day A7) |

### The check: `--loaded-stop`

A stop from 2 m/s at 1 m/s², four times. The prediction is always the **empty** car
model's. Pass: within 0.25 m.

### Numbers (8 Nov): the car model alone

`test_adaptive_brake.py`, from 2 m/s; the empty model says 3.35 m.

| Aboard | Adaptive brake | Stopped in | Off by | Most brake |
|---|---|---|---|---|
| 0 kg | off | 3.35 m | 0.00 m | 0.30 |
| 150 kg | off | 3.93 m | **+0.58 m** | 0.42 |
| 0 kg | on | 3.35 m | 0.00 m | 0.30 |
| 150 kg | on | 3.40 m | +0.04 m | 0.59 |
| 150 kg | on, the next stop | 3.23 m | −0.12 m | 0.55 |

It learnt 0.55, the right share. The next stop brakes harder from the start, so it stops
a little short.

### In the simulator (8 Nov)

`pixi run sim-check --loaded-stop-only`, our simulator, the empty world. Each against
the empty car model fed the same commands.

| Aboard | Adaptive brake | From | Model | Truth | Off by | |
|---|---|---|---|---|---|---|
| 0 kg | off | 1.77 m/s | 3.00 m | 3.05 m | +0.04 m | ✅ |
| 150 kg | off | 2.05 m/s | 3.20 m | 4.08 m | **+0.87 m** | ✅ (must miss) |
| 0 kg | on | 1.77 m/s | 3.00 m | 3.05 m | +0.05 m | ✅ |
| 150 kg | on | 2.05 m/s | 3.22 m | 3.43 m | +0.21 m | ✅ |
| 150 kg | on, the next stop | 2.05 m/s | 3.22 m | 3.27 m | +0.05 m | ✅ |

The first loaded stop learns while it brakes; the next one starts knowing.
