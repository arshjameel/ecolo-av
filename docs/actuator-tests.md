# Actuator tests: measure how the car moves

One script runs the stage-3 and stage-4 tests of `tests.md`: it drives the car,
logs what happened, and prints the numbers for `vehicle.yaml`. Built and proven in Gazebo
(15 Oct); the same script runs on the car.

!!! note "The short version"
    - `pixi run actuator-test --list` shows the tests. Add `--sim` for Gazebo.
    - **On the car:** stage 3 on stands (`--stands`), stage 4 in an empty lot. It asks you
      to type the test's name first, and never goes above 1 m/s (0.3 on stands).
    - **To stop:** Ctrl-C, the e-stop, or the joystick's deadman. Any of them, any time.
    - It prints the `vehicle.yaml` lines to paste. It never writes the file itself.

## How it fits

```text
 actuator_test ──/drivers/tests/cmd_vel──► twist_mux ──► ego ──► backend ──► the car
   (priority 60)                               ▲                    (or Gazebo)
 joystick 100, RC 95, keyboard 90 ─────────────┘ always win
        │
        └── reads back: /vehicle/odom (speed), /vehicle/steering, /throttle, /brake,
            the IMU, /localization/odom/local, GPS, /safety/estop, /system/source
```

It's a driver like any other ([Build a driver](howto-driver.md)), in twist_mux's `tests`
slot: below every person (joystick, RC, keyboard), above Nav2 and every other program.

## The tests

| Test | # | Where | What it does | Gives (`vehicle.yaml`) |
|---|---|---|---|---|
| `steer-sweep` | 3.10 | stands | steps full left → centre → full right and back, 2.5 s each | angle per command; `max_steering_angle`, `steer_offset` |
| `steer-rate` | 3.11 | stands | centre → full lock → centre, each way | `steer_rate`, `steer_lag` |
| `steer-centre` | 3.12 | stands | straight, then tiny steps (0.002 rad) each way | `steer_offset`, `steer_deadband` |
| `speed-up` | 4.1 | lot | from still to the speed cap | time to 0.5, 1, 2, 3 m/s; g (wheels and IMU); `max_accel`, `throttle_lag` |
| `coast` | 4.2 | lot | to 2 m/s, then neither throttle nor brake | `drag` |
| `brake` | 4.3 | lot | to 1, 2, 3 m/s (up to the cap), then stop | stopping distance, g; `max_brake`, `brake_lag` |
| `throttle-map` | 4.4 | lot | speed-ups to ¼, ½, ¾ and all of the cap | throttle % → acceleration; `throttle_deadband` |
| `circles` | 4.5, 4.6 | lot | one full circle per angle, both ways, at 1 m/s | real wheel angle per command; tightest circle |
| `straight` | 4.7 | lot | 10 m at 0.5 m/s, steering straight | sideways drift; `steer_offset` |
| `preflight` | 3.P | stands | speed up, hold, slow, brake; steer left, right, centre; every sensor counted | PASS / FAIL per line, no `vehicle.yaml` lines ([below](#the-preflight-on-stands)) |

The steering tests creep at 0.2 m/s: ego only turns the wheels while the car moves. On
stands the wheels spin slowly; in Gazebo the car crawls in arcs.

## In Gazebo

```bash
pixi run sim world:=empty gui:=false rviz:=false nav:=false sim_speed:=3   # terminal 1
pixi run actuator-test speed-up --sim                                      # terminal 2
```

- `sim_speed:=3` lets ego go to 3 m/s, so `speed-up` and `brake` reach 2 and 3 m/s.
- `--sim` keeps ROS on this computer and uses Gazebo's clock. The script refuses to run
  without it when Gazebo announces SIM, and with it when anything announces REAL.
- `nav:=false`: nothing else drives. The empty world has room for the circles.

## On the car

The task is **not** `--local`, so it reaches the car's computers. Run it from the laptop
on the car's network, with the car's stack running (`pixi run car ...`).

### Stage 3: on stands

```bash
pixi run actuator-test steer-sweep --stands
pixi run actuator-test steer-rate --stands
pixi run actuator-test steer-centre --stands
```

1. The script prints the checklist from `tests.md` stage 3. Check it out loud.
2. Type the test's name to start. Anything else cancels; nothing moves.
3. **No steering sensor yet?** (The 2025-26 ESP32 reports none: team-todo.md B6.) At each
   measuring point the car stops and asks for the angle in degrees: read it with a
   protractor or a phone app on the wheel. `steer-rate` then needs a video instead.

### Stage 4: in an empty lot

```bash
pixi run actuator-test speed-up
pixi run actuator-test coast
pixi run actuator-test brake --speeds 0.5 1.0
pixi run actuator-test circles --angles 0.25 0.5
pixi run actuator-test straight
```

Run `coast` first: the throttle fit uses `vehicle.yaml`'s `drag`. Then paste the drag in,
then run the rest. Start at walking pace: `--max-speed 0.5`.

### The safety layers

| Layer | What it does |
|---|---|
| **Typing the test's name** | on the real car (or when nobody announces `/system/source`), nothing moves until you type it. No keyboard (a script, a pipe): refused |
| **Speed cap** | 1 m/s on the car, 0.3 m/s on stands; `--max-speed` can lower it, never raise it there |
| **ego must answer** | on the car, no answer from ego (its speed cap and steering limits): refused. In Gazebo, a warning |
| **Stage check** | on the car, stage-3 tests need `--stands`, stage-4 tests refuse it |
| **Envelope** | each test has a speed, distance and time limit (printed at the start): crossing one stops the car |
| **Ctrl-C, errors** | zero commands until the car stands still, then it exits. A second Ctrl-C can't cut that short |
| **`/safety/estop`** | stops the test at once (twist_mux's lock and the backend brake too) |
| **Someone takes over** | the joystick or keyboard outranks the test in twist_mux; the script sees it and stops |
| **Stale speed** | no `/vehicle/odom` for 0.5 s: stop |
| **Nothing through twist_mux** | 2 s with no `/control/cmd_vel`: stop (a lock, or a mux without the `tests` slot) |
| **A speed nobody sees** | told to drive, but under 0.05 m/s measured for 4 s: stop (4, not 2: a Hall sensor with 8 clicks a turn says nothing until 2 clicks, about 0.24 m, which a slow start takes 2-3 s to roll; seen on the bench, 29 Oct). On stands the front wheel sensors read 0 while the rear (driven) wheels spin ([TODO 49b](../TODO.md)), so the speed cap can't work |
| **A runaway** | the speed rises 0.1 m/s in 1 s while the command falls or sits at 0 (a stuck throttle): stop |
| **A cut-short run** | still saves its log and table, but prints no `vehicle.yaml` lines |

!!! warning "ego's steering rate"
    `steer-rate` lifts ego's `max_steering_rate` (0.6 rad/s) to 3 rad/s while it runs, so it
    times the motor, not ego, and puts it back after. ego allows a live change to that one
    limit only. If the script is killed (not Ctrl-C), restart ego to be sure.

### What the car can measure today

The 2025-26 ESP32 drives the car until the spine replaces it (`legacy_uart_bridge`).

| Measurement | Today (ESP32) | With the spine |
|---|---|---|
| Speed | ✅ the front wheel sensors (the hardware lead's Arduino on CAN, `spine_bridge role:=sensors`), `/vehicle/odom` | ✅ |
| IMU g, yaw rate | ✅ once the IMU runs | ✅ |
| Pose (circles, straight) | ✅ the local EKF; GPS logged too | ✅ |
| Steering angle | ❌ no sensor: type it in on stands (✅ with the uplink, once H9 says yes, if the Jrk has feedback) | ✅ `/vehicle/steering` |
| Throttle and brake levels | ❌ not reported: no `max_accel` or `max_brake` fit, only the g figures. A speed command **is** the throttle there: speed ÷ `full_throttle_speed` (`legacy_protocol.py`). ✅ with the uplink (H9) | ✅ `/vehicle/throttle`, `/vehicle/brake` |
| Coasting | ❌ a low speed command still pushes, and 0 brakes | ✅ inside the speed loop's coast band |
| A fixed brake % (4.3 as written) | ❌ no brake command exists: the tests brake with a normal stop | ❌ the same |

What each fit can still give, with what is missing:

| Missing | The fit falls back to | Gives |
|---|---|---|
| Throttle level (ESP32) | ego's speed ÷ `full_throttle_speed` → wheel speed | `max_accel`; the delay only if the pulses resolve it |
| Brake level | brake demand → IMU deceleration | `max_brake`, `brake_lag`, `brake_dead_time` |
| Brake level **and** demand | "stop asked for" → IMU deceleration | the whole stop's dead time and lag; no `vehicle.yaml` line |
| Fine wheel speed (Hall pulses) | each speed compared with the model's mean since the last pulse | the gain; a delay under 2 pulse intervals is refused |

## The preflight on stands

One run before the car goes on the ground (Ben, 29 Oct; the design:
`docs/designs/2026-10-29_preflight.md`). It drives and checks every sensor at once.

```bash
pixi run preflight --sim                          # Gazebo (pixi run sim first)
pixi run preflight --bench                        # bench mode (pixi run bench first)
pixi run preflight --no-drive --skip steering     # the car today: the sensors only
pixi run preflight --skip steering gps lidar      # the car, once it has a rear-wheel speed
```

| Step | Command |
|---|---|
| still, settle 3 s | 0 (the IMU's bias, the GPS's start) |
| speed up, hold 5 s | 0.3 m/s |
| slow down 3 s | 0.15 m/s |
| brake | 0, until still |
| steer left, right, centre | 3 s each, creeping at 0.2 m/s |

Then one line per check: `PASS`, `FAIL`, `WARN` or `SKIP`.

| Lines | PASS when |
|---|---|
| one per sensor | at least 0.8 x its documented rate (IMU 200 Hz, wheels 100, GPS 1, lidar and radar 10, camera 15), no gap over 5 messages, values in range (gravity 9.81 ± 0.5 at rest) |
| speeds up, holds, slows, brakes | 80% of the cap; within 25% in the hold (the open-loop ESP32: a WARN with its throttle gain); still within 5 s |
| steers | left and right measured, centre within `steer_offset` ± (2 x `steer_deadband` + 0.02 rad); no sensor: it asks you |
| stands: IMU still | its speed change while speeding up under 0.4 x the wheels' |
| stands: GPS still | Doppler under 0.1 m/s; with no Doppler only a WARN |

!!! warning "On the car, the drive is refused today"
    The car is rear-wheel drive; its wheel sensors are on the **front**. On stands
    `/vehicle/odom` reads 0 while the motor spins, so no speed cap can work
    ([TODO 49b](../TODO.md)). Until `vehicle.yaml`'s `speed_wheels` says `rear`, run
    `--no-drive`: the sensors only, nothing moves. Also refused: the C++ ESP32 (its
    no-timeout lock), and the bridge on `speed_loop.mode pi`.

In Gazebo and on the bench the car is **on the ground**, so the stands lines check the
opposite (the IMU and GPS agree with the wheels) and say so. Who must be there on the car:
the **e-stop person** (they are the stop), someone watching the **rear wheels** with a
tachometer app, the laptop person; run it on the Orin (ssh + tmux) or a wired laptop.
The word to type: `preflight`. If the car is still moving 5 s after the stop, it prints
**NOT STOPPED: E-STOP NOW** and sets `/safety/estop`.

### Tested, 29 Oct

| Where | Result |
|---|---|
| Gazebo, empty world (`--sim --skip camera radar battery`) | every drive and sensor line PASS: 0.34 m/s held for 0.30, still 0.5 s after the brake, steering +0.50 / -0.50 / 0.000 rad, IMU 0.25 vs wheels 0.25 m/s, Doppler 0.36 vs 0.34. Lidar FAIL: the empty world has none (name it with `--skip lidar`) |
| Bench (`--bench`), spine armed | every sensor line PASS (IMU 50 Hz, \|a\| 9.78; GPS 5 Hz; camera 640x360 10 Hz; battery 50.9 V). The drive stopped itself: the front wheels read 0 for over 2 s of a slow start (8 Hall clicks a turn), so the "speed nobody sees" stop, then at 2 s, fired; now 4 s. The run again at 4 s: not yet done |
| `--no-drive`, the real car | untested: no car |

## Trusting the fits

Each lag fit is now a **dead time, then a lag** (FOPDT). The old fit (a lag alone)
stays beside it, so you can compare.

```text
 command ──► wait (dead time) ──► first-order lag (τ) ──► measured
```

| Rule | What it means |
|---|---|
| **The path is named** | each row and each `vehicle.yaml` line says *from* which signal *to* which: e.g. `throttle level -> wheel speed`, never `cmd_vel -> speed` (that would measure ego's ramp and the speed loop too) |
| **The rms is printed** | with each fit, in the signal's units; the old fit's rms beside it |
| **True times** | the fits use each message's arrival time (`t_*` columns), not the 50 Hz copies |
| **A dead time counts** | only if it is longer than the resolution, and fits at least 1.1 × better than a lag alone |
| **Refused, with a reason** | a lag shorter than the data can resolve gets no `vehicle.yaml` line, and a NOTE says why |

**The resolution** is two fresh values (Nyquist), plus what noise blurs:

```text
resolution = 2 × (time between fresh values) + rms ÷ (slope × √moving samples)
```

| Signal | Fresh values every | Resolution |
|---|---|---|
| a level at 50 Hz | 0.02 s | 0.04 s |
| Hall wheel speed, 8 holes on a 0.30 m wheel, at 1 m/s | 0.118 m ÷ 1 m/s = 0.12 s | 0.24 s |
| ... at 0.5 m/s | 0.24 s | 0.47 s |

So at walking pace a 0.1 s throttle lag can't be read from the wheel speed: use the
throttle level, or the IMU. `analysis.pulse_interval()` works it out before a run.

**Steering play** is measured first (`steer_play`: where it settles after moves up and
moves down), then the lag is fitted with it. Left out, play looks like a longer lag.

**Tested on pretend data** (`test/test_robustness.py`) with play, Hall pulses, noise,
no brake level and known dead times:

| Case | Tolerance it meets |
|---|---|
| dead time + lag, from a level | 10 ms (40 ms with 5% noise) |
| steering with 0.02-0.04 rad play | rate 3%, play 3 mrad, lag 10 ms, dead time 25 ms |
| Hall pulses, up to 3 m/s | `max_accel` 2%; total delay 40 ms or 10% |
| Hall pulses, under 1 m/s | `max_accel` 5%; the 0.15 s delay **refused** |
| brake from the IMU only | `max_brake` 2%; lag and dead time 20 ms |

## The speed loop's profile (TODO 27)

The spine's speed loop has two profiles ([Tuning the speed
loop](vehicle-model.md#tuning-the-speed-loop)): `default`, today's gains (overshoots about
15%), and `smooth` (no overshoot on the model, robust to the throttle's 5x guess). The
firmware keeps `default`. Try `smooth` here, in this order:

| When | What to run | What you should see |
|---|---|---|
| Before the car | `pixi run sim-check --speed-steps-only --speed-loop smooth` | each step ends within 5%, overshoot under 3% |
| After 4.1, 4.2, 4.4 (`max_accel`, `drag`) | `pixi run vehicle-model --set max_accel=... --set drag=... --speed-loop` | `smooth` still 0% at K with the measured numbers; if not, redo its gains (pole placement, then b) |
| Stage 3, on stands, **once the spine closes the loop** (it has the wheel speed) | build the board with `speed_loop: SpeedLoopConfig::smooth()` in its `SpineConfig`; a held step (below) at 0.3 m/s | the wheels reach 0.3 m/s without passing 0.31; `/vehicle/throttle` steady, no jitter |
| Stage 4, in the lot | the held step at 0.5 and 1.0 m/s, each profile, the same day | the peak speed in `/vehicle/odom` against the target; then pick one |

The held step is a scripted drive, 20 s at the target (the `speed-up` test stops 1 s after
reaching 95%: too soon to see the peak, which comes 5-7 s in):

```bash
ros2 action send_goal /scripted_drive aav_interfaces/action/ScriptedDrive \
  "{target_speed: 0.5, acceleration: 1.0, deceleration: 1.0, hold_time: 20.0}" --feedback
```

!!! warning "Today's ESP32 has no closed speed loop"
    It drives the throttle open loop (speed ÷ `full_throttle_speed`), so neither profile
    runs on the car until the spine gets the wheel speed. On stands the wheels carry no
    load: the car is nearly a pure integrator there, the case where today's gains overshoot
    most. The e-stop person watches.

## What it saves

Each run gets a folder, `~/aav-tests/<date>-<time>-<test>[-sim]/` (`--out` or
`$AAV_TESTS_DIR` moves it):

| File | Holds |
|---|---|
| `log.csv` | 50 rows a second: commands, speed, steering, throttle, brake, wheels, IMU, pose, GPS, ego's command; and each value's arrival time (`t_speed`, `t_imu`...) |
| `summary.txt` | the table and the `vehicle.yaml` lines, as printed |
| `meta.yaml` | the test, the source, the cap and envelope, ego's limits, `vehicle.yaml` then, the steps; and `shaping`: what shaped the commands |
| `bag/` | with `--bag`: a rosbag of every topic it reads and sends |

`pixi run actuator-test --analyse <folder>` prints the report again from `log.csv`.
Older logs (no `t_*` columns, no `shaping`) still work: the fits use the rows' times.

`shaping` in `meta.yaml`:

| Key | Holds | When unknown |
|---|---|---|
| `backend` | `legacy`, `spine`, `gazebo`, `gazebo-spine`, `bench` | `unknown` |
| `speed_loop` | the profile (`default`, `smooth`), or "none: open loop" on the ESP32 | on the car's spine: built into the firmware, so write it down |
| `full_throttle_speed` | the legacy bridge's (its open loop) | `unknown`, or `n/a` |
| `ego`, `ego_changed` | ego's limits (its ramp is before the backend); what the test lifted | "ego did not answer" |

## Proven in Gazebo (15 Oct)

`world:=empty`, `sim_speed:=3`, `vehicle.yaml`'s guessed numbers in the actuator model.
The script should give them back:

| Number | `vehicle.yaml` | Measured | Test |
|---|---|---|---|
| `max_accel` | 1.5 m/s² | 1.496 | `speed-up`; 1.495 in `throttle-map` |
| `throttle_lag` | 0.1 s | 0.100 | `speed-up` |
| `throttle_deadband` | 0 | 0.113 ⚠️ | `throttle-map`: Gazebo's car starts rolling about 0.3 s after the model does |
| `drag` | 0.1 /s | 0.100 | `coast` |
| `max_brake` | 3.0 m/s² | 3.045 | `brake` |
| `brake_lag` | 0.2 s | 0.200 | `brake` |
| `steer_rate` | 0.8 rad/s | 0.817 | `steer-rate` |
| `steer_lag` | 0.1 s | 0.105 | `steer-rate` |
| `steer_offset` | 0 | 0.000 | `steer-centre`, `steer-sweep`, `straight` |
| `steer_deadband` | 0.005 rad | 0.005 | `steer-centre` |
| `max_steering_angle` | 0.5 rad | 0.495 | `steer-sweep` (the deadband leaves 0.005 short) |
| real angle, full-lock circle | 0.5 rad | 0.484 (was 0.433) | `circles`: fixed 15 Oct (below) |
| real angle at 0.2 / 0.35 rad | 0.2 / 0.35 | 0.196 / 0.341 | `circles`: yaw rate, odometry circle and GPS circle agree within 1% |

Other figures from the same runs: 0 → 3 m/s in 6.2 s; IMU and wheel-speed g agree
(0.054 vs 0.056 g mean); stopping from 1.1 / 2.2 / 3.1 m/s took 1.05 / 2.57 / 4.46 m;
drift over 10 m was 0.06 m; the full-lock circle was 6.8 m across, not 5.75 (6.0 m since the fix below).

!!! success "Fixed: Gazebo's car now reaches full lock (S3, 15 Oct)"
    Told 0.5 rad, the car turned as if 0.433. The cause was Gazebo's Ackermann system
    (gz-sim 8): it turns `steering_limit` into a smallest circle, wheelbase ÷ sin(limit),
    so a limit of 0.5 stopped the middle wheel at 0.447. Tyre slip (about 3%) took the
    rest. Now `vehicle.gazebo_steering_limit` gives Gazebo 0.578, worked out from
    `max_steering_angle`. `vehicle.yaml` and the real car's meaning don't change.

    | `circles --sim`, 1 m/s | Before | After |
    |---|---|---|
    | full lock, left / right | 0.433 | 0.485 / 0.484 rad (3% short) |
    | 0.35 rad, left / right | 0.339 | 0.342 / 0.341 |
    | 0.2 rad, left / right | 0.194 | 0.197 / 0.196 |
    | circle across, full lock | 6.8 m | 5.93-5.98 m |
    | Gazebo's wheels at full lock, inside / outside | 0.498 / 0.405 (worked out) | 0.563 / 0.449 (Ackermann: 0.563 / 0.449) |

    The wheels now point exactly where Ackermann says. The last 3% is the tyres
    sliding sideways, at every angle: the real car will slide too, and `circles` on the
    car measures it. `steer-sweep --sim` still reads 0.495 (the deadband).

!!! note "Gazebo's IMU"
    Gazebo's own IMU never felt the car speed up or brake (0.00 m/s² while speeding up at
    0.6). `gazebo_imu` now gives it the car's acceleration ([Simulation](simulation.md)).

### Checked in Gazebo and against a pretend real car (15 Oct)

| What was done | What happened |
|---|---|
| Ctrl-C during `speed-up`, at about 1.5 m/s | "STOPPING: Ctrl-C"; stood still, exited in 3.2 s |
| `/safety/estop` true during `speed-up` | "STOPPING: /safety/estop is true"; stood still |
| The keyboard slot sends zeros during `straight` | "someone else is driving"; stood still |
| Gazebo, without `--sim` | refused: add `--sim` |
| A pretend REAL announcer, no keyboard attached | checklist printed, then refused |
| ... a stage-3 test without `--stands` | refused |
| ... `--sim`; `--max-speed 1.5` | refused; refused |
| ... typed "yes" | "Cancelled: nothing moved" |
| ... typed "speed-up", but no car | refused: no `/vehicle/odom` |

## The code

| File | What |
|---|---|
| `ros/drivers/aav_actuator_tests/aav_actuator_tests/actuator_test.py` | the ROS node and the command line |
| `procedures.py` | each test's steps |
| `safety.py` | the rules: REAL or SIM, caps, envelope, confirmation |
| `analysis.py`, `reports.py` | the maths and the report: no ROS, unit-tested |
| `test/synthetic.py`, `test/test_robustness.py` | pretend data with play, Hall pulses, noise; the fits must give the numbers back or refuse |
| `test/fake_car.py` | ego's model + Gazebo's actuator model: the tests check each test gets the numbers back |
| `tools/actuator_test.sh` | the pixi task: `--local` only with `--sim` |
