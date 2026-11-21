# Actuator tests: measure how the car moves

One script runs the stage-3 and stage-4 tests of `tests.md`: it drives the car,
logs what happened, and prints the numbers for `vehicle.yaml`. Built and proven in Gazebo, the simulator then
(15 Oct); the same script runs on the car.

!!! note "The short version"
    - `pixi run actuator-test --list` shows the tests. Add `--sim` for Ravensim.
    - **On the car:** stage 3 on stands (`--stands`), stage 4 in an empty lot. It asks you
      to type the test's name first, and never goes above 1 m/s (0.3 on stands).
    - **To stop:** Ctrl-C, the e-stop, or the joystick's deadman. Any of them, any time.
    - It prints the `vehicle.yaml` lines to paste. It never writes the file itself.

## How it fits

```text
 actuator_test ──/drivers/tests/cmd_vel──► twist_mux ──► ego ──► backend ──► the car
   (priority 60)                               ▲                    (or Ravensim)
 joystick 100, RC 95, keyboard 90 ─────────────┘ always win
        │
        └── reads back: /vehicle/odom (speed), /vehicle/steering, /throttle, /brake,
            the IMU, /localization/odom/local, GPS, /safety/estop, /system/source
```

It's a driver like any other ([Build a driver](howto-driver.md)), in twist_mux's `tests`
slot: below every person (joystick, RC, keyboard), above navigation and every other program.

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
| `actuator-test preflight --stands` | 3.P | stands | speed up, hold, slow, brake; steer left, right, centre; every sensor counted | PASS / FAIL per line, no `vehicle.yaml` lines ([below](#the-preflight-on-stands)) |
| `wheel-steps` | 3.14 | stands | 0.1, 0.2, 0.3, 0.15 m/s, 4 s each: throttle on free wheels | does the wheel speed follow; no `vehicle.yaml` lines |

The steering tests creep at 0.2 m/s: ego only turns the wheels while the car moves. On
stands the wheels spin slowly; in the simulator the car crawls in arcs.

## In Ravensim

```bash
pixi run sim world:=empty rviz:=false nav:=false sim_speed:=3              # terminal 1
pixi run actuator-test speed-up --sim                                      # terminal 2
```

- `sim_speed:=3` lets ego go to 3 m/s, so `speed-up` and `brake` reach 2 and 3 m/s.
- `--sim` keeps ROS on this computer and uses the simulator's clock. The script refuses to run
  without it when the simulator announces SIM, and with it when anything announces REAL.
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
| **ego must answer** | on the car, no answer from ego (its speed cap and steering limits): refused. In the simulator, a warning |
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

## Find the steering actuator's ends and centre

**Why:** the car steers to the right. Its centre is a guess, and the 2025-26 C++ disagrees
with itself: centre 1950 and range 400-3500 in `setSteer()`, but centre 500 and range
0-1550 in `hardware_configs.h`; it also steers with two gains (1950 counts a unit one
side, 1550 the other). Our Rust firmware copied 1950 / 400-3500 / 3100 counts a radian
(`calibration.rs`, `TODO(Q1)`, `TODO(Q2)`). Nobody has measured the real ends.

**Gives:** the centre, equal limits each side of it, the counts a radian, and
`max_steering_angle`. Test 3.0 in [tests.md](tests.md#3b-steering-on-stands): do it
before 3.10. **3.10 then runs only inside the measured ends:** both ends outside today's
software limits (400-3500), or the measured ends flashed in `calibration.rs` first
([car day, A6](car-day.md#a6-the-steering-actuators-ends-way-a)).

!!! warning "Before any sweep"
    - **The car on stands**, all four wheels off the ground, the e-stop in someone's hand.
    - **The steering actuator's current limit set low first** (its Jrk's settings, in
      Pololu's utility), so a wrong target pushes gently against the stop. The value is
      the hardware lead's: **______** (write it here and on the sheet).
    - **The steering actuator's own (Jrk) limits are the last line of defence.** They are
      the hardware lead's to set. Our software limits come first; they don't replace them.
    - Hands clear of the wheels and the linkage while the motor is on.

### Counts: the unit here

A position or a target is a number of **counts**, 0 to 4095: the steering actuator's
feedback (its Jrk's "scaled feedback"), in the same units as its targets. Not radians:
radians are worked out *from* this calibration.

```text
   full left            centre                full right
   (left end)        (wheels straight)        (right end)
      |---- travel left ----|---- travel right ----|
            |-- half --|-- half --|        half = smaller travel - 5 %
            lo       centre       hi       the limits: the same each side
```

### Two ways

| | Way A: car day, whatever firmware is on the ESP32 | Way B: later, the Rust firmware with the uplink and the hardware panel |
|---|---|---|
| Needs | a laptop with Pololu's free **Jrk G2 configuration utility**, a USB cable to the steering actuator's Jrk | the Rust ESP32 firmware with `uplink` (session 6), the uplink's wire (H9), the Jrk's feedback mode set to analog |
| Reads | the utility's Status tab: feedback, scaled feedback, target | `/vehicle/steering_actuator/raw` (counts), in the panel |
| Moves the wheels | by hand, the motor stopped | **buttons:** small jog steps; or **by hand:** limp (motor off) |
| Who | the hardware lead, or someone they show | the laptop person, with the hardware lead there |

### Way A: Pololu's utility over USB (car day)

| # | Do | Write down |
|---|---|---|
| A1 | The car off. USB from the laptop to the steering actuator's Jrk. Open the utility; it finds the Jrk | its serial number |
| A2 | **Save its settings first, as a backup**: File, then Save settings file. Name it `steering-jrk-<date>.txt`; keep it with the car's files (not in git if it holds anything private) | the file's name |
| A3 | Read its feedback mode (Feedback tab): it must be **Analog**. Not analog: there is no position to read; stop, tell the hardware lead | the mode |
| A4 | With the hardware lead: the current limit low (Motor tab), applied | the old and new values |
| A5 | The ESP32 sends a steering target all the time, so the motor won't stay off. **Power everything off first.** Then, with the hardware lead, stop it reaching the Jrk: the ESP32 left off, or its I2C unplugged **with the power still off** (never with it on). Power the Jrk back, then press **Stop motor** in the utility | how |
| A6 | Turn the wheels by hand, gently, to **full left** (the steering stops, not the tyre on the frame). Read **Scaled feedback** | left end |
| A7 | Wheels **straight** (a straight edge on both front wheels, or a string line to the rear wheels). Read it | centre |
| A8 | **Full right**. Read it | right end |
| A9 | At each end, the wheel angle with a protractor (both front wheels; write the mean) | angles |
| A10 | The Jrk's own limits: change them **only with the hardware lead**. Otherwise leave everything as it was: load the backup (File, Open settings file) if anything changed | what changed |

Menu and tab names are from Pololu's user's guide; the version on the laptop may name
them a little differently.

### Way B: the hardware panel (Rust firmware)

**Not on car day.** Car day runs link version 2 throughout
([one link version on the car](car-day.md#one-link-version-on-the-car)). Way B comes later,
once the Rust firmware with the uplink is flashed and H9 says the wire is there. Then the
Motor Pi's `legacy.yaml`, for this session only, with the hardware lead's say-so:

```yaml
frame_version: 3
uplink: true
steering_calibration: stands     # back to none afterwards
```

Then, on the laptop (on the car's network, the car's `ROS_DOMAIN_ID`; [mac.md](mac.md)),
the hardware panel's **STEERING CAL** page (Tab):

```bash
pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py    # then Tab
```

Not `pixi run hw-panel`: it is localhost-only, so it never sees the car. The page:
[hud.md](hud.md#the-steering-calibration-page).

![The steering calibration page (--demo)](img/hw-panel-calibration.png)

| # | Do (button, key) | Expect |
|---|---|---|
| B1 | Nothing driving; the brake on. Open the page | **ALLOWED**; the bridge's line says `OFF`; the position in counts |
| B2 | **By hand:** GO LIMP (L) | firmware `LIMP`; the motor off; the counts follow the wheels |
| B3 | Full left: MARK LEFT (1). Straight: MARK CENTRE (2). Full right: MARK RIGHT (3) | the three marks; the limits and any warnings under them |
| B4 | **Or with the motor:** HOLD (H), then STEP LEFT / STEP RIGHT (Left, Right), 25 counts a press, a few seconds apart | the wheels move slowly (300 counts a second at most) |
| B5 | Stop a step before the steering binds or the motor strains; MARK that end | the end |
| B6 | The same toward the other end, and back to straight, MARK CENTRE | |
| B7 | HOLD near the centre, then LOCK-TO-LOCK CHECK (K) | centre to the left limit and back, to the right limit and back, at 100 counts a second; **PASSED**, or **STOPPED** and why, holding where the wheels are |
| B8 | SAVE (W) | the file's path on screen and in the log (below) |
| B9 | END (O) | `RETURN`, then `OFF`: the wheels go back slowly to the steering asked for |

**The check stops** (and holds) when the position falls 150 counts behind the target, stops
moving for 0.6 s while the target moves, misses a leg's end, is lost, or the firmware stops
jogging. A stop near an end: mark that end again, a little inside; then check again.

**Where the result goes.** SAVE writes
`~/.local/share/aav/steering-calibration/steering-<UTC time>.txt`: the marks, the check's
result, and the lines for `calibration.rs` and `vehicle.yaml`. The panel never writes
either file: copy them by hand, as below, with the sheet. For `max_steering_angle`, add
the wheel angles with the `steering_limits` command (the file has it, filled in).

Without the panel, from any laptop on the car's network:

```bash
ros2 topic echo /vehicle/steering_actuator/raw                 # position: counts
ros2 topic echo /vehicle/steering_actuator/calibration          # what it does, and why not
# limp (mode 1), sent at 10 Hz while it runs; Ctrl-C ends it within 0.5 s:
ros2 topic pub -r 10 /drivers/buttons/steering_calibration aav_interfaces/msg/SteeringCalibration "{mode: 1}"
# jog (mode 2) to a target, in counts:
ros2 topic pub -r 10 /drivers/buttons/steering_calibration aav_interfaces/msg/SteeringCalibration "{mode: 2, target: 2000}"
```

**Refused?** The line says why: not braked, a speed asked for, an e-stop, the RC or a
test driving, no position (the Jrk's feedback isn't analog), or the bridge's own rules
(`steering_calibration` is `none`, no uplink, ego driving, the car moving).

A jog target never goes outside today's software limits (400-3500). If an end lies
beyond them, use limp: the counts are read there all the same.

| What keeps it safe | Where |
|---|---|
| only standing still and braked; the throttle held at 0 throughout | the firmware (`legacy-link` `SteerCal`) and the bridge |
| jog clamped to the software limits, 300 counts a second | the firmware |
| leaving limp holds where the wheels are, then moves back slowly | the firmware |
| the Orin's heartbeat lost (100 ms): leave, and hold there | the firmware |
| the panel stops asking (0.5 s): leave | the bridge |
| the panel asks nothing with an e-stop, the car moving, or no position; its jog stays in 400-3500; its check stays inside the limits and stops on resistance | the panel (panel-core `calibration`, `sweep`) |
| `steering_calibration: none` by default | the bridge, `legacy.yaml` |

### The sheet

Print it; fill it in at the car. One sheet a session.

| | Left end | Centre | Right end |
|---|---|---|---|
| Counts | ________ | ________ | ________ |
| Wheel angle, left front (deg) | ________ | 0 | ________ |
| Wheel angle, right front (deg) | ________ | 0 | ________ |
| Wheel angle, the mean (deg) | ________ | 0 | ________ |

| Worked out (`steering_limits`) | Value |
|---|---|
| travel left, travel right (counts) | ________ , ________ |
| half = smaller travel - 5 % (counts) | ________ |
| limits: lo, hi (counts) | ________ , ________ |
| counts a radian (sign: + if left raises the count) | ________ |
| `max_steering_angle` (rad) | ________ |
| warnings | ________________________ |
| date, way (A or B), who, the hardware lead | ________________________ |

```bash
ros2 run aav_legacy steering_limits --left <counts> --centre <counts> --right <counts> \
    --left-deg <mean> --right-deg <mean>
```

It prints the lines for `firmware/spine/crates/spine-board/src/calibration.rs` (`Steering`:
centre, counts a radian, min, max) and for `vehicle.yaml` (`max_steering_angle`). It flags
a centre far from the middle of the travel, a lopsided travel, feedback that doesn't change
or is clipped at 0 or 4095, and two sides whose gains differ. A person puts them in, in a
pull request, with the sheet's photo; the hardware lead reviews it.

## The preflight on stands

One run before the car goes on the ground (Ben, 29 Oct; the design:
`docs/designs/2026-10-29_preflight.md`). It drives and checks every sensor at once.

```bash
pixi run actuator-test preflight --stands --sim                          # Ravensim (pixi run sim first)
pixi run actuator-test preflight --stands --bench                        # bench mode (pixi run bench first)
pixi run actuator-test preflight --stands --no-drive --skip steering     # the car today: the sensors only
pixi run actuator-test preflight --stands --skip steering gps lidar      # the car, once it has a rear-wheel speed
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

In the simulator and on the bench the car is **on the ground**, so the stands lines check the
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

## car-test: the whole stands test

Phase A of [the testing plan](testing-plan.md#phase-a-the-car-on-stands) in one command
(TODO 246). It runs every step below in order and writes **one report folder**.

```bash
pixi run car-test --stands                       # the real car, wheels off the ground
pixi run car-test --stands --sim --no-manual     # its own Ravensim (this computer only)
pixi run car-test --stands --only faults         # some steps (or a group: steering, faults)
```

| # | Step | How | Passes when |
|---|---|---|---|
| 1 | the preflight | `actuator-test preflight --stands` | every line PASS |
| 2 | every sensor | `actuator-test preflight --stands --no-drive` (15 s still) | each at its documented rate |
| 3a-c | steering | `steer-sweep`, `steer-rate`, `steer-centre` | its `vehicle.yaml` lines printed |
| 4 | throttle on free wheels | `wheel-steps` (new, 3.14): 0.1, 0.2, 0.3, 0.15 m/s, 4 s each | the wheel speed follows: each step up within 25 % (or 0.05 m/s), the step down slower |
| 5a | the Orin is lost | car: a person pulls the Orin's cable to the Motor Pi. Sim: the runner pauses its own output mux | car: the ESP32 brakes, and **holds** while still asked to roll once the cable is back. Sim: still within 2 s |
| 5b-d | GPS, IMU, camera lost | car: a person unplugs it. Sim: `fault.<sensor> stop` | the sensor watchdog names it within its `max_age` + 1 s; still 3 s later |
| 5e | lidar lost | car: a person unplugs it. Sim: `fault.lidar stop` | the sensor watchdog names it (its `lidar` input, since 19 Nov) within 1.0 s + 1 s; still 3 s later |
| 5f-g | a Pi's link; a cable | a person pulls it (car only) | the car stops, and something says why |
| 5h | the e-stop | car: a person presses it. Sim: the runner publishes `/safety/estop` true | still within 2 s |

**Each fault** starts from a slow roll (0.2 m/s, inside the stands' 0.3 cap) and records
**who said why** (the sensor watchdog's line, `/vehicle/status`, `/safety/estop`, the output
gate) and **how soon the car stood still**.

!!! warning "The sensor faults: the runner stops itself"
    The watchdog's lock (priority 30) holds navigation and the AI slot, **never the tests
    (60)**: that is by design. So the runner rolls, waits for the watchdog to **say** the
    sensor's name, then stops itself, as navigation would be held. The step measures how fast
    the watchdog notices, not a lock on the tests.

### The rules

| Rule | Why |
|---|---|
| `--stands` required; `--ground` refused | phase A is on stands only |
| Any FAIL **ends the run**: the steps after it are NOT RUN | the car stays on stands |
| After each step: zeros until still; not still in 5 s → `/safety/estop` **true** (never false) | as the preflight does |
| On the car: type `car-test` to start, and each test asks again | a person confirms every move |
| A person's step prints what to do and waits for Enter; `skip` skips it | the e-stop person stays at the button |
| `--no-manual`: every person's step is SKIPPED, and listed | for the stand-in and CI |
| `--sim`: its own Ravensim, its own ROS domain (`AAV_SIM_DOMAIN`, or a free one), `--local` | never reaches a car |
| The e-stop is the last step | it latches: nothing drives after it |

### The report folder

`~/aav-tests/<time>-stands[-sim]/`:

| File | Holds |
|---|---|
| `report.md` | the table: PASS / FAIL / SKIPPED / NOT RUN per step, what happened, the `vehicle.yaml` lines |
| `report.yaml` | the same, for tools |
| `run.log` | everything printed |
| `<step>/` | each test's own folder (`log.csv`, `summary.txt`, `meta.yaml`) and `output.log` |
| `sim-launch.log` | `--sim`: our Ravensim's log |

### No new hook for the heartbeat

The output mux makes the heartbeat. Rather than add a "stop the heartbeat" switch to a
safety lock, the runner:

- **in `--sim`**, pauses **its own** Ravensim's `output_mux` (SIGSTOP, then SIGCONT),
  found in its own launch log and checked in its own process group. Commands and heartbeat
  stop together; Ravensim's backend brakes on its 0.2 s timeout. The ESP32 isn't in
  Ravensim's loop: its own check is `pixi run esp qemu-check`;
- **on the car**, asks a person to pull the Orin's cable to the Motor Pi. The Pi keeps
  sending the last command with a **frozen counter** for 0.3 s; the ESP32 brakes after
  100 ms on its own. Once the cable is back, the runner still asks to roll for 2 s: the
  wheels must stay braked (the ESP32's latch), and `/vehicle/status` say disarmed or
  `CMD_TIMEOUT`.

### Tested, 19 Nov (Ravensim only)

`pixi run car-test --stands --sim --no-manual`, Ben's MacBook, its own Ravensim (road world,
domain 110), about 7 minutes:

| Step | Result |
|---|---|
| 1-2 preflight, sensors | PASS (Ravensim has no battery: skipped, and said) |
| 3a-c steering | PASS: lines for `max_steering_angle`, `steer_offset`, `steer_lag`, `steer_rate`, `steer_deadband` |
| 4 wheel-steps | PASS: 0.10 → 0.09, 0.20 → 0.21, 0.30 → 0.31, 0.15 → 0.23 m/s |
| 5a heartbeat | PASS: still 0.50 s after the output mux paused |
| 5b-d GPS, IMU, camera | PASS: the watchdog named each 2.11, 0.57, 1.09 s after its last message; still 0.41-0.45 s after its word |
| 5e lidar (since 19 Nov, TODO 246) | PASS: the watchdog named it 1.12 s after its last cloud (limit 2 s); still 0.45 s after its word |
| 5f-g Pi, cable | SKIPPED: not in Ravensim |
| 5h e-stop | PASS: still 0.67 s after it |

The first runs found: Ravensim's cameras build on first use (the runner now waits for the
camera), and the watchdog writes `STOP: gps STALE (...)`, not `STOP - ...` as the contract
said (fixed there). **On the car: untested.**

## car-test --hallway: the transfer functions

Phase B of [the testing plan](testing-plan.md#phase-b-the-transfer-functions-in-the-hallway)
in one command (TODO 248). It measures the throttle, the drag, the brake and the steering
in a hallway, **with the IMU as well as the wheels**, and gives each number **with an error
bar**.

```bash
pixi run car-test --hallway                      # the real car: asks the hallway's length
pixi run car-test --hallway --length 12          # the same, length given
pixi run car-test --hallway --sim --no-manual    # a pretend 15 m hallway in its own Ravensim
pixi run car-test --analyse ~/aav-tests/<session>   # the report again, from the files
```

```
 start line                                                     far wall
 |<------------------------- length (tape) ------------------------->|
 |  speed up -->  brake point  stop  |<------- margin (3 m) -------->|
 |<-------------- room ------------->|   never driven into
```

### What one pass does

| # | Phase | Why |
|---|---|---|
| 1 | wait until still, then **stand still 10 s** | the IMU's offset and noise (`fusion.still_imu`) |
| 2 | **let go**: told 0.02 m/s for 1.5 s | the speed loop lets the hold brake go, too gently to move the car |
| 3 | speed up to the top speed | the throttle |
| 4 | coast, brake, or steer left, right, left | the drag, the brake, the steering |
| 5 | stop | every pass ends still |

**Every driving step is fenced**: it ends at the **brake point**, where the car, at its
speed, needs all the room left to stop (0.3 s to react, then 0.5 m/s²). Past the room the
test's envelope stops it at once, with the margin still ahead.

| Pass | Test | Gives |
|---|---|---|
| B1 | `hall-speed-up` | `max_accel`, `throttle_lag`, `throttle_dead_time` |
| B2 | `hall-coast` | `drag` (and any rolling resistance) |
| B3 | `hall-brake` | `max_brake`, `brake_lag`, `brake_dead_time`, the stopping distance |
| B4 | `hall-steer` | `steer_lag`, `steer_dead_time` on the ground |

The session runs coast first (its drag goes into the others), then speed-up, brake, steer,
**three times** (`--repeats`). Between passes a person **turns the car round** (`--one-way`:
no), so the passes face both ways: the floor's slope shows up, then cancels.

### The top speed

From the length you type: speed up at 0.5 m/s², 0.3 s to react, stop at 0.5 m/s², plus the
margin. **Never above 1.0 m/s.**

| Hallway | Margin | Top speed |
|---|---|---|
| 4 m | 3 m | 0.64 m/s |
| 5.3 m | 3 m | 1.0 m/s |
| 12 m | 3 m | 1.0 m/s |
| under 3.2 m | 3 m | refused (under 0.2 m/s) |

### The IMU and the wheels

| Step | How |
|---|---|
| Offset | the IMU's mean while still: its bias + the slope |
| Slope | the offsets facing each way: bias = the mean, slope = half the difference. One way only: unknown |
| Level enough? | under 1 % (0.1 m/s²) |
| Blend | a Kalman filter: the IMU moves the speed on, each wheel speed pulls it back; it learns the IMU's bias drift |
| Forces | F = m a, with `vehicle.yaml`'s mass: the hardest push, the hardest stop, the drag at 1 m/s |

### Error bars and trust

Each pass gives its own fit; the number is their **mean ± sd** (and a bootstrap 90 %
interval). The bar is never under the time the data can resolve, nor under 3 % of a gain
(the wheel's size is known to about that until the rollout test, M3).

A number is **not trusted** when the data can't tell: one pass only, a fit under its
resolution, or passes that disagree (sd over 25 % of the mean). **Only trusted numbers are
printed to paste**; the rest are commented out, with why.

!!! note "Dead time and lag trade off"
    ego and the speed loop ramp the throttle, so a dead time and a lag look alike through
    it. Their **sum** (`throttle_delay`) is what the data pins down. When the dead time is
    not trusted, the line to paste is the lag = the whole delay, and no dead time (the model
    takes 0), as the single tests already do.

### The model against the car

Every pass's pedal demands and steering go through the car model, once with the fitted
numbers and once with `vehicle.yaml`'s. **Passes when** the speed is within 10 % (rms, of
the pass's top speed) and the stop within 0.2 m.

### The safety rules

| Rule | Why |
|---|---|
| Phase A passed first | stands before the ground |
| The hallway **measured with a tape**; the length typed, not guessed | the top speed and the room come from it |
| **A spotter at each end**, the hallway cleared | people walk there |
| A safety driver on the joystick (its deadman outranks the test); someone else at the e-stop, walking beside the car | the one sure stop |
| The car started **indoors**: `pixi run car <machine> gps:=false stands:=true` | no GPS indoors, so the geofence can't fence: it says STANDS. Only the hallway tests accept that; this test's own fence (the wheels' distance) stands in. NEAR, OUTSIDE, LOST, JUMP still stop it |
| Type `hallway` to start; each pass asks again (its test's name) | a person confirms every move |
| Before each pass: put the car on the start line, facing the way asked; Enter (`s` skips, `q` stops) | the room counts from the start line |
| Any pass refused or stopped by a safety rule **ends the session** | as phase A |
| Every stop rule of the other tests still holds | Ctrl-C, the e-stop, a takeover, stale odometry, a runaway |

### Step by step, on the car

1. Measure the hallway with a tape: start line to the far wall. Mark the start line.
2. Clear it; a spotter at each end.
3. Start the car indoors: `pixi run car <machine> gps:=false stands:=true`.
4. Put the car on the start line, pointing straight down the hallway.
5. `pixi run car-test --hallway` (or `--length 12`); read the top speed it works out.
6. Check the list out loud; type `hallway`; type each pass's name when asked.
7. Between passes: turn the car round (or drive it back with the joystick) onto the line.
8. Read `report.md`; paste the trusted lines from `vehicle-yaml.txt`, by hand.

### The report folder

`~/aav-tests/<time>-hallway[-sim]/`:

| File | Holds |
|---|---|
| `report.md` | the result, the stand-still and slope, every number ± its bar and trust, the forces, the model against the car, the lines to paste, the passes |
| `vehicle-yaml.txt` | the lines to paste (trusted only) |
| `fits.csv` | each number: mean, sd, n, interval, resolution, trust, and each pass's own value |
| `compare.csv` | each pass: the model's rms speed error and stop error, fitted and `vehicle.yaml`'s |
| `overlays/` | each pass's speed, measured and both models (CSV; `overlays.png` with matplotlib) |
| `session.yaml` | the hallway, the passes, in Ravensim the true numbers |
| `NN-hall-<kind>-<facing>/` | each pass's own `actuator_test` folder and `output.log` |

### In Ravensim

`--sim` starts its own Ravensim: the empty world, no cameras, its own ROS domain, 15 m
of pretend hallway. `--sim-actuators "throttle_lag=0.3,drag=0.2,..."` gives **the
simulated car its own numbers** (`sim.launch.py actuators:=`), while every driver still
reads `vehicle.yaml`; the report then adds **the recovery table**: true against fitted.
`--wheel-noise 0.05 --wheel-slip 0.02` makes the wheels noisy and slipping.

### Proven: known numbers found again

The key evidence: give the car numbers we know, run the whole procedure, and check the fits
find them. "Within" = within 2 error bars (the bar: the passes' sd, at least the
resolution, at least 3 % of a gain). 19 Nov, Ben's MacBook, 12 passes each (3 x coast,
speed-up, brake, steer), about 6 minutes each:

| Number | Ravensim A: true | fitted ± sd | Ravensim B (wheels 0.02 m/s noise, 2 % slip): true | fitted ± sd | Car model alone (0.05 m/s noise, 2 % slip, IMU bias): true | fitted ± sd |
|---|---|---|---|---|---|---|
| `drag` (1/s) | 0.2 | 0.202 ± 0.001 | 0.15 | 0.150 ± 0.004 | 0.2 | 0.199 ± 0.014 |
| `max_accel` (m/s²) | 1.2 | 1.204 ± 0.002 | 1.0 | 1.016 ± 0.002 | 1.2 | 1.214 ± 0.004 |
| `throttle_lag` (s) | 0.3 | 0.335 ± 0.085 | 0.25 | 0.228 ± 0.024 | 0.3 | 0.320 ± 0.048 |
| `throttle_dead_time` (s) | 0.1 | 0.085 ± 0.076 | 0.06 | 0.090 ± 0.025 | 0.1 | 0.085 ± 0.073 |
| throttle delay, the sum (s) | 0.4 | 0.420 ± 0.025 | 0.31 | 0.318 ± 0.018 | 0.4 | 0.405 ± 0.025 |
| `max_brake` (m/s²) | 2.5 | 2.582 ± 0.169 | 2.0 | 1.850 ± 0.133 | 2.5 | 2.537 ± 0.321 |
| `brake_lag` (s) | 0.25 | 0.340 ± 0.127 | 0.3 | 0.275 ± 0.005 | 0.25 | 0.145 ± 0.126 |
| `brake_dead_time` (s) | 0.06 | 0.052 ± 0.050 | 0.1 | 0.122 ± 0.052 | 0.06 | 0.148 ± 0.115 |
| brake delay, the sum (s) | 0.31 | 0.392 ± 0.081 | 0.4 | 0.397 ± 0.056 | 0.31 | 0.293 ± 0.023 |
| `steer_lag` (s) | 0.2 | 0.200 ± 0.000 | 0.15 | 0.150 ± 0.000 | 0.2 | 0.200 ± 0.000 |
| `steer_dead_time` (s) | 0.08 | 0.062 ± 0.000 | 0.04 | 0.023 ± 0.000 | 0.08 | 0.062 ± 0.000 |

**Every number within its error bars, in all three.** The fitted model matched all 12
passes each time (speed within 3 %, stop within 0.1 m); `vehicle.yaml`'s guesses matched
none (40-70 % off, stops 0.4-1.0 m off).

What it shows:

- The **gains** (`drag`, `max_accel`, `max_brake`) and the **sums** (delays) are pinned; a
  dead time and a lag apart often are not, and say so (NOT TRUSTED): only the sum is then
  pasted, as the lag.
- With 2 % wheel slip, `max_accel` reads 2 % high: the wheel's size, not the fit (the 3 %
  floor on the bar). The rollout test (M3) fixes it.
- The steering's dead time reads 0.02 s short: the model's deadband (0.005 rad) starts the
  motor a step late, which the fit counts as lag. Within its 0.04 s resolution.

!!! warning "Noisy wheels trip the runaway rule"
    With Ravensim's `wheel_noise:=0.05` **every** actuator test stops itself, even standing
    still: `safety.runaway` compares two single speed readings 1 s apart against 0.1 m/s, and
    0.05 m/s of noise on each crosses that. The rule is right to stop; the noise is too big
    for it. So the 0.05 m/s proof ran on the car model alone (`test/fake_hallway.py`, in the
    tests), and Ravensim's at 0.02 m/s. The car's Hall sensors read exactly 0 standing still,
    so this is unlikely on the car, but **watch for it**: the rule is unchanged (a safety
    rule; a person decides). Each pass also holds its top speed 2.5 s before it slows, so a
    speed loop still creeping up doesn't look like a runaway.

Run it again: `pixi run car-test --hallway --sim --no-manual --sim-actuators
"max_accel=1.2,throttle_lag=0.3,throttle_dead_time=0.1,max_brake=2.5,brake_lag=0.25,brake_dead_time=0.06,drag=0.2,steer_lag=0.2,steer_dead_time=0.08"`;
the car model alone (seconds): `pixi run python ros/drivers/aav_actuator_tests/test/fake_hallway.py`.

**On the car: untested.**

A hallway shorter than 15 m gives short coasts: the drag then comes out NOT TRUSTED.

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
| After 4.1, 4.2, 4.4 (`max_accel`, `drag`) | `pixi run python3 tools/vehicle_model.py --set max_accel=... --set drag=... --speed-loop` | `smooth` still 0% at K with the measured numbers; if not, redo its gains (pole placement, then b) |
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
| `backend` | `legacy`, `spine`, `ravensim`, `ravensim-spine`, `ravensim-ideal`, `bench` (`gazebo...` in runs before 16 Nov) | `unknown` |
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
    0.6). `sim_imu` now gives it the car's acceleration ([Simulation](simulation.md)).

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
| `car_test.py`, `car_test_node.py`, `test/test_car_test.py` | car-test: its steps and judging (no ROS, unit-tested), and its ROS runner |
| `tools/car_test.sh` | the pixi task `car-test`: `--local` only with `--sim` |
