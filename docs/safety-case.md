# The safety case

One section per hazard: **what stops it**, **the number** that shows it works, and **who
measured it**. TODO 159, from [the whole car](designs/2026-11-14_the-whole-car.md)
("What is missing", row 8) and [weak links](designs/2026-11-04_weak-links.md) 4.7.

!!! warning "Nothing here has been measured on the car"
    Every number below comes from our simulator, Gazebo, bench mode, QEMU or unit tests.
    The car drove on 13 Nov "and stopped well" (Ben), with only the localization update
    installed: no recording, no numbers ([status](status.md#on-the-car)).

## How to read it

| Word | Means |
|---|---|
| **car** | measured on the real car, with a date |
| **bench** | [bench mode](bench.md): our software with stand-in hardware |
| **sim** | our simulator or Gazebo |
| **QEMU** | the real ESP32 firmware in an emulator |
| **tests** | unit tests only: the logic is right, nothing was timed |
| **not measured** | no number anywhere in the repo |

Who: the person who ran or recorded it (`docs/features.md`, the git log, the page named).

## The layers, in one picture

```text
 a person      joystick (100), RC (95), keyboard (90) ── always outrank every program
     │
 hardware      Kar-Tech wireless stop, red mushroom button ── cut power, outside all software
     │
 firmware      ESP32 / spine: 100 ms command timeout, e-stop flag, overspeed, driver faults
     │
 ROS locks     twist_mux: e-stop (255), no-timeout (60), geofence (51), stop sign and
               sensor watchdog (30), stop hold (21) ── a lock holds every slot below it
     │
 the planner   the stop zone (aav_stop_zone), the costmap, the behaviour planner
```

The locks are in `ros/command_mux/aav_command_mux/config/twist_mux.yaml`.

## All hazards at a glance

| # | Hazard | What stops it | Best number | Where | On the car? |
|---|---|---|---|---|---|
| 1 | A person or obstacle ahead | stop zone + stop hold | stopped 0.16-0.55 m before a post | sim | ❌ |
| 2 | An obstacle behind, reversing | the rear stop zone | stopped 0.75-0.85 m before a post | sim | ❌ |
| 3 | Runaway speed | ego's cap; the spine's overspeed rule; the watchdog's OVERSPEED | **14 % over** the command; throttle stuck: OVERSPEED 4.6 s after | sim | 🟡 |
| 4 | The e-stop | `/safety/estop`; Kar-Tech; red button | brake within 2-12 ms (spine) | sim | ❌ |
| 5 | A dead or frozen node | sensor watchdog; mux timeouts; respawn | 15 faults stop within 6 s (the lidar's since 19 Nov) | sim | ❌ |
| 6 | Commands to the spine lost | the firmware's 100 ms timeout | brakes 111-120 ms after | QEMU | ❌ |
| 7 | Steering stuck or off centre | driver-fault brake; calibration | stuck in a bend: the watchdog's STUCK after 3.6 s, stood after 5 s (was: nothing noticed, 3.1 m off) | sim | 🟡 |
| 8 | A weak brake | the adaptive brake (a trial) | 0.87 → 0.21 m past, 150 kg; a dead brake: 9.7 m | sim | ❌ |
| 9 | Leaving the area | the geofence | stopped 1.06 m after the lock | bench | ❌ |
| 10 | A GPS jump, bad localization | watchdog; geofence LOST, JUMP | stops within 6 s | sim | ❌ |
| 11 | A stop sign missed | RF-DETR + the stop-sign lock | missed 1 in 8 runs | sim | ❌ |
| 12 | The screen freezes | the HUD's heartbeat | STALE in 1 s | a Mac | ❌ |
| 13 | A kerb | the step finder (`kerbs:=true`), kept 10 s; the costmap; the stop zone | a 0.3 m kerb seen 8.4 m out, stopped 0.80-0.82 m before it | sim | ❌ |
| 14 | Something moving near | the behaviour's hold (WAIT, moving), ego, the stop zone | held for an oncoming car, a person on the roadside, cross traffic and a person crossing, every run; the parked box passed without a hold | sim | ❌ |

## 1. A person or obstacle ahead

| Layer | What | Where |
|---|---|---|
| Stop zone | `aav_stop_zone` zeroes `/drivers/nav/cmd_vel` when the radar or lidar sees 3 points in front of the bumper, or a sensor goes silent. Nav2's `collision_monitor` until 17 Nov; ours took the lock after its shadow run ([the swap](#the-stop-zones-swap-17-nov)) | `aav_stop_zone` |
| Stop hold | holds the stop until the zone stays clear 1 s, so it can't flicker (twist_mux 21) | `aav_safety` `stop_hold.py` |
| Costmap | the follower's own collision check can stop first | `aav_costmap`, `aav_mppi_lag` |
| Behaviour planner | slows before the stop zone (a trial, off) | `aav_behaviour_rs` |

| Number | Where, when | Who |
|---|---|---|
| radar's box: 0.23 m before it | sim (Gazebo), 3 Nov | Ben |
| lidar alone, a post: 0.16-0.55 m before it; 0.16 m | sim (Gazebo), 15 Oct; 3 Nov | Ben |
| a busy Mac: **0.13 m past** the post | sim (Gazebo), 20 Oct | Ben |
| a ghost person: 0.21 m; 0.51 m before it | sim (Gazebo), 16 Oct; 3 Nov | Ben |
| radar: 0.3-0.4 m; lidar: 0.88 m before a box | bench, Oct; 20 Oct | Ben |
| the cubes trial: **1 of 3 drove 1.30 m into the post** | sim (Gazebo), 3 Nov | Ben |
| the cubes trial again: 0.20 and 0.19 m before the post, 2 of 2 | sim (ours), 15 Nov, TODO 93 | Ben |
| a 0.3 m kerb across the road (`--kerb`): **not seen**, the car drove into it, lidar alone and with the radar | sim (ours), 15 Nov, TODO 119 | Ben |
| the same kerb with the step finder (`kerbs:=true`): stopped 0.80-0.82 m before it ([13](#13-a-kerb)) | sim (ours), 16 Nov, TODO 119 | Ben |
| the car | not measured | |

**Gaps:** the stop zone and the stop hold hold only navigation and the AI slot. The point driver
(25), scripts and replay are not stopped by an obstacle. The roof lidar can't see a 1 m
box inside the stop zone: the radar's job ([perception](perception.md)). The flat scan
sees no kerb 0.3 m high or lower; the step finder does, with `kerbs:=true` ([13](#13-a-kerb)).
The ghost test on
the car is written down but not run ([ghost obstacles](ghost-obstacles.md#on-the-real-car)).

### The stop zone's swap (17 Nov)

Ours (`aav_stop_zone`) ran in shadow beside Nav2's `collision_monitor` on every obstacle
check, 3 rounds: **Nav2 stopped 24 times; ours stopped on the same command all 24, never
later** (each node's own log of its decision). Ours also stopped alone, up to 3.7 s, at
start-up: only extra caution. So ours took the lock, and Nav2 went
([the table](history/nav2-as-it-was.md#the-swap-of-the-stop-zone)).

## 2. An obstacle behind, while reversing

| Layer | What | Where |
|---|---|---|
| Rear stop zone | the stop zone's box behind the rear bumper, from the lidar | `aav_stop_zone`, `reversing:=true` |

| Number | Where, when | Who |
|---|---|---|
| stopped 0.75 m short of a post | sim (Gazebo), 22 Oct | Ben |
| backing to a goal 5 m behind: ego never asked past -0.5 m/s | sim (Gazebo), 15 Oct | Ben |
| 0.78 m | sim (Gazebo), 3 Nov | Ben |
| 0.85 m, all our own nodes | sim (ours), 14 Nov | Ben |
| the car | not measured | |

**Gap:** on the car the wheels can't tell direction yet (H5): the speed's sign is the last
command's.

## 3. Runaway speed (faster than asked)

| Layer | What | Where |
|---|---|---|
| ego | one top speed for ego and the spine: 1.0 m/s (`max_speed:=0.3` on the Pi alone) | `aav_ego`, `spine-core` |
| Spine overspeed | over 1.25 m/s: throttle 0, brake 0.5; still over after 2 s: full brake, latched | `spine-core` (Arsh's stop rules) |
| ESP32 throttle cap | DAC at most 700 of 1023; open-loop cap 1.0 / 3.0 (3.0 a guess) | `esp32-umod` |
| Actuator tests | stop on a speed rising while the command falls (`runaway`) | `aav_actuator_tests` |
| Watchdog (OVERSPEED) | the wheels over ego's speed by 0.2 m/s while not slowing (under 0.3 m/s²): OVER (a warning) after 1 s, OVERSPEED after 2 s, latched, holds the programs (TODO 207). Only fresh numbers count: a wheel speed older than the input's `max_age` (1 s), or a command after a gap longer than that, is no check; quiet wheels are odometry's own fault, STALE (16 Nov) | `aav_safety` `watchdog.py`, input `speed`; the sim only |

| Number | Where, when | Who |
|---|---|---|
| **1.145 m/s against 1.0 asked: 14 % over** (today's speed loop) | sim (ours), 14 Nov, TODO 163 | Ben |
| 9-14 % over at 1 m/s, the spine in the loop | sim (Gazebo), 29 Oct ([controllers](history/controllers-log.md#2a-lqi-against-the-pi-results-29-oct)) | Ben |
| `smooth` loop: 1.00 m/s, no overshoot | sim (ours), 14 Nov | Ben |
| a 25 m drive to a point: `default` 1.14 m/s, 7 s over the road ahead's limit; `firm` 1.00 m/s, 0 s over (but more sway on the lot tour: a challenger, TODO 163) | sim (ours), 16 Nov, 2 runs each | Ben |
| the overspeed rule | tests only (`spine-core`) | Arsh |
| the throttle stuck at its cruise level, then the e-stop from 0.9 m/s: 1.48-1.50 m (healthy 1.18-1.20), stood in 2.2 s: the brake beats it | sim (ours), 15 Nov, 3 runs | Ben |
| the throttle stuck while speeding up (at 0.4 m/s, `--actuator-faults`): the car held 1.37 m/s for 1.00 asked; the watchdog said OVER 3.6 s after, OVERSPEED 4.6 s after | sim (ours), 16 Nov, 2 runs | Ben |
| OVERSPEED never in a healthy drive or stop: every loop profile on the car model; the healthy, weak-brake and stuck-throttle e-stops in the sim | the car model (tests); sim (ours), 16 Nov, 3 runs | Ben |
| **Fixed 16 Nov:** with the wheels quiet (`--faults`' odom stop), the watchdog kept their last 0.5 m/s against ego's 0 and latched OVERSPEED, so the ten fault cases after it found the lock already on. Now a stale wheel speed is no check: `sim-check --faults` passed all 14 fault cases and `--actuator-faults` all 6 (the Python watchdog, first run); a test in each language; the shared vectors replay it. In 4 runs since (Python and Rust watchdogs), OVERSPEED never latched after the odom stop, but 1-4 cases a run failed "Nav2 would not drive before the fault": the global EKF jumps 9 m as the wheels come back and the path follower loses its path (a navigation flake, TODO 207) | sim (ours), 16 Nov | Ben |
| stuck wide open | not measured | |
| the car | not measured | |

**Gap:** the speed rules are **off on the car**: the ESP32 gets no wheel speed yet, so it
has no overspeed check ([ESP32 safety](esp32-safety.md)). The stopping law assumes the car
goes no faster than told (TODO 163).

## 4. The e-stop (software and hardware)

| Layer | What | Where |
|---|---|---|
| Software | the joystick's button, `pixi run estop`, the HUD: `/safety/estop`. twist_mux locks every driver (255); ego and the bridge brake on it directly | `aav_command_mux`, `aav_ego` |
| Firmware | the e-stop flag: full brake on the next frame, latched | `spine-core`, `esp32-umod` |
| Hardware | the Kar-Tech wireless stop and the red mushroom button cut power | the car's wiring |

| Number | Where, when | Who |
|---|---|---|
| twist_mux path, no spine: brake in 0.46 s | sim (Gazebo), Oct ([simulation](simulation.md)) | Ben |
| through the spine: brake in 1-12 ms; 2 ms | sim (Gazebo), 15 Oct; 3 Nov | Ben |
| a stand-in hardware e-stop: brake within 10 ms | sim (Gazebo), Oct | Ben |
| the e-stop stopped the point driver | sim (Gazebo), 15 Oct | Ben |
| what the Kar-Tech and the red button cut | **not measured** (S1, S2) | |

**Gap:** nobody has written down what the Kar-Tech or the red button cuts, or what the
brake does with no power ([ESP32 safety](esp32-safety.md), row 7; tests 3.5, 3.8).

## 5. A dead or frozen node

| Layer | What | Where |
|---|---|---|
| Sensor watchdog | a sensor stale, NaN, frozen or jumped: holds the point driver, navigation and the AI slot (twist_mux 30). It sends 10 times a second: 1 s of its own silence holds them too. The roof lidar too since 19 Nov (TODO 246): STALE after 1 s, FROZEN while moving | `aav_safety` `sensor_watchdog.py` |
| Wheels cross-check | frozen wheels caught against the IMU and GPS | `aav_safety` `wheel_check.py` |
| twist_mux | every driver counts only for 0.5 s after its last message | `twist_mux.yaml` |
| ego | stops on 0.3 s of quiet | `aav_ego` |
| Joystick alive | `SILENT` after 0.5 s, `FROZEN` after 2 s of the same values | `aav_safety` `joy_alive.py` |
| Respawn | our navigation nodes restart 1 s after they die (14 Nov) | `navigation.launch.py`, `test_respawn.py` |
| Age checks | `mppi_lag` and our planner refuse a stale costmap; our costmap stops on stale odometry; a `/tf` over 1 s old is no pose (14 Nov) | `aav_mppi_lag`, `aav_planner`, `aav_costmap`, `aav_navigator` |

| Number | Where, when | Who |
|---|---|---|
| 13 faults on purpose, each stops the car within 6 s | sim (Gazebo), `sim-check --faults` | Ben |
| frozen wheels: 25 of 25 caught | sim (Gazebo), since 24 Oct | Ben |
| `--faults` 3 of 3 | sim (ours), 8 Nov | Ben |
| **Fixed 17 Nov (TODO 207):** 1-6 fault cases a run failed after `odom stop`: with the wheels quiet, both EKFs kept their last speed and acceleration, `ekf_local` ran 15 m on while the car stood, and `map -> odom` jumped 9-10 m back when the wheels returned. Now the car's command may slow each EKF's speed (robot_localization's control term, never speeding it up). And the watchdog waits 10 s, not 3, after its own stall before anything latches (DDS took 7 s to bring the HUD's beats back, and the HUD latched LOST). `sim-check --faults` **14 of 14, 3 runs in a row** (the Rust watchdog, a computer at load 10-34) | sim (ours), 17 Nov | Ben |
| **The lidar (TODO 246):** car-test's step 5e found no lidar input, so a dead lidar stopped only navigation (the stop zone's silent source), and said nothing. Now the watchdog's `lidar` input stops, as the camera. `sim-check --faults` with the lidar: **15 of 15**; `lidar stop`: "lidar STALE" 1.2 s after its last cloud, stood still 2.3 s after the fault, 1.3 m on | sim (ours), 19 Nov | Ben |
| a 2.5 s stall of the whole Mac flagged, then let go | bench, Oct | Ben |
| joystick alive: `bench-check` 11 of 11 | bench, 2 Nov | Ben |
| respawn and age checks | tests only | Ben |
| the car | not measured | |

**Gap:** device nodes have no respawn (N3). No watchdog reads our nodes' status yet (TODO 161).

## 6. Commands to the spine lost

| Layer | What | Where |
|---|---|---|
| Rust ESP32 | no fresh command (or a frozen counter) for 100 ms: full brake, throttle 0, latched | `spine-core`, `esp32-umod` (TODO 45) |
| Pico 2 spine | the same, plus a 100 ms hardware watchdog | `boards/pico2` |
| Today's C++ ESP32 | **no timeout**: the last throttle stays. So the no-timeout lock (twist_mux 60) holds every program; only people drive | `legacy_uart_bridge` |

| Number | Where, when | Who |
|---|---|---|
| brakes 111-120 ms after the last frame | QEMU, 24 Oct | Ben |
| 112-119 ms, 6 of 6 | QEMU, 3 Nov | Ben |
| counter frozen: 113-118 ms | QEMU | Ben |
| the spine in the loop: full brake 195-209 ms; 202-205 ms | sim (Gazebo), 15 Oct; 3 Nov | Ben |
| test 3.9, the serial cable pulled | **not measured**: fails with today's C++ | |

**Gap:** the Rust firmware has never been flashed. This is blocker 1 (TODO 158).

## 7. Steering stuck, or the wrong centre

| Layer | What | Where |
|---|---|---|
| Boot self-test | nothing arms until both Jrks and the DAC answer | `spine-core` (Arsh) |
| Driver fault | a Jrk error: full brake, latched | `esp32-umod`, `spine-core` |
| Calibration | the steering actuator's ends and centre, `steering_limits` | [actuator tests](actuator-tests.md#find-the-steering-actuators-ends-and-centre) |
| Watchdog (STUCK) | the steering asked against measured: over 0.15 rad apart for 1 s latches STUCK and holds the programs (TODO 207) | `aav_safety` `watchdog.py`, input `steering` |

| Number | Where, when | Who |
|---|---|---|
| the centre (test 3.12: under 1°) | **not measured**: the car pulls right (TODO 67) | |
| stuck in a bend (`--actuator-faults`): mppi_lag backed and turned 3 times, drove 19.3 m, up to 3.1 m off the straight to the goal, stood after 16 s; **nothing reported a fault**, the goal kept running | sim (ours), 15 Nov, 1 run | Ben |
| the same with the watchdog's STUCK (TODO 207): **noticed 3.6 s after**, the car stood 5 s after; drove 3.9 m, up to 1.3 m off | sim (ours), 15 Nov, 1 run | Ben |

**Gap:** on the car today the measured side is NaN (the 2025-26 uplink has no steering
feedback, Q2), so the check stays silent there until the steering's position is read;
its 0.15 rad and 1 s are the sim's, not tuned on the car.

## 8. A weak brake

| Layer | What | Where |
|---|---|---|
| Adaptive brake | learns how well the brake works and brakes harder, never past `safe_max` (a trial, off) | `aav_description` `adaptive_brake.py` |
| Geofence `COASTING` | holds every program until test 4.3 measures the stop | `aav_safety` `geofence.py` |
| Spine | the overspeed rule (section 3) | `spine-core` |
| Watchdog (OVERSPEED) | a car that doesn't slow when told: latched, holds the programs (section 3) | `aav_safety`, input `speed` |

| Number | Where, when | Who |
|---|---|---|
| 150 kg aboard: +0.58 m past → +0.04 m with it | the car model, 8 Nov | Ben |
| 150 kg: +0.87 m past → +0.21 m; the next stop +0.05 m | sim (ours), 8 Nov | Ben |
| the e-stop from 0.9 m/s (`--actuator-faults`): healthy 1.18-1.20 m; a weak brake (30 %) 2.05-2.08 m, 3.6 s; a dead brake 9.7 m, still rolling at 0.05 m/s after 30 s | sim (ours), 15 Nov, 3 runs | Ben |
| the dead brake is noticed: the watchdog's OVERSPEED (0.8 m/s for 0 asked, 2 s), 4 runs of 4 | sim (ours), 16 Nov | Ben |
| the brake's limits (`released`, `full`, `safe_max`) | **placeholders** until car day A7 | |

**Gap:** refused with today's ESP32. Today's car brakes only below `stop_speed`;
[the geofence page](geofence.md) says it rolls "about 10 m" from 1 m/s, with no record of a
measurement.

## 9. Leaving the area

| Layer | What | Where |
|---|---|---|
| Geofence | any stopping path within 1 m (+ the pose's spread) of the edge: holds every program (twist_mux 51); people still drive | `aav_safety` `geofence.py` |
| Point driver, drive server | refuse a goal outside the fence | `aav_drive_to_point`, `aav_language` |

| Number | Where, when | Who |
|---|---|---|
| lock at 1.08 m/s; stopped 1.06 m on (it allowed 1.56 m); inside a 12 m fence | bench, 29 Oct | Ben |
| in a simulator with a fence you can reach | not measured | |
| the car | not measured | |

**Gap:** no site polygon yet; the fence is a 30 m circle round the start spot.

## 10. A GPS jump, or bad localization

| Layer | What | Where |
|---|---|---|
| Sensor watchdog | GPS lost, NaN or jumped: holds the point driver, navigation and the AI slot | `aav_safety` |
| Geofence | `LOST` (no pose 1 s, spread over 3 m, no GPS into the EKF 2 s), `JUMP` (latched) | `aav_safety` `geofence.py` |
| Localization health | warns only: gyro bias, GPS quality, clock skew, runaway EKFs | `aav_localization_health_rs` (Betsi) |

| Number | Where, when | Who |
|---|---|---|
| GPS lost, NaN, jump: each stops the car within 6 s | sim (Gazebo), `sim-check --faults` | Ben |
| a GPS fault stopped the point driver | sim (Gazebo), 15 Oct | Ben |
| the health checks' thresholds | tuned in Gazebo only | Betsi |
| the car | not measured | |

## 11. A stop sign missed

| Layer | What | Where |
|---|---|---|
| Detector | RF-DETR on the front camera | `aav_detector` |
| Stop-sign lock | holds the point driver, navigation and the AI slot about 4.7 m before the sign (twist_mux 30); a watcher that dies while true keeps the car stopped | `stop_sign` watcher |
| A person | the safety driver | |

| Number | Where, when | Who |
|---|---|---|
| lock on at 20.7 m from the sign | bench, 20 Oct | Ben |
| bumper 4.4 m before the sign | sim (Gazebo), 24 Oct | Ben |
| stood 3.9-4.4 m before it, 3 of 3 | sim (ours), 8 Nov | Ben |
| **the lock never came on: 1 of 8 runs** | sim (ours), 8 Nov | Ben |
| the car | not measured: the Orin's TensorRT engine isn't built | |

**Gap:** a missed sign has no second layer but the person in the seat.

## 12. The screen freezes

| Layer | What | Where |
|---|---|---|
| Heartbeat | the HUD beats 10 Hz on `/hud/heartbeat` while it draws | `aav_hud` |
| Watchdog | quiet or stuck 1 s: holds the point driver, navigation and the AI slot; after the first 5 s, latched (`hud LOST`) | `aav_safety` `sensor_watchdog.py` |

| Number | Where, when | Who |
|---|---|---|
| beat at 9.3 Hz; frozen or closed: `hud STALE` in 1 s | a Mac, 23 Oct | Ben |
| on a Pi or the car's screen | not measured | |


## 13. A kerb

Ben, 15 Nov: "we don't want to hit a curb" (TODO 119; [kerbs](kerbs.md)).

| Layer | What | Where |
|---|---|---|
| The map | every road edge on the campus is a lethal kerb line | `campus.pgm` and its chunks |
| Step finder | a point 0.10-0.5 m above another within 0.3 m, kept 10 s in odom (the lidar loses a kerb 3-4 m before the bumper) | `aav_voxels_rs`, `kerbs:=true` |
| Costmap | the local costmap's `kerb_layer` marks it, and nothing clears it | `aav_costmap` |
| Stop zone | the stop zone's `steps` source (Nav2's `collision_monitor`'s until 17 Nov) | `aav_stop_zone` |

| Number | Where, when | Who |
|---|---|---|
| a 0.3 m kerb across the road, the flat scan only: **drove into it** | sim (ours), 15 Nov | Ben |
| the step finder, no memory: 0.12 m before it (the controller); with the radar **0.24 m into it** | sim (ours), 16 Nov | Ben |
| the step finder with its memory: seen 8.4 m past the bumper; the stop zone stopped the car 0.82 m (lidar) and 0.80 m (lidar and radar) before it, 1 run each | sim (ours), 16 Nov | Ben |
| cast clouds: a 0.3 m kerb found from 8.6 m, a 0.15 m one from 4.7 m, a 0.10 m one never; no steps on slopes or ramps to 10°, camber, a 7.5 cm bump, grass | tests | Ben |
| the car | not measured | |

**Gaps:** a 0.15 m kerb is found only in the last 4.7 m: enough at 1 m/s, not at 3 m/s.
A 0.10 m kerb is never found. The memory forgets after 10 s: a car stood longer before a
kerb it saw only from far forgets it. Not tried on the real VLP-16.

## 14. Something moving near the car

Ben, 16 Nov: "if we see a car people or stuff moving around us we should likely just
stop." ([stopping](stopping.md#anything-moving-near-stop-and-wait-17-nov))

| Layer | What | Where |
|---|---|---|
| The things | the lidar's cubes grouped and followed: each one's place and speed | `aav_things_rs`, `things:=true` (on in `stack:=ours`) |
| Behaviour planner | anything moving 0.3 m/s or more within 10 m, or onto the road ahead within 3 s: WAIT (moving), the road ahead ends at the car; GO 1 s after it has gone or stood still | `aav_behaviour_rs` (`aav_behaviour_rs`) |
| Ego | holds the automatic drivers to the road ahead's limit: 0 at its end | `aav_ego_rs` |
| Stop zone | still stops the car for anything in front of the bumper, moving or not | `aav_navigation` |

| Number | Where, when | Who |
|---|---|---|
| `sim-check --movers`, 2 runs each: held every time. Oncoming car 11.6 s, 2.4-3.5 m clear; a person on the roadside 20.3 s, 6.6-6.8 m clear; cross traffic 2.0 s, 19 m clear | sim (ours), 17 Nov | Ben |
| `sim-check --moving` (a person crossing): APPROACH, then held 17.7 s, 1.9 m clear | sim (ours), 17 Nov | Ben |
| `sim-check --behaviour-things` (a parked box): passed, never held, once the hold waits for 1.5 s of moving, not growing (it held 8 times before) | sim (ours), 17 Nov | Ben |
| the car | not measured | |

**Gaps:** a hold needs the things, which need the lidar: the radar alone holds nothing.
Someone walking beside the car the same way, within 10 m, holds it for as long as they
walk. The things' speeds come from the lidar's cubes: a still thing whose shape changes as
the car passes can look like it moves.

## The biggest gaps

| Gap | Hazards | TODO |
|---|---|---|
| **Nothing measured on the car** | all | 157, 158 |
| Today's ESP32 has no command timeout; the Rust one is unflashed | 6 | 45, 158 |
| What the Kar-Tech and the red button cut | 4 | S1, S2 |
| The steering's centre, and the brake's limits | 7, 8 | 67, A7 |
| A steering actuator that doesn't follow is noticed in the sim only (no feedback on the car yet); nothing but the e-stop and the drag stops a dead brake (the watchdog's OVERSPEED only notices it, in the sim) | 7, 8 | 159, 207 |
| The car runs 14 % over its command in the sim | 3 | 163 |
| Obstacles stop only navigation and the AI slot | 1 | |

## Add a number

1. Run the test; write down where (car, bench, sim), the date, and who.
2. Put it in the hazard's table, newest last.
3. A car number replaces nothing: keep the sim's beside it.
