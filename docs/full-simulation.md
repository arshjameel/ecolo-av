# The full simulation: the whole car in Gazebo

Written 15 October 2026, checked 25 October. The goal (Ben, 15 Oct): "fully simulate the car inside
Gazebo with all the sensors, and build the full pipeline." This page says what we have,
what's missing, and how the car's measurements feed in.

> **The short version**
> - **Most of it exists.** Gazebo already plays the car: IMU, GPS, wheel speed, three cameras, the radar, the lidar. The same EKFs, Nav2, perception and HUD run on it.
> - **The big gap is the actuators.** Gazebo's car sets its speed directly, with made-up limits. It ignores the weight, the motor and the brakes. So it doesn't yet speed up, brake or steer like the real car.
> - **The fix:** an **actuator model** between our commands and Gazebo, filled with the numbers from the tests (`tests.md`, stage 4). Then the real firmware logic **in the loop**, as the bench already does. Both built 15 Oct: the model is on by default, the spine with `spine:=true`.
> - **Measurements come first:** the sheet (`measure-the-car.md`) for where everything is, the tests for how it moves.

## Status (15 Oct)

| # | Step | Status |
|---|---|---|
| 1 | `/system/source` + the SIMULATED banner | ✅ built: Gazebo, turtlesim, the bench, the car's backends and replay announce; the raylib HUD shows the banner and a tag per topic |
| 2 | `/vehicle/steering`, `/throttle`, `/brake`, `/wheels` | ✅ built: Gazebo all four; the bench and the car (spine_bridge) steering, throttle, brake; turtlesim steering |
| 3 | The actuator model (A), guessed numbers | ✅ built: `aav_gazebo/actuator_model.py`, numbers in `vehicle.yaml` `actuators:`, on by default (`actuator_model:=false` for the old car) |
| 4 | The spine in the loop | ✅ 15 Oct: `pixi run sim spine:=true`. ego's commands go over the CAN frames to the spine's own firmware logic, then the actuator model. `pixi run sim-check --spine` passes: its timeout, both e-stops and arming stop the car ([below](#the-spine-in-the-loop-step-4)) |
| 4b | Today's car in the loop | ✅ 29 Oct: `pixi run sim esp32:=true`. ego's commands go through the car's own UART bridge and the 2025-26 C++ ESP32 firmware's logic (its throttle ramp and brake), then the actuator model. `pixi run sim-check --drive-to-point-only --esp32 pi` passes ([below](#todays-car-in-the-loop-step-4b)) |
| 5 | The lidar into Nav2 | ✅ 15 Oct: `lidar_scan` (`aav_lidar`) flattens the roof lidar into a 2D scan; the local costmap's `lidar_layer` and `collision_monitor` read it. `lidar:=true` in Gazebo, and on the car (the stock `velodyne` driver, since 20 Oct). With the radar off, the lidar alone stopped the car 0.16-0.55 m before a post |
| 6 | Sensor failures on purpose | ✅ 15 Oct: `pixi run sim-fault <sensor> <fault>`, and the sensor watchdog (`aav_safety`): it stops Nav2 and the AI slot, and says why on the HUD and the map window. All 13 faults stopped the car on 15 Oct, the watchdog's own death and frozen wheels too; on 24 Oct, with the HUD's heartbeat added, all 25 checks passed three times ([the table](simulation.md#what-the-car-does-before-and-after)) |
| 7 | The actuator test script (`tests.md` 1.10) | ✅ 15 Oct: `pixi run actuator-test <test> --sim`, all 9 tests in Gazebo; they give back the model's numbers ([docs/actuator-tests.md](actuator-tests.md)). It found Gazebo's IMU blind to the car's acceleration: fixed in `gazebo_adapter` |
| 8-10 | the real numbers, the test lot, forces | not started |

Tested (15 Oct, Ben's Mac):

| Run | Result |
|---|---|
| `pixi run sim-check --spine` (step 4) | ✅ all 21: the direct path, then the spine in the loop: graph vs the spine contract, drive 6.00 m, the EKF 0.03 m from the truth, a Nav2 goal, the command stream lost, both e-stops, not armed ([stop times](#stop-times-the-direct-path-vs-the-spine)) |
| `pixi run sim-check` (the model on) | ✅ all 6: graph vs contract (the new topics too), drive 6.01 m of 6.5 |
| `pixi run sim-check --obstacle --lane` | ✅ all: stood still 0.51 m before the box; lane keeping back within 5 cm in 5.4 s |
| `pixi run sim-check --obstacle --lidar-obstacle` (steps 5, 6 branch; twice) | ✅ the box: 0.32-0.46 m before it (radar and lidar); the post, radar off: 0.16-0.55 m before it, the costmap 100 of 100 there; both goals reached once they were gone |
| `pixi run sim-check --faults` | ✅ all 10 faults: the car stood still 1.7-4.1 s after each, the watchdog (or `collision_monitor`, for the radar) said why, and let go once mended |
| `pixi run sim-check --faults`, with the 3 new faults (later on 15 Oct) | ✅ all 13, twice: frozen wheels caught 3.4-4.8 s after they went wrong; the watchdog hung or killed: stood still 2.6-2.8 s after, through twist_mux's 1 s lock timeout; the global EKF 0.1-0.4 m from the truth through a 20 m GPS jump |
| `pixi run bench-check` | ✅ all 6, graph vs contract with the bench's new topics |
| turtle profile vs contract | ✅ `/system/source` and `/vehicle/steering` from `turtlesim_adapter` |
| Gazebo with `actuator_model:=false` | ✅ the old car: no throttle or brake (NaN), an instant steer |
| The HUD's banner | ✅ its logic (`cargo test`); ❌ not seen on screen: the window couldn't open from that session |

What's where:

| Thing | Where |
|---|---|
| The messages | `aav_interfaces`: `DataSource` + `SensorSource` (REAL, SIM, BENCH, REPLAY per sensor), `ActuatorState` (command + measured), `WheelSpeeds` |
| The contract | `docs/interfaces.yaml`; `/system/` is a new topic area (`docs/naming.md`) |
| The model and its tests | `ros/sim/aav_gazebo/aav_gazebo/actuator_model.py`, `test/test_actuator_model.py` |
| How it's documented | `docs/simulation.md`, "The actuator model"; `docs/hud.md` for the banner |

## 1. The full pipeline, in one picture

```text
  drivers (keyboard, joystick, Nav2, plain English)
        │
  twist_mux → ego ──────────── /ego/actuator_cmd (speed + steering)
        │
  ┌─────┴─────── spine:=false (default) ─────────────┐   ┌────── spine:=true (15 Oct) ──────┐
  │ gazebo_adapter → a Python copy of the spine's    │   │ spine_bridge → CAN frames        │
  │ speed loop → ACTUATOR MODEL: throttle, brake,    │   │   → the spine's real logic       │
  │ steering, as measured → Gazebo                   │   │     (timeouts, e-stop, limits)   │
  │ (no CAN, no arming, no e-stop latch; brakes      │   │   → ACTUATOR MODEL: throttle,    │
  │ 0.2 s after ego goes quiet)                      │   │     brake, steering, as measured │
  │                                                  │   │   → Gazebo                       │
  └──────────────────────────────────────────────────┘   └──────────────────────────────────┘
        │
  Gazebo's sensors → /sensors/... → EKFs, Nav2, perception, HUD   (the same as the car)
```

## 2. What we have, and what's missing

| Part | Today in Gazebo | Missing for the full version |
|---|---|---|
| IMU, GPS, wheel speed | ✅ with noise | noise levels from the real devices (the car day) |
| 3 cameras | ✅ 15 Oct: two front, one rear | the real lens of each (N4); perception reads only the front left |
| Radar (grill) | ✅ a stand-in, shaped like the TI radar | its real config file, from the car |
| Lidar (roof) | ✅ a generic 16-beam one, on by default (15 Oct); ✅ Nav2's costmap and the collision monitor read it (step 5) | the VLP-16's exact beam pattern; the car's driver is untested with the real lidar |
| **Throttle and brake** | ✅ 15 Oct: the actuator model, with **guessed** numbers | the real numbers, from tests 4.1-4.4 |
| **Steering** | ✅ 15 Oct: speed, lag, centre offset, deadband, with **guessed** numbers | the real numbers and angles, from tests 3.10-3.12 and 4.5 |
| Weight | in the model, but hardly matters (see the box below) | the real weight (scales, M7) |
| The firmware's safety logic | ✅ 15 Oct, with `spine:=true`: the CAN frames, the spine's timeout, both e-stops, arming ([step 4](#the-spine-in-the-loop-step-4)). Off by default | the real CAN bus and boards: that's the car |
| What's real and what's fake | ✅ 15 Oct: `/system/source`, and the HUD's orange SIMULATED banner | - |
| Per-actuator readings | ✅ 15 Oct: `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`, `/vehicle/wheels` (and still `/vehicle/status`) | the real measured values from the car |
| The world | a test road, an empty lot | **a copy of the real test lot** |
| Sensor failures | ✅ 15 Oct: `pixi run sim-fault`, and the sensor watchdog stops Nav2 (step 6) | the same on the car, on stands: pull a cable |

!!! note "Why the weight hardly matters in Gazebo today"
    Gazebo's Ackermann system **sets the wheels' speed** (`aav.gazebo.xacro` allows
    ±10 m/s², so the actuator model sets the real limits). A heavier car gets the same numbers. For the
    weight to matter, the model must push with a **force** (from the motor) against the
    **mass**. For driving well, what matters most is the **measured g**: how fast the real
    car speeds up and stops. The tests give that directly.

## 3. Two ways to model the actuators

| | A. An actuator model in front of Gazebo (recommended first) | B. Forces inside Gazebo |
|---|---|---|
| How | a small node: throttle % → acceleration, brake % → deceleration, steering with its speed and lag; then Gazebo's speed control as today | the motor's torque on the rear wheels, the brakes' torque, tyre grip, the mass |
| Weight matters? | through the measured g (which includes it) | directly |
| Effort | about a day | about a week, and tricky to tune |
| Good for | Nav2, the smooth stop, the actuator tests: everything we drive | slopes, skids, a heavy load |

**Recommendation:** A first, with its numbers in `vehicle.yaml` (a new `actuators:` section).
B only if A isn't close enough to the real car.

## 4. How the measurements feed the simulation

| From | Measured | Goes into | Changes in Gazebo |
|---|---|---|---|
| The sheet (`measure-the-car.md`) | wheelbase, track, wheels, body | `vehicle.yaml` → the model | how it turns; its outline for Nav2 |
| The sheet | each sensor's x, y, z and angles | `vehicle.yaml` → the sensor frames | what each camera, the radar and the lidar see |
| Scales (M7) | the weight, per wheel | `vehicle.yaml` | the model's mass (and B, later) |
| Tests 4.1, 4.4 | acceleration per throttle % | `vehicle.yaml` → the actuator model | how fast it speeds up |
| Test 4.2 | coasting | the actuator model | how it slows with no brake |
| Test 4.3 | braking g | the actuator model; the smooth stop | how fast it stops |
| Tests 3.10-3.12, 4.5 | steering: real angles, speed, centre | the actuator model | how it steers |

```text
 measure the car  →  vehicle.yaml  →  Gazebo behaves like the car  →  test in Gazebo  →  trust it on the car
```

## 5. The order to build it

| # | Build | Needs the car? | Size |
|---|---|---|---|
| 1 | `/system/source` + the SIMULATED banner ✅ 15 Oct | no | small |
| 2 | Per-actuator readings: `/vehicle/steering`, `/throttle`, `/brake`, `/wheels` ✅ 15 Oct | no | small |
| 3 | **The actuator model (A)**, with guessed numbers for now, in `vehicle.yaml` ✅ 15 Oct | no; the numbers later | 1 day |
| 4 | **The spine in the loop**: ego → CAN → the spine's logic → the actuator model → Gazebo ✅ 15 Oct | no | 1-2 days |
| 5 | The lidar into Nav2's costmap and the collision monitor ✅ 15 Oct | no | small |
| 6 | Sensor failures on purpose, and checks that the car stops ✅ 15 Oct | no | 1 day |
| 7 | The actuator test script (`tests.md` 1.10), run in Gazebo ✅ 15 Oct | no | 1 day |
| 8 | Fill in the real numbers: measurements and tests (`pixi run actuator-test`, on stands then in the lot) | **yes**: on stands, then the lot | - |
| 9 | A Gazebo copy of the test lot | the lot chosen | 1 day |
| 10 | B, forces in Gazebo, only if needed | the real numbers | a week |

Steps 1-7 can all be built before the car is measured: they start with today's guesses,
and step 8 swaps in the real numbers without changing any code.

## The spine in the loop (step 4)

Built 15 Oct. Gazebo can now run the spine's **real firmware logic** between ego and the
car, as the bench does. Off by default; one switch turns it on:

```bash
pixi run sim spine:=true          # Gazebo, with the spine in the loop
pixi run sim-check --spine        # proves it: the normal checks, then the spine's stops
```

```text
 ego ──► output_mux ──/control/gazebo/actuator_cmd──► spine_bridge ──CAN frames──► spine_sim: the spine's own logic
                                                         (arming, 100 ms watchdog, both
                                                          e-stops, its speed loop)
                                                              │ throttle, brake, gear, steering
                                                              ▼
                                gazebo_adapter: the actuator model ──► Gazebo's car
                                                              │ Gazebo's wheels and steering
                                                              ▼
 /vehicle/* ◄── spine_bridge ◄──CAN frames── spine_sim: VEHICLE_FEEDBACK, STATUS, wheel frames
```

| What | How |
|---|---|
| One backend | `spine_bridge` alone reads ego's commands (`/control/gazebo/actuator_cmd`, from the output mux). `gazebo_adapter` then only drives the actuator model from the spine's outputs, and no longer publishes `/vehicle/*` |
| `/system/source` | still **SIM**: `spine_bridge` says so (`source:=sim`) |
| The clock | `spine_sim` ticks every 10 ms of Gazebo's time, so the spine's 100 ms watchdog counts Gazebo's time |
| Arming | `arm:=true` (the default) arms it 12 s after start-up, as the bench does, once the output mux says `OPEN` (`arm_when_open` asks each second). After a stop: `ros2 service call /spine/arm std_srvs/srv/SetBool "{data: true}"`. Refused while the safety gate is shut |
| Hardware e-stop | the bench's: `/vehicle/bench/hw_estop` (true = pressed) |
| The contract | profile `spine` in `docs/interfaces.yaml` (`pixi run check spine`) |

### Stop times: the direct path vs the spine

Each while driving at 0.8 m/s, Gazebo's road world, Ben's Mac, 15 Oct: three runs of
`pixi run sim-check --spine`, the last one all green. Times are Gazebo's.

| Stop | Direct path (`gazebo_adapter`) | Spine in the loop |
|---|---|---|
| Command stream lost (ego's process stopped) | brake 200-220 ms, still 1.24-1.25 s | **full** brake 195-209 ms, still 0.64-0.66 s, and **latched** until armed again |
| `/safety/estop` | brake 460 ms (through twist_mux and ego), still 1.70 s | brake 1-12 ms (ESTOP_CMD straight to the spine), still 0.18-0.48 s, latched |
| Hardware e-stop | (no such thing) | brake 2-10 ms, still 0.46-0.48 s, latched after it's let go |
| Not armed | (always armed) | a drive asking 1 m/s for 8 s: no throttle, no motion |

- **Why the spine stops sooner:** it brakes **fully** (`stop_brake` 1.0); the direct path's
  speed loop only slows to a stop.
- **Its 195-209 ms** is `spine_bridge`'s 0.1 s (`spine.yaml` `cmd_timeout`) plus the
  spine's own 0.1 s watchdog: inside the car's 200 ms target (`tests.md` 3.9).
- **Latched:** after any stop, the spine holds the brake, its drive still asking, until
  someone arms it again. The direct path drives on as soon as commands come back.
- **Found:** in two of the three runs, the spine latched once during the first drive: ego's 50 Hz
  commands stalled for more than 0.1 s (a busy Mac, just after the graph check). That's
  `spine_bridge`'s `cmd_timeout` doing its job, but it is tight for a Python ego on a
  loaded computer. `sim-check` arms it and drives again, as `bench-check` does. Worth
  watching on the car's computer.

## Today's car in the loop (step 4b)

Built 29 Oct. Today's car doesn't run the spine: the Motor Pi's `legacy_uart_bridge`
sends 13-byte frames to the 2025-26 C++ ESP32 firmware. Gazebo can now run that path:

```bash
pixi run sim esp32:=true                                # the bridge's speed loop on (pi)
pixi run sim esp32:=true esp32_loop:=open               # the 2025-26 mapping, to compare
pixi run sim-check --drive-to-point-only --esp32 pi     # proves it
```

```text
 ego ──► output_mux ──► legacy_uart_bridge ──13-byte frames, local TCP──► gazebo_esp32
          (the car's own; speed_loop.mode pi or open)      (the C++ firmware's logic:
                ▲                                           its throttle ramp, its brake)
                │                                                   │
          /vehicle/odom ◄── gazebo_adapter (the actuator model) ◄───┘
```

| Part | What stands in for the car | Faithful to |
|---|---|---|
| `legacy_uart_bridge` | nothing: the car's own node, its port a TCP socket | the car |
| `gazebo_esp32` (`esp32_firmware.py`) | the ESP32 | `ros2_handler.cpp`, `dac.cpp`: throttle ramp 6 up, 12 down per frame; brake = \|accel\| x 3500 from 400; no command timeout |
| `gazebo_adapter` | the motor, brake, steering and wheels | the actuator model (`vehicle.yaml`, guesses) |

### Results, 29 Oct (drive to point, Gazebo's truth)

| Mode | 1 m/s (5 drives) | 3 m/s (4 drives) |
|---|---|---|
| `pi`: feed-forward + PI on the wheel speed | all stopped 0.03-0.17 m from B; braking at most 1.43 m/s² | all 0.12-0.22 m from B; at most 1.62 m/s² (the check allows 1.875) |
| `open`: the 2025-26 mapping | every drive failed: the car overshot its speed (1.51 m/s for 1) and drive-to-point stopped it, 4-55 m short | the same |

- **Why `open` fails here:** the model's throttle means "push this hard"; the old mapping
  treats it as "go this fast". Which one the real motor controller does is actuator test 4.1.
  `pi` measures the speed, so it works either way.
- **A reference model:** the first `pi` run braked at 2.02 m/s² once (over the band). The
  brake takes 0.2 s to bite, so the car falls behind ego, and the PI braked harder to catch
  up. Now the PI compares the car with ego's speed delayed by 0.3 s (`response_lag`), and
  the stop meets its plan.
- **Without Gazebo:** `aav_gazebo/test/test_legacy_chain.py` runs the same chain (loop,
  frames, firmware, actuator model) in a unit test.

## Sensor failures: before and after

Each fault while Nav2 drives, with the watchdog and without it: the whole table, with
the 24 Oct results, is in [Simulation](simulation.md#what-the-car-does-before-and-after).
How to make a fault, and what the watchdog watches:
[Simulation](simulation.md#sensor-failures-on-purpose).

**Later on 15 Oct** ([docs/simulation.md](simulation.md#if-the-watchdog-itself-dies)):

| Gap | Now |
|---|---|
| A watchdog that dies while all is well stopped nothing | ✅ fail-safe: twist_mux holds Nav2 after 1 s of its silence |
| Frozen wheel odometry wasn't caught | ✅ the wheels against the IMU and the GPS: DISAGREE |
| The EKF took a 20 m GPS jump | ✅ partly: `ekf_global` refuses it while the watchdog holds the car (0.1-0.5 m, was 19 m); a car that drives on takes it after a few seconds |
| The map window didn't show the verdict | ✅ the HUD's banner; not seen on a screen yet |

**What's left:** the cross check's limits on the car (Gazebo's numbers today), and the
same tests on the car, on stands.

## Faster drives in Gazebo (15 Oct)

`pixi run sim world:=test_lot sim_speed:=3`: ego and Nav2 up to 3 m/s (5 at most), in
Gazebo only. The real car keeps 1 m/s. Details: [docs/simulation.md](simulation.md).
