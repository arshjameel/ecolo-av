# The preflight: one run on stands, before the ground

* **Author:** Ben (the ask), written up 29 Oct 2026
* **Status:** Tentative: revised after the EGM review (29 Oct). Built on `actuator-test`;
  the drive is **refused on the real car** until a driven-wheel speed exists (TODO 49b)

Ben, 29 Oct: "Then we need maybe a test that speeds up, maintains and slows down and tests
all the sensors before we put it on the ground."

## The idea in one line

`pixi run preflight`: **one scripted run** with the wheels off the ground (speed up, hold,
slow down, brake; steer left, right, centre) while **every sensor is checked at the same
time**, then one PASS / FAIL line per check, as `car-day-check` prints.

## What the review changed

| # | Finding | Now |
|---|---|---|
| 1 | The car is rear-wheel drive; the wheel sensors are on the **front**. On stands `/vehicle/odom` reads ~0 while the motor spins: the speed cap can never fire | the drive is refused on the real car unless `vehicle.yaml` says `speed_wheels: rear` (TODO 49b). `--no-drive` checks the sensors alone. Every actuator test now stops after 4 s of "told to drive, measured < 0.05 m/s" (4: an 8-click Hall sensor needs 2-3 s of a slow start to say anything); the speed-up and brake steps stop the test when they time out |
| 1 | `speed_loop.mode pi` winds the throttle up against a speed it can't see | refused while the legacy bridge runs `pi` |
| 2 | A lost link with the C++ ESP32 (no command timeout, S3) | refused while the bridge's no-timeout lock is ON; the checklist asks for the Rust firmware with 6c and test 3.9 passed, and a wired link (the Orin over ssh + tmux, or a wired laptop) |
| 3 | A stuck throttle: the stop gave up after 5 s and said "Stopped" | **NOT STOPPED: E-STOP NOW**, `/safety/estop` true (set, never cleared), zeros for 10 s more, exit 6. Live: the speed rising while the command falls or sits at 0 stops every test |
| 4 | Slipping off the stands isn't caught live; 2.5 m of GPS can't tell stands from rolling | a live IMU watch on stands (below); the GPS line uses Doppler, else it is a WARN only |
| 5 | The IMU window | from the first command until the wheels reach 95% of it, at most 3 s; the wheels' change over the same window |
| 6 | Thresholds | each sensor's documented rate, pass at 0.8 x; gravity ± 0.5 m/s²; the centre from `steer_offset` and `steer_deadband`; a missed hold speed on the open-loop ESP32 is a WARN (a throttle-gain finding) |
| 7 | Reuse | `preflight` is an actuator test (`pr.TESTS`); `pixi run preflight` = `actuator-test preflight --stands`; the same `decide()`, typed word and stops. `--bench` is local, like `--sim` |
| 8 | People | the e-stop person is the stop; someone watches the rear wheels with a tachometer |

## The script

The tests slot of twist_mux (`/drivers/tests/cmd_vel`, priority 60), as every actuator test.

| # | Step | Command | Ends when |
|---|---|---|---|
| 1 | wait until still | 0 | still 1 s |
| 2 | settle | 0 | 3 s: the IMU's bias, the GPS's start |
| 3 | speed up | the cap (0.3 m/s) | 95% of it for 1 s; **10 s: stop the test** |
| 4 | hold | the cap | 5 s |
| 5 | slow down | half the cap | 3 s |
| 6 | brake | 0 | still 1 s; **10 s: stop the test** |
| 7-9 | steer left, right, centre | creep 0.2 m/s, full lock left, right, straight | 3 s each |
| 10 | stop | 0 | still 1 s; **10 s: stop the test** |

Steering creeps because ego holds the wheels below 0.05 m/s. About 40 s.
`--no-drive`: steps 1-2 only, 15 s, zeros only.

## The checks

Rates: pass at **0.8 x** the documented rate (on the car; the stand-ins have their own).

| Check | Topic | Car rate (source) | Also |
|---|---|---|---|
| wheel speed | `/vehicle/odom` | 100 Hz (`aav.dbc` 0x200/0x201) | finite |
| each wheel | `/vehicle/wheels` | 100 Hz (the same) | a wheel measured |
| steering | `/vehicle/steering` | 50 Hz (the uplink; **needs H9**: `--skip steering` until then) | within 1.2 x full lock |
| raw IMU | `/sensors/imu/data_raw` | 200 Hz (Yahboom, `learn-sensors.md`) | at rest: \|a\| = 9.81 ± 0.5 m/s², gyro < 0.1 rad/s; not frozen |
| filtered IMU | `/sensors/imu/data` | 200 Hz (one for one) | finite |
| GPS | `/sensors/gps/fix` | 1 Hz (ZED-F9R factory) | a fix |
| lidar | `/sensors/lidar/points` | 10 Hz (VLP-16) | some points |
| radar | `/sensors/radar/points` | 10 Hz (TI mmWave) | an empty cloud is fine |
| front camera | `/sensors/camera/front_left/image_raw` | 15 Hz | width, height > 0 |
| battery | `/vehicle/battery` | 1 Hz (Victron) | a voltage |
| speeds up | wheels | | ≥ 80% of the cap |
| holds | wheels | | within 25% of the cap (open loop: WARN) |
| slows down | wheels | | the slow step 0.2 x cap under the hold |
| brakes | wheels | | still within 5 s |
| steers | `/vehicle/steering` | | left > 0.3 lock, right < -0.3 lock, centre within `steer_offset` ± (2 x `steer_deadband` + 0.02 rad); no sensor: the person says y/n |

A sensor meant to be off is named: `--skip gps lidar` gives SKIP, not FAIL. Nothing is
skipped by itself. Measured on a laptop, the lidar and camera rates include the WiFi.

## Stands: disagreeing on purpose

From [learn-speed.md §9](../learn-speed.md#9-on-stands-disagreeing-on-purpose).

| Line | On stands: PASS when | Gazebo, bench (on the ground): PASS when |
|---|---|---|
| IMU | its speed change (step 3, to 95%, ≤ 3 s) < 0.4 x the wheels' | ≥ 0.6 x the wheels' |
| GPS, with Doppler (`fix_velocity`) | under 0.1 m/s while the wheels spin | the hold's Doppler ≥ 0.6 x the wheels' |
| GPS, fixes only | **WARN only** (2.5 m of noise can't tell stands from 2 m of rolling) | moved ≥ 0.5 x the wheels' distance |

The stand-ins say "(sim: on the ground, so the opposite check)" on each line: the same
maths runs both ways. On stands with a driven-wheel speed, the sensor watchdog may latch
**DISAGREE** (wheels against a still GPS); it holds only Nav2 and friends, not the
tests: `pixi run watchdog-clear` after.

## Safety

| What | How |
|---|---|
| Who must be there | **the e-stop person** on the Kar-Tech: they are the stop (Ctrl-C and the joystick stop the software only); **a rear-wheel watcher** with a tachometer app; the laptop person; the safety driver on the joystick |
| The word to type | **`preflight`**, after the checklist is read out loud. No keyboard: refused |
| Caps | 0.3 m/s (`STANDS_CAP`); `--max-speed` lowers it only. On the car it is always a stands run |
| Refused on the car | no driven-wheel speed (`speed_wheels` ≠ `rear`, TODO 49b); the no-timeout lock ON (the C++ ESP32); `speed_loop.mode pi`; no `/vehicle/odom`; ego not answering; `/system/source` not REAL |
| Stops it live | Ctrl-C; `/safety/estop`; a person taking over; twist_mux silent 2 s; `/vehicle/odom` stale 0.5 s; over 0.46 m/s; the envelope's time or distance; **told to drive, < 0.05 m/s for 4 s**; **the speed rising while the command falls**; a must step timing out; on stands, **the IMU says it moves** (over 0.15 m/s in 2 s, or turning over 0.1 rad/s while steering) |
| If it won't stop | 5 s after the stop: NOT STOPPED, `/safety/estop` true, zeros for 10 s, exit 6 |
| Where it runs | `--sim` Gazebo, `--bench` bench mode: `tools/ros.sh --local`. Neither: the car (not local) |

A lost link, layer by layer:

| Link lost | What stops the wheels |
|---|---|
| the laptop ↔ the Orin (WiFi) | twist_mux's tests slot times out (0.5 s), ego sends 0: why the checklist wants a wired link |
| the preflight dies | the same: twist_mux times out |
| ego ↔ the bridge | the bridge's command goes stale (`cmd_timeout`): brake frames |
| the bridge ↔ the ESP32 (the Rust firmware) | the ESP32 brakes 100 ms after the counter stops (test 3.9) |
| the same, the C++ firmware | **nothing**: refused while the no-timeout lock is ON |
| everything | the e-stop person |

The spine's hardware test mode (Arsh, #15) is not used: no frame carries a test request to
the spine yet. The preflight drives the normal way, through twist_mux and ego.

## The code

| Part | Where |
|---|---|
| the steps | `procedures.preflight`, in `pr.TESTS` |
| the checks, the lines, the IMU watch (no ROS) | `aav_actuator_tests/preflight.py`, `test/test_preflight.py` |
| the refusals, the live stops | `safety.py`: `preflight_refusal`, `unseen`, `runaway`, `PREFLIGHT_CHECKLIST` |
| the node: counters, the e-stop, the report | `aav_actuator_tests/preflight_node.py`, used by `actuator_test` |
| the task | `pixi run preflight` → `tools/actuator_test.sh preflight --stands` |
| the runbook | car-day session 9; `docs/actuator-tests.md` |

Each run saves `~/aav-tests/<time>-preflight/`: `log.csv`, `summary.txt` (the lines),
`meta.yaml`, `preflight.yaml`.
