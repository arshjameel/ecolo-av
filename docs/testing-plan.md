# The testing plan

Written 18 November 2026 by Ben Woodhead, for the team and our professor. It says **what we
test, in what order, and how we know each step passed**. The detail of each test is in
[Every test](tests.md) and [Actuator tests](actuator-tests.md); the day itself in
[Car day](car-day.md).

> **The short version**
> - **Seven phases**, from the car on stands to a twin of the campus in Ravensim.
> - **One command per phase** runs its tests and writes one report. Today most tests run one by one; the plan makes them one automated system (our professor's ask).
> - **Stands first** (the car is being upgraded now), then **the hallway** this winter, then mapping drives outside.
> - Every row says **Built**, **To build** or **Untested on the car**. Nothing is "works" until it has run on the car.

## The order

| Phase | What | Where | Needs |
|---|---|---|---|
| **0** | **After the install**: every laptop and car computer checked, then the car together ([its page](after-install.md)) | laptops, then the car | the upgrade done |
| **A** | The car on stands: preflight, every sensor, steering, throttle, **faults on purpose** (losing the Orin too) | the shop, wheels off the ground | the hardware lead; phase 0 passed |
| **B** | **The transfer functions**: accelerate, brake, steer, measured and compared with the model | **the long hallway** (winter), then a lot | phase A passed |
| **C** | **The recording kit**: every sensor, clocks in step | the car | phase A |
| **D** | **Maps from hand drives**: SLAM makes the map, NDT and AMCL find the car on it | campus, driven by hand | phase C |
| **E** | **The map editor**: an overhead map, street maps and ours, editable | a laptop | phase D's maps |
| **F** | **The labelling loop**: trees, buildings, parked cars; a better model after each drive | a laptop with a GPU, then the Orin | phases C and E |
| **G** | **The campus twin** in Ravensim, built from the labelled maps | a laptop | phases D to F |

Alongside A and B: **the debug screen**: who is sending commands, and what (below; built 19 Nov, the map window's D page).

## What runs automatically today

| Level | Command | When | Status |
|---|---|---|---|
| The code | `pixi run test` (2149 tests, 18 Nov) | every push and pull request (CI: Linux, ARM) | Built |
| The simulator | `pixi run sim-check` and its modes (`--obstacle`, `--kerb`, `--movers`, `--faults`, `--actuator-faults`, `--three-point`...) | each night on CI; by hand before a release | Built |
| The ESP32 | in QEMU: it brakes when the Orin's heartbeat stops | in `pixi run test` | Built; untested on the real board |
| The bench | `pixi run bench`, `pixi run bench-check` | by hand | Built |
| On stands | `pixi run actuator-test preflight --stands` | by hand | Built; **untested on the car** |
| On stands, all of phase A | `pixi run car-test --stands` (`--sim --no-manual`: against its own Ravensim) | by hand | Built 19 Nov; against Ravensim 11 PASS, 3 SKIPPED (car only); **untested on the car** |
| On the ground | `pixi run actuator-test speed-up / coast / brake` | by hand | Built; **untested on the car** |
| In the hallway, all of phase B | `pixi run car-test --hallway` (`--sim --no-manual`: a pretend hallway in its own Ravensim) | by hand | Built 19 Nov (TODO 248); against Ravensim and the pretend car; **untested on the car** |

## Phase A: the car on stands

**Goal:** before the wheels touch the ground, every part answers, and every fault stops the car.

### A1. The preflight (built)

`pixi run actuator-test preflight --stands`: speed up, hold, slow, brake, steer, and every
sensor at once. **PASS / FAIL per line.** It presses the e-stop itself if the car won't stop.

### A2. One command for the whole stands test (built 19 Nov; untested on the car)

`pixi run car-test --stands` runs, in order, and writes one report folder
(`~/aav-tests/<time>-stands/report.md`). Any FAIL ends the run. How it works:
[car-test](actuator-tests.md#car-test-the-whole-stands-test).

| Step | Test | Passes when | Status |
|---|---|---|---|
| 1 | The preflight (A1) | every line PASS | Built; untested on the car |
| 2 | Every sensor read: lidar, the front camera, radar, IMU, GPS, wheels (the preflight's `--no-drive`) | each at its documented rate | Built; untested on the car |
| 3 | Steering: sweep, rate, centre (`steer-sweep`, `steer-rate`, `steer-centre`) | its numbers printed for `vehicle.yaml` | Built; untested on the car |
| 4 | Throttle on free wheels (`wheel-steps`, 3.14: 0.1, 0.2, 0.3, 0.15 m/s) | the wheels' speed follows the command | Built; untested on the car |
| 5 | **Faults on purpose** (table below) | each one stops the car, and something says why | Built; untested on the car |

| Fault | How the runner causes it | Expected | Status |
|---|---|---|---|
| **The Orin is lost** | car: asks a person to pull the Orin's cable to the Motor Pi; sim: pauses its own Ravensim's output mux | the ESP32 brakes and holds, on its own | Built; untested on the car. No new hook in the output mux |
| The GPS, the front camera or the IMU is lost | car: asks a person to unplug it; sim: Ravensim's `fault.<sensor> stop` | the sensor watchdog **names it** within its `max_age` + 1 s | Built; untested on the car |
| The lidar is lost | car: asks a person to unplug it; sim: Ravensim's `fault.lidar stop` (`sim_lidar_relay`) | the same: the watchdog's `lidar` input (1.0 s, stops, as the camera; TODO 246) | Built; untested on the car |
| A Pi is lost | asks a person to pull the camera Pi's cable | the car stops, something says why | Built; car only |
| The e-stop | car: asks a person to press it; sim: the runner presses `/safety/estop` | everything stops | Built; untested on the car |
| A cable is pulled | asks a person to pull it | the same as the software version | Built; car only |

> ℹ️ The sensor watchdog's lock holds navigation and the AI slot, never a test: so on a
> sensor fault the runner stops itself once the watchdog **names** the sensor. The step
> measures how fast the watchdog notices.

> ⚠️ **On stands, never test braking distance.** Free wheels carry no weight, so the brake
> numbers mean nothing there. Braking is phase B, on the ground.

## Phase B: the transfer functions, in the hallway

**Goal:** the numbers in `vehicle.yaml` are measured, not guessed, and the model matches the car.

### What "the transfer function" is here

Each actuator is modelled as **a dead time, then a first-order lag**: the command goes in,
nothing happens for the dead time, then the car moves towards the command with time constant
"lag". Today these are **guesses** in `vehicle.yaml`:

| Actuator | Numbers to measure | Test | Status |
|---|---|---|---|
| Throttle | `throttle_dead_time`, `throttle_lag`, top acceleration | `hall-speed-up` (B1; 4.1 in a lot) | Built; **untested on the car** |
| Drag | how fast the car slows on its own | `hall-coast` (B2; 4.2) | Built; **untested on the car** |
| Brake | `brake_dead_time`, `brake_lag`, deceleration, stopping distance | `hall-brake` (B3; 4.3) | Built; **untested on the car** |
| Steering | `steer_dead_time`, `steer_lag` on the ground; rate, centre on stands | `hall-steer` (B4); `steer-rate` (3.11), `steer-centre` (3.12) | Built; **untested on the car** |

**One command runs them all:** `pixi run car-test --hallway` (TODO 248;
[how](actuator-tests.md#car-test-hallway-the-transfer-functions)).

### Why the hallway works

We can't go fast indoors, but **the model's numbers don't depend on speed** at our speeds.
Distance needed to reach a speed and stop again:

| Top speed | Speed up (0.5 m/s²) | Stop (1 m/s²) | Plus a safety margin | Total |
|---|---|---|---|---|
| 0.5 m/s | 0.25 m | 0.13 m | 3 m | about 3.5 m |
| 1.0 m/s | 1.0 m | 0.5 m | 4 m | about 5.5 m |
| 1.5 m/s | 2.3 m | 1.1 m | 5 m | about 8.5 m |

Measure the hallway first, and keep the shared 1.0 m/s limit unless it is clearly long enough.

`car-test --hallway` works the top speed out from the length you type
(`safety.hallway_limits`: speed up at 0.5 m/s², 0.3 s to react, stop at 0.5 m/s², plus the
margin), never above 1.0 m/s. Each pass brakes in time to stand still before
**length − margin**, by the wheels' distance; past it, it stops at once.

### Using the IMU as well as the wheels

| Source | Gives | Good | Weak |
|---|---|---|---|
| Wheel speed | speed | no drift | its acceleration (the change in speed) is noisy |
| **IMU** | **acceleration directly**, 100 times a second | quick and smooth | a small offset; the floor's slope shows up as acceleration |
| Both together | acceleration and braking force you can trust | | |

How: stand still for 10 s first to measure the IMU's offset; check the hallway is level;
then blend the IMU's acceleration with the wheels' speed (the tests already record both:
`ax` and the wheel speed in each run's CSV). The fit gives each number **with error bars**
(TODO 104).

| Step | Status |
|---|---|
| 10 s still before each pass: the IMU's offset and noise | Built (`fusion.still_imu`) |
| The slope: the offsets facing each way (the car turned round between passes) | Built (`fusion.slope_from`) |
| The blend: a small Kalman filter, speed and IMU bias | Built (`fusion.fuse`) |
| Forces: F = m a, with `vehicle.yaml`'s mass | Built (`fusion.forces`) |
| Error bars: mean ± sd over the passes, trusted or not | Built (`errorbars.py`) |

### Compare with the model

For each run: feed the same commands into the model (the simulator's car) and lay the two
speed curves over each other. **Passes when** the model's speed stays within 10 % of the
car's, and its stopping distance within 0.2 m. Then the simulator, ego's predictions and the
planners all use the measured car.

Built (`hallway.replay`, `compare`): every pass, with the fitted numbers and with
`vehicle.yaml`'s, into `compare.csv` and `overlays/`. **Untested on the car.**

## The debug screen

Our professor's idea: the map screen doubles as a **debug screen**, like the hardware panel.

| It shows | From | Status |
|---|---|---|
| **Who is driving**: joystick, keyboard, the navigator (the AI), a test, the buttons | `/control/command_mux/state` (`aav_mux_state`: twist_mux's rule, beside it) | **Built 19 Nov**; in the sim it named the keyboard, the navigator, a takeover and the hand-back, agreeing with twist_mux's output 226 times of 227 |
| What that driver asks (speed, turn) | `/control/cmd_vel` (the chosen driver's, as twist_mux passed it) | **Built 19 Nov** |
| What ego sends to the car (steering, braking or speeding up) | `/ego/actuator_cmd` | **Built 19 Nov** |
| Whether the output gate is open, and why not | `/safety/output_mux/state` | **Built 19 Nov** |
| What the car did (speed, steering, throttle, brake, mode, faults) | `/vehicle/status` | **Built 19 Nov** |
| The locks, and whom they hold; the sensor watchdog | `/control/command_mux/state`, `/safety/sensor_watchdog/state` | **Built 19 Nov** |
| Real or simulated data | `/system/source` | **Built 19 Nov** (and the orange band) |

It is the map window's **debug page**: press D ([how it looks](hud.md#the-debug-page-who-is-driving)).
`pixi run map --demo --debug-page` shows it with pretend data. **Untested:** the page drawn
on a screen (no screen was free when it was built), and on the car.

## Phase C: the recording kit

**Goal:** one drive records everything the later phases need.

| Item | Status |
|---|---|
| `tools/record.sh --lidar` (cameras, radar, detections, localization, lidar) | Built |
| **`pixi run record-drive`** ([Recording a mapping drive](recording.md)): raw lidar, the IMU at full rate, wheels, raw GPS, every camera, the commands and who drives; MCAP + zstd in 2 GB / 5 min files | **Built 19 Nov**; tested in Ravensim (`--sim`), untested on the car |
| Checks first: disk, every topic's rate, stamps, `/tf_static`'s sensor frames | **Built 19 Nov**; untested on the car |
| A summary after: per topic count, rate, gaps, size; "good for" the map, SLAM, labels, the transfer functions | **Built 19 Nov** |
| The clocks on every computer in step: `chronyc tracking` here and over ssh, in the checks | **Built 19 Nov**; to check on the car (blocker 5) |
| Space: about **87 GB an hour** before zstd (3 JPEG cameras 49, lidar 37 with its packets), so a fast SSD on the Orin | Estimated; to measure on the car |

## Phase D: maps from hand drives

**Goal:** the car finds itself on our own map, so a failing GPS stops mattering.

| Tool | Its job | Status |
|---|---|---|
| **KISS-SLAM** (offline: `pixi run map-drive`) | **makes** the map from a hand drive, closing loops, placed by the GPS | Built 19 Nov ([Maps from drives](maps-from-drives.md)); in Ravensim 2.3 cm off the truth's map (the EKF's: 27-33 cm); no loop closure found on the plain test lot; **untested on the car** |
| NDT's map from those poses | the drive into NDT's map, in 64 m chunks | Built (`map-drive`; `ndt-map build --poses-file`) |
| A 2D map from the same points | the planner's walls and kerbs: 0.3-2 m occupied, the rays free | Built 19 Nov (`map-drive`): every occupied cell within one of the truth's |
| Several drives merged | keep what both saw (`map-drive --merge`) | Built 19 Nov, two drives; more than two: to build |
| **NDT** | finds the car on the map, precisely | Built; 5 cm in the sim |
| **AMCL** (Rust, 18 Nov) | finds a **lost** car, then hands over to NDT | Being built |
| NDT as the main position (GPS only checks) | TODO 210 | To build |

**Several drives on different days:** what is there every time is permanent; what comes and
goes (parked cars) drops out.

**Passes when** NDT on our own map stays within 0.2 m of the GPS on a good-GPS day, and
keeps the car on its lane where the GPS fails.

## Phase E: the map editor

**Goal:** an overhead map where a person can check and fix everything.

| Feature | Why |
|---|---|
| Layers: OpenStreetMap's roads, our SLAM map, the labels | see all of it at once |
| Drag our map onto OpenStreetMap | line them up |
| Box-select and delete | remove parked cars, people |
| Relabel ("tree, not wall") | fix the labels |
| Approve, then save the chunks | a person always has the last word |

To build, in the developer window (Bevy). The campus from OpenStreetMap is already built
(`pixi run campus`: 574 buildings, 7918 trees, 396 roads, 18 Nov).

## Phase F: the labelling loop

**Goal:** after each drive, a model that tells **trees, buildings, poles, kerbs, parked cars
and people** apart, and a score that shows whether it improved.

```
a drive ─► big offline models label it ─► labels onto the lidar points
              ("tree", "building", "car")         │
                                                  ▼
                                   the editor: a person fixes mistakes
                                                  │
                                                  ▼
             train our small model ─► score it on the drive ─► better than last time?
```

| Part | Status |
|---|---|
| The detector (RF-DETR, Apache-2.0): cars, people, signs as boxes | Built |
| Trees and buildings | To build: the detector doesn't know them |
| Camera boxes onto lidar points (camera-to-lidar calibration, TODO 156) | To build |
| The score after each drive | To build |

> ⚠️ **Licences, if the work is ever commercial:** the YOLO option is AGPL-3.0; the big
> public datasets (SemanticKITTI, nuScenes, Cityscapes) are non-commercial. Train on our own
> labelled drives.

## Phase G: the campus twin

**Goal:** Ravensim's campus matches the real one, so every test runs there first.

| From the labelled map | In Ravensim |
|---|---|
| Buildings | OpenStreetMap's outlines, heights from our lidar |
| Trees, poles, signs | at their measured places |
| Kerbs and roads | from our map |
| Parking areas | cars parked at random, to test "not on the map" |

**Passes when** the sim's lidar and the real lidar see the same scene: a drive replayed in
the twin scores within NDT's error of the real drive.

## Safety rules for every phase

| Rule | Why |
|---|---|
| A named e-stop person at every test | the one sure stop |
| Wheels off the ground until phase A passes | nothing can run away |
| Never the `pi` brake loop on stands | it needs a real wheel speed (TODO 101) |
| The hallway cleared, a spotter at each end | people walk there |
| The 1.0 m/s shared limit until phase B's numbers are in | the stop distance is still a guess |
| Record every session | the report is the evidence |

## See also

- [Every test](tests.md): every test, step by step
- [Actuator tests](actuator-tests.md): the tests behind phase B
- [Car day](car-day.md): the runbook for a day at the car
- [The safety case](safety-case.md): the hazards, and the tests behind each
