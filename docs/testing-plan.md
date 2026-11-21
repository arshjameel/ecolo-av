# The testing plan

**The testers' strategy**: what the people on the team do with the car, in order, and what
each session gives the next. First written 18 November 2026 by Ben Woodhead; rewritten
19 November around the testers' sessions (Ben: "Now that we know what we have and what is
possible then lets start figuring out testing").

The detail of each test is in [Every test](tests.md) and [Actuator tests](actuator-tests.md);
the day itself in [Car day](car-day.md); the checks after an install in
[After the install](after-install.md).

> **The short version**
> 1. **Bench → stands → hallway → lot → campus mapping drives → offline processing → the car
>    drives itself.** Each step has a pass rule; nothing moves on until it passes.
> 2. **A boot self-test** at every start: every sensor, actuator and lock answers, or the car
>    **refuses to arm**. Built 19 Nov (TODO 255, `aav_self_test_rs`); untested on the car.
> 3. **Stands**: speed up, slow down, steering lock to lock. Numbers into `vehicle.yaml`.
> 4. **Hallway**: start and stop runs, measured three ways (wheels, IMU, NDT), **empty and
>    with a person aboard**. The transfer functions tune the car model and the adaptive brake.
> 5. **Mapping drives, then offline processing on a laptop**: several passes of each campus
>    area → NDT maps first → occupancy later, to remove parked cars and people.
> 6. Every row says **Built**, **To build** or **Untested on the car**. Nothing is "works"
>    until it has run on the car. Today **nothing here has run on the car** (it is being upgraded).

## The strategy on one page

### The ladder

```text
 0 bench / laptops ─▶ 1 boot self-test ─▶ 2 stands ─▶ 3 hallway ─▶ 4 lot
   (after-install)      (every start)       (A)         (B)          (B, faster)
                                                                        │
       7 the car drives itself ◀── 6 offline processing ◀── 5 campus mapping drives
         (NDT leads, routes)        (laptop: D, E, F, G)       (C: record by hand)
```

### Each step: why, its pass rule, and what it feeds

| # | Step | Why | Passes when | Feeds |
|---|---|---|---|---|
| 0 | **Bench and laptops** ([after the install](after-install.md)) | the code and every computer work before the car | every box ✅ | step 1 |
| 1 | **Boot self-test** | "make sure we have control over everything" | every sensor, actuator and lock answers; the car arms | every session after |
| 2 | **Stands** (phase A) | steering and throttle measured with nothing able to run away | `car-test --stands` all PASS; every fault stops the car | steering numbers → `vehicle.yaml`; step 3 |
| 3 | **Hallway** (phase B) | the transfer functions, indoors, in winter | the model's speed within 10 % of the car's, its stop within 0.2 m | delays and lags → `vehicle.yaml` → the sim's car model → ego and the planners; the adaptive brake |
| 4 | **Lot** (phase B, faster) | the same at 2-3 m/s; circles; drift | the same rule, at the higher speeds | the same, at speed |
| 5 | **Mapping drives** (phase C) | the raw points and pictures every map is built from | `record-drive`'s summary says "good for" the map and SLAM | step 6 |
| 6 | **Offline processing** (phases D-G) | maps, labels and the twin, made on a laptop, never on the car | NDT on our map within 0.2 m of a good GPS | NDT's map → step 7; the campus twin in Ravensim |
| 7 | **The car drives itself** | the goal | the route done at low speed, no takeover needed for safety | the scores we track |

### How the numbers flow

```text
 stands ─────── steering: offset, deadband, rate, max angle ──┐
 hallway, lot ─ throttle, brake, steering: dead time + lag ───┼─▶ vehicle.yaml ─▶ the sim's car model
 scales ─────── mass, empty and with a person ────────────────┘        │            │
                                                                        ▼            ▼
                                                     the adaptive brake      ego's limits, MPPI, MPC
 mapping drives ─▶ map-drive (offline) ─▶ NDT's map + 2D map ─▶ NDT leads ─▶ the car drives itself
```

## What runs automatically today

| Level | Command | When | Status |
|---|---|---|---|
| The code | `pixi run test` (2149 tests, 18 Nov) | every push and pull request (CI: Linux, ARM) | Built |
| The simulator | `pixi run sim-check` and its modes (`--obstacle`, `--kerb`, `--movers`, `--faults`, `--actuator-faults`, `--three-point`, `--loaded-stop`...) | each night on CI; by hand before a release | Built |
| The ESP32 | in QEMU: it brakes when the Orin's heartbeat stops | in `pixi run test` | Built; untested on the real board |
| The bench | `pixi run bench`, `pixi run bench-check` | by hand | Built |
| After the install | [one page of checks](after-install.md) (phase 0) | after each upgrade | Written; untested on the car |
| On stands | `pixi run actuator-test preflight --stands` | by hand | Built; **untested on the car** |
| On stands, all of phase A | `pixi run car-test --stands` (`--sim --no-manual`: against its own Ravensim) | by hand | Built 19 Nov; against Ravensim 11 PASS, 3 SKIPPED (car only); **untested on the car** |
| In the hallway, all of phase B | `pixi run car-test --hallway` (`--sim --no-manual`: a pretend hallway) | by hand | Built 19 Nov (TODO 248); **untested on the car** |
| On the ground | `pixi run actuator-test speed-up / coast / brake / circles / straight` | by hand, in a lot | Built; **untested on the car** |
| A mapping drive | `pixi run record-drive` (`--sim`: its own test) | by hand | Built 19 Nov; **untested on the car** |
| Maps from a drive | `pixi run map-drive` (offline) | by hand, on a laptop | Built 19 Nov; Ravensim only |

## The boot self-test

Ben, 19 Nov: "Make sure we have control over everything with a basic boot self test."

### What exists today

| Check | What it covers | When it runs | Status |
|---|---|---|---|
| `car_setup.sh <role> check` | one computer: its release, its devices (`/dev/aav/*`), its service | by hand, after an install | Built; untested on the upgraded car |
| `pixi run check car` | every topic in the contract, at its rate | by hand, from a laptop | Built; untested on the car |
| `actuator-test preflight --stands` | every sensor's rate and values; speed up, hold, slow, brake; steer | by hand, on stands | Built; untested on the car |
| `car-test --stands` | the preflight, steering, throttle, faults on purpose | by hand, on stands | Built; untested on the car |
| `record-drive --check` | disk, rates, stamps, clocks, sensor frames | by hand, before a drive | Built; untested on the car |
| **The spine's self-test** (`spine-core`, `Lifecycle::SelfTest`) | the steering and brake drivers, the throttle DAC, the wheel encoder: **nothing arms until they answer** | at every power-up, on the board | Built in the Rust spine; **not on the car** (it runs the 2025-26 C++ ESP32). TODO 0b (Arsh): today it passes after one tick with no fault |
| `spine_bridge`'s arm rule | refuses `/spine/arm` during an e-stop or while the output gate isn't OPEN | every arm | Built |
| **The boot self-test** (`boot_self_test`, `aav_self_test_rs`) | everything below, in one READY / NOT READY | always, next to the mux on the car | Built 19 Nov (TODO 255); **untested on the car** |

### The boot self-test (TODO 255)

**One check, at start-up, that covers the whole car and refuses to arm until it passes.**

```text
 power on ─▶ every computer up ─▶ BOOT SELF-TEST ─┬─ all ✅ ─▶ READY ─▶ a person arms
                                                   └─ any ❌ ─▶ NOT READY: says which; arming refused
```

| It checks | Pass | Reads |
|---|---|---|
| Every sensor | the sensor watchdog speaks and doesn't say STOP: every sensor it watches at its rate (lidar, IMU, GPS, wheels, EKFs, cameras, HUD) | `/safety/sensor_watchdog/state` |
| Every actuator answers | `actuators:=can`: the spine's status fresh, no steering, brake, throttle or CAN fault. Today's UART: the bridge's frames go out | `/vehicle/status`; `/control/car/link_frame` |
| Every lock is alive | the sensor watchdog, the geofence, the output gate each speaking; the stop zone running where navigation is | their `/safety/*/state` topics; the ROS graph |
| The clocks | the IMU's, the GPS's and the wheels' stamps within 0.5 s of the Orin's | their headers |

All passing for 2 s: **READY**. Any failing: **NOT READY - ** and each one, why.
`/safety/self_test/status` (DiagnosticStatus; `ros2 topic echo`, Foxglove). Not on the
HUD yet.

| Who | How it refuses |
|---|---|
| `actuators:=can` | `spine_bridge` (`require_ready`, both languages) refuses `/spine/arm` until it said READY within 2 s; silence refuses too. Disarming and the e-stop never wait |
| `actuators:=uart` (today) | nothing arms on that path (the UART bridge has no arm): READY is **for the person**, card 1 |
| Ravensim, the bench | not run; their `spine_bridge` doesn't wait for it (`arm_when_open` arms them) |

It moves nothing: no actuator steps (that stays on stands). **Gaps:** the person's e-stop
press stays a card 1 row; the spine's own self-test passes after one tick (TODO 0b, Arsh's
firmware), so a fault-free status says only that no driver reported a fault yet.

## Phase A: the car on stands

**Goal:** before the wheels touch the ground, every part answers, every number the stands can
give is measured, and every fault stops the car.

**Who:** the hardware lead (the car, the stands, the cables), the e-stop person, the laptop
person, someone watching the rear wheels.

### Speed up and slow down

| Test | What | Status |
|---|---|---|
| `actuator-test preflight --stands` | speed up to 0.3 m/s, hold 5 s, slow to 0.15, brake, steer | Built; untested on the car |
| `wheel-steps` (3.14, in `car-test --stands`) | 0.1, 0.2, 0.3, 0.15 m/s, 4 s each: does the wheel speed follow | Built; untested on the car |

> ⚠️ **The known gap (TODO 49b): nothing measures the driven rear wheels on stands.** The
> wheel sensors are on the **front** wheels; the motor drives the **rear**. On stands
> `/vehicle/odom` reads 0 while the rear wheels spin, so no speed cap can work. Until
> `vehicle.yaml`'s `speed_wheels` says `rear`, the preflight runs `--no-drive` (sensors only).
> Until then: a tachometer app on a rear wheel, read by a person, and the throttle by hand.

### Steering lock to lock

| Test | What | Gives (`vehicle.yaml`) | Status |
|---|---|---|---|
| **LOCK-TO-LOCK CHECK** (hardware panel, key K) | the steering actuator's ends and centre marked, then centre → left limit → right limit → centre at 100 counts a second; **PASSED** or **STOPPED** and why | the ends and centre (`calibration.rs`); `max_steering_angle` | Built; untested on the car |
| `steer-sweep` (3.10) | full left → centre → full right and back | `max_steering_angle`, `steer_offset` | Built; untested on the car |
| `steer-rate` (3.11) | centre → full lock → centre, each way | `steer_rate`, `steer_lag` | Built; untested on the car |
| `steer-centre` (3.12) | tiny steps (0.002 rad) each way | `steer_offset`, `steer_deadband` | Built; untested on the car |

No steering sensor yet? Each test stops and asks for the angle: read it with a protractor or
a phone on the wheel. How: [Actuator tests](actuator-tests.md#find-the-steering-actuators-ends-and-centre).

> ⚠️ **Never test braking distance on stands.** Free wheels carry no weight, so the brake
> numbers mean nothing there. Braking is phase B, on the ground.

### One command for the whole stands test

`pixi run car-test --stands` runs, in order, and writes one report
(`~/aav-tests/<time>-stands/report.md`). Any FAIL ends the run.
[How it works](actuator-tests.md#car-test-the-whole-stands-test).

| Step | Test | Passes when |
|---|---|---|
| 1 | The preflight | every line PASS |
| 2 | Every sensor read (`--no-drive`) | each at its documented rate |
| 3 | Steering: sweep, rate, centre | its numbers printed for `vehicle.yaml` |
| 4 | Throttle on free wheels (`wheel-steps`) | the wheels' speed follows the command |
| 5 | **Faults on purpose** (below) | each one stops the car, and something says why |

| Fault | How the runner causes it | Expected | Status |
|---|---|---|---|
| **The Orin is lost** | car: a person pulls the Orin's cable to the Motor Pi; sim: pauses the output mux | the ESP32 brakes and holds, on its own | Built; untested on the car |
| The GPS, the front camera or the IMU is lost | car: unplug it; sim: Ravensim's `fault.<sensor> stop` | the sensor watchdog **names it** within its `max_age` + 1 s | Built; untested on the car |
| The lidar is lost | car: unplug it; sim: `fault.lidar stop` | the watchdog's `lidar` input (1.0 s) | Built; untested on the car |
| A Pi is lost | a person pulls the camera Pi's cable | the car stops, something says why | Built; car only |
| The e-stop | car: a person presses it; sim: the runner presses `/safety/estop` | everything stops | Built; untested on the car |
| A cable is pulled | a person pulls it | the same as the software version | Built; car only |

The numbers it prints are **pasted into `vehicle.yaml` by a person**: no test writes the file.

## Phase B: the transfer functions, in the hallway

**Goal:** the numbers in `vehicle.yaml` are measured, not guessed, for the car **empty and
with a person aboard**, and the model matches the car. Ben, 19 Nov: "going down the hallway
start and stop using the imu and ndt data to measure the transfer function ... We need to
tune the tf".

### What "the transfer function" is here

Each actuator is modelled as **a dead time, then a first-order lag**: the command goes in,
nothing happens for the dead time, then the car moves towards the command with time constant
"lag". Today these are **guesses** in `vehicle.yaml`.

```text
 command ──┐                ┌──────────────── car's speed
           │  dead time     │  lag (63 % of the way after one lag)
           └────────────────┘
```

| Actuator | Numbers | Pass | Status |
|---|---|---|---|
| Throttle | `throttle_dead_time`, `throttle_lag`, `max_accel` | `hall-speed-up` (B1) | Built; **untested on the car** |
| Drag | how fast the car slows on its own | `hall-coast` (B2) | Built; **untested on the car** |
| Brake | `brake_dead_time`, `brake_lag`, `max_brake`, stopping distance | `hall-brake` (B3) | Built; **untested on the car** |
| Steering | `steer_dead_time`, `steer_lag` on the ground | `hall-steer` (B4) | Built; **untested on the car** |

**One command runs them all:** `pixi run car-test --hallway`
([how](actuator-tests.md#car-test-hallway-the-transfer-functions)). Each pass: still 10 s,
speed up, then coast, brake or steer, then stop. Three times each, the car turned round
between passes so the floor's slope cancels.

### Why the hallway works

We can't go fast indoors, but **the model's numbers don't depend on speed** at our speeds.
`car-test --hallway` works the top speed out from the length you type (never above 1.0 m/s):

| Hallway (taped) | Margin | Top speed |
|---|---|---|
| 4 m | 3 m | 0.64 m/s |
| 5.3 m | 3 m | 1.0 m/s |
| 12 m | 3 m | 1.0 m/s |
| under 3.2 m | 3 m | refused |

A 15 m+ hallway is needed for a **trusted drag** (TODO 248's finding).

### Three measures of distance and speed

| Source | Gives | Good | Weak | Status |
|---|---|---|---|---|
| **Wheels** | speed, distance | no drift | the change in speed is noisy | Built |
| **IMU** | acceleration, 100 times a second | quick and smooth | a small offset; the floor's slope looks like acceleration | Built (`fusion.py`) |
| **NDT** on a hallway map | position, so distance and speed | independent of wheel slip and the wheel's size | **can slide along a plain corridor** (below) | On the car: built, **untested** (TODO 261); its pose in `car-test --hallway`: **to build** (TODO 256) |

How the first two are blended, all built, in `fusion.py` and `errorbars.py`:

| Step | How |
|---|---|
| Offset | stand still 10 s before each pass: the IMU's offset and noise (`fusion.still_imu`) |
| Slope | the offsets facing each way (`fusion.slope_from`); level enough under 1 % |
| Blend | a small Kalman filter: speed and the IMU's bias (`fusion.fuse`) |
| Forces | F = m a, with `vehicle.yaml`'s mass (`fusion.forces`) |
| Error bars | mean ± sd over the passes; a number is **trusted** or not (`errorbars.py`) |

### NDT in the hallway: the third measure

NDT matches the lidar to a map, so it measures distance without the wheels. **Its weakness
here:** a long, plain corridor looks the same all along. Along the hallway NDT can slide: it
"finds the wrong place". Across the hallway (the walls) it is sharp.

```text
   wall  ║▓▓   ▓▓         ▓▓   ▓▓║   ▓▓ = a box or cone, fixed for the session
         ║    ─────car────▶      ║   across: the walls pin it (trust)
   wall  ║  ▓▓        ▓▓        ▓║   along:  only the boxes pin it (trust less)
```

| Step | How |
|---|---|
| 1 | Put **a few fixed objects** (boxes, cones) along both walls, at uneven gaps; leave them all session |
| 2 | **Map the hallway first**: `pixi run record-drive --name hallway`, pushed slowly by hand, then `pixi run map-drive bags/<drive>` on a laptop |
| 3 | Run NDT on that map during the passes; record its pose beside the wheels and the IMU |
| 4 | **Trust NDT across more than along**: use it to check the wheels' distance, never alone |
| 5 | Where NDT and the wheels disagree by more than the error bars: look at the slip, the wheel's size, or NDT sliding |

NDT on the car (TODO 261, built 19 Nov; **untested on the car**):

```sh
pixi run car orin ndt:=true ndt_map:=<the hallway's .pcd> [start_x:= start_y:= start_yaw:=]
```

It runs on the Orin (the machine file's `ndt` part, beside the lidar), as the sim runs it:
from the start pose, scored only. Off by default; refused without its map or the lidar.
`pose_source:=ndt` (NDT leads, the GPS checks) is allowed only with `ndt:=true` and
`ndt_map:=`; the default stays `gps`.

> ⚠️ **To build (TODO 256):** `car-test --hallway` doesn't record NDT's pose yet. With no
> GPS indoors, `map-drive` places the map from the drive's start (it says so in its YAML).

### With and without a person aboard

The same brake stops a heavier car more slowly (a = F / m). In the car model, 150 kg aboard a
181 kg car brakes at 55 % of its empty strength ([How the car stops](stopping.md#4-a-heavier-car-the-adaptive-brake)).

| Step | How | Status |
|---|---|---|
| 1 | **Weigh the car**, empty and with the person (TODO 14) | To do on the car |
| 2 | `car-test --hallway` **empty** (driven from the laptop, a walker on the joystick beside it) | Built; untested on the car |
| 3 | `car-test --hallway` **with a person aboard**, the same passes | Built; it has no way to note the load yet (TODO 257) |
| 4 | Fit a transfer function for each load | Built (one fit per session) |
| 5 | Feed both to the **adaptive brake** (`adaptive_brake:=true`): it brakes harder when the car stops slower than the model, never above `safe_max` | Built in the sim (`sim_adapter`) and on the car's brake path (TODO 258: `legacy_uart_bridge`, after its pi loop; [where and why](stopping.md#on-the-car-todo-258-19-nov)); **untested on the car** |

### Tune the TF

```text
 hallway passes ─▶ fits ± error bars ─▶ vehicle-yaml.txt ─▶ a person pastes ─▶ vehicle.yaml
                                                                                     │
       re-run the passes through the model ◀────────── the sim's car model ◀─────────┘
              │
              └─▶ within 10 % speed and 0.2 m stop?  yes: done   no: more passes, or a better model
```

| Step | How | Status |
|---|---|---|
| Fit | each pass's dead time and lag; their mean ± sd | Built |
| Only trusted numbers | printed to paste; the rest commented out, with why | Built |
| Paste | a person pastes the lines into `vehicle.yaml` (no test writes the file) | by hand |
| Check | every pass replayed through the model with the fitted numbers and the old ones (`hallway.replay`, `compare.csv`, `overlays/`). **Passes when** the speed is within 10 % and the stop within 0.2 m, inside the error bars | Built; **untested on the car** |
| Then | the simulator, ego's predictions and the planners all use the measured car | automatic, from `vehicle.yaml` |

### Then the lot

The same tests faster, outdoors: `actuator-test speed-up`, `coast`, `brake --speeds 0.5 1.0`,
`circles`, `straight` ([stage 4](actuator-tests.md#stage-4-in-an-empty-lot)). Same pass rule.
Built; untested on the car.

## The debug screen

Our professor's idea: the map screen doubles as a **debug screen**, like the hardware panel.
Press **D** on the map window ([how it looks](hud.md#the-debug-page-who-is-driving)).

| It shows | From | Status |
|---|---|---|
| **Who is driving**: joystick, keyboard, the navigator, a test, the buttons | `/control/command_mux/state` | Built 19 Nov; agreed with twist_mux 226 times of 227 in the sim |
| What that driver asks (speed, turn) | `/control/cmd_vel` | Built 19 Nov |
| What ego sends to the car | `/ego/actuator_cmd` | Built 19 Nov |
| Whether the output gate is open, and why not | `/safety/output_mux/state` | Built 19 Nov |
| What the car did (speed, steering, throttle, brake, mode, faults) | `/vehicle/status` | Built 19 Nov |
| The locks, and whom they hold; the sensor watchdog | `/control/command_mux/state`, `/safety/sensor_watchdog/state` | Built 19 Nov |
| Real or simulated data | `/system/source` | Built 19 Nov (and the orange band) |

`pixi run map --demo --debug-page` shows it with pretend data. **Untested:** drawn on a screen,
and on the car. Keep it open at every session.

## Mapping drives and offline processing

Ben, 19 Nov: "Add offline processing to the list of stuff we can and will do. So we want to
build ndt maps of the campus and then add occupancy later so we can remove cars and stuff
with multiple drives being merged together."

**The plan: NDT maps of the campus first; occupancy and removing parked cars after.**

```text
 ON THE CAR (people drive)                 OFFLINE, ON A LAPTOP (never on the car)
 ─────────────────────────                 ─────────────────────────────────────────
 pass 1 (Mon 9:00) ─┐                      map-drive each ─▶ NDT map + 2D map + chunks
 pass 2 (Wed 14:00) ┼─ record-drive ─▶ SSD ─▶   │  placed by the GPS
 pass 3 (Fri 17:00) ┘   points + cameras        ▼
                                           merge the passes ─▶ occupancy across passes
                                                │               (parked cars out)
                                                ▼
                                           camera labels ─▶ the map editor ─▶ the campus twin
                                            (phase F)         (phase E)          (phase G)
```

### Phase C: the recording kit

**Goal:** one drive records everything the offline steps need.

| Item | Status |
|---|---|
| **`pixi run record-drive`** ([Recording a mapping drive](recording.md)): raw lidar, the IMU at full rate, wheels, raw GPS, every camera, the commands and who drives; MCAP + zstd in 2 GB / 5 min files | Built 19 Nov; tested in Ravensim (`--sim`), **untested on the car** |
| Checks first (`--check`): disk, every topic's rate, stamps, `/tf_static`'s sensor frames, the clocks over ssh | Built 19 Nov; untested on the car |
| A summary after: per topic count, rate, gaps, size; "good for" the map, SLAM, labels, the transfer functions | Built 19 Nov |
| Space: about **87 GB an hour** before zstd (3 cameras 49, lidar 37), so a fast SSD on the Orin | Estimated; to measure on the car |

### The driving

| Rule | Why |
|---|---|
| **By hand**, at walking to jogging pace (under 2 m/s) | the lidar's sweep blurs less; nothing drives itself on an unmapped place |
| **The same route each pass** | `map-area` can only judge what several passes looked at |
| **Several passes of each area, at different times and days** | different parked cars and people each time: what stays is the building |
| **Loops**: come back past where you started | SLAM closes the loop and removes the drift |
| One campus area a session (about 30-60 min) | under the SSD's room; one area's maps checked before the next |
| Note each pass (`--name`, `--note`): area, time, weather, parked cars | the offline steps and the labels need it |

### Offline, on a laptop: the pipeline

`pixi install --locked -e slam` once (x86_64 Linux, Apple-silicon Macs; not the Orin).

| # | Step | Command | Makes | Status |
|---|---|---|---|---|
| 1 | SLAM on each drive | `pixi run map-drive bags/<drive>` (KISS-SLAM) | the path, NDT's 3D map, a 2D map (walls, kerbs), 64 m chunks | Built 19 Nov; in Ravensim 2.3 cm off the truth's map; **untested on the car** |
| 2 | Placed on the earth | inside `map-drive`: fitted to the good GPS fixes | the map in the site's frame, lined up with OpenStreetMap | Built; untested with a real GPS |
| 3 | Loop closures | inside `map-drive` (MapClosures) | less drift on long drives | Built; **none found in the plain sim lot**; a real place should give features: untested |
| 4 | Passes merged | inside `map-area` (ICP onto the first drive); or `pixi run map-drive --merge <A> <B> <C>... --out maps/<area>` | one map; the 2D maps merged too | Built 19 Nov, 2 or more drives; 4-6 mm between Ravensim drives |
| 5 | **Occupancy across passes** | inside `map-area` (`tools/passes.py`) | static points kept, parked cars and people removed (below) | **Built 19 Nov (TODO 259)**: a parked box in 1 of 3 sim drives: 2,550 points to 0, the walls kept; **untested on real drives** |
| 6 | Camera labels onto the points | phase F | "car", "tree", "building" on each point: a parked car told from a wall even if it never moved | To build (TODO 252) |
| 7 | A person's final say | the map editor, phase E | box-delete, relabel, approve, save the chunks | To build (TODO 251, next) |
| 8 | The campus twin | phase G | Ravensim's campus from our maps | To build (TODO 253) |
| — | All of it, one command | `pixi run map-area <area> <drives...>` | steps 1-5 and the chunks, `labels.yaml` for step 7, one `report.md` | **Built 19 Nov (TODO 260)**; Ravensim only ([Maps from drives](maps-from-drives.md#many-drives-of-one-area)) |

### Occupancy across passes

`--merge` today keeps what **both** drives saw. That drops a parked car, but also anything
one pass missed because a truck hid it. Occupancy counts properly:

```text
 a cell, pass by pass:   pass 1   pass 2   pass 3     verdict
 a building wall           hit      hit      hit       static: keep
 a parked car              hit     free     hit       came and went: remove
 behind a truck            hit     unseen    hit       never seen free: keep
 a person                 free      hit     free      remove
```

| Idea | How |
|---|---|
| **Hit** | a lidar point landed in the cell |
| **Free** | a lidar beam passed **through** the cell (the 2D map already traces these rays) |
| **Unseen** | neither: no evidence, so it doesn't count against the cell |
| Keep | hit in every pass that could see it |
| Remove | hit in some passes, **free** in others |
| Still unsure | a car parked in the same spot every pass: occupancy can't tell; **phase F's camera labels** can |

Built 19 Nov as `map-area`, from the ideas of Removert (CC BY-NC-SA 4.0), ERASOR (GPL-3.0)
and DUFOMap (BSD-3-Clause): ideas only, no code. A thing seen by one pass only, never
seen free, is kept and labelled `seen_in` for the editor.

### Phase D: the car finds itself on our map

**Goal:** a failing GPS stops mattering.

| Tool | Its job | Status |
|---|---|---|
| **NDT** | finds the car on the map, precisely | Built; 5 cm in the sim; **not in `car.launch.py` yet** (TODO 261) |
| **AMCL** (Rust) | finds a **lost** car, then hands over to NDT | Built 18 Nov; ties stock on the lot |
| **NDT leads, the GPS checks** (`pose_source:=ndt`) | NDT gives the position, the GPS only checks it | Built 19 Nov, a challenger (TODO 210): Ravensim's lot, every leg with the GPS clean, stopped, jumped or drifting; the campus not scored yet; the car refuses it until a real map |

**Passes when** NDT on our own map stays within 0.2 m of the GPS on a good-GPS day, and
keeps the car on its lane where the GPS fails.

### Phase E: the map editor

**Goal:** an overhead map where a person checks and fixes everything (TODO 251, next).

| Feature | Why |
|---|---|
| Layers: OpenStreetMap's roads, our SLAM map, the 2D map, the labels | see all of it at once |
| Drag our map onto OpenStreetMap | line them up |
| Box-select and delete | remove the parked cars occupancy missed |
| Relabel ("tree, not wall") | fix the labels |
| Approve, then save the chunks | a person always has the last word |

**Built, 19 Nov:** [the map editor](map-editor.md), `pixi run map-edit bags/<drive>/map`
(Bevy, like the developer window; no ROS). Each row above is in it; it saves a new folder,
never over the old one. Proved in Ravensim: the parked cars deleted, NDT still finds the car.
Not yet looked at on a screen, nor run on a real drive.

### Phase F: the labelling loop

**Goal:** after each drive, a model that tells **trees, buildings, poles, kerbs, parked cars
and people** apart, and a score that shows whether it improved (TODO 252).

```text
 a drive ─▶ big offline models label it ─▶ labels onto the lidar points
                                                 │
                                                 ▼
                                  the editor: a person fixes mistakes
                                                 │
                                                 ▼
            train our small model ─▶ score it on the drive ─▶ better than last time?
```

| Part | Status |
|---|---|
| Open-vocabulary boxes: trees, buildings, poles, signs, cars (OWLv2, Apache-2.0) | Built 19 Nov: `pixi run label-drive` ([Labelling](labelling.md)) |
| Camera boxes onto lidar points, fused into `labels.yaml` | Built; Ravensim only. On the car: the camera-to-lidar calibration first (TODO 156) |
| A person fixes them | `pixi run map-edit` reads them (`source: camera`) |
| Our small model, retrained after each drive | Built: `pixi run label-train`, a point segmenter, ONNX |
| The score after each drive | Built: held-out frames; Ravensim's truth where it exists |

In Ravensim's campus (two lots): labels 73 % right, 50-77 % of what the lidar saw found.

> ⚠️ **Licences, if the work is ever commercial:** the YOLO option is AGPL-3.0; the big public
> datasets (SemanticKITTI, nuScenes, Cityscapes) are non-commercial. Train on our own drives.

### Phase G: the campus twin

**Goal:** Ravensim's campus matches the real one, so every test runs there first (TODO 253).

| From the labelled map | In Ravensim |
|---|---|
| Buildings | OpenStreetMap's outlines, heights from our lidar |
| Trees, poles, signs | at their measured places |
| Kerbs and roads | from our map |
| Parking areas | cars parked at random, to test "not on the map" |

`pixi run twin-world <map>` builds it; `pixi run sim world:=maps/twins/<name>/twin.sdf`
drives in it ([Labelling](labelling.md#the-twin-todo-253), built 19 Nov). Today: flat ground,
labelled boxes as buildings, trees and posts; parked cars and what moved left out. Not yet:
OpenStreetMap's outlines, kerbs, random parked cars.

**Passes when** a drive replayed in the twin scores within NDT's error of the real drive
(stage 2: not yet tried).

## The car driving itself

Only on an area that has passed phase D, after phases A and B.

| # | Step | Pass |
|---|---|---|
| 1 | The car started with NDT on that area's map, `pose_source:=ndt` (TODO 210, 261) | NDT within 0.2 m of the GPS, standing and pushed by hand |
| 2 | The same route **in Ravensim first** (the campus, then the twin) | the route done, no stop-zone surprises |
| 3 | On the car, **at walking pace** (0.5 m/s), a short route, a safety driver ready | done with no takeover needed for safety |
| 4 | Longer routes, then up to the shared 1.0 m/s limit | the same |
| 5 | Every drive recorded and scored ([the league](league.md)) | the scores tracked, not gates |

## The sessions, as cards

Copy a card, tick the boxes, put a photo in the team chat with the report folder's name.
Every card: **an e-stop person**, named before the start, hand on the button the whole time.

### Card 0: after the install

| Who | Where | How long |
|---|---|---|
| everyone; then the hardware lead and two more | laptops; then the car, wheels off the ground | 30 min a laptop; 1-2 h the car |

- [ ] Every box in [After the install](after-install.md) ✅
- **Feeds:** card 1.

### Card 1: the boot self-test (every start)

| Who | Where | How long |
|---|---|---|
| e-stop person, laptop person | wherever the car is | 5 min |

- [ ] `pixi run doctor`, then `pixi run check car`: every topic at its rate
- [ ] The HUD and the debug page (key D) up
- [ ] Press the e-stop, see it, release it
- [ ] `ros2 topic echo /safety/self_test/status`: **READY** (TODO 255; untested on the car). NOT READY: fix what it names first
- **Pass:** all ✅; else don't arm. **Save:** nothing. **Feeds:** every session.

### Card 2: stands (phase A)

| Who | Where | How long |
|---|---|---|
| hardware lead, e-stop person, laptop person, a rear-wheel watcher (tachometer app) | the shop, rear wheels off the ground | 2-3 h |

- [ ] Card 1 passed
- [ ] Steering's ends and centre: hardware panel B1-B9, **LOCK-TO-LOCK CHECK** PASSED
- [ ] `pixi run car-test --stands` (today with `--no-drive` until TODO 49b)
- [ ] Throttle by hand on the stands: speed up and slow down, the watcher reads the rear wheels
- [ ] Every fault on purpose stops the car
- **Pass:** `report.md` all PASS. **Save:** `~/aav-tests/<time>-stands/`, the steering file.
- **Feeds:** `vehicle.yaml`'s steering lines (pasted by a person); card 3.

### Card 3: map the hallway

| Who | Where | How long |
|---|---|---|
| two people to push, laptop person | the hallway, boxes and cones placed | 30 min, then 30 min offline |

- [ ] Tape the hallway; mark the start line; place the boxes and cones (photo them)
- [ ] `pixi run record-drive --check`, then `pixi run record-drive --name hallway`
- [ ] Push the car slowly down and back, twice
- [ ] Offline: `pixi run map-drive bags/<drive>`
- **Pass:** the map shows both walls and every box sharp. **Save:** the bag, the map.
- **Feeds:** NDT in cards 4 and 5 (once TODO 256 and 261 are built).

### Card 4: the hallway, empty (phase B)

| Who | Where | How long |
|---|---|---|
| e-stop person (walks beside), safety driver on the joystick, laptop person, a spotter at each end | the hallway, cleared | 2 h |

- [ ] Card 2 passed; the car weighed empty
- [ ] Start indoors: `pixi run car <machine> gps:=false stands:=true`
- [ ] `pixi run car-test --hallway --length <taped>`; three passes of each, turned round between
- [ ] The boxes still where card 3 put them
- **Pass:** the trusted numbers' model within 10 % speed and 0.2 m stop.
- **Save:** `~/aav-tests/<time>-hallway/`. **Feeds:** `vehicle.yaml` (pasted); the sim's car model.

### Card 5: the hallway, a person aboard

| Who | Where | How long |
|---|---|---|
| the same, plus the person aboard (weighed) | the hallway | 2 h |

- [ ] Card 4 passed the same day or with the boxes unmoved
- [ ] The person's weight written on the card (until TODO 257 records it)
- [ ] The same passes as card 4
- **Pass:** as card 4. **Save:** the session folder, the weights.
- **Feeds:** the loaded transfer function → the adaptive brake (TODO 258).

### Card 6: the lot

| Who | Where | How long |
|---|---|---|
| e-stop person, safety driver, laptop person, a spotter | an empty lot, the fence walked (site file) | half a day |

- [ ] Cards 4 and 5 passed
- [ ] `actuator-test coast`, then `speed-up`, `brake --speeds 0.5 1.0`, `circles`, `straight`
- **Pass:** as card 4, at speed. **Save:** each test's folder. **Feeds:** `vehicle.yaml`.

### Card 7: a campus mapping drive

| Who | Where | How long |
|---|---|---|
| driver, e-stop person, laptop person | one campus area | 30-60 min a pass |

- [ ] The SSD has room (`record-drive --check`: disk vs 87 GB/h)
- [ ] `pixi run record-drive --name <area>-<n> --note "<time, weather, parked cars>"`
- [ ] By hand, the same route as last pass, with a loop
- [ ] Read the summary: "good for" the map and SLAM
- **Pass:** no gaps, every topic at rate. **Save:** the bag, copied off the car the same day.
- **Feeds:** card 8. Repeat at least three times, on different days.

### Card 8: offline processing

| Who | Where | How long |
|---|---|---|
| anyone with a laptop (Linux x86_64 or an Apple-silicon Mac) | anywhere | hours, unattended |

- [ ] `pixi run map-area <area> bags/<drive 1> bags/<drive 2> bags/<drive 3>`: each
      drive's map, merged, parked cars out; read `maps/<area>/report.md` and each
      `drives/<drive>/report.md`
- [ ] `pixi run map-edit maps/<area>`: check the `seen_in` labels, then approve
- [ ] Later: labels (phase F)
- **Pass:** loop closures found or drift small; NDT on the map within 0.2 m of a good GPS.
- **Save:** `maps/<area>/`. **Feeds:** card 9; the campus twin.

### Card 9: the car drives itself

| Who | Where | How long |
|---|---|---|
| safety driver, e-stop person, laptop person, a spotter | a mapped area | half a day |

- [ ] Cards 1-8 passed for this area; the route run in Ravensim first
- [ ] NDT leads (`pose_source:=ndt`), checked standing still
- [ ] 0.5 m/s first; record every drive
- **Pass:** the route done, no takeover needed for safety. **Save:** the bag, the score.

## Safety rules for every phase

| Rule | Why |
|---|---|
| A named e-stop person at every test | the one sure stop |
| The boot self-test passed before anyone arms | "control over everything" |
| Wheels off the ground until phase A passes | nothing can run away |
| Never the `pi` brake loop on stands | it needs a real wheel speed (TODO 101) |
| Never test braking distance on stands | free wheels carry no weight |
| The hallway taped, cleared, a spotter at each end | people walk there |
| A person aboard only after the empty hallway passed, and only with a safety driver's stop in reach | the brake must be measured first |
| The 1.0 m/s shared limit until phase B's numbers are in | the stop distance is still a guess |
| Mapping drives by hand; self-driving only on a mapped area | NDT needs the map |
| Record every session | the report is the evidence |

## See also

- [After the install](after-install.md): phase 0, before the stands
- [Every test](tests.md): every test, step by step
- [Actuator tests](actuator-tests.md): the tests behind phases A and B
- [How the car stops](stopping.md): the adaptive brake
- [Recording a mapping drive](recording.md) and [Maps from drives](maps-from-drives.md)
- [Car day](car-day.md): the runbook for a day at the car
- [The safety case](safety-case.md): the hazards, and the tests behind each
