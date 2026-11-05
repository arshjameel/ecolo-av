# Every test we've talked about

Written 15 October 2026, checked 25 October, for the team: every test from our talks, in one place,
in the order to run them. Each one says what it proves, how to run it, and when it passes.

> **The short version**
> - **Five stages**, safest first: the simulator, the bench, the car on stands, the car in an empty lot, then the whole drive.
> - Never skip a stage: a test only runs once the stage before it passes.
> - **Status:** ✅ exists and passes; 🔧 exists, not yet run on the car; ⬜ still to build.
> - On the car, always: a safety driver, the e-stop in someone's hand, and a speed cap.
> - **The first drive on the ground is recorded with the lidar** (4e): it gives the lot's maps and real scores for the trials.
> - **To learn the car** (its wheel size, speed-up, braking, drag, steering), the stage 3 and 4 tests in order, run again until the error is small: [Know the car](know-the-car.md).

```text
 1 simulator  →  2 bench  →  3 car on stands  →  4 car, empty lot  →  5 the whole drive
 (Gazebo,        (stand-ins,   (wheels off the      (walking pace,       (A to B by GPS)
  turtle)         real CAN)     ground)              safety driver)
```

## Stage 1: the simulator (Gazebo and the turtle)

Nothing real moves. Run these on any laptop.

| # | Test | Run | Passes when | Status |
|---|---|---|---|---|
| 1.1 | The whole car graph in Gazebo | `pixi run sim-check` | all checks PASS: sensors, a 6.5 m drive, the EKF within 0.5 m of the truth | ✅ 3 Nov: 9 of 9 (the EKF 0.15 m off); also 9 of 9 with each speed loop and `--voxels`, 10 of 10 `--kiss-ekf`, 7 of 7 `--no-lidar` |
| 1.2 | All three cameras send pictures | part of 1.1 | front left 10 Hz; front right and rear 5 Hz or more, each in its own frame | ✅ 15 Oct |
| 1.3 | The roof lidar | `pixi run sim-check` (on by default) | 1800 × 16 rays at 10 Hz in `lidar_link` | ✅ |
| 1.4 | Stop for an obstacle | `pixi run sim-check --obstacle` | stops 0-2 m before the box; drives on when it's gone | ✅ at 272 kg; ⚠️ at 181 kg, unconfirmed (the Mac was overloaded); ✅ 3 Nov: 0.23 m before it |
| 1.5 | Lane keeping | `pixi run sim-check --lane` | back to the lane's centre within 5 cm | ✅ 15 Oct; 3 Nov: within 5 cm after 5.4 s (its contract check fixed: it asked for stop_hold, which needs Nav2) |
| 1.6 | The HUD, with pretend data | `pixi run hud --demo` | the window opens with speed, camera, GPS, e-stop | ✅ Ben, 15 Oct |
| 1.7 | The HUD with the turtle | `pixi run turtle`, `pixi run hud`, `pixi run keyboard --local` | speed and steering move as you drive | ✅ Ben, 15 Oct |
| 1.8 | Plain-English driving | `pixi run -e llm talk`: "drive 10 m forward" | drives 10 m (±0.2 m) after you type yes | ✅ Gazebo |
| 1.9 | **"SIMULATED" on the HUD** | run Gazebo, look at the HUD | an orange SIMULATED banner; REAL / SIM tag per sensor | ✅ built 15 Oct (its logic tested); not yet seen on a screen |
| 1.10 | **The actuator tests (stage 3-4), in Gazebo first** | `pixi run actuator-test <test> --sim` ([docs](actuator-tests.md)) | the script runs end to end and reports g and angles | ✅ 15 Oct: all 9 tests; they give back `vehicle.yaml`'s numbers |
| 1.11 | **The lidar alone stops the car** | `pixi run sim-check --lidar-obstacle` (and `--plan-around`: Nav2 drives round a box it sees) | radar off: stops 0-2 m before a person-sized post, which shows in the local costmap; drives on when it's gone | ✅ 15 Oct: 0.16-0.55 m before it; 3 Nov: 0.16 m. ❌ with `--voxels`: 1 of 3 runs drove 1.30 m into the post (2 of 3 stopped 0.25 and 0.29 m before it) |
| 1.12 | **Sensor failures on purpose** | `pixi run sim-check --faults` (`--no-watchdog`: what happens without the watchdog) | each of 13 faults (GPS lost, NaN, jump; IMU stopped, frozen, NaN; wheels stopped, frozen, NaN; camera frozen; radar stopped; the watchdog hung, killed) stops the car within 6 s (frozen wheels: 10 s from when they go wrong), and says why; normal driving never trips the wheels' cross check | ✅ 15 Oct: all 13; 24 Oct, with the HUD's heartbeat: all 25 checks, 3 runs of 3; 3 Nov: all 25 (14 faults) |
| 1.13 | **The spine in the loop** | `pixi run sim-check --spine` | in Gazebo with `spine:=true`: the normal checks pass; ego's commands stopped → the spine brakes fully within 0.28 s and latches; `/safety/estop` and the hardware e-stop → brake within 0.15 s, latched until armed again; disarmed → no motion | ✅ 15 Oct: full brake 195-209 ms, e-stops 1-12 ms; 3 Nov: all PASS, full brake 202-205 ms, e-stops 2 ms |
| 1.14 | **Drive to a GPS point, smooth stop** | `pixi run sim-check --drive-to-point` ([A to B](a-to-b.md)) | every stop within 2 m of B by Gazebo's truth, no overshoot, braking about half `max_brake`; a cancel, the e-stop and a GPS fault stop it | ✅ 15 Oct: all within 0.4 m. ⚠️ 3 Nov, 2 runs: every stop within 0.46 m of B by the truth, but 1 of 9 drives each run ended 0.61 m past B by its own EKF (the limit 0.5 m): GPS noise, or the GPS's new position, to look at |
| 1.15 | **Ghost obstacles: a pretend person only Nav2 sees** | `pixi run sim-check --ghost` ([docs](ghost-obstacles.md)) | a ghost 15 m ahead (nothing in Gazebo): stops 0-2 m before it (the stop zone or the controller), the local costmap marks it; drives on once it's off | ✅ 16 Oct: 0.21 m before it, the stop zone fired (one run, radar and lidar on too); 3 Nov: 0.51 m before it |
| 1.16 | **Every screen in Initial D**, with pretend data | `pixi run hud --demo`, `map --demo`, `debug-hud --demo`, `hw-panel --demo` ([Themes](hud.md#themes-the-screens-colours)) | each opens black and red with thin white edges and sharp corners, no Catppuccin blue-grey; the e-stop red with white words; GO green, WAIT amber; the localizers keep their colours | ✅ 2 Nov: all four, screenshots on Ben's Mac |
| 1.17 | The theme's contrast test | `cargo test -p hud-core theme` (in `tools/test.sh`) | every word 4.5 : 1, every shape and mark 3 : 1 on each screen's backgrounds; Initial D has no Catppuccin colour but the localizers' | ✅ 2 Nov |
| 1.18 | **The diffusion planner's lot tour** | `pixi run measure controllers --planners lattice diffusion --reversing --tours lot` ([trials](trials.md#diffusion-2-nov-a-model-draws-the-lattice-guards)) | every leg, as the lattice; no plan tighter than full lock; the share of plans from the model (`/drivers/nav/diffusion/status`) | ✅ 2 Nov, 2 runs: 8 of 8 legs, 63 % from the model; 3 Nov, 3 runs: 12 of 12 legs, 73 % from the model, never tighter than full lock |
| 1.19 | **The diffusion planner's guard falls back** | `pixi run measure controllers --planners diffusion_nomodel --reversing --tours lot` (no model file); the guard's checks: `tools/test.sh` (`aav_diffusion_planner/test`) | every plan the lattice's, every leg; each check G1-G8 refuses its bad path in the unit tests | ✅ 2 Nov: 156 of 156 plans the lattice's, 4 of 4 legs; 3 Nov (one leg): 25 of 25 |
| 1.20 | **Every switch starts and drives a leg** | `pixi run measure controllers --tours lot --legs 1` with each switch ([switches](switches.md), [the runs](trials.md#3-nov-everything-for-02)) | each planner, follower, localizer and `voxels` starts, and the car finishes the first leg | ✅ 3 Nov: 23 of 23 (lattice32 and smac2d on a second try: the first, on a busy Mac, stalled) |

## Stage 2: the bench

The car's software path with stand-ins for the hardware, and the real CAN frames.

| # | Test | Run | Passes when | Status |
|---|---|---|---|---|
| 2.1 | The whole car graph, e-stops and the joystick-alive check included | `pixi run bench-check` | 11 of 11 | ✅ 3 Nov: 11 of 11 |
| 2.2 | The test suite | `pixi run test` | all green | ✅ |
| 2.3 | **Replay a real drive** through our EKFs | `ros2 bag play` a recording from the car | the EKFs' path matches the GPS track | ⬜ needs a recording from stage 3 or 4 |

## Stage 3: the car on stands (wheels off the ground)

**Before:** all four wheels off the ground; the e-stop in someone's hand; the 2025-26 drive
software stopped; speed capped (`max_speed:=0.3`).

### 3a. Does it obey? (history/friday-2026-10-16.md part 5)

| # | Test | Expect | Status |
|---|---|---|---|
| 3.1 | Nothing pressed | wheels still; the HUD shows 0 | 🔧 |
| 3.2 | Forward, gently | the drive wheels turn **forward**, slowly | 🔧 |
| 3.2b | **Backward, gently** (the keyboard's back key) | the drive wheels turn **backward**, slowly; `/vehicle/odom` reads **negative** (its sign is the command's until H5) | 🔧 |
| 3.3 | Left | the front wheels turn **left** | 🔧 |
| 3.4 | Let go of the key | slows to 0 within 0.3 s; brake frames sent | 🔧 |
| 3.5 | The HUD's e-stop button | brakes at once; nothing moves until `pixi run clear estop` | 🔧 |
| 3.6 | Stop the keyboard program | the bridge brakes: its commands go stale | 🔧 |
| 3.7 | Ctrl-C the Motor-Pi bridge | it sends a last stop frame | 🔧 |
| 3.8 | The Kar-Tech e-stop | everything stops | 🔧 |
| 3.9 | ⚠️ **Unplug the Motor-Pi's serial cable while driving** | the car brakes within 200 ms | ⬜ fails with today's C++ ESP32 (team-todo.md B2). The Rust ESP32 firmware brakes 111-120 ms after the last frame in QEMU; not yet flashed |

### 3a-joy. The joystick on stands {#3a-joy-the-joystick-on-stands}

Car day's [A9b](car-day.md#a9b-the-joystick-on-stands) (TODO 74a). The joystick-alive check (`joy_alive`) reports on `/drivers/joy/alive`. Each test: hold the
trigger, a little throttle, wheels turning, then:

| # | Test | Expect | Status |
|---|---|---|---|
| 3.J1 | **Unplug the receiver** | the wheels stop within 1 s; `/drivers/joy/alive` says `SILENT`. Plugged in again with the trigger held: still stopped, until it is released and pressed | 🔧 bench ✅ |
| 3.J2 | **Turn the pad off** (receiver still in) | the wheels stop within 3 s: `SILENT` if the receiver drops out or goes quiet, `FROZEN` if it repeats the last values. **Neither:** the receiver goes neutral (the wheels stop, the trigger reads 0): write it down | 🔧 |
| 3.J3 | **Hold the stick still**, 10 s, then pinned fully forward, 10 s | write down when `FROZEN` comes. Expected: a still hand rarely trips it (it jitters); pinned at the edge trips it after 2 s, and release-and-press drives again. Too often: raise `frozen_after` (`aav_teleop` `joy_alive.yaml`) | 🔧 |

### 3b. Steering, on stands

| # | Test | Measure | Passes when | Status |
|---|---|---|---|---|
| 3.0 | **The steering actuator's ends and centre**: before any sweep, its current limit low | left end, centre, right end in counts, and the wheel angle at each end (protractor). Way A (car day): Pololu's utility, the power off before any I2C is unplugged; way B (later, link version 3): the hardware panel, limp or jog ([the procedure and sheet](actuator-tests.md#find-the-steering-actuators-ends-and-centre)) | the sheet filled in; `steering_limits` gives equal limits each side; the hardware lead reviews them | ⬜ |
| 3.10 | **Steering sweep**: full left, centre, full right, slowly; **only inside the ends 3.0 measured** (or once they're flashed) | the wheel angle at each command (protractor or phone app on the wheel). `pixi run actuator-test steer-sweep --stands`: it asks for each angle | a table: command → real angle, each way | 🔧 |
| 3.11 | **Steering speed**: centre to full lock | seconds, from a video (or `actuator-test steer-rate --stands`, once a steering sensor reports) | written in `vehicle.yaml` | 🔧 |
| 3.12 | **Centre**: command straight | the wheels' angle; `actuator-test steer-centre --stands` | under 1°, or the offset written down | 🔧 |
| 3.13 | Wheel speed on stands | `/vehicle/status` speed vs a phone tachometer app | within 10% | ⬜ |

## Stage 4: the car in an empty lot

**Before:** stage 3 passed; the timeout fix (3.9) in; a safety driver; walking pace first.
The test script (`pixi run actuator-test <test>`, [docs](actuator-tests.md)) logs the IMU (g),
the wheel speed, the pose and the GPS for every test.
Which test fits which number, and when to stop repeating it: [Know the car](know-the-car.md).

### 4a. Speed up, coast, brake

| # | Test | How | Measure | Status |
|---|---|---|---|---|
| 4.1 | **Speed up** | from still, a fixed throttle (25%, 50%, full), 10 m. `actuator-test speed-up`: a speed command, so the speed loop picks the throttle | acceleration in **g** (IMU), and the time to 1, 2, 3 m/s | 🔧 |
| 4.2 | **Coast** | at 2 m/s, let go: no throttle, no brake. `actuator-test coast` (with the spine; not the ESP32) | how fast it slows: rolling resistance and drag | 🔧 |
| 4.3 | **Brake** | at 1, 2, 3 m/s, a fixed brake (50%, full). `actuator-test brake`: a normal stop, as no brake command exists | stopping distance and deceleration in **g** | 🔧 |
| 4.4 | **Throttle map** | repeat 4.1 at several throttles; `actuator-test throttle-map` | a table: throttle % → acceleration; and the deadband (the throttle where it starts to move) | 🔧 |

### 4b. Steering on the ground

| # | Test | How | Measure | Status |
|---|---|---|---|---|
| 4.5 | **Circles** | a fixed steering command at 1 m/s, a full circle each, both ways, several angles; `actuator-test circles` | the circle's size (GPS, or a tape), and yaw rate ÷ speed (IMU): the **real wheel angle** for each command | 🔧 |
| 4.6 | **Tightest turn** | full lock, both ways (the last pair of `actuator-test circles`) | the turning circle: checks `max_steering_angle` | 🔧 |
| 4.7 | **Straight line** | steering straight, 10 m at 0.5 m/s; `actuator-test straight` | sideways drift: under 0.5 m over 10 m | 🔧 |

### 4c. The car's weight (audit M7)

| # | Test | How | Gives | Status |
|---|---|---|---|---|
| 4.8 | **Scales** | 4 bathroom scales, one under each wheel, add them up | the mass, ±2%: the best way | ⬜ |
| 4.9 | **Speed up twice** | test 4.1 at full throttle, alone, then with a person of known weight (say 80 kg) | mass = 80 × a2 ÷ (a1 − a2), ±10-15%; and the **motor's force** | ⬜ |
| 4.10 | **Brake twice** | test 4.3 at full brake, alone, then with the same person | the mass again; and the **braking force** | ⬜ |

For 4.9 and 4.10: the same battery charge, the same spot and direction, three runs each,
and the coast-down (4.2) subtracted, so drag doesn't count as motor or brake.

### 4d. Sensors on the ground

| # | Test | Passes when | Status |
|---|---|---|---|
| 4.11 | GPS | `/sensors/gps/fix` at 5-10 Hz, a 3D (or RTK) fix on the HUD | ⬜ |
| 4.12 | IMU direction | turning left shows +z rotation; braking shows −x acceleration | ⬜ |
| 4.13 | Local EKF | drive 10 m straight: the EKF says 10 m (±0.3 m) | ⬜ |
| 4.14 | Global EKF + GPS | drive a loop: the EKF's track stays within 2 m of the GPS track | ⬜ |
| 4.15 | Cameras: the freeze | 1 hour with no freeze, power and heat logged (`history/friday-todo-2026-10-16.md` part 5) | ⬜ |
| 4.16 | Radar | `/perception/radar/clusters` shows a person walking in front | ⬜ |
| 4.17 | Lidar (the roof VLP-16, stock `velodyne` driver) | `car.launch.py` (lidar on by default): an obstacle shows in RViz's local costmap (`lidar_layer`), and a person in front stops Nav2 | ⬜ untested with the real lidar; ✅ in Gazebo (1.11) and against a pretend VLP-16 |
| 4.18 | **Sensor failures on the car** | pull the GPS's, the IMU's, the wheel sensors' CAN cable while Nav2 drives at walking pace: the HUD's red banner names it, and the car stops | ⬜ |

### 4e. Record the lot: the maps and the scores

The first drive on the ground is by hand, **recorded with the lidar**. At home, that
recording gives the maps, and real scores for every trial ([Trials](trials.md)).

**Before:** the lot's corner (latitude, longitude: a phone is close enough) in
`navsat.yaml`'s `datum`. Today it's a placeholder near Carleton. **Keep it the same for
every drive**, or the maps won't line up. The Orin needs 20 GB free (the lidar is about
5 GB per 10 min).

| # | Test | How | Gives | Status |
|---|---|---|---|---|
| 4.19 | **Record the lot** | the joystick, slowly: 2 laps, a figure 8, a stop by each cone, then 10 m by tape. `tools/record.sh --lidar lot` on the Orin | one recording with the lidar, GPS, IMU and wheels | 🔧 |
| 4.20 | **The wheel diameter** | from 4.19's taped 10 m: the wheels' distance against the tape's | `vehicle.yaml` `wheel_diameter` (TODO 60) | 🔧 |
| 4.21 | **NDT's map of the lot** | `pixi run ndt-map build bags/<4.19> --poses /localization/odom/global --out maps/ndt/<lot>` | `ndt:=true` on the car. Only as good as the EKF that placed it | 🔧 |
| 4.22 | **Score the trials** | `replay.launch.py` and `ros2 bag play bags/<4.19>`: `ndt_ekf`, `kiss_ekf`, `voxels`, on and off | the best of, decided on real data | 🔧 |

### 4f. Reversing and the U-turn

Nav2 may back up on the car (at 0.5 m/s at most), where the lidar watches behind. Until
the wheels tell direction (H5), the speed's sign is the **last command's**: 4.25 checks
where that goes wrong.

| # | Test | How | Passes when | Status |
|---|---|---|---|---|
| 4.23 | **Reverse straight** | the keyboard, 5 m back at 0.3 m/s, then `actuator-test straight` reversed | the local EKF goes **back** 5 m (±0.3 m); the GPS agrees | 🔧 |
| 4.24 | **Stop for something behind** | a cone 3 m behind, Nav2 told to back up past it | stops before it: collision_monitor's box behind the bumper (Gazebo: 0.75 m short, `sim-check --rear-obstacle`; 3 Nov: 0.78 m) | 🔧 |
| 4.25 | **Rolling back** (the known flaw) | on a gentle slope, drive forward, stop, let it roll back by hand on the brake | written down: how far the EKF thinks it went **forward**, and whether the watchdog noticed (it compares speeds, not directions: maybe not). Too far: wait for H5 before reversing on slopes | 🔧 |
| 4.26 | **The U-turn** | Nav2, a goal behind the car in a 14 m lane of cones (Gazebo's "turn back" leg) | a three-point turn: forward, back about 9 m, forward; no cone touched | 🔧 in Gazebo, 16 of 16 legs with the lattice (30 Oct) |

Nav2's map stays `empty.yaml`: the sensors see obstacles live. A map made from the lidar
needs a tool we don't have yet.

### The stack for every test on the ground

The car's defaults: the best of the Gazebo trials. `test_car_launch.py` pins them. The
trials wait for 4.22.

| Job | On the car | Why this one |
|---|---|---|
| Where am I | the two EKFs, with the GPS | NDT needs a map of the lot first (4.21) |
| Draws the route | the lattice (Hybrid-A* if forward only) | with reversing: every leg, 29 s quicker a tour |
| Steers along it | MPPI | reads the costmap; the trackers don't |
| Sees obstacles | the flat lidar scan | the cubes are a trial |
| Reverses? | yes, 0.5 m/s at most, where the lidar sees behind | the U-turn needs it (4.26); the sign is the command's until H5 |

## Stage 5: the whole drive

| # | Test | How | Passes when | Status |
|---|---|---|---|---|
| 5.1 | **Scripted drive** | 10 m at 0.5 m/s (`a-to-b.md`, test 5) | stops 10 m on (±0.5 m), straight | 🔧 in Gazebo, `sim-check`'s 6.5 m plan passes; not yet on the car |
| 5.2 | **Smooth stop at a GPS point** | `pixi run drive-to-point --lat .. --lon ..`: speed up, hold, slow down at **half the measured brake g** (`vehicle.yaml` `max_brake`, from 4.3), stop at B ([A to B](a-to-b.md)) | stops within 1 m of B (2 m without RTK) | 🔧 in Gazebo 15 Oct (`sim-check --drive-to-point`): 10-60 m, any angle, 1 and 3 m/s, every stop within 0.4 m of B, braking 1.4-1.6 m/s² for a plan of 1.5. 🔧 the car: needs 4.3 |
| 5.3 | Nav2 to a GPS point | the GPS goal, empty lot | reaches B, stops | 🔧 in Gazebo, goals of 8 m and 9.5 m; not yet on the car |
| 5.4 | **Ghost obstacles** | the real car with a **simulated** person in Nav2's costmap | slows and stops for it, with nothing real to hit | ⬜ |
| 5.5 | Plain-English driving on the car | `AAV_DISCOVERY=SUBNET pixi run -e llm talk --car`: "drive 10 m forward" | the same as 5.1, after you type yes | ⬜ last |

## Checks at the car (not driving)

| # | Check | Run | Status |
|---|---|---|---|
| F.1 | Every machine on the network, with its ROS domain | `pixi run bash tools/net_map.sh` | ✅ at home |
| F.2 | The Mac can join ROS | `pixi run doctor` | ✅ at home |
| F.3 | Every machine surveyed: what runs, what's plugged in, what exists only there | `pixi run bash tools/survey_car.sh ~/aav-audit/hosts.txt` | ✅ Linux CI; 🔧 the car |
| F.4 | Multicast between the car's machines, wired and WiFi | `ros2 multicast send` / `receive` | 🔧 |
| F.5 | Clocks within a second | `date` on each | 🔧 |
| F.6 | The camera freeze: the 10-minute watch | `history/friday-todo-2026-10-16.md` part 5 | 🔧 |
| F.7 | **Every screen opens in Initial D on the car's monitors** (HUD, map, debug screen, hardware panel) | `tools/car_screen.sh` at boot, or `pixi run hud` / `map` / `debug-hud` / `hw-panel`: black, white and red, no Catppuccin | 🔧 |
| F.8 | **The screens read in daylight and at night** | on the car, in sun and after dark: every word, the speed, GO / WAIT / STOP and the E-STOP read from the driver's seat; if not, `theme: latte` for day ([Themes](hud.md#themes-the-screens-colours)) | ⬜ |

## What the tests give back

```text
 4.1 speed up  ─┐
 4.3 brake     ─┼─► vehicle.yaml: acceleration, braking g, steering map, mass
 4.5 circles   ─┤        │
 4.8 scales    ─┘        ├─► Gazebo behaves like the real car (tests 1.x mean more)
                         └─► the smooth stop (5.2): it knows how hard it can brake
```

## Still to build for these tests

| What | For | Size |
|---|---|---|
| Ghost obstacles on the real car | 5.4 | 1 day |
| The ESP32 timeout on the car: flash the Rust firmware (built, proven in QEMU) | 3.9, and everything after | team-todo.md B2 |
