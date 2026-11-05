# Starting points: one per team member

Pick your area, and start with its card: what to read, what to run, your first tasks, the
files you'll work in, and the open questions you own. Everyone starts with
[Your first day](first-day.md). Owners aren't named yet: that's Q33.

## The areas at a glance

| Area | In one line | Phase it unblocks |
|---|---|---|
| [Integration](#integration-ben) | the whole stack, the infrastructure, the decisions | all of them |
| [Spine firmware and CAN](#spine-firmware-and-can) | the microcontroller that moves the actuators | 0 |
| [Today's car](#todays-car-the-legacy-path) | drive the car we have, with the 2025-26 firmware | 0, on today's hardware |
| [Ego and vehicle control](#ego-and-vehicle-control) | one command into speed and steering the car can do | 0 and 1a |
| [Localization and sensors](#localization-and-sensors) | where the car is: IMU, GPS, wheels, two EKFs | 1b–2 |
| [Navigation and costmaps](#navigation-and-costmaps) | Nav2: plan through a map and follow the path | 1b–2 and 3 |
| [Drivers and teleop](#drivers-and-teleop) | joystick, keyboard, scripted, and new drivers | 0 and 1a |
| [HUD and UI](#hud-and-ui) | the touchscreen and the remote views | later |
| [Simulation and testing](#simulation-and-testing) | Gazebo, replay, and the test ladder | every phase, before the car |

## Integration (Ben)

| | |
|---|---|
| Read first | [How it fits](how-it-fits.md), [Infrastructure](infrastructure.md), [Decisions](decisions.md), [Work](work.md) |
| Run first | everything in [Your first day](first-day.md), on a Mac and on Linux |
| First tasks | 1. A team meeting: accept or change each Proposed decision, and name an owner per area (Q33). 2. Secrets (Q34) and backups of the Orin and the Pis. 3. A plan for Jazzy on the car (Q24): Pixi on today's Ubuntu 22.04 first. 4. One `ROS_DOMAIN_ID` for everyone. (Done: `ecolo-av` is on GitHub, `AAV-2025-2026/ecolo-av`, public, with CI) |
| Files | `CLAUDE.md`, `pixi.toml`, `docs/interfaces.yaml`, `ros/bringup/aav_bringup/`, `tools/` |
| Open questions | Q9, Q23, Q24, Q32, Q33, Q34 |

## Spine firmware and CAN

| | |
|---|---|
| Read first | [Firmware](firmware.md), then `firmware/can/aav.dbc` and `firmware/spine/crates/spine-core` |
| Run first | `pixi run test`: its Rust part includes the spine logic's 84 tests |
| First tasks | 1. Choose the microcontroller and the board (Q10–Q13). 2. Write the board program: it implements `spine-board`'s `Board` trait; the drivers and glue are written. 3. Bench-test it with a USB-CAN adapter and `candump`: the watchdog, arming, limits. 4. Agree the frame map with ROS (`aav_spine`). Until then, [Bench mode](bench.md) runs `spine-core` on a laptop |
| Files | `firmware/can/`, `firmware/spine/`, `ros/can_bridge/aav_spine/` |
| Open questions | Q10–Q17, Q3, Q4, Q6, Q7 |

## Today's car: the legacy path

Phase 0 on the hardware we have now, before the spine exists.

| | |
|---|---|
| Read first | [Firmware](firmware.md) (the legacy section and its warnings), [Platforms](platforms.md) |
| Run first | `pixi run test`, then `pixi run car all` on a laptop: it comes up without hardware |
| First tasks | 1. Run Jazzy on the car's computers (Pixi, no reflash). 2. `pixi run car motor_pi` and `pixi run car orin`, with the wheels off the ground and the e-stop in hand. 3. The steering calibrations disagree (Q2): measure them |
| Files | `ros/can_bridge/aav_legacy/`, `firmware/legacy/`, `ros/bringup/aav_bringup/config/machines/` |
| Open questions | Q2, Q5, Q8, Q11 |

!!! warning "Today's ESP32 has no command timeout"
  If the bridge dies, the last throttle stays on. Wheels off the ground, and someone on
  the e-stop, until the spine's watchdog exists.

## Ego and vehicle control

| | |
|---|---|
| Read first | [How it fits](how-it-fits.md) (section 4, ego), [Car numbers](car_measurements.md) |
| Run first | `pixi run turtle` and `pixi run keyboard --local`; then `ros2 topic echo /ego/actuator_cmd` (inside `pixi shell`) prints what ego sends |
| First tasks | 1. Measure the car (Q1) and replace the guesses in `vehicle.yaml`. 2. Decide steering at standstill (Q27). 3. Agree with the spine owner what ego's output means (Q12): target speed, or raw throttle |
| Files | `ros/ego/aav_ego/` (`ackermann.py` is the maths), `ros/bringup/aav_description/config/vehicle.yaml` |
| Open questions | Q1, Q12, Q27 |

## Localization and sensors

| | |
|---|---|
| Read first | [How it fits](how-it-fits.md) (sections 1 and 2), [Simulation](simulation.md) (replay) |
| Run first | `pixi run sim`, then watch `/localization/odom/local` and `/localization/odom/global` in RViz |
| First tasks | 1. Identify the GPS (Q18) and pick its driver. 2. The IMU on the car (Q20); its magnetometer is in raw counts, not tesla. 3. Wheel encoder counts per turn (Q5). 4. Record a drive on the car, then tune the EKFs against it with replay |
| Files | `ros/localization/aav_localization/config/`, `ros/sources/` |
| Open questions | Q5, Q18, Q19, Q20, Q22 |

## Navigation and costmaps

| | |
|---|---|
| Read first | [Costmaps](costmaps.md) |
| Run first | `pixi run sim world:=test_lot`, then send a Nav2 goal from RViz |
| First tasks | 1. Draw your first map: copy `maps/example_lot.spec.yaml`, change a wall, run `tools/make_map.py`, and drive it in Gazebo. 2. The real test site (Q21): survey the datum, measure, draw. 3. Nav2's lookahead: its turn command flips sign about once a second in the sim (N7). 4. GPS waypoints for A→B |
| Files | `ros/drivers/aav_navigation/` (`config/nav2.yaml`, `maps/`), `tools/make_map.py`, `ros/localization/aav_localization/config/navsat.yaml` (the datum) |
| Open questions | Q21, and the Proposed Nav2 setup |

## Drivers and teleop

| | |
|---|---|
| Read first | [Build a driver](howto-driver.md), [How it fits](how-it-fits.md) (section 3) |
| Run first | `pixi run turtle teleop:=true` with a joystick plugged in: hold the trigger and steer |
| First tasks | 1. The first real joystick test (untested so far). 2. The e-stop button's message reaches the mux only on a good network (N2): make it robust. 3. Joystick handover (Q28). 4. The scripted drive on the car, phase 1a |
| Files | `ros/drivers/aav_teleop/`, `ros/drivers/aav_scripted/`, `ros/command_mux/aav_command_mux/config/twist_mux.yaml` |
| Open questions | Q7, Q27, Q28, Q30 |

## HUD and UI

| | |
|---|---|
| Read first | [HUD](hud.md) |
| Run first | Foxglove or Lichtblick on `ws://<the computer>:8765` while `pixi run sim` runs: see [Your Mac](mac.md) |
| First tasks | 1. Run the HUD and the map window: `pixi run hud --demo`, `pixi run map --demo`. 2. Try both on the car's screens ([The two screens](car-screens.md)). (Q29 answered: the raylib HUD replaced the Flutter app, archived 25 Oct) |
| Files | `ros/hud/aav_hud/` |
| Open questions | Q30 |

## Simulation and testing

| | |
|---|---|
| Read first | [Simulation](simulation.md) |
| Run first | `pixi run sim`, `pixi run check sim`, and the replay of a recorded drive |
| First tasks | 1. Gazebo's window and RViz on a Mac: confirm what they show. 2. The test lot world against the real site, once Q21 is answered. 3. Recordings from the car, for replay. 4. CARLA on the NVIDIA computer: the adapter, `aav_carla`, is ready ([CARLA](carla.md), Q31) |
| Files | `ros/sim/aav_gazebo/`, `ros/bringup/aav_bringup/launch/sim.launch.py` and `replay.launch.py`, `tools/check_graph.py` |
| Open questions | Q31, Q32 |
