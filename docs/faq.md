# Questions people ask

Short, true answers to what a professor or a new teammate asks first. Each one links to
the page with the detail.

**You'll learn:**
- what works today, and what has never run on the car;
- how the car is kept safe, today and after the KEFC changes;
- how to work on the code without the car.

"Untested" on this page means the code exists but has never run. Words you may not know
are in the [Glossary](glossary.md).

## The project

### What is this?

The software for Carleton's Accessible Autonomous Vehicle (AAV), a capstone project.
This repo is the 2026-27 team's: one ROS 2 Jazzy workspace, plus the firmware for the
boards that move the actuators.

| Part | Where |
|---|---|
| ROS 2 packages | `ros/`, one folder per area of the picture |
| Firmware and the CAN contract | `firmware/` |
| Scripts | `tools/` |
| These pages | `docs/` |

### Does the car drive itself yet?

**No.** None of our code has run on the real car yet.

| Where | What runs |
|---|---|
| turtlesim | the command path, on a turtle that steers like the car |
| Gazebo | the whole stack: the EKFs, Nav2 driving to a goal |
| Bench mode | the car's own software, with stand-ins for every board |
| The real car | the 2025-26 team's code, on ROS 2 Humble |

Details: [Status](status.md), and [Sim and the car](sim-and-car.md).

### Why not run it on the car today?

| Blocker | What it means |
|---|---|
| The car runs Humble; we use Jazzy | the two can't share one network (Q24) |
| Our software isn't installed there yet | the audits (16 and 21 Oct) found what runs where: everything on Humble, ROS domain 0 |
| The drive path is the 2025-26 one | our `aav_legacy` backend speaks it, but it is untested |
| The CAN parts aren't bought yet | the KEFC request |

### Why ROS 2 Jazzy and not Humble?

Humble's support ends in May 2027, Jazzy's in May 2029. ROS can't mix versions on one
network, so every computer moves together. Decided by the team (Decision 6 in
[Decisions](decisions.md)).

### Why not just keep the 2025-26 team's code?

Most of their own estimator, planner and command picker was where the bugs were. Stock
ROS packages now do those jobs:

| Their code | Ours now |
|---|---|
| their odometry and SLAM | `robot_localization`: two EKFs |
| their grid planner (A*, D* Lite) | Nav2 |
| the QNX command picker | `twist_mux` |
| their lidar costmap and e-stop | Nav2's costmaps and `collision_monitor` |

We kept what works: the ESP32 firmware, the wheel encoder, the IMU and radar drivers,
lane keeping and the HUD. Each keeps a `PROVENANCE.md`. See [The older code](older-code.md).

## Safety

### What stops the car if the software crashes?

Layers, from the person outward. Each one works even if the ones after it fail.

| # | Layer | Stops the car when |
|---|---|---|
| 1 | Kar-Tech wireless e-stop, red button, RC override | a person presses it: hardware, beats all software |
| 2 | Joystick deadman (priority 100) | held with the stick centred |
| 3 | `/safety/estop` | true: twist_mux blocks every driver, the backends brake |
| 4 | ego's timeout | no command for 0.3 s |
| 5 | `spine_bridge` | ego goes quiet: it stops sending drive frames |
| 6 | The board's watchdog | no good CAN frame for 100 ms (DBC v0.2.0) |

!!! warning "Today's car has no layer 6"
    The 2025-26 ESP32 firmware has no timeout. If the Motor-Pi dies, the last throttle
    stays on. Wheels off the ground, e-stop in hand: see [Firmware](firmware.md).

### How do I press the e-stop from a laptop?

`pixi run estop` publishes `true` on `/safety/estop`. `pixi run clear estop` clears it.
The HUD's button only ever sets it: clearing is always a person's deliberate act.

### Who wins when two drivers talk at once?

The highest priority that spoke in the last 0.5 s.

| Driver | Priority |
|---|---|
| Joystick | 100 |
| RC transmitter | 95 |
| Keyboard | 90 |
| Actuator tests | 60 |
| Scripted | 50 |
| Replay | 40 |
| Go-to (one GPS point) | 25 |
| Nav2 | 20 |
| AI | 10 |

A lock holds every driver below its priority. A stop sign (30) and the sensor watchdog
(30) hold go-to, Nav2 and the AI, never a person or a test. The no-timeout lock (60)
holds every program while the car's ESP32 runs firmware with no command timeout
(`frame_version: 1`): only people and the actuator tests drive then. The e-stop (255)
holds everyone. The table lives in `ros/command_mux/aav_command_mux/config/twist_mux.yaml`.

### What happens when I let go of the joystick?

Nothing is sent for 0.5 s, then twist_mux hands control to the next driver that is
talking, such as Nav2. Whether it should stay stopped instead is open (Q28).

## The hardware

### Why move from the Motor-Pi to CAN?

| | Today: Motor-Pi, UART | After KEFC: CAN |
|---|---|---|
| Boxes in the path | Orin → Motor-Pi → ESP32 | Orin → ESP32 |
| A frozen computer | the last throttle stays on | the board brakes |
| A garbled message | a checksum only | a counter and a CRC on every frame |
| Wheel speed | a Pico 2 over UDP | two wheel nodes on the same bus |

The plan: [CAN plan](can-plan.md) (Proposed).

### Which board will run the CAN firmware?

**Still open** (Q10–Q13). The KEFC request uses the ESP32 control board already on the
car. The team's Tentative choice was a new Pico 2 or STM32, and a Pico 2 program exists
(untested on a chip). The [CAN plan](can-plan.md) proposes the ESP32.

### Why two Orins?

The KEFC request splits the work. Orin #1 keeps control (the EKFs, twist_mux, ego, the
CAN master); Orin #2 (new) does the heavy seeing: cameras, lidar, costmaps and Nav2. A
slow camera frame then can't delay a brake command. See [The car, box by box](learn-car.md).

### Is there a lidar?

Yes: a Velodyne VLP-16 on the roof (the 16 Oct audit). We run the stock ROS `velodyne`
driver for it, no code of our own: `pixi run car orin` starts it on the Jetson, where the
lidar sends its data ([Perception](perception.md#the-roof-lidar)). Gazebo has a generic
one too: `pixi run sim world:=road` ([Simulation](simulation.md#the-lidar-gazebo-only)).

### How does the car know where it is?

Two EKFs from `robot_localization`:

| | `ekf_local` | `ekf_global` |
|---|---|---|
| Uses | wheel speed, IMU | wheel speed, IMU, GPS |
| Gives | smooth, but drifts | true, but may jump |

GPS meets the map at one point, the datum: today a placeholder near Carleton. See
`maps-and-position.md` and [Topic areas](naming.md#why-the-two-ekfs-dont-talk).

### Does Nav2 follow the roads?

Not by itself. `street_router` (`aav_routes`) asks Valhalla, an offline router on
OpenStreetMap, for the way along the streets, and gives Nav2 that route as GPS points.
Nav2 doesn't know where the road is between them. Staying in the lane is lane keeping's
job. See [Street routes](routes.md).

## Working on it

### Do I need the car to work on the code?

No. Almost everything runs on one laptop:

| Run | You get |
|---|---|
| `pixi run turtle` | the command path, on a turtle |
| `pixi run sim` | Gazebo: the car, its IMU, GPS and wheel speed; `world:=road` adds its camera and radar |
| `pixi run bench` | the car's real software, with pretend boards |

### Can I use a Mac?

Yes. Pixi installs the same ROS 2 Jazzy on macOS and on Linux. See [Your Mac](mac.md).

### How do I know I didn't break anything?

| Run | Checks | Time |
|---|---|---|
| `tools/test.sh` | the unit tests, the Rust tests, the topic list | about a minute |
| `pixi run test` | all of that, plus the build, the style check and `colcon test` | a few minutes |
| `pixi run bench-check` | the whole car graph: driving, both e-stops | about 2 minutes |
| `pixi run check turtle` | the running graph against `docs/interfaces.yaml` | seconds |

CI runs `pixi run test` on every pull request, in the merge queue, and on each push to
`main`.

### Where is the list of every topic?

`docs/interfaces.yaml` is the contract: every topic, its type, who publishes and who
reads. It is shown as [Topics](interfaces.md). Change it first, then the code.

### Why are topic names like `/drivers/joy/cmd_vel`?

Each name starts with its area: `/sensors/`, `/drivers/`, `/control/`, `/ego/`,
`/safety/` and so on. You can tell where a message comes from by its name. Decided by
Ben on 15 Oct: [Topic areas](naming.md).

### Why Python for the nodes and Rust for the firmware?

| Language | For | Why |
|---|---|---|
| Python | the ROS nodes we write | our nodes run at 50 Hz or less; easy to read and test |
| Rust | firmware and hardware drivers | the compiler catches memory bugs before they reach the car (Ben, 10 Oct) |
| none of our own C++ | | the inherited C++ moves to Rust |

See [Languages](languages.md).

### How do I record a drive and play it back?

| Do | Run |
|---|---|
| Record | `pixi run tools/record.sh <name>`: a bag in `bags/` |
| Drive it again | `pixi run tools/ros.sh tools/replay_commands.sh <bag>`: priority 40 |
| Re-run the EKFs on it | the `replay` profile: see [Simulation](simulation.md) |

### Where are the open questions?

In the team design doc, numbered **Q1–Q34**; a short copy is in [Decisions](decisions.md).
Answer by number: "Q12: ...".

## Try it

```bash
pixi run turtle              # terminal 1: a turtle that steers like the car
pixi run keyboard --local    # terminal 2: hold i to drive
pixi run estop               # terminal 3: the turtle stops
pixi run clear estop         #             and drives again
```

## Where this lives in the code

| Question | Where |
|---|---|
| What works | `docs/status.md` |
| Decisions, Q1–Q34 | `docs/decisions.md` |
| Driver priorities | `ros/command_mux/aav_command_mux/config/twist_mux.yaml` |
| ego's limits and timeout | `ros/ego/aav_ego/config/` |
| Every topic | `docs/interfaces.yaml` |
| The CAN frames | `firmware/can/aav.dbc` |
| The car's measurements | `ros/bringup/aav_description/config/vehicle.yaml` |
| The rules for code and docs | `CLAUDE.md` |
