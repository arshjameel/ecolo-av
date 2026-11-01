# Start here

The software for Carleton's autonomous car: a ROS 2 Jazzy workspace, plus the firmware
for the microcontroller that drives the actuators. These pages explain how it fits
together, and how to run it.

## The whole stack in one picture

![How the stack fits together](img/architecture.svg)

Read it top to bottom: **sensors become telemetry, drivers become one command, and ego
moves the car.**

| Step | What happens |
|---|---|
| 1. Sense | Sensors publish raw data. Two filters (EKFs) work out where the car is. |
| 2. Propose | Each driver proposes a speed and a turn rate: joystick, keyboard, script or Nav2. |
| 3. Pick | `twist_mux` passes on the highest-priority driver. `/safety/estop` blocks them all. |
| 4. Convert | **ego**, the virtual car, turns that into a speed and steering angle the car can do. |
| 5. Move | One backend moves something: the real car, Gazebo, or a turtle on the screen. |

## New here?

| You are | Start with |
|---|---|
| after everything on one page | [Overview](overview.md) |
| after what this version does, at a glance | [Release notes](release-notes.md) |
| new to the team | [Getting started](getting-started.md) (30 minutes), then [Your first day](first-day.md) and your card in [Starting points](team.md) |
| making a change | [CONTRIBUTING.md](../CONTRIBUTING.md): fork, branch, pull request, merge queue |
| new to the car, or to ROS | [The car, box by box](learn-car.md), then [A command's journey](learn-journey.md); words in the [Glossary](glossary.md) |
| a professor, or anyone with a question | [Questions people ask](faq.md) |
| going to the car | [Car day](car-day.md): the runbook, session by session, and `pixi run car-day-check` |
| looking for what the car audit found (16 Oct) | [The hardware audit](audit-friday.md) |
| putting the whole thing together | [Infrastructure](infrastructure.md), then [Work](work.md) and [Decisions](decisions.md) |
| looking for what a package does | [Packages](packages.md); the deeper cards in [Every module](modules.md) |
| writing a driver, or drawing a map | [Build a driver](howto-driver.md), [Costmaps](costmaps.md) |
| testing the car's software without the car | [Bench mode](bench.md) |
| working on cameras, the detector, stop signs or lanes | [Perception](perception.md) |
| bringing in the previous teams' code | [The older code](older-code.md) |
| connecting Claude or another AI app | [AI apps (MCP)](mcp.md) |

## Learn the ideas, from zero

New to cars, robots or ROS? Each page explains one idea simply, then shows exactly how
our car uses it.

| Idea | Page |
|---|---|
| Sensors: every one on the car, and what it's good for | [Learn: sensors](learn-sensors.md) |
| GPS: where the car is on the earth, and RTK | [Learn: GPS](learn-gps.md) |
| Localization: odometry, Kalman filters, frames and TF | [Learn: localization](learn-localization.md) |
| Speed: how fast the car is going, from wheels, IMU, GPS and lidar | [Learn: speed](learn-speed.md) |
| Lidar: points, cubes, the costmap, named objects, NDT | [Learn: lidar](learn-lidar.md) |
| Control: feedback loops, PID, steering geometry, safe stops | [Learn: control](learn-control.md) |
| State space: from a drawing to a car simulator; poles, Bode, roll | [Learn: state space](learn-state-space.md) |
| Nav2: planning a path and steering along it | [Learn: Nav2](learn-nav2.md) |
| CAN: the wire for steering, brake and throttle | [Learn: CAN](learn-can.md) |

## Read these, in order

[[pages]]

## Try it in five minutes

The full hour, with rqt_graph, Gazebo and a Nav2 goal, is [Your first day](first-day.md).
On a Mac or on Linux, once [pixi is installed](run.md):

```bash
pixi run build       # compile our packages
pixi run turtle      # terminal 1: a turtle that steers like the car
pixi run keyboard --local    # terminal 2: hold i to drive, u or o to turn
```

## Where things live

| Folder | What's in it |
|---|---|
| `ros/` | ROS 2 packages, one folder per area of the picture |
| `firmware/` | microcontroller code: the CAN contract, the Rust spine, today's ESP32 |
| `tools/` | scripts: tests, keyboard, graph check, recording, these docs |
| `vendor/` | other people's source, to look things up. Not ours, and not in git |
| `docs/` | these pages. Edit a `.md` file, then run `pixi run docs`. |

## Where decisions live

| What | Where |
|---|---|
| The architecture, its decisions, and open questions Q1–Q34 | the team design doc (a Claude Doc); a short copy in [Decisions](decisions.md) |
| Every topic: its type, and who publishes and reads it | `docs/interfaces.yaml`, shown as [Topics](interfaces.md) |
| The CAN frames between the computer and the spine | `firmware/can/aav.dbc`, explained in [Firmware](firmware.md) |
| The car's measurements | `ros/bringup/aav_description/config/vehicle.yaml`: [Car numbers](car_measurements.md) |
| Rules for code, names and secrets | `CLAUDE.md` |
| How a change gets into `main` | [CONTRIBUTING.md](../CONTRIBUTING.md) |
