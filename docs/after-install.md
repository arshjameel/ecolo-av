# After the install: is everything working?

Ben, 19 Nov: "we need a testing document after everybody installs stuff". The car is being
upgraded (the Pis to Ubuntu 24.04, the Orin to JetPack 7.2) and everyone installs our code.
This page is the checklist **after** that, **before** [phase A](testing-plan.md#phase-a-the-car-on-stands)
puts the car on stands. Nothing here moves the car.

> **The short version**
> 1. **Each laptop**: install, build, test, a sim drive.
> 2. **Each car computer**: `car_setup.sh <role> check`, its service, its clock, its devices.
> 3. **The car together**, wheels off the ground, e-stop in hand: every topic, every screen,
>    the e-stop.
> 4. All ✅ → [phase A](testing-plan.md#phase-a-the-car-on-stands): `pixi run car-test --stands`.

How to install: laptops [Your first day](first-day.md); car computers [Car setup](car-setup.md)
and [From a USB stick](car-install.md).

## The order

```text
 1. laptops (anyone, any time)
 2. each car computer, alone      Orin ─ Motor Pi ─ localization Pi ─ dash Pi ─ camera Pis
 3. the car together              network ─ clocks ─ topics ─ screens ─ e-stop
 4. phase A on stands             pixi run car-test --stands
```

Write a ✅ or ❌ in each box, your name and the date, and put the filled page (a photo is
fine) in the team chat. A ❌ stops here: fix it, or ask, before the next part.

## 1. Each laptop (about 30 minutes)

Who: everyone on the team. Mac or Linux.

| ☐ | Do | You should see |
|---|---|---|
| ☐ | `git pull` (on `main` of your fork), then `pixi install` | no errors |
| ☐ | `pixi run build` | `Summary: ... packages finished`, no `failed` |
| ☐ | `pixi run test` | `0 errors, 0 failures` (about 30-45 min the first time) |
| ☐ | `pixi run sim-check` | every line PASS: the contract, the sensors, a drive |
| ☐ | `pixi run sim-check --faults` | 15 of 15: each fault stops the car |
| ☐ | `pixi run map --demo --debug-page` | the map window; press **D**: who is driving, what each asks |
| ☐ | `pixi run doctor` (on the car's WiFi) | OK lines; any FIX says what to change |

No GPU (a Linux laptop without one): put `tools/software_gl.sh` in front of `sim-check`.

## 2. Each car computer, alone

Who: the person who installed it, with the hardware lead. Over `ssh`. Logins from the team's
vault, typed, never pasted into a file or chat.

### On every car computer

| ☐ | Do | You should see |
|---|---|---|
| ☐ | `bash ~/car-kit/car_setup.sh <role> check` | the new release (Ubuntu 24.04; JetPack 7.2 on the Orin) and **no STOP** line |
| ☐ | `systemctl --user status aav-car` (not the camera Pis) | `active (running)` |
| ☐ | `journalctl --user -u aav-car -n 50` | no `ERROR`, no restarts in a loop |
| ☐ | `chronyc sources` | the Orin with `*` (the Orin itself: its own source) |
| ☐ | `~/aav-jazzy/aav.sh tools/car_listen.sh` | the devices it should have, each found (read-only: opens nothing) |
| ☐ | Reboot, wait 2 min, run `systemctl --user status aav-car` again | it came back by itself |

### And per computer

| Computer | ☐ | Also check |
|---|---|---|
| **Orin** | ☐ | `nvidia-smi` or `tegrastats`: the GPU is seen |
| | ☐ | The lidar (VLP-16, over Ethernet) answers: `ros2 topic hz /sensors/lidar/points` about 10 Hz |
| | ☐ | The TensorRT engine rebuilt for JetPack 7.2's TensorRT ([session 4d](car-day.md#4d-rf-detrs-tensorrt-engine)) |
| | ☐ | Untested: the ONNX models from Rust on TensorRT, the detector and the diffusion worker; delete the old engines in `models/*.cache/tensorrt/` first ([ONNX, the Orin](onnx.md#untested-the-orin)) |
| **Motor Pi** | ☐ | The ESP32's serial port is back (`check`'s `UART` line is OK) |
| | ☐ | The 2025-26 `aav_serial_bridge` is **not** running beside ours (never two drive paths) |
| **Localization Pi** | ☐ | The IMU and the GPS: `/dev/aav/imu`, `/dev/aav/gps` |
| **Dash Pi** | ☐ | Both screens come up at boot: `aav-hud` and `aav-map` |
| **Camera Pis** | ☐ | MediaMTX running; the stream opens on a laptop |

## 3. The car together

Who: three people. **The e-stop person** (hand on the button the whole time), the person at
the laptop, the hardware lead. The car on stands or with its wheels blocked; the drive
motor's power off until the last row.

| ☐ | Do (on a laptop on the car's WiFi) | You should see |
|---|---|---|
| ☐ | `pixi run doctor` | it finds the car (`/ego`, `/twist_mux`); domain 42 |
| ☐ | `pixi run check car` | every topic in the contract, at its rate; nothing missing |
| ☐ | `pixi run tools/ros.sh ros2 topic hz /sensors/lidar/points` (and the IMU, GPS, wheels, each camera) | about the rate in [Interfaces](interfaces.md) |
| ☐ | Look at the HUD and the map screen on the dash | the car's speed, the GPS fix, the map; press **D** on the map: the debug page |
| ☐ | Press the **e-stop** | the HUD says E-STOP; the debug page's gate says why; `/safety/estop` true |
| ☐ | Release it, then the joystick: steer left and right only (drive power off) | the wheels follow, positive = left |
| ☐ | Unplug one sensor (the hardware lead says which), then plug it back | the watchdog says STALE for it within 1 s, the gate stops; it clears when back |
| ☐ | `pixi run record-drive --check` | every check OK: disk space, rates, clocks, sensor frames |

## 4. Then phase A

All ✅? The car goes on stands: `pixi run car-test --stands`, one command, one report
([phase A](testing-plan.md#phase-a-the-car-on-stands)). It repeats the sensor checks above and
adds steering, throttle on free wheels and faults on purpose, losing the Orin among them.

## When something fails

| Symptom | Try |
|---|---|
| `check` says STOP | its line says why; [Car setup: when it stops](car-setup.md#when-it-stops) |
| `aav-car` restarts in a loop | `journalctl --user -u aav-car -n 200`: the first error; often a device name missing (`udev`) |
| A laptop can't see the car | `pixi run doctor`; the same `ROS_DOMAIN_ID` (42 for the car) and the car's WiFi |
| A topic is missing in `check car` | the computer that publishes it ([the machines](car-day.md#the-machines)): its service and its log |
| The clocks disagree | `chronyc sources` on each; the Orin serves the time |
| A test fails on a laptop | run it again; still failing: the last 20 lines into the team chat or Claude |

## What's been tested

| Part | Status |
|---|---|
| Part 1 (laptops) | [[works]] on Ben's Mac, 19 Nov (`pixi run test`, `sim-check --faults`) |
| Parts 2 and 3 | [[untested]]: written for the upgraded car; each command's own page says what was tried |
