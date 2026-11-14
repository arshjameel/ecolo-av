# Rolling out on the car

How our code goes onto the car: the names, the order, and each computer's steps.
The full steps, with sign-offs: [Car day](car-day.md), [Car setup](car-setup.md) and
[The ESP32 on the bench](esp32-bench.md). **Back up first, every time.**

!!! note "Where the car is (checked 14 Nov)"
    Written 8 Nov, for Monday 9 Nov. **The repo has no record of that day.** By the
    records, only the **localization Pi** has our kit (by 11 Nov), and the car drove on
    13 Nov with only that update. The ESP32 (the C++, seen 6 Nov), the Motor Pi and the
    Orin still run the 2025-26 software: steps 1-5 below are still to do.

## The names

One name per computer: its **role** (what `car_setup.sh` and the docs call it) is also its
planned **hostname**, with `aav-` in front. `car_setup.sh` accepts the old hostname and
the new one, so renaming can wait. Renaming is the hardware lead's call.

| Computer | Role | Planned hostname | Today's hostname | Old nickname | Keychain login |
|---|---|---|---|---|---|
| Jetson AGX Orin | `orin` | `aav-orin` | `ubuntu` | JETSON | `aav-jetson` |
| Pi wired to the ESP32 | `motor_pi` | `aav-motor` | `ubuntu-cortex` | CORTEX | `aav-cortex` |
| IMU and GPS Pi | `localization` | `aav-localization` | `localhost.localdomain` | LOCALIZATION | `aav-localization` |
| Dashboard Pi (two screens) | `dash` | `aav-dash` | `aav-desktop` | | `aav-desktop` |
| Camera Pis | `camera_pi` | `aav-cam1`, `aav-cam2` | the same | CAM1-PI, CAM2-PI | `aav-cam1`, `aav-cam2` |
| ESP32 control board | (firmware) | `aav-esp32`, only if its WiFi page is on | | | |
| Velodyne lidar | (no computer) | `aav-lidar`, its name on the router | | | |

The Keychain also has `aav-qnxpi`: a Pi whose job we don't know. Ask the hardware lead.

To rename one, after its backup: `sudo hostnamectl set-hostname aav-motor`, change the old
name in `/etc/hosts`, reboot. Then update the Keychain entry's name and `hosts.txt`.

## What runs where

| Computer | Plugged in | Runs | Machine file |
|---|---|---|---|
| Orin | the lidar (network), CAN | the brain: ego, Nav2, localization, perception | `machines/orin.yaml` |
| Motor Pi | the ESP32 (UART) | the bridge to the ESP32 | `machines/motor_pi.yaml`; alone: `motor_pi_alone.yaml` |
| Localization Pi | the IMU, the GPS (USB) | their drivers only; the Orin does the maths | `machines/localization.yaml` |
| Camera Pis | the front cameras | video streams, no ROS (Pi OS) | none |
| Dashboard Pi | the two screens | the HUD and the map | none |

- **The ESP32 has no YAML.** Its settings (steering limits, brake positions, speed limits,
  timeouts) are built into its firmware; its `CONFIG` line prints them all at boot.
- **The lidar** sends its packets straight to the Orin.

## The order

The plan for Monday 9 Nov, still the order: **the ESP32 and the Motor Pi first.** One step at a time; each has a way back.

| # | Step | Where it's written | The way back |
|---|---|---|---|
| 0 | At home: `pixi run car-day-check` all OK; the car kit on a USB stick | [A0](car-day.md#session-0-before-leaving-at-home) | |
| 1 | Back up: an image of the Motor Pi's SD card; the ESP32's flash (below) | [A2](car-day.md#a2-back-up-every-machine) | |
| 2 | The Motor Pi: our stack from the kit (below) | [A5 4e](car-day.md#4e-the-motor-pi) | the SD image |
| 3 | The Motor Pi's UART ready: console off, theirs off. **Needs Ben's OK** | [A7b](car-day.md#a7b-the-motor-pis-uart-ready) | its table's last column |
| 4 | The ESP32: back up, flash the Rust, the first check, the checks on stands | [A8](car-day.md#session-6-the-esp32), `pixi run esp walk` | flash the backup |
| 5 | Drive on stands, the joystick or the keyboard, on the Motor Pi alone | [A9b](car-day.md#a9b-the-joystick-on-stands) | |
| 6 | Later: the Orin, the localization Pi, the cameras, the dashboard | below | each one's image |

**Without the Orin** (steps 4-5): the Motor Pi runs everything,
`~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi_alone max_speed:=0.3`.
Its output mux sends the heartbeat the ESP32 needs. Never run the Orin's stack at the same time.

## What 0.3 changes on the car

The kit now holds 0.3: everything from the simulator work too. **On the car, the defaults
stay as they were.** The new pieces are off, or run only in the simulator.

| Piece | On the car | How to tell |
|---|---|---|
| ESP32 firmware | Same behaviour (one line changed, for the car model) | `CONFIG` line at boot |
| Motor Pi setup | No change | `car_setup.sh motor_pi check` |
| Ego (Orin) | **New:** the lag lead, the 2 s trace ahead, the stop point | `/ego/trace`, `/ego/stop_point` |
| Nav2 | Still the default: costmap, planner, navigator, smoother | |
| Our costmap, planner, navigator, smoother, MPPI with lag, MPC with lag | Simulator switches only | [Switches](switches.md) |
| Diffusion planner | **Locked off.** `diffusion_trial:=true` turns it on, capped at 0.5 m/s | [A12](car-day.md#a12-the-diffusion-planners-trial) |
| Adaptive brake | Refused with the ESP32 (the firmware brakes). Needs A7's brake limits first | [Stopping](stopping.md) |

**Test ego on stands first** (A9b), before the car drives on the ground with the Orin.

## The ESP32 (a USB cable, from a laptop with the repo)

```sh
pixi run esp tools                       # once per laptop
vendor/cargo-tools/bin/espflash read-flash 0 0x400000 backup-esp32.bin
                                         # the backup of the C++ (4 MB): a secret inside, never in git
pixi run esp walk                        # talks you through: the flash, the first check
```

- What goes on the board: `car-day/esp32/spine-esp32-umod-v2.elf`, made by
  `pixi run car-day-build` (version 2 frames, no uplink: Day A's choice).
- The way back to the C++: `vendor/cargo-tools/bin/espflash write-bin 0 backup-esp32.bin`,
  and `frame_version: 1` again on the Motor Pi.

## A Pi with Ubuntu (`motor_pi`, `localization`, `dash`)

```sh
# an image of the SD card first; the kit copied over: scp -r car-kit <user>@<address>:~/
bash ~/car-kit/car_setup.sh <role>                 # a dry run: read what it will do
bash ~/car-kit/car_setup.sh <role> --apply         # do it: backup, upgrade, install, services, check
```

**The Motor Pi, the first time,** takes only three steps:
`bash ~/car-kit/car_setup.sh motor_pi backup install check --apply`.

- No `upgrade`: it keeps Ubuntu 22.04 until the Rust ESP32 has passed car day's session 6
  (`--after-esp32`). Its old bridge is the way back to the C++.
- No `services` yet: it refuses while the old bridge starts at boot. After A7b, start ours
  by hand (A7b step 6); `services --apply` once the team agrees.

## A Pi with Pi OS (the camera Pis: they stay on Pi OS)

```sh
# an image of the SD card first
bash car_setup.sh camera_pi upgrade --apply
bash car_setup.sh camera_pi services --apply
```

## The Orin

```sh
# a full image first (NVIDIA's backup, from a host PC): car-day.md B3
bash car_setup.sh orin                   # a dry run
bash car_setup.sh orin --apply
```

Later (Day B): the reflash to JetPack 7.2, then `car_setup.sh orin` again. Not yet run on
the real Orin.

## Upgrading later: a newer version on every computer

The same steps for every new version. Each computer keeps its old version until the new
one passes its check.

**At home, on a Mac with the repo:**

```sh
git pull                                  # the new main
pixi run test                             # all green first
pixi run car-day-build                    # the ESP32 firmware, into car-day/esp32/
pixi run car-kit                          # the install kit, into car-kit/
pixi run car-day-check                    # everything OK?
```

Copy `car-kit/` and `car-day/esp32/` to the USB stick.

**On the car:**

| Computer | Back up | Upgrade | Check | The way back |
|---|---|---|---|---|
| ESP32 | `espflash read-flash 0 0x400000 backup-esp32.bin` | `pixi run esp walk` (flashes the new `.elf`) | the first check, then A8's checks on stands | `espflash write-bin 0 backup-esp32.bin` |
| Pi on Ubuntu (`motor_pi`, `localization`, `dash`) | an SD card image | `bash ~/car-kit/car_setup.sh <role> install check --apply` | `car_setup.sh <role> check` | the SD image, or the old kit's `install` again |
| Pi on Pi OS (cameras) | an SD card image | `bash car_setup.sh camera_pi upgrade services --apply` | the video stream opens | the SD image |
| Orin | NVIDIA's full image (B3) | `bash ~/car-kit/car_setup.sh orin install check --apply` | `~/aav-jazzy/aav.sh car orin`, then stands (A9b) | the image |

- **Copy the kit first** on each Pi and the Orin: `scp -r car-kit <user>@<address>:~/`
  (the logins are in the Keychain: `aav-*`).
- `install` run again with a newer kit updates the code. It rebuilds the environment only if
  it changed. Everything lives in `~/aav-jazzy`, so nothing else on the computer changes.
- **One computer at a time**, in the order above: ESP32 and Motor Pi, then the Orin, then
  the rest.
- After the Orin, restart its service (`sudo systemctl restart aav-car`) or reboot.
