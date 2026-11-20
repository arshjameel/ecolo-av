# Flash the ESP32, then upgrade the Motor Pi

Senura's runbook (17 Nov), checked against the repo the same night by Ben's session.
One page, in order. Each step links to the car-day section that has the details.
The night before, the whole Motor Pi path ran on a Mac against the ESP32 in QEMU:
[the rehearsal](rehearsal-2026-11-18.md) (what passed, what didn't, the morning's checklist).

!!! danger "On stands, e-stops in hand"
    The ESP32 drives the brake, throttle, steering and direction. The Rust firmware has
    never been flashed on the car: only `cargo test` and QEMU so far
    ([ESP32 safety](esp32-safety.md)). Wheels off the ground, the motor and safety team
    there, the red button and the Kar-Tech stop **tested first**.

## The order

| # | Step | Where the details are | Gate |
|---|---|---|---|
| 0 | The Motor Pi is alive on the network | [Car day A5](car-day.md#session-4-the-orin-and-the-motor-pi) | `ping`; `nmap -sn 192.168.1.0/24` if its address moved |
| 1 | **Back up** the Motor Pi | [Car day A2](car-day.md#a2-back-up-every-machine) | an SD-card image, checked |
| 2 | **Our stack on the Motor Pi**, still on Ubuntu 22.04 (the car kit runs there too; CI checks it) | [Car day A5](car-day.md#session-4-the-orin-and-the-motor-pi), [Car setup step 4](car-setup.md#step-4-install-our-code) | `bash ~/car-kit/car_setup.sh motor_pi install --apply` ends with no STOP |
| 3 | **The Motor Pi's UART, ready** (console off it, theirs off at boot) | [Car day A7b](car-day.md#a7b-the-motor-pis-uart-ready) | L1-L5 show nobody on `/dev/ttyAMA0` |
| 4 | The firmware in the emulator, on a Mac | below | `pixi run esp qemu-check`: PASS |
| 5 | **Flash the Rust ESP32**, guided | [Car day A8](car-day.md#session-6-the-esp32) | `pixi run esp first-check`: SAFE TO GO ON |
| 6 | The on-stands checks (session 6): our bridge on the Pi makes the heartbeat | [Car day A8](car-day.md#session-6-the-esp32) | every answer yes; any no rolls back |
| 7 | Upgrade the Motor Pi to Ubuntu 24.04 | [Car day, Day B](car-day.md#day-b-upgrades-and-the-reflash), [Car setup step 2](car-setup.md#step-2-the-ubuntu-2204-pis-to-2404) | `car_setup.sh motor_pi check`: no STOP |

Steps 2 and 3 were missing from the first draft. Session 6 needs our bridge running on
the Pi (step 2), and without step 3 the Pi's kernel console and the 2025-26 bridge both
write to the ESP32's wire.

## Before you start

| Need | Note |
|---|---|
| A laptop with the repo and Pixi | Flashing and QEMU work from a Mac (QEMU is set up for Apple silicon) |
| The ESP32 tools | `pixi run esp tools`, once (several minutes, needs `rustup`). On Ben's Mac already: `espflash` 4.3.0 in `vendor/cargo-tools/bin` |
| A USB cable to the ESP32 | find its port: `ls /dev/cu.usb*` (Mac), `ls /dev/ttyUSB* /dev/ttyACM*` (Linux) |
| The logins | [Car day, "Logins"](car-day.md) (never copy a password anywhere) |
| The team agrees to retire the 2025-26 C++ path | needed before step 7, which is one-way |

## Steps 4-6: the ESP32

| Run | Does |
|---|---|
| `pixi run car-day-build` | builds the firmware into `car-day/esp32/` and runs the QEMU check (3-5 min). Do it the night before |
| `pixi run esp qemu-check` | boots it in QEMU: it brakes when the heartbeat stops |
| `pixi run esp walk <port>` | **the first flash, one step at a time**, shown and spoken: saves the C++ (the whole 4 MB flash), flashes the Rust, runs first-check, asks the on-stands questions, **rolls back to the C++ on any no**, and writes `bench-logs/<date>-esp32-first-flash.md`. `--dry-run` rehearses it |
| `pixi run esp first-check <port>` | reads the boot log for 30 s: SAFE TO GO ON or STOP |

A belt-and-braces backup before the walk (the walk does the same):

```
vendor/cargo-tools/bin/espflash read-flash --port <port> 0 0x400000 esp32-cpp-backup-$(date +%F).bin
```

Restore: `vendor/cargo-tools/bin/espflash write-bin --port <port> 0 <backup.bin>`.
Keep the `.bin` on the laptop and the shared drive.

## Step 7: the Motor Pi to 24.04

On the Motor Pi, from the car kit. Run each line without `--apply` first: it shows the plan.

| # | Run |
|---|---|
| 1 | `bash ~/car-kit/car_setup.sh motor_pi` (dry run of everything) |
| 2 | `bash ~/car-kit/car_setup.sh motor_pi backup --apply` (and the SD-card image, car day A2) |
| 3 | `bash ~/car-kit/car_setup.sh motor_pi upgrade --apply --after-esp32`, then `sudo reboot` |
| 4 | `bash ~/car-kit/car_setup.sh motor_pi upgrade --apply --after-esp32 --release-upgrade` (22.04 to 24.04, interactive: keep the **installed** config files; say yes to the spare ssh on port 1022), then `sudo reboot` |
| 5 | Build our code again on 24.04, from clean (as on the localization Pi): `rm -rf ~/aav-jazzy/aav/build ~/aav-jazzy/aav/install ~/aav-jazzy/aav/log`, then `bash ~/car-kit/car_setup.sh motor_pi install --apply` |
| 6 | `bash ~/car-kit/car_setup.sh motor_pi services --apply --machine motor_pi_alone`, then `sudo reboot` |
| 7 | `bash ~/car-kit/car_setup.sh motor_pi check`: Ubuntu 24.04, and the UART to the ESP32 is there |

The upgrade doesn't need our stack installed first: it was installed in step 2 for
session 6 anyway. `services` refuses while the 2025-26 `aav_serial_bridge` is enabled
(step 3 disabled it).

| Check first | Why |
|---|---|
| Free disk: `bash ~/car-kit/car_install.sh --dry-run` | the localization Pi needed a 64 GB card |
| `motor_pi_alone` or `motor_pi` | alone: the Pi runs the mux, ego and the bridge, so the heartbeat is made beside the ESP32 and no Orin is needed. `motor_pi`: the Orin runs the mux and ego. Decide with the team |
| `--after-esp32` | the upgrade refuses without it: step 6 must have passed |

## After: what to check

| Check | Expect |
|---|---|
| `~/aav-jazzy/aav.sh ros2 topic hz /control/car/link_frame` | about 10 Hz: the bridge writes frames |
| `~/aav-jazzy/aav.sh ros2 topic echo /safety/output_mux/state --once` | the mux's state |
| A drive on stands: the Pi started with `max_speed:=0.3`, then `pixi run keyboard` on the laptop (same `ROS_DOMAIN_ID`, 42) | the wheels turn; let go and they stop |

!!! note "`/vehicle/odom` is not the Motor Pi's"
    The wheel sensors are an Arduino on the **Orin's** CAN bus (`aav.dbc` 0x200, 0x201);
    the Orin's spine bridge publishes `/vehicle/odom`. The Motor Pi's old Pico encoder is
    off (`legacy_encoder: false`). So with only the ESP32 and the Motor Pi, there is no
    `/vehicle/odom`: check it on the Orin's day, with the wheel Arduino wired.

## If the Rust nodes misbehave

Since 17 Nov the Motor Pi's nodes run in Rust by default. Each has the Python as a
fall-back, one switch away ([0.4 on the car](release-0.4-rc1.md)):

| Node | Fall-back |
|---|---|
| The muxes (output and sensor) | `~/aav-jazzy/aav.sh car motor_pi_alone -- io_mux_impl:=python` |
| The ESP32's UART bridge | `~/aav-jazzy/aav.sh car motor_pi_alone -- legacy_impl:=python` |
| The RC driver | `~/aav-jazzy/aav.sh car motor_pi_alone -- rc_impl:=python` |
| The joystick's check (joy_alive) | none since 20 Nov: the Python was removed (`save/python-safety-nodes`) |
| The stop-sign watcher | `~/aav-jazzy/aav.sh car motor_pi_alone -- stop_sign_impl:=python` |
| The point driver (on the Orin, not the Motor Pi) | `~/aav-jazzy/aav.sh car orin -- drive_to_point_impl:=python` |
| The others (ego, the watchdog...) | the list in [0.4 on the car](release-0.4-rc1.md) |

## Rollback

| What | How |
|---|---|
| The ESP32 | the walk rolls back by itself on a no; otherwise `espflash write-bin` the backup |
| The Motor Pi's OS | the release upgrade is not easily undone: restore the SD-card image from step 1 |
| Stop half-way | fine: the old C++ and the old OS keep running until the team tries again |
