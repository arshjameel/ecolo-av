# The team's work list, after Friday's audit

> **A snapshot from 15 October 2026.** What's true now: [Status](status.md), [The plan](plan.md) and `TODO.md`. The audits ran on 16 and 21 Oct (`docs/history/audit-2026-10-16-car.md`, `audit-2026-10-21-car.md`).

Written 15 October 2026, **before** the audit, as a draft. After Friday, update it from
what the audit found: tick the "Audit says" column, fix each task's size, and put a name
in each "Who" (Q33). Each task is sized for one or two people.

> **The short version**
> - **Five tracks** that people can work on side by side: A the car's computers, B the drive path, C the sensors, D our software on the car, E power and wiring.
> - **Nothing is reflashed or replaced before its backup is checked.** A1 comes first, and A2 (upgrading the Jetson) waits for it.
> - **Safety first in track B:** the ESP32's missing command timeout (B2) is fixed before the car drives on the ground.
> - Each task says what it **needs** first, and when it's **done**. Pick one with nothing left in "Needs".

## The order, in one picture

```text
 Friday's audit
      │
      ▼
 A1 back up everything ──► A2 Jetson: JetPack 7.2 ──► A3 Jazzy + our stack on the Orin ──► D tasks on the car
      │                └─► A4 Pis: Ubuntu 24.04  ──┘
      │
      ├─► B1 drive on stands (legacy path) ──► B2 ESP32 timeout ──► B5 calibrate ──► first drive on the ground
      │
      └─► C sensors, one by one (each needs its device known from the audit)
```

## How to use this list

| Column | Means |
|---|---|
| **Size** | S = an afternoon; M = a few days; L = a week or more |
| **Needs** | tasks or audit answers that must come first |
| **Done when** | the check that proves it, written so anyone can run it |
| **Audit says** | fill in after Friday: ✅ as planned, ⚠️ changed, ❌ not needed |
| **Who** | a name (Q33) |

Tick a task by moving it to "Done" at the bottom, with the date and the commit.

## A. The car's computers

| # | Task | Size | Needs | Done when | Audit says | Who |
|---|---|---|---|---|---|---|
| A1 | **Back up every computer**: home folders and `/etc` (Friday); SD-card images of each Pi; a **full image of the Orin** with `l4t_backup_restore.sh` | M | Friday's audit; an x86 Ubuntu PC for the Orin image; the encrypted drive | each backup **restored once** onto a spare card or checked by listing it; the files that exist only on the car listed in the audit write-up | | |
| A2 | **Upgrade the Jetson (Orin) to JetPack 7.2** (Ubuntu 24.04). It's a reflash, not an upgrade: everything on it is erased | M | A1 done and checked; the module's part number (-501 is fine, **-502 has a reported flashing problem**); the team agrees | `cat /etc/nv_tegra_release` shows the new version; CUDA and TensorRT run (`tools/build_engine.py` makes an engine) | | |
| A3 | Jazzy and our stack on the new Orin, from apt; replace the Pixi kit there | S | A2 | `pixi run test` passes there, or `colcon test`; `ros2 launch aav_bringup car.launch.py machine:=orin gps:=false` starts | | |
| A4 | The Pis to **Ubuntu 24.04** (or keep their OS and use the Pixi kit), one at a time | M each | A1 (that Pi's SD image); which Pis we still need (audit, W9) | each Pi runs its part of our stack (`machine:=motor_pi` etc.) | | |
| A5 | **Rotate the leaked secrets**: revoke the GitHub token, the Firebase keys; change the car's WiFi password; set new logins | S | whoever controls the accounts (Q34) | the old token fails; the old WiFi password fails | | |
| A6 | **The network**: fixed IPs for every machine; one ROS domain (42); multicast working (or static peers); a written network map | S | audit W1-W5 | `pixi run doctor` OK on a laptop; `pixi run bash tools/net_map.sh` matches the network map, written in the audit page | | |
| A7 | **Time**: chrony on every machine, the Orin as the server | S | A6 | `chronyc tracking` under 10 ms on each | | |
| A8 | **Start at boot**: our stack as systemd services on each machine, replacing the 2025-26 ones (kept, disabled, for a way back) | M | A3, A4; the boot list from the audit | power on → `pixi run check car` passes, with no one logged in | | |

### A2 in detail: upgrading the Jetson

| # | Step | ☐ |
|---|---|---|
| 1 | A1 is done: the full Orin image **and** a separate copy of `/home/aavjetson`, the detector weights, `camera_decode` and the OSRM data. Restore-test the home copy | ☐ |
| 2 | Write down from the audit: the JetPack version, the module part number, the disk (eMMC or NVMe), what was running | ☐ |
| 3 | Agree as a team. **Note:** NVIDIA's Isaac ROS is Humble-only on the Orin today, so JetPack 7.2 + Jazzy means no Isaac ROS for now. We don't use it (`perception-and-hud-map.md`) | ☐ |
| 4 | Flash with SDK Manager from an x86 Ubuntu 22.04/24.04 PC, recovery mode, USB-C cable | ☐ |
| 5 | First boot: user, hostname and IP **as in the network map** (A6), so the other machines still find it | ☐ |
| 6 | Check CUDA and TensorRT; `sudo nvpmodel -m 0` (full power) if the team wants it | ☐ |
| 7 | Then A3 | ☐ |
| ↩ | **Way back:** restore the full image with `l4t_backup_restore.sh` | |

Details: [docs/platforms.md](platforms.md), "Moving the Orin to JetPack 7.2".

## B. The drive path: throttle, steering, brake, e-stop

| # | Task | Size | Needs | Done when | Audit says | Who |
|---|---|---|---|---|---|---|
| B1 | **Our stack drives the car on stands**, through today's ESP32 (`aav_legacy` on the Motor-Pi) | S | the audit (D2-D5); the 2025-26 Motor-Pi software stopped | the 8 tests in `history/friday-2026-10-16.md` part 5 pass | | |
| B2 | ⚠️ **A command timeout on the ESP32**: no command for 100 ms → brake. Either flash the Rust ESP32 (`boards/esp32-umod`, merged 15 Oct) after its bench plan, or add a timeout to today's firmware. Close the open update page | M | Ben's go-ahead to flash; a spare ESP32 for the bench (`docs/esp32-bench.md`) | unplug the Motor-Pi's UART while driving on stands → the car brakes | | |
| B3 | **The e-stop input**: wire the e-stop to the board, so the firmware knows | S | audit S1 (what the Kar-Tech and the mushroom cut) | press each e-stop → `/vehicle/status` shows it | | |
| B4 | **Brake safe state**: what the brake does with power cut, and at boot | S | audit S2; the Jrk G2 settings (D6) | written in `vehicle.yaml` / the firmware; tested on stands | | |
| B5 | **Calibrate** steering (centre, full lock each way), throttle (deadband, top speed), brake (travel). The tests are a script: `pixi run actuator-test <test> --stands`, then in the lot ([docs/actuator-tests.md](actuator-tests.md)) | M | B1; the Jrk settings (D6, N9); measurements (M5) | `vehicle.yaml` has measured numbers; the scripted drive goes straight within 0.5 m over 10 m | | |
| B6 | **Steering position feedback**: the actuator's own pot, or the KTC-100. The 2024-25 firmware read it from the steering Jrk G2 (`getScaledFeedback()`, docs/older-code.md): try that first | M | audit D6, D7 (wire count) | steering angle in `/vehicle/status`, within 1° | | |
| B7 | **CAN (the KEFC plan)**: **the hardware lead decides** (CAN or not, the wheel boards, the 100 ms timeout), then order the parts | M | audit C1 (is U7 wired to the ESP32?); the hardware lead's decision. The ROS side is ready: CAN v0.2.0 (wheel speed frames, 100 ms timeout) and `spine_bridge`'s `/vehicle/wheels` merged 15 Oct (`can-v2-work`) | `candump` shows the spine's frames on the Orin | | |
| B8 | **The first drive on the ground**, walking pace, empty lot, safety driver | S | B1, B2, B3, B5 | `a-to-b.md` test 5: a scripted 10 m at 0.5 m/s | | |

## C. The sensors

| # | Task | Size | Needs | Done when | Audit says | Who |
|---|---|---|---|---|---|---|
| C1 | **GPS** on our driver (`ublox_gps`), with RTK corrections if there's a source | S | audit N1, N2 | `/sensors/gps/fix` at 5-10 Hz; the HUD shows 3D or RTK | | |
| C2 | **IMU** on our Rust driver; its mounting angles in `vehicle.yaml` | S | audit N3, M6 | `/sensors/imu/data` at 100 Hz; turning left shows +z rotation | | |
| C3 | **Cameras**, with their calibration files (now: both front ribbon cameras on one Pi 5, MediaMTX → `aav_camera_stream` on the Orin) | M | audit N4; the backed-up calibration | `/sensors/camera/front_left/image_raw` steady for 30 min | | |
| C4 | **Fix the camera freezes** | M | Friday's 10-minute watch (`history/friday-todo-2026-10-16.md` part 5) | 1 hour with no freeze, logged | | |
| C5 | **Radar** on `aav_radar_ti`, its clusters in Nav2 | S | audit N7 | `/perception/radar/clusters` while someone walks in front | | |
| C6 | **Lidar driver** (done: the roof Velodyne VLP-16 on the stock `velodyne` driver, on by default on the car; untested with the real lidar), its points into Nav2's costmap and the collision monitor | M | audit N6 (make, model, IP) | an obstacle in front shows in RViz's local costmap | | |
| C7 | **Wheel encoder** (now: the hardware lead's Arduino on the Orin's CAN, 0x200/0x201; the 2025-26 UDP encoder is off since 23 Oct): alive, clean sensor; finer speed (more spokes, or the KEFC Hall sensors: `wheel-core` and `spine_bridge` already read them, 15 Oct) | M | audit D4, C2 | `/vehicle/status` speed matches a GPS speed within 10% at 1 m/s | | |
| C8 | **Stable device names**: USB serial numbers into `tools/udev/99-aav.rules` | S | the survey's "by-id" lists | unplug and replug everything: same names | | |
| C9 | **Battery**: read the KM110F, or drop it from the plan | S | audit N8 | `/vehicle/battery` shows the voltage, or a decision written down | | |

## D. Our software on the car

| # | Task | Size | Needs | Done when | Audit says | Who |
|---|---|---|---|---|---|---|
| D1 | **Measure the car**, and replace every guess in `vehicle.yaml` | S | Friday's tape measure (M1-M7) | no "guess" comments left in `vehicle.yaml`; the URDF matches photos in RViz | | |
| D2 | **Bring in what existed only on the car**: code into git with a `PROVENANCE.md`; weights into `models/` with checksums | M | A1; the audit's list | each item either in git or written off in `docs/older-code.md` | | |
| D3 | **The detector on TensorRT** on the Orin (RF-DETR Nano by default; YOLO with `detector:=yolo`); test it against the 2025-26 detector on a recording | S | A3; a recording from C3 | ms per frame on the Orin written in `docs/perception.md` | | |
| D4 | **The HUD and the map on the dash's two screens**, full screen, starting at boot. **Ready to install (15 Oct):** `car_install.sh --screens`, then two monitor names in `~/.config/aav/screens.env` ([below](#d4-in-detail-the-two-screens)) | S | audit W10 (which computer, the outputs' names, X11 or Wayland); auto-login (an admin); E1 | power on → the HUD on the small screen, the map on the big one, both full screen; kill one → back in 5 s | | |
| D5 | **The HUD's map panel**, with offline tiles of the test area | M | the test area chosen | the car's dot moves on the map in Gazebo, then on the car | | |
| D6 | **A-to-B**: the test lot's datum, a start spot, then a GPS goal with `pixi run drive-to-point --lat .. --lon ..` (the driver, `aav_drive_to_point`, is built and proven in Gazebo, 15 Oct) | M | C1, C2, B8; the brake test (4.3) for `max_brake` | `a-to-b.md` test 5 with a GPS goal: stops within 1 m of B (2 m without RTK) | | |
| D7 | **Python to Rust, step 1**: the spine bridge and ego | L | Ben's go-ahead | the same tests pass with the Rust nodes; `bench-check` 7 of 7 | | |

### D4 in detail: the two screens

How it works: [docs/car-screens.md](car-screens.md). Two systemd *user* units start
`hud` and `map` at login, each with `--fullscreen --monitor <name>`.

| Ready (15 Oct, tested on a Mac) | The audit must confirm |
|---|---|
| `--monitor NAME\|N` and `--list-monitors` in both programs | **Which computer** drives the screens: the dash Pi (DASH-UI) or the Orin? Both screens on one computer? |
| `aav-hud.service`, `aav-map.service`, an autostart entry | **The outputs' names** (`xrandr --listmonitors`, or `wlr-randr`, or `kmsprint`) and resolutions; which is the small one |
| `~/.config/aav/screens.env`: monitors, ROS domain 42, the map's files | **X11 or Wayland** (`echo $XDG_SESSION_TYPE`): on Wayland the names may read `XWAYLAND0` |
| `car_install.sh --screens` and `car_screens_install.sh [--dry-run]` | **The Pi's GPU** copes with two full-screen OpenGL windows (`glxinfo -B`), and its OS is 64-bit (the kit needs `aarch64`) |
| The kit carries the street map and the place database | **Auto-login** to the desktop: on already? (an admin setting) |
| | **What starts the 2025-26 app** at boot now (its services), so ours replaces it cleanly (A8) |
| | **E1: the dash's 5 V** (3.48 A seen): two screens plus a Pi may brown out |

## E. Power and wiring

| # | Task | Size | Needs | Done when | Audit says | Who |
|---|---|---|---|---|---|---|
| E1 | **5 V for the Pis**: converters that give 5 A each (the dash one showed 3.48 A) | S | audit P1; C4's findings | `vcgencmd get_throttled` = `0x0` after an hour of driving | | |
| E2 | **A wiring diagram**: every computer, sensor, converter and fuse | M | the audit photos | a picture in `docs/img/`, linked from `docs/car-photos.md` | | |
| E3 | **The Orin's power**: its supply and fuse | S | audit P2 | written in E2's diagram | | |

## Done

| Date | Task | Commit |
|---|---|---|
| | | |
