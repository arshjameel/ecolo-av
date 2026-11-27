# Car day: the runbook

Written 24 October 2026, for the team; split into two days on 1 November (Ben: "Split
it"). This page **owns the order**: [Car setup](car-setup.md) only says how to set up
one computer.

> **The short version**
> - **Two days.** **Day A changes nothing** you can't undo: backups, the steering's ends, the brake, the Rust ESP32 and the joystick on stands, then walking pace. **Day B**, later: the upgrades and the Orin's reflash, once a restore is proven.
> - **Before you leave:** `pixi run car-day-build`, then `pixi run car-day-check` until it says **0 missing**.
> - **Name the people first** (the roles table). Every ✍ step needs a signature: who, and the time.
> - **Wheels off the ground** until step A10's signature.
> - **Record every session**, stands too: `record.sh --lidar` (session 10).
> - 🔒 = waits on the hardware lead's answer (session 3). Skip it, and go on.
> - **Secrets:** logins come from the team's password vault. Type them; never paste them into a file, a chat or the repo.

!!! note "Where this stands (checked 14 Nov)"
    This runbook has **not been run end to end**, and no step here carries a signature.
    What the records say was done on the car:

    | When | What | Record |
    |---|---|---|
    | 16, 21 Oct | read-only audits | [16 Oct](history/audit-2026-10-16-car.md), [21 Oct](history/audit-2026-10-21-car.md) |
    | 6 Nov | drove the 2025-26 stack as is; nothing flashed ([Car evening](car-evening.md)) | [6 Nov](history/audit-2026-11-06-car.md) |
    | 9 Nov | a car day was planned ([Rolling out](rollout.md)) | **none in the repo** |
    | by 11 Nov | our kit on the localization Pi; its first install broke on `aav_hud` (PR #28) | [14 Nov review](history/audit-2026-11-14-review.md), section 5 |
    | 13 Nov | the car drove "with only the localization update installed"; the full suite isn't on it | `history/messages-2026-11-13.md` |

    So A2's backups, A7b, A8 (the Rust ESP32) and Day B are still to do, by the records.
    **Which page when:** this one for a whole day at the car, in order;
    [Rolling out](rollout.md) for the short version and for a newer version later;
    [Car setup](car-setup.md) for one computer; [Car install](car-install.md) for what the
    kit is and how to undo it.


**The ESP32 and the Motor Pi, on one page:** [Flash the ESP32, upgrade the Motor Pi](esp32-motor-pi-runbook.md) (Senura's runbook, 17 Nov).

## The people

Fill this in before the day. Nobody does two jobs at once.

| Role | Does | Name | Phone |
|---|---|---|---|
| **Hardware lead** | decides every hardware question; sets the steering and brake actuators' settings | | |
| **E-stop person** | holds the **Kar-Tech wireless stop** whenever the car has power. Does nothing else | | |
| **Safety driver** | in the seat for every ground move; the brake pedal | | |
| **Laptop person** | runs the commands; keeps the recordings | | |
| **Approver** | signs each ✍ step. Not the laptop person | | |

**The Kar-Tech stop is the main stop**, all day. The HUD's button, `pixi run estop` and the
joystick's button come second.

**A ✍ step** can't be undone, or moves the car more than before. Its line:
`✍ Signed: ________ at __:__`. No signature, no step.

## Logins

Today the logins are only in Ben's Mac's Keychain, so only Ben can log in. The team needs
a **shared password vault** (Q34). Write here only where it is and what type, never a
password:

| ☐ | The vault | |
|---|---|---|
| ☐ | Where (which service, who invites) | ________ |
| ☐ | Type | a password manager the whole team can open |
| ☐ | Each machine's entry, named as in the table below (`aav-jetson`...) | |

## The machines

As the 21 Oct audit found them ([audit](history/audit-2026-10-21-car.md)), with what the
[6 Nov audit](history/audit-2026-11-06-car.md) added. Every address
is `192.168.1.x`. **Role** is the name `car_setup.sh` uses. Usernames: none are in the
repo yet: fill them in from the vault.

| Machine | Address | Role | Username | Vault entry | Runs today (2025-26, Humble, domain 0) | Day B: should be |
|---|---|---|---|---|---|---|
| JETSON, **the Orin** (AGX Orin) | .140 | `orin` | ☐ | `aav-jetson` | the Velodyne driver (UDP 2368), lidar SLAM, `foxglove-bridge` (8765). JetPack 6 (R36.4.7, 22.04). 6 Nov: `can0` and `can1` down | JetPack 7.2 (24.04): a reflash |
| CORTEX, **the Motor Pi?** (H1), Pi 4 | .145 (21 Nov; .120 until then) | `motor_pi` | ☐ | `aav-cortex` | `aav_serial_bridge` on `/dev/ttyAMA0` (to the ESP32), joystick teleop. Ubuntu 22.04. 6 Nov: it does the Motor Pi's job today; the hardware lead still confirms H1. 21 Nov: **on WiFi**, its cable port down, its address from the router (so it moved): a cable and a reserved address are the hardware lead's to decide; our kit of 20 Nov is on it too; 3.8 GB free of 29 | 24.04, only after the Rust ESP32 |
| LOCALIZATION, Pi 4 | .113 | `localization` | ☐ | `aav-localization` | the IMU (200 Hz) and GPS (10 Hz) drivers, ours, in domain 42 (seen 21 Nov). **Ubuntu 24.04** (seen 21 Nov). Our kit of 9 Nov is on it | done: 24.04; its machine file is `machines/localization.yaml` |
| aav-desktop, **the dash**, Pi 4 | .196 (WiFi), .195 (its cable; seen 21 Nov) | `dash` | ☐ | `aav-desktop` | the two screens. Ubuntu 22.04 Desktop. 21 Nov (Ben): Flutter, the 2025-26 UI, its map data and Docker's containers removed (copies on Ben's Mac); 35 GB free; the 2025-26 screens no longer start | 24.04 Desktop, our two screens |
| CAM1-PI, **the front cameras**, Pi 5 | .108 (also seen at .102: C1) | `camera_pi` | ☐ | `aav-cam1` | MediaMTX, RTSP on 8554. Pi OS | Pi OS, updated |
| CAM2-PI, Pi 5 | .118 | `camera_pi` | ☐ | `aav-cam2` | MediaMTX with a USB camera. Pi OS | Pi OS, updated |
| `.211`, a Pi: the voltage meter's? (Ben, 21 Nov) | .211 | none | ☐ | ☐ | Ubuntu 22.04; one ROS program in domain 0 (21 Nov). No login known | ask the hardware lead first |
| the ESP32 | .132 (WiFi) | none | none | none | the 2025-26 C++ firmware (seen 6 Nov; no record of a flash since): **no command timeout** | the Rust firmware (Day A, A8) |
| the Velodyne | .201 | none | none | none | sends to the Orin | |
| **the lab's Ubuntu desktop** (x86, NVIDIA; stays in the lab) | ☐ | `lab_desktop` | ☐ | ☐ | CARLA (☐ which version) | **frozen** until the Orin's reflash is done (Day B) |

Our stack runs in **ROS domain 42**, so it never mixes with theirs (domain 0).

## The order

```text
 DAY A: nothing destroyed                                     DAY B: later
 ─────────────────────────────────────────────                ───────────────────────────
 A0 at home ─► A1 safety ─► A2 back up everything ✍           B1 a restore proven ✍
                              │                                B2 the Pis: upgrade ✍
                              ├─► A3 measure   A4 questions    B3 the Orin: image, reflash ✍
                              ▼                                B4 the lab desktop ✍
 A5 our stack, beside theirs ─► A6 steering ends (Way A) ─► A7 brake on stands
   ─► A7b the Motor Pi's UART ✍ ─► A8 the Rust ESP32 ✍ ─► A9 keyboard and joystick on stands ─► A10 walking pace ✍
   ─► A11 the car tests and the map            (recording all day: session 10)
```

| Step | What | Section | Who | Needs |
|---|---|---|---|---|
| A0 | Before leaving, at home | [session 0](#session-0-before-leaving-at-home) | laptop person | internet |
| A1 | Safety first | [session 1](#session-1-safety-first) | everyone | the people named |
| A2 ✍ | **Back up every machine**, checked | [A2](#a2-back-up-every-machine) | laptop person | the encrypted drive |
| A3 | Measure and weigh (any time on Day A) | [session 2](#session-2-measure-and-weigh) | 2 measurers | A1 |
| A4 | Questions for the hardware lead (best: the week before) | [session 3](#session-3-questions-for-the-hardware-lead) | hardware lead + 1 | |
| A5 | Our stack, beside theirs, on the Orin and the Motor Pi | [session 4](#session-4-the-orin-and-the-motor-pi) | laptop person | A2 |
| A6 | **The steering's ends, Way A** (Pololu's utility), on stands | [A6](#a6-the-steering-actuators-ends-way-a) | hardware lead + laptop person | A2 |
| A7 | **The brake, on stands** | [A7](#a7-the-brake-on-stands) | hardware lead, e-stop person | A1 |
| A7b ✍ | **The Motor Pi's UART, ready**: the console off it, theirs off at boot, ours on | [A7b](#a7b-the-motor-pis-uart-ready) | laptop person, hardware lead | A2, A5; H1 🔒 |
| A8 ✍ | **The Rust ESP32**, on stands | [session 6](#session-6-the-esp32) | laptop person, e-stop person, hardware lead | A2, A5, A7, A7b; H1 🔒 |
| A9 | The keyboard, then the **joystick**, on stands | [session 9](#session-9-the-first-drive-on-stands), [A9b](#a9b-the-joystick-on-stands) | safety driver, e-stop person, laptop person | A8 |
| A10 ✍ | **The ground, at walking pace** | [A10](#a10-the-ground-at-walking-pace) | all five | A6-A9 passed |
| A11 | The car tests and the lot's map | [Know the car](know-the-car.md) | all five | A10 |
| A12 ✍ | **The diffusion planner's trial** (0.3), its own day; not run yet | [A12](#a12-the-diffusion-planners-trial) | safety driver, e-stop person, laptop person, approver | A11; 4.23-4.26 with the lattice |
| any | The camera Pi; CAN, read-only; Foxglove | sessions [5](#session-5-the-camera-pi), [7](#session-7-can-read-only), [8](#session-8-foxglove-from-a-laptop) | 1 each | A5 |
| all day | What to record | [session 10](#session-10-what-to-record) | laptop person | |
| B1-B4 | Day B | [Day B](#day-b-upgrades-and-the-reflash) | laptop person, hardware lead, approver | Day A done; a restore proven |

Autonomy (navigation goals on the ground) comes after A11, on another day: [Every test](tests.md), stage 5.

## Ready for car day (the 0.2 checklist)

0.2 is "car-ready" when every box here is ticked. [Release 0.2](release-notes.md#whats-left) links here.

| ☐ | Check | How |
|---|---|---|
| ☐ | Every test green | `pixi run test` |
| ☐ | The kit ready | `pixi run car-day-check`: **0 missing** |
| ☐ | The ESP32's emulator check | `pixi run esp qemu-check`: PASS |
| ☐ | `car_setup.sh`'s dry run, every role | `bash tools/car_setup.sh <role>` on a laptop (`--host-ok` on a Linux one): no STOP but `upgrade`'s two refusals (`localization`, `motor_pi`) |
| ☐ | The first-move gaps (TODO 74) | each closed, or signed off by the hardware lead |
| ☐ | The people named | the table above: names and phones |
| ☐ | The hardware lead's answers | H1 (which Pi is the Motor Pi), C1 (CAM1's address), `.211`'s job, who owns the router |

## What to bring

| ☐ | Thing | For |
|---|---|---|
| ☐ | the laptop, after session 0, and its charger | everything |
| ☐ | the USB stick with `car-kit/` | session 4 |
| ☐ | an SD card reader for the Mac, and **a spare SD card** (B1's restore) | A2, B1 |
| ☐ | a laptop with Pololu's **Jrk G2 utility**, a USB cable for the Jrks | A2, A6, A7 |
| ☐ | USB-C Ethernet dongle, 2 Ethernet cables, a USB-C hub | the car's switch |
| ☐ | a USB data cable for the ESP32 board's USB port (check its plug before) | session 6 |
| ☐ | **stands** for all four wheels, wheel chocks | sessions 1, 6, 9; A6, A7 |
| ☐ | the **RC transmitter**, charged | sessions 1, 6 |
| ☐ | the printed sheet, [Measure the car](measure-the-car.md), 2 copies, pens | session 2 |
| ☐ | tape measure, masking tape, string, a plumb line, a phone level app | session 2 |
| ☐ | **4 bathroom scales** (or 1 and blocks), 2 blocks 20-30 cm | session 2 |
| ☐ | a protractor | sessions 2, 9 |
| ☐ | a multimeter | sessions 6, 7 |
| ☐ | a printed checkerboard on a stiff board (9 x 6 inner corners, 25 mm squares) | session 5 |
| ☐ | 2 phones: a stopwatch, and one to film | session 5 |
| ☐ | the backup drive (**encrypted**: the backups hold passwords), room for every card image | A2 |

## A0 · Session 0: before leaving (at home) {#session-0-before-leaving-at-home}

**Who:** the laptop person. **How long:** 30-60 min. **Bring:** internet.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `git fetch upstream && git switch main && git merge --ff-only upstream/main` (the team's main: [CONTRIBUTING.md](../CONTRIBUTING.md)), then `pixi run build` | `Summary: ... packages finished` | `pixi run build` again; the end says which package |
| 2 | `pixi run get models` (once) | `models/rf-detr-nano.pth`, `models/yolo11n.pt` | the internet |
| 3 | The ONNX for the Orin (box below) | `models/rf-detr-nano.onnx` (108 MB) | skip: session 4 has a slower way |
| 4 | `pixi run car-day-build` | `v2: PASS`, `v1: PASS`, `esp qemu-check: PASS`, `legacy-esp32: PASS`, `docs: PASS` | each step's log is in `car-day/esp32/` |
| 5 | `pixi run car-kit`, then copy `car-kit/` to the stick | `car-kit/` (about 2 GB); `cp -R car-kit /Volumes/<stick>/` | [From a USB stick](car-install.md) |
| 6 | The laptop's Cyclone file (box below) | `~/.config/aav/cyclonedds-car.xml` | |
| 7 | `pixi run bash tools/net_map.sh` and `pixi run doctor` once | macOS asks about the local network: **Allow** | System Settings → Privacy → Local Network |
| 8 | **`pixi run car-day-check`** | `0 missing` | run each `->` line it prints, then again |
| 9 | Print the measuring sheet; pack the bag (above) | | |

Step 4 puts the ESP32 firmware here, out of git:

| File | What |
|---|---|
| `car-day/esp32/spine-esp32-umod-v2.elf` | the Rust firmware, version 2 frames (the Orin's heartbeat): **the one to flash** |
| `car-day/esp32/spine-esp32-umod-v1.elf` | the same, 13-byte frames: if the Motor Pi must stay on version 1 |
| `car-day/esp32/spine-esp32-umod-v2-uplink.elf` | version 2 with the uplink on TX1 and the three-position RC switch: **only once H9 and H10 say yes** (6e) |
| `car-day/esp32/qemu-check.log` | the emulator's proof: it brakes 111-120 ms after the heartbeat stops |
| `car-day/esp32/BUILD.txt`, `SHA256SUMS` | when, from which commit; the check reads it |

!!! note "Step 3: the ONNX, made on the Mac"
    The Orin then needs only TensorRT to build its engine: no PyTorch, no pip, no internet.
    5 s on Ben's Mac.

    ```bash
    pixi run -e ml python3 -m venv --system-site-packages /tmp/onnx-venv
    /tmp/onnx-venv/bin/pip install onnx onnxscript
    /tmp/onnx-venv/bin/python tools/export_model.py --format onnx models/rf-detr-nano.pth
    ```

    `pixi run car-kit` (step 5) then copies it to the stick, with the other models.

!!! note "Step 6: Cyclone uses one network card"
    Cyclone DDS (our ROS messaging) picks **one** network card. With the phone's hotspot
    on WiFi and the car on the dongle, it may pick the wrong one ([Your Mac](mac.md)).
    Plug the dongle in, find its name (`ifconfig`: the one that appears, such as `en7`),
    and write this file with that name:

    ```xml
    <!-- ~/.config/aav/cyclonedds-car.xml: copy it to cyclonedds.xml at the car -->
    <CycloneDDS xmlns="https://cdds.io/config">
      <Domain id="any">
        <General>
          <Interfaces>
            <NetworkInterface name="en7"/>
          </Interfaces>
        </General>
      </Domain>
    </CycloneDDS>
    ```

    Keep it as `cyclonedds-car.xml` at home: a card named in `cyclonedds.xml` that isn't
    there stops every ROS program. At the car:
    `cp ~/.config/aav/cyclonedds-car.xml ~/.config/aav/cyclonedds.xml`; at home again,
    `rm ~/.config/aav/cyclonedds.xml`.

### `pixi run car-day-check` on Ben's Mac, 24 Oct

Moved to [the car-day log](history/car-day-log.md#pixi-run-car-day-check-on-bens-mac-24-oct) on 18 Nov (a dated record).

## The clocks, before any recording {#the-clocks}

Bags from two computers only line up if their clocks agree. `car_setup.sh`'s `services`
step installs chrony (the Orin keeps the car's time, the others ask it); until that has
run on a computer, check by hand.

| # | Do, on each computer that records | You should see | If not |
|---|---|---|---|
| 1 | `date -u`, at the same moment as the laptop's | the same time, to the second | write the difference on the sheet, with the computer's name |
| 2 | `chronyc tracking` (only where `services` has run) | `Reference ID` is the Orin; `System time` under 0.01 s | `car_setup.sh <role> check`, its `time` line; without chrony, step 1's note is the record |

## Day A: nothing destroyed

Every change on Day A undoes from a checked backup: our code goes in `~/aav-jazzy`, beside
theirs; the ESP32's flash goes back from its copy. **Nothing is upgraded, reflashed or
disabled.** Theirs is only stopped (`systemctl stop`), never turned off at boot.
The one exception is [A7b](#a7b-the-motor-pis-uart-ready), signed: the Motor Pi's UART, each
change with its own way back.

## A1 · Session 1: safety first {#session-1-safety-first}

**Who:** everyone, as named in [the people](#the-people) (Q32). **How long:**
15 min. **Bring:** stands, chocks, the RC transmitter.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | The car on **stands: all four wheels off the ground**, chocked | each wheel spins freely by hand | nothing else happens until it is |
| 2 | The e-stop person holds the **Kar-Tech wireless e-stop**, and stands by the **red button** | they say "e-stop ready" | |
| 3 | The **RC transmitter** on, its override switch **off** | the ESP32 ignores the sticks | switch it off before anything starts |
| 4 | Press the wireless e-stop once, with everything on | the boards' LEDs and the motor: write down what it cuts (S1) | it's S1: ask the hardware lead (session 3) |
| 5 | Say out loud: today's ESP32 C++ has **no command timeout** (S3) | everyone knows: until session 6, only the e-stop stops a frozen Pi | |
| 6 | Agree: **one drive path at a time**. Theirs (`aav_serial_bridge`) or ours, never both | | |
| 7 | Start recording, and keep it on for every session, stands too: `~/aav-jazzy/aav.sh tools/record.sh --lidar <session>` on the Orin once A5 is done; before that, theirs (session 10, rows 1-2) | a folder in `bags/` per session | |

**Stop point:** stands, e-stop in a hand, RC off.

## A2 · Back up every machine

**Who:** the laptop person. **How long:** 2-4 h (the copies run while A3 and A4 go on).
**Bring:** the **encrypted** backup drive (the backups hold passwords: WiFi, keys), an SD
card reader for the Mac. Nothing else starts until this step's signature.

| Machine | What | How | Checked by |
|---|---|---|---|
| The Orin | its home folder and `/etc` | [4a's](#4a-look-the-backup-is-a2) look, then the two rows below | `tar tzf etc.tgz > /dev/null` |
| The Motor Pi, LOCALIZATION, the dash (Ubuntu) | the **whole SD card**, as an image | the box below | `gunzip -t` |
| CAM1-PI, CAM2-PI (Pi OS) | the whole SD card, the same way | the box below | `gunzip -t` |
| Every Linux computer | the package lists, services, crontab, addresses, `/etc` (a second copy) | `bash car_setup.sh <role> backup --apply` ([Car setup](car-setup.md#running-the-script)) | its folder lists the files |
| The ESP32 | its whole flash (4 MB) | [6a](#6a-back-up-whats-on-the-board) | the file is 4194304 bytes |
| The steering and brake actuators | their settings files (Pololu's utility, File → Save settings file) | [Way A, A2](actuator-tests.md#way-a-pololus-utility-over-usb-car-day), for each | the file opens |

The Orin, from the laptop (the user from the vault):

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `rsync -a --exclude .cache --exclude aav-jazzy <user>@192.168.1.140:/home/<user>/ /Volumes/<drive>/aav-backup/jetson-$(date +%F)/home/` | it copies (1-3 h); go on meanwhile | it stops: run it again, it resumes |
| 2 | `ssh <user>@192.168.1.140 'sudo tar czf /tmp/etc.tgz /etc'`, then `scp` it to the same folder, then `sudo rm /tmp/etc.tgz` there | `etc.tgz`, under 50 MB | no sudo: write it down |

The Orin's **full disk image** needs the lab desktop: Day B ([B3](#b3-the-orin-its-image-then-the-reflash)).

!!! note "A Pi's SD card, from a Mac: the whole card"
    Undo is then a card swap. One Pi at a time: shut it down (`sudo poweroff`), take its
    card out, into the Mac's reader.

    ```bash
    diskutil list                                   # the card: "external, physical", its size
    diskutil unmountDisk /dev/disk4                 # disk4: the number diskutil showed
    sudo dd if=/dev/rdisk4 of=/Volumes/<drive>/aav-backup/cortex-$(date +%F).img bs=1m   # Ctrl-T: progress
    gzip /Volumes/<drive>/aav-backup/cortex-$(date +%F).img
    gunzip -t /Volumes/<drive>/aav-backup/cortex-$(date +%F).img.gz && echo GOOD
    diskutil eject /dev/disk4
    ```

    - Check the disk number twice: `dd` only reads the card here, so a wrong number only wastes time.
    - No `GOOD`: the copy is bad. Do it again before the card goes back in.
    - To put it back (Day B, B1): Raspberry Pi Imager → Choose OS → **Use custom** → the `.img.gz`.

| ☐ | Backed up and checked | File on the drive |
|---|---|---|
| ☐ | the Orin: home, `etc.tgz` | |
| ☐ | the Motor Pi's card (`gunzip -t`: GOOD) | |
| ☐ | LOCALIZATION's card (GOOD) | |
| ☐ | the dash's card (GOOD) | |
| ☐ | CAM1-PI's card (GOOD) | |
| ☐ | CAM2-PI's card (GOOD) | |
| ☐ | the ESP32's flash (4194304 bytes) | |
| ☐ | the steering actuator's settings | |
| ☐ | the brake actuator's settings | |

**Stop point:** every box ticked. `✍ Signed (approver): ________ at __:__`

## A3 · Session 2: measure and weigh {#session-2-measure-and-weigh}

**Who:** 2 measurers. **How long:** 90 min. **Bring:** the printed sheet, tape, string,
plumb line, phone level, 4 scales, 2 blocks, a protractor. **Due 30 Oct** (Gate 1,
[PLAN](plan.md)).

Follow [Measure the car](measure-the-car.md): it is the sheet.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Mark the zero point (sheet part 1), on flat ground | an X of tape under the middle of the rear axle | find flatter ground |
| 2 | M1-M5d: the car, the steering (wheels **off** the stands for M5c only, pushed slowly) | every "Measured" box filled, or "?" | "?" is fine: never a guess |
| 3 | S1-S8: every sensor's x, y, z and angles; the IMU's arrows (photo) | | |
| 4 | M7: the 4 corners on scales; M7b: the rear on blocks | 4 numbers and a total near 181 kg (400 lb) | one scale, a wheel at a time, the others on blocks of the same height |
| 5 | Photograph the sheet and the IMU's arrows | | |

**Stop point:** the sheet photographed. At home the numbers go into
`ros/bringup/aav_description/config/vehicle.yaml`, then `pixi run test` and `pixi run view`.

## A4 · Session 3: questions for the hardware lead {#session-3-questions-for-the-hardware-lead}

**Who:** the hardware lead, and one person writing. **How long:** 30 min. **Best the
week before:** H1, C1, `.211` and the router's owner are on the [0.2 checklist](#ready-for-car-day-the-02-checklist). **Bring:** this
table, a pen. The hardware lead decides these: write down what they say, don't argue it.

| # | Question | Unblocks |
|---|---|---|
| H1 ⚠️ | Is the Motor Pi to ESP32 link serial (UART)? Which port, which speed (we assume `/dev/ttyAMA0`, 115200)? **Is the Motor Pi the one called CORTEX (.120)?** | sessions 6 and 9; `legacy.yaml` |
| H2 | Does the ESP32 read the steering angle and the brake position (the Jrks' feedback)? | Gate 3, closed loop |
| H3 | What else is on the CAN bus, with IDs and rates? | session 7; `aav.dbc` |
| H4 | The wheel sensors' Arduino: which board and CAN chip; one for both wheels? Does it send `aav.dbc` 0x200 and 0x201 as they are? | session 7 |
| H5 | Can the wheel sensors tell direction (two per wheel)? | the speed's sign |
| H6 | Is the 2025-26 wheel encoder (Pico 2, UDP) still on the car? | `motor_pi.yaml` |
| H7 | CAN: 500 kbit/s? Terminated at both ends? Which interface on the Orin (`can0`, a USB adapter)? | session 7 |
| H8 | Will the ESP32 ever move onto CAN, or stay on the Motor Pi? | `actuators:=` |
| J1 | **The Jrk wires:** how long from the ESP32 to each Jrk, and do they run beside the motor cables? (TODO 49) | I2C stays, or the Jrks' serial with a CRC |
| T1 | **The TX1 wire:** is the ESP32's GPIO17 (TX1) wired to the Motor Pi's RX? (TODO 50) | the uplink frame: faults and positions back up |
| C1 | **The camera Pi:** is CAM1-PI at .108 or .102 (it answered on both)? Can it keep one address (the router)? | session 5; `orin.yaml` |
| W7 | **Which Orin:** AGX Orin 32 or 64 GB? (`cat /etc/nv_tegra_release`, session 4, says the JetPack) | the reflash (Q24) |
| S1-S3 | What the e-stops cut; the brake without power; anything else that stops a frozen Pi | session 9; A7 |
| R1 | **The router:** who owns it? Can the car's computers keep fixed addresses (reservations)? | C1; `car_setup.sh --address` |
| P1 | **`.211`:** what is it for? Can we leave it alone? | the machines table |
| L1 | The steering actuator's current limit for A6, and the brake actuator's: which numbers? | A6, A7 |

All the other questions: [audit questions](audit-questions.md).

**Stop point:** the answers written down, with who said them.

## A5 · Session 4: our stack, on the Orin and the Motor Pi {#session-4-the-orin-and-the-motor-pi}

**Who:** the laptop person. **How long:** 1-3 h; the backup runs while you do the rest.
**Bring:** the stick, the backup drive, a cable to the car's switch.

Day A: our stack goes **beside** theirs, in `~/aav-jazzy`; nothing of theirs changes.
The upgrades wait for [Day B](#day-b-upgrades-and-the-reflash).

First, on the laptop: `cp ~/.config/aav/cyclonedds-car.xml ~/.config/aav/cyclonedds.xml`,
dongle in, then `pixi run doctor` (it should find the car's network).

### 4a. Look (the backup is A2) {#4a-look-the-backup-is-a2}

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `ssh <user>@192.168.1.140` (the user: `hosts.txt`, JETSON) | a prompt on `ubuntu` | `pixi run bash tools/net_map.sh`: the Orin's address may have changed |
| 2 | `cat /etc/nv_tegra_release` | `R36 (release), REVISION: 4.7` = JetPack 6 (21 Oct) | write down what it says (W7) |
| 3 | `cat /proc/device-tree/model; free -h; df -h ~` | the module (AGX Orin), its memory (32 or 64 GB), **13 GB free** or more | not enough room: stop, ask the team |

The home folder and `/etc`: [A2](#a2-back-up-every-machine). The full disk image needs the lab desktop: Day B, [B3](#b3-the-orin-its-image-then-the-reflash).

### 4b. Our stack, from the stick

Changes nothing of theirs: everything goes in `~/aav-jazzy` ([From a USB stick](car-install.md)).

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `scp -r /Volumes/<stick>/car-kit <user>@192.168.1.140:~/` (or plug the stick into the Orin) | `~/car-kit/` | |
| 2 | `bash ~/car-kit/car_install.sh --dry-run` | every line OK or NOTE | a STOP line says what's wrong |
| 3 | `bash ~/car-kit/car_install.sh` | about 10 min; `done` | `~/aav-jazzy/build.log`'s end names the package |
| 4 | `~/aav-jazzy/aav.sh tools/car_listen.sh` | the devices, and who holds each | read-only: it opens nothing |

### 4c. Cyclone's network card, on the Orin

The Orin has more cards than the car's: a USB bridge (`l4tbr0`, 192.168.55.1) and maybe
Docker's. Name the right one.

| # | Do | You should see |
|---|---|---|
| 1 | `ip -br addr \| grep 192.168.1.` | one line: its first word is the card (such as `eno1`) |
| 2 | `mkdir -p ~/.config/aav`, then write `~/.config/aav/cyclonedds.xml` as in session 0, with that card's name | `tools/ros.sh` reads it, after the team's file |
| 3 | `~/aav-jazzy/aav.sh ros2 multicast receive` here, `ros2 multicast send` from the laptop (domain 42 both) | "Received from 192.168.1.x" |

The same on the Motor Pi in 4e (`eth0`, probably), and on any computer with WiFi too.

### 4d. RF-DETR's TensorRT engine

A TensorRT engine is built **on the Orin itself**, by its own TensorRT. Plain shell, not
`aav.sh`: this is JetPack's TensorRT, not Pixi's.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `cd ~/aav-jazzy/aav && ls -l models/` | `rf-detr-nano.onnx` (from session 0) | no ONNX: way 2 below |
| 2 | `/usr/src/tensorrt/bin/trtexec --onnx=models/rf-detr-nano.onnx --fp16 --saveEngine=models/rf-detr-nano.engine` | 2-10 min, then `PASSED`, and `models/rf-detr-nano.engine` | the error's last lines: write them down |
| 3 | In its output, `GPU Compute Time: ... median = ... ms` | the time per picture: write it down (PLAN T8.3). Guess: under 10 ms | |

**Way 2, with no ONNX** (needs internet): NVIDIA's PyTorch for JetPack 6 in a venv that
sees JetPack's TensorRT, then the repo's tool, which checks first and says what's missing:

```bash
python3 -m venv --system-site-packages ~/trt-venv
~/trt-venv/bin/pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
~/trt-venv/bin/pip install rfdetr onnx           # not rfdetr[tensorrt]: that brings PyPI's TensorRT
cd ~/aav-jazzy/aav && ~/trt-venv/bin/python tools/build_engine.py
```

The same venv also runs the engine once on the test stop sign, with the score and the time:
`~/trt-venv/bin/python tools/build_engine.py models/rf-detr-nano.onnx`.

!!! warning "RF-DETR inside the camera stream waits for the reflash"
    `car.launch.py` runs the detector in the `python3` of our Pixi Jazzy (3.12), and JetPack
    6's TensorRT is for its own Python (3.10). So on JetPack 6 the `camera_stream` part
    starts **without** the detector, with a warning. The streams, the previews and the
    watchdog still run (`detector:=none`, session 5). RF-DETR in the stream: after the
    reflash to JetPack 7.2 (Q24), and the engine built again there (an engine is tied to
    its TensorRT version).

### 4e. The Motor Pi

🔒 H1: is it CORTEX (.145 on 21 Nov, by WiFi: check `hosts.txt`)? Then the same kit, the same steps 4b and 4c there.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `ssh <user>@192.168.1.145`, then `uname -m` | `aarch64` | `armv7l`: a 32-bit OS, the kit can't run: write it down |
| 2 | 4b steps 1-4 there | 30-60 min on a Pi 4 | |
| 3 | 4c there (`eth0`, probably: CORTEX has WiFi too?) | | |

**Stop point:** our stack built on the Orin and the Motor Pi; the engine built (or why not).

### 4f. The Rust nodes are the ones running

Since 16 Nov (Ben) four nodes run in **Rust** on the car by default; since 17 Nov six. Since
20 and 22 Nov most have no Python twin left (below: "none"); only the ESP32's bridge, `rc_driver` and `camera_stream` stay one switch away. Check this each time you start `car.launch.py`.

| Node | Rust package (default) | Fall back to Python |
|---|---|---|
| ego + look_ahead | `aav_ego_rs` | none: the Python removed 22 Nov (`save/python-fallbacks-1`) |
| behaviour (with `behaviour:=true`) | `aav_behaviour_rs` | none: the Python removed 20 Nov (`save/python-behaviour`) |
| sensor watchdog | `aav_watchdog_rs` | none: the Python removed 20 Nov (`save/python-safety-nodes`) |
| localization_health | `aav_localization_health_rs` | none: the Python removed 20 Nov (`save/python-localization-health`) |
| lidar_scan (the flat scan) | `aav_lidar_rs` | none: the Python removed 22 Nov (`save/python-fallbacks-1`) |
| things (with `things:=true`) | `aav_things_rs` | none: the Python removed 20 Nov (`save/python-things`) |
| sensor mux + output mux (the ESP32's heartbeat; the Motor Pi alone too) | `aav_io_mux_rs` | none: the Python removed 22 Nov (`save/python-fallbacks-1`) |
| legacy_uart_bridge (the ESP32's UART, on the Motor Pi) | `aav_legacy_rs` | `legacy_impl:=python` |
| rc_driver (beside the bridge) | `aav_rc_rs` | `rc_impl:=python` |
| lane_finder (with the `lane` part; since 18 Nov) | `aav_lane_finder_rs` | none: the Python removed 22 Nov (`save/python-fallbacks-1`) |
| joy_alive (the joystick's check, wherever teleop runs) | `aav_joy_alive_rs` | none: the Python removed 20 Nov (`save/python-safety-nodes`) |
| stop_sign_watcher (the stop_signs part, next to the mux) | `aav_stop_sign_rs` | none: the Python removed 22 Nov (`save/python-fallbacks-1`) |
| camera_stream (the camera_stream part, only with `detector: none` in `orin.yaml`: RF-DETR runs in the Python one; since 18 Nov) | `aav_camera_stream_rs` | `camera_stream_impl:=python` |

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | read the first lines of `car.launch.py` (`~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin ...`) | `languages: ego_impl:=rust behaviour_impl:=rust watchdog_impl:=rust localization_health_impl:=rust lidar_impl:=rust things_impl:=rust io_mux_impl:=rust legacy_impl:=rust rc_impl:=rust lane_finder_impl:=rust lane_follower_impl:=rust joy_alive_impl:=rust stop_sign_impl:=rust camera_stream_impl:=rust` | a word other than `rust`: someone passed `:=python` |
| 2 | on the Orin: `ps -eo args \| grep '_rs/lib/' \| grep -v grep` | `aav_ego_rs/lib/...`, `aav_watchdog_rs/lib/...`, `aav_localization_health_rs/lib/...`, `aav_lidar_rs/lib/...`, `aav_io_mux_rs/lib/...`, `aav_stop_sign_rs/lib/...` (and `aav_lane_finder_rs/lib/...` with the lane part); on the Motor Pi `aav_legacy_rs/lib/...`, `aav_rc_rs/lib/...` (and `aav_joy_alive_rs/lib/...` with teleop there) (and `aav_behaviour_rs`, `aav_things_rs` with behaviour and things on) | a package missing: its build failed, see `~/aav-jazzy/build.log` |
| 3 | `ros2 node list` | `/ego`, `/look_ahead`, `/sensor_watchdog`, `/localization_health`, `/lidar_scan`, `/sensor_mux`, `/output_mux` (and `/things`): the same names as the Python's | |

**If a Rust node misbehaves:** stop the car (e-stop) and stop the launch. Write down what
you saw.

| The node | Then |
|---|---|
| has a fall-back (the bridge, `rc_driver`, `camera_stream`) | start again with its switch added, such as `... car.launch.py machine:=motor_pi ... legacy_impl:=python`. Short form, with `--` before the switch: `~/aav-jazzy/aav.sh car motor_pi -- legacy_impl:=python` (without `--`, Pixi stops: "received more arguments than expected") |
| has none (the rest, since 20 and 22 Nov) | no switch: the car stays stopped until the Rust is fixed. The Python as it was is in git at the `save/python-*` tags |

## A6 · The steering actuator's ends, Way A

**Who:** the hardware lead, and the laptop person. **How long:** 45 min. **Bring:** a
laptop with Pololu's Jrk G2 utility, a USB cable, a protractor,
[the sheet](actuator-tests.md#the-sheet) printed. On stands.

**Way A is the car-day path.** Way B (the hardware panel) needs the Rust firmware with the
uplink and link version 3: a later day ([why](#one-link-version-on-the-car)).

| # | Do | You should see |
|---|---|---|
| 1 | The steering actuator's settings saved (A2 did it) | the file |
| 2 | **Power off first.** Then, with the hardware lead, keep the ESP32 from driving the Jrk: leave the ESP32 off, or unplug its I2C **while everything is off**. Never unplug I2C with the power on | the ESP32 can't send a target |
| 3 | The current limit low, set by the hardware lead (Motor tab). The number: **______** (old: ______) | written on the sheet |
| 4 | Steps A1-A10 of [Way A](actuator-tests.md#way-a-pololus-utility-over-usb-car-day): Stop motor, then by hand full left, straight, full right; the angle at each end | the sheet: left end, centre, right end (counts), the two angles |
| 5 | Power off, I2C back in (power still off), the settings as they were (A10 there) | |

**Stop point:** the sheet filled in. No analog feedback: tell the hardware lead, and skip
3.10 in session 9.

**The sweep (session 9, test 3.10) runs only inside the measured ends.** The firmware's
software limits today are 400-3500 counts. Run 3.10 only if both measured ends lie
**outside** 400-3500, or after the measured ends are in `calibration.rs` and that
firmware is flashed. Otherwise skip it: write "3.10 skipped: ends inside the limits".

## A7 · The brake, on stands

**Who:** the hardware lead and the e-stop person. **How long:** 20 min. **Before any
ground move**, with whatever firmware is on the ESP32 now. Wheels off the ground.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Power on, nothing driving | the brake actuator goes to full. Push the car's brake pedal: it's already down, or hard | stop: tell the hardware lead |
| 2 | Try to turn a rear wheel by hand | it won't turn | the brake doesn't hold: no ground today |
| 3 | The Kar-Tech stop pressed | write down what it cuts (S1); the brake stays full | |
| 4 | Power off, then try the rear wheel again | write down: does the brake hold without power? (S2) | |
| 5 | The brake actuator's settings saved (A2) | the file | |
| 6 | The brake's limits, with the hardware lead: in Pololu's Jrk program (on its USB), the brake actuator's target and feedback at **released**, at the **touch point** (where the pads start to grip: turn the wheel by hand while it moves slowly) and at **full** | released ______ touch ______ full ______ (the firmware uses 400 and 3500, never measured) | |
| 7 | The most the brake may ever be pushed (the hardware lead's choice): a target, a pressure or the actuator's current limit; and can it **hold full brake for minutes** without overheating (the ESP32 holds full at boot) | max safe ______ ; current limit ______ A ; hold OK? ______ | |
| 8 | Does the brake actuator report its position (analog feedback)? | yes / no | |

**Stop point:** 1-2 pass; 3-8 written down. Rows 6-8 are what the brake needs, like the
steering's ends: the safe range a stronger stop may use when the car is loaded. The brake's strength is measured on the
ground later (tests 4.3, [Know the car](know-the-car.md)).

## A7b · The Motor Pi's UART, ready

**Who:** the laptop person, with the hardware lead there. **How long:** 30 min.
🔒 H1 (is CORTEX the Motor Pi, which port?). **Before:** A2 (the Motor Pi's backup) and A5.

**Why** (review 2, B6): the Pi's kernel console is on the ESP32's UART today
(`console=ttyAMA0`, audit of 6 Nov). The kernel's messages, and maybe a login prompt, go to the
ESP32 as noise. And theirs (`aav_serial_bridge`) only stopped comes back at the next boot:
**two writers on one wire**. The link check sees that as more frames than the Pi wrote.

`✍ Signed (approver): ________ at __:__ · Hardware lead: ________`

### Look first (changes nothing)

| # | Do, on the Motor Pi | Write down |
|---|---|---|
| L1 | `cat /proc/cmdline` | every `console=`: ______ |
| L2 | `ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0` | where `serial0` points: ______ |
| L3 | `systemctl list-units 'serial-getty@*'` | a login prompt on: ______ (none?) |
| L4 | `systemctl is-enabled aav_serial_bridge` | `enabled` or `disabled`: ______ |
| L5 | `sudo fuser -v /dev/ttyAMA0` (the port from L2) | who holds it: ______ |

### The three changes

| # | Do | You should see | The way back |
|---|---|---|---|
| 1 | Keep the boot line: `sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.2025-26` | the copy, one line long | |
| 2 | **The console off the UART:** `sudo nano /boot/firmware/cmdline.txt`. Delete only `console=ttyAMA0,115200` (or `console=serial0,115200`). Keep `console=tty1`. It stays **one line** | `cat` shows one line, no serial `console=` | `sudo cp /boot/firmware/cmdline.txt.2025-26 /boot/firmware/cmdline.txt` |
| 3 | **No login prompt on it:** `sudo systemctl disable --now serial-getty@ttyAMA0.service` (the name from L3; skip if none) | L3 shows none | `sudo systemctl enable --now serial-getty@ttyAMA0.service` |
| 4 | **Theirs off at boot:** `sudo systemctl disable --now aav_serial_bridge` | L4: `disabled`; L5: nobody | `sudo systemctl enable --now aav_serial_bridge` |
| 5 | `sudo reboot`, then L1-L5 again | no serial console, no prompt, theirs disabled, nobody on the port | the way back of 2-4, then reboot |
| 6 | **Ours on:** `~/aav-jazzy/aav.sh car motor_pi` | `serial port /dev/ttyAMA0 open`; L5 shows **one** process: `legacy_uart_bridge` (the Rust bridge, the default since 17 Nov), or `python3` with `-- legacy_impl:=python` | Ctrl-C |
| 7 | A second terminal: `~/aav-jazzy/aav.sh ros2 topic hz /control/car/link_frame` | about 10 Hz: the bridge reports what it writes | |

!!! warning "The boot line is one line"
    A line break in `cmdline.txt` stops the Pi booting. Edit with care, check with `cat`,
    and keep step 1's copy. Ubuntu on a Pi keeps it in `/boot/firmware/`; older Raspberry
    Pi OS in `/boot/`.

Ours at boot is `car_setup.sh motor_pi services` ([car setup](car-setup.md)): it refuses
while theirs is enabled. On Day A, start ours by hand (step 6).

**Back to the 2025-26 way** (the C++ on the ESP32, 6d): the way back of 4, so theirs runs
again. Leave the console off the UART: the C++ doesn't need it there either.

**Stop point:** 1-7 done and the sheet signed, or the way back done and why written down.

## A8 · Session 6: the Rust ESP32, on stands {#session-6-the-esp32}

**Who:** the laptop person, the e-stop person, the hardware lead. **How long:** 90 min.
**Bring:** the ESP32's USB cable, the RC transmitter, a multimeter.
🔒 **H1** (the link to the Motor Pi) and **Ben's OK** (on a call):
[First flash on the bench](esp32-bench.md) says "do it with Ben".
Or let it lead you: `pixi run esp walk` shows and says each step below, one at a time
([walk me through it](esp32-bench.md#walk-me-through-it-pixi-run-esp-walk)).

**Before:** A2 (the board's backup, 6a) and A7 (the brake) done; the Motor Pi and the Orin have our stack (session 4); the
laptop's USB cable in the ESP32 MicroMod. Every command below runs on the laptop, in the
repo.

### One link version on the car

The serial link between the Motor Pi and the ESP32 has a version, and both ends must
agree. **All of Day A: version 2.**

| Firmware on the ESP32 | `legacy.yaml` `frame_version` | When |
|---|---|---|
| the 2025-26 C++ | **1** | until 6b; and after a roll-back (6d) |
| the Rust, `spine-esp32-umod-v2.elf` (or `-v2-uplink`) | **2** | Day A, from 6b |
| the Rust with the uplink, for Way B (the hardware panel) | **3**, with `uplink: true` | a later day, once H9 says yes and the hardware lead agrees ([Way B](actuator-tests.md#way-b-the-hardware-panel-rust-firmware)) |

The Rust reads version 2 and 3 alike, so moving to 3 later needs no new flash; only
`legacy.yaml` changes, in one place, on the Motor Pi.

### 6a. Back up what's on the board

Done in A2. If not:

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash list-ports` | `/dev/cu.usbserial-…` | another cable: some carry only power |
| 2 | `vendor/cargo-tools/bin/espflash read-flash 0 0x400000 car-day/esp32/backup-esp32-$(date +%F).bin` | a 4 MB file (4194304 bytes) | hold the board's BOOT button while it starts |
| 3 | Copy it to the backup drive | | |

The backup holds the C++'s WiFi password: it's a secret. Never in git (`car-day/` isn't).

### 6b. Flash the Rust, and switch the Motor Pi to version 2

The flash overwrites the C++: only with 6a's file checked (4194304 bytes).
`✍ Signed (approver): ________ at __:__ · Hardware lead: ________ · Ben (on the call): ________`

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | On the Motor Pi: theirs off (A7b step 4); check `systemctl status aav_serial_bridge` | inactive, and disabled | never two drive paths: do A7b first |
| 2 | `vendor/cargo-tools/bin/espflash flash --chip esp32 --monitor car-day/esp32/spine-esp32-umod-v2.elf` | `spine-esp32-umod 0.1.0, link: Motor-Pi UART, version 2 frames (the Orin's heartbeat)` | roll back (6d) |
| 3 | On the Motor Pi: `legacy.yaml` to version 2: in `~/aav-jazzy/aav/ros/can_bridge/aav_legacy/config/legacy.yaml`, `frame_version: 2`, then `~/aav-jazzy/aav.sh build` | | |
| 4 | Start the Orin: `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin max_speed:=0.3 gps:=false lidar:=false stands:=true` | the output mux makes the heartbeat. Its `foxglove_bridge` can't open port 8765 (theirs holds it): harmless, see session 8 | |
| 5 | Start the Motor Pi: `~/aav-jazzy/aav.sh car motor_pi` | the ESP32's log: `LINK frames=` rising, `bad=0` | `bad=` rising: the speed or the wire (H1) |

### The RC transmitter {#the-rc-transmitter}

Before 6c, on the FS-i6X (Arsh and Ben, 21 Nov; [the rules](firmware.md#the-rc-transmitter-arsh-21-nov)).
The Rust reads it differently from the C++:

```text
right stick: down past 88 % = HARD BRAKE while held, the RC takes the car (it springs back)
             up past 88 %, steering centred, 0.5 s = give the car to the computer
left stick:  pushed away ◄── forward ──┤ the middle ±20 %: throttle off ├── reverse ──► pulled back
             (it doesn't spring back; past 88 % the other way while moving: the hard brake too)
CH6:         low = normal (the computer, once given) · high = manual
             (the team's FS-i6X has two positions: RC via ROS, the middle, can't be picked)
SwC (CH5):   does nothing. The left stick sideways: nothing
```

**To stop and stay stopped, use the right stick.** The left stick held all the way back
brakes, then reverses once the car has stopped.

| # | Do | You should see | If not |
|---|---|---|---|
| T1 | Write down the old values first (21 Nov: CH3 55%, CH4 0%, the rest Off), for 6d. Then **System setup → RX setup → Fail safe**, the car on (the receiver stores it). Each channel: pick it, OK, UP or DOWN to turn it on, put the stick or switch where it should be, and **hold CANCEL** (a short press doesn't save). CH6 with the switch low (**-100%**); CH3 with the left stick in the middle (**0%**); CH4 0%; CH2 with the right stick **held fully down** (about **+97%**); the rest Off. **Only then** Functions setup → End points → CH6, low side **80%**, hold CANCEL | Fail safe shows CH6 -100%, CH3 0%, CH4 0%, CH2 about +97% | **don't drive**: the spine can't tell a lost link otherwise. The order matters: the failsafe saves the switch's output as it is. CH2 never up: that is the give |
| T2 | Its display or the ESP32's `STATUS` line (`ch=`): CH6 at each position | about 1100 low, 2000 high | low under 1050 reads as a lost RC: End points CH6 isn't 80% |
| T3 | The left stick pushed away, then pulled back | CH3 near **1000** pushed away, near 2000 pulled back | the other way round: `throttle_reversed: true` in `ibus::RcMap` (Q7), rebuild |
| T3a | The right stick up, then down, then let go | CH2 under 1060 up, over 1940 down, 1500 let go | not past 88 %: the stop and the give can't work |
| T4 | The transmitter off | CH6 reads 1000 for a moment, then `faults` has 256 | the failsafe isn't set, or End points moved it: back to T1 |
| T5 | The ESP32's `CONFIG` line, and its `Ready` line at boot | `rc_band=0.30 rc_brake_ms=1000 rc_lost_below=1050 rc_fitted=true`; `Ready` ends `CH2 stop and give: past 88%` | the wrong build (no CH2: the 21 Nov one) |

Write the values here, so the next person can check them: CH6 low ____, high ____,
failsafe ____. CH3 pushed away ____, pulled back ____.

**The transmitter must be on for every session with the Rust**, CH6 low and the left stick
in the middle: in normal mode a lost RC is a hard stop, so without it nothing drives. The
RC has the car at power-up: the right stick up for half a second gives it to the computer.

### 6c. The checks, wheels off the ground

From [First flash on the bench](esp32-bench.md), in order. **Stop at the first that
fails, and roll back.** Drive from the laptop: `export ROS_DOMAIN_ID=42`, `pixi run keyboard`.

| # | Do | Expect |
|---|---|---|
| 1 | Power on, the transmitter on, CH6 low, the left stick in the middle | brake full (3500), steering centred (1950), DAC 0 V; `faults=0x0000`, `mode=Rc`. Then the right stick up, steering centred, for half a second: `mode=Manual` (the computer has the car, not armed yet) |
| 2 | Unplug a Jrk's I2C, plug it back | red LED; fault 8 (steering) or 16 (brake) while out |
| 3 | The bridge running, ego stale | stop frames: brake stays full; `frames=` rising, `bad=0` |
| 4 | Drive slowly from the keyboard | `mode=Computer`, green LED, brake lets go, the wheels turn **forward** |
| 5 | Steer left | the wheels turn **left** (if right: `steering_sign` in `legacy.yaml`, Q2) |
| 6 | Stop | throttle 0; hold brake, target 1050 |
| 7 | Reverse slowly | the direction changes only with the DAC at 0 |
| 8 | While driving, Ctrl-C the Motor Pi's bridge | within 0.2 s: throttle 0, brake full, `faults` has 1 |
| 9 | Start the bridge again | braked until a stop frame, then it drives |
| 9a | While driving, unplug the Orin's network cable, **the Motor Pi's bridge still running** | within 0.2 s: brake full, `faults` has 1; `frames=` still rising, `repeats=` rising. **The heartbeat (TODO 50)** |
| 9b | Plug the Orin back in | braked until ego stands still, then it drives |
| 9c | `pixi run estop` while driving | `estop=true` at once, brake full |
| 10 | CH6 high (manual), the left stick in the middle for a second, then pushed a little | `mode=Rc`, blue LED; ROS ignored; the wheels turn forward. Pushed before the middle: nothing moves |
| 10a | The left stick to the middle | throttle 0, brake released (the motor slows the wheels) |
| 10b | The left stick all the way back, held | brake full; with no speed on the ESP32, reverse after 1 s |
| 10c | The right stick down, then let go, the left stick still pushed | brake full at once; still braked after letting go, until the left stick moves |
| 11 | RC: transmitter off | within 0.1 s: brake full, `faults` has 256 |
| 12 | CH6 low (normal), the right stick up for half a second; drive slowly from the keyboard | the computer drives |
| 12a | While the computer drives, push the left stick | nothing changes: the computer still drives |
| 12b | The right stick down | brake full at once, `mode=Rc`; let go: still braked until the left stick moves |
| 12c | The right stick up for half a second, the keyboard still driving | the computer drives again |
| 13 | The red e-stop button | the car stops by itself |
| 14 | 10 min of 1-13 mixed | no unexpected reset (`the last reset:` stays the power-on one) |

### 6d. Roll back to the C++

Any time something is wrong: e-stop, Ctrl-C the bridge, then:

| # | Do | You should see |
|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash write-bin 0 car-day/esp32/backup-esp32-<date>.bin` | the board as it was: `=== ESP32: AAV Controller ===` in its log |
| 2 | On the Motor Pi: `frame_version: 1` back, `~/aav-jazzy/aav.sh build` | the C++ reads only the 13-byte frame |
| 3 | `sudo systemctl start aav_serial_bridge` (theirs), if that's the plan for the rest of the day | |
| 4 | The transmitter as the C++ had it: End points CH6 low side back to 100%, and Fail safe back to the values T1 wrote down (21 Nov: CH3 55%, CH4 0%, the rest Off, CH2 Off too) | CH6 low reads about 1000 again |

Or the C++ from source: `pixi run -e firmware legacy-esp32 -t upload` (`src/secrets.h` holds
its WiFi; with the placeholders it can't join the WiFi). The Rust with 13-byte frames:
`car-day/esp32/spine-esp32-umod-v1.elf`, `frame_version: 1`.

### 6e. Only if H9 and H10 say yes: the uplink and the RC modes 🔒

H9: the ESP32's TX1 (GPIO17) reaches the Motor Pi's RX. H10: CH6 is a three-position
switch. Both yes, and 1-14 passed:

| # | Do | You should see |
|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash flash --chip esp32 --monitor car-day/esp32/spine-esp32-umod-v2-uplink.elf` | the first log line ends `with the uplink on TX1 and the three-position mode switch` |
| 2 | On the Motor Pi: `legacy.yaml`, `uplink: true`, then `~/aav-jazzy/aav.sh build`, restart the bridge | the bridge logs `uplink: frames from the ESP32` |
| 3 | Steps U1-U6, then R1-R8, in [the bench page](esp32-bench.md) ("The uplink and the RC modes") | as each step says |

Either is no: stay on `spine-esp32-umod-v2.elf` with `uplink: false`; the car drives as
before, without the positions coming back.

**Stop point:** 1-14 and 9a-9c pass, or the C++ is back and the reason is written down.

## A9 · Session 9: the keyboard, on stands {#session-9-the-first-drive-on-stands}

**Who:** the safety driver, the e-stop person, the laptop person. **How long:** 45 min.
🔒 H1 and S1-S3. **Wheels off the ground**, the speed capped at **0.3 m/s**.

| ☐ | Before |
|---|---|
| ☐ | all four wheels off the ground (session 1) |
| ☐ | the e-stop person ready; everyone knows what it cuts (S1) |
| ☐ | theirs stopped: `aav_serial_bridge` inactive. **Never two drive paths** |
| ☐ | which ESP32 firmware: the Rust (session 6 passed). The C++ (**no timeout**) only for 3.1-3.8, never the joystick or the ground |
| ☐ | `legacy.yaml`'s `frame_version`: 2 with the Rust, 1 with the C++ |
| ☐ | A6 done: the steering's ends on the sheet (3.0). A7 done: the brake holds |
| ☐ | with the C++ (1): the bridge's log says `no-timeout lock ON`. twist_mux then holds every program (scripted, point, navigation, AI); the keyboard and the actuator tests below still drive. Leave `allow_programs_without_timeout: false` |
| ☐ | everyone knows: **the throttle's strength is a guess.** Two guesses disagree by 5 times ([the car's model](vehicle-model.md): full throttle holds 15 m/s by the drag guess, 3 m/s by the ESP32's open loop). 0.3 m/s asked may spin the wheels much faster: the e-stop person watches 3.2 |

| Where | Run |
|---|---|
| the Motor Pi | `~/aav-jazzy/aav.sh car motor_pi` |
| the Orin | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin max_speed:=0.3 gps:=false lidar:=false stands:=true` |
| the laptop, each terminal | `export ROS_DOMAIN_ID=42`, then `pixi run keyboard`, and `pixi run hud` |
| the Orin, a second terminal | `~/aav-jazzy/aav.sh tools/record.sh --lidar stands` (session 10) |

The HUD's GO / STOP panel says whether the car may move, and why not. A latched gate:
`pixi run clear gate`; a latched watchdog (such as `hud LOST`): `pixi run clear watchdog`.

| # | Test | Expect | If not |
|---|---|---|---|
| 3.1 | Nothing pressed | wheels still; the HUD shows 0 | e-stop |
| 3.2 | Forward, gently | the drive wheels turn **forward**, slowly | backwards: write it down (`legacy_protocol.py`), not fixed at the car. **Fast:** e-stop, write down the speed (3.12's tachometer), and no ground driving until tests 4.2 and 4.4 |
| 3.2b | Backward, gently (the keyboard's back key) | the drive wheels turn **backward**, slowly; the HUD's speed reads negative | forward: the ESP32's direction switch (ask the hardware lead). **Nav2 must not reverse on the ground until this passes:** `reversing:=false` |
| 3.3 | Left | the front wheels turn **left** | right: `steering_sign` (Q2) |
| 3.4 | Let go of the key | throttle off within 0.5 s (the QEMU rehearsal: 0.4 s), then the hold brake | |
| 3.5 | The HUD's e-stop button | brakes at once; nothing moves until `pixi run clear estop` | |
| 3.6 | Stop the keyboard program | the bridge brakes: its commands go stale | |
| 3.7 | Ctrl-C the Motor Pi's bridge | a last stop frame | |
| 3.8 | The Kar-Tech e-stop | everything stops | |
| 3.9 | Unplug the Motor Pi's serial cable while driving | the Rust: brakes within 200 ms. The C++: it doesn't (S3) | |
| 3.0 | The steering actuator's ends and centre | done in [A6](#a6-the-steering-actuators-ends-way-a) | |
| 3.10 | **Only inside the measured ends** ([A6's rule](#a6-the-steering-actuators-ends-way-a)): `pixi run actuator-test steer-sweep --stands` (the laptop) | it asks each wheel angle: the protractor | the ends inside 400-3500: skip it, write why |
| 3.11 | `pixi run actuator-test steer-centre --stands` | under 1°, or the offset written down | |
| 3.12 | Wheel speed: `/vehicle/odom` against a phone tachometer app | within 10% | |
| 3.P | `pixi run actuator-test preflight --stands --no-drive --skip steering gps lidar` (the Orin, ssh + tmux) | every sensor line PASS; nothing moves | a FAIL names the sensor: its driver, its cable, its rate |

**The preflight's drive** (speed up, hold, brake, steer, every sensor at once) is refused
on the car today: the wheel sensors are on the front, the motor drives the rear, so on
stands nothing measures the spinning wheels (TODO 49b). Once a rear-wheel speed exists
(`vehicle.yaml` `speed_wheels: rear`): `pixi run actuator-test preflight --stands --skip steering gps lidar`,
with the e-stop person and someone on the rear wheels' tachometer ([the preflight](actuator-tests.md#the-preflight-on-stands)).

**The ground** comes only through [A10](#a10-the-ground-at-walking-pace): walking pace, signed. Tests 4.2 (`coast`)
and 4.4 (`throttle-map`) then measure the drag and the throttle, and settle the 5 times
disagreement (TODO 52b): [Every test](tests.md#stage-4-the-car-in-an-empty-lot), stage 4, recorded with the lidar (4e).

Next: the joystick, [A9b](#a9b-the-joystick-on-stands). At the end of the day, put it back: Ctrl-C everything; theirs again (`sudo systemctl start aav_serial_bridge`)
only with the C++ on the ESP32 (6d), as the team decides.

**Stop point:** 3.1-3.8 and 3.2b pass. Anything that turned the wrong way: written down.

## A9b · The joystick, on stands

Ben, 1 Nov: "We do still need a way to move the vehicle to get to our first test. Should
we allow the joystick to have direct control like before? ... if the connection drops we
stop?" **Yes, through our stack, all on the Motor Pi** (`machine:=motor_pi_alone`): the
heartbeat is made beside the ESP32, so the ESP32 brakes when the Pi, the bridge or the
wire stops. No Orin needed.

```text
joystick (USB, on the Pi) ─► joy_alive ─► teleop ─► twist_mux ─► ego ─► output mux ─► legacy bridge ─► ESP32
                             silent 0.5 s  hold-to-   0.5 s       0.3 s   the heartbeat   0.3 s           brakes 100 ms
                             frozen 2 s    drive      timeout     timeout counter         stale→brake     after it stops
```

**The joystick-alive check** (`joy_alive`, TODO 74a) stops the joystick when it goes
silent for 0.5 s (receiver unplugged) or its values freeze for 2 s while driving (a dead
pad repeated by its receiver). It latches: release the trigger, press again.
`/drivers/joy/alive` says `OK`, `WAITING`, `SILENT` or `FROZEN`, and why.

**The Rust firmware only** (A8 passed): the C++ has no timeout.

| # | Do | You should see |
|---|---|---|
| 1 | Stands; the Kar-Tech stop in the e-stop person's hand. The Rust on the ESP32, `frame_version: 2` | |
| 2 | Theirs stopped: `aav_serial_bridge` inactive. The Orin **not** running `car orin` (two muxes would both drive) | |
| 3 | On the Motor Pi: `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi_alone max_speed:=0.3` | |
| 4 | The buttons and sticks: `~/aav-jazzy/aav.sh ros2 topic echo /drivers/joy/joy` (deadman: button 0; throttle: axis 1; e-stop: button 2) | the numbers match; write down any that don't (TODO 74d), and change `teleop_twist_joy.yaml` and `joy_alive.yaml` together |
| 4b | A second terminal: `~/aav-jazzy/aav.sh ros2 topic echo /drivers/joy/alive` | `WAITING`, then `OK - idle` once the trigger was released |
| 5 | Session 9's tests 3.1-3.9 with the joystick | as there |
| 6 | **The joystick-off test:** hold the trigger, a little throttle, then **turn the joystick off** (its receiver still in) | the wheels stop within 3 s; `alive` says `SILENT` or `FROZEN` ([tests 3.J1-3.J3](tests.md#3a-joy-the-joystick-on-stands)). They keep going: **e-stop**, and no ground with the joystick |
| 6b | **Unplug the receiver** while driving, plug it in with the trigger held | stops within 1 s (`SILENT`); stays stopped until the trigger is released and pressed |
| 7 | **Hold the stick still** 10 s, then pinned fully forward 10 s | write down when `FROZEN` comes. Pinned trips it after 2 s (release, press again). A still hand that trips it often: raise `frozen_after`; a pad whose receiver goes silent when off: `frozen_after: 0.0` is allowed |

### What keeps it safe

| Rule | With the Rust ESP32 | With the 2025-26 C++ |
|---|---|---|
| Hold-to-drive: let go of the trigger, it stops | ✅ teleop_twist_joy, one zero on release | ✅ |
| The joystick's USB receiver unplugged | ✅ no commands: twist_mux 0.5 s, ego 0.3 s, the bridge brakes | ✅ |
| **The joystick turned off or out of range, its receiver still in** | ✅ on the bench: `joy_node` repeats the last state 20 times a second, so `joy_alive` stops it when the values freeze for 2 s while driving (`FROZEN`) or stop coming for 0.5 s (`SILENT`). **Step 6 tests the real pad** | ✅ the same (it's in ROS) |
| The Pi, the bridge or the serial wire stops | ✅ brakes 100 ms after the heartbeat stops (QEMU: 111-120 ms) | ❌ the last throttle stays: only the e-stop |
| The network drops | ✅ nothing on the network: all on the Pi | ✅ |
| A speed cap | ✅ ego's `max_speed:=0.3`, on the Pi | ✅ the same |
| A **throttle cap** in the firmware | ⚠️ only the throttle's top (DAC 700 of 1023). The ESP32 can't see the speed, so it can only cap the throttle (TODO 74b) | ⚠️ the same |
| The joystick's e-stop button | ✅ `/safety/estop`: the bridge brakes | ✅ |
| The RC transmitter turned off | ✅ brakes within 0.1 s | ❌ no RC timeout |
| **The Kar-Tech stop** (the main stop) | ✅ hardware: what it cuts is S1 | ✅ |

The 2025-26 path ("like before": `joy_to_ackermann` → `aav_serial_bridge` → the C++) has
the deadman but **no timeout anywhere**: we don't use it.

**Stop point:** 1-7 pass, or why not written down.

## A10 · The ground, at walking pace

**Who:** all five people. The car comes off the stands only when every box is ticked.

| ☐ | Before |
|---|---|
| ☐ | A6 (the steering's ends), A7 (the brake), A8 (session 6c, 9a-9c) passed |
| ☐ | Session 9: 3.1-3.9 and 3.2b passed |
| ☐ | A9b passed, **the joystick-off test too** (or only the keyboard drives) |
| ☐ | An empty lot; the safety driver in the seat; the e-stop person beside it, the Kar-Tech stop in hand |
| ☐ | `max_speed:=0.3`; recording (`record.sh --lidar ground`) |

`✍ Signed (approver): ________ at __:__ · Hardware lead: ________`

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Forward 2 m, at walking pace, then let go | it stops by itself | Kar-Tech stop |
| 2 | The Kar-Tech stop, rolling slowly | it stops; the distance written down | no more ground today |
| 3 | 10 m straight, slowly | it drives straight (or the pull written down) | |

Then the car tests and the map: [Know the car](know-the-car.md) (A11), bagged with the lidar.

## A12 · The diffusion planner's trial (0.3) {#a12-the-diffusion-planners-trial}

Ben, 7 Nov: "We want diffusion." Allowed on the car in 0.3 as a **trial**: low speed,
behind the guard and the locks. **Not run yet.** Its own day, after A11 and after
the lattice has driven the same goals on the ground ([4f](tests.md#4f-reversing-and-the-u-turn)).

**Who:** the safety driver, the e-stop person, the laptop person, the approver.
**How long:** 90 min. 🔒 A10 passed; 4.23-4.26 passed with the lattice.

### What holds it

| Layer | What | Changed for the trial? |
|---|---|---|
| The switch | `diffusion_trial:=true`; without it `planner:=diffusion` is **refused** on the car | new |
| The speed | **0.5 m/s at most** for ego, navigation and the geofence; `max_speed:=` above 0.5 is **refused** | new |
| The guard | G1-G8 on every path the model draws; the lattice's path if none passes, or after 150 ms | no |
| Below it | MPPI, collision_monitor (front and back), twist_mux's locks, ego, the geofence, the watchdog, the spine | **no** |
| A person | the joystick or keyboard outranks navigation; the Kar-Tech stop | no |

Why 0.5 m/s: walking pace; the car's own reverse limit, so forwards and backwards alike;
at ego's 2 m/s² it stops in 6 cm, plus 15 cm for a 0.3 s stale command. Half of ego's
own 1.0 m/s cap, never above it.

### Before: the Orin's timing (no driving)

| # | Do (on the Orin) | You should see | If not |
|---|---|---|---|
| 1 | Copy `models/diffusion/planner.onnx` (and `planner_fp16.onnx`) from the laptop into `~/aav-jazzy/aav/models/diffusion/` | the files there | `pixi run diffusion train` on the laptop first |
| 2 | Make the model's Python (below): NVIDIA's ONNX Runtime, which sees JetPack's TensorRT | `~/ort-venv/bin/python -c "import onnxruntime as o; print(o.get_available_providers())"` lists `TensorrtExecutionProvider` | no internet, or it fails: no trial today (the car kit has no ONNX Runtime) |
| 3 | The CPU: `cd ~/aav-jazzy/aav && ~/ort-venv/bin/python ros/drivers/aav_diffusion_planner/training/measure.py` | a table: crop, model, guard, total against 2, 50, 10, 100 ms | write the table down |
| 4 | The GPU: the same with `--provider TensorrtExecutionProvider` | the first start builds TensorRT's engine (minutes) into `models/diffusion/engines/`, where the node finds it; then the table, `ran on TensorrtExecutionProvider` | `ran on CPUExecutionProvider`: no GPU in that ONNX Runtime; try `--provider CUDAExecutionProvider` |
| 5 | Pick: the total's 95th percentile under 100 ms, and the worst under 150 | write down the engine; the Python is `~/ort-venv/bin/python` | over 150 on every engine: **no trial**; every plan would be the lattice's anyway |

The model's Python, once (needs internet; the index is NVIDIA's Jetson AI Lab, as for
[4d](#4d-rf-detrs-tensorrt-engine)):

```bash
python3 -m venv --system-site-packages ~/ort-venv
~/ort-venv/bin/pip install numpy onnxruntime-gpu --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

(JetPack 7.2 after the reflash: that index's `jp7` folder, if NVIDIA has one; check first.)

### On stands

| ☐ | Before |
|---|---|
| ☐ | all four wheels off the ground; the e-stop person ready |
| ☐ | the Orin's timing above done; the engine written down |

| Where | Run |
|---|---|
| the Orin | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin diffusion_trial:=true stands:=true diffusion_provider:=<engine> diffusion_python:=$HOME/ort-venv/bin/python` |
| the laptop | `pixi run hud`; `ros2 topic echo /drivers/nav/diffusion/status` |

| # | Test | Expect | If not |
|---|---|---|---|
| D.1 | The same command **without** `diffusion_trial:=true` but with `planner:=diffusion` | refused at once: "a trial: on the car only with diffusion_trial:=true" | it starts: **stop**, no trial |
| D.2 | With `max_speed:=0.8` | refused: "at most 0.5 m/s" | it starts: **stop**, no trial |
| D.3 | The launch log | within a minute: `diffusion_planner: 16 samples, budget 150 ms, ... worker up` | `the lattice plans alone`: every plan the lattice's (the trial proves nothing): fix the model or Python |
| D.4 | A goal 10 m ahead in RViz | the wheels turn forward, never over 0.5 m/s (the HUD) | faster: **e-stop** |
| D.5 | `status` while it plans | `source: model` or `lattice`, and why; `model_and_guard_ms` under 150 | every plan `late`: the engine is too slow |
| D.6 | The HUD's e-stop, then the joystick's trigger | brakes at once; the person drives | |

### On the ground

| ☐ | Before |
|---|---|
| ☐ | the stands passed (D.1-D.6) |
| ☐ | an empty lot, cones for a 14 m lane; the safety driver in the seat; the e-stop person beside it, the Kar-Tech stop in hand; a person on the joystick |
| ☐ | recording: `tools/record.sh --lidar diffusion` |

`✍ Signed (approver): ________ at __:__`

| # | Test | Expect | If not |
|---|---|---|---|
| D.7 | A goal 15 m straight ahead | it drives there at walking pace; `status` says how many plans came from the model | swerves or stops short: Kar-Tech stop, write down `status` |
| D.8 | A goal round one cone | it drives round it, no cone touched | |
| D.9 | The U-turn: a goal behind the car in the cone lane (4.26) | forward, back, forward; never over 0.5 m/s; no cone touched | |
| D.10 | The same goals with the lattice (no `diffusion_trial`) | the times, to compare | |

**Stop point:** D.1-D.3 pass on stands, or no trial. Any cone touched: stop, and the
record goes to the team before another run.

## Any time on Day A, after A5

These three need our stack (A5) and nothing else. One person each, beside the main line.

## Session 5: the camera Pi

**Who:** one person, with the laptop. **How long:** 60 min. **Bring:** the checkerboard,
2 phones. 🔒 C1: which address. The plan and the settings: [camera pipeline](camera-pipeline.md).

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `ssh <user>@192.168.1.108`; `hostname; ip -br addr` | `aav-cam1`; its addresses (.108, .102?) | try .102. Write down which is wired |
| 2 | Keep theirs: `systemctl status mediamtx`, then `sudo cp /usr/local/etc/mediamtx.yml /usr/local/etc/mediamtx.yml.2025-26` | the service, and its file's place | the file is elsewhere: `status` names it; use that path |
| 3 | `rpicam-hello --list-cameras` | 2 cameras, 0 and 1 | one: ribbon loose? (N4). USB? `v4l2-ctl --list-devices` |
| 4 | Ours, from the laptop: `scp ros/sources/aav_camera_stream/pi/mediamtx.yml <user>@192.168.1.108:/tmp/`, then `ssh <user>@192.168.1.108 'sudo cp /tmp/mediamtx.yml /usr/local/etc/mediamtx.yml && sudo systemctl restart mediamtx'` | `systemctl status mediamtx`: active | put theirs back (step 2's copy) |
| 5 | Which is left: in a browser, `http://192.168.1.108:8889/front_left`; cover the **left** lens with a hand | the picture goes dark | the right one went dark: swap `rpiCameraCamID` 0 and 1 in the file, step 4 again |
| 6 | The Orin decodes both: `~/aav-jazzy/aav.sh ros2 launch aav_camera_stream camera_stream.launch.py host:=192.168.1.108 decoder:=gst-nvidia detector:=none` | `~/aav-jazzy/aav.sh ros2 topic echo /diagnostics`: `fps_in` 15 for each | no `/usr/bin/gst-launch-1.0`: JetPack's GStreamer is missing, write it down |
| 7 | The Pi's CPU, both streams on: `top` on the Pi, then `vcgencmd measure_temp` | under 50% of the Pi; under 80 °C | write down the numbers anyway |
| 8 | Bitrate: `curl http://127.0.0.1:9997/v3/paths/list` on the Pi, twice, 10 s apart | `bytesSent` up about 2.5 MB each 10 s (2 Mbit/s) | |
| 9 | **The delay:** a stopwatch on phone 1 in front of the camera, the browser view beside it; phone 2 films both | the difference between the two times: under 150 ms. Do 3 photos | write down what you got |
| 10 | **Calibration:** the steps below, for `front_left`, then `front_right` | `ros/sources/aav_camera_stream/config/calibrations/front_left.yaml` (and `front_right`) written; commit them at home | no picture: the stream (step 6); save pictures as a backup (below) |

Step 10, calibration:

1. The printed checkerboard: 9x6, 25 mm squares (`pixi run calibrate-camera board`).
   Measure one square with the ruler.
2. On the laptop: `pixi run calibrate-camera front_left --host 192.168.1.108 --square <measured, in m>`.
3. Someone moves the board slowly: near, far, the corners, tilted. Wait until the X, Y,
   Size and Skew bars go green.
4. CALIBRATE, then SAVE. Then the same for `front_right`.
5. Backup, if the window shows no picture: `mkdir -p ~/aav-calib`, then
   `pixi run ffmpeg -rtsp_transport tcp -i rtsp://192.168.1.108:8554/front_left -vf fps=1 -frames:v 30 ~/aav-calib/front_left_%02d.png`

At home: the delay goes in `orin.yaml`'s `camera_stream` `delay` (0.06 s today); commit
the two calibration files (`docs/camera-calibration.md`). Until they're in, the stop-sign
rule uses the simulator's lens and the car stops too early.

**Stop point:** both streams at 15 fps on the Orin, the delay measured, both cameras calibrated.

## Session 7: CAN, read-only

**Who:** one person, on the Orin. **How long:** 30 min. **Bring:** a multimeter.
🔒 H3, H4, H7: the bitrate, the interface, what's on the bus. We send nothing.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Power off: across CAN-H and CAN-L, the multimeter | about 60 Ω (120 Ω at each end) | 120 Ω: one end only; open: none. Tell the hardware lead |
| 2 | `ip -br link \| grep -i can` | `can0` (and maybe `can1`) | none: a USB adapter? (H7) |
| 3 | `sudo ip link set can0 up type can bitrate 500000` | no error; `ip -details link show can0`: `ERROR-ACTIVE` | the bitrate (H7) |
| 4 | `candump can0` (if can-utils is there) | `can0  200  [8] ...` and `201`, 100 a second each | nothing: the Arduino's power, the wiring, H4 |
| 5 | Or ours: `~/aav-jazzy/aav.sh ros2 launch aav_spine spine.launch.py interface:=can0 role:=sensors`, then `~/aav-jazzy/aav.sh ros2 topic echo /vehicle/wheels` | spin a front wheel by hand: its speed | `/can/rx` has frames but no wheels: the CRC or counter (H4) |
| 6 | `ip -details -statistics link show can0` | errors 0 | write the counts down |
| 7 | `sudo ip link set can0 down` | | |

`role:=sensors` can't drive: no sender, no arm, no e-stop path ([CAN plan](can-plan.md)).
If other boards are on the bus, `listen-only on` after `bitrate 500000` stops even the
Orin's acknowledgements; with the Arduino alone, leave it off (someone must acknowledge).

**Stop point:** both wheels' frames seen, or the reason written down.

## Session 8: Foxglove from a laptop

**Who:** anyone with a laptop. **How long:** 15 min. **Bring:** Lichtblick installed.
Their `foxglove-bridge` already holds port 8765 on the Orin, so ours goes on 8766.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Theirs, read-only: `pixi run python3 tools/layout_check.py ws://192.168.1.140:8765` | their topics (Humble, domain 0); ours aren't there | |
| 2 | Ours, on the Orin: `~/aav-jazzy/aav.sh ros2 launch aav_bringup foxglove.launch.py foxglove_address:=0.0.0.0 foxglove_port:=8766 foxglove_control:=false` | `listening on port 8766` | |
| 3 | `pixi run python3 tools/layout_check.py ws://192.168.1.140:8766 --data` | every layout topic of the running parts served, and kB/s each | missing ones: their part isn't running |
| 4 | Lichtblick → Open connection → Foxglove WebSocket → `ws://192.168.1.140:8766`; Layouts → Import → `tools/foxglove/driving.json` (then the other two) | the car's model, the gate, the e-stop | [Watch from a laptop](foxglove.md) |
| 5 | The same over the car's WiFi | under 2.5 MB/s for every laptop together | the WiFi drops multicast? ([Friday](history/friday-2026-10-16.md)) |

The map panel needs the internet for its tiles: offline it shows the dots on a blank map.

**Stop point:** a layout open on a laptop.

## Session 10: what to record

**Who:** the laptop person. **All day.** These feed the sim-versus-real report (PLAN T2.7),
the sensors' real noise (T2.4) and the replay (T2.5). Bags stay out of git.

| # | Record | How | For |
|---|---|---|---|
| 1 | **Parked, 5 min**: the IMU and the GPS, with their drivers (Humble, so on their Pi; it only writes `~/aav-bags`) | on LOCALIZATION: `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/parked /imu/data_raw /imu/mag /aav/gps/fix` | the IMU's bias, the GPS's noise over 10 s (TODO 40) |
| 2 | **The lidar, 1 min**, parked and with someone walking past | on the Orin, theirs: `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/lidar /velodyne_points /scan` | the lidar layer, ghost obstacles |
| 3 | **Every session of ours**, stands too (A6-A11) | on the Orin: `~/aav-jazzy/aav.sh tools/record.sh --lidar <session>` | ego and the actuator model against real; the lot's map |
| 3b | **A mapping drive**, by hand, round the lot | on the Orin: `~/aav-jazzy/aav.sh tools/record.sh --mapping --name lot` ([Recording a mapping drive](recording.md)): checks first, a summary after; up to 87 GB an hour | the lot's NDT and SLAM maps, labels, the transfer functions |
| 4 | The actuator tests | they save themselves: `~/aav-tests/` on the laptop | `vehicle.yaml`'s steering numbers |
| 5 | **Every topic's size and rate**, our stack running | on the Orin: `~/aav-jazzy/aav.sh bandwidth --markdown` | the bandwidth budget (T1.7) |
| 6 | The camera delay photos, the calibration pictures, the sheet | the phones; `~/aav-calib/` | session 5; Gate 1 |
| 7 | The ESP32's log during 6c | copy the `espflash` terminal to a file | Gate 2's evidence |

Copy home: `rsync -a <user>@192.168.1.140:aav-jazzy/aav/bags/ ~/aav-bags/jetson/`, and
`~/aav-bags/` from LOCALIZATION and the Orin the same way.

## Day B: upgrades and the reflash

**Not before Day A is done, and a restore is proven (B1).** Each machine as
[Car setup](car-setup.md) says; this page says the order. Every step is ✍: an upgrade
ends the 2025-26 software on that computer, and the backup is the only way back.

| Step | What | Before it | ✍ |
|---|---|---|---|
| B1 | **A restore, proven**: one Pi's card image written to a **spare** card (Raspberry Pi Imager → Use custom), booted in that Pi | it boots, answers `ssh`, and runs as before. Then its own card back in | `✍ ________ at __:__` |
| B2a | The dash: `upgrade` ([Car setup, step 2](car-setup.md#step-2-the-ubuntu-2204-pis-to-2404)) | its card image GOOD (A2) | `✍ ________ at __:__` |
| B2b | The camera Pis: `apt full-upgrade` only ([step 3](car-setup.md#step-3-the-pis-on-raspberry-pi-os)) | their card images GOOD | `✍ ________ at __:__` |
| B2c | The Motor Pi: `upgrade --after-esp32` | the Rust ESP32 passed session 6; its card image GOOD | `✍ ________ at __:__` |
| B2d | LOCALIZATION: `upgrade` ([step 2](car-setup.md#step-2-the-ubuntu-2204-pis-to-2404)). Its machine file exists since 8 Nov (`machines/localization.yaml`), so `car_setup.sh` no longer refuses | its card image GOOD; our IMU and GPS drivers seen running from the kit | `✍ ________ at __:__` |
| B3 | The Orin: its full image, then the reflash | the box below | `✍ ________ at __:__` |
| B4 | The lab desktop: its update | B3 done | `✍ ________ at __:__` |

## B3 · The Orin: its image, then the reflash

| ☐ | Before starting |
|---|---|
| ☐ | **JetPack 7.2 supports Jetson AGX Orin**: NVIDIA's 7.2 release notes, read today. The result, and where it says so: ________ (no: stop, stay on JetPack 6) |
| ☐ | Which Orin (W7: 32 or 64 GB), and its module's part number ([Platforms](platforms.md#moving-the-orin-to-jetpack-72)) |
| ☐ | SDK Manager's host Ubuntu for 7.2: the lab desktop has it, **as it is now** (it stays frozen) |
| ☐ | The hardware lead's OK |

1. The full disk image: [session 11b](#11b-in-the-lab-the-orins-full-image), on the lab desktop.
2. Its restore rehearsed, or at least the image's size and files checked.
3. The reflash: [Car setup, step 6](car-setup.md#step-6-the-orin-to-jetpack-72).

## B4 · The lab desktop stays frozen

The lab desktop is the Orin's flashing host, and holds its image. **Don't upgrade its
OS until the Orin's reflash is done** and the Orin runs: an upgrade could stop SDK Manager
or `l4t_backup_restore.sh` working. Then: [Car setup, step 9](car-setup.md#step-7-the-lab-desktop).

## Session 11: the lab's Ubuntu desktop

The team's x86 Ubuntu desktop with the NVIDIA card **stays in the lab**: it already runs
CARLA. It's also the one computer that can take a **full disk image of the Orin** (a Mac
can't). At the drive, Ben's MacBook is the ground station: it does everything else here.
**Its OS stays as it is until the Orin's reflash is done** ([B4](#b4-the-lab-desktop-stays-frozen)).

| Job | Where | When |
|---|---|---|
| **Rehearse car day in CARLA** | the lab | the week before (11a) |
| **The Orin's full disk image** (`l4t_backup_restore.sh`) | the lab, the Orin beside the desktop | Day B, before the reflash (11b) |
| Reflash the Orin with SDK Manager (Q24) | the lab | Day B, [B3](#b3-the-orin-its-image-then-the-reflash): after the image, with the hardware lead's OK |
| Record, replay, build NDT's map at the drive | Ben's MacBook | the day (11c) |

### 11a. In the lab, the week before

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | Which CARLA: `docker images \| grep carla`, or the folder's name (`CarlaUE4` = 0.9.x, `CarlaUnreal` = 0.10); `lsb_release -d; nvidia-smi` | the version, Ubuntu's, the GPU and its memory: write them in the machines table | 0.10 needs 16 GB of video memory; Carleton's map needs 0.9.16 ([CARLA](carla.md)) |
| 2 | Clone ecolo-av, then `pixi run build` and `pixi run test` | all green | as on any laptop ([Getting started](getting-started.md)) |
| 3 | CARLA's server, headless ([CARLA](carla.md), steps 1-2), its Python client the same version | the server listens on 2000 | the client must match the server |
| 4 | **The rehearsal:** `pixi run carla-car host:=127.0.0.1`, then the car day's own steps: the keyboard, a scripted drive, a goal, a goal behind (the U-turn) | the car drives each, in CARLA | write down what failed: fix it before the day |

### 11b. In the lab: the Orin's full image

Only with the Orin in the lab beside the desktop, and the hardware lead's OK: the Orin
reboots into recovery mode and is off for the whole copy. Not possible at the drive.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | NVIDIA's L4T BSP **matching the Orin's release** (session 4a, step 2: R36.4.x), unpacked on the desktop; then `sudo ./tools/l4t_flash_prerequisites.sh` in it. Read `tools/backup_restore/README` there | `l4t_backup_restore.sh` | NVIDIA's Jetson Linux archive |
| 2 | Room for the image: the Orin's disk size plus 10 % | free space, or an external drive | |
| 3 | The home folder and `/etc` copied first (session 4a, 4-5) | done | |
| 4 | USB-C from the desktop to the Orin's flashing port; the Orin into **recovery mode** (its buttons: the dev kit's guide) | `lsusb` shows NVIDIA (`0955:`) | a data cable, the right port |
| 5 | `sudo ./tools/backup_restore/l4t_backup_restore.sh -e <disk> -b jetson-agx-orin-devkit` (`<disk>`: `mmcblk0` built-in, `nvme0n1` an SSD: 4a, step 3's `df`). The README is the authority | every partition copied (30-90 min) | the BSP's version must match the Orin's |
| 6 | Reboot the Orin normally; `ssh` in; the image onto the encrypted backup drive too | it boots as before; two copies | the hardware lead |

### 11c. At the drive: Ben's MacBook

| # | Do | For |
|---|---|---|
| 1 | Copy each recording off the Orin as it ends (session 10's `rsync`) | the recordings in two places |
| 2 | After the lot's recording (tests.md 4.19): `pixi run ndt-map build ... --poses /localization/odom/global` | **NDT's map before leaving**: if it blurs, record again while the car is there |
| 3 | Back in the lab: play the recordings on the desktop too | its screen, and CARLA beside it |

## Before you leave the car

| ☐ | Do |
|---|---|
| ☐ | Ctrl-C all our programs; `sudo ip link set can0 down` |
| ☐ | The ESP32: the Rust or the C++, as the team decided; `legacy.yaml`'s `frame_version` to match |
| ☐ | Theirs back the way they were (`aav_serial_bridge`, the camera Pi's file if ours went wrong); check they drive as before |
| ☐ | `~/aav-jazzy` stays for next time (or `rm -rf ~/aav-jazzy`: that undoes everything) |
| ☐ | The laptop: `rm ~/.config/aav/cyclonedds.xml` (the car file stays as `cyclonedds-car.xml`) |

## Afterwards, at home

| Do | Where |
|---|---|
| The write-up, no secrets | `docs/history/audit-<date>-car.md`, like [21 Oct's](history/audit-2026-10-21-car.md) |
| The numbers | `vehicle.yaml`; then `pixi run test` and `pixi run view` |
| The car's linear model, refit | `pixi run python3 tools/vehicle_model.py`: new poles and margins from the measured lags ([The car as a linear model](vehicle-model.md)) |
| The answers | `docs/audit-questions.md`; the hardware lead's in `docs/can-plan.md` |
| Gate 1 and Gate 2 | [The plan](plan.md) |
