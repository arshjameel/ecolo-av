# Car day: the runbook

Written 24 October 2026, for the team. Ben, 24 Oct: "Lets get everything prepared so the
only thing left is to get into the car." Everything that can be done without the car is
done or built; this page is the rest, **in order**.

> **The short version**
> - **Before you leave:** `pixi run car-day-build`, then `pixi run car-day-check` until it says **0 missing**.
> - **At the car:** ten sessions, in order. Each says who, how long, what to bring, the commands, what you should see, and what to do if not.
> - **Wheels off the ground all day.** Nothing drives on the ground.
> - 🔒 = waits on the hardware lead's answer (session 3). Skip it until then, and go on.
> - **Stop at each stop point.** Something failed? Write it down, then go to the next session that doesn't need it.
> - **Secrets:** the logins are in Ben's Mac's Keychain. Type them; never paste them into a file, a chat or the repo.

## The day at a glance

```text
 0 at home ──► 1 safety ──► 2 measure ──────────────────────────────────────► 10 record
                  │         3 questions (hardware lead) ──┐                    (all day)
                  └──► 4 the Orin (+ Motor Pi) ──┬──► 5 camera Pi
                                                 ├──► 6 ESP32 🔒 ──► 9 first drive, stands
                                                 ├──► 7 CAN 🔒
                                                 └──► 8 Foxglove
```

| # | Session | Who | How long | Needs | 🔒 Blocked on |
|---|---|---|---|---|---|
| 0 | [Before leaving](#session-0-before-leaving-at-home) | the laptop person | 30-60 min | internet | |
| 1 | [Safety first](#session-1-safety-first) | everyone | 15 min | | |
| 2 | [Measure and weigh](#session-2-measure-and-weigh) | 2 measurers | 90 min | 1 | due **30 Oct** (Gate 1) |
| 3 | [Questions for the hardware lead](#session-3-questions-for-the-hardware-lead) | the hardware lead + 1 | 30 min | | it answers the 🔒 |
| 4 | [The Orin (and the Motor Pi)](#session-4-the-orin-and-the-motor-pi) | the laptop person | 1-3 h (the backup runs on its own) | 1 | |
| 5 | [The camera Pi](#session-5-the-camera-pi) | 1 + the laptop | 60 min | 4 | which Pi (.108 or .102) |
| 6 | [The ESP32](#session-6-the-esp32) | laptop person, e-stop person, hardware lead | 90 min | 1, 4 | H1, Ben's OK (on a call) |
| 7 | [CAN, read-only](#session-7-can-read-only) | 1 | 30 min | 4 | H3, H4, H7 |
| 8 | [Foxglove from a laptop](#session-8-foxglove-from-a-laptop) | anyone | 15 min | 4 | |
| 9 | [The first drive, on stands](#session-9-the-first-drive-on-stands) | safety driver, e-stop person, laptop person | 45 min | 1, 4, 6 (or the C++) | H1, S1-S3 |
| 10 | [What to record](#session-10-what-to-record) | the laptop person | all day | 4 | |

## The machines

As the 21 Oct audit found them ([audit](history/audit-2026-10-21-car.md)).
Usernames: `~/aav-audit/hosts.txt` on Ben's Mac. Passwords: the Keychain entry (Keychain
Access → search the entry → Show password). Every address is `192.168.1.x`.

| Machine | Address | Keychain entry | Runs today (2025-26, Humble, domain 0) |
|---|---|---|---|
| JETSON, **the Orin** | .140 | `aav-jetson` | the Velodyne driver (UDP 2368), lidar SLAM, `foxglove-bridge` (port 8765) |
| CORTEX, **the Motor Pi?** (H1) | .120 | `aav-cortex` | `aav_serial_bridge` on `/dev/ttyAMA0` (to the ESP32), joystick teleop |
| CAM1-PI, **the front cameras** | .108 (also seen at .102) | `aav-cam1` | MediaMTX, RTSP on 8554 |
| LOCALIZATION | .113 | `aav-localization` | the IMU (200 Hz) and GPS (10 Hz) drivers |
| CAM2-PI | .118 | `aav-cam2` | MediaMTX with a USB camera |
| the ESP32 | .132 (WiFi) | none | the 2025-26 C++ firmware: **no command timeout** |
| the Velodyne | .201 | none | sends to the Orin |

Our stack runs in **ROS domain 42**, so it never mixes with theirs (domain 0).

## What to bring

| ☐ | Thing | For |
|---|---|---|
| ☐ | the laptop, after session 0, and its charger | everything |
| ☐ | the USB stick with `car-kit/` | session 4 |
| ☐ | USB-C Ethernet dongle, 2 Ethernet cables, a USB-C hub | the car's switch |
| ☐ | a USB data cable for the ESP32 board's USB port (check its plug before) | session 6 |
| ☐ | **stands** for all four wheels, wheel chocks | sessions 1, 6, 9 |
| ☐ | the **RC transmitter**, charged | sessions 1, 6 |
| ☐ | the printed sheet, [Measure the car](measure-the-car.md), 2 copies, pens | session 2 |
| ☐ | tape measure, masking tape, string, a plumb line, a phone level app | session 2 |
| ☐ | **4 bathroom scales** (or 1 and blocks), 2 blocks 20-30 cm | session 2 |
| ☐ | a protractor | sessions 2, 9 |
| ☐ | a multimeter | sessions 6, 7 |
| ☐ | a printed checkerboard on a stiff board (9 x 6 inner corners, 25 mm squares) | session 5 |
| ☐ | 2 phones: a stopwatch, and one to film | session 5 |
| ☐ | the backup drive (encrypted: the backups hold passwords) | sessions 4, 6 |

## Session 0: before leaving (at home)

**Who:** the laptop person. **How long:** 30-60 min. **Bring:** internet.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `git fetch upstream && git switch main && git merge --ff-only upstream/main` (the team's main: [CONTRIBUTING.md](../CONTRIBUTING.md)), then `pixi run build` | `Summary: ... packages finished` | `pixi run build` again; the end says which package |
| 2 | `pixi run get-models` (once) | `models/rf-detr-nano.pth`, `models/yolo11n.pt` | the internet |
| 3 | The ONNX for the Orin (box below) | `models/rf-detr-nano.onnx` (108 MB) | skip: session 4 has a slower way |
| 4 | `pixi run car-day-build` | `v2: PASS`, `v1: PASS`, `esp-qemu-check: PASS`, `legacy-esp32: PASS`, `docs: PASS` | each step's log is in `car-day/esp32/` |
| 5 | `pixi run car-kit`, then copy `car-kit/` to the stick | `car-kit/` (about 2 GB); `cp -R car-kit /Volumes/<stick>/` | [From a USB stick](car-install.md) |
| 6 | The laptop's Cyclone file (box below) | `~/.config/aav/cyclonedds-car.xml` | |
| 7 | `pixi run net-map` and `pixi run doctor` once | macOS asks about the local network: **Allow** | System Settings → Privacy → Local Network |
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

After step 4, before steps 1, 5 and 6: so the repo, the build and the kit are its 3
missing, and Cyclone a reminder. (The repo line's advice is today's wording.) No secrets: the Keychain is only asked whether each
entry exists.

```text
Car day: is /Users/ben/Code/ecolo-av ready? (car-day.md)

  MISSING  repo         main at 54304074 has commits GitHub hasn't
                        -> pixi run test, then move them to a branch and open a pull request (CONTRIBUTING.md)
  MISSING  build        ros/ changed since the last build (24 Oct 15:06)
                        -> pixi run build
  MISSING  car kit      made 2026-10-20 13:54; its code is 37a7b549, main is 54304074
                        -> pixi run car-kit
  REMIND   USB stick    no stick with car-kit/ plugged in
                        -> cp -R car-kit /Volumes/<stick>/
  OK       model        models/rf-detr-nano.pth (366 MB): the detector (RF-DETR Nano): the Orin builds its engine from it
  OK       model        models/yolo11n.pt (6 MB): the YOLO fallback (detector:=yolo)
  OK       model        models/rf-detr-nano.onnx (108 MB): the Orin's engine needs only TensorRT from it (trtexec)
  OK       MediaMTX     vendor/mediamtx (to test a stream on the laptop)
  OK       ESP32 tools  Xtensa Rust, espflash (vendor/esp, vendor/cargo-tools)
  OK       ESP32 Rust   v2 and v1 built 24 Oct 2026 15:47, in car-day/esp32/
  OK       ESP32 QEMU   esp-qemu-check passed 24 Oct 2026 15:47: brakes when the heartbeat stops
  OK       ESP32 C++    the 2025-26 C++ builds (the way back), 24 Oct 2026 15:47
  REMIND   ESP32 C++    secrets.h has the placeholders: a C++ built again can't join the WiFi; the backup (session 6) restores it all
                        -> firmware/legacy/aav_umod_esp32/src/secrets.h: the WiFi from the team, typed in, never committed
  OK       layouts      driving (20 topics), map (3 topics), sensors-health (6 topics); checked live at the car (session 8)
  OK       Lichtblick   /Applications/Lichtblick.app
  OK       docs PDF     docs/html/aav-docs.pdf, built 24 Oct 15:48
  REMIND   Cyclone      Cyclone DDS uses one network card: name the car's (the dongle) in ~/.config/aav/cyclonedds-car.xml
                        -> car-day.md, session 0
  OK       logins       7 of 7 in the Keychain: aav-cam2, aav-cortex, aav-jetson, aav-desktop, aav-localization, aav-cam1, aav-qnxpi
  OK       SSH key      ~/.ssh/id_ed25519.pub
  OK       disk         275 GB free for bags
  REMIND   sheet        print measure-the-car.md (2 copies) and a pen
                        -> session 2
  REMIND   kit bag      stands, the RC transmitter, the e-stop, tape, plumb line, 4 scales, blocks, multimeter, dongle, cable
                        -> car-day.md, "What to bring"
  REMIND   questions    the hardware lead is coming? H1-H8 need them
                        -> session 3

14 OK, 3 missing, 6 to tick by hand.
```

## Session 1: safety first

**Who:** everyone; name the **e-stop person** and the **safety driver** (Q32). **How long:**
15 min. **Bring:** stands, chocks, the RC transmitter.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | The car on **stands: all four wheels off the ground**, chocked | each wheel spins freely by hand | nothing else happens until it is |
| 2 | The e-stop person holds the **Kar-Tech wireless e-stop**, and stands by the **red button** | they say "e-stop ready" | |
| 3 | The **RC transmitter** on, its override switch **off** | the ESP32 ignores the sticks | switch it off before anything starts |
| 4 | Press the wireless e-stop once, with everything on | the boards' LEDs and the motor: write down what it cuts (S1) | it's S1: ask the hardware lead (session 3) |
| 5 | Say out loud: today's ESP32 C++ has **no command timeout** (S3) | everyone knows: until session 6, only the e-stop stops a frozen Pi | |
| 6 | Agree: **one drive path at a time**. Theirs (`aav_serial_bridge`) or ours, never both | | |

**Stop point:** stands, e-stop in a hand, RC off.

## Session 2: measure and weigh

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

## Session 3: questions for the hardware lead

**Who:** the hardware lead, and one person writing. **How long:** 30 min. **Bring:** this
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
| S1-S3 | What the e-stops cut; the brake without power; anything else that stops a frozen Pi | session 9 |

All the other questions: [audit questions](audit-questions.md).

**Stop point:** the answers written down, with who said them.

## Session 4: the Orin (and the Motor Pi)

**Who:** the laptop person. **How long:** 1-3 h; the backup runs while you do the rest.
**Bring:** the stick, the backup drive, a cable to the car's switch.

First, on the laptop: `cp ~/.config/aav/cyclonedds-car.xml ~/.config/aav/cyclonedds.xml`,
dongle in, then `pixi run doctor` (it should find the car's network).

### 4a. Look, then back up

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `ssh <user>@192.168.1.140` (the user: `hosts.txt`, JETSON) | a prompt on `ubuntu` | `pixi run net-map`: the Orin's address may have changed |
| 2 | `cat /etc/nv_tegra_release` | `R36 (release), REVISION: 4.7` = JetPack 6 (21 Oct) | write down what it says (W7) |
| 3 | `cat /proc/device-tree/model; free -h; df -h ~` | the module (AGX Orin), its memory (32 or 64 GB), **13 GB free** or more | not enough room: stop, ask the team |
| 4 | On the laptop, a second terminal: `rsync -a --exclude .cache --exclude aav-jazzy <user>@192.168.1.140:/home/<user>/ ~/aav-backup/jetson-$(date +%F)/home/` | it copies (1-3 h); go on meanwhile | it stops: run it again, it resumes |
| 5 | `/etc`: `ssh <user>@192.168.1.140 'sudo tar czf /tmp/etc.tgz /etc'`, then `scp` it to the same folder, then `sudo rm /tmp/etc.tgz` | `etc.tgz`, under 50 MB | no sudo: skip, write it down |

The full disk image (`l4t_backup_restore.sh`) needs an x86 Ubuntu PC: not today. It comes
before the reflash (T1.3), never after.

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

🔒 H1: is it CORTEX (.120)? Then the same kit, the same steps 4b and 4c there.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `ssh <user>@192.168.1.120`, then `uname -m` | `aarch64` | `armv7l`: a 32-bit OS, the kit can't run: write it down |
| 2 | 4b steps 1-4 there | 30-60 min on a Pi 4 | |
| 3 | 4c there (`eth0`, probably: CORTEX has WiFi too?) | | |

**Stop point:** our stack built on the Orin and the Motor Pi; the engine built (or why not).

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
rule uses Gazebo's lens and the car stops too early.

**Stop point:** both streams at 15 fps on the Orin, the delay measured, both cameras calibrated.

## Session 6: the ESP32

**Who:** the laptop person, the e-stop person, the hardware lead. **How long:** 90 min.
**Bring:** the ESP32's USB cable, the RC transmitter, a multimeter.
🔒 **H1** (the link to the Motor Pi) and **Ben's OK** (on a call):
[First flash on the bench](esp32-bench.md) says "do it with Ben".

**Before:** session 1 done; the Motor Pi and the Orin have our stack (session 4); the
laptop's USB cable in the ESP32 MicroMod. Every command below runs on the laptop, in the
repo.

### 6a. Back up what's on the board

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash list-ports` | `/dev/cu.usbserial-…` | another cable: some carry only power |
| 2 | `vendor/cargo-tools/bin/espflash read-flash 0 0x400000 car-day/esp32/backup-esp32-$(date +%F).bin` | a 4 MB file (4194304 bytes) | hold the board's BOOT button while it starts |
| 3 | Copy it to the backup drive | | |

The backup holds the C++'s WiFi password: it's a secret. Never in git (`car-day/` isn't).

### 6b. Flash the Rust, and switch the Motor Pi to version 2

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | On the Motor Pi: stop theirs, `sudo systemctl stop aav_serial_bridge` | `systemctl status aav_serial_bridge`: inactive | never two drive paths |
| 2 | `vendor/cargo-tools/bin/espflash flash --chip esp32 --monitor car-day/esp32/spine-esp32-umod-v2.elf` | `spine-esp32-umod 0.1.0, link: Motor-Pi UART, version 2 frames (the Orin's heartbeat)` | roll back (6d) |
| 3 | On the Motor Pi: `legacy.yaml` to version 2: in `~/aav-jazzy/aav/ros/can_bridge/aav_legacy/config/legacy.yaml`, `frame_version: 2`, then `~/aav-jazzy/aav.sh build` | | |
| 4 | Start the Orin: `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin max_speed:=0.3 gps:=false lidar:=false` | the output mux makes the heartbeat. Its `foxglove_bridge` can't open port 8765 (theirs holds it): harmless, see session 8 | |
| 5 | Start the Motor Pi: `~/aav-jazzy/aav.sh car motor_pi` | the ESP32's log: `LINK frames=` rising, `bad=0` | `bad=` rising: the speed or the wire (H1) |

### 6c. The checks, wheels off the ground

From [First flash on the bench](esp32-bench.md), in order. **Stop at the first that
fails, and roll back.** Drive from the laptop: `export ROS_DOMAIN_ID=42`, `pixi run keyboard`.

| # | Do | Expect |
|---|---|---|
| 1 | Power on, nothing else | brake full (3500), steering centred (1950), DAC 0 V; `mode=Manual`, `faults=0x0000` |
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
| 10 | RC override on, a little throttle | `mode=Rc`, blue LED; ROS ignored |
| 11 | RC: transmitter off | within 0.1 s: brake full, `faults` has 256 |
| 12 | RC override off | back to the computer, or Manual |
| 13 | The red e-stop button | the car stops by itself |
| 14 | 10 min of 1-13 mixed | no unexpected reset (`the last reset:` stays the power-on one) |

### 6d. Roll back to the C++

Any time something is wrong: e-stop, Ctrl-C the bridge, then:

| # | Do | You should see |
|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash write-bin 0 car-day/esp32/backup-esp32-<date>.bin` | the board as it was: `=== ESP32: AAV Controller ===` in its log |
| 2 | On the Motor Pi: `frame_version: 1` back, `~/aav-jazzy/aav.sh build` | the C++ reads only the 13-byte frame |
| 3 | `sudo systemctl start aav_serial_bridge` (theirs), if that's the plan for the rest of the day | |

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
| 1 | Theirs, read-only: `pixi run layout-check ws://192.168.1.140:8765` | their topics (Humble, domain 0); ours aren't there | |
| 2 | Ours, on the Orin: `~/aav-jazzy/aav.sh ros2 launch aav_bringup foxglove.launch.py foxglove_address:=0.0.0.0 foxglove_port:=8766 foxglove_control:=false` | `listening on port 8766` | |
| 3 | `pixi run layout-check ws://192.168.1.140:8766 --data` | every layout topic of the running parts served, and kB/s each | missing ones: their part isn't running |
| 4 | Lichtblick → Open connection → Foxglove WebSocket → `ws://192.168.1.140:8766`; Layouts → Import → `tools/foxglove/driving.json` (then the other two) | the car's model, the gate, the e-stop | [Watch from a laptop](foxglove.md) |
| 5 | The same over the car's WiFi | under 2.5 MB/s for every laptop together | the WiFi drops multicast? ([Friday](history/friday-2026-10-16.md)) |

The map panel needs the internet for its tiles: offline it shows the dots on a blank map.

**Stop point:** a layout open on a laptop.

## Session 9: the first drive, on stands

**Who:** the safety driver, the e-stop person, the laptop person. **How long:** 45 min.
🔒 H1 and S1-S3. **Wheels off the ground**, the speed capped at **0.3 m/s**.

| ☐ | Before |
|---|---|
| ☐ | all four wheels off the ground (session 1) |
| ☐ | the e-stop person ready; everyone knows what it cuts (S1) |
| ☐ | theirs stopped: `aav_serial_bridge` inactive. **Never two drive paths** |
| ☐ | which ESP32 firmware: the Rust (session 6 passed) or the C++ (**no timeout**: test 3.9 then fails, as expected) |
| ☐ | `legacy.yaml`'s `frame_version`: 2 with the Rust, 1 with the C++ |
| ☐ | with the C++ (1): the bridge's log says `no-timeout lock ON`. twist_mux then holds every program (scripted, point, Nav2, AI); the keyboard and the actuator tests below still drive. Leave `allow_programs_without_timeout: false` |
| ☐ | everyone knows: **the throttle's strength is a guess.** Two guesses disagree by 5 times ([the car's model](vehicle-model.md): full throttle holds 15 m/s by the drag guess, 3 m/s by the ESP32's open loop). 0.3 m/s asked may spin the wheels much faster: the e-stop person watches 3.2 |

| Where | Run |
|---|---|
| the Motor Pi | `~/aav-jazzy/aav.sh car motor_pi` |
| the Orin | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin max_speed:=0.3 gps:=false lidar:=false` |
| the laptop, each terminal | `export ROS_DOMAIN_ID=42`, then `pixi run keyboard`, and `pixi run hud` |
| the Orin, a second terminal | `~/aav-jazzy/aav.sh tools/record.sh stands` (session 10) |

The HUD's GO / STOP panel says whether the car may move, and why not. A latched gate:
`pixi run gate-clear`; a latched watchdog (such as `hud LOST`): `pixi run watchdog-clear`.

| # | Test | Expect | If not |
|---|---|---|---|
| 3.1 | Nothing pressed | wheels still; the HUD shows 0 | e-stop |
| 3.2 | Forward, gently | the drive wheels turn **forward**, slowly | backwards: write it down (`legacy_protocol.py`), not fixed at the car. **Fast:** e-stop, write down the speed (3.12's tachometer), and no ground driving until tests 4.2 and 4.4 |
| 3.3 | Left | the front wheels turn **left** | right: `steering_sign` (Q2) |
| 3.4 | Let go of the key | slows to 0 within 0.3 s | |
| 3.5 | The HUD's e-stop button | brakes at once; nothing moves until `pixi run estop-clear` | |
| 3.6 | Stop the keyboard program | the bridge brakes: its commands go stale | |
| 3.7 | Ctrl-C the Motor Pi's bridge | a last stop frame | |
| 3.8 | The Kar-Tech e-stop | everything stops | |
| 3.9 | Unplug the Motor Pi's serial cable while driving | the Rust: brakes within 200 ms. The C++: it doesn't (S3) | |
| 3.10 | `pixi run actuator-test steer-sweep --stands` (the laptop) | it asks each wheel angle: the protractor | |
| 3.11 | `pixi run actuator-test steer-centre --stands` | under 1°, or the offset written down | |
| 3.12 | Wheel speed: `/vehicle/odom` against a phone tachometer app | within 10% | |
| 3.P | `pixi run preflight --no-drive --skip steering gps lidar` (the Orin, ssh + tmux) | every sensor line PASS; nothing moves | a FAIL names the sensor: its driver, its cable, its rate |

**The preflight's drive** (speed up, hold, brake, steer, every sensor at once) is refused
on the car today: the wheel sensors are on the front, the motor drives the rear, so on
stands nothing measures the spinning wheels (TODO 49b). Once a rear-wheel speed exists
(`vehicle.yaml` `speed_wheels: rear`): `pixi run preflight --skip steering gps lidar`,
with the e-stop person and someone on the rear wheels' tachometer ([the preflight](actuator-tests.md#the-preflight-on-stands)).

**Not today: the car on the ground.** That waits for tests 4.2 (`coast`) and 4.4 (`throttle-map`): they measure the drag and the throttle, and settle the 5 times disagreement (TODO 52b).

Then put it back: Ctrl-C everything; theirs again (`sudo systemctl start aav_serial_bridge`)
only with the C++ on the ESP32 (6d), as the team decides.

**Stop point:** 3.1-3.8 pass. Anything that turned the wrong way: written down.

## Session 10: what to record

**Who:** the laptop person. **All day.** These feed the sim-versus-real report (PLAN T2.7),
the sensors' real noise (T2.4) and the replay (T2.5). Bags stay out of git.

| # | Record | How | For |
|---|---|---|---|
| 1 | **Parked, 5 min**: the IMU and the GPS, with their drivers (Humble, so on their Pi; it only writes `~/aav-bags`) | on LOCALIZATION: `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/parked /imu/data_raw /imu/mag /aav/gps/fix` | the IMU's bias, the GPS's noise over 10 s (TODO 40) |
| 2 | **The lidar, 1 min**, parked and with someone walking past | on the Orin, theirs: `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/lidar /velodyne_points /scan` | the lidar layer, ghost obstacles |
| 3 | **The stands drive** (session 9) | on the Orin: `~/aav-jazzy/aav.sh tools/record.sh stands` | ego and the actuator model against real |
| 4 | The actuator tests | they save themselves: `~/aav-tests/` on the laptop | `vehicle.yaml`'s steering numbers |
| 5 | **Every topic's size and rate**, our stack running | on the Orin: `~/aav-jazzy/aav.sh bandwidth --markdown` | the bandwidth budget (T1.7) |
| 6 | The camera delay photos, the calibration pictures, the sheet | the phones; `~/aav-calib/` | session 5; Gate 1 |
| 7 | The ESP32's log during 6c | copy the `espflash` terminal to a file | Gate 2's evidence |

Copy home: `rsync -a <user>@192.168.1.140:aav-jazzy/aav/bags/ ~/aav-bags/jetson/`, and
`~/aav-bags/` from LOCALIZATION and the Orin the same way.

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
| The car's linear model, refit | `pixi run vehicle-model`: new poles and margins from the measured lags ([The car as a linear model](vehicle-model.md)) |
| The answers | `docs/audit-questions.md`; the hardware lead's in `docs/can-plan.md` |
| Gate 1 and Gate 2 | [The plan](plan.md) |
