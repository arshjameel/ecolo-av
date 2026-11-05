# Car setup: one computer

Written 1 November 2026, for a student on their own. Ben, 1 Nov: "we are going to login
to each node, upgrade the os, and install our code. Make sure we setup the boot process".
This page sets up **one computer**. **Which one, and when:** [Car day](car-day.md) owns the
order; the upgrades are its [Day B](car-day.md#day-b-upgrades-and-the-reflash).

> **The short version**
> - **One script per computer:** `bash car_setup.sh <role>`. With no `--apply` it only prints what it would do.
> - **Back up first** ([car day A2](car-day.md#a2-back-up-every-machine)). Upgrades are Day B, after a restore is proven.
> - **The script stops** on a pending reboot, a hostname that isn't the role's, and an upgrade that's too early (below).
> - The computers, their roles and addresses: [the machines table](car-day.md#the-machines).
> - **Logins** come from the team's password vault ([Logins](car-day.md#logins)). Type them; never paste them into a file, a chat or the repo.
> - Changes to the hardware (the ESP32, the Orin's reflash) wait for **the hardware lead's OK**.

The internet is there: `apt` and GitHub work.

## Running the script

Copy it to the computer, log in, and run it there. `tmux` keeps it running if the ssh drops.

```bash
scp tools/car_setup.sh <user>@192.168.1.120:        # from the laptop, in the repo
ssh -t <user>@192.168.1.120
tmux new -s setup                                  # back in after a drop: tmux attach -t setup
bash car_setup.sh motor_pi                         # a dry run: prints every step, changes nothing
bash car_setup.sh motor_pi backup --apply          # really: it asks you to type "motor_pi"
```

| Step | Does | Needs sudo |
|---|---|---|
| `backup` | `~/aav-backup/<host>-<date>/`: the package lists, services, crontab, addresses, `/etc` | yes (`/etc`) |
| `upgrade` | `apt full-upgrade`; prints `do-release-upgrade` (runs it with `--release-upgrade`) | yes |
| `install` | our code from the car kit (`car_install.sh`); the dash adds `--screens` | no |
| `services` | chrony, udev, groups, Cyclone's card, a fixed address (`--address`), our stack at boot | yes |
| `check` | read-only: OK, NOTE or STOP per line. Runs in a dry run too | no |

With no step named, it does all five, in that order. The units it installs:
[tools/systemd](../tools/systemd/README.md).

### When it stops

A STOP line ends the run before anything changes (with `--apply`, before the next step).

| STOP | Why | What to do |
|---|---|---|
| `reboot` | a reboot is waiting: an upgrade on top of it can break the boot | `sudo reboot`, log in, run it again |
| `hostname` | this isn't the hostname the audit found for the role: maybe the wrong computer | check the address and the vault's entry. Right after all (a new card, a reflash)? add `--host-ok` |
| `upgrade` on `localization` | its IMU and GPS drivers run on Humble, which 24.04 ends; no machine file of ours runs them yet | wait for `machines/localization.yaml` (TODO 75) |
| `upgrade` on `motor_pi` | the 2025-26 `aav_serial_bridge` (Humble) is the way back to the C++ ESP32 | only after the Rust ESP32 passed car day's session 6: add `--after-esp32` |
| `UART`, `udev` (in `check`, after an upgrade) | the ESP32's serial port, or the sensors' `/dev/aav` names, didn't come back | `enable_uart=1` in `/boot/firmware/config.txt` and reboot; the cables, then `sudo udevadm trigger` |

On a Mac (a rehearsal) the hostname is only a NOTE.

## Step 1: back up

Before anything changes, on the **encrypted** backup drive (the backups hold passwords),
never in git. The whole list, with the checks, is car day's
[A2](car-day.md#a2-back-up-every-machine): every Pi's SD card as an image (checked with
`gunzip -t`), the Orin's home and `/etc`, the ESP32, the actuators' settings.

On the computer itself, a second copy of what only it has:

| # | Where | Do |
|---|---|---|
| 1 | the computer | `bash car_setup.sh <role> backup --apply` |
| 2 | the laptop | the `rsync` line it prints: the home folder, to the backup drive |

## Step 2: the Ubuntu 22.04 Pis, to 24.04

`dash`; `motor_pi` with `--after-esp32`; `localization` once its machine file exists.
An upgrade in place: no new card. Car day's [Day B](car-day.md#day-b-upgrades-and-the-reflash) says when.

| # | Do | You should see | If not |
|---|---|---|---|
| 1 | `bash car_setup.sh <role> upgrade --apply` | the updates install | an apt error: write it down, ask the team |
| 2 | `sudo reboot`, log in again, `tmux new -s setup` | | |
| 3 | `bash car_setup.sh <role> upgrade --apply --release-upgrade` | it asks questions: keep the **installed** version of each config file | it refuses: fewer than 5 GB free, or a reboot is pending |
| 4 | Over ssh it says it opens a **spare ssh on port 1022**: say yes | | the ssh drops: `ssh -p 1022 <user>@<address>` |
| 5 | At the end, reboot | `lsb_release -d`: Ubuntu 24.04 | the SD card image (car day A2) |
| 6 | `bash car_setup.sh <role> check` | `Ubuntu 24.04` | |

!!! warning "This ends the 2025-26 software on that Pi"
    Humble (`/opt/ros/humble`) is built for 22.04: after the upgrade it stops working.
    The backup is the way back. Our code doesn't need it.

## Step 3: the Pis on Raspberry Pi OS

Pi OS can't become Ubuntu in place: that is a **new SD card** with Ubuntu Server 24.04
(Raspberry Pi Imager → Other general-purpose OS → Ubuntu).

The camera Pis **stay on Pi OS** ([What's new](whats-new.md#the-computers-which-ubuntu)):
they run no ROS, and the Pi camera works best on Pi OS. So for them:
`bash car_setup.sh camera_pi upgrade --apply`, then `services --apply` (MediaMTX at boot,
the Orin's time).

A new card instead, for any Pi: keep the old card, labelled. Undo is a card swap.

## Step 4: install our code

`orin`, `motor_pi`, `localization`, `dash`: from the car kit, made on the laptop.

| # | Where | Do |
|---|---|---|
| 1 | the laptop | `pixi run car-kit` (once), then `scp -r car-kit <user>@<address>:~/` |
| 2 | the computer | `bash ~/car-kit/car_setup.sh <role> install --apply` |
| 3 | the computer | `~/aav-jazzy/aav.sh tools/car_listen.sh`: the devices (read-only) |

The kit's details: [From a USB stick](car-install.md). The camera Pis get only our
`mediamtx.yml` (car day's [session 5](car-day.md#session-5-the-camera-pi)).

## Step 5: start at boot

`bash ~/car-kit/car_setup.sh <role> services --apply`, then `sudo reboot`.

| What | Which roles | How |
|---|---|---|
| Our stack, `car.launch.py machine:=<role>` | `orin`, `motor_pi` | `aav-car.service`, a user unit; linger starts it at boot |
| Its settings | the same | `~/.config/aav/car.env`: the machine file, `max_speed:=0.3` |
| The two screens | `dash` | `aav-hud`, `aav-map` ([The two screens](car-screens.md)); turn on automatic login, by hand |
| The camera stream | `camera_pi` | MediaMTX, enabled |
| Time | all but the lab desktop | chrony: the Orin keeps the car's time, the others ask it |
| Device names | `orin`, `motor_pi`, `localization` | `tools/udev/99-aav.rules`: `/dev/aav/...` |
| Groups | the same, and `dash` | `dialout` (serial), `plugdev`, `input` (the joystick), `video` |
| ROS | the same | domain 42 (`aav.sh`); Cyclone on the 192.168.1.x card |
| A fixed address | any | `--address 192.168.1.120/24`: beside DHCP's, through `netplan try` (undoes itself after 120 s unless you press Enter). The router's reservation is better |

| Look | Command |
|---|---|
| Is it running? | `systemctl --user status aav-car` |
| Its log | `journalctl --user -u aav-car -f` |
| Stop it until the next boot | `systemctl --user stop aav-car` |

!!! warning "Never two drive paths"
    On the Motor Pi the script won't enable ours while the 2025-26 `aav_serial_bridge`
    starts at boot. Turn theirs off only when the team agrees:
    `sudo systemctl disable --now aav_serial_bridge`.

The `localization` Pi gets no ROS service yet: no machine file has its IMU and GPS
(`orin.yaml` assumes they're on the Orin: a TODO for the hardware lead's answer). For the
same reason `upgrade` refuses there.

**The first moves** (`machine:=motor_pi_alone`, the joystick on the Motor Pi): car day's
[A9b](car-day.md#a9b-the-joystick-on-stands).

## Step 6: the Orin, to JetPack 7.2

A **reflash**, not an upgrade: `do-release-upgrade` must never run on a Jetson (the
script refuses). Everything on the Orin is wiped. **Day B, after the hardware lead's OK**
([B3](car-day.md#b3-the-orin-its-image-then-the-reflash)).

| ☐ | Check first |
|---|---|
| ☐ | NVIDIA's JetPack 7.2 release notes list **Jetson AGX Orin**, and its memory size (W7: 32 or 64 GB). Write what they say here: ________ |
| ☐ | SDK Manager's page: which Ubuntu the **host** needs for 7.2. The lab desktop has it? |
| ☐ | The module's part number ([Platforms](platforms.md#moving-the-orin-to-jetpack-72): -502 has a reported flashing problem) |
| ☐ | Home and `/etc` copied ([A2](car-day.md#a2-back-up-every-machine)); the full image taken ([11b](car-day.md#11b-in-the-lab-the-orins-full-image)) |

| # | Do | You should see |
|---|---|---|
| 1 | The Orin beside the lab desktop, a USB-C data cable to its flashing port, the Orin in **recovery mode** | `lsusb` on the desktop: NVIDIA (`0955:`) |
| 2 | SDK Manager on the desktop: Jetson AGX Orin, JetPack 7.2; flash and install the SDK components | 30-90 min; the Orin boots its first-start setup |
| 3 | Make the user (as in `hosts.txt`), on the car's network | `ssh` works |
| 4 | `bash car_setup.sh orin upgrade --apply`, then `install --apply`, `services --apply` (a new hostname: `--host-ok`) | as steps 4-5 |
| 5 | The TensorRT engine again: [session 4d](car-day.md#4d-rf-detrs-tensorrt-engine) | an engine is tied to its TensorRT version |

## Step 7: the lab desktop

**Its OS stays frozen until the Orin's reflash is done**: it is the flashing host
([B4](car-day.md#b4-the-lab-desktop-stays-frozen)).

| # | Do | You should see |
|---|---|---|
| 1 | `bash car_setup.sh lab_desktop backup upgrade --apply` | updated. A release upgrade only if CARLA's version supports 24.04 ([CARLA](carla.md)) |
| 2 | `bash car_setup.sh lab_desktop install --apply` | `~/ecolo-av` cloned and built (Pixi first, if missing) |
| 3 | CARLA running, and the rehearsal: car day's [session 11a](car-day.md#11a-in-the-lab-the-week-before) | the car drives in CARLA |

## This computer's checklist

`check` is `bash car_setup.sh <role> check`. Which computers, in which order: [Car day](car-day.md#the-order).

| ☐ | Step |
|---|---|
| ☐ | 1 backed up (car day A2), and its own `backup --apply` |
| ☐ | 2 or 3: updated, rebooted; `check` says the release you wanted |
| ☐ | 4 our code installed (not the camera Pis) |
| ☐ | 5 services; rebooted; `check` has no STOP |
| ☐ | 5 time from the Orin (`chronyc sources`; the Orin serves) |

## What's untested

| Part | Status |
|---|---|
| `car_setup.sh`'s dry run, every role, and its STOPs | [[works]] on a Mac (`tools/tests/test_car_setup.py`); on Linux: CI runs the same tests, not yet seen there |
| `car_setup.sh --apply` on a car computer | [[untested]]: this day is its first run |
| `aav-car.service` at boot | [[untested]] |
| `machine:=motor_pi_alone` | [[untested]] on the car; its launch is checked like every machine file |
| `do-release-upgrade` on these Pis | [[untested]] |
| Which computer is the Motor Pi (H1), CAM1-PI's address (C1), `.211`'s job | [[needs-you]]: the hardware lead |
