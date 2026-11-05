# Our stack on the car, from a USB stick

How to put our ROS 2 Jazzy on the Orin and the Motor-Pi **with no internet**, **without
sudo**, and **without touching** the 2025-26 Humble software. It all goes in one folder,
`~/aav-jazzy`. Deleting that folder undoes everything.

!!! warning "Not on the car yet"
    The kit is made and checked on a Mac, and installed offline on 64-bit ARM Linux by CI
    on every push (GitHub's ARM runner, Ubuntu 24.04). On the Orin and the Pis it is
    [[untested]]: car day is its first run there ([Car day](car-day.md), session 4). Back
    up first, then install.

## The plan in one picture

```text
  Mac (internet)                    USB stick                 car computer (no internet)
  pixi run car-kit   ──────────►   car-kit/  (1.8 GB)  ──►   car_install.sh  ──►  ~/aav-jazzy/
                                                              car_listen.sh       (read-only)
```

| # | Where | Step | Time (measured) |
|---|---|---|---|
| 1 | Mac | `pixi run car-kit` | 26 s with Pixi's cache full; else the time to download 1.6 GB |
| 2 | Mac | copy `car-kit/` to a stick | a few minutes |
| 3 | car | `bash .../car_install.sh --dry-run` | 1 min (the checksums) |
| 4 | car | `bash .../car_install.sh` | GitHub's ARM runner (4 cores, 16 GB), `--all`, no network: 4.5 min (24 Oct). A Mac took 91 s; guess 10 min on the Orin, 30-60 min on a Pi 4 |
| 5 | car | `~/aav-jazzy/aav.sh tools/car_listen.sh` | seconds |

## 1. Make the kit (on a Mac, with internet)

```bash
cd ~/ecolo-av
git fetch upstream && git switch main && git merge --ff-only upstream/main
                                    # the team's main (CONTRIBUTING.md); the kit holds
                                    # what is committed, nothing else
pixi run car-kit                    # writes car-kit/ (git ignores it)
```

What it holds (made on Ben's Mac, 15 Oct):

| File | What | Size |
|---|---|---|
| `aav-jazzy-linux-aarch64.tar` | every package of our default Pixi environment, for 64-bit ARM, from `pixi.lock` (packed by pixi-pack) | 1.7 GB |
| `bin/pixi`, `bin/pixi-unpack` | the programs that unpack and run it, for 64-bit ARM, version-pinned and checksummed | 78 MB |
| `aav.bundle` | the repo (git bundle) | 5 MB |
| `aav-src.tar.gz` | the same code, for a computer with no git | 4 MB |
| `rust-crates.tar.gz` | the 152 Rust crates our four Rust packages use, so `cargo` needs no internet | 43 MB |
| `models/` | the YOLO model files from `models/`, if you have them (`pixi run get models`) | 6 MB |
| `car_install.sh`, `README.md`, `KIT.txt`, `SHA256SUMS` | the installer, a short how-to, the commit, the checksums | small |

Any stick works, even FAT32: the biggest file is under 4 GB.

## 2. Check first (on the car)

Log in as your normal user (ask the team for the login). Then:

```bash
uname -m                                            # must say aarch64
bash /media/$USER/<stick>/car-kit/car_install.sh --dry-run
```

`--dry-run` changes nothing. Each line says OK, NOTE or STOP:

| Check | Needs |
|---|---|
| system, processor | Linux on `aarch64`. A Pi on a 32-bit OS says `armv7l`: then the kit can't run there |
| user | not root: no `sudo` |
| glibc | 2.28 or newer (Ubuntu 22.04 has 2.35) |
| kit, checksums | every file there, and copied whole |
| disk | 13 GB free in your home folder (3 GB when the environment is already there) |
| git | optional: without it, the code comes from `aav-src.tar.gz` |
| memory | under 6 GB (a 4 GB Pi), it builds one package at a time |

## 3. Install

```bash
bash /media/$USER/<stick>/car-kit/car_install.sh
```

| Step | What happens | Where |
|---|---|---|
| 1 | unpacks ROS 2 Jazzy: 9.0 GB of package contents | `~/aav-jazzy/env` |
| 2 | clones the code, and links the environment in as Pixi's own | `~/aav-jazzy/aav` |
| 3 | unpacks the Rust crates, and tells cargo to stay offline | `~/aav-jazzy/cargo` |
| 4 | writes `aav.sh`, which runs a command inside our Jazzy | `~/aav-jazzy/aav.sh` |
| 5 | `colcon build`, offline. The HUD (`aav_hud`) only with `--all` | `~/aav-jazzy/aav/install` |

- Run it again any time: it skips what is done. A newer kit updates the code, and the
  environment only if `pixi.lock` changed.
- A package that fails to build doesn't stop the rest: the end of `~/aav-jazzy/build.log`
  says which.
- On the computer that drives the dash's two screens, add **`--screens`**: it builds the
  HUD and the map, and starts them at login, full screen, one on each screen
  ([The two screens](car-screens.md)).

!!! note "What it never touches"
    `sudo`, `/opt/ros` (Humble), `~/.bashrc`, `~/.cargo`, system services, the network
    settings. Everything is in `~/aav-jazzy`. Only `--screens` adds files outside it:
    two systemd *user* units and an autostart entry in `~/.config`.

## 4. Look, read-only: car_listen

```bash
~/aav-jazzy/aav.sh tools/car_listen.sh
```

It lists the serial, camera and CAN devices, and says whether each of our drivers
**could** open its device. It opens none of them: opening a USB serial port can reset an
ESP32.

| Column | Means |
|---|---|
| Wants | the name our config uses, like `/dev/aav/imu` |
| Found | that device, or the raw one with the right USB IDs (`/dev/ttyUSB0 (no udev name)`): our udev rules need sudo, so they aren't on the car yet |
| Can open? | `yes`, or `needs group dialout` (an admin adds you later) |
| In use by | a program already holding it: the 2025-26 software, probably |

## Why ROS_DOMAIN_ID 42

A domain is a separate ROS network on the same wires: each one uses its own UDP ports.
The 2025-26 software runs on domain 0 (the 21 Oct audit; their camera README says 10). `aav.sh` always sets **42**, so our nodes and
theirs never see each other: no mixed Humble and Jazzy messages, no topic clashes.

- To watch our nodes from a Mac: `export ROS_DOMAIN_ID=42` there too.
- A domain keeps the **messages** apart, not the **devices**: two programs can't share a
  serial port. Stop the 2025-26 driver before ours uses its device, and only when the
  team agrees.

## 5. Run

| Want | Run |
|---|---|
| any ROS command | `~/aav-jazzy/aav.sh ros2 topic list` |
| the Motor-Pi's part of the car | `~/aav-jazzy/aav.sh car motor_pi` |
| the Orin's part | `~/aav-jazzy/aav.sh car orin` |
| a rebuild after an edit | `~/aav-jazzy/aav.sh build` |

`aav.sh` runs `pixi run --as-is` in `~/aav-jazzy/aav`, after clearing anything a Humble
terminal set (`AMENT_PREFIX_PATH`, `PYTHONPATH`...). Pixi and cargo never go online.

!!! warning "Before `car`"
    Wheels off the ground, the e-stop in a hand, and the 2025-26 software stopped
    ([Platforms](platforms.md), "The first hardware test"; [Car day](car-day.md), session 9).

## Undo

```bash
bash ~/aav-jazzy/aav/tools/car_screens_install.sh uninstall   # only after --screens
rm -rf ~/aav-jazzy
```

Nothing else changed, so nothing else needs undoing.

## When there's internet

The online way works too: install Pixi, clone, `pixi install` ([Platforms](platforms.md),
"Try Jazzy on the car before the reflash"). The kit's clone already points `origin` at
GitHub, so `git pull` in `~/aav-jazzy/aav` works once the computer has a GitHub key.

## What was tested

| Part | Status |
|---|---|
| `pixi run car-kit` on a Mac | [[works]] 15 Oct: 1.8 GB, 26 s with Pixi's cache full |
| the linux-aarch64 pack | [[works]] pixi-pack packed all 1040 packages from `pixi.lock` |
| `car_install.sh --dry-run` on a Mac | [[works]] it refuses: not Linux, not aarch64, nothing changed |
| the same installer on a Mac, with a Mac pack, the network blocked | [[works]] 15 Oct: unpack 44 s, build all 25 packages 43 s (the Rust ones from the kit's crates), run again 16 s; `aav.sh` cleared a Humble `PYTHONPATH` and set domain 42 |
| `car_listen.sh` | [[works]] on a Mac (bash 3.2); its Linux parts (sysfs, /proc, `ss`) are [[untested]] |
| `car-kit`, then `car_install.sh --all` on 64-bit ARM Linux, no network | [[works]] 24 Oct, CI's job `car-kit` (every push): GitHub's ARM runner, Ubuntu 24.04; the kit 1 min, the install and the build of every package, the HUD too, 4.5 min |
| the install on the Orin or a Pi | [[untested]]: car day |
