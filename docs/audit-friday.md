# The hardware audit, step by step (Friday 16 October)

> **A snapshot from 16 October 2026.** The audits ran on 16 and 21 Oct: [16 Oct](history/audit-2026-10-16-car.md), [21 Oct](history/audit-2026-10-21-car.md). What's true now: [Status](status.md).

The first time the 2026-27 team logs in to the car's computers. The goal: **know what is
there, and save a copy, without changing anything.** This page is steps 2 and 3 of
[Work](work.md) ("Backups", "Inventory the car"), and it covers the checklist at the end of
[Car photos](car-photos.md).

!!! warning "Read-only first"
    Look, copy and photograph. Don't install, upgrade, reflash, restart services, change
    the network, or drive. The "don't" list is at the end of this page.


**The plan for the whole day** (audit, network, our stack on the car):
[Friday 16 Oct](history/friday-2026-10-16.md). **Every question to answer:**
[Friday's questions](audit-questions.md).

!!! note "After the backups: our stack, from a USB stick"
    If the team agrees, once a machine is backed up: [From a USB stick](car-install.md)
    puts our Jazzy in `~/aav-jazzy` with no internet and no sudo, and leaves Humble alone.
    Its `car_listen.sh` is read-only: it says which devices our drivers could open.

## The day in one picture

```text
 1. Safe and powered  →  2. Find each machine  →  3. Back it up  →  4. Read-only checks
                                                                          ↓
              7. Save the results (no secrets)  ←  6. Photos  ←  5. Measure (Q1-Q9, Q18, Q20, Q22)
```

| # | Step | Who | Time (guess) |
|---|---|---|---|
| 1 | The car on stands, wheels off the ground, e-stop in reach; power on | everyone | 15 min |
| 2 | Find every machine on the network | one laptop | 20 min |
| 3 | Back up each Linux machine | one laptop, one big drive | 1-3 h, in the background |
| 4 | The read-only checks, machine by machine | pairs | 2 h |
| 5 | Measure the car: Q1-Q9, Q18, Q20, Q22 ([Car numbers](car_measurements.md)) | a tape measure | 1 h |
| 6 | Photograph every label | a phone | while you go |
| 7 | Write it up as `docs/history/audit-2026-10-16-car.md` | one person | 30 min |

## Remember: the car speaks Humble, not our names

!!! note "Use ros2 topic list; don't assume our names"
    The car runs the 2025-26 software on **ROS 2 Humble**, with **its own topic names**
    (such as `/gps/fix`, `/cmd_vel`). Our names, like `/sensors/gps/fix`, only exist in our
    Jazzy code. On the car, list what is really there with `ros2 topic list -t`, and write
    down the names you see. Humble and Jazzy don't mix: don't run our stack against it.

## What to bring

| Item | Why |
|---|---|
| A laptop with an Ethernet port or USB adapter, and a cable | plug into a spare port on the TP-Link switch |
| An external drive, 1 TB or more, with an encrypted volume | the backups: they hold passwords and keys |
| A microSD reader | to image a Pi's card, if the team agrees to power it down |
| A phone camera | labels, wiring, screens |
| A tape measure, and the questions Q1-Q9, Q18, Q20, Q22 | [Decisions](decisions.md) |
| The machine logins | **ask the team.** Never write them in git, in these pages, or in chat |

## Step 1: safe and powered

| Check | Done |
|---|---|
| Wheels off the ground, on stands | |
| The red mushroom e-stop and the Kar-Tech remote in someone's hand | |
| RC transmitter off, or its override switch off | |
| Power on; note which LEDs light on the control board (Jrk power, error) and the e-stop board (TR, ES) | |

Nothing on Friday should make a wheel turn. If one does, press the e-stop and stop the audit.

## Step 2: find each machine

Plug the laptop into a spare switch port (or join the car's WiFi). Then, on a Mac or Linux:

```bash
ifconfig | grep "inet "          # your own address: the car's network is probably 192.168.1.x
arp -a                           # every machine your laptop has heard from
nmap -sn 192.168.1.0/24          # if nmap is installed: every machine that answers
```

Also open the router's page (step 4, "The router") and copy its client list: it names each
machine with its address.

What the old code already tells us (to confirm, not to trust):

| Machine | Address hint | From |
|---|---|---|
| Motor-Pi | 192.168.1.193 | the wheel encoder sends to it (`wiznet.ino`) |
| Wheel-encoder Pico 2 | MAC address `54:65:6d:70:65:6c` (it spells "Tempel") | `wiznet.ino` |
| Router | 192.168.1.1 (guess) | the usual Linksys default |
| A camera | 192.168.1.101 or .118 | the old HUD code (the two disagree) |

Fill in this table as you go. Put the switch port label in the first column.

| Switch label | Hostname | IP | MAC | Answers ssh? |
|---|---|---|---|---|
| JETSON | `ubuntu` (likely) | 192.168.1.140 | 48:b0:2d:a6:56:86 | yes — and Foxglove bridge on :8765 |
| CAM1-PI | *(not up / not found)* | | | |
| CAM2-PI | `aav-cam2` | 192.168.1.118 | 88:a2:9e:20:6c:b5 | yes |
| RTK-PI | *(candidate: .211)* | 192.168.1.211 | d8:3a:dd:19:44:08 | yes |
| DASH-UI | *(not yet identified)* | | | |
| ES-PI | *(not yet identified)* | | | |
| QNX-PI | *(not yet identified)* | | | |
| LIDAR | Velodyne VLP-16 | 192.168.1.201 | 60:76:88:38:66:a9 | no — web page on :80 |
| ROUTER | `cisco00956` (not the guessed Linksys) | 192.168.1.1 | c8:d7:19:3d:51:26 | no — web page on :80 |
| (not on the switch) Motor-Pi | *(silent; not at .193 now)* | | | |
| (not on the switch) wheel encoder | *(silent; MAC `54:65:…` not seen)* | | | |

### What the scan actually found (16 Oct, ~14:17, from a Mac: ping sweep + `arp -a` + port probe)

`nmap` wasn't installed; this is a ping sweep, the ARP table, and a probe of common ports.

| IP | Hostname | MAC | OS (SSH banner) | Open | Read as |
|---|---|---|---|---|---|
| .1 | cisco00956 | c8:d7:19:3d:51:26 | — | 80 | **Router** (Cisco, not the guessed Linksys) |
| .118 | **aav-cam2** | 88:a2:9e:20:6c:b5 | Debian 13 (OpenSSH 10.0p2) | 22 | Camera Pi, freshly imaged |
| .120 | **ubuntu-cortex** | 88:a2:9e:76:0b:f0 | Ubuntu 22.04 | 22 | Pi — the Cortex/actuator host? |
| .132 | esp32-4cad8c | 34:98:7a:4c:ad:8c | — | — | **ESP32** control board |
| .140 | ubuntu | 48:b0:2d:a6:56:86 | Ubuntu 22.04 | 22, **8765** | **ROS box** — Foxglove bridge live (likely the Jetson) |
| .195 | — | d8:3a:dd:2c:0d:49 | Ubuntu 22.04 | 22 | **Same machine as .196** (see note) |
| .196 | **aav-desktop** | d8:3a:dd:2c:0d:4c | Ubuntu 22.04 | 22 | Desktop, dual-homed |
| .201 | — | 60:76:88:38:66:a9 | — | 80 | **Velodyne VLP-16 LiDAR** (web UI) |
| .211 | — | d8:3a:dd:19:44:08 | Ubuntu 22.04 | 22 | A Pi, role TBD |

!!! note "aav-desktop is dual-homed (.195 = .196)"
    `.195` and `.196` have **byte-for-byte identical SSH host keys** (RSA and ed25519) and
    consecutive MACs (`…0d:49` / `…0d:4c`): one physical machine with two Ethernet ports
    on the switch, not two machines. Untangle this — two interfaces on one subnet can make
    DDS bind oddly.

!!! warning "Not seen on the network at scan time"
    The **Motor-Pi** (expected .193), the **wheel-encoder Pico** (MAC `54:65:6d:70:65:6c`),
    and a **CAM1-PI** did not answer. Either powered off, or on a separate wired segment not
    bridged to this router. Re-check with the car fully powered.

## Step 3: back up first

Before any check, copy each Linux machine to the encrypted drive. Check the sizes first:

```bash
ssh <user>@<ip> 'df -h /; sudo du -sh /home /etc /opt 2>/dev/null'
```

| Way | When | Size (guess) | Command (run on the laptop) |
|---|---|---|---|
| **rsync the home folder** | every machine, Friday | 1-20 GB a Pi; more on the Orin (models, bags) | `rsync -a --exclude .cache <user>@<ip>:/home/<user>/ ~/aav-backup/<machine>/home/` |
| **tar /etc** | every machine, Friday | under 50 MB | `ssh <user>@<ip> 'sudo tar czf /tmp/etc.tgz /etc'`, then `scp <user>@<ip>:/tmp/etc.tgz ~/aav-backup/<machine>/`, then `ssh <user>@<ip> 'sudo rm /tmp/etc.tgz'` |
| **SD card image** | a Pi, only if the team agrees to shut it down | the card's size (16-128 GB) before gzip | on a Mac: below the table |
| **Full Orin image** | not Friday: needs an x86 Ubuntu PC and recovery mode | the whole eMMC or SSD | `l4t_backup_restore.sh` ([Platforms](platforms.md)) |

The SD card image, on a Mac (the card in the reader, `N` from `diskutil list`):

```bash
diskutil list                                   # find the card: /dev/diskN, its size
diskutil unmountDisk /dev/diskN
sudo dd if=/dev/rdiskN bs=4m | gzip > ~/aav-backup/<machine>.img.gz
```

- The `/tmp/etc.tgz` file is the only thing written on the car on Friday, and it is deleted
  straight after.
- With `dd`, `if=` is the **card** you read from. Swap `if` and `of` and you erase it. Have a
  second person read the command before you press Enter.
- **The backups hold secrets** (WiFi passwords in `/etc/NetworkManager`, SSH keys, password
  hashes). Keep them on the encrypted drive. Never in git, never in a shared folder.
- The Orin holds the only copies of the 2025-26 stop-sign detector, `camera_decode` and the
  OSRM map data ([Older code](older-code.md)). Back it up first.

## Step 4: the read-only checks

### The block for every Linux machine

!!! note "The logins, from the Keychain (20 Oct)"
    Each line of `~/aav-audit/hosts.txt` may name a Keychain entry as its third column
    (`JETSON aavjetson@192.168.1.140 aav-jetson`). The survey then gets that machine's
    password from the Mac's Keychain itself (`tools/keychain_askpass.sh`): nobody copies
    passwords, and none is ever written to a file. Add an entry with
    `security add-generic-password -s aav-jetson -a aavjetson -w` (it asks).

!!! note "Now a script: `pixi run bash tools/survey_car.sh`"
    [Friday 16 Oct: the to-do list](history/friday-todo-2026-10-16.md) part 2: `tools/machine_survey.sh` does all of the block
    below and more (boot scripts, who holds each device, camera and power errors, code and
    weights found only here), on every machine in `~/aav-audit/hosts.txt`. The block below
    stays as the by-hand version.

Save this as `audit.sh` **on your laptop**, then run it from the laptop:

```bash
ssh <user>@<ip> 'bash -s' < audit.sh > ~/aav-audit/<switch label>.txt 2>&1
```

Nothing is saved on the car. The output stays on your laptop.

```bash
# audit.sh: read-only. Prints what a machine is and what it runs.
echo "== who";        hostname; whoami; date; uptime
echo "== os";         cat /etc/os-release; uname -a
echo "== disk";       df -h
echo "== memory";     free -h
echo "== network";    ip -br addr; ip route
echo "== running";    systemctl list-units --type=service --state=running --no-pager
echo "== enabled";    systemctl list-unit-files --state=enabled --no-pager
echo "== added";      ls -la /etc/systemd/system
echo "== cron";       crontab -l 2>&1; ls /etc/cron.d
echo "== devices";    ls -l /dev/tty* /dev/serial* /dev/video* /dev/can* /dev/i2c* 2>/dev/null | grep -v "tty[0-9]"
echo "== by-id";      ls -l /dev/serial/by-id 2>/dev/null
echo "== usb";        lsusb
echo "== ros env";    printenv | grep -i ros; grep -n "ROS\|source" ~/.bashrc
echo "== ros";        ls /opt/ros 2>/dev/null
echo "== workspaces"; ls -d ~/*/src ~/*/install 2>/dev/null
echo "== docker";     docker ps -a 2>&1
echo "== busiest";    ps aux --sort=-%cpu | head -25
echo "== udp ports";  ss -uapn 2>/dev/null | head -30
echo "== ros graph"
for d in /opt/ros/*/setup.bash; do source "$d"; done
export ROS_DOMAIN_ID=$(grep -o 'ROS_DOMAIN_ID=[0-9]*' ~/.bashrc | tail -1 | cut -d= -f2)
echo "domain: ${ROS_DOMAIN_ID:-0 (not set)}"
timeout 30 ros2 node list --no-daemon --spin-time 5
timeout 30 ros2 topic list -t --no-daemon --spin-time 5
```

- An ssh session doesn't read `.bashrc`, so the block reads the domain from it. The
  2025-26 team used **10** on the dashboard and **0** on the UI. If the graph is empty, try
  both: `export ROS_DOMAIN_ID=10`.
- `--no-daemon` keeps `ros2` from starting its background daemon on the car.
- **Before you share an output file, read it.** `printenv`, `.bashrc` and `crontab` can hold
  tokens or passwords. Cut them out.

### JETSON (Orin #1)

| | |
|---|---|
| **Find it** | switch port JETSON. Home folder `/home/aavjetson` ([Platforms](platforms.md)) |
| **Also run** | `cat /etc/nv_tegra_release`; `cat /proc/device-tree/model`; `timeout 5 tegrastats` (5 seconds, then it stops); `nvpmodel -q` |
| **Look for** | the JetPack version (6 = Ubuntu 22.04); the ROS workspaces and what starts at boot; the YOLO detector and its model file (`*.pt`, `*.engine`); `camera_decode`; any lidar driver; OSRM data; free disk for the USB kit (13 GB) |
| **Answers** | car-photos #1, #3 (which computer reads the lidar?) |
| **Photograph** | the module's label: its part number (699-13701-0005-**501** or **-502**: the -502 has a reported flashing problem) |

### CAM1-PI and CAM2-PI

| | |
|---|---|
| **Find it** | switch ports CAM1-PI, CAM2-PI. Pi 5s at the top of the windscreen |
| **Also run** | `rpicam-hello --list-cameras` (or `libcamera-hello --list-cameras` on an older OS). If a service already holds the camera, it says so: that's an answer too |
| **Look for** | the camera model; what streams the pictures (a ROS node, GStreamer, a web stream) and to which address; the topic names |
| **Answers** | Q22, car-photos #6 (is the rear camera on one of these?) |
| **Photograph** | each camera's lens and board label; the ribbon cable |

### RTK-PI (with the ZED-F9R)

| | |
|---|---|
| **Find it** | switch port RTK-PI ("RIG-Pi + GPS" in the KEFC PDF). On the ceiling panel, beside the ZED-F9R |
| **Also run** | `ls -l /dev/ttyACM* /dev/serial/by-id`; `sudo fuser -v /dev/ttyACM0` (who has the port open); `ps aux`, and look for ntrip, str2str, rtk, ublox or gps in it |
| **Look for** | the ZED-F9R on `/dev/ttyACM*`; which program reads it; **where the RTCM corrections come from** (an NTRIP client and its caster's name; the 2024-25 team used the public rtk2go caster); whether a GPS topic is live: `ros2 topic hz <the GPS topic you saw>` |
| **Don't** | send the receiver any configuration (no u-center, no `ubxtool` writes). Don't copy NTRIP usernames or passwords: write only "an NTRIP login, in file X" |
| **Answers** | Q18, car-photos #2 ("Is there GPS data?") |
| **Photograph** | the ZED-F9R board and its u.FL antenna lead; the antenna's cable path |

### DASH-UI

| | |
|---|---|
| **Find it** | switch port DASH-UI. The small touchscreen on the dash (guess) |
| **Look for** | the 2025-26 Flutter app and its Flask back end; the boot scripts (`flask_bootup.sh`, which sources `/home/aavui`); `ottawa.db`; the OSRM route data and the tile server; which screen it drives (the Innoview or the small one) |
| **Answers** | which computer runs our HUD later ([HUD](hud.md)) |
| **Photograph** | both screens, with what they show |

### ES-PI

| | |
|---|---|
| **Find it** | switch port ES-PI. Maybe the Pi in a black case beside fuse block H1-H4 (guess) |
| **Also run** | `pinctrl get` (a Pi 5) or `raspi-gpio get` (a Pi 4): reads the pins, changes nothing |
| **Look for** | what it runs; whether it is in the e-stop path (relays, GPIO to the e-stop board's TR or ES) |
| **Answers** | car-photos #4, Q6 |
| **Photograph** | its wiring, end to end |

### QNX-PI

| | |
|---|---|
| **Find it** | switch port QNX-PI. Probably the 2025-26 `Real-Time-OS` layer, a Pi 4 on QNX 8 |
| **Log in** | ssh may land in a QNX shell, not Linux. Use `uname -a`, `pidin` (QNX's process list), `df -h`, `ls /dev`. The Linux block will partly fail: that's fine |
| **Look for** | whether anything else depends on it: is it in the driving path? |
| **Answers** | car-photos #4 |
| **Photograph** | the board and its cables |

### Motor-Pi (the Raspberry Pi 4 on the control board)

| | |
|---|---|
| **Find it** | not on the switch's labels; expected at 192.168.1.193 |
| **Also run** | `ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0`; `grep -v "^#" /boot/firmware/config.txt` (or `/boot/config.txt`); `sudo fuser -v /dev/serial0 /dev/ttyAMA0`; `ss -uapn` (look for port 10111) |
| **Look for** | **what talks to the ESP32**: which program writes the UART, and from which ROS topic; what listens on UDP port 10111 for the wheel encoder; whether Bluetooth is turned off (`dtoverlay=disable-bt`) |
| **Don't** | run `i2cdetect`: it probes the bus the actuators share |
| **Answers** | the legacy path in [Work](work.md); `aav_legacy` runs here later |
| **Photograph** | the board's header and UART wires |

### The LIDAR

| | |
|---|---|
| **Find it** | switch port LIDAR, then its address in the router's client list or `arp -a` |
| **Log in** | none. If it has a web page, open it and only read it |
| **Look for** | the model and its IP; which computer receives its packets (on the Orin: `ss -uapn` shows the port); any driver in a workspace |
| **Answers** | Q22, car-photos #3. Since 20 Oct we run the stock velodyne driver for it |
| **Photograph** | its label (model, serial) on the roof tripod |

### The router (Cisco `cisco00956`, found 16 Oct; the old code guessed a Linksys EA3500)

| | |
|---|---|
| **Find it** | switch port ROUTER; its page at the gateway address (`ip route` on any machine shows it) |
| **Log in** | ask the team. Don't write the admin password anywhere |
| **Look for** | the client list (names, IPs, MACs): a screenshot; the address range; the firmware version |
| **Don't** | change any setting; photograph the note taped to it up close |
| **Photograph** | the client list page (check it shows no password first) |

### The ESP32 control board

| | |
|---|---|
| **Find it** | the red SparkFun MicroMod on the control board. It joins the WiFi; its address is in the router's list |
| **Log in** | none. Its web page at `http://<ip>/status` is read-only: open it |
| **Look for** | is the CAN transceiver **U7** fitted?; the "Throttle enable" jumper; how many wires go to the steering and brake actuators |
| **Don't** | open its update page (no password: anyone could reflash the car); plug USB into it (it may reset); open `/scan` unless the car is on stands |
| **Answers** | car-photos #5 |
| **Photograph** | both sides of the board, U3 and U7, the CANH/CANL plug, both Jrk G2 labels |

### The wheel-encoder Pico 2 (if present)

| | |
|---|---|
| **Find it** | by its MAC `54:65:6d:70:65:6c` in `arp -a` or the router's list; near a wheel |
| **Log in** | none |
| **Look for** | on the Motor-Pi, whether its packets arrive: `sudo tcpdump -n -c 5 udp port 10111` (if tcpdump is installed); the IR sensor's state (the KEFC request says it is fouled) |
| **Photograph** | the board, the W5500, the spoke sensor |

## Step 5: measure, and the power

The tape-measure list is [Car numbers](car_measurements.md) (Q1-Q9, Q18, Q20, Q22). Also, from car-photos #7:
read the rating off each 12 V → 5 V converter, and note what each one feeds (the dash one
showed 3.5 A; a Pi 5 wants 5 A).

## Step 6: photos

Photograph every label: the GPS, the IMU, the motor controller, both actuators, the lidar,
each Pi's case, the switch. Name the files after the switch label (`RTK-PI-1.jpg`). Photos
stay out of git; pick a few small ones for `docs/img/car/` afterwards.

## Step 7: save the results

Copy this table into `docs/history/audit-2026-10-16-car.md`, one row per machine:

| Machine | Hostname, IP | OS, kernel | ROS, domain | Runs at boot | Devices | Topics seen | Backed up? | Open questions |
|---|---|---|---|---|---|---|---|---|
| JETSON | | | | | | | | |
| CAM1-PI | | | | | | | | |
| CAM2-PI | | | | | | | | |
| RTK-PI | | | | | | | | |
| DASH-UI | | | | | | | | |
| ES-PI | | | | | | | | |
| QNX-PI | | | | | | | | |
| Motor-Pi | | | | | | | | |
| LIDAR | | | n/a | n/a | | | n/a | |
| Router | | | n/a | n/a | n/a | n/a | n/a | |
| ESP32 board | | | n/a | | | n/a | n/a | |
| Encoder Pico 2 | | | n/a | | | n/a | n/a | |

Then answer the car-photos checklist (#1-#7) under it, and list each question (Q1-Q34) the
day answered.

!!! warning "No secrets in the write-up"
    Before you commit, search the file:
    `grep -n -i -E "pass|psk|token|secret|key|ntrip" docs/history/audit-2026-10-16-car.md`.
    For each hit, write only where the secret is and what kind it is (as
    [Older code](older-code.md) does). The raw outputs in `~/aav-audit/` stay off git.

Commit the page on its own branch, then `pixi run docs` if you link it from a page.

## Don't

| Don't | Why |
|---|---|
| `apt upgrade`, `apt install`, `pip install` | it changes what the 2025-26 software runs on |
| Reflash, or move the Orin to JetPack 7.2 | not before a full image ([Platforms](platforms.md)) |
| Change the network: IPs, WiFi, the router, `netplan` | the machines find each other by fixed addresses |
| Start, stop or restart services, or reboot | something may be driving the car |
| Drive, or start our stack (`pixi run car`) | Humble and Jazzy don't mix; nothing is tested on the car |
| Configure the GPS receiver, the radar or the lidar | their settings are part of what we're recording |
| Open the ESP32's update page, or flash any board | no password, and no way back without a backup |
| Copy passwords, tokens, keys or the WiFi password into notes, chat, photos or git | they must be changed, not spread (Q34) |
