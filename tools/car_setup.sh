#!/usr/bin/env bash
# Set up one car computer for its role: back up, upgrade, install our code, start it at
# boot, check. Run it ON that computer, as your normal user (it asks for sudo itself).
# The runbook: docs/car-setup.md.
#
# usage: bash car_setup.sh <role> [step...] [--apply] [options]
#
#   roles   orin          the Jetson AGX Orin (JETSON, .140)
#           motor_pi      the Pi wired to the ESP32 (CORTEX, .120; H1)
#           localization  the IMU and GPS Pi (LOCALIZATION, .113)
#           dash          the dashboard Pi with the two screens (aav-desktop, .196)
#           camera_pi     a camera Pi on Pi OS, MediaMTX (CAM1-PI .108, CAM2-PI .118)
#           lab_desktop   the lab's x86 Ubuntu desktop with CARLA (stays in the lab)
#   steps   backup upgrade install services check   (none given: all five, in this order)
#
#   --apply            really do it. Without it, a dry run: every step is printed, nothing
#                      changes. With it, you type the role first, to show it's the right computer
#   --release-upgrade  in `upgrade`: run do-release-upgrade (Ubuntu 22.04 to 24.04,
#                      interactive). Without it, the command is only printed
#   --kit DIR          the car kit, for `install` (default ~/car-kit)
#   --machine NAME     the machine file aav-car.service starts (default: the role's;
#                      motor_pi_alone for the first moves)
#   --address A/NN     `services`: a fixed address beside DHCP's (such as 192.168.1.120/24)
#   --interface NAME   the car's network card (default: the one on 192.168.1.x)
#   --host-ok          this hostname isn't the one the audit found for the role, and you
#                      checked it's the right computer (a new SD card, a reflash)
#   --after-esp32      `upgrade` on the motor_pi: the Rust ESP32 is flashed and car-day
#                      session 6 passed, so the 2025-26 C++ path may end (Day B)
#
# STOP lines: a pending reboot; a hostname that isn't the role's (unless --host-ok);
# `upgrade` on the localization Pi (no machine file for its IMU and GPS yet) or on the
# motor_pi without --after-esp32. With --apply a STOP before the steps changes nothing.
# `check` only reads, so it runs in a dry run too. It never sends email, never copies a
# password: logins live in the team's password vault (car-day.md, "Logins").
# Portable: bash 3.2, no GNU-only flags (its dry run also runs on a Mac).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROLES="orin motor_pi localization dash camera_pi lab_desktop"
STEPS="backup upgrade install services check"
OS_RELEASE=${AAV_OS_RELEASE:-/etc/os-release}
REBOOT_FLAG=${AAV_REBOOT_FLAG:-/var/run/reboot-required}   # the tests point it elsewhere
TODAY=$(date +%Y-%m-%d)
NEED_UPGRADE_GB=5

usage() { sed -n '2,37p' "$0"; }

role="" steps="" apply=0 release=0 host_ok=0 after_esp32=0 kit="$HOME/car-kit" machine="" address="" iface=""
while (($#)); do
  case "$1" in
    --apply) apply=1 ;;
    --release-upgrade) release=1 ;;
    --host-ok) host_ok=1 ;;
    --after-esp32) after_esp32=1 ;;
    --kit) kit=${2:?--kit needs a folder}; shift ;;
    --machine) machine=${2:?--machine needs a name}; shift ;;
    --address) address=${2:?--address needs A/NN}; shift ;;
    --interface) iface=${2:?--interface needs a name}; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    *)
      if [[ -z "$role" ]]; then role=$1
      else steps="$steps $1"; fi ;;
  esac
  shift
done

if [[ -z "$role" ]]; then
  echo "Which computer is this? Give its role: $ROLES (docs/car-setup.md)" >&2
  exit 2
fi
case " $ROLES " in *" $role "*) ;; *)
  echo "unknown role: $role. Pick one of: $ROLES" >&2; exit 2 ;;
esac
for s in $steps; do
  case " $STEPS " in *" $s "*) ;; *)
    echo "unknown step: $s. Pick from: $STEPS" >&2; exit 2 ;;
  esac
done
if [[ -z "${steps// /}" ]]; then steps=$STEPS; fi
if [[ -n "$address" && "$address" != */* ]]; then
  echo "--address needs its prefix length, such as 192.168.1.120/24" >&2; exit 2
fi

# The machine file aav-car.service starts, by role. Empty: no ROS service on that one.
default_machine() {
  case "$1" in
    orin) echo orin ;;
    motor_pi) echo motor_pi ;;
    *) echo "" ;;
  esac
}
if [[ -z "$machine" ]]; then machine=$(default_machine "$role"); fi

# The hostname the 21 Oct audit found, by role (it may change on a new SD card).
expected_host() {
  case "$1" in
    orin) echo ubuntu ;;
    motor_pi) echo ubuntu-cortex ;;
    localization) echo localhost.localdomain ;;
    dash) echo aav-desktop ;;
    camera_pi) echo "aav-cam1 aav-cam2" ;;
    *) echo "" ;;
  esac
}

# What this computer is.
host=${AAV_HOSTNAME:-$(hostname 2>/dev/null || echo unknown)}   # the tests set it
kernel=$(uname -s) arch=$(uname -m)
os_id="" os_version="" os_name="unknown"
if [[ -r "$OS_RELEASE" ]]; then
  os_id=$(. "$OS_RELEASE"; echo "${ID:-}")
  os_version=$(. "$OS_RELEASE"; echo "${VERSION_ID:-}")
  os_name=$(. "$OS_RELEASE"; echo "${PRETTY_NAME:-unknown}")
fi
jetson=0
if [[ -f /etc/nv_tegra_release ]]; then jetson=1; fi

problems=0
ok()   { printf '  OK    %-14s %s\n' "$1" "$2"; }
bad()  { printf '  STOP  %-14s %s\n' "$1" "$2"; problems=$((problems + 1)); }
note() { printf '  NOTE  %-14s %s\n' "$1" "$2"; }
step() { printf '\n== %s\n' "$*"; }

# Run a command, or in a dry run only show it. A failure stops the script.
act() {
  if [[ $apply == 1 ]]; then
    printf '  $ %s\n' "$*"
    "$@" || { echo "FAILED: $*  (nothing after it was done)" >&2; exit 1; }
  else
    printf '  would run: %s\n' "$*"
  fi
}
# The same for a line with pipes or redirections.
act_sh() {
  if [[ $apply == 1 ]]; then
    printf '  $ %s\n' "$1"
    bash -c "$1" || { echo "FAILED: $1  (nothing after it was done)" >&2; exit 1; }
  else
    printf '  would run: %s\n' "$1"
  fi
}
# Write a file from stdin (sudo for a system file), or in a dry run show it.
put() {   # path mode [sudo]
  local path=$1 mode=$2 how=${3:-}
  local body
  body=$(cat)
  if [[ $apply == 1 ]]; then
    printf '  writing %s\n' "$path"
    if [[ "$how" == sudo ]]; then
      printf '%s\n' "$body" | sudo tee "$path" >/dev/null && sudo chmod "$mode" "$path"
    else
      mkdir -p "$(dirname "$path")"
      printf '%s\n' "$body" > "$path" && chmod "$mode" "$path"
    fi
  else
    printf '  would write %s:\n' "$path"
    printf '%s\n' "$body" | sed 's/^/      | /'
  fi
}

# The repo's files (the units): beside this script, else in the installed copy.
repo() {
  if [[ -f "$HERE/systemd/aav-car.service" ]]; then (cd "$HERE/.." && pwd)
  elif [[ -f "$HOME/aav-jazzy/aav/tools/systemd/aav-car.service" ]]; then echo "$HOME/aav-jazzy/aav"
  else echo ""; fi
}

# The card on the car's network (192.168.1.x), for Cyclone and netplan.
car_card() {
  if [[ -n "$iface" ]]; then echo "$iface"; return; fi
  if command -v ip >/dev/null; then
    ip -br addr 2>/dev/null | awk '/ 192\.168\.1\./ {print $1; exit}'
  fi
}

ubuntu_role() { case "$1" in orin|motor_pi|localization|dash|lab_desktop) return 0 ;; esac; return 1; }
ros_role() { case "$1" in orin|motor_pi|localization|dash) return 0 ;; esac; return 1; }

printf 'car_setup.sh: %s on %s (%s, %s)\n' "$role" "$host" "$os_name" "$arch"
if [[ $apply == 0 ]]; then
  printf 'Dry run: nothing changes. Add --apply to do it.\n'
fi

# Before any step: the right computer, and no reboot waiting.
step "before: the right computer, no reboot waiting"
want=$(expected_host "$role")
if [[ -z "$want" ]]; then
  note "hostname" "$host (no hostname on record for the $role)"
else
  case " $want " in
    *" $host "*) ok "hostname" "$host, as the audit found the $role" ;;
    *)
      if [[ $host_ok == 1 ]]; then
        note "hostname" "$host, not \"$want\": you said it's right (--host-ok)"
      elif [[ "$kernel" != Linux ]]; then
        note "hostname" "$host, not \"$want\": fine for a rehearsal on $kernel"
      else
        bad "hostname" "$host, not \"$want\" (the audit's $role). The wrong computer? Check its address and hosts.txt; if it's right (a new card, a reflash), add --host-ok"
      fi ;;
  esac
fi
if [[ -f "$REBOOT_FLAG" ]]; then
  bad "reboot" "a reboot is waiting: sudo reboot, log in again, then run this again"
else
  ok "reboot" "none waiting"
fi

# Real steps: only on the car's Linux, as a normal user, after typing the role.
needs_apply=0
for s in $steps; do [[ "$s" != check ]] && needs_apply=1; done
if [[ $apply == 1 && $needs_apply == 1 ]]; then
  if (( problems > 0 )); then
    echo "Refused: $problems STOP line(s) above. Nothing changed." >&2
    exit 1
  fi
  if [[ "$kernel" != Linux ]]; then
    echo "Refused: this is $kernel. --apply runs on the car's Linux computers only." >&2
    exit 1
  fi
  if [[ "$(id -u)" == 0 ]]; then
    echo "Refused: run it as your normal user, not root: it asks for sudo itself." >&2
    exit 1
  fi
  printf '\nSteps: %s\nType this computer'"'"'s role (%s) to go on: ' "${steps# }" "$role"
  typed=""
  read -r typed || true
  if [[ "$typed" != "$role" ]]; then
    echo "Stopped: you typed \"$typed\", not \"$role\". Nothing changed." >&2
    exit 1
  fi
fi

do_backup() {
  local dir="$HOME/aav-backup/$host-$TODAY"
  step "backup: what only this computer has, into $dir (docs/car-setup.md, step 1)"
  echo "  The home folder and the full disk image are taken from the laptop (the page says how)."
  act mkdir -p "$dir"
  act chmod 700 "$dir"
  act_sh "cp $OS_RELEASE '$dir/os-release'; uname -a > '$dir/uname.txt'"
  if [[ $jetson == 1 ]]; then act cp /etc/nv_tegra_release "$dir/nv_tegra_release"; fi
  {
    act_sh "dpkg --get-selections > '$dir/packages.txt'"
    act_sh "apt-mark showmanual > '$dir/packages-manual.txt'"
    act_sh "systemctl list-unit-files --state=enabled > '$dir/services.txt'"
    act_sh "systemctl --user list-unit-files > '$dir/user-services.txt' || true"
    act_sh "crontab -l > '$dir/crontab.txt' 2>/dev/null || true"
    act_sh "ip -br addr > '$dir/addresses.txt' || true"
    # /etc holds WiFi passwords and keys: the backup drive is encrypted, never git.
    act_sh "sudo tar czf '$dir/etc.tgz' /etc && sudo chown \$(id -un) '$dir/etc.tgz'"
  }
  echo "  Then, on the laptop (the user and address from hosts.txt; the password: the Keychain):"
  echo "    rsync -a --exclude .cache --exclude aav-jazzy <user>@<address>:/home/<user>/ ~/aav-backup/$host-$TODAY/home/"
}

# The machine file our stack would start on this role, in the repo or the installed copy.
machine_file() {
  local r
  r=$(repo)
  [[ -n "$r" && -f "$r/ros/bringup/aav_bringup/config/machines/$1.yaml" ]]
}

do_upgrade() {
  step "upgrade: this release's updates (docs/car-setup.md, step 2)"
  # Day B only, and not these two until their way forward exists (car-day.md, Day B).
  if [[ "$role" == localization ]] && ! machine_file localization; then
    bad "upgrade" "refused: its IMU and GPS drivers run on the 2025-26 Humble, which 24.04 ends, and no machine file of ours (machines/localization.yaml) runs them yet. Upgrading now loses the car's IMU and GPS"
    return
  fi
  if [[ "$role" == motor_pi && $after_esp32 == 0 ]]; then
    bad "upgrade" "refused: the 2025-26 aav_serial_bridge (Humble) is the way back to the C++ ESP32, and 24.04 ends it. Only once the Rust ESP32 is flashed and car-day session 6 passed: add --after-esp32"
    return
  fi
  act sudo apt-get update
  act sudo apt-get full-upgrade
  act_sh "if [ -f $REBOOT_FLAG ]; then echo '  REBOOT NOW: sudo reboot, then run this step again'; fi"

  step "upgrade: the next release"
  if [[ "$role" == orin || $jetson == 1 ]]; then
    note "release" "JetPack: never do-release-upgrade. JetPack 7.2 is a reflash from the lab desktop (the page, step 6)"
    return
  fi
  case "$os_id" in
    debian|raspbian)
      note "release" "Pi OS ($os_name) can't become Ubuntu in place: a new SD card (the page, step 3)"
      [[ "$role" == camera_pi ]] && note "camera Pi" "stays on Pi OS (whats-new.md): its updates above are enough"
      return ;;
  esac
  if [[ "$os_id" == ubuntu && "$os_version" == 24.04 ]]; then
    ok "release" "already Ubuntu 24.04"
    return
  fi
  if [[ "$os_id" != ubuntu || "$os_version" != 22.04 ]]; then
    note "release" "$os_name: not Ubuntu 22.04, so no release upgrade here"
    return
  fi
  local free_kb free_gb
  free_kb=$(df -Pk / | awk 'NR == 2 {print $4}')
  free_gb=$((free_kb / 1024 / 1024))
  if (( free_gb < NEED_UPGRADE_GB )); then
    bad "disk" "$free_gb GB free on /, the release upgrade needs $NEED_UPGRADE_GB"
    return
  fi
  echo "  Ubuntu 22.04 to 24.04. Humble (/opt/ros/humble) is built for 22.04: after this the"
  echo "  2025-26 software on this computer stops working. The backup is the way back."
  echo "  Over ssh it opens a spare ssh on port 1022, in case this one drops."
  [[ "$role" == lab_desktop ]] && echo "  The lab desktop: check CARLA's version supports 24.04 first (docs/carla.md)."
  if [[ $release == 1 ]]; then
    act sudo do-release-upgrade
  else
    echo "  Not run (add --release-upgrade). When ready, after a reboot:"
    echo "    sudo do-release-upgrade"
  fi
}

do_install() {
  step "install: our code (docs/car-setup.md, step 4)"
  case "$role" in
    camera_pi)
      note "camera Pi" "no ROS here: MediaMTX and our mediamtx.yml, from the laptop (car-day.md, session 5)"
      return ;;
    lab_desktop)
      if [[ -d "$HOME/ecolo-av/.git" ]]; then
        act git -C "$HOME/ecolo-av" pull --ff-only
      else
        act git clone https://github.com/AAV-2025-2026/ecolo-av.git "$HOME/ecolo-av"
      fi
      if command -v pixi >/dev/null; then
        act_sh "cd '$HOME/ecolo-av' && pixi run build"
      else
        note "pixi" "not installed: curl -fsSL https://pixi.sh/install.sh | bash, then a new terminal (docs/getting-started.md)"
      fi
      return ;;
  esac
  local flags=""
  [[ "$role" == dash ]] && flags=" --screens"
  if [[ ! -f "$kit/car_install.sh" ]]; then
    if [[ $apply == 1 ]]; then
      bad "kit" "no $kit/car_install.sh. From the laptop: scp -r car-kit <user>@<address>:~/"
      return
    fi
    note "kit" "no $kit/car_install.sh yet: scp -r car-kit <user>@<address>:~/ first"
  fi
  act bash "$kit/car_install.sh" --dry-run$flags
  act bash "$kit/car_install.sh"$flags
}

do_services() {
  step "services: start at boot (docs/car-setup.md, step 5)"
  if [[ "$role" == lab_desktop ]]; then
    note "lab desktop" "nothing starts at boot: CARLA runs by hand (docs/carla.md)"
    return
  fi

  echo "  Time: chrony. The Orin keeps the car's time; the others ask it."
  act sudo apt-get install -y chrony
  if [[ "$role" == orin ]]; then
    put /etc/chrony/conf.d/aav.conf 644 sudo <<'EOF'
# The Orin keeps the car's time: the Pis and laptops ask it (docs/car-setup.md).
allow 192.168.1.0/24
# With no internet, it still serves its own clock, so the car's computers agree.
local stratum 10
EOF
  else
    put /etc/chrony/sources.d/aav.sources 644 sudo <<'EOF'
# The car's time comes from the Orin (docs/car-setup.md).
server 192.168.1.140 iburst prefer
EOF
  fi
  act sudo systemctl restart chrony

  if [[ "$role" == camera_pi ]]; then
    echo "  The camera stream: MediaMTX, the 2025-26 team's service, at boot."
    act sudo systemctl enable mediamtx
  fi

  local card
  card=$(car_card)
  if [[ -n "$address" ]]; then
    if [[ -z "$card" ]]; then
      bad "address" "no card on 192.168.1.x found: give --interface"
    elif [[ "$os_id" == debian || "$os_id" == raspbian ]]; then
      echo "  Pi OS uses NetworkManager: the fixed address beside DHCP's:"
      act_sh "sudo nmcli connection modify \"\$(nmcli -g GENERAL.CONNECTION device show $card)\" +ipv4.addresses $address && sudo nmcli device reapply $card"
    else
      echo "  A fixed address beside DHCP's (DHCP keeps the internet; the router's reservation is better)."
      put /etc/netplan/60-aav-address.yaml 600 sudo <<EOF
# A fixed address on the car's network, beside DHCP's (docs/car-setup.md).
network:
  version: 2
  ethernets:
    $card:
      dhcp4: true
      addresses: [$address]
EOF
      echo "  netplan try keeps the old settings unless you press Enter within 120 s: safe over ssh."
      act sudo netplan try
    fi
  fi

  if ! ros_role "$role"; then return; fi

  local r
  r=$(repo)
  if [[ -z "$r" ]]; then
    if [[ $apply == 1 ]]; then bad "repo" "no tools/systemd here nor in ~/aav-jazzy/aav: run the install step first"; return; fi
    r="$HOME/aav-jazzy/aav"
  fi

  echo "  Devices: groups (serial, USB, joystick, cameras) and the udev names (/dev/aav/...)."
  act sudo usermod -aG dialout,plugdev,input,video "$(id -un)"
  if [[ "$role" != dash ]]; then
    act sudo cp "$r/tools/udev/99-aav.rules" /etc/udev/rules.d/99-aav.rules
    act sudo udevadm control --reload
    act sudo udevadm trigger
  fi

  echo "  ROS: Cyclone on the car's card (aav.sh sets domain 42)."
  if [[ -f "$HOME/.config/aav/cyclonedds.xml" ]]; then
    ok "Cyclone" "$HOME/.config/aav/cyclonedds.xml is there: kept"
  elif [[ -z "$card" ]]; then
    note "Cyclone" "no card on 192.168.1.x yet: write ~/.config/aav/cyclonedds.xml by hand (car-day.md, 4c)"
  else
    put "$HOME/.config/aav/cyclonedds.xml" 644 <<EOF
<!-- Written by car_setup.sh: Cyclone DDS on the car's network card only (car-day.md, 4c). -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$card"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
  fi

  if [[ "$role" == dash ]]; then
    echo "  The screens: install --screens set up aav-hud and aav-map (docs/car-screens.md)."
    echo "  They start with the desktop: turn on automatic login (Settings > Users), once, by hand."
    return
  fi
  if [[ -z "$machine" ]]; then
    note "ROS" "no machine file for the $role yet (orin.yaml's IMU and GPS TODO): nothing starts here"
    return
  fi

  if [[ "$role" == motor_pi ]] && command -v systemctl >/dev/null \
     && systemctl is-enabled aav_serial_bridge >/dev/null 2>&1; then
    bad "two paths" "the 2025-26 aav_serial_bridge starts at boot. Never two drive paths: when the team agrees, sudo systemctl disable --now aav_serial_bridge, then this step again"
    return
  fi

  echo "  Our stack at boot: aav-car.service (a user unit), machine $machine."
  act mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/aav"
  act cp "$r/tools/systemd/aav-car.service" "$HOME/.config/systemd/user/aav-car.service"
  sed "s/@MACHINE@/$machine/" "$r/tools/systemd/car.env" 2>/dev/null \
    | put "$HOME/.config/aav/car.env" 644
  act sudo loginctl enable-linger "$(id -un)"
  act systemctl --user daemon-reload
  act systemctl --user enable aav-car
  echo "  It starts at the next boot (sudo reboot: the new groups need it too)."
  echo "  Its log: journalctl --user -u aav-car -f"
}

do_check() {
  step "check: read-only (docs/car-setup.md, the checklist)"
  if [[ "$kernel" == Linux ]]; then ok "system" "$os_name, $arch"; else note "system" "$kernel: not a car computer"; fi
  case "$role:$os_id:$os_version" in
    camera_pi:debian:*|camera_pi:raspbian:*) ok "release" "Pi OS: right for a camera Pi" ;;
    lab_desktop:ubuntu:*) ok "release" "Ubuntu $os_version" ;;
    *:ubuntu:24.04) ok "release" "Ubuntu 24.04" ;;
    orin:ubuntu:22.04) note "release" "JetPack 6 (22.04): our kit runs; 7.2 is the reflash" ;;
    *:ubuntu:22.04) note "release" "Ubuntu 22.04: the upgrade step's do-release-upgrade is next" ;;
    *) note "release" "${os_name}" ;;
  esac
  if [[ $jetson == 1 ]]; then note "JetPack" "$(head -1 /etc/nv_tegra_release)"; fi
  local free_kb
  free_kb=$(df -Pk "$HOME" | awk 'NR == 2 {print $4}')
  note "disk" "$((free_kb / 1024 / 1024)) GB free in $HOME"
  if [[ -d /opt/ros/humble ]]; then note "2025-26" "/opt/ros/humble is here"; fi

  if command -v chronyc >/dev/null; then
    ok "time" "$(chronyc tracking 2>/dev/null | awk -F': ' '/^Reference ID/ {print "from " $2}')"
  elif [[ "$role" != lab_desktop ]]; then
    note "time" "no chrony yet (the services step)"
  fi

  if ros_role "$role"; then
    if [[ -x "$HOME/aav-jazzy/aav.sh" ]]; then ok "our stack" "$HOME/aav-jazzy/aav.sh"; else note "our stack" "not installed (the install step)"; fi
    if [[ -f "$HOME/.config/aav/cyclonedds.xml" ]]; then ok "Cyclone" "~/.config/aav/cyclonedds.xml"; else note "Cyclone" "no ~/.config/aav/cyclonedds.xml"; fi
    if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then ok "groups" "$(id -nG)"; else note "groups" "not in dialout (the services step, then a reboot)"; fi
  fi
  # After the upgrade (Ubuntu 24.04), the devices must come back: a STOP if not.
  local upgraded=0
  [[ "$kernel" == Linux && "$os_id" == ubuntu && "$os_version" == 24.04 ]] && upgraded=1
  if [[ "$role" != dash ]] && ros_role "$role"; then
    local names
    names=$(ls /dev/aav 2>/dev/null | tr '\n' ' ' || true)
    if [[ ! -f /etc/udev/rules.d/99-aav.rules ]]; then
      note "udev" "no 99-aav.rules (the services step)"
    elif [[ -n "$names" ]]; then
      ok "udev" "/dev/aav: $names"
    elif [[ $upgraded == 1 && ( "$role" == orin || "$role" == localization ) ]]; then
      bad "udev" "nothing in /dev/aav after the upgrade: the sensors' cables, then sudo udevadm trigger"
    else
      note "udev" "nothing in /dev/aav yet"
    fi
  fi
  if [[ "$role" == motor_pi ]]; then
    if [[ -e /dev/ttyAMA0 ]]; then ok "UART" "/dev/ttyAMA0"
    elif [[ "$kernel" != Linux ]]; then note "UART" "no /dev/ttyAMA0 (not Linux)"
    elif [[ $upgraded == 1 ]]; then bad "UART" "no /dev/ttyAMA0 after the upgrade: enable_uart=1 in /boot/firmware/config.txt, then reboot (H1)"
    else bad "UART" "no /dev/ttyAMA0: enable_uart=1 in /boot/firmware/config.txt (H1)"; fi
    if command -v systemctl >/dev/null && systemctl is-active aav_serial_bridge >/dev/null 2>&1; then
      note "2025-26" "aav_serial_bridge is running: theirs holds the ESP32's port"
    fi
    if ls /dev/input/js* >/dev/null 2>&1; then ok "joystick" "$(ls /dev/input/js* | tr '\n' ' ')"; else note "joystick" "no /dev/input/js*: its USB receiver?"; fi
  fi
  if [[ "$role" == camera_pi ]] && command -v systemctl >/dev/null; then
    if systemctl is-active mediamtx >/dev/null 2>&1; then ok "MediaMTX" "active"; else note "MediaMTX" "not running"; fi
  fi
  if command -v systemctl >/dev/null && [[ "$kernel" == Linux ]]; then
    if [[ -n "$machine" ]]; then
      note "aav-car" "$(systemctl --user is-enabled aav-car 2>/dev/null || echo 'not installed'), $(systemctl --user is-active aav-car 2>/dev/null || true)"
      note "linger" "$(loginctl show-user "$(id -un)" -p Linger 2>/dev/null || echo unknown)"
    fi
    if [[ "$role" == dash ]]; then
      note "screens" "aav-hud $(systemctl --user is-enabled aav-hud 2>/dev/null || echo 'not installed'), aav-map $(systemctl --user is-enabled aav-map 2>/dev/null || echo 'not installed')"
    fi
  fi
}

for s in $steps; do
  case "$s" in
    backup) do_backup ;;
    upgrade) do_upgrade ;;
    install) do_install ;;
    services) do_services ;;
    check) do_check ;;
  esac
  if [[ $apply == 1 && $problems -gt 0 && "$s" != check ]]; then
    printf '\nStopped after %s: %d STOP line(s). The steps after it were not done.\n' "$s" "$problems" >&2
    exit 1
  fi
done

echo
if (( problems > 0 )); then
  printf '%d STOP line(s): fix them, then run again.\n' "$problems"
  exit 1
fi
if [[ $apply == 0 ]]; then
  printf 'Dry run done: nothing changed. The same with --apply does it.\n'
else
  printf 'Done: %s. Tick it in docs/car-setup.md.\n' "${steps# }"
fi
