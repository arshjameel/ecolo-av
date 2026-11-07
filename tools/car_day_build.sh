#!/usr/bin/env bash
# Build, before car day, what the laptop takes to the car (docs/car-day.md):
#   car-day/esp32/spine-esp32-umod-v2.elf   the Rust ESP32 firmware, version 2 frames (the
#                                           Orin's heartbeat): what goes on the board
#   car-day/esp32/spine-esp32-umod-v1.elf   the same with the 13-byte frames (link-v1), in
#                                           case the Motor Pi's bridge must stay on version 1
#   car-day/esp32/spine-esp32-umod-v2-uplink.elf  version 2 with the uplink on TX1 and the
#                                           three-position RC switch: only once H9 and H10
#                                           say yes (docs/esp32-bench.md)
#   car-day/esp32/qemu-check.log            esp qemu-check: it brakes when the heartbeat stops
#   car-day/esp32/legacy-esp32.log          the 2025-26 C++ still builds: the way back
#   car-day/esp32/BUILD.txt, SHA256SUMS     when, from which commit, each step's result
# and the docs (pixi run docs: the PDF, to read with no internet).
# pixi run car-day-check reads BUILD.txt. car-day/ is out of git: make it again any time.
#
# usage: pixi run car-day-build       (about 3-5 min; the ESP32 needs pixi run esp tools,
#                                      QEMU pixi run esp qemu-tools)
# Flash one at the car: vendor/cargo-tools/bin/espflash flash --chip esp32 --monitor \
#                         car-day/esp32/spine-esp32-umod-v2.elf
# Portable: bash 3.2, no GNU-only flags.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
OUT=car-day/esp32
BOARD=firmware/spine/boards/esp32-umod
LEGACY=firmware/legacy/aav_umod_esp32
ELF=xtensa-esp32-none-elf/release/spine-esp32-umod
mkdir -p "$OUT"
rm -f "$OUT"/*.elf "$OUT"/BUILD.txt "$OUT"/SHA256SUMS

# The committed tree of a folder, "-dirty" if it has uncommitted changes: the check
# compares it with today's, so a firmware edit makes the build out of date.
tree_id() {
  local id
  id=$(git rev-parse "HEAD:$1" 2>/dev/null || echo '?')
  [[ -z "$(git status --porcelain -- "$1")" ]] || id="$id-dirty"
  echo "$id"
}

step() { printf '\n== %s\n' "$*"; }
FAILED=0
# PASS or FAIL for an exit code, and FAILED=1 for a FAIL. Not in $(...): that's a subshell.
RESULT=
result() {
  if [[ $1 == 0 ]]; then RESULT=PASS; else RESULT="FAIL (exit $1)"; FAILED=1; fi
  echo "$RESULT"
}

step "The Rust ESP32 firmware, version 2 frames (the default)"
tools/esp32.sh build -q --no-default-features --features legacy-uart --target-dir target/no-uplink
rc=$?
[[ $rc == 0 ]] && cp "$BOARD/target/no-uplink/$ELF" "$OUT/spine-esp32-umod-v2.elf"
result $rc
V2=$RESULT

step "The same, version 1 frames (link-v1)"
tools/esp32.sh build -q --no-default-features --features legacy-uart,link-v1 --target-dir target/v1
rc=$?
[[ $rc == 0 ]] && cp "$BOARD/target/v1/$ELF" "$OUT/spine-esp32-umod-v1.elf"
result $rc
V1=$RESULT

step "Version 2 with the uplink and the three-position RC switch (uplink)"
tools/esp32.sh build -q --features uplink --target-dir target/uplink
rc=$?
[[ $rc == 0 ]] && cp "$BOARD/target/uplink/$ELF" "$OUT/spine-esp32-umod-v2-uplink.elf"
result $rc
UPLINK=$RESULT

step "esp qemu-check: the firmware in QEMU brakes when the Orin's heartbeat stops"
tools/esp32.sh qemu-check > "$OUT/qemu-check.log" 2>&1
rc=$?
tail -3 "$OUT/qemu-check.log"
if grep -q '^skipped' "$OUT/qemu-check.log"; then
  QEMU='skipped: no QEMU here (pixi run esp qemu-tools)'
  FAILED=1
  echo "$QEMU"
else
  result $rc
  QEMU=$RESULT
fi

step "The 2025-26 C++ firmware builds: the way back (PlatformIO)"
# The other environments, without the default one's PYTHONPATH (ROS's and Gazebo's
# modules): under it, the docs' Python loaded the default environment's Pillow and failed.
other_env() { env -u PYTHONPATH pixi run "$@"; }
other_env -e firmware legacy-esp32 > "$OUT/legacy-esp32.log" 2>&1
rc=$?
tail -2 "$OUT/legacy-esp32.log"
result $rc
LEGACY_RESULT=$RESULT

step "The docs: the HTML and the PDF (docs/html/aav-docs.pdf)"
other_env docs
result $?
DOCS=$RESULT

cat > "$OUT/BUILD.txt" <<EOF
made: $(date '+%d %b %Y %H:%M')
on: $(uname -n)
commit: $(git log -1 --format='%h %s')
firmware/spine: $(tree_id firmware/spine)
$LEGACY: $(tree_id "$LEGACY")
v2: $V2
v1: $V1
uplink: $UPLINK
esp qemu-check: $QEMU
legacy-esp32: $LEGACY_RESULT
docs: $DOCS
EOF
( cd "$OUT" && for f in *.elf; do
    [[ -f $f ]] && printf '%s  %s\n' "$(shasum -a 256 "$f" | cut -d' ' -f1)" "$f"
  done ) > "$OUT/SHA256SUMS"

step "Done: $ROOT/$OUT"
cat "$OUT/BUILD.txt"
ls -l "$OUT"
[[ $FAILED == 0 ]] || echo "Something failed: read the logs above, then pixi run car-day-check."
exit $FAILED
