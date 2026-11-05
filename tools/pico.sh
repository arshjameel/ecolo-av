#!/usr/bin/env bash
# Our Rust programs for the Pico 2 boards (docs/rust-firmware.md).
#
# usage: tools/pico.sh <spine|wheel|sentinel> <tools|build|flash>   (pixi run pico ...)
#
#   spine     the spine (firmware/spine/boards/pico2)
#   wheel     the wheel encoder, with a W5500 (boards/wheel-encoder-pico2)
#   sentinel  the status display (boards/sentinel-pico2)
#
#   tools     install probe-rs 0.32.0 into vendor/cargo-tools/bin, for flash (once, for
#             every board; several minutes)
#   build     make the board's .uf2 file in its folder: hold BOOTSEL, plug in, drag it on
#   flash     write it onto the Pico 2 through a debug probe, then show its logs.
#             It writes a board: on the bench only.
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
TARGET=thumbv8m.main-none-eabihf

usage() {
  echo "usage: pixi run pico <spine|wheel|sentinel> <tools|build|flash>" >&2
  echo "  spine | wheel | sentinel   which Pico 2: the spine, the wheel encoder, the status display" >&2
  echo "  tools   install probe-rs, for flash (once, for every board)" >&2
  echo "  build   make the .uf2 file to drag on (hold BOOTSEL, plug in)" >&2
  echo "  flash   write it through a debug probe and show its logs (bench only)" >&2
  exit 2
}

[[ $# -eq 2 ]] || usage
case "$1" in
  spine)    dir=pico2                bin=spine-pico2 ;;
  wheel)    dir=wheel-encoder-pico2  bin=wheel-encoder-pico2 ;;
  sentinel) dir=sentinel-pico2       bin=sentinel-pico2 ;;
  *) usage ;;
esac
board=firmware/spine/boards/$dir
elf=target/$TARGET/release/$bin

case "$2" in
  tools)
    exec cargo install --locked --root vendor/cargo-tools probe-rs-tools@0.32.0 ;;
  build)
    cd "$board"
    cargo build --release --locked
    python3 "$ROOT/tools/elf2uf2.py" "$elf" "$bin.uf2"
    echo "made $board/$bin.uf2: hold BOOTSEL, plug the Pico 2 in, drag it on" ;;
  flash)
    probe=$ROOT/vendor/cargo-tools/bin/probe-rs
    [[ -x "$probe" ]] || { echo "no probe-rs yet: run pixi run pico $1 tools first" >&2; exit 1; }
    cd "$board"
    cargo build --release --locked
    exec "$probe" run --chip RP235x "$elf" ;;
  *) usage ;;
esac
