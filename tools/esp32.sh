#!/usr/bin/env bash
# The Rust firmware for today's ESP32 control board: firmware/spine/boards/esp32-umod.
# pixi run esp <step> runs it (the steps below).
# usage: tools/esp32.sh tools          install Espressif's Rust and espflash (once)
#        tools/esp32.sh build [args]   build it (args go to cargo: --no-default-features --features can)
#        tools/esp32.sh flash [args]   build, flash it on the USB cable, show its log (args go to espflash)
#        tools/esp32.sh check          build both links (and version 1 frames), with clippy; "skipped" without the toolchain
#        tools/esp32.sh qemu-tools     download Espressif's QEMU into vendor/qemu (once; macOS arm64)
#        tools/esp32.sh qemu-check [args]  build with --features qemu,uplink, boot it in QEMU, and
#                                      check it brakes when the Motor-Pi's frames stop, its
#                                      uplink, and the RC's three modes (args go to
#                                      tools/esp32_qemu.py); "skipped" without the tools
#
# The ESP32 has Xtensa cores, which the stock Rust (and Pixi's) can't build for. `tools`
# installs, for this user only (no admin rights):
#   - espup and espflash, at fixed versions, into vendor/cargo-tools/bin (git ignores it);
#   - with espup: the Xtensa Rust toolchain "esp" into ~/.rustup/toolchains/esp, and the
#     Xtensa GCC linker beside it;
#   - vendor/esp/export-esp.sh: the two environment variables the build needs.
# It needs rustup (https://rustup.rs): espup adds the "esp" toolchain to it.
# docs/rust-firmware.md has the details.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD

ESPUP_VERSION=0.19.0
ESPFLASH_VERSION=4.3.0          # 4.4 and later need rustc 1.95
XTENSA_RUST_VERSION=1.97.0.0
XTENSA_GCC_VERSION=16.2.0_20260914

# Espressif's QEMU fork (GPL-2.0; a tool here, never linked in), from Espressif's
# releases page (espressif/qemu), and the one library of its that Pixi's environment
# lacks: libgcrypt (LGPL-2.1+), with libgpg-error (LGPL-2.1), from conda-forge. Pixi's
# environment has its other two, pixman and SDL2. The SHA-256s are the published ones.
QEMU_RELEASE=esp-develop-9.2.2-20260417
QEMU_TARBALL=qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-aarch64-apple-darwin.tar.xz
QEMU_SHA256=bb8c15810565d3df1665dc34962430885e11bc95575b228fb44698146be1e9d6
QEMU_URL=https://github.com/espressif/qemu/releases/download/$QEMU_RELEASE/$QEMU_TARBALL
CONDA_FORGE=https://conda.anaconda.org/conda-forge/osx-arm64
QEMU_LIBS="libgcrypt-lib-1.12.2-hb001647_2:45b1b91ff0b7dcfb2d31ffcdc156be2cb93998d80764f9a1f4bbd6a51e5a318a
libgpg-error-1.61-h784d473_2:eb84229d2ac791a4241044dc3ad172389beb4a730cc4a2afb1ddbe51e8233d11"
QEMU_DIR=$ROOT/vendor/qemu
QEMU_BIN=$QEMU_DIR/$QEMU_RELEASE/qemu/bin/qemu-system-xtensa

TOOLS=$ROOT/vendor/cargo-tools/bin
EXPORT_FILE=$ROOT/vendor/esp/export-esp.sh
BOARD=$ROOT/firmware/spine/boards/esp32-umod
ELF=$BOARD/target/xtensa-esp32-none-elf/release/spine-esp32-umod

have_toolchain() {
  [[ -f $EXPORT_FILE ]] && command -v rustup >/dev/null &&
    rustup toolchain list | grep -qE '^esp( |$)'
}

need_toolchain() {
  if ! have_toolchain; then
    echo "No Xtensa Rust here: run 'pixi run esp tools' first (docs/rust-firmware.md)." >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  . "$EXPORT_FILE"
  # The esp toolchain's own cargo and rustc first: under `pixi run`, Pixi's stock Rust
  # comes first on PATH, and cargo would build with its rustc.
  PATH="$(dirname "$(rustup which --toolchain esp rustc)"):$PATH"
  export PATH
}

# Download a file, and check its SHA-256; a mismatch deletes it.
fetch() {
  local url=$1 out=$2 want=$3
  curl -fsSL -o "$out" "$url"
  if [[ "$(shasum -a 256 "$out" | cut -d' ' -f1)" != "$want" ]]; then
    rm -f "$out"
    echo "checksum mismatch: $url" >&2
    exit 1
  fi
}

qemu_platform_ok() {
  [[ "$(uname -s)-$(uname -m)" == Darwin-arm64 ]]
}

# QEMU looks for its libraries in Homebrew's folders; these are the fallbacks.
qemu_env() {
  DYLD_FALLBACK_LIBRARY_PATH="$QEMU_DIR/lib:${CONDA_PREFIX:-$ROOT/.pixi/envs/default}/lib"
  export DYLD_FALLBACK_LIBRARY_PATH
}

qemu_tools() {
  if ! qemu_platform_ok; then
    echo "Only set up for macOS on Apple silicon so far: Espressif publishes QEMU for" \
         "Linux too, but its libraries aren't wired up here." >&2
    exit 1
  fi
  local tmp entry name want
  tmp=$(mktemp -d)
  # shellcheck disable=SC2064
  trap "rm -rf '$tmp'" EXIT
  mkdir -p "$QEMU_DIR/$QEMU_RELEASE" "$QEMU_DIR/lib"
  fetch "$QEMU_URL" "$tmp/qemu.tar.xz" "$QEMU_SHA256"
  tar -xJf "$tmp/qemu.tar.xz" -C "$QEMU_DIR/$QEMU_RELEASE"
  # A .conda package is a zip holding pkg-*.tar.zst: Pixi's bsdtar reads both.
  for entry in $QEMU_LIBS; do
    name=${entry%%:*} want=${entry##*:}
    fetch "$CONDA_FORGE/$name.conda" "$tmp/$name.conda" "$want"
    mkdir -p "$tmp/$name"
    bsdtar -xf "$tmp/$name.conda" -C "$tmp/$name"
    bsdtar -xf "$tmp/$name/pkg-$name.tar.zst" -C "$tmp/$name"
    cp "$tmp/$name"/lib/*.dylib "$QEMU_DIR/lib/"
  done
  qemu_env
  "$QEMU_BIN" --version | head -1
  echo "Installed: $QEMU_BIN, and libgcrypt in $QEMU_DIR/lib."
}

build() {
  need_toolchain
  (cd "$BOARD" && cargo build --release --locked "$@")
}

case "${1:-}" in
  tools)
    command -v rustup >/dev/null || {
      echo "espup needs rustup: install it from https://rustup.rs, then run this again." >&2
      exit 1
    }
    cargo install --locked --root "$ROOT/vendor/cargo-tools" \
      "espup@$ESPUP_VERSION" "espflash@$ESPFLASH_VERSION"
    mkdir -p "$(dirname "$EXPORT_FILE")"
    "$TOOLS/espup" install --targets esp32 \
      --toolchain-version "$XTENSA_RUST_VERSION" \
      --crosstool-toolchain-version "$XTENSA_GCC_VERSION" \
      --export-file "$EXPORT_FILE"
    echo "Installed: espup and espflash in vendor/cargo-tools/bin; Xtensa Rust" \
         "$XTENSA_RUST_VERSION in ~/.rustup/toolchains/esp; $EXPORT_FILE."
    ;;
  build)
    shift
    build "$@"
    ;;
  flash)
    shift
    build
    "$TOOLS/espflash" flash --chip esp32 --monitor "$ELF" "$@"
    ;;
  qemu-tools)
    qemu_tools
    ;;
  qemu-check)
    shift
    if ! have_toolchain || [[ ! -x $QEMU_BIN ]]; then
      echo "skipped: needs the Xtensa Rust (pixi run esp tools) and QEMU (pixi run esp qemu-tools)"
      exit 0
    fi
    # With the uplink: the check reads its frames on UART1, and feeds iBUS on UART2.
    build -q --features qemu,uplink --target-dir target/qemu
    image=$BOARD/target/qemu/flash.bin
    "$TOOLS/espflash" save-image --chip esp32 --merge --flash-size 4mb \
      "$BOARD/target/qemu/xtensa-esp32-none-elf/release/spine-esp32-umod" "$image" \
      >/dev/null 2>&1
    qemu_env
    python3 tools/esp32_qemu.py --qemu "$QEMU_BIN" --flash "$image" "$@"
    ;;
  check)
    if ! have_toolchain; then
      echo "skipped: no Xtensa Rust here (pixi run esp tools)"
      exit 0
    fi
    need_toolchain
    cd "$BOARD"
    cargo fmt --check
    cargo clippy -q --release --locked -- -D warnings
    cargo clippy -q --release --locked --no-default-features \
      --features can,estop-input --target-dir target/can -- -D warnings
    cargo clippy -q --release --locked --features qemu --target-dir target/qemu -- -D warnings
    cargo clippy -q --release --locked --features link-v1 --target-dir target/v1 -- -D warnings
    cargo clippy -q --release --locked --features uplink --target-dir target/uplink -- -D warnings
    cargo clippy -q --release --locked --features qemu,uplink --target-dir target/qemu -- -D warnings
    cargo build -q --release --locked
    echo "builds: the Motor-Pi UART link (version 2, the default; version 1; with the uplink), CAN, and the QEMU builds"
    ;;
  *)
    sed -n '2,13p' "$0" >&2
    exit 2
    ;;
esac
