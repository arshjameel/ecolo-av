#!/usr/bin/env bash
# Make the USB kit that installs our Jazzy stack on the car's ARM computers with no internet.
# Run it on a Mac (or any computer with Pixi and internet); copy the folder to a stick.
#
# usage: pixi run car-kit                (writes car-kit/, which git ignores)
#        tools/car_kit.sh [<folder>]
#
# The kit holds:
#   aav.bundle                      the repo (git bundle: HEAD and main)
#   aav-src.tar.gz                  the same code, for a computer without git
#   aav-jazzy-linux-aarch64.tar     the default Pixi environment for linux-aarch64, from
#                                   pixi.lock, packed by pixi-pack (every package, offline)
#   bin/pixi-unpack, bin/pixi       their linux-aarch64 programs, pinned and checksummed
#   rust-crates.tar.gz              every crate our Rust packages use (cargo vendor)
#   models/                         the YOLO model files in models/, if any
#   maps/                           the map window's street map and place database, if
#                                   downloaded (pixi run get street-map, get places)
#   car_install.sh, car_setup.sh, README.md, KIT.txt, SHA256SUMS
#
# Downloads are cached in vendor/car-kit-cache, so a second run is quick.
# Portable: bash 3.2, no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
OUT=${1:-car-kit}
CACHE=vendor/car-kit-cache

# Pinned: pixi-pack and its pixi-unpack (github.com/Quantco/pixi-pack), and pixi itself at
# the version CI uses (.github/workflows/test.yml, PIXI_VERSION). musl builds: one static
# file each, so any glibc on the car will do.
PIXI_PACK_VERSION=0.7.11
PIXI_UNPACK_URL=https://github.com/Quantco/pixi-pack/releases/download/v$PIXI_PACK_VERSION/pixi-unpack-aarch64-unknown-linux-musl
PIXI_UNPACK_SHA256=7cf766c38436406fedc5f4eb9143b5433487e20358d8d1080c8b44b40c43d454
PIXI_VERSION=0.81.0
PIXI_URL=https://github.com/prefix-dev/pixi/releases/download/v$PIXI_VERSION/pixi-aarch64-unknown-linux-musl
PIXI_SHA256=7fcd20f7c18339f13552a6767ab146ce6b439ce29db46cc03ece005ca9696bb1
PACK=aav-jazzy-linux-aarch64.tar

grep -q "PIXI_VERSION: v$PIXI_VERSION\$" .github/workflows/test.yml \
  || echo "WARNING: CI's PIXI_VERSION isn't v$PIXI_VERSION: update PIXI_VERSION here" >&2

sha256() {
  if command -v sha256sum >/dev/null; then sha256sum "$1" | awk '{print $1}'
  else shasum -a 256 "$1" | awk '{print $1}'; fi
}

# Download a pinned file into the cache once, and check it.
fetch() {   # url sha256 name
  local file="$CACHE/bin/$3"
  mkdir -p "$CACHE/bin"
  if [[ ! -f "$file" || "$(sha256 "$file")" != "$2" ]]; then
    curl -fsSL -o "$file.part" "$1"
    mv "$file.part" "$file"
  fi
  if [[ "$(sha256 "$file")" != "$2" ]]; then
    echo "checksum mismatch for $1: not using it" >&2
    rm -f "$file"
    exit 1
  fi
  echo "$file"
}

step() { printf '\n== %s\n' "$*"; }
T0=$(date +%s)

if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "WARNING: uncommitted changes aren't in the kit: only commits are" >&2
fi
rm -rf "$OUT"
mkdir -p "$OUT/bin"

step "The code: $(git log --oneline -1)"
git bundle create "$OUT/aav.bundle" HEAD main 2>&1 | tail -1
git archive --format=tar.gz -o "$OUT/aav-src.tar.gz" HEAD

step "The environment: default, linux-aarch64, from pixi.lock (pixi-pack $PIXI_PACK_VERSION)"
t=$(date +%s)
pixi exec --spec "pixi-pack==$PIXI_PACK_VERSION" pixi-pack --environment default \
  --platform linux-aarch64 --use-cache "$CACHE/packages" --output-file "$OUT/$PACK" pixi.toml
echo "took $(( $(date +%s) - t )) s"

step "pixi-unpack and pixi for linux-aarch64"
cp "$(fetch "$PIXI_UNPACK_URL" "$PIXI_UNPACK_SHA256" pixi-unpack)" "$OUT/bin/pixi-unpack"
cp "$(fetch "$PIXI_URL" "$PIXI_SHA256" pixi)" "$OUT/bin/pixi"
chmod +x "$OUT/bin/pixi-unpack" "$OUT/bin/pixi"

step "Rust crates for our Rust packages (cargo vendor)"
# Every Rust workspace colcon builds has its Cargo.lock in ros/. One vendor folder for all.
set --
for lock in $(git ls-files 'ros/*Cargo.lock'); do
  if [[ $# == 0 ]]; then set -- --manifest-path "${lock%.lock}.toml"
  else set -- "$@" --sync "${lock%.lock}.toml"; fi
  echo "  ${lock%/Cargo.lock}"
done
crates=$(mktemp -d)
trap 'rm -rf "$crates"' EXIT
cargo vendor --locked --versioned-dirs --quiet "$@" "$crates/vendor" > /dev/null
echo "$(ls "$crates/vendor" | wc -l | tr -d ' ') crates"
tar -czf "$OUT/rust-crates.tar.gz" -C "$crates" vendor

step "Models"
n=0
for m in models/*; do
  case "$m" in
    models/README.md|*.mlpackage) continue ;;   # Core ML is a Mac's; the Orin makes TensorRT
  esac
  [[ -e "$m" ]] || continue
  mkdir -p "$OUT/models"
  cp -R "$m" "$OUT/models/"
  echo "  $m"
  n=$((n + 1))
done
[[ $n -gt 0 ]] || echo "  none in models/ (pixi run get models downloads the stock one)"

step "The map window's offline files (docs/car-screens.md)"
for m in maps/streets/ottawa.pmtiles maps/places/ottawa.db; do
  if [[ -f "$m" ]]; then
    mkdir -p "$OUT/$(dirname "$m")"
    cp "$m" "$OUT/$m"
    echo "  $m"
  else
    echo "  no $m (pixi run get street-map, pixi run get places): the map works without it"
  fi
done

step "The installer, the README, the checksums"
cp tools/car_install.sh "$OUT/car_install.sh"
cp tools/car_setup.sh "$OUT/car_setup.sh"   # the per-machine setup (docs/car-setup.md)
chmod +x "$OUT/car_install.sh" "$OUT/car_setup.sh"
cat > "$OUT/KIT.txt" <<EOF
AAV car kit, made $(date '+%Y-%m-%d %H:%M') on $(uname -n)
commit:     $(git log -1 --format='%h %s')
branch:     $(git rev-parse --abbrev-ref HEAD)
pixi:       $PIXI_VERSION (linux-aarch64, musl)
pixi-pack:  $PIXI_PACK_VERSION
lock file:  pixi.lock $(sha256 pixi.lock | cut -c1-12)
EOF
cat > "$OUT/README.md" <<'EOF'
# The AAV car kit: our ROS 2 Jazzy, offline, for the car's ARM computers

The full steps are in the repo: docs/car-install.md.

## On the car computer (Orin or Motor-Pi), as your normal user

1. Plug in the stick. Check: `uname -m` must say `aarch64`.
2. `bash /media/$USER/<stick>/car-kit/car_install.sh --dry-run`   (checks only)
3. `bash /media/$USER/<stick>/car-kit/car_install.sh`             (20-60 min)
4. `~/aav-jazzy/aav.sh tools/car_listen.sh`                       (read-only)

On the computer that drives the dash's two screens, add `--screens` to step 3: the HUD
and the map then start at login, full screen, one on each (docs/car-screens.md).

| It does | It doesn't |
|---|---|
| fills one folder, ~/aav-jazzy | use sudo |
| runs on ROS_DOMAIN_ID 42 | touch /opt/ros (Humble), ~/.bashrc or services |
| works with no internet | start any driver by itself |

Undo everything: `rm -rf ~/aav-jazzy`

The whole setup of one computer (back up, upgrade, this install, start at boot):
`bash car_setup.sh <role>` prints it; docs/car-setup.md is the runbook.

KIT.txt says which commit this is. SHA256SUMS checks the copy (car_install.sh does it).
EOF
(cd "$OUT" && for f in $(find . -type f ! -name SHA256SUMS | sort); do
   printf '%s  %s\n' "$(sha256 "$f")" "${f#./}"
 done > SHA256SUMS)

step "Done in $(( $(date +%s) - T0 )) s: $ROOT/$OUT"
du -sh "$OUT"/* | sed 's/^/  /'
echo "  total: $(du -sh "$OUT" | awk '{print $1}')"
echo "Copy the folder to a USB stick. Any format: its biggest file is under FAT32's 4 GB."
