#!/usr/bin/env bash
# Download one pinned MediaMTX into vendor/mediamtx/, for the camera stream's test on a
# laptop (pixi run measure stream, docs/camera-pipeline.md). Like the rest of vendor/, it's
# other people's code, and not in git.
#
# usage: tools/get_mediamtx.sh      (or: pixi run get mediamtx)
#
# MediaMTX (github.com/bluenviron/mediamtx, MIT) is the RTSP server the camera Pi runs.
# conda-forge has it only for Intel Macs and Linux x64, not Apple-silicon Macs or ARM
# Linux, so it can't come from Pixi on every machine: its own release instead. No admin
# rights needed. The archive is checked against the SHA-256 that the release publishes
# (checksums.sha256), copied here.
set -euo pipefail
cd "$(dirname "$0")/.."

version=v1.21.2
base=https://github.com/bluenviron/mediamtx/releases/download/$version
dest=vendor/mediamtx

case "$(uname -s)/$(uname -m)" in
  Darwin/*)
    # An Apple-silicon Mac says x86_64 under Rosetta: ask the hardware instead.
    if [[ "$(sysctl -n hw.optional.arm64 2>/dev/null || echo 0)" == 1 ]]; then
      arch=darwin_arm64
      sum=7d0d0c5c11527cbd10bde1c3d728864eb0334a9b653566cd73c0259750cbf093
    else
      arch=darwin_amd64
      sum=87a00062b6a588aa1158f875504a3b56c80d62e2611dc450b83d0534e1b49198
    fi
    ;;
  Linux/x86_64)
    arch=linux_amd64
    sum=121be6e00397e473a95d83c75f4fcd3819a6af3b0e1a50889cba43ab4fab6a27
    ;;
  Linux/aarch64)
    arch=linux_arm64
    sum=180a235374a548d9133cdd5e20f5dbd8b01317be1c86e0d7e66df585dc42ed6f
    ;;
  *)
    echo "MediaMTX $version has no release here for $(uname -s) $(uname -m)." >&2
    exit 1
    ;;
esac
archive=mediamtx_${version}_$arch.tar.gz

if [[ -x $dest/mediamtx ]]; then
  got=$("$dest/mediamtx" --version 2>/dev/null || echo none)
  if [[ $got == "$version" ]]; then
    echo "$dest: MediaMTX $version is already there"
    exit 0
  fi
  echo "$dest holds MediaMTX $got, not $version: replacing it"
fi

# A new, empty folder for the download, next to where it ends up (the same disk).
mkdir -p vendor
tmp=$(mktemp -d vendor/.get-mediamtx.XXXXXX)
trap 'rm -rf "$tmp"' EXIT
file=$tmp/$archive
echo "Downloading MediaMTX $version: $archive (about 30 MB)"
curl -fL --retry 3 -o "$file" "$base/$archive"
if command -v sha256sum >/dev/null; then
  got=$(sha256sum "$file" | cut -d' ' -f1)
else
  got=$(shasum -a 256 "$file" | cut -d' ' -f1)
fi
if [[ $got != "$sum" ]]; then
  echo "$archive isn't the file MediaMTX published (SHA-256 $got): not unpacked" >&2
  exit 1
fi
echo "Checked (SHA-256). Unpacking..."
mkdir "$tmp/unpacked"
tar -xzf "$file" -C "$tmp/unpacked" mediamtx LICENSE
rm -rf "$dest"
mv "$tmp/unpacked" "$dest"
echo "$dest/mediamtx: MediaMTX $("$dest/mediamtx" --version), ok"
