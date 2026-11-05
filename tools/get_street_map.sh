#!/usr/bin/env bash
# Download the street map of Ottawa for the map window into maps/streets/. It stays out
# of git: a download, made again by this script.
#
# usage: tools/get_street_map.sh      (or: pixi run get street-map)
#        STREET_MAP_BUILD=latest tools/get_street_map.sh   # the newest build instead
#
# ottawa.pmtiles: vector tiles (Mapbox Vector Tile format, in a PMTiles file) of the
# Ottawa area, zoom 0 to 15, cut from Protomaps' daily world basemap (build.protomaps.com).
# The data is © OpenStreetMap contributors, under the ODbL: whatever shows it must say so
# (the map window does). Protomaps' basemap code is BSD-3-Clause.
#
# How: go-pmtiles' `pmtiles extract` reads only the tiles inside the box from the world
# file over HTTP (about a hundred range requests), so the 139 GB world file is never
# downloaded. conda-forge's `pmtiles` (Python) can't extract, so the Go binary is
# downloaded once into maps/tools/, checked against the release's SHA-256.
#
# The build: Protomaps keeps every daily build for a week, and the last build of each
# version for good. BUILD is the last build of version 4.15.1, so it stays, and the same
# extract comes out each time (SUM). STREET_MAP_BUILD=latest takes the newest build:
# newer streets, but no checksum to compare with.
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD=20260811            # Protomaps basemap v4.15.1: OpenStreetMap as of 11 Aug 2026
SUM=29b1d6a96a339fc77e922b30b2f98cde12bd94a10ba4ddbb0dbb0088f0db1cdf   # sha256 of the extract below
BBOX=-76.0,45.2,-75.4,45.55   # west,south,east,north: Ottawa, Carleton in the middle
MAXZOOM=15
PMTILES_VERSION=1.31.2

dir=maps/streets
tools=maps/tools
file=$dir/ottawa.pmtiles
mkdir -p "$dir" "$tools"

sha256() {
  if command -v sha256sum >/dev/null; then
    sha256sum "$1" | cut -d' ' -f1
  else
    shasum -a 256 "$1" | cut -d' ' -f1
  fi
}

# go-pmtiles: the release asset for this computer, and its SHA-256 (GitHub's digest).
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64) asset=go-pmtiles-${PMTILES_VERSION}_Darwin_arm64.zip
    asset_sum=40528f7f616fcbf91207cd48c8fc023d213f6d86c0cbf1f748732803d1880f3d ;;
  Darwin-x86_64) asset=go-pmtiles-${PMTILES_VERSION}_Darwin_x86_64.zip
    asset_sum=1f0dc02eee6c58312dd6c509faee1b5c32f0596568af1bf51f1b034e7a88a65b ;;
  Linux-aarch64 | Linux-arm64) asset=go-pmtiles_${PMTILES_VERSION}_Linux_arm64.tar.gz
    asset_sum=f8bd47e7ea866863489cad588fbaf2f31f42e5821f7a03f009b3769f05801cb1 ;;
  Linux-x86_64) asset=go-pmtiles_${PMTILES_VERSION}_Linux_x86_64.tar.gz
    asset_sum=3ed7dbf4ec2e6dfe5e25b6f70d1ffc932729f93c86db353bf514dd71010a312f ;;
  *) echo "no go-pmtiles download for $(uname -s) $(uname -m)" >&2; exit 1 ;;
esac
pmtiles=$tools/pmtiles-$PMTILES_VERSION
if [[ ! -x $pmtiles ]]; then
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  curl -fsSL --retry 3 -o "$tmp/$asset" \
    "https://github.com/protomaps/go-pmtiles/releases/download/v$PMTILES_VERSION/$asset"
  if [[ $(sha256 "$tmp/$asset") != "$asset_sum" ]]; then
    echo "$asset isn't the expected download: not using it" >&2
    exit 1
  fi
  mkdir "$tmp/x"
  case $asset in
    *.zip) unzip -q "$tmp/$asset" pmtiles -d "$tmp/x" ;;
    *) tar -xzf "$tmp/$asset" -C "$tmp/x" pmtiles ;;
  esac
  mv "$tmp/x/pmtiles" "$pmtiles"
  chmod +x "$pmtiles"
fi

build=$BUILD
if [[ ${STREET_MAP_BUILD:-} == latest ]]; then
  build=$(curl -fsSL https://build-metadata.protomaps.dev/builds.json |
    grep -o '"key":"[0-9]*\.pmtiles"' | tail -1 | grep -o '[0-9]\{8\}')
  echo "the newest build: $build"
fi

if [[ ! -f $file ]]; then
  # Three tries: Protomaps' server sometimes drops a long transfer part way.
  for try in 1 2 3; do
    rm -f "$file.part"
    if "$pmtiles" extract "https://build.protomaps.com/$build.pmtiles" "$file.part" \
      --bbox="$BBOX" --maxzoom="$MAXZOOM"; then
      break
    fi
    if [[ $try == 3 ]]; then
      echo "the extract failed three times: try again later" >&2
      exit 1
    fi
    echo "the extract failed; trying again" >&2
  done
  mv "$file.part" "$file"
  echo "$build" >"$dir/ottawa.build"
fi

got=$(sha256 "$file")
have=$(cat "$dir/ottawa.build" 2>/dev/null || echo unknown)
if [[ $have == "$BUILD" && $got != "$SUM" ]]; then
  echo "$file isn't the expected extract of build $BUILD: delete it and run this again" >&2
  exit 1
fi
echo "$file: ok (Protomaps build $have, $(du -h "$file" | cut -f1 | tr -d ' '))"
