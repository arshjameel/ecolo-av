#!/usr/bin/env bash
# Download OpenStreetMap's Ottawa and build the street router's graph from it, into
# maps/routes/. It all stays out of git: a download, made again by this script.
# docs/routes.md says how street_route uses it.
#
# usage: pixi run get routes
#        ROUTES_DIR=/elsewhere pixi run get routes   # another folder
#        ROUTES_PBF=/path/x.osm.pbf pixi run get routes   # your own extract
#
# The data: BBBike's weekly extract of Ottawa (about 50 MB of .osm.pbf), cut from
# OpenStreetMap: © OpenStreetMap contributors, under the ODbL. BBBike keeps only the
# newest week's file, so this checks it against BBBike's own CHECKSUM.txt (a whole,
# unbroken download), not against a fixed sum. Any other .osm.pbf works too (ROUTES_PBF):
# Geofabrik's Ontario (1 GB) for a bigger area.
#
# The router: Valhalla (MIT), from the pyvalhalla wheel in the `routes` environment.
# valhalla_build_config writes valhalla.json; valhalla_build_tiles turns the extract
# into the routing graph (tiles/: about 60 MB, in 5 s on an M-series Mac). No admins
# database: a city extract has no whole country boundary, so it came out empty (16 Oct).
# The service (tools/router.sh) listens on 127.0.0.1 only.
set -euo pipefail
cd "$(dirname "$0")/.."

URL=https://download.bbbike.org/osm/bbbike/Ottawa
dir=${ROUTES_DIR:-maps/routes}
port=${ROUTER_PORT:-8002}
mkdir -p "$dir"
dir=$(cd "$dir" && pwd)

if ! command -v valhalla_build_config >/dev/null; then
  echo "no Valhalla here: run this as pixi run get routes" >&2
  exit 1
fi

md5() {
  if command -v md5sum >/dev/null; then
    md5sum "$1" | cut -d' ' -f1
  else
    md5 -q "$1"
  fi
}

# 1. The extract.
pbf=${ROUTES_PBF:-$dir/download/Ottawa.osm.pbf}
if [[ -z ${ROUTES_PBF:-} && ! -f $pbf ]]; then
  mkdir -p "$dir/download"
  curl -fsSL --retry 3 -o "$dir/download/CHECKSUM.txt" "$URL/CHECKSUM.txt"
  want=$(awk '$2 == "Ottawa.osm.pbf" {print $1}' "$dir/download/CHECKSUM.txt")
  if [[ -z $want ]]; then
    echo "BBBike's CHECKSUM.txt has no Ottawa.osm.pbf: try again later" >&2
    exit 1
  fi
  curl -fL --retry 3 -o "$pbf.part" "$URL/Ottawa.osm.pbf"
  if [[ $(md5 "$pbf.part") != "$want" ]]; then
    rm -f "$pbf.part"
    echo "the download doesn't match BBBike's checksum: not using it" >&2
    exit 1
  fi
  mv "$pbf.part" "$pbf"
fi
if [[ ! -f $pbf ]]; then
  echo "no extract at $pbf" >&2
  exit 1
fi
echo "the extract: $pbf ($(du -h "$pbf" | cut -f1 | tr -d ' '))"

# 2. Valhalla's settings: tiles in $dir/tiles, the service on this computer only.
valhalla_build_config \
  --mjolnir-tile-dir "$dir/tiles" \
  --mjolnir-tile-extract "$dir/tiles.tar" \
  --mjolnir-timezone "$dir/timezones.sqlite" \
  --httpd-service-listen "tcp://127.0.0.1:$port" \
  >"$dir/valhalla.json"

# 3. The graph.
rm -rf "$dir/tiles"
python -m valhalla valhalla_build_tiles -c "$dir/valhalla.json" "$pbf"
echo "the graph: $dir/tiles ($(du -sh "$dir/tiles" | cut -f1 | tr -d ' ')); start it: pixi run -e routes router"
