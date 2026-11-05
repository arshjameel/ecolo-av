#!/usr/bin/env bash
# Download what git doesn't keep: source to look things up, models, maps, MediaMTX.
#
# usage: tools/get.sh <what> [args]   (pixi run get <what>)
#
#   vendor      ROS 2 and ros2-rust source into vendor/, to look things up (about 560 MB)
#   models      the stock RF-DETR Nano and YOLO models into models/ (for pixi run detect)
#   mediamtx    MediaMTX (MIT, about 30 MB) into vendor/mediamtx, for pixi run measure stream
#   places      the place database (51 MB) into maps/places/, for the map window's search
#   street-map  Ottawa's offline street map (40 MB) into maps/streets/, for the map window
#   carla-osm   Carleton's OpenStreetMap data into maps/carla/ (--roads: roads only; docs/carla.md)
#   routes      Ottawa's OpenStreetMap and Valhalla's street graph into maps/routes/, in the
#               routes environment (docs/routes.md)
#   all         every one of them, in that order (about 1 GB)
#
# Each script checks what it downloads where it says so, and skips what is already there.
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

ALL="vendor models mediamtx places street-map carla-osm routes"

usage() {
  echo "usage: pixi run get <vendor|models|mediamtx|places|street-map|carla-osm|routes|all> [args]" >&2
  echo "  vendor      ROS 2 and ros2-rust source into vendor/ (about 560 MB)" >&2
  echo "  models      RF-DETR Nano and YOLO into models/" >&2
  echo "  mediamtx    MediaMTX into vendor/mediamtx (the camera Pi's RTSP server)" >&2
  echo "  places      the place database into maps/places/ (the map window's search)" >&2
  echo "  street-map  Ottawa's street map into maps/streets/ (the map window)" >&2
  echo "  carla-osm   Carleton's OpenStreetMap data into maps/carla/ [--roads]" >&2
  echo "  routes      Valhalla's street graph of Ottawa into maps/routes/" >&2
  echo "  all         all of these" >&2
  exit 2
}

get() {
  local what=$1
  shift
  case "$what" in
    vendor)     python3 tools/vendor.py "$@" ;;
    models)     tools/get_models.sh "$@" ;;
    mediamtx)   tools/get_mediamtx.sh "$@" ;;
    places)     tools/get_places.sh "$@" ;;
    street-map) tools/get_street_map.sh "$@" ;;
    carla-osm)  tools/get_carla_osm.sh "$@" ;;
    # Valhalla lives in its own environment, so the default one stays small.
    routes)     pixi run -e routes tools/get_routes.sh "$@" ;;
    *) usage ;;
  esac
}

[[ $# -ge 1 ]] || usage
if [[ "$1" == all ]]; then
  [[ $# -eq 1 ]] || usage
  for what in $ALL; do
    echo "== get $what"
    get "$what"
  done
else
  get "$@"
fi
