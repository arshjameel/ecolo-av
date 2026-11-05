#!/usr/bin/env bash
# Start the street router: Valhalla's service, on 127.0.0.1:8002 (docs/routes.md).
# street_route asks it for routes. Stop it with Ctrl-C.
#
# usage: pixi run -e routes router          (pixi run get routes first)
#        ROUTES_DIR=/elsewhere pixi run -e routes router
#
# It listens on this computer only (valhalla.json, written by get_routes.sh). It answers
# Valhalla's own /route, and OSRM's JSON when asked ("format": "osrm"), which is what
# street_route asks for.
set -euo pipefail
cd "$(dirname "$0")/.."

dir=${ROUTES_DIR:-maps/routes}
threads=${ROUTER_THREADS:-2}
if [[ ! -f $dir/valhalla.json || ! -d $dir/tiles ]]; then
  echo "no street graph in $dir: run pixi run get routes first" >&2
  exit 1
fi
echo "the street router: $(grep -o 'tcp://[^"]*' "$dir/valhalla.json" | head -1)"
exec python -m valhalla valhalla_service "$dir/valhalla.json" "$threads"
