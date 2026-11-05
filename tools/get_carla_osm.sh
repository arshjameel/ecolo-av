#!/usr/bin/env bash
# Download Carleton's OpenStreetMap data into maps/carla/, for a CARLA map of the campus
# lined up with our datum (docs/carla.md, "A digital twin of Carleton"). It stays out of
# git: a download, made again by this script.
#
# usage: tools/get_carla_osm.sh            (or: pixi run get carla-osm)
#        tools/get_carla_osm.sh --roads    the roads alone: all carla_adapter's osm:= reads
#        CARLA_OSM_AROUND=1500,1500,1500,1500 tools/get_carla_osm.sh   a bigger box (m
#                                          west,south,east,north of the datum)
#
# The box: round navsat.yaml's datum (aav_localization), 900 m west and south, 700 m
# east and north by default: the campus and its streets (aav_carla georef.CARLETON).
# The whole map (roads, buildings, paths) is for CARLA's Digital Twin Tool; --roads is
# enough for carla_adapter osm:=, which builds the roads alone (carla.Osm2Odr).
#
# The data is © OpenStreetMap contributors, under the ODbL: whatever shows it must say
# so. Overpass API (overpass-api.de) is a free public server: one download now and then
# is fine, a loop is not (https://dev.overpass-api.de/overpass-doc/en/preface/commons.html).
set -euo pipefail
cd "$(dirname "$0")/.."

roads=false
if [[ "${1:-}" == "--roads" ]]; then
  roads=true
fi
around=${CARLA_OSM_AROUND:-900,900,700,700}
server=${OVERPASS_SERVER:-https://overpass-api.de/api}
# Overpass refuses curl's own name (HTTP 406, found 15 Oct): say who we are. A busy server
# answers 502 or 504: wait and try again.
fetch=(curl -fsS --retry 2 --retry-delay 10 -A "aav-capstone-get_carla_osm/1.0 (github.com/AAV-2025-2026/ecolo-av)")

# The datum and the box round it, from the same code carla_adapter uses (no ROS needed).
read -r lat lon west south east north < <(PYTHONPATH=ros/sim/aav_carla python3 -m \
  aav_carla.georef ros/localization/aav_localization/config/navsat.yaml --around "$around")
echo "datum $lat, $lon; box west $west, south $south, east $east, north $north"

dir=maps/carla
mkdir -p "$dir"
if [[ $roads == true ]]; then
  file=$dir/carleton-roads.osm
  # Overpass QL: every way tagged highway in the box (south,west,north,east), and its nodes.
  query="[out:xml][timeout:90];(way[\"highway\"]($south,$west,$north,$east);>;);out body;"
  if ! "${fetch[@]}" -o "$file.part" --data-urlencode "data=$query" "$server/interpreter"; then
    echo "Overpass is busy: try again later, or take the whole map (no --roads), which" >&2
    echo "falls back on the OpenStreetMap API; carla_adapter reads its roads just the same" >&2
    exit 1
  fi
else
  file=$dir/carleton.osm
  # Overpass first; when it's too busy (often: 15 Oct, all three public servers), the
  # OpenStreetMap API itself, which serves boxes this small (up to 0.25 square degrees,
  # 50,000 nodes) the way an editor like JOSM downloads them. Once, not in a loop:
  # https://operations.osmfoundation.org/policies/api/
  if ! "${fetch[@]}" -o "$file.part" "$server/map?bbox=$west,$south,$east,$north"; then
    echo "Overpass is busy: asking the OpenStreetMap API instead" >&2
    "${fetch[@]}" -o "$file.part" \
      "https://api.openstreetmap.org/api/0.6/map?bbox=$west,$south,$east,$north"
  fi
fi

# Overpass answers an overload with a short XML remark instead of data: refuse that.
if ! grep -q '<way ' "$file.part"; then
  echo "the download has no roads in it (the server may be busy: try again later):" >&2
  head -c 400 "$file.part" >&2
  rm -f "$file.part"
  exit 1
fi
mv "$file.part" "$file"
echo "$file: $(grep -c '<way ' "$file") ways, $(du -h "$file" | cut -f1 | tr -d ' ')"
echo "next: pixi run carla-car host:=<the CARLA computer> osm:=$file   (CARLA 0.9.16)"
