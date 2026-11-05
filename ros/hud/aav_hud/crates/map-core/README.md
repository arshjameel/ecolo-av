# map-core

## Overview

The map window's logic, with no ROS and no raylib, so it is tested on any laptop. The
`map` program (`map-app`) feeds it Nav2's map, the car's position, the plan and the
goal, and draws what it says: the map, the car, the distance left, an offline street
map of Ottawa, and the place search.

## Interface

| Module | Holds |
|---|---|
| `config` | `MapConfig` from `config/map.yaml` (built in); `Topic`, `NODE_NAME` = `map` |
| `earth` | `LatLon`, UTM, `Datum` (`from_navsat_yaml`, `to_map`, `to_lat_lon`): as navsat_transform does; web Mercator |
| `geom` | points and poses in the map frame (m, rad) |
| `grid` | `Grid`: Nav2's occupancy grid, `to_rgba`; `GridInfo`, `Bounds` |
| `route` | `Route::progress(car)`: the nearest point, the distance left |
| `car` | `CarShape::from_vehicle_yaml`: the outline and arrow |
| `robot` | `CarMarker` (`car_marker`: `outline`, `robot`); `pieces(shape, pose, min_length, travelled)`: the little robot on the car's true outline and heading, from above (2.5D) |
| `view` | map to screen and back: follow the car, the whole map, zoom, the scale bar |
| `trip` | `Trip`: NO GOAL, DRIVING, ARRIVED...; `time_left` |
| `tiles`, `pmtiles`, `streets`, `tilecache` | the street map: which tiles, reading the PMTiles file, water, parks, buildings and roads as triangles, the cache |
| `places` | `Places::open(path)`, `search(query, near)`: names and addresses from `ottawa.db` |
| `destination` | the search box, picking a place, whether it can be Nav2's goal |
| `keyboard` | the on-screen keyboard: its keys, which one a tap hit |
| `state` | everything above, as one value the screen draws |

Example: `cargo run --release -p map-core --example search -- carleton university`.

It builds on [`hud-core`](../hud-core/README.md) (a path dependency): `state` and `trip`
use its `health`, `origin`, `units` and `watchdog`.

## Invariants

- No ROS and no raylib.
- Map frame: metres, radians, x east, y north (ENU), as Nav2's `/map`.
- The datum is navsat_transform's (`aav_localization`'s `navsat.yaml`), so the streets
  sit where the car's map puts them.
- The car's size is `aav_description`'s `vehicle.yaml`, never a copy.
- Built-in defaults (`map.yaml`, `vehicle.yaml`, `navsat.yaml`) are compiled in with
  `include_str!`; unit tests check each one parses.
- A place off Nav2's map, or no car position, can't become a goal.
- The place database is opened read-only.
- The little robot's treads cover exactly the car's outline: its middle is the car's
  middle, its eyes face the car's heading (unit tests).

## Side Effects

- `Places::open` opens the SQLite file read-only. Everything else is pure logic on
  bytes the caller reads.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p map-core
```

`tools/test.sh` runs it (with `hud-core`).
