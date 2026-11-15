# lanelet-map

## Overview

Lanelet2 maps in Rust, with no ROS: step 6 of the three map levels
(`docs/designs/2026-11-15_three-levels.md`, TODO 175). Ben, 14 Nov: "convert lanelet2 to
rust". Our own reader of Lanelet2's standard `.osm` format; no crate for it exists.
Nothing uses it yet: next, the route level and the behaviour planner.

```text
 map.osm ─▶ from_osm ─▶ lines, lanelets, rules ─▶ the routing graph
 from, to ─▶ route (A*) ─▶ [lanelets] ─▶ centre lines, stops_on (the stop lines met)
```

| Word | Means |
|---|---|
| lanelet | one stretch of one lane: a left and a right line, driven first point to last |
| follows | B starts where A's two lines end |
| beside | they share a line; a lane change only over a dashed (or virtual) one |
| rule | a stop sign, right of way, an all-way stop, a light, and its stop line |

## Interface

| Item | What |
|---|---|
| `LaneletMap::from_osm(text, Origin)` | read a map; points from `local_x`/`local_y`, or lat/lon east and north of the origin (WGS-84) |
| `lanelets()`, `lanelet(id)`, `line(id)`, `rule(id)` | the parts |
| `following(id)`, `beside(id)` | the routing graph |
| `nearest(x, y)` | the lanelet whose centre line passes nearest (an R-tree of the centre lines' segments, `rstar`; of two as near, the smaller id) |
| `route(from, to)` | A* over lanes: length driven, a lane change counted as 10 m; `None` if no way |
| `stops_on(route)` | each stop line met: (lanelet, rule, the line's middle) |
| `MapError` | why a file couldn't be read |
| `lanelet-route <map.osm> x0 y0 x1 y1` (a binary) | route between two map points, timed; the campus map: `tools/campus_lanelet.py` (run by `pixi run campus`) |

Not yet: lanes driven both ways (`one_way=no`), areas, a map writer, Lanelet2's traffic
rules by country.

## Testing

`cargo test` here (`tools/test.sh` runs it, with `cargo fmt` and `cargo clippy`).
