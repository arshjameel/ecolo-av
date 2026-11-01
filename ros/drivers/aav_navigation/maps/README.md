# Maps

A Nav2 map is two files: a greyscale image and a YAML file that says how to read it.
`map_server` loads them and publishes `/map` (`nav_msgs/OccupancyGrid`); the global
costmap's static layer reads `/map`.

| File | What it is |
|---|---|
| `empty.yaml` + `empty.pgm` | Phase 2: 200 m x 200 m, all free, 0.25 m cells, centred on the datum. Nav2 plans straight to a goal within the car's turning limits. |
| `example_lot.yaml` + `.pgm` | Phase 3 example, made up: an 80 m x 60 m lot with a wall, a building, a row of parked cars and an entrance gap. Shows how a pre-rendered costmap is drawn. |
| `example_lot_amcl.yaml` + `.pgm` | The same lot for AMCL only (`amcl:=true`): 0.05 m cells, each obstacle only its outline. Made from the spec's `amcl:` section. |
| `*.spec.yaml` | The short specs `tools/make_map.py` builds the maps from. Edit these, not the PGMs. |

### Why AMCL has a map of its own

AMCL scores each lidar beam by how far its end is from the nearest blocked cell's centre.

| On Nav2's map (0.25 m, solid) | On AMCL's map (0.05 m, outlines) |
|---|---|
| a beam on a wall's face lands in the free cell before it: "0.25 m off" | lands within 0.05 m |
| pushing beams into the solid building scores *better* than the truth | the inside is unknown: no reward for going in |
| best score 0.3 m from the truth (mean), 0.7 m worst | ~0.07 m mean |

Nav2's planner and costmaps keep the solid map: the inside of a building is not a road.
A SLAM map of the real lot is already outlines.

Rebuild after editing a spec:

```bash
tools/make_map.py ros/drivers/aav_navigation/maps/example_lot.spec.yaml
```

Use one: `ros2 launch aav_navigation navigation.launch.py map:=$(ros2 pkg prefix aav_navigation)/share/aav_navigation/maps/example_lot.yaml`

## The YAML

```yaml
image: example_lot.pgm      # next to this file
mode: trinary               # free / occupied / unknown, nothing in between
resolution: 0.25            # metres per pixel
origin: [-10.0, -10.0, 0.0] # map-frame x, y (m) and yaw of the BOTTOM-LEFT pixel
negate: 0                   # 0: white = free, black = occupied
occupied_thresh: 0.65       # darker than this (as a 0..1 occupancy) = occupied
free_thresh: 0.25           # lighter than this = free; between = unknown
```

## The image

PGM (`P5`, 8-bit greyscale), one pixel per cell, row 0 at the top (the north edge).
254 = free (white), 0 = occupied (black), 205 = unknown (grey). PNG works too; any
image editor can paint one.

## Where the map sits on the Earth

The map frame's (0, 0) is the datum in `aav_localization/config/navsat.yaml`, with x
east and y north. So "20 m east and 5 m north of the datum" is map (20, 5), on the
map and in a Nav2 goal. To draw the real test site: pick the datum (a corner or the
start spot), measure the drivable area in metres east/north of it (tape, or satellite
imagery), list the blocked rectangles in a spec, and build it.

A road network (which roads join where) is a different thing: that is the street
router (Valhalla, `aav_routes`). The costmap only says which cells the car may drive on. See
docs/costmaps.md.
