# Street routes (PLAN S8)

Pick a destination, get the way there **along real streets**, and hand it to Nav2 as GPS
waypoints. Built 16 October. **On the campus** (7 Nov): by a building's name, on the
campus's own road graph, no server: [On the campus](#on-the-campus).

!!! warning "For real streets later: the lot first"
  This is for driving on streets, after the car has proven itself in a lot (PLAN C8, C9).
  A route is only a **suggestion**. Nav2 still plans every step on its own costmaps, and
  twist_mux, the e-stop, ego's limits and timeout, the sensor watchdog and the spine stay
  in charge. Nothing goes to Nav2 until a person asks.

## The picture

![Street routes: destination and GPS fix in, Valhalla asked, waypoints out, Nav2 only on request](img/routes.svg)

| Step | Who | What |
|---|---|---|
| 1 | you | a destination: lat/lon on `/drivers/nav/route/destination`, or a map point on `/drivers/nav/route/goal` |
| 2 | `street_router` | asks the router for the way from the car's newest fix (`/localization/gps/fix`) |
| 3 | Valhalla | answers in OSRM's JSON, offline, from Ottawa's OpenStreetMap |
| 4 | `street_router` | cuts it into waypoints 10-30 m apart; publishes them, a map path, and a status line |
| 5 | you | `/drivers/nav/route/follow`: only now does Nav2 get the route (`/follow_gps_waypoints`) |

## Which router? (the survey)

| Router | Licence | Runs offline on a Mac, Linux, the Jetson? | Graph for Ottawa | Its answer |
|---|---|---|---|---|
| **Valhalla** (picked) | MIT | **yes, all three, no compiling**: the `pyvalhalla` wheel (Mac arm64, Linux x86_64 and aarch64) carries `valhalla_build_tiles` and `valhalla_service`. Not Intel Macs | 59 MB of tiles, built in **5 s** on an M-series Mac | its own JSON, **or OSRM's** (`"format": "osrm"`) |
| OSRM | BSD-2 | Mac: Homebrew. Linux: Docker or build it (C++, Boost). Jetson: build from source | three steps: extract, partition, customize | OSRM's JSON |
| GraphHopper | Apache-2.0 | needs Java 17+ (conda-forge's openjdk is 190 MB) and a 40 MB jar | one import step, at start-up | its own JSON |

**Why Valhalla:** it's the only one that installs through Pixi on every machine we have,
the Jetson too, with nothing to compile. Its graph builds in seconds. It speaks OSRM's
format, so our parser reads OSRM too: switch with `router: osrm` if a team ever runs one.

Checked 16 Oct: the same route from both, Carleton to Lansdowne. Valhalla: **1986.9 m**.
OSRM's public demo server, asked once by hand: **1986.4 m**, the same shape.

## Run it

```bash
pixi run get routes    # once: Ottawa (50 MB, BBBike), then the graph
pixi run -e routes router        # the router, on 127.0.0.1:8002 (keep it running)
pixi run build
tools/ros.sh ros2 launch aav_routes street_router.launch.py
```

Ask for a route, then send it:

```bash
tools/ros.sh ros2 topic pub --once /drivers/nav/route/destination \
  geographic_msgs/msg/GeoPointStamped "{position: {latitude: 45.3983, longitude: -75.6838}}"
tools/ros.sh ros2 topic echo --once /drivers/nav/route/status
tools/ros.sh ros2 service call /drivers/nav/route/follow std_srvs/srv/Trigger
```

| Where the data goes | Size | In git? |
|---|---|---|
| `maps/routes/download/Ottawa.osm.pbf` | 49 MB | no (`/maps/` is ignored) |
| `maps/routes/tiles/` | 59 MB | no |
| `maps/routes/valhalla.json` | 10 kB | no: `get routes` writes it |

`ROUTES_DIR=/elsewhere` puts it all somewhere else; `ROUTES_PBF=file.osm.pbf` uses your
own extract (Geofabrik's Ontario, 1 GB, for a bigger area).

## On the campus

Directions to a building, Tesla-like (Ben, 7 Nov: TODO 140). Say where; the car gets the
way along the campus roads, as waypoints for Nav2.

```
"Fieldhouse"  ──►  the place's middle  ──►  the nearest road the car may use
car's GPS fix ──►  the nearest road    ──►  shortest way on the campus graph ──► waypoints
```

### Run it

```bash
pixi run campus                    # once: the map, the world (about 10 s)
pixi run campus-route Fieldhouse   # no ROS: 453 m from P4 ...
pixi run campus-route --list       # every place it knows
```

In Gazebo (the car starts in lot P4):

```bash
pixi run sim world:=campus reversing:=false gui:=false rviz:=false geofence_radius:=600
tools/ros.sh --local ros2 launch aav_routes street_router.launch.py \
  router:=campus use_sim_time:=true speed:=2.5
tools/ros.sh --local ros2 topic pub --once /drivers/nav/route/place std_msgs/msg/String "{data: Fieldhouse}"
tools/ros.sh --local ros2 service call /drivers/nav/route/follow std_srvs/srv/Trigger
```

`geofence_radius:=600`: the fence is a circle round the start, 200 m by default; most of
the campus is further.

| Say where | How |
|---|---|
| a place's name | `/drivers/nav/route/place`: "Mackenzie Building", "minto", "P4" (any case, part of a name if only one fits) |
| a lat/lon | `/drivers/nav/route/destination`, as on streets |
| a click on a map | `/drivers/nav/route/goal` (a map point), as on streets |

The route shows where the screens already draw `/drivers/nav/route/path` (`nav_msgs/Path`,
map frame, latched): the map window, RViz, Foxglove.

### Why our own small graph, not Valhalla

| | Campus graph (picked) | Valhalla on a campus extract |
|---|---|---|
| Server | none: in `street_router` | `valhalla_service`, tiles built first |
| Its map | `maps/carla/carleton.osm`, the one Gazebo's world and Nav2's map come from | a `.pbf`: a second download or a conversion |
| Roads | exactly the kinds `campus_world.py` draws free on Nav2's map | its car profile avoids parking aisles and private service roads: most of a campus |
| Size | 1,665 corners, 1,749 road pieces; a route in about 2 ms | built for a city |
| Licence | ours | MIT |

So a small Dijkstra in Python, with tests. Valhalla stays for real streets.

### The site file

`ros/drivers/aav_routes/config/campus_site.yaml`: the first piece of TODO 100's site file.

| Part | What |
|---|---|
| `datum` | must be `navsat.yaml`'s (a test checks) |
| `osm` | the map the graph is built from |
| `roads` | road kinds the car may use. Not trunk or primary (Bronson, the Airport Parkway); never footways, paths, cycleways, tunnels, `access=no` |
| `oneway` | one-way roads driven one way (true) |
| `max_snap` | 80 m: a place further from any road has no route |
| `places` | 19 named places: engineering, the middle, sport, lots. Any building or lot OpenStreetMap names works too (76 in all) |

A place's point is the middle of its building. The route ends on the nearest road to it:
up to 56 m away (Tory). Add `stop: [lat, lon]` to a place for the real drop-off point.

### Routes from P4 (no ROS)

| To | Length |
|---|---|
| Fieldhouse | 453 m |
| Richcraft Hall | 592 m |
| P18 | 634 m |
| Minto Centre | 956 m |
| Mackenzie Building | 1,091 m |
| MacOdrum Library | 1,480 m |

Long ways round are the map's: one-way roads, and buildings whose nearest road is a dead
end. Gazebo's map lets Nav2 go either way on a one-way road; the route doesn't.

### What was tested (7-8 Nov, a Mac)

| What | Result |
|---|---|
| `test_campus_graph.py`: a made-up map, 19 tests; every site place reached from P4 on the real map | all pass |
| `test_street_router.py`: a place by name, with a fake car and Nav2 (`colcon test`) | passes |
| Gazebo, `world:=campus`, headless: P4 to **Fieldhouse** (route 453 m, 18 waypoints) | **did not arrive** (below). With our navigator, 8 Nov: 1 of 2 |

The route, the waypoints and the follow call all worked. Nav2 did not finish the drive:

| Run | Drove | Stopped at | Why |
|---|---|---|---|
| `reversing:=false` | 45 m in 50 s | the lot's exit, a sharp right turn | MPPI: "Optimizer fail to compute path", then patience exceeded on every try |
| `reversing:=true` | 310 m in 300 s (sim), through the lot's exit after a 90 s three-point turn | the bend by the roundabout, 143 m short | the car ended on a blocked cell at the road's edge: the planner said "Start occupied" from then on |

**What it shows.** Straight roads are fine: about 1.4 m/s between the bends. The bends are
not. Each waypoint is a Nav2 goal *at* the corner, facing the next road, so the car must
stop on the corner already turned. Our car can't turn that tightly there.

| Next (TODO 140) | Why |
|---|---|
| put a bend's waypoint a few metres *before* the corner, facing along the road in | no stop half-turned at a corner; Nav2 plans the turn itself |
| or send one goal through all the points (Nav2's `navigate_through_poses`) | no stop at each waypoint at all. **Our navigator does this** (`navigator:=own`, a trial): 8 Nov, our simulator, 2 runs: it arrived 1 of 2 (263 s), Nav2 0 of 2 ([scores](own-navigator.md#the-campus-25-ms-2-runs)) |
| `stop:` points for the places, measured on the day | the middle of a building is not where a car pulls up |

NDT was off in these runs (its campus floor is still TODO 121); the EKF used GPS.

## The parts

| Part | File | Does |
|---|---|---|
| The logic, no ROS | `ros/drivers/aav_routes/aav_routes/osrm_route.py` | the request (Valhalla or OSRM), the answer (GeoJSON or an encoded polyline), the waypoints |
| The node | `aav_routes/street_router.py` | the fix, the destination, the router over HTTP, `/fromLL`, `/toLL`, the follow service |
| Its settings | `config/street_router.yaml` | router URL, spacing, `max_length` |
| The campus graph, no ROS | `aav_routes/campus_graph.py` | roads from the `.osm`, places, snapping, Dijkstra |
| The campus's site file | `config/campus_site.yaml` | roads the car may use, named places |
| Directions without ROS | `tools/campus_route.py` | `pixi run campus-route` |
| The map window | `aav_hud` (Rust) | draws `/drivers/nav/route/path` in violet, under Nav2's blue plan |
| The tasks | `tools/get_routes.sh`, `tools/router.sh` | download and build; start the router |

The waypoints reuse the route maths from the 2025-26 HUD's back end, now
`aav_routes/route.py` (no ROS, tested): bends kept, headings along the street. Ours adds the gaps Nav2 needs:

| Setting | Default | Why |
|---|---|---|
| `min_spacing` | 10 m | Nav2 slows at each waypoint: not too many |
| `max_spacing` | 30 m | each goal stays near the car, inside Nav2's costmaps |
| `tolerance` | 1 m | a bend may be cut by this much |
| `max_length` | 3000 m | longer routes are refused: the lot first |
| `fix_max_age` | 2 s | no fresh GPS fix, no route |
| `follow_at_once` | false | true sends every route to Nav2 by itself: not on the car |

## Topics

| Name | Type | |
|---|---|---|
| `/drivers/nav/route/destination` | `geographic_msgs/GeoPointStamped` | in: where to go |
| `/drivers/nav/route/goal` | `geometry_msgs/PoseStamped` (map) | in: where to go, as a map point |
| `/drivers/nav/route/place` | `std_msgs/String` | in: where to go, by name (`router: campus` only) |
| `/drivers/nav/route/waypoints` | `geographic_msgs/GeoPath`, latched | out: the GPS waypoints |
| `/drivers/nav/route/path` | `nav_msgs/Path` (map), latched | out: the route, to draw |
| `/drivers/nav/route/status` | `std_msgs/String`, latched | out: "route: 1.99 km, about 4 min..." or why not |
| `/drivers/nav/route/follow` | `std_srvs/Trigger` (a service) | a person sends the route to Nav2 |

Full contract: [Topics](interfaces.md).

## What was tested (16 October, a Mac)

| What | Result |
|---|---|
| `osrm_route.py` unit tests: 27, on three recorded answers (Valhalla polyline6 and GeoJSON, OSRM's demo) and an error | all pass |
| `street_router` with a fake router, fake `/fromLL` and fake Nav2 (`colcon test`): 4 | all pass; nothing reaches Nav2 before `follow` |
| `get routes` on this Mac | 49 MB download, graph in 5 s |
| A real route, Carleton (45.3876, -75.6960) to Lansdowne | 1.99 km, 4 min, 76 waypoints |
| Gazebo, headless, `empty` world, private domain: a street 80 m south of the datum | the car drove it and stopped **0.5 m** from the route's end, in 140 s |
| `map-core` test: a street route is shown, never treated as the plan | passes |

!!! note "Gazebo, 16 Oct"
  Valhalla snapped the car (at the datum) to the nearest street, 30 m west: its
  line at x = -29.6 m, and routed 80 m south along it: 3 waypoints. The map path came back
  through the sim's `/fromLL` at (-29.6, -0.3) to (-28.9, -80.2). After `follow`, Nav2
  drove to the street, then down it (x stayed within 1 m of -29), and arrived 0.5 m from
  the end. Gazebo's empty world has no streets drawn: the car drove on open ground.

| Not tested yet | Why |
|---|---|
| The map window drawing the route | the window couldn't open from this session (as for the watchdog banner, [HUD](hud.md)); its logic has a test |
| `/drivers/nav/route/goal` with the real `/toLL` | only the lat/lon destination was driven |
| On the car, on the Jetson, on Linux | not yet; the wheel exists for both |
| OSRM run locally | only its demo server's one recorded answer |

## Things to know

- **Valhalla prints errors while building** about countries ("NOT NULL constraint
  failed... Finland"). Harmless: a city extract has no country boundaries.
- **`driving_side: "left"`** in Valhalla's OSRM answers, even in Ottawa. We don't read it.
- The router's time is for **a normal car** on the speed limit, not ours (1 m/s to start).
- Nav2 drives to each waypoint in turn and slows at each: that's why they're 10-30 m apart.
- The data: © OpenStreetMap contributors, ODbL. BBBike's extract changes every week, so
  `get routes` checks it against BBBike's own checksum, not a fixed one.
- **No older code copied.** The 2025-26 team's Navigation-LocalPathGenerator read OSRM
  routes too, and followed them itself ([Older code](older-code.md)). Here Nav2 follows,
  so only a small parser was needed: written new, with tests.
- No API keys, no paid service. A public demo server only for a one-off check by hand,
  never as the default.
