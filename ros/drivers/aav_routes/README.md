# aav_routes

Street routes for Nav2 (`docs/routes.md`). `street_router` asks an offline router
(Valhalla on this computer, or an OSRM server) for the way along the streets from the
car's GPS fix to a destination. It cuts the route into GPS waypoints spaced for Nav2,
and publishes them. For real streets later; the test lot first.

```
destination (lat/lon or map point) -> street_router -> router over HTTP (127.0.0.1:8002)
   -> /drivers/nav/route/waypoints, /path, /status
   -> on /drivers/nav/route/follow: Nav2's /follow_gps_waypoints
```

## Overview

A route is only a suggestion. Nav2 still plans and drives each step on its own
costmaps, and twist_mux, ego, the sensor watchdog and the spine stay in charge.
Nothing goes to Nav2 until a person calls the follow service (or `follow_at_once`).

```sh
pixi run get routes      # once: Ottawa's streets, Valhalla's graph
pixi run -e routes router          # the router, in another terminal
ros2 launch aav_routes street_router.launch.py
ros2 topic pub --once /drivers/nav/route/destination geographic_msgs/msg/GeoPointStamped \
  "{position: {latitude: 45.3983, longitude: -75.6838}}"
ros2 service call /drivers/nav/route/follow std_srvs/srv/Trigger
```

## Interface

Node `street_router`: `ros2 launch aav_routes street_router.launch.py [use_sim_time:=false]`.

| Item | What |
|---|---|
| Subscribes | `/localization/gps/fix` (`sensor_msgs/NavSatFix`, `fix_topic`); `/drivers/nav/route/destination` (`geographic_msgs/GeoPointStamped`); `/drivers/nav/route/goal` (`geometry_msgs/PoseStamped`, map frame, turned to lat/lon by `/toLL`) |
| Publishes (latched) | `/drivers/nav/route/waypoints` (`geographic_msgs/GeoPath`); `/drivers/nav/route/path` (`nav_msgs/Path`, map frame); `/drivers/nav/route/status` (`std_msgs/String`: length and time, or why no route) |
| Serves | `/drivers/nav/route/follow` (`std_srvs/Trigger`): send the newest waypoints to Nav2 |
| Calls | the router over HTTP; `/fromLL`, `/toLL` (navsat_transform); action `/follow_gps_waypoints` (`nav2_msgs/action/FollowGPSWaypoints`), only on follow |

Parameters (`config/street_router.yaml`):

| Name | Default | Meaning |
|---|---|---|
| `router` | `valhalla` | or `osrm` |
| `router_url` | `http://127.0.0.1:8002` | this computer only |
| `costing` | `auto` | Valhalla's car profile |
| `timeout` | 5.0 s | wait for the router |
| `tolerance` | 1.0 m | waypoints may cut a bend by this |
| `min_spacing`, `max_spacing` | 10 m, 30 m | between waypoints |
| `max_length` | 3000 m | longer routes are refused |
| `fix_max_age` | 2.0 s | an older fix is no start |
| `map_frame` | `map` | |
| `follow_at_once` | false | true: send every route to Nav2 straight away |
| `fix_topic` | `/localization/gps/fix` | navsat_transform's filtered fix |

Modules:

| Module | Holds |
|---|---|
| `osrm_route.py` | no ROS: `request()`, `parse_response()`, `decode_polyline()`, `waypoints()`, `fill_in()`, `summary()`; `StreetRoute`, `Request`, `RouteError`, `ROUTERS`. Uses `route.py` for the geometry |
| `route.py` | no ROS: `parse_route()`, `simplify()`, `space_out()`, `headings()`, `to_waypoints()`, `remaining()`, `distance()`, `bearing()`, `quaternion()`; `Waypoint`. Moved here from the 2025-26 HUD's back end (`aav_hud`) when that went to `archive/` (25 Oct) |
| `street_router.py` | the ROS node `StreetRouter` |

## Invariants

- `osrm_route.py` and `route.py` have no ROS imports.
- One router answer format: OSRM's JSON (Valhalla asked for `"format": "osrm"`).
  Geometry is GeoJSON or an encoded polyline (precision 6).
- Waypoints: simplified at bends, at least `min_spacing` apart (Nav2 slows at each),
  at most `max_spacing` apart (each goal stays inside Nav2's costmaps), each facing
  along the street; the car's own start is left out.
- Positions are (lat, lon) in degrees; yaw is ENU (0 = east), REP-103.
- One route at a time. Never a paid or public routing server.
- Nothing reaches Nav2 without the follow call, unless `follow_at_once`.
- One thread (`rclpy.spin`): a `MultiThreadedExecutor` spins a whole core (`docs/mac.md`).
  The service calls inside callbacks are awaited, so they never block it; the router's
  HTTP request does, for up to `timeout`.

## Side Effects

- HTTP requests to `router_url` (this computer).
- Publishes the latched route topics; sends a Nav2 goal on follow.
- No files written. The router's map data lives in `maps/routes/` (`pixi run get routes`).

## Verification

```sh
pixi run build && colcon test --packages-select aav_routes && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_routes && python3 -m pytest -q` (`tools/test.sh`
runs it). `pixi run test` runs everything.

| Test | Covers |
|---|---|
| `test_osrm_route.py` | parsing, polylines, waypoint spacing, against saved router answers in `test/fixtures/` |
| `test_street_router.py` | the node on one thread, with a fake router, navsat_transform and Nav2: route out, sent to Nav2 only when asked (needs ROS) |
