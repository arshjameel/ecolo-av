#!/usr/bin/env python3
"""
Directions on the campus, without ROS: the route from one place to another (docs/routes.md).

    pixi run campus-route "Mackenzie Building"               from P4, where the sim starts
    pixi run campus-route --from "Dunton Tower" Fieldhouse
    pixi run campus-route --from 45.3853,-75.6954 P18 --waypoints
    pixi run campus-route --list                             every place it knows

The same graph, places and waypoints street_router uses with `router: campus`. Reads the
site file (ros/drivers/aav_routes/config/campus_site.yaml) and its .osm (pixi run campus
downloads it). Map data (c) OpenStreetMap contributors, ODbL.
"""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros' / 'drivers' / 'aav_routes'))
from aav_routes import campus_graph, osrm_route  # noqa: E402, I100

SITE = ROOT / 'ros' / 'drivers' / 'aav_routes' / 'config' / 'campus_site.yaml'


def where(campus, text):
    """Return (lat, lon) for 'lat,lon' or a place's name."""
    try:
        lat, lon = (float(v) for v in text.split(','))
        return lat, lon
    except ValueError:
        return campus_graph.target(campus.find(text))


def main():
    parser = argparse.ArgumentParser(description='Directions on the campus, without ROS.')
    parser.add_argument('to', nargs='?', help="a place's name, or lat,lon")
    parser.add_argument('--from', dest='start', default='P4',
                        help="where from: a place's name, or lat,lon (default P4)")
    parser.add_argument('--site', type=Path, default=SITE, help=f'default {SITE}')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='m/s, for the time (default 1.0: the real car)')
    parser.add_argument('--waypoints', action='store_true', help='print the GPS waypoints')
    parser.add_argument('--list', action='store_true', help='every place it knows')
    args = parser.parse_args()
    try:
        campus = campus_graph.Campus(args.site, ROOT)
        if args.list:
            for place in sorted(campus.places.values(), key=lambda p: (p.kind, p.name)):
                print(f'{place.kind:9} {place.name}')
            return 0
        if not args.to:
            parser.error('where to? (or --list)')
        route = campus.route(where(campus, args.start), where(campus, args.to), args.speed)
    except (osrm_route.RouteError, ValueError, OSError) as e:
        print(f'no route: {e}', file=sys.stderr)
        return 1
    points = osrm_route.waypoints(route.points)
    print(f'{route.length:.0f} m, about {route.duration / 60:.1f} min at {args.speed} m/s, '
          f'{len(points)} waypoints')
    if args.waypoints:
        for w in points:
            print(f'{w.lat:.7f},{w.lon:.7f}  yaw {w.yaw:+.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
