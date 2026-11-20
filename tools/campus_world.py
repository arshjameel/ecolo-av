#!/usr/bin/env python3
"""
Carleton's campus for Ravensim, our costmap and NDT, from OpenStreetMap.

    pixi run campus              download the map if it's missing, then build maps/campus/
    pixi run campus --osm FILE   build from another .osm (tools/get_carla_osm.sh makes one)
    pixi run sim world:=campus   drive on it

It makes, in maps/campus/ (not in git; this tool makes it again):

    campus.sdf            the world: pavement, grass, kerbs, buildings, trees
    meshes/, textures/    what the world draws, with bump maps
    campus.pgm, .yaml     one Nav2 map of the whole campus: only roads and lots are free
    place.yaml            where the car starts, what went in, the OpenStreetMap credit
    lanes.yaml            the painted roads' centre lines and widths, to score lane keeping;
                          also the lane map (aav_description lane_map.py)
    objects.yaml          its buildings, fences and trees: the truth label-drive scores by

and maps/ndt/campus.pcd: NDT's map, **a guess** sampled from the same shapes. A real drive
replaces it (pixi run ndt-map build); a drive suggests changes and a person accepts them.

What's measured and what's guessed:

    | From OpenStreetMap            | Guessed here                               |
    |-------------------------------|--------------------------------------------|
    | where everything is, its      | building heights without a tag, road       |
    | shape, most road widths       | widths without a tag, where kerbs run,     |
    |                               | tree sizes, trees inside woods, all looks  |

The campus's shape (Ben, 5 Nov): everything that isn't pavement is a foot up, held in by a
concrete kerb, with grass on top. So a kerb runs along every edge of every road and lot.

The data is (c) OpenStreetMap contributors, under the ODbL: whatever shows it must say so.
No ROS here: tools/tests/test_campus_world.py tests the maths.
"""

import argparse
from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
from xml.etree import ElementTree

import cv2
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros' / 'sim' / 'aav_carla'))
sys.path.insert(0, str(ROOT / 'tools'))
from aav_carla.frames import geodetic_to_enu  # noqa: E402, I100
from aav_carla.georef import CARLETON  # noqa: E402
import campus_lanelet  # noqa: E402, I100
from ndt_map import write_pcd  # noqa: E402, I100

NAVSAT = ROOT / 'ros' / 'localization' / 'aav_localization' / 'config' / 'navsat.yaml'
OSM = ROOT / 'maps' / 'carla' / 'carleton.osm'
OUT = ROOT / 'maps' / 'campus'
NDT_OUT = ROOT / 'maps' / 'ndt' / 'campus'

CELL = 0.25            # m: one cell of Nav2's map, the same as the test lot's
KERB_HEIGHT = 0.30     # m: "foot tall" (Ben, 5 Nov)
KERB_TOP = 0.30        # m wide on top: it also hides the grass's 0.25 m steps along its edge
NDT_SPACING = 0.33     # m between the guessed map's points: 9 to a square metre of wall
NDT_FLOOR = 0.45       # m: above the grass (aav_sim_models worlds.CAMPUS_NDT_FLOOR)
NDT_CEILING = 16.0     # m: the lidar's top beam (15 degrees) reaches about this at its 45 m
START_LOT = 'P4'       # the car's first home: a lot beside the datum
SIMPLIFY_OVER = 400    # corners: rings longer than this are simplified to SIMPLIFY_TO metres
SIMPLIFY_TO = 1.0
WOOD_TREE_SPACING = 9.0  # m between the trees scattered in a wood
MAX_WOOD_TREES = 5000
ROW_TREE_SPACING = 8.0   # m along a tree row
FLOOR_HEIGHT = 3.6     # m
DEFAULT_FLOORS = 3     # a building with no height in OpenStreetMap
# Floors we know that OpenStreetMap doesn't say: Carleton's tallest building.
KNOWN_FLOORS = {'Dunton Tower': 22}

# The heights of the layers, m. The car drives on z = 0; the rest of the campus is a kerb up.
BASE_Z = -0.08                       # the foot of every building
UNDER_Z = -0.05                      # the pavement under everything
LOT_Z, ROAD_Z, PAINT_Z = -0.03, -0.01, -0.005
GRASS_Z, PATH_Z, KERB_TOP_Z = KERB_HEIGHT, KERB_HEIGHT + 0.004, KERB_HEIGHT + 0.006
# What the raised ground is, cell by cell.
PAVEMENT, GRASS, GREEN_AREA, WATER_AREA = 0, 1, 2, 3

# Width of a road with no width tag, m.
ROAD_WIDTHS = {'trunk': 9.0, 'trunk_link': 6.0, 'primary': 9.0, 'primary_link': 6.0,
               'secondary': 8.0, 'secondary_link': 6.0, 'tertiary': 7.0, 'unclassified': 6.0,
               'residential': 6.0, 'service': 5.0}
LANE_WIDTH = 3.5       # m
# Roads with paint: a yellow dashed centre line, and a solid white line along each edge,
# EDGE_INSET in from it, so the lane finder sees both sides of the car's lane.
PAINTED = {'trunk', 'primary', 'secondary', 'tertiary'}
LINE_WIDTH = 0.15      # m: every painted line
EDGE_INSET = 0.5       # m: from the road's edge to its edge line's centre: tarmac each side
PATHS = {'footway', 'path', 'cycleway', 'pedestrian', 'steps'}
PATH_WIDTH = 2.0       # m
GREEN = {'grass', 'meadow', 'park', 'garden', 'pitch', 'playground', 'allotments', 'farmland'}
WOOD = {'forest', 'wood'}
BARRIERS = {'fence': 1.5, 'wall': 2.0, 'retaining_wall': 1.0, 'hedge': 1.5}


@dataclass
class Area:
    """A filled shape: its tags, its outer ring and its holes, in metres."""

    tags: dict
    ring: list
    holes: list = field(default_factory=list)
    key: str = ''


@dataclass
class Line:
    """A line on the map: its tags and its points, in metres."""

    tags: dict
    points: list


@dataclass
class Campus:
    """What the map holds, sorted by what the world does with it."""

    areas: list = field(default_factory=list)
    lines: list = field(default_factory=list)
    trees: list = field(default_factory=list)


def join_rings(ways):
    """Join a relation's ways end to end into closed rings; an open one is dropped."""
    rings, left = [], [list(w) for w in ways if len(w) >= 2]
    while left:
        ring = left.pop()
        grew = True
        while grew and ring[0] != ring[-1]:
            grew = False
            for i, way in enumerate(left):
                if way[0] == ring[-1]:
                    ring += way[1:]
                elif way[-1] == ring[-1]:
                    ring += way[-2::-1]
                elif way[-1] == ring[0]:
                    ring = way[:-1] + ring
                elif way[0] == ring[0]:
                    ring = way[:0:-1] + ring
                else:
                    continue
                left.pop(i)
                grew = True
                break
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring)
    return rings


def read_osm(path, project):
    """Read an .osm file into Campus; project(lat, lon) gives metres east and north."""
    root = ElementTree.parse(path).getroot()
    campus, nodes, ways = Campus(), {}, {}

    def tags_of(element):
        return {t.get('k'): t.get('v') for t in element.iter('tag')}

    for node in root.iter('node'):
        at = project(float(node.get('lat')), float(node.get('lon')))
        nodes[node.get('id')] = at
        if tags_of(node).get('natural') == 'tree':
            campus.trees.append(at)
    for way in root.iter('way'):
        points = [nodes[nd.get('ref')] for nd in way.iter('nd') if nd.get('ref') in nodes]
        ways[way.get('id')] = points
        tags = tags_of(way)
        if len(points) >= 4 and points[0] == points[-1] and not is_line(tags):
            campus.areas.append(Area(tags, points, key=f'w{way.get("id")}'))
        elif len(points) >= 2:
            campus.lines.append(Line(tags, points))
    for relation in root.iter('relation'):
        tags = tags_of(relation)
        if tags.get('type') != 'multipolygon':
            continue
        members = list(relation.iter('member'))
        part = [m for m in members if m.get('type') == 'way']
        outers = join_rings(ways.get(m.get('ref'), []) for m in part if m.get('role') == 'outer')
        inners = join_rings(ways.get(m.get('ref'), []) for m in part if m.get('role') == 'inner')
        for number, ring in enumerate(outers):
            campus.areas.append(Area(tags, ring, inners if number == 0 else [],
                                     key=f'r{relation.get("id")}.{number}'))
    return campus


def is_line(tags):
    """Return True for a closed way that's a loop of road, kerb or fence, not a filled shape."""
    if tags.get('area') == 'yes':
        return False
    return 'highway' in tags or 'barrier' in tags or 'railway' in tags


def below_ground(tags):
    """Return True for what OpenStreetMap puts under the ground or indoors: Carleton's tunnels."""
    try:
        layer = int(tags.get('layer', '0'))
    except ValueError:
        layer = 0
    return (layer < 0 or tags.get('tunnel') not in (None, 'no') or tags.get('indoor') == 'yes'
            or tags.get('location') == 'underground' or tags.get('highway') == 'corridor')


def signed_area(ring):
    """Return a ring's area, m^2: positive when it goes anticlockwise."""
    points = open_ring(ring)
    return sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(points, points[1:] + points[:1])) / 2.0


def open_ring(ring):
    """Return a ring without its repeated last point."""
    return list(ring[:-1]) if len(ring) > 1 and ring[0] == ring[-1] else list(ring)


def centroid(ring):
    """Return the mean of a ring's corners: good enough to say which box or lot it's in."""
    points = open_ring(ring)
    return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))


def simplify(ring):
    """Return a very long ring (a river's bank) with far fewer corners."""
    if len(ring) <= SIMPLIFY_OVER:
        return ring
    points = np.array(open_ring(ring), dtype=np.float32).reshape(-1, 1, 2)
    fewer = cv2.approxPolyDP(points, SIMPLIFY_TO, True).reshape(-1, 2)
    return [tuple(map(float, p)) for p in fewer] + [tuple(map(float, fewer[0]))]


def triangulate(ring):
    """
    Cut a simple polygon into triangles, by clipping ears: a list of index triples.

    The indices are into open_ring(ring). A polygon that crosses itself (rare, in hand-drawn
    maps) gets the triangles found before the clipping stuck; the rest of it isn't drawn.
    """
    points = open_ring(ring)
    if len(points) < 3:
        return []
    order = list(range(len(points)))
    if signed_area(ring) < 0.0:
        order.reverse()

    def cross(a, b, c):
        return ((points[b][0] - points[a][0]) * (points[c][1] - points[a][1])
                - (points[b][1] - points[a][1]) * (points[c][0] - points[a][0]))

    triangles = []
    while len(order) > 3:
        for i, b in enumerate(order):
            a, c = order[i - 1], order[(i + 1) % len(order)]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(p not in (a, b, c) and cross(a, b, p) >= 0.0 and cross(b, c, p) >= 0.0
                   and cross(c, a, p) >= 0.0 for p in order):
                continue
            triangles.append((a, b, c))
            order.pop(i)
            break
        else:
            return triangles
    triangles.append(tuple(order))
    return triangles


class Mesh:
    """Triangles for one material. Each face has its own corners, so its edges stay sharp."""

    def __init__(self):
        self.vertices, self.uvs, self.normals = [], [], []

    def triangle(self, a, b, c, uv_a, uv_b, uv_c):
        """Add one triangle, its corners anticlockwise from outside."""
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        size = math.sqrt(nx * nx + ny * ny + nz * nz)
        if size < 1e-12:
            return
        normal = (nx / size, ny / size, nz / size)
        self.vertices += [a, b, c]
        self.uvs += [uv_a, uv_b, uv_c]
        self.normals += [normal, normal, normal]

    def flat(self, ring, z, tile=1.0):
        """Fill a polygon at height z, facing up; the texture repeats every `tile` metres."""
        points = open_ring(ring)
        for tri in triangulate(ring):
            a, b, c = (points[i] for i in tri)
            self.triangle((a[0], a[1], z), (b[0], b[1], z), (c[0], c[1], z),
                          (a[0] / tile, a[1] / tile), (b[0] / tile, b[1] / tile),
                          (c[0] / tile, c[1] / tile))

    def wall(self, a, b, z0, z1, along=0.0, tile=(1.0, 1.0)):
        """Add one upright rectangle from a to b, facing to the right of a -> b."""
        s0, s1 = along / tile[0], (along + math.dist(a, b)) / tile[0]
        v0, v1 = z0 / tile[1], z1 / tile[1]
        lo_a, lo_b = (a[0], a[1], z0), (b[0], b[1], z0)
        hi_a, hi_b = (a[0], a[1], z1), (b[0], b[1], z1)
        self.triangle(lo_a, lo_b, hi_b, (s0, v0), (s1, v0), (s1, v1))
        self.triangle(lo_a, hi_b, hi_a, (s0, v0), (s1, v1), (s0, v1))

    def prism_walls(self, ring, z0, z1, tile=(1.0, 1.0)):
        """Add the outside walls of a ring lifted from z0 to z1."""
        points = open_ring(ring)
        if signed_area(ring) < 0.0:
            points.reverse()
        along = 0.0
        for a, b in zip(points, points[1:] + points[:1]):
            self.wall(a, b, z0, z1, along, tile)
            along += math.dist(a, b)

    def write(self, path):
        """Write a Wavefront OBJ."""
        with open(path, 'w') as out:
            out.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.vertices)
            out.writelines(f'vt {u:.4f} {v:.4f}\n' for u, v in self.uvs)
            out.writelines(f'vn {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.normals)
            for i in range(1, len(self.vertices), 3):
                out.write(f'f {i}/{i}/{i} {i + 1}/{i + 1}/{i + 1} {i + 2}/{i + 2}/{i + 2}\n')


def strip_quads(points, width, corners=True):
    """Return a line widened into rectangles; corners adds a many-sided disc at each bend."""
    half = width / 2.0
    shapes = []
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length < 1e-6:
            continue
        nx, ny = -(b[1] - a[1]) / length * half, (b[0] - a[0]) / length * half
        shapes.append([(a[0] - nx, a[1] - ny), (b[0] - nx, b[1] - ny),
                       (b[0] + nx, b[1] + ny), (a[0] + nx, a[1] + ny)])
    for p in points[1:-1] if corners else []:
        shapes.append([(p[0] + half * math.cos(t * math.pi / 6.0),
                        p[1] + half * math.sin(t * math.pi / 6.0)) for t in range(12)])
    return shapes


def straighten(points, tolerance=0.02):
    """Return a line without the points that lie on the way between their neighbours."""
    if len(points) < 3:
        return list(points)
    fewer = cv2.approxPolyDP(np.array(points, dtype=np.float32).reshape(-1, 1, 2),
                             tolerance, False).reshape(-1, 2)
    return [tuple(map(float, p)) for p in fewer]


def offset_line(points, d):
    """
    Return a line moved d metres to its left (right if d < 0), corner by corner.

    Each corner moves along the mean of its two segments' left normals, scaled so both
    segments stay d away (a mitre), at most 3 d at a sharp bend.
    """
    pts = [p for i, p in enumerate(points) if i == 0 or math.dist(p, points[i - 1]) > 1e-6]
    if len(pts) < 2:
        return list(pts)

    def normal(a, b):
        length = math.dist(a, b)
        return (-(b[1] - a[1]) / length, (b[0] - a[0]) / length)

    normals = [normal(a, b) for a, b in zip(pts, pts[1:])]
    out = []
    for i, p in enumerate(pts):
        before, after = normals[max(i - 1, 0)], normals[min(i, len(normals) - 1)]
        mx, my = before[0] + after[0], before[1] + after[1]
        length = math.hypot(mx, my)
        if length < 1e-6:          # a turn right back: keep the segment's own normal
            mx, my, length = before[0], before[1], 1.0
        mx, my = mx / length, my / length
        scale = min(1.0 / max(mx * after[0] + my * after[1], 1e-6), 3.0)
        out.append((p[0] + mx * d * scale, p[1] + my * d * scale))
    return out


def edge_lines(points, width):
    """Return a painted road's two solid edge lines, as rectangles, EDGE_INSET in."""
    shift = width / 2.0 - EDGE_INSET
    return [shape for side in (1.0, -1.0)
            for shape in strip_quads(offset_line(points, side * shift), LINE_WIDTH)]


def dashes(points, width, on=3.0, off=6.0):
    """Return a road's centre line, as the rectangles of its dashes."""
    shapes, gone = [], 0.0
    period = on + off
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length < 1e-6:
            continue
        ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        start = (period - gone % period) % period
        while start < length:
            end = min(start + on, length)
            shapes += strip_quads([(a[0] + ux * start, a[1] + uy * start),
                                   (a[0] + ux * end, a[1] + uy * end)], width)
            start += period
        gone += length
    return shapes


class Grid:
    """Nav2's map being drawn: a picture of the box, with y going up."""

    def __init__(self, box, cell=CELL):
        self.west, self.south, east, north = box
        self.cell = cell
        self.width = int(round((east - self.west) / cell))
        self.height = int(round((north - self.south) / cell))
        self.free = np.zeros((self.height, self.width), dtype=np.uint8)

    def pixels(self, points):
        """Return points as a picture's coordinates, in OpenCV's shape."""
        at = np.asarray(points, dtype=np.float64)
        xs = (at[:, 0] - self.west) / self.cell
        ys = self.height - 1 - (at[:, 1] - self.south) / self.cell
        return np.round(np.stack([xs, ys], axis=1)).astype(np.int32).reshape(-1, 1, 2)

    def fill(self, ring, value):
        """Set every cell in a polygon: 1 free, 0 blocked."""
        cv2.fillPoly(self.free, [self.pixels(open_ring(ring))], int(value))

    def line(self, points, value, width=2):
        """Set every cell on a line."""
        cv2.polylines(self.free, [self.pixels(points)], False, int(value), width)

    def is_free(self, x, y):
        """Return True if a point is on the map and drivable."""
        col = int(round((x - self.west) / self.cell))
        row = int(round(self.height - 1 - (y - self.south) / self.cell))
        return (0 <= col < self.width and 0 <= row < self.height
                and bool(self.free[row, col]))

    def nearest_free(self, x, y, reach=60.0):
        """Return the nearest drivable point with room round it, or None."""
        wide = cv2.erode(self.free, np.ones((13, 13), np.uint8))
        rows, cols = np.nonzero(wide)
        if len(rows) == 0:
            return None
        xs = cols * self.cell + self.west
        ys = (self.height - 1 - rows) * self.cell + self.south
        distance = np.hypot(xs - x, ys - y)
        best = int(np.argmin(distance))
        return (float(xs[best]), float(ys[best])) if distance[best] <= reach else None

    def save(self, folder, name, datum):
        """Write the map's PGM and its YAML."""
        picture = np.where(self.free > 0, 254, 0).astype(np.uint8)
        with open(folder / f'{name}.pgm', 'wb') as out:
            out.write(f'P5\n{self.width} {self.height}\n255\n'.encode())
            out.write(picture.tobytes())
        (folder / f'{name}.yaml').write_text(
            f'# Carleton, {self.width} x {self.height} cells. tools/campus_world.py made it from '
            f'OpenStreetMap,\n# (c) OpenStreetMap contributors, ODbL. Datum: {datum}.\n'
            f'image: {name}.pgm\nmode: trinary\nresolution: {self.cell}\n'
            f'origin: [{self.west}, {self.south}, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def rectangles(mask):
    """
    Return a mask's True cells as few rectangles: (first col, end col, first row, end row).

    Each row is cut into runs, and a run the same as the row above's grows that rectangle.
    """
    done, open_runs = [], {}
    padded = np.zeros((mask.shape[0], mask.shape[1] + 2), dtype=np.int8)
    padded[:, 1:-1] = mask
    steps = np.diff(padded, axis=1)
    for row in range(mask.shape[0]):
        starts = np.flatnonzero(steps[row] == 1)
        ends = np.flatnonzero(steps[row] == -1)
        runs = set(zip(starts.tolist(), ends.tolist()))
        for run in set(open_runs) - runs:
            done.append((run[0], run[1], open_runs.pop(run), row))
        for run in runs - set(open_runs):
            open_runs[run] = row
    done.extend((run[0], run[1], first, mask.shape[0]) for run, first in open_runs.items())
    return done


def raised_ground(mesh, surface, value, grid, z, tile):
    """Add the cells of one kind of raised ground to a mesh, as flat rectangles."""
    for col0, col1, row0, row1 in rectangles(surface == value):
        west, east = grid.west + col0 * grid.cell, grid.west + col1 * grid.cell
        top = grid.south + (grid.height - row0) * grid.cell
        bottom = grid.south + (grid.height - row1) * grid.cell
        mesh.flat([(west, bottom), (east, bottom), (east, top), (west, top)], z, tile)


def pavement_edges(pavement, grid, tolerance=1.0):
    """Return the edges of the pavement as closed lines, in metres: where the kerbs go."""
    found, _ = cv2.findContours(pavement, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    edges = []
    for contour in found:
        if len(contour) < 3:
            continue
        fewer = cv2.approxPolyDP(contour, tolerance, True).reshape(-1, 2)
        points = [(grid.west + float(col) * grid.cell,
                   grid.south + (grid.height - 1 - float(row)) * grid.cell)
                  for col, row in fewer]
        edges.append(points + points[:1])
    return edges


def off_pavement(points, grid, step=1.0):
    """Return the parts of a line that aren't over pavement: a path doesn't float over a road."""
    parts, current = [], []
    for a, b in zip(points, points[1:]):
        for i in range(max(1, int(math.dist(a, b) / step))):
            n = max(1, int(math.dist(a, b) / step))
            p0 = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            p1 = (a[0] + (b[0] - a[0]) * (i + 1) / n, a[1] + (b[1] - a[1]) * (i + 1) / n)
            if grid.is_free((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0):
                if len(current) >= 2:
                    parts.append(current)
                current = []
            else:
                current = current or [p0]
                current.append(p1)
    if len(current) >= 2:
        parts.append(current)
    return parts


def floors_of(tags, key=''):
    """Return a building's height, m: OpenStreetMap's, or a guess that varies by building."""
    for name, floors in KNOWN_FLOORS.items():
        if name in tags.get('name', ''):
            return floors * FLOOR_HEIGHT
    if 'height' in tags:
        try:
            return float(tags['height'].split()[0])
        except ValueError:
            pass
    if 'building:levels' in tags:
        try:
            return max(1.0, float(tags['building:levels'])) * FLOOR_HEIGHT
        except ValueError:
            pass
    if tags.get('amenity') == 'shelter' or tags.get('building') in ('shed', 'roof', 'garage'):
        return 3.0
    if tags.get('building') in ('house', 'residential', 'detached'):
        return 7.0
    return (DEFAULT_FLOORS + stable(key) % 2) * FLOOR_HEIGHT


def stable(text):
    """Return a number from a text that's the same on every run and computer."""
    return int(hashlib.sha256(text.encode()).hexdigest()[:8], 16)


def road_width(tags):
    """Return a road's width, m: its tag, its lanes, or its kind's usual."""
    for key, scale in (('width', 1.0), ('lanes', LANE_WIDTH)):
        try:
            return max(2.5, float(tags[key].split()[0]) * scale)
        except (KeyError, ValueError, IndexError):
            pass
    return ROAD_WIDTHS[tags['highway']]


def low_poly_ball():
    """Return an icosahedron's triangles, on a unit ball."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
         (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [tuple(c / math.sqrt(1.0 + t * t) for c in p) for p in v]
    return [(v[a], v[b], v[c]) for a, b, c in (
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
        (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
        (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1))]


BALL = low_poly_ball()


def add_tree(meshes, at, rng):
    """Add one tree: a six-sided trunk and a rounded crown. Returns its trunk and height."""
    height = rng.uniform(5.0, 10.0)
    trunk_top = height * 0.45
    radius = 0.08 + height * 0.02
    ring = [(at[0] + radius * math.cos(i * math.pi / 3.0),
             at[1] + radius * math.sin(i * math.pi / 3.0)) for i in range(6)]
    meshes['bark'].prism_walls(ring + ring[:1], BASE_Z, trunk_top, (1.0, 2.0))
    crown = height * rng.uniform(0.28, 0.38)
    middle = (at[0], at[1], trunk_top + crown * 0.8)
    for a, b, c in BALL:
        corners = [(middle[0] + p[0] * crown, middle[1] + p[1] * crown,
                    middle[2] + p[2] * crown) for p in (a, b, c)]
        meshes['leaves'].triangle(*corners, (0.0, 0.0), (1.0, 0.0), (0.5, 1.0))
    return radius, height, crown, middle


def scatter(ring, rng, spacing=WOOD_TREE_SPACING):
    """Return places for trees inside a ring, on a jittered grid so they don't look planted."""
    points = np.array(open_ring(ring), dtype=np.float32).reshape(-1, 1, 2)
    xs, ys = points[:, 0, 0], points[:, 0, 1]
    found = []
    for x in np.arange(xs.min(), xs.max(), spacing):
        for y in np.arange(ys.min(), ys.max(), spacing):
            p = (float(x + rng.uniform(-3.0, 3.0)), float(y + rng.uniform(-3.0, 3.0)))
            if cv2.pointPolygonTest(points, p, False) > 0:
                found.append(p)
    return found


def in_box(point, box):
    """Return True if a point is inside (west, south, east, north)."""
    return box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3]


def is_surface_lot(tags):
    """Return True for a parking lot on the ground."""
    return tags.get('amenity') == 'parking' and tags.get('parking') not in (
        'multi-storey', 'underground', 'rooftop', 'street_side')


def is_building(tags):
    """Return True for a building."""
    return tags.get('building', 'no') != 'no' and 'building:part' not in tags


def kind_of_area(tags):
    """Return what an area is: lot, building, wood, green, water, or None to skip it."""
    if below_ground(tags):
        return None
    if is_building(tags):
        return 'building'
    if is_surface_lot(tags):
        return 'lot'
    if tags.get('natural') == 'water' or tags.get('waterway') == 'riverbank':
        return 'water'
    if tags.get('natural') == 'wood' or tags.get('landuse') == 'forest':
        return 'wood'
    if tags.get('landuse') in GREEN or tags.get('leisure') in GREEN:
        return 'green'
    return None


def build(campus, box):
    """Turn a Campus into meshes, Nav2's grid, NDT's guessed shapes and the painted roads."""
    names = ['grass', 'green', 'water', 'lot', 'path', 'road', 'paint', 'edge', 'rail', 'kerb',
             'brick', 'stone', 'roof', 'fence', 'bark', 'leaves']
    meshes = {name: Mesh() for name in names}
    solids = Mesh()
    grid = Grid(box)
    rng = random.Random(1867)    # the same world every run; the year's only a number
    trees = [t for t in campus.trees if in_box(t, box)]
    walls = []            # (ring, height) for NDT's guess
    trunks = []           # (x, y, trunk radius, crown centre, crown radius)
    lanes = []            # the painted roads, for lanes.yaml: scoring lane keeping
    report = {'buildings': 0, 'lots': 0, 'roads': 0, 'paths': 0, 'kerbs_m': 0.0, 'islands': 0,
              'fences': 0, 'greens': 0, 'water': 0, 'unknown_height': 0}

    by_kind = {'lot': [], 'building': [], 'wood': [], 'green': [], 'water': []}
    for area in campus.areas:
        kind = kind_of_area(area.tags)
        if kind and in_box(centroid(area.ring), box):
            by_kind[kind].append(area)

    # Roads and lots first: kerbs need to know what's drivable.
    for line in campus.lines:
        highway = line.tags.get('highway')
        if below_ground(line.tags) or not any(in_box(p, box) for p in line.points):
            continue
        if highway in ROAD_WIDTHS:
            width = road_width(line.tags)
            for shape in strip_quads(line.points, width):
                meshes['road'].flat(shape, ROAD_Z, tile=8.0)
                grid.fill(shape, 1)
            if highway in PAINTED:
                for shape in dashes(line.points, LINE_WIDTH):
                    meshes['paint'].flat(shape, PAINT_Z)
                for shape in edge_lines(line.points, width):
                    meshes['edge'].flat(shape, PAINT_Z)
                lanes.append({'name': line.tags.get('name', ''), 'kind': highway,
                              'width': round(width, 2),
                              'oneway': line.tags.get('oneway') in ('yes', '1', 'true'),
                              'points': [[round(x, 2), round(y, 2)] for x, y in line.points]})
            report['roads'] += 1
    for area in by_kind['lot']:
        ring = simplify(area.ring)
        meshes['lot'].flat(ring, LOT_Z, tile=8.0)
        grid.fill(ring, 1)
        report['lots'] += 1

    # Islands inside lots go back up with the grass.
    lot_mask = np.zeros_like(grid.free)
    for area in by_kind['lot']:
        cv2.fillPoly(lot_mask, [grid.pixels(open_ring(area.ring))], 1)
    islands = [hole for area in by_kind['lot'] for hole in area.holes]
    for area in by_kind['green']:
        x, y = centroid(area.ring)
        col = int(round((x - grid.west) / grid.cell))
        row = int(round(grid.height - 1 - (y - grid.south) / grid.cell))
        if 0 <= col < grid.width and 0 <= row < grid.height and lot_mask[row, col]:
            islands.append(area.ring)
    for ring in islands:
        grid.fill(ring, 0)
        report['islands'] += 1

    # A kerb along every edge of the pavement, then the raised ground behind it.
    pavement = grid.free.copy()
    edges = pavement_edges(pavement, grid)
    edges += [line.points for line in campus.lines if line.tags.get('barrier') == 'kerb'
              and any(in_box(p, box) for p in line.points)]
    for points in edges:
        points = straighten(points)
        for shape in strip_quads(points, KERB_TOP, corners=False):
            meshes['kerb'].flat(shape, KERB_TOP_Z, tile=4.0)
            solids.prism_walls(shape + shape[:1], 0.0, KERB_HEIGHT)
        for a, b in zip(points, points[1:]):
            meshes['kerb'].wall(a, b, ROAD_Z, KERB_TOP_Z, 0.0, (4.0, 4.0))
            meshes['kerb'].wall(b, a, ROAD_Z, KERB_TOP_Z, 0.0, (4.0, 4.0))
        report['kerbs_m'] += sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        grid.line(points, 0)

    # What's on the raised ground. Painted on a grid, not laid on top: a park's outline takes
    # in the roads through it, and a road must stay a hole in the grass.
    surface = np.full(pavement.shape, GRASS, dtype=np.uint8)
    for area in by_kind['green'] + by_kind['wood']:
        cv2.fillPoly(surface, [grid.pixels(open_ring(simplify(area.ring)))], GREEN_AREA)
        report['greens'] += 1
    for area in by_kind['water']:
        ring = simplify(area.ring)
        cv2.fillPoly(surface, [grid.pixels(open_ring(ring))], WATER_AREA)
        grid.fill(ring, 0)
        report['water'] += 1
    surface[pavement > 0] = PAVEMENT
    raised_ground(meshes['grass'], surface, GRASS, grid, GRASS_Z, 6.0)
    raised_ground(meshes['green'], surface, GREEN_AREA, grid, GRASS_Z, 6.0)
    raised_ground(meshes['water'], surface, WATER_AREA, grid, GRASS_Z, 8.0)
    wood_trees = []
    for area in by_kind['wood']:
        wood_trees += scatter(simplify(area.ring), rng)
    rng.shuffle(wood_trees)
    trees += [t for t in wood_trees[:MAX_WOOD_TREES] if in_box(t, box)]

    # Paths and rails are for the eye: not being roads, they're blocked already, and a road
    # that crosses a rail stays open. Fences and walls do block.
    grid_of_pavement = Grid(box)
    grid_of_pavement.free = pavement
    for line in campus.lines:
        tags = line.tags
        if below_ground(tags) or not any(in_box(p, box) for p in line.points):
            continue
        if tags.get('highway') in PATHS:
            for part in off_pavement(line.points, grid_of_pavement):
                for shape in strip_quads(straighten(part), PATH_WIDTH):
                    meshes['path'].flat(shape, PATH_Z, tile=4.0)
            report['paths'] += 1
        elif tags.get('railway') == 'light_rail':
            for part in off_pavement(line.points, grid_of_pavement):
                for shape in strip_quads(straighten(part), 4.0):
                    meshes['rail'].flat(shape, PATH_Z, tile=4.0)
        elif tags.get('natural') == 'tree_row':
            for a, b in zip(line.points, line.points[1:]):
                for i in range(max(1, int(math.dist(a, b) / ROW_TREE_SPACING))):
                    t = i * ROW_TREE_SPACING / max(math.dist(a, b), 1e-6)
                    trees.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        elif tags.get('barrier') in BARRIERS:
            height = BARRIERS[tags['barrier']]
            for shape in strip_quads(straighten(line.points), 0.1, corners=False):
                meshes['fence'].prism_walls(shape + shape[:1], 0.0, height, (3.0, 3.0))
                solids.prism_walls(shape + shape[:1], 0.0, height)
            grid.line(line.points, 0)
            walls.append((line.points, height))
            report['fences'] += 1

    for area in by_kind['building']:
        ring = simplify(area.ring)
        height = floors_of(area.tags, area.key)
        report['unknown_height'] += ('building:levels' not in area.tags
                                     and 'height' not in area.tags)
        material = 'stone' if stable(area.key) % 3 == 0 else 'brick'
        meshes[material].prism_walls(ring, BASE_Z, height, (12.0, 3 * FLOOR_HEIGHT))
        solids.prism_walls(ring, 0.0, height)
        meshes['roof'].flat(ring, height, tile=8.0)
        grid.fill(ring, 0)
        walls.append((open_ring(ring) + open_ring(ring)[:1], height))
        report['buildings'] += 1

    for at in trees:
        radius, height, crown, middle = add_tree(meshes, at, rng)
        col = (at[0], at[1])
        grid.line([col, (col[0] + 0.01, col[1])], 0)
        trunks.append((at[0], at[1], radius, middle, crown))
    report['trees'] = len(trees)
    report['kerbs_m'] = round(report['kerbs_m'])

    # Past the box, plain raised grass to the horizon.
    west, south, east, north = box
    # Under it all, pavement: where a road's edge and a kerb's don't quite meet, no sky shows.
    meshes['lot'].flat([(west, south), (east, south), (east, north), (west, north)],
                       UNDER_Z, tile=8.0)
    far = 2500.0
    for ring in ([(-far, -far), (far, -far), (far, south), (-far, south)],
                 [(-far, north), (far, north), (far, far), (-far, far)],
                 [(-far, south), (west, south), (west, north), (-far, north)],
                 [(east, south), (far, south), (far, north), (east, north)]):
        meshes['grass'].flat(ring, GRASS_Z, tile=6.0)
    return meshes, solids, grid, walls, trunks, report, by_kind['lot'], lanes


def objects(walls, trunks):
    """
    Return what the world holds, as truth for labelling (TODO 252: tools/label_drive.py).

    [{'class', 'ring' (m, map frame), 'height'}]. Buildings (walls at least a floor high),
    fences and walls (class other), and each tree as its crown's eight-sided outline.
    """
    out = []
    for ring, height in walls:
        cls = 'building' if height >= FLOOR_HEIGHT else 'other'
        out.append({'class': cls, 'ring': [[round(x, 2), round(y, 2)] for x, y in ring],
                    'height': round(height, 2)})
    for x, y, _, middle, crown in trunks:
        ring = [[round(x + crown * math.cos(k * math.pi / 4), 2),
                 round(y + crown * math.sin(k * math.pi / 4), 2)] for k in range(8)]
        out.append({'class': 'tree', 'ring': ring, 'height': round(middle[2] + crown, 2)})
    return out


def on_triangles(triangles, spacing):
    """Return points evenly over triangles (T x 3 x 3), about `spacing` metres apart."""
    edge = np.linalg.norm(triangles[0, 1] - triangles[0, 0])
    steps = max(1, int(math.ceil(edge / spacing)))
    weights = np.array([(i, j, steps - i - j) for i in range(steps + 1)
                        for j in range(steps + 1 - i)], dtype=np.float64) / steps
    return np.einsum('ni,tij->tnj', weights, triangles).reshape(-1, 3)


def guess_ndt(walls, trunks, spacing=NDT_SPACING):
    """Return NDT's map, a guess: points on the walls, trunks and crowns (N x 3)."""
    chunks = []
    for points, height in walls:
        top = min(height, NDT_CEILING)
        heights = np.arange(NDT_FLOOR, top + 1e-6, spacing)
        if len(heights) == 0:
            continue
        for a, b in zip(points, points[1:]):
            length = math.dist(a, b)
            if length < 1e-6:
                continue
            ts = np.arange(0.0, length, spacing) / length
            xs = a[0] + (b[0] - a[0]) * ts
            ys = a[1] + (b[1] - a[1]) * ts
            grid_x, grid_z = np.meshgrid(xs, heights)
            grid_y, _ = np.meshgrid(ys, heights)
            chunks.append(np.stack([grid_x.ravel(), grid_y.ravel(), grid_z.ravel()], axis=1))
    faces = np.array(BALL)
    for x, y, radius, middle, crown in trunks:
        heights = np.arange(NDT_FLOOR, middle[2], spacing)
        if len(heights):
            turn = np.arange(6) * math.pi / 3.0
            for tx, ty in zip(x + radius * np.cos(turn), y + radius * np.sin(turn)):
                chunks.append(np.stack([np.full_like(heights, tx), np.full_like(heights, ty),
                                        heights], axis=1))
        # On the crown's own flat faces, not on a ball: the lidar hits what the world draws.
        crown_faces = faces * crown + np.array(middle)
        chunks.append(on_triangles(crown_faces, spacing))
    return np.concatenate(chunks) if chunks else np.zeros((0, 3))


def periodic_noise(size, blur, seed):
    """Return smooth noise, -1 to 1, that repeats at its edges so tiles don't show seams."""
    rng = np.random.default_rng(seed)
    freq = np.fft.fftfreq(size)
    fx, fy = np.meshgrid(freq, freq)
    filtered = np.fft.ifft2(np.fft.fft2(rng.standard_normal((size, size)))
                            * np.exp(-2.0 * (np.pi * blur) ** 2 * (fx * fx + fy * fy))).real
    return filtered / max(float(np.abs(filtered).max()), 1e-9)


def normal_map(height, strength):
    """Return a bump map (RGB, 0-255) from a height picture."""
    dx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * strength
    dy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * strength
    length = np.sqrt(dx * dx + dy * dy + 1.0)
    normal = np.stack([-dx / length, dy / length, 1.0 / length], axis=2)
    return np.clip((normal + 1.0) * 127.5, 0, 255).astype(np.uint8)


def colour(height, low, high):
    """Return a picture blending from one colour to another with height (-1 to 1)."""
    t = ((height + 1.0) / 2.0)[..., None]
    picture = np.array(low, dtype=np.float64) * (1.0 - t) + np.array(high, dtype=np.float64) * t
    return np.clip(picture, 0, 255).astype(np.uint8)


def facade(size=512, seed=7, wall=((120, 72, 52), (150, 92, 68)), window=(42, 58, 72)):
    """Return a wall's picture: four windows wide, three floors tall, with brick or stone."""
    grain = periodic_noise(size, 1.0, seed) * 0.6 + periodic_noise(size, 12.0, seed + 1) * 0.4
    picture = colour(grain, *wall)
    height = grain * 0.3
    for floor in range(3):
        for col in range(4):
            x0, x1 = int((col + 0.22) * size / 4), int((col + 0.78) * size / 4)
            y0, y1 = int((floor + 0.30) * size / 3), int((floor + 0.72) * size / 3)
            picture[y0:y1, x0:x1] = window
            picture[y0:y1:9, x0:x1] = (70, 92, 110)
            height[y0:y1, x0:x1] = -1.0
    return picture, height


def make_textures(folder):
    """Draw the materials' pictures and their bump maps."""
    folder.mkdir(parents=True, exist_ok=True)
    size = 1024

    def save(name, picture, height, strength):
        Image.fromarray(picture).save(folder / f'{name}.png')
        Image.fromarray(normal_map(height, strength)).save(folder / f'{name}_n.png')

    def blend(seed, *parts):
        return sum(periodic_noise(size, blur, seed + i) * weight
                   for i, (blur, weight) in enumerate(parts))

    # Pavement: fine stones, broad patches, and a few long cracks (Ben: bump-mapped).
    fine = blend(1, (0.8, 0.7), (3.0, 0.3))
    patches = blend(5, (60.0, 1.0))
    cracks = np.abs(periodic_noise(size, 30.0, 11)) < 0.004
    height = fine - cracks * 0.8
    road = colour(fine * 0.75 + patches * 0.25, (58, 58, 60), (112, 112, 114))
    road[cracks] = (50, 50, 52)
    save('road', road, height, 5.0)
    lot = colour(fine * 0.75 + patches * 0.25, (72, 72, 74), (128, 128, 130))
    lot[cracks] = (60, 60, 62)
    save('lot', lot, height, 5.0)
    concrete = blend(21, (1.0, 0.5), (24.0, 0.5))
    save('kerb', colour(concrete, (150, 148, 142), (205, 203, 196)), concrete, 2.0)
    save('path', colour(concrete, (140, 138, 132), (190, 188, 180)), concrete, 2.0)
    blades = blend(31, (0.7, 0.75), (40.0, 0.25))
    save('grass', colour(blades, (46, 86, 34), (102, 150, 58)), blades, 5.0)
    save('green', colour(blades, (56, 100, 40), (120, 166, 68)), blades, 5.0)
    ripple = blend(41, (10.0, 1.0))
    save('water', colour(ripple, (34, 66, 92), (66, 104, 132)), ripple, 3.0)
    stones = blend(51, (2.0, 1.0))
    save('rail', colour(stones, (70, 66, 60), (140, 134, 124)), stones, 6.0)
    roof = blend(61, (1.5, 0.7), (30.0, 0.3))
    save('roof', colour(roof, (70, 70, 72), (120, 118, 114)), roof, 3.0)
    save('fence', colour(concrete, (90, 92, 92), (130, 132, 132)), concrete, 2.0)
    bark = blend(71, (2.0, 1.0))
    save('bark', colour(bark, (54, 40, 30), (96, 74, 54)), bark, 6.0)
    leaves = blend(81, (2.0, 0.7), (12.0, 0.3))
    save('leaves', colour(leaves, (64, 118, 48), (122, 172, 72)), leaves, 5.0)
    painted = np.full((64, 64, 3), 225, dtype=np.uint8)
    painted[..., 2] = 150
    Image.fromarray(painted).save(folder / 'paint.png')
    flat = np.full((64, 64, 3), (128, 128, 255), dtype=np.uint8)
    Image.fromarray(flat).save(folder / 'paint_n.png')
    Image.fromarray(np.full((64, 64, 3), 235, dtype=np.uint8)).save(folder / 'edge.png')
    Image.fromarray(flat).save(folder / 'edge_n.png')
    brick, bump = facade()
    save('brick', brick, bump, 3.0)
    stone, bump = facade(seed=9, wall=((158, 150, 134), (196, 188, 170)))
    save('stone', stone, bump, 3.0)


def visual(name):
    """Return one material's mesh, drawn with its textures."""
    return (f"""        <visual name="{name}">
          <geometry><mesh><uri>meshes/{name}.obj</uri></mesh></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient><specular>0 0 0 1</specular>
            <pbr><metal>
              <albedo_map>textures/{name}.png</albedo_map>
              <normal_map>textures/{name}_n.png</normal_map>
              <metalness>0.0</metalness><roughness>0.95</roughness>
            </metal></pbr>
          </material>
        </visual>
""")


CAMPUS_NOTE = ("Carleton's campus: made by tools/campus_world.py, from OpenStreetMap.\n"
               "  Data (c) OpenStreetMap contributors, ODbL. Don't edit this: make it again.")


def world_sdf(names, datum, note=CAMPUS_NOTE):
    """Return the world: the same systems, name and origin as the test lot's."""
    return (f"""<?xml version="1.0"?>
<!--
  {note}
  Its spherical_coordinates MUST match navsat.yaml's datum, so world = map.
-->
<sdf version="1.9">
  <world name="aav">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Ravensim draws the cameras and casts the lidar and radar here (worlds.has_sensors). -->
    <plugin filename="ravensim" name="ravensim::sensors"/>

    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{datum[0]}</latitude_deg>
      <longitude_deg>{datum[1]}</longitude_deg>
      <elevation>70.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 200 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>5000 5000</size></plane></geometry>
        </collision>
      </link>
    </model>

    <model name="campus">
      <static>true</static>
      <link name="link">
        <collision name="solids">
          <geometry><mesh><uri>meshes/solids.obj</uri></mesh></geometry>
        </collision>
{''.join(visual(name) for name in names)}      </link>
    </model>
  </world>
</sdf>
""")


def start_pose(lots, grid):
    """Return where the car starts: the middle of START_LOT, or the nearest room to it."""
    wanted = [a for a in lots if a.tags.get('name') == START_LOT or a.tags.get('ref') == START_LOT]
    at = centroid(wanted[0].ring) if wanted else (0.0, 0.0)
    free = grid.nearest_free(*at)
    if free is None:
        raise ValueError(f'no room for the car near {at}: is the map empty?')
    return free


def box_round(around=CARLETON):
    """Return (west, south, east, north), m from the datum."""
    west, south, east, north = around
    return (-west, -south, east, north)


def main():
    parser = argparse.ArgumentParser(description="Build Carleton's world from OpenStreetMap.")
    parser.add_argument('--osm', type=Path, default=OSM, help=f'default {OSM}')
    parser.add_argument('--out', type=Path, default=OUT, help=f'default {OUT}')
    parser.add_argument('--ndt-out', type=Path, default=NDT_OUT, help=f'default {NDT_OUT}')
    args = parser.parse_args()
    if not args.osm.exists():
        subprocess.run([str(ROOT / 'tools' / 'get_carla_osm.sh')], check=True)
    datum_list = yaml.safe_load(NAVSAT.read_text())['navsat_transform']['ros__parameters']['datum']
    lat0, lon0 = datum_list[0], datum_list[1]

    def project(lat, lon):
        east, north, _ = geodetic_to_enu(lat0, lon0, 0.0, lat, lon, 0.0)
        return (east, north)

    campus = read_osm(args.osm, project)
    box = box_round()
    meshes, solids, grid, walls, trunks, report, lots, lanes = build(campus, box)
    # The stop lines, white, where the Lanelet2 map's are (the car stops at them).
    bars = [bar for bar in campus_lanelet.site_stop_bars((lat0, lon0))
            if any(in_box(p, box) for p in bar)]
    for bar in bars:
        meshes['edge'].flat(bar, PAINT_Z)
    report['stop_lines'] = len(bars)
    (args.out / 'meshes').mkdir(parents=True, exist_ok=True)
    names = [name for name, mesh in meshes.items() if mesh.vertices]
    triangles = 0
    for name in names:
        meshes[name].write(args.out / 'meshes' / f'{name}.obj')
        triangles += len(meshes[name].vertices) // 3
    solids.write(args.out / 'meshes' / 'solids.obj')
    make_textures(args.out / 'textures')
    (args.out / 'campus.sdf').write_text(world_sdf(names, (lat0, lon0)))
    grid.save(args.out, 'campus', f'{lat0}, {lon0}')
    start = start_pose(lots, grid)
    ndt = guess_ndt(walls, trunks)
    args.ndt_out.parent.mkdir(parents=True, exist_ok=True)
    write_pcd(args.ndt_out.with_suffix('.pcd'), ndt)
    args.ndt_out.with_suffix('.yaml').write_text(yaml.safe_dump({
        'kind': 'guess: sampled from OpenStreetMap shapes by tools/campus_world.py',
        'frame': 'map', 'datum': [lat0, lon0], 'points': len(ndt), 'spacing_m': NDT_SPACING,
    }))
    report.update(triangles=triangles, ndt_points=len(ndt),
                  map_cells=[grid.width, grid.height])
    (args.out / 'place.yaml').write_text(yaml.safe_dump({
        'start': {'x': round(start[0], 2), 'y': round(start[1], 2), 'yaw': 0.0},
        'datum': [lat0, lon0], 'box': list(box), 'source': str(args.osm.name),
        'credit': '(c) OpenStreetMap contributors, ODbL', 'made': report,
    }))
    # Truth for labelling (TODO 252): what's where, as label-drive scores its labels.
    (args.out / 'objects.yaml').write_text(yaml.safe_dump(
        {'objects': objects(walls, trunks)}, default_flow_style=None))
    (args.out / 'lanes.yaml').write_text(
        '# The painted roads: centre line (m, map frame), width, one way or not. For scoring\n'
        '# lane keeping (tools/lane_score.py). (c) OpenStreetMap contributors, ODbL.\n'
        + yaml.safe_dump({'roads': lanes}, default_flow_style=None))
    print(f'{args.out}: {report}')
    print(f'start at {start}')
    chunk_map(args.out / 'campus.yaml', args.out / 'chunks')
    lanelets = campus_lanelet.write(args.out / 'campus.osm', (lat0, lon0))
    print(f'lanelets: {lanelets} in {args.out / "campus.osm"}')


def chunk_map(map_yaml: Path, out: Path):
    """Cut the map, and NDT's, into 64 m chunk files (chunk-map, ndt-chunks: TODO 175)."""
    crate = ROOT / 'ros' / 'bringup' / 'aav_description' / 'map-chunks'
    if shutil.which('cargo') is None:
        print('chunks: skipped (no cargo here); the whole map is written')
        return
    subprocess.run(['cargo', 'run', '-q', '--release', '--locked', '--bin', 'chunk-map', '--',
                    str(map_yaml), str(out)], cwd=crate, check=True)
    ndt = ROOT / 'maps' / 'ndt' / 'campus.pcd'
    if ndt.exists():
        origin = yaml.safe_load(map_yaml.read_text())['origin']
        subprocess.run(['cargo', 'run', '-q', '--release', '--locked', '--bin', 'ndt-chunks',
                        '--', str(ndt), str(out / 'ndt'), str(origin[0]), str(origin[1])],
                       cwd=ROOT / 'ros' / 'localization' / 'aav_ndt', check=True)


if __name__ == '__main__':
    main()
