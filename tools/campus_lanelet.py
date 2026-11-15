"""
The campus's roads as a Lanelet2 map (.osm), for lanelet-map. No ROS.

TODO 175, step 6: the route level's map, on the same roads the street router drives
(campus_site.yaml's kinds, campus_graph.drivable), cut at every junction. Each road
becomes one lanelet per lane, its
lines offset from the road's centre line (3.5 m lanes: a one-way road all one way, a
two-way road half each way); where road ends meet (within JOIN m), short lanelets with
virtual lines join each lane coming in to each lane going out, except a U-turn or a turn
sharper than MAX_TURN; at a dead end, a lane turns round onto its road's other way.
Points carry local_x, local_y (lanelet-map reads them) and lat/lon
(other Lanelet2 readers).

    pixi run python3 tools/campus_lanelet.py [maps/campus/campus.osm]

pixi run campus calls it. (c) OpenStreetMap contributors, ODbL.
"""

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from xml.sax.saxutils import quoteattr


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros' / 'sim' / 'aav_carla'))
sys.path.insert(0, str(ROOT / 'ros' / 'drivers' / 'aav_routes'))
from aav_carla.frames import enu_to_geodetic, geodetic_to_enu  # noqa: E402, I100
from aav_routes import campus_graph  # noqa: E402, I100

SITE = ROOT / 'ros' / 'drivers' / 'aav_routes' / 'config' / 'campus_site.yaml'

LANE = 3.5                  # m: one lane's width
JOIN = 0.5                  # m: road ends this close meet
MAX_TURN = math.radians(150.0)   # rad: sharper is a U-turn, never joined


@dataclass
class Osm:
    """The map being written: nodes, ways and relations, each with its own id."""

    nodes: list = field(default_factory=list)       # (id, x, y)
    ways: list = field(default_factory=list)        # (id, [node ids], tags)
    relations: list = field(default_factory=list)   # (id, [(kind, role, ref)], tags)
    next_id: int = 1

    def new_id(self) -> int:
        self.next_id += 1
        return self.next_id - 1

    def line(self, points, kind='line_thin', subtype='solid', nodes=None) -> tuple:
        """Add a way through points (new nodes unless given); return (way id, node ids)."""
        if nodes is None:
            nodes = []
            for x, y in points:
                nodes.append(self.new_id())
                self.nodes.append((nodes[-1], x, y))
        way = self.new_id()
        self.ways.append((way, list(nodes), {'type': kind, 'subtype': subtype}))
        return way, nodes

    def lanelet(self, left, right, tags) -> int:
        rel = self.new_id()
        self.relations.append((rel, [('way', 'left', left), ('way', 'right', right)],
                               {'type': 'lanelet', 'subtype': 'road', **tags}))
        return rel


def offset(points, d):
    """Return the line moved d m to its left (negative: right), mitred at each corner."""
    out = []
    for i, (x, y) in enumerate(points):
        a = points[max(i - 1, 0)]
        b = points[min(i + 1, len(points) - 1)]
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        nx, ny = -math.sin(heading), math.cos(heading)
        # Keep the lane's width at a corner (up to twice, at very sharp ones).
        if 0 < i < len(points) - 1:
            h1 = math.atan2(y - a[1], x - a[0])
            h2 = math.atan2(b[1] - y, b[0] - x)
            half = abs(math.remainder(h2 - h1, math.tau)) / 2.0
            scale = min(1.0 / max(math.cos(half), 1e-9), 2.0)
        else:
            scale = 1.0
        out.append((x + nx * d * scale, y + ny * d * scale))
    return out


def lane_counts(width: float, oneway: bool) -> tuple[int, int]:
    """Return (lanes along the road's direction, lanes against it)."""
    lanes = max(1, round(width / LANE))
    if oneway:
        return lanes, 0
    return max(1, lanes - lanes // 2), max(1, lanes // 2)


@dataclass
class Lane:
    """One lane of a road, as driven: its lanelet and the nodes its lines start and end on."""

    road: int
    forward: bool
    lanelet: int
    start: tuple        # (left node, right node)
    end: tuple
    heading_in: float   # rad: the heading at its start
    heading_out: float  # rad: the heading at its end


def heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def road_lanes(osm: Osm, index: int, road: dict) -> list[Lane]:
    """Add one road's lanes; return them."""
    points = [tuple(p) for p in road['points']]
    ahead, back = lane_counts(float(road['width']), bool(road['oneway']))
    width = float(road['width'])
    tags = {'one_way': 'yes', 'location': 'urban', 'name': str(road.get('name') or '')}
    lanes = []
    for forward, count in ((True, ahead), (False, back)):
        if count == 0:
            continue
        line = points if forward else points[::-1]
        # This direction's lanes lie from its right edge to the road's middle (or, one
        # way, across the whole road): the k-th line from the right edge, offset to the left.
        right_edge = -width / 2.0
        lane = (width / 2.0 if not road['oneway'] else width) / count
        lines = []
        for k in range(count + 1):
            # The edges (and a two-way road's middle) solid; between lanes, dashed.
            lines.append(osm.line(offset(line, right_edge + k * lane),
                                  subtype='solid' if k in (0, count) else 'dashed'))
        for k in range(count):
            (right, rn), (left, ln) = lines[k], lines[k + 1]
            rel = osm.lanelet(left, right, tags)
            lanes.append(Lane(index, forward, rel, (ln[0], rn[0]), (ln[-1], rn[-1]),
                              heading(line[0], line[1]), heading(line[-2], line[-1])))
    return lanes


def join(osm: Osm, roads: list, lanes: list[Lane]) -> int:
    """Join each lane ending at a meeting point to each lane starting there; return how many."""
    def end_point(lane, at_end):
        """Return where the lane ends (at_end) or starts, on its road's centre line."""
        pts = roads[lane.road]['points']
        return tuple(pts[-1] if at_end == lane.forward else pts[0])

    def connect(a, b):
        left, _ = osm.line(None, 'virtual', '', nodes=[a.end[0], b.start[0]])
        right, _ = osm.line(None, 'virtual', '', nodes=[a.end[1], b.start[1]])
        osm.lanelet(left, right, {'one_way': 'yes', 'location': 'urban', 'turn': 'yes'})

    joined = 0
    for a in lanes:
        here = end_point(a, True)
        onward = [b for b in lanes if b.road != a.road
                  and math.dist(here, end_point(b, False)) <= JOIN
                  and abs(math.remainder(b.heading_in - a.heading_out, math.tau)) <= MAX_TURN]
        if not onward:
            # A dead end: turn round onto the same road's other way, if it has one.
            onward = [b for b in lanes if b.road == a.road and b.forward != a.forward]
        for b in onward:
            connect(a, b)
            joined += 1
    return joined


def build(roads: list) -> tuple[Osm, dict]:
    """Return the map of all the roads, and a report."""
    osm = Osm()
    lanes = [lane for i, road in enumerate(roads) for lane in road_lanes(osm, i, road)]
    joins = join(osm, roads, lanes)
    return osm, {'roads': len(roads), 'lanes': len(lanes), 'joins': joins}


def to_xml(osm: Osm, datum) -> str:
    """Return the map as Lanelet2 .osm text; lat/lon from local x, y round the datum."""
    lat0, lon0 = datum
    out = ["<?xml version='1.0' encoding='UTF-8'?>", "<osm version='0.6' generator='aav'>"]
    for n, x, y in osm.nodes:
        lat, lon, _ = enu_to_geodetic(lat0, lon0, 0.0, x, y, 0.0)
        out.append(f"  <node id='{n}' visible='true' version='1' lat='{lat:.9f}' "
                   f"lon='{lon:.9f}'><tag k='local_x' v='{x:.3f}'/>"
                   f"<tag k='local_y' v='{y:.3f}'/></node>")
    for w, nodes, tags in osm.ways:
        nds = ''.join(f"<nd ref='{n}'/>" for n in nodes)
        tg = ''.join(f'<tag k={quoteattr(k)} v={quoteattr(v)}/>' for k, v in tags.items() if v)
        out.append(f"  <way id='{w}' visible='true' version='1'>{nds}{tg}</way>")
    for r, members, tags in osm.relations:
        ms = ''.join(f"<member type='{k}' role='{role}' ref='{ref}'/>"
                     for k, role, ref in members)
        tg = ''.join(f'<tag k={quoteattr(k)} v={quoteattr(v)}/>' for k, v in tags.items() if v)
        out.append(f"  <relation id='{r}' visible='true' version='1'>{ms}{tg}</relation>")
    out.append('</osm>')
    return '\n'.join(out) + '\n'


def roads_from_osm(osm, kinds, datum, oneway=True) -> list:
    """
    Return the roads the car may use (campus_graph.drivable: the router's own) as roads.

    Each way is cut at every node another drivable way shares (a junction, even mid-way),
    turned to run along its one-way direction, and put east and north of the datum. Its
    width: the `lanes` tag x LANE, else one lane each way (one lane, one way).
    """
    lat0, lon0 = datum
    ways = [(tags, [r for r in refs if r in osm.nodes]) for tags, refs in osm.ways.values()
            if campus_graph.drivable(tags, kinds)]
    uses = {}
    for _, refs in ways:
        for k, r in enumerate(refs):
            ends = k in (0, len(refs) - 1)
            uses[r] = uses.get(r, 0) + (2 if ends else 1)

    def xy(ref):
        east, north, _ = geodetic_to_enu(lat0, lon0, 0.0, *osm.nodes[ref], 0.0)
        return [round(east, 2), round(north, 2)]

    roads = []
    for tags, refs in ways:
        direction = campus_graph.one_way(tags) if oneway else 0
        if direction < 0:
            refs = refs[::-1]
        try:
            lanes = max(1, int(tags.get('lanes', '0')))
        except ValueError:
            lanes = 0
        width = (lanes or (1 if direction else 2)) * LANE
        piece = refs[:1]
        for r in refs[1:]:
            piece.append(r)
            if uses.get(r, 0) > 1 and r != refs[-1] or r == refs[-1]:
                if len(piece) > 1:
                    roads.append({'points': [xy(p) for p in piece], 'width': width,
                                  'oneway': direction != 0, 'name': tags.get('name', ''),
                                  'kind': tags.get('highway', '')})
                piece = [r]
    return roads


def write(out: Path, datum, site: Path = SITE) -> dict:
    """Write the site's drivable roads as a Lanelet2 map at out; return the report."""
    info = campus_graph.read_site(site)
    osm_data = campus_graph.read_osm(ROOT / info.osm)
    osm, report = build(roads_from_osm(osm_data, info.roads, datum, info.oneway))
    out.write_text(to_xml(osm, datum))
    return report


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'maps/campus/campus.osm'
    print(f'{out}: {write(out, campus_graph.read_site(SITE).datum)}')


if __name__ == '__main__':
    main()
