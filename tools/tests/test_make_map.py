"""make_map: the maps match their specs, and AMCL's map keeps only the obstacles' edges."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import make_map  # noqa: E402
import yaml  # noqa: E402

MAPS = Path(__file__).resolve().parents[2] / 'ros' / 'drivers' / 'aav_navigation' / 'maps'
BOX = {'name': 'box', 'resolution': 1.0, 'size': [6, 5], 'origin': [0, 0],
       'blocked': [[1, 1, 5, 4]]}


def test_a_solid_box_is_blocked_all_through():
    grid, width, height, _, _ = make_map.make(BOX)
    assert (width, height) == (6, 5)
    assert sum(row.count(make_map.BLOCKED) for row in grid) == 4 * 3


def test_an_outline_keeps_the_edge_and_unknowns_the_inside():
    grid, *_ = make_map.make(dict(BOX, outline=True))
    assert sum(row.count(make_map.BLOCKED) for row in grid) == 4 * 3 - 2
    assert grid[2][2] == grid[2][3] == make_map.UNKNOWN     # the middle row's inside
    assert grid[1][1] == grid[3][4] == make_map.BLOCKED     # corners stay
    assert grid[0][0] == make_map.FREE


def test_a_wall_on_the_maps_edge_keeps_its_inner_face():
    grid, *_ = make_map.make({'name': 'lot', 'resolution': 1.0, 'size': [5, 5],
                              'border': 2.0, 'outline': True})
    assert grid[2][2] == make_map.FREE
    assert grid[1][2] == make_map.BLOCKED       # the inner face
    assert grid[0][2] == make_map.UNKNOWN       # behind it, against the edge


def test_amcl_gets_the_lots_own_finer_map_and_nav2_keeps_its_own():
    spec = yaml.safe_load((MAPS / 'example_lot.spec.yaml').read_text())
    amcl = make_map.amcl_spec(spec)
    assert amcl['name'] == 'example_lot_amcl' and amcl['outline'] is True
    assert amcl['resolution'] < spec['resolution']
    assert amcl['blocked'] == spec['blocked'] and amcl['origin'] == spec['origin']
    assert make_map.amcl_spec(BOX) is None
    # The files in git are what the spec makes.
    for one in (spec, amcl):
        meta = yaml.safe_load((MAPS / f'{one["name"]}.yaml').read_text())
        assert meta['resolution'] == one['resolution']
        assert meta['origin'][:2] == [float(v) for v in one['origin']]
        header = (MAPS / meta['image']).read_bytes()[:80].split(b'\n')
        width, height = map(int, header[2].split())
        assert (width, height) == tuple(round(v / one['resolution']) for v in one['size'])
