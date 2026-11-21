"""
Ravensim's worlds: which ones render the car's sensors, and the road world against the bench.

No ROS needed. The road world must stay the bench world (aav_bench/config/bench.yaml): the
same lane, paint, stop sign and obstacle, so a check passed on one means the same on the
other. Every world sits on the GPS datum (aav_localization/config/navsat.yaml).
"""

from pathlib import Path
from xml.etree import ElementTree

from aav_sim_models import worlds
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parents[1]
BENCH = yaml.safe_load((SRC / 'sim' / 'aav_bench' / 'config' / 'bench.yaml').read_text())
CAMERA = BENCH['mock_camera']['ros__parameters']
RADAR = BENCH['mock_radar']['ros__parameters']


def sdf(path):
    return ElementTree.parse(path).getroot()


def numbers(text):
    return [float(v) for v in text.split()]


def visual(root, name):
    found = [v for v in root.iter('visual') if v.get('name') == name]
    assert len(found) == 1, f'one visual called {name}'
    return found[0]


def test_world_files_and_an_unknown_world():
    for world in ('empty', 'test_lot', 'road'):
        assert worlds.world_file(str(PACKAGE), world).endswith(f'worlds/{world}.sdf')
    with pytest.raises(ValueError, match='road'):
        worlds.world_file(str(PACKAGE), 'moon')


def test_the_campus_is_made_not_kept(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match='pixi run campus'):
        worlds.world_file(str(PACKAGE), worlds.CAMPUS)
    sdf = tmp_path / worlds.CAMPUS_SDF
    sdf.parent.mkdir(parents=True)
    sdf.write_text('<sdf/>')
    assert worlds.world_file(str(PACKAGE), worlds.CAMPUS) == str(sdf.resolve())


def test_the_campus_says_where_the_car_starts():
    place = 'start: {x: 49.25, y: -70.75, yaw: 0.0}\n'
    assert worlds.campus_start(place) == (49.25, -70.75, 0.0)


def test_the_road_and_lot_worlds_render_sensors():
    def renders(world):
        return worlds.has_sensors(worlds.world_file(str(PACKAGE), world))
    assert renders('road')
    assert not renders('empty')      # headless anywhere: CI on Linux, oirp-spss
    assert renders('test_lot')


def test_ravensim_renders_cameras_where_the_world_has_sensors():
    road = worlds.world_file(str(PACKAGE), 'road')
    empty = worlds.world_file(str(PACKAGE), 'empty')
    assert worlds.DEFAULT_SIMULATOR == 'ravensim'
    assert worlds.has_cameras(road) and not worlds.has_cameras(empty)
    with pytest.raises(ValueError, match='simulator:=unity'):
        worlds.check_simulator('unity')


def test_no_gazebo_plugin_is_left_in_a_world():
    for path in sorted((PACKAGE / 'worlds').glob('*.sdf')):
        names = [p.get('name') for p in sdf(path).iter('plugin')]
        assert all(n == worlds.SENSORS for n in names), path.name


def test_a_campus_made_before_gazebo_went_still_renders(tmp_path):
    old = tmp_path / 'campus.sdf'
    old.write_text('<sdf><world><plugin name="gz::sim::systems::Sensors"/></world></sdf>')
    assert worlds.has_sensors(str(old))


def test_the_old_simulator_name_works_for_one_release_and_gazebo_is_gone():
    assert worlds.check_simulator('ravensim') == 'ravensim'
    with pytest.raises(ValueError, match='old name'):
        worlds.check_simulator('own')
    with pytest.raises(ValueError, match='Gazebo was removed'):
        worlds.check_simulator('gazebo')


def test_the_cameras_lockstep_when_drawn_in_software():
    assert not worlds.camera_lockstep('auto', {})
    assert worlds.camera_lockstep('auto', {worlds.SOFTWARE_GL_VAR: '1'})
    assert worlds.camera_lockstep('true', {})
    assert not worlds.camera_lockstep('False', {worlds.SOFTWARE_GL_VAR: '1'})
    with pytest.raises(ValueError, match='auto, true or false'):
        worlds.camera_lockstep('sometimes', {})


def test_our_cameras_run_from_the_repository_on_the_world_file():
    root = worlds.repo_root(__file__)
    assert (Path(root) / worlds.SIM_CAMERAS).is_file()
    cmd = worlds.sim_cameras_command(root, '/w/road.sdf', ['/m1', '/m2'], chase=False,
                                     relay='/vehicle/sim/camera/front_left/image_raw')
    assert cmd[:2] == ['cargo', 'run'] and '--features' in cmd
    tail = cmd[cmd.index('--') + 1:]
    assert tail[:3] == ['--ros', '--pose', 'truth']
    assert tail[tail.index('--world') + 1] == '/w/road.sdf'
    assert tail[tail.index('--models') + 1] == '/m1:/m2'
    assert tail[tail.index('--cameras') + 1] == 'front_left,front_right,rear'
    relay = 'front_left=/vehicle/sim/camera/front_left/image_raw'
    assert tail[tail.index('--image-topic') + 1] == relay
    assert '--cameras' not in worlds.sim_cameras_command(root, 'w', [])
    assert tail[-2:] == ['--graphics', 'auto']
    looks = worlds.sim_cameras_command(root, 'w', [], looks='Simple')
    assert looks[-2:] == ['--graphics', 'simple']


def test_graphics_words():
    assert [worlds.graphics(w) for w in ('auto', 'pretty', 'SIMPLE')] == [
        'auto', 'pretty', 'simple']
    with pytest.raises(ValueError, match='auto, simple or pretty'):
        worlds.graphics('fancy')


def test_software_rendering_reaches_the_cameras():
    # tools/software_gl.sh's settings: more cargo features, and the Vulkan loader on a Mac.
    assert worlds.sim_cameras_features({}) == 'ros'
    extra = {worlds.FEATURES_VAR: 'vulkan-on-mac'}
    assert worlds.sim_cameras_features(extra) == 'ros,vulkan-on-mac'
    lib = {worlds.VULKAN_LIB_VAR: '/tmp/vk'}
    assert worlds.sim_cameras_env(lib, 'Darwin') == {'DYLD_LIBRARY_PATH': '/tmp/vk'}
    assert worlds.sim_cameras_env(lib, 'Linux') == {}
    assert worlds.sim_cameras_env({}, 'Darwin') == {}
    # In front of ROS's own libraries, never in their place.
    both = worlds.sim_cameras_env({**lib, 'DYLD_LIBRARY_PATH': '/ros/lib'}, 'Darwin')
    assert both == {'DYLD_LIBRARY_PATH': '/tmp/vk:/ros/lib'}


def test_every_world_is_named_aav_and_sits_on_the_datum():
    navsat = SRC / 'localization' / 'aav_localization' / 'config' / 'navsat.yaml'
    lat, lon, _ = yaml.safe_load(navsat.read_text())['navsat_transform']['ros__parameters'][
        'datum']
    for path in sorted((PACKAGE / 'worlds').glob('*.sdf')):
        world = sdf(path).find('world')
        assert world.get('name') == 'aav', path.name
        coordinates = world.find('spherical_coordinates')
        assert float(coordinates.findtext('latitude_deg')) == lat, path.name
        assert float(coordinates.findtext('longitude_deg')) == lon, path.name
        assert float(coordinates.findtext('heading_deg')) == 0.0, path.name


def test_the_road_is_the_bench_road():
    road = sdf(PACKAGE / 'worlds' / 'road.sdf')
    half = CAMERA['lane_width'] / 2.0
    tarmac = visual(road, 'tarmac')
    assert numbers(tarmac.find('pose').text)[1] == CAMERA['road_y']
    assert numbers(tarmac.find('geometry/box/size').text)[1] == pytest.approx(
        2.0 * (half + CAMERA['shoulder']))
    for name, side in (('left_line', 1.0), ('right_line', -1.0)):
        line = visual(road, name)
        assert numbers(line.find('pose').text)[1] == pytest.approx(CAMERA['road_y'] + side * half)
        assert numbers(line.find('geometry/box/size').text)[1] == CAMERA['line_width']
    # The road starts behind the car and runs well past the stop sign.
    x, length = numbers(tarmac.find('pose').text)[0], numbers(tarmac.find('geometry/box/size')
                                                              .text)[0]
    assert x - length / 2.0 <= -10.0 and x + length / 2.0 >= 60.0


def test_the_stop_sign_is_the_benchs():
    road = sdf(PACKAGE / 'worlds' / 'road.sdf')
    signs = [i for i in road.iter('include') if i.findtext('uri') == 'model://stop_sign']
    assert len(signs) == 1
    x, y, z, _, _, yaw = numbers(signs[0].findtext('pose'))
    assert [x, y] == CAMERA['stop_signs']
    assert yaw == pytest.approx(3.14159265)      # facing the traffic, which drives east
    plate = visual(sdf(PACKAGE / 'models' / 'stop_sign' / 'model.sdf'), 'plate')
    assert numbers(plate.find('pose').text)[2] == CAMERA['sign_height']
    mesh = (PACKAGE / 'models' / 'stop_sign' / 'meshes' / 'stop_sign.obj').read_text()
    corners = [numbers(line[2:]) for line in mesh.splitlines() if line.startswith('v ')]
    assert 2.0 * max(abs(c[1]) for c in corners) == pytest.approx(CAMERA['sign_size'])


def test_the_obstacle_is_the_benchs():
    x1, y1, x2, y2 = RADAR['boxes']
    assert worlds.OBSTACLE == ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    box = sdf(PACKAGE / 'models' / 'obstacle' / 'model.sdf')
    size = numbers(box.find('.//visual/geometry/box/size').text)
    assert size == [x2 - x1, y2 - y1, RADAR['box_height']]
    assert numbers(box.find('.//link/pose').text)[2] == RADAR['box_height'] / 2.0  # on the ground


def test_the_road_worlds_lane_map_matches_its_tarmac_and_lines():
    lanes = yaml.safe_load((PACKAGE / 'worlds' / 'road.lanes.yaml').read_text())
    (road,) = lanes['roads']
    tarmac = visual(sdf(PACKAGE / 'worlds' / 'road.sdf'), 'tarmac')
    x, y = numbers(tarmac.find('pose').text)[:2]
    length, width = numbers(tarmac.find('geometry/box/size').text)[:2]
    assert road['width'] == pytest.approx(width)
    assert road['width'] - 2 * road['edge_inset'] == pytest.approx(CAMERA['lane_width'])
    assert road['points'] == [[x - length / 2, y], [x + length / 2, y]]
    assert road['lanes'] == 1


def test_the_lane_map_by_world(tmp_path, monkeypatch):
    assert worlds.lane_map_file(str(PACKAGE), 'road', '') == ''
    assert worlds.lane_map_file(str(PACKAGE), 'road', 'world').endswith('road.lanes.yaml')
    with pytest.raises(ValueError, match='no lane map'):
        worlds.lane_map_file(str(PACKAGE), 'test_lot', 'world')
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match='pixi run campus'):
        worlds.lane_map_file(str(PACKAGE), worlds.CAMPUS, 'world')
    mine = tmp_path / 'mine.yaml'
    mine.write_text('roads: []\n')
    assert worlds.lane_map_file(str(PACKAGE), 'empty', 'mine.yaml') == str(mine)


def test_a_twin_world_by_its_path(tmp_path):
    sdf = tmp_path / 'twin.sdf'
    with pytest.raises(ValueError, match='twin-world'):
        worlds.world_file(str(PACKAGE), str(sdf))
    sdf.write_text('<sdf/>')
    assert worlds.is_world_file(str(sdf)) and not worlds.is_world_file('test_lot')
    assert worlds.world_file(str(PACKAGE), str(sdf)) == str(sdf)
    assert worlds.twin(str(sdf)) == {'map': str(tmp_path / 'map2d.yaml'), 'ndt': '',
                                     'start': (0.0, 0.0, 0.0)}
    (tmp_path / 'twin.yaml').write_text('start: {x: 49.5, y: -60.0, yaw: 1.5}\n')
    (tmp_path / 'ndt.pcd').write_text('')
    t = worlds.twin(str(sdf))
    assert t['start'] == (49.5, -60.0, 1.5) and t['ndt'] == str(tmp_path / 'ndt.pcd')
