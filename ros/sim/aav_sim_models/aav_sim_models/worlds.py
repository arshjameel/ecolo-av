"""
What each Ravensim world gives the car. No ROS imports: the launch files and tests read it.

The worlds are SDF files (worlds/*.sdf, model:// from models/), the format Gazebo read
before it was removed (16 Nov). A world renders the car's cameras, roof lidar and radar
stand-in only if it says so: <plugin name="ravensim::sensors"/>. Today worlds/road.sdf,
worlds/test_lot.sdf and the campus do. The empty world renders nothing, and runs
anywhere.
"""

from __future__ import annotations

import os
from typing import Tuple
from xml.etree import ElementTree

import yaml

SENSORS = 'ravensim::sensors'
# A campus.sdf made before 16 Nov still loads Gazebo's Sensors system instead: the same.
OLD_SENSORS = 'gz::sim::systems::Sensors'

# The bench's obstacle (aav_bench/config/bench.yaml mock_radar: boxes 15-16 m east, -0.5 to
# 0.5 m north): where obstacle:=true puts models/obstacle, the centre of its base (m).
OBSTACLE: Tuple[float, float] = (15.5, 0.0)

# Carleton: too big for git, so tools/campus_world.py makes it (pixi run campus). These paths
# are from the repo's root, where pixi runs everything, like maps/ndt/test_lot.pcd.
CAMPUS = 'campus'
CAMPUS_SDF = os.path.join('maps', 'campus', 'campus.sdf')
CAMPUS_MAP = os.path.join('maps', 'campus', 'campus.yaml')
CAMPUS_PLACE = os.path.join('maps', 'campus', 'place.yaml')
CAMPUS_NDT = os.path.join('maps', 'ndt', 'campus.pcd')
# The same map in 64 m chunks (pixi run campus; TODO 175): NDT holds only the 3 x 3 round
# the car. Scored against the whole map, 15 Nov: as good, ready in 0.02 s, not 2.5 s.
CAMPUS_NDT_CHUNKS = os.path.join('maps', 'campus', 'chunks', 'ndt')
# Its occupancy map in the same chunks (chunks.yaml, fine/, coarse/): our global costmap
# holds only those round the car and along the route (costmap_chunks:=auto).
CAMPUS_CHUNKS = os.path.join('maps', 'campus', 'chunks')
# The campus's lane map: its painted roads (aav_description lane_map.py's format).
CAMPUS_LANES = os.path.join('maps', 'campus', 'lanes.yaml')
# NDT's poor-fit floor on the campus. ndt.yaml's 2.3 is the test lot's: walls close by. Here
# (5 Nov, 60 s drive at P11's road) fits scored 1.73-1.94 on the guess and 2.00-2.31 on a
# recorded map, and stayed within 0.12 m of the truth. Whether a WRONG pose can score this
# high isn't measured, so don't feed it to the EKF here (ndt_ekf) until it is.
CAMPUS_NDT_MIN_SCORE = 1.5
# NDT's median score per point at which it wins a disagreement with the GPS there
# (pose_source:=ndt, aav_pose_check's trust_score): NDT's own floor, as in the lot (where a
# right NDT scored 2.0-4.0 and one 5 m off 0-2.0, 19 Nov). TODO 121: a WRONG pose's score
# on the campus is still unmeasured.
CAMPUS_POSE_TRUST = CAMPUS_NDT_MIN_SCORE
# The grass is a kerb up, 0.30 m (Ben, 5 Nov), right on the car's usual 0.3 m floor, so half of
# every hit on it would count as an obstacle, or as a wall for NDT. NDT looks above it. The
# screen's cubes look below it, so the kerb and the grass show as the lidar's rings.
CAMPUS_NDT_FLOOR = 0.45
# m/s: the campus's roads are long. The car's own limit stays 1 m/s; this is the sim only.
CAMPUS_SPEED = '2.5'
CAMPUS_CUBE_FLOOR = 0.12

# Who plays the world (simulator:=): Ravensim, our own simulator (aav_simulator, Rust), the
# only one since Gazebo was removed (16 Nov). It casts the lidar's and radar's rays; its
# cameras are their own program (aav_sim_cameras, Bevy), drawing the same world file.
# simulator:=own, its name before 16 Nov, went on 20 Nov (TODO 241).
RAVENSIM = 'ravensim'
SIMULATORS = (RAVENSIM,)
DEFAULT_SIMULATOR = RAVENSIM


def is_world_file(world: str) -> bool:
    """Return whether world:= names a world file (a twin: tools/twin_world.py), not a name."""
    return world.endswith('.sdf')


def twin(world: str) -> dict:
    """
    Return a twin world's files, from the twin.yaml tools/twin_world.py writes beside it.

    {'map': its 2D map's YAML, 'ndt': NDT's points or '', 'start': (x, y, yaw)}.
    """
    folder = os.path.dirname(os.path.abspath(world))
    place = os.path.join(folder, 'twin.yaml')
    start = (0.0, 0.0, 0.0)
    if os.path.isfile(place):
        with open(place) as f:
            s = (yaml.safe_load(f) or {}).get('start') or {}
        start = (float(s.get('x', 0.0)), float(s.get('y', 0.0)), float(s.get('yaw', 0.0)))
    ndt = os.path.join(folder, 'ndt.pcd')
    return {'map': os.path.join(folder, 'map2d.yaml'),
            'ndt': ndt if os.path.isfile(ndt) else '', 'start': start}


def world_file(share: str, world: str) -> str:
    """
    Return the path of a world in aav_sim_models's share folder, or say which ones exist.

    A path ending .sdf (a twin from tools/twin_world.py, TODO 253) is taken as it is.
    """
    if is_world_file(world):
        path = os.path.abspath(world)
        if not os.path.isfile(path):
            raise ValueError(f'world:={world}: no such file. Make a twin with '
                             '`pixi run twin-world <map>`')
        return path
    if world == CAMPUS:
        path = os.path.abspath(CAMPUS_SDF)
        if not os.path.isfile(path):
            raise ValueError(f'world:={CAMPUS}: no {CAMPUS_SDF}. Make it with `pixi run campus`')
        return path
    folder = os.path.join(share, 'worlds')
    path = os.path.join(folder, f'{world}.sdf')
    if not os.path.isfile(path):
        known = sorted(f[:-len('.sdf')] for f in os.listdir(folder) if f.endswith('.sdf'))
        raise ValueError(f'world:={world}: no such world. Pick one of {known}')
    return path


def lane_map_file(share: str, world: str, given: str) -> str:
    """
    Return the lane map the behaviour planner reads (sim.launch.py lane_map:=), or ''.

    given: '' (none: the road ahead follows /plan), 'world' (this world's own:
    worlds/<world>.lanes.yaml, or the campus's maps/campus/lanes.yaml), or a file's path.
    """
    if not given:
        return ''
    if given != 'world':
        path = os.path.abspath(given)
    elif world == CAMPUS:
        path = os.path.abspath(CAMPUS_LANES)
    else:
        path = os.path.join(share, 'worlds', f'{world}.lanes.yaml')
    if not os.path.isfile(path):
        hint = ' Make it with `pixi run campus`' if world == CAMPUS and given == 'world' else ''
        raise ValueError(f'lane_map:={given}: no lane map at {path}.{hint}')
    return path


def campus_start(place_text: str) -> Tuple[float, float, float]:
    """Return where place.yaml says the car starts on the campus: x, y (m), yaw (rad)."""
    start = yaml.safe_load(place_text)['start']
    return float(start['x']), float(start['y']), float(start['yaw'])


def check_simulator(name: str) -> str:
    """Return the simulator's name, checked: one of SIMULATORS."""
    if name not in SIMULATORS:
        gone = {'gazebo': ' (Gazebo was removed on 16 Nov)',
                'own': " (own is Ravensim's old name, gone on 20 Nov)"}.get(name, '')
        raise ValueError(f'simulator:={name}: pick one of {", ".join(SIMULATORS)}{gone}')
    return name


def has_cameras(path: str, simulator: str = RAVENSIM) -> bool:
    """Return True if this world renders cameras: where it renders sensors."""
    check_simulator(simulator)
    return has_sensors(path)


# Set by tools/software_gl.sh: the cameras draw in software (Mesa's lavapipe, no GPU).
SOFTWARE_GL_VAR = 'AAV_SOFTWARE_GL'


def camera_lockstep(value: str, environ) -> bool:
    """
    Return whether Ravensim's clock waits for each camera's picture (camera_lockstep:=).

    auto: when tools/software_gl.sh draws the cameras in software, too slow for real
    time (1.6 Hz on CI's runners, free-running); true or false: either way.
    """
    value = value.lower()
    if value == 'auto':
        return environ.get(SOFTWARE_GL_VAR, '') == '1'
    if value in ('true', 'false'):
        return value == 'true'
    raise ValueError(f'camera_lockstep:={value}: auto, true or false')


GRAPHICS = ('auto', 'simple', 'pretty')


def graphics(value: str) -> str:
    """
    Return how much Ravensim's cameras draw (graphics:=): auto, simple or pretty.

    pretty: grain on the ground, a sky with haze, longer shadows (TODO 244); simple: plain
    colours and a flat sky, as Gazebo drew. auto: pretty on a GPU, simple drawn in software
    (tools/software_gl.sh, CI), where every extra costs real time. The sensors' numbers
    don't change: only the pictures' looks.
    """
    value = value.lower()
    if value not in GRAPHICS:
        raise ValueError(f'graphics:={value}: auto, simple or pretty')
    return value


# Ravensim's cameras: their own Cargo workspace, not a colcon package (Bevy).
SIM_CAMERAS = os.path.join('ros', 'sim', 'aav_sim_cameras', 'Cargo.toml')


def repo_root(here: str) -> str:
    """
    Return the repository's root: the folder pixi runs in, or else where `here` really is.

    here is a file in the repository, as colcon --symlink-install installs it (a link
    back to the source).
    """
    if os.path.isfile(SIM_CAMERAS):
        return os.getcwd()
    folder = os.path.dirname(os.path.realpath(here))
    while folder != os.path.dirname(folder):
        if os.path.isfile(os.path.join(folder, SIM_CAMERAS)):
            return folder
        folder = os.path.dirname(folder)
    raise ValueError(f'Ravensim: no {SIM_CAMERAS} above {here}: run from the repository')


# Set by tools/software_gl.sh: more cargo features for the cameras (vulkan-on-mac, to draw
# with Mesa's lavapipe on a Mac), and the folder holding the Vulkan loader on a Mac.
FEATURES_VAR = 'AAV_SIM_CAMERAS_FEATURES'
VULKAN_LIB_VAR = 'AAV_VULKAN_LIB_DIR'


def sim_cameras_features(environ) -> str:
    """Return the cameras' cargo features: ros, and any software_gl.sh adds."""
    extra = [f for f in environ.get(FEATURES_VAR, '').split(',') if f]
    return ','.join(['ros'] + extra)


def sim_cameras_env(environ, system: str) -> dict:
    """
    Return what the cameras' process needs added to its environment (none on a GPU).

    On a Mac, the Vulkan loader for the software renderer is found through
    DYLD_LIBRARY_PATH, which macOS takes out of any program started through /usr/bin/env
    (every tools/*.sh): tools/software_gl.sh gives the folder another name, and it comes
    back here, in front of ROS's own (the cameras' message libraries). On Linux it stays in
    LD_LIBRARY_PATH: nothing to add.
    """
    folder = environ.get(VULKAN_LIB_VAR, '')
    if system != 'Darwin' or not folder:
        return {}
    ros = environ.get('DYLD_LIBRARY_PATH', '')
    return {'DYLD_LIBRARY_PATH': folder + (':' + ros if ros else '')}


def sim_cameras_command(root: str, world: str, models, chase: bool = True,
                        relay: str = '', looks: str = 'auto') -> list:
    """
    Return the command that runs Ravensim's cameras beside it.

    They draw world (an SDF file, model:// from models) where /vehicle/sim/truth puts the
    car, on the car's camera topics. cargo builds them first if they changed (Bevy: minutes
    the first time). chase=False leaves out the chase camera. relay: the front left
    picture's topic instead of its own (its camera_info stays), for sim_camera_relay's faults.
    looks: graphics:= (auto, simple or pretty).
    """
    cmd = ['cargo', 'run', '--release', '-q', '--features', sim_cameras_features(os.environ),
           '--manifest-path', os.path.join(root, SIM_CAMERAS), '--',
           '--ros', '--pose', 'truth', '--world', world, '--models', ':'.join(models)]
    if not chase:
        cmd += ['--cameras', 'front_left,front_right,rear']
    if relay:
        cmd += ['--image-topic', 'front_left=' + relay]
    return cmd + ['--graphics', graphics(looks)]


def has_sensors(path: str) -> bool:
    """Return True if the world says it renders sensors: cameras, the lidar and the radar."""
    root = ElementTree.parse(path).getroot()
    return any(p.get('name') in (SENSORS, OLD_SENSORS) for p in root.iter('plugin'))
