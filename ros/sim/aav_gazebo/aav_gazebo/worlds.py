"""
What each Gazebo world gives the car. No ROS imports: the launch files and tests read it.

A world renders the car's camera and radar stand-in (aav_description's
aav.sensors.gazebo.xacro) only if it loads Gazebo's Sensors system, which needs a GPU,
or ogre2 on a Mac. So the world file itself says whether the car has them: today
worlds/road.sdf, worlds/test_lot.sdf and the campus do. The empty world renders nothing, and
runs headless anywhere.
"""

from __future__ import annotations

import os
from typing import Tuple
from xml.etree import ElementTree

import yaml

SENSORS = 'gz::sim::systems::Sensors'

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
# The campus's lane map: its painted roads (aav_description lane_map.py's format).
CAMPUS_LANES = os.path.join('maps', 'campus', 'lanes.yaml')
# NDT's poor-fit floor on the campus. ndt.yaml's 2.3 is the test lot's: walls close by. Here
# (5 Nov, 60 s drive at P11's road) fits scored 1.73-1.94 on the guess and 2.00-2.31 on a
# recorded map, and stayed within 0.12 m of the truth. Whether a WRONG pose can score this
# high isn't measured, so don't feed it to the EKF here (ndt_ekf) until it is.
CAMPUS_NDT_MIN_SCORE = 1.5
# The grass is a kerb up, 0.30 m (Ben, 5 Nov), right on the car's usual 0.3 m floor, so half of
# every hit on it would count as an obstacle, or as a wall for NDT. NDT looks above it. The
# screen's cubes look below it, so the kerb and the grass show as the lidar's rings.
CAMPUS_NDT_FLOOR = 0.45
# m/s: the campus's roads are long. The car's own limit stays 1 m/s; this is Gazebo only.
CAMPUS_SPEED = '2.5'
CAMPUS_CUBE_FLOOR = 0.12

# Who plays the world (simulator:=, docs/own-simulator.md): our own Rust simulator
# (aav_simulator), a drop-in for gz sim and its bridges, the default since 8 Nov (it passed
# the same checks as Gazebo); or Gazebo (simulator:=gazebo), kept for comparison and for a
# computer with no GPU (CI, oirp-spss), where our cameras can't draw but Mesa's
# software GL draws Gazebo's (tools/software_gl.sh). Ours casts the lidar's and radar's
# rays; its cameras are their own program (aav_sim_cameras, Bevy), drawing the same world file.
SIMULATORS = ('gazebo', 'own')
DEFAULT_SIMULATOR = 'own'


def world_file(share: str, world: str) -> str:
    """Return the path of a world in aav_gazebo's share folder, or say which ones exist."""
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
        raise ValueError(f'simulator:={name}: pick one of {", ".join(SIMULATORS)}')
    return name


def has_cameras(path: str, simulator: str) -> bool:
    """Return True if this world renders cameras: where it renders sensors, either simulator."""
    check_simulator(simulator)
    return has_sensors(path)


# Our simulator's cameras: their own Cargo workspace, not a colcon package (Bevy).
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
    raise ValueError(f'simulator:=own: no {SIM_CAMERAS} above {here}: run from the repository')


def sim_cameras_command(root: str, world: str, models, chase: bool = True,
                        relay: str = '') -> list:
    """
    Return the command that runs our simulator's cameras beside it (simulator:=own).

    They draw world (an SDF file, model:// from models) where /vehicle/sim/truth puts the
    car, on Gazebo's camera topics. cargo builds them first if they changed (Bevy: minutes
    the first time). chase=False leaves out the chase camera. relay: the front left
    picture's topic instead of its own (its camera_info stays), for gazebo_camera's faults.
    """
    cmd = ['cargo', 'run', '--release', '-q', '--features', 'ros',
           '--manifest-path', os.path.join(root, SIM_CAMERAS), '--',
           '--ros', '--pose', 'truth', '--world', world, '--models', ':'.join(models)]
    if not chase:
        cmd += ['--cameras', 'front_left,front_right,rear']
    if relay:
        cmd += ['--image-topic', 'front_left=' + relay]
    return cmd


def has_sensors(path: str) -> bool:
    """Return True if the world loads the Sensors system, so it renders cameras and lidars."""
    root = ElementTree.parse(path).getroot()
    return any(p.get('name') == SENSORS for p in root.iter('plugin'))
