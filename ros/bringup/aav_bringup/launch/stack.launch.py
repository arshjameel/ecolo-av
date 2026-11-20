"""
One launch for every target: target:= picks who plays the car, and so the profile.

  ros2 launch aav_bringup stack.launch.py target:=turtle|ravensim|bench|car|carla
      [source:=]  [any argument of that target's own file, such as world:=road, machine:=orin]

  target   starts             source (where the sensor data comes from)
  turtle   turtle.launch.py   sim
  ravensim sim.launch.py      sim
  bench    bench.launch.py    bench
  car      car.launch.py      real
  carla    carla.launch.py    sim

source:= may only say the target's own: the real car only ever runs on real data, and a
simulator never drives on real sensors. The sensor mux and the output mux (aav_io_mux)
check the same while it runs, so a wrong mix that slips past here still can't move the
car. The `pixi run` tasks (turtle, sim, bench, car) start the same
files directly.
"""

import os

from aav_io_mux.provenance import choose
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PROFILES = {'turtle': 'turtle.launch.py', 'ravensim': 'sim.launch.py',
            'bench': 'bench.launch.py', 'car': 'car.launch.py', 'carla': 'carla.launch.py'}
# target:= here names the program; the output mux only knows the job: any simulator is `sim`.
MUX_TARGET = {'ravensim': 'sim'}


def launch_setup(context):
    target = LaunchConfiguration('target').perform(context).strip()
    if target not in PROFILES:
        raise ValueError(f'target:={target}: use one of {", ".join(PROFILES)}')
    choose(MUX_TARGET.get(target, target),
           LaunchConfiguration('source').perform(context).strip())
    path = os.path.join(get_package_share_directory('aav_bringup'), 'launch', PROFILES[target])
    # The rest of the command line's arguments reach the profile as they are: launch
    # arguments are shared with an included file.
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', description='turtle, ravensim, bench, car or carla'),
        DeclareLaunchArgument('source', default_value='',
                              description="real, sim, bench: only the target's own (empty: "
                                          'that one)'),
        OpaqueFunction(function=launch_setup),
    ])
