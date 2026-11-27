"""
The simulator's named set-ups: a few of sim.launch.py's switches under one name.

No ROS. docs/wrap-up.md section 3: "sim.launch.py's switches become a short list of named
set-ups (as stacks.yaml does for algorithms)". stacks.yaml names the algorithms; a set-up
names the world around them.

    pixi run sim                    = sim.launch.py setup:=car (the virtual car)
    pixi run sim world:=test_lot    the car's set-up, in the test lot: yours wins
    ros2 launch aav_bringup sim.launch.py      no set-up: each switch's own default

| Set-up | Switches | What it is |
|---|---|---|
| (none) | world:=empty esp32:=false | the launch file's defaults, as every test starts it |
| car | world:=road esp32:=true | Ravensim as today's car: every topic the car has |
|     |                         | (pixi run check virtual) |

The switches a set-up may set are declared empty in sim.launch.py (CHOSEN), so one
given by hand is never mistaken for "not given": world:=empty wins over setup:=car.
"""

from __future__ import annotations

from typing import Dict, Mapping

NONE = ''        # setup:= empty: no set-up, each switch's own default
CHOSEN = ''      # a set-up switch's declared default: "the set-up's choice"
CAR = 'car'      # what pixi run sim passes (Ben, 25 Nov: "pixi run sim" is the virtual car)

# What each switch is with no set-up and nothing given by hand.
FALLBACK = {'world': 'empty', 'esp32': 'false'}

SETUPS = {
    CAR: {'what': "Ravensim as today's car: the road world and the simulated ESP32, so "
                  'every topic the car has (pixi run check virtual)',
          'world': 'road', 'esp32': 'true'},
}


def names() -> tuple:
    """Return the set-ups' names."""
    return tuple(SETUPS)


def switches(setup: str) -> Dict[str, str]:
    """Return the switches a set-up sets (none for NONE); ValueError for an unknown one."""
    setup = setup.strip().lower()
    if setup == NONE:
        return {}
    if setup not in SETUPS:
        raise ValueError(f'setup:={setup}: pick one of {", ".join(names())}, or none')
    return {k: v for k, v in SETUPS[setup].items() if k != 'what'}


def resolve(setup: str, given: Mapping[str, str]) -> Dict[str, str]:
    """
    Return each set-up switch's value: given by hand, else the set-up's, else FALLBACK.

    given: the switches as the launch file read them; empty (CHOSEN) means not given.
    """
    picked = switches(setup)
    return {name: (given.get(name) or CHOSEN).strip() or picked.get(name, fallback)
            for name, fallback in FALLBACK.items()}


def describe(setup: str, values: Mapping[str, str]) -> str:
    """Return one line for the launch's log: the set-up and what it gave."""
    shown = ' '.join(f'{k}:={values[k]}' for k in FALLBACK)
    if setup.strip() == NONE:
        return f'no set-up: {shown}'
    return f'setup:={setup.strip().lower()}: {shown}'
