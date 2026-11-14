"""
The league: config/stacks.yaml's levels and named stacks, as a launch file's switches.

No ROS (only config_path() asks ament_index where the installed file is). The design:
docs/designs/2026-11-13_one-shape.md, "The league"; the page: docs/league.md.

    stacks.yaml ──▶ load ──▶ switches('ours') ──▶ {'costmap': 'own', 'tracker': 'mppi_lag', ...}
                                    │
    sim.launch.py's arguments ──────┴──▶ resolve ──▶ what the launch uses: a switch given
                                                     on the command line (not its default)
                                                     wins over the stack
"""

from __future__ import annotations

import os
from typing import Dict, Mapping, Optional

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE_FILE = os.path.join(HERE, '..', 'config', 'stacks.yaml')
NONE = ''          # stack:= empty: no stack, every switch as given


def config_path() -> str:
    """Return where stacks.yaml is: installed (ament_index), else the source tree's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('aav_description'), 'config',
                            'stacks.yaml')
    except (ImportError, LookupError):
        return os.path.normpath(SOURCE_FILE)


def load(path: Optional[str] = None) -> dict:
    """Return stacks.yaml as a dict, checked (ValueError says what is wrong)."""
    with open(path or config_path()) as f:
        league = yaml.safe_load(f)
    check(league)
    return league


def check(league: Mapping) -> None:
    """Raise ValueError unless every champion and every stack's choice is an option."""
    levels = league.get('levels') or {}
    for name, level in levels.items():
        options = level.get('options') or {}
        if str(level.get('champion')) not in options:
            raise ValueError(f'stacks.yaml: level {name}: champion '
                             f'{level.get("champion")!r} is not one of its options')
    for stack, picks in (league.get('stacks') or {}).items():
        for level, option in picks.items():
            if level == 'what':
                continue
            if level not in levels:
                raise ValueError(f'stacks.yaml: stack {stack}: no level {level!r}')
            if str(option) not in levels[level]['options']:
                raise ValueError(f'stacks.yaml: stack {stack}: {level}: {option!r} is not '
                                 f'one of {", ".join(levels[level]["options"])}')


def names(league: Mapping) -> tuple:
    """Return the stacks' names, in the file's order."""
    return tuple(league.get('stacks') or {})


def choices(league: Mapping, stack: str) -> Dict[str, str]:
    """Return each level's option in `stack`: its own pick, else the level's champion."""
    stacks = league.get('stacks') or {}
    if stack not in stacks:
        raise ValueError(f'stack:={stack}: pick one of {", ".join(stacks)} '
                         '(or leave it empty)')
    picks = stacks[stack] or {}
    return {level: str(picks.get(level, spec['champion']))
            for level, spec in league['levels'].items()}


def switches(league: Mapping, stack: str) -> Dict[str, str]:
    """Return the launch switches `stack` sets, as text."""
    out: Dict[str, str] = {}
    for level, option in choices(league, stack).items():
        for switch, value in league['levels'][level]['options'][option].items():
            out[switch] = str(value)
    return out


def resolve(league: Mapping, stack: str, given: Mapping[str, str],
            defaults: Mapping[str, str]) -> Dict[str, str]:
    """
    Return the switches to use: the stack's, except where a switch was set by hand.

    given: every switch's value as the launch has it; defaults: each one's declared default.
    A switch whose value differs from its default was set by hand, and is kept. With no
    stack (empty), `given` comes back unchanged.
    """
    out = dict(given)
    if not stack:
        return out
    for switch, value in switches(league, stack).items():
        if switch not in defaults:
            raise ValueError(f'stacks.yaml sets {switch}:=, which this launch file '
                             'does not have')
        if str(given.get(switch, defaults[switch])).lower() == str(defaults[switch]).lower():
            out[switch] = value
    return out


def describe(league: Mapping, stack: str) -> str:
    """Return one line for the log: 'stack:=ours - pose ekf, costmap ours, ...'."""
    if not stack:
        return 'no stack: every switch as given'
    picks = ', '.join(f'{level} {option}' for level, option in choices(league, stack).items())
    return f'stack:={stack} - {picks}'
