"""
The league: config/stacks.yaml's levels and named stacks, as a launch file's switches.

No ROS (only config_path() asks ament_index where the installed file is). The design:
docs/designs/2026-11-13_one-shape.md, "The league"; the page: docs/league.md.

    stacks.yaml ──▶ load ──▶ switches('ours') ──▶ {'costmap': 'own', 'tracker': 'mppi_lag', ...}
                                    │
    sim.launch.py's arguments ──────┴──▶ resolve ──▶ what the launch uses: a switch given
                                                     on the command line wins over the stack

The stack's switches are declared empty (CHOSEN) in sim.launch.py, so a value given by
hand is never mistaken for "not given": `costmap:=nav2` wins over `stack:=ours` too.

One switch per level (one shape, "From 36 switches to 8"): `pose:=`, `path:=`, `steer:=`
and `lanes:=` (LEVEL_SWITCHES) each pick that level's option by its league name, over the
stack's pick for it: `path:=hybrid_rs` sets planner and planner_search, and puts back the
fallback of every other switch the level's options set (plan_smoothing, lattice_moves).
A switch given by hand still wins over both. The old switches keep working.
Ours is the simulator's default (DEFAULT, Ben, 14 Nov: "Ours default now, then delete").
"""

from __future__ import annotations

import os
from typing import Dict, Mapping, Optional

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE_FILE = os.path.join(HERE, '..', 'config', 'stacks.yaml')
NONE = ''          # stack:= empty: no stack, each switch as given, else its fallback
DEFAULT = 'ours'   # sim.launch.py's stack:= when none is given (Ben, 14 Nov)
CHOSEN = ''        # a switch's declared default: "the stack's choice"
# The levels with a switch of their own, named as the level (sim.launch.py declares them).
# costmap, navigator, smoother and behaviour are switches already, by other values.
LEVEL_SWITCHES = ('pose', 'path', 'steer', 'lanes')


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


def picked(league: Mapping, stack: str,
           levels: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    """
    Return each level's option: the stack's (none without one), then `levels`' own.

    levels: a level's switch (`path:=hybrid_rs`), empty where not given.
    """
    out = choices(league, stack) if stack else {}
    for level, option in (levels or {}).items():
        option = str(option).strip()
        if option == CHOSEN:
            continue
        spec = (league.get('levels') or {}).get(level)
        if spec is None:
            raise ValueError(f'{level}:= is not a level of stacks.yaml')
        if option not in spec['options']:
            raise ValueError(f'{level}:={option}: pick one of {", ".join(spec["options"])}')
        out[level] = option
    return out


def switches(league: Mapping, stack: str,
             levels: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    """
    Return the launch switches `stack` and the `levels` given set, as text.

    A level picked by its own switch owns every switch its options set: those its option
    leaves out are absent here, so they take their fallback.
    """
    out: Dict[str, str] = {}
    for level, option in picked(league, stack, levels).items():
        for switch, value in league['levels'][level]['options'][option].items():
            out[switch] = str(value)
    for level, option in (levels or {}).items():
        if str(option).strip() == CHOSEN:
            continue
        options = league['levels'][level]['options']
        mine = options[str(option).strip()]
        for other in options.values():
            for switch in other:
                if switch not in mine:
                    out.pop(switch, None)
    return out


def resolve(league: Mapping, stack: str, given: Mapping[str, str],
            fallback: Mapping[str, str],
            levels: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    """
    Return the switches to use: each one set by hand, else the stack's, else its fallback.

    given: each switch's value as the launch has it, CHOSEN (empty) where nobody set it;
    fallback: what a switch takes when neither a hand nor the stack sets it (the
    champions' values, Nav2's today). With no stack (empty), only the hand, the levels'
    switches and the fallback count.
    levels: each level switch's value (LEVEL_SWITCHES), empty where not given.
    """
    picks = switches(league, stack, levels)
    for switch in picks:
        if switch not in fallback:
            raise ValueError(f'stacks.yaml sets {switch}:=, which this launch file '
                             'does not have')
    out = dict(given)
    for switch, value in fallback.items():
        mine = str(given.get(switch, CHOSEN)).strip()
        out[switch] = mine if mine != CHOSEN else picks.get(switch, str(value))
    return out


def describe(league: Mapping, stack: str,
             levels: Optional[Mapping[str, str]] = None) -> str:
    """Return one line for the log: 'stack:=ours - pose ekf, costmap ours, ...'."""
    own = {k: str(v).strip() for k, v in (levels or {}).items() if str(v).strip() != CHOSEN}
    mine = ', '.join(f'{level}:={option}' for level, option in own.items())
    if not stack:
        base = "no stack: every switch as given, else the champions' (Nav2's)"
        return f'{base}; {mine}' if mine else base
    picks = ', '.join(f'{level} {option}'
                      for level, option in picked(league, stack, own).items())
    line = f'stack:={stack} - {picks}'
    return f'{line} (by hand: {mine})' if mine else line
