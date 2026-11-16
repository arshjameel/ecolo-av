"""
Every node of our own in navigation.launch.py comes back after it dies.

Nav2's servers are lifecycle nodes with a bond; ours are plain nodes, so the launch file
respawns them (audit 2026-11-14, "No lifecycle, bond or respawn"). No ROS needed: the
launch file is read as text.
"""

from pathlib import Path
import re

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'navigation.launch.py'
# Ours, in Nav2's place: the costmaps, the planner, the navigator, the smoother, the
# followers, and the behaviour planner (its package by behaviour_impl:=, Python or Rust).
OURS = ('aav_costmap', 'aav_planner', 'aav_navigator', 'aav_smoother', 'aav_mppi_lag',
        'aav_track', 'BEHAVIOUR_IMPLS')


def node_calls(text):
    """Return each Node(...) call's text, by its package."""
    calls = {}
    for match in re.finditer(r"Node\(\s*package=(?:'(\w+)'|vehicle\.(\w+)\[)", text):
        depth, i = 0, match.start() + len('Node')
        while True:
            depth += {'(': 1, ')': -1}.get(text[i], 0)
            i += 1
            if depth == 0:
                break
        calls.setdefault(match.group(1) or match.group(2), []).append(text[match.start():i])
    return calls


def test_each_of_our_nodes_respawns():
    calls = node_calls(LAUNCH.read_text())
    for package in OURS:
        assert package in calls, f'{package} is not started by navigation.launch.py'
        for call in calls[package]:
            assert 'respawn=True' in call, f'{package} does not respawn'
