#!/usr/bin/env python3
"""
A test session's report, read for a screen: tools/ladder.py's `session` and `tests`.

car-test writes a folder per session (~/aav-tests/<time>-<stands|hallway|lot>[-sim]/):
report.yaml (a step a line), and per step its run folder (log.csv, summary.txt,
meta.yaml); a hallway session adds fits.csv (each number with its error bar). This file
turns one folder into one dict, so the ladder window never opens those files itself:

    steps    name, number, status, detail, the vehicle.yaml lines it produced, its key
             numbers (the test's own table), and one graph: commanded against measured
             over time (speed, or steering for the steering tests), thinned to GRAPH_POINTS
    fits     a hallway session's numbers: mean, sd, n, trusted or why not

It also lists a rung's single tests, from the runner's own lists (car_test.STEPS,
HALL_KINDS, LOT_STEPS): never a hand-copied list.

No ROS: the actuator tests' logic modules are plain Python, read from the source tree.
Tested in tools/tests/test_ladder_report.py.
"""
import csv
import math
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros', 'drivers', 'aav_actuator_tests'))

GRAPH_POINTS = 300      # at most this many points a line: enough for a screen
NUMBERS = 12            # at most this many key numbers a step
STEERING = ('steer-sweep', 'steer-rate', 'steer-centre', 'hall-steer')
# What each graph draws: (the log's commanded column, its measured column, unit, words).
GRAPHS = {
    'speed': ('cmd_speed', 'speed', 'm/s', 'speed'),
    'steering': ('cmd_steer', 'steer', 'rad', 'steering angle'),
}


def finite(text):
    """Return a log cell as a number, or None when it is empty or not finite."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def graph_kind(test):
    """Return which graph a test gets: steering for the steering tests, else speed."""
    return 'steering' if test in STEERING else 'speed'


def graph(rows, test):
    """
    Return one graph from a run's log rows (dicts of text), or None when it has no line.

    Commanded: what the test asked for, before ego. Measured: the wheels' speed
    (/vehicle/odom) or the steering angle (/vehicle/steering). Every k-th row is kept.
    """
    kind = graph_kind(test)
    asked, got, unit, words = GRAPHS[kind]
    points = [(finite(r.get('t')), finite(r.get(asked)), finite(r.get(got))) for r in rows]
    points = [p for p in points if None not in p]
    if len(points) < 2:
        return None
    keep = max(1, math.ceil(len(points) / GRAPH_POINTS))
    points = points[::keep] + ([points[-1]] if (len(points) - 1) % keep else [])
    return {'kind': kind, 'unit': unit, 'what': words,
            't': [round(p[0], 3) for p in points],
            'commanded': [round(p[1], 4) for p in points],
            'measured': [round(p[2], 4) for p in points]}


def yaml_values(lines):
    """Return the `key: value` lines a test offers for vehicle.yaml, comments cut."""
    out = []
    for line in lines.splitlines():
        if line.startswith('  ') and ':' in line:
            out.append(line.split('#', 1)[0].strip())
    return out


def numbers(test, rows, meta):
    """Return the test's own table as rows of words (reports.analyse); [] if it can't."""
    if test == 'preflight':
        return []       # its lines are PASS / FAIL per sensor: the detail says them
    from aav_actuator_tests import reports
    try:
        rep = reports.analyse(test, reports.parse_rows(rows), reports.context_from_meta(meta))
    except Exception:   # a run cut short has too little to fit: no numbers, not no report
        return []
    return [{'what': r.what, 'value': r.value, 'unit': r.unit, 'now': r.now, 'note': r.note}
            for r in rep.rows[:NUMBERS]]


def step(folder, entry):
    """Return one step of report.yaml with what its run folder holds."""
    out = {'name': entry['step'], 'number': str(entry.get('number', '')),
           'status': entry['status'], 'detail': entry.get('detail', ''),
           'yaml': [], 'numbers': [], 'graph': None}
    run = os.path.join(folder, entry.get('files') or '')
    log, meta_path = os.path.join(run, 'log.csv'), os.path.join(run, 'meta.yaml')
    if not entry.get('files') or not (os.path.isfile(log) and os.path.isfile(meta_path)):
        return out
    with open(meta_path) as f:
        meta = yaml.safe_load(f) or {}
    with open(log, newline='') as f:
        rows = list(csv.DictReader(f))
    test = meta.get('test', '')
    out['graph'] = graph(rows, test)
    out['numbers'] = numbers(test, rows, meta)
    summary = os.path.join(run, 'summary.txt')
    if os.path.isfile(summary):
        from aav_actuator_tests import car_test
        with open(summary) as f:
            out['yaml'] = yaml_values(car_test.yaml_lines_of(f.read()))
    return out


def fits(folder):
    """Return a hallway session's fitted numbers (fits.csv), each with its error bar."""
    path = os.path.join(folder, 'fits.csv')
    if not os.path.isfile(path):
        return []
    with open(path, newline='') as f:
        return [{'name': r['number'], 'unit': r['unit'], 'value': finite(r['mean']),
                 'sd': finite(r['sd']), 'n': int(finite(r['n']) or 0),
                 'trusted': r['trusted'] == 'True', 'why': r['why'],
                 'now': finite(r['now'])} for r in csv.DictReader(f)]


def session(folder):
    """Return a session folder as one dict; `found` False (and why) when it has no report."""
    out = {'folder': folder, 'found': False, 'why': '', 'started': '', 'where': '',
           'counts': {}, 'steps': [], 'fits': []}
    path = os.path.join(folder, 'report.yaml') if folder else ''
    if not folder:
        out['why'] = 'no session yet: run it once'
        return out
    if not os.path.isfile(path):
        out['why'] = f'no report.yaml in {folder}'
        return out
    with open(path) as f:
        report = yaml.safe_load(f) or {}
    out.update(found=True, started=str(report.get('started', '')),
               where=report.get('where', ''), counts=report.get('counts', {}),
               steps=[step(folder, e) for e in report.get('steps', [])],
               fits=fits(folder))
    return out


def single_tests(kind):
    """
    Return a rung's single tests, from the runner's own lists: (name, number, title).

    kind: rungs.yaml's `tests:` (stands, hallway or lot).
    """
    from aav_actuator_tests import car_test as ct
    from aav_actuator_tests import procedures as pr
    if kind == 'stands':
        return [(s.name, s.number, s.title) for s in ct.STEPS if s.sim is ct.Manner.AUTO]
    if kind == 'hallway':
        return [(k, pr.TESTS[f'hall-{k}'].number, pr.TESTS[f'hall-{k}'].title)
                for k in ct.HALL_KINDS]
    if kind == 'lot':
        return [(s.name, s.number, s.title) for s in ct.LOT_STEPS]
    raise ValueError(f'no single tests for {kind!r}: stands, hallway or lot')


def text(report):
    """Return a session as a person reads it in a terminal."""
    if not report['found']:
        return report['why']
    lines = [f"{report['folder']}", f"{report['started']}  {report['where']}", '']
    for s in report['steps']:
        lines.append(f"  {s['status']:<8} {s['number']:<4} {s['name']:<24} {s['detail'][:90]}")
        lines += [f"           {' ' * 4} vehicle.yaml: {line}" for line in s['yaml']]
    for fit in report['fits']:
        if fit['value'] is not None:
            trust = '' if fit['trusted'] else '   NOT TRUSTED'
            lines.append(f"  {fit['name']:<20} {fit['value']:.3f} ± {fit['sd'] or 0:.3f} "
                         f"{fit['unit']} (n {fit['n']}){trust}")
    return '\n'.join(lines)
