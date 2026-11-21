#!/usr/bin/env python3
"""
Graph the actuator tests' runs, the sim's and the car's drawn over each other.

Ben, 20 Nov: "baseline testing like the step function stuff ... So we can do the same
tests on the car itself and see the difference ... and graph the results."

Every `actuator-test` run (alone, or inside `car-test --stands` / `--hallway`) leaves a
folder with `log.csv` (reports.COLUMNS). This finds those folders under each path given,
groups them by test (steer-rate, wheel-steps, brake...), and draws one figure per test:
what was asked, what ego sent, what the car did (speed, steering, throttle and brake),
and the IMU's forward acceleration. Each path is one line style, labelled by --label.

usage:
    pixi run python3 tools/step_plots.py <sim session> --label sim --out <folder>
    pixi run python3 tools/step_plots.py <sim> <car> --label sim car --out <folder>
    pixi run python3 tools/step_plots.py --hallway <car-test --hallway session> --out <folder>

--hallway draws each pass as in a system-identification talk: the step that went in, what
the car did, and the car model's answer to the same step, with the fitted numbers (the
best-matching curve) and with vehicle.yaml's (what the simulator and the planners assume).

Only reads the runs; writes PNGs into --out. matplotlib (in the default environment).
"""

import argparse
import csv
import math
import os
import re
import sys

# The run folder's name ends in the test's name, then -sim / -bench / nothing (the car).
RUN = re.compile(r'^\d{4}-\d\d-\d\d-\d{6}-(?P<test>.+?)(?:-(?:sim|bench|stands))?$')

# Each panel: its title, its unit, and the columns drawn on it (column, its words).
PANELS = (
    ('Speed', 'm/s', (('cmd_speed', 'asked'), ('ego_speed', 'ego sent'),
                      ('speed', 'wheels'))),
    ('Steering', 'rad', (('cmd_steer', 'asked'), ('steer_cmd', 'ego sent'),
                         ('steer', 'measured'))),
    ('Pedals', '0..1', (('throttle', 'throttle'), ('brake', 'brake'))),
    ('IMU', 'm/s²', (('ax', 'forward accel'),)),
    ('Distance', 'm', (('distance', 'by the wheels'),)),
)
STYLES = ('-', '--', ':', '-.')
# Columns whose readings shake (the IMU): drawn faint, with a running mean over them.
NOISY = ('ax',)


def smooth(times, values, window):
    """Average the readings within window/2 seconds each side (NaN ones left out)."""
    out, lo, hi, total, count = [], 0, 0, 0.0, 0
    for t in times:
        while hi < len(times) and times[hi] <= t + window / 2:
            if not math.isnan(values[hi]):
                total, count = total + values[hi], count + 1
            hi += 1
        while times[lo] < t - window / 2:
            if not math.isnan(values[lo]):
                total, count = total - values[lo], count - 1
            lo += 1
        out.append(total / count if count else math.nan)
    return out


def number(text):
    """Read a CSV cell as a float, or NaN when it is empty or not a number."""
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def read_run(folder):
    """Read one run's log.csv as columns of floats (the phase column kept as text)."""
    with open(os.path.join(folder, 'log.csv'), newline='') as f:
        rows = list(csv.DictReader(f))
    columns = {}
    for name in rows[0] if rows else ():
        if name == 'phase':
            columns[name] = [r[name] for r in rows]
        else:
            columns[name] = [number(r[name]) for r in rows]
    return columns


def find_runs(root):
    """Find each run folder under root: {test name: [folder, ...]}, oldest first."""
    found = {}
    for here, _, files in sorted(os.walk(os.path.expanduser(root))):
        if 'log.csv' not in files:
            continue
        match = RUN.match(os.path.basename(here))
        test = match.group('test') if match else os.path.basename(here)
        found.setdefault(test, []).append(here)
    return found


def flat(values):
    """Say whether a column says nothing: every value NaN, or all the same."""
    real = [v for v in values if not math.isnan(v)]
    return not real or max(real) - min(real) < 1e-9


def plot_test(plt, test, runs, out):
    """Draw one figure for one test: a panel per quantity, every run over the others."""
    panels = [p for p in PANELS
              if any(not flat(cols.get(c, [math.nan])) for _, cols in runs for c, _ in p[2])]
    if not panels:
        return None
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 2.6 * len(panels)), sharex=True,
                             squeeze=False)
    for ax, (title, unit, columns) in zip(axes[:, 0], panels):
        for (label, cols), style in zip(runs, STYLES):
            for i, (column, words) in enumerate(columns):
                values = cols.get(column)
                if values is None or flat(values):
                    continue
                name = f'{words} ({label})' if len(runs) > 1 else words
                if column in NOISY:
                    # The raw readings faint, a 0.5 s running mean over them.
                    ax.plot(cols['t'], values, linestyle=style, color=f'C{i}', alpha=0.25,
                            linewidth=0.8)
                    ax.plot(cols['t'], smooth(cols['t'], values, 0.5), linestyle=style,
                            color=f'C{i}', label=f'{name}, 0.5 s mean')
                else:
                    ax.plot(cols['t'], values, linestyle=style, color=f'C{i}', label=name)
        ax.set_ylabel(f'{title} ({unit})')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
    axes[-1, 0].set_xlabel('time (s)')
    fig.suptitle(test)
    fig.tight_layout()
    path = os.path.join(out, f'{test}.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


OVERLAY = (('input', 'input: what ego sent', 'k', ':'),
           ('measured', 'what happened (measured)', 'C0', '-'),
           ('wheels', 'the wheels', 'C0', '--'),
           ('model_fitted', 'model, fitted (best match)', 'C3', '-'),
           ('model_vehicle_yaml', 'model, vehicle.yaml (the sim)', 'C2', '-.'))


def read_csv(path):
    """Read a CSV file as columns of floats."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    return {k: [number(r[k]) for r in rows] for k in (rows[0] if rows else ())}


def hallway_passes(session):
    """Find each pass of a hallway session: (name, its overlay CSV, its run's log.csv)."""
    folder = os.path.expanduser(session)
    overlays = os.path.join(folder, 'overlays')
    found = []
    for name in sorted(os.listdir(overlays)) if os.path.isdir(overlays) else ():
        if not name.endswith('.csv'):
            continue
        stem = name[:-len('.csv')]
        logs = [here
                for here, _, files in os.walk(os.path.join(folder, stem)) if 'log.csv' in files]
        found.append((stem, os.path.join(overlays, name), logs[0] if logs else None))
    return found


def plot_pass(plt, stem, overlay, log, out):
    """
    Draw one pass: the input, what happened, and the model's answers to the same input.

    The overlay is always the speed (the hallway's model replays the pedals). A steering
    pass adds a second panel from its run's log: the steering asked against the measured.
    """
    cols = read_csv(overlay)
    run = read_run(log) if log else {}
    if run:
        by_time = dict(zip(run['t'], run.get('ego_speed', [])))
        cols['input'] = [by_time.get(t, math.nan) for t in cols['t']]
    steering = 'steer' in stem and not flat(run.get('steer', [math.nan]))
    fig, axes = plt.subplots(2 if steering else 1, 1, figsize=(10, 6.5 if steering else 4),
                             sharex=True, squeeze=False)
    ax = axes[0, 0]
    for column, words, colour, style in OVERLAY:
        values = cols.get(column)
        if values is None or flat(values):
            continue
        ax.plot(cols['t'], values, color=colour, linestyle=style, label=words)
    ax.set_ylabel('speed (m/s)')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    if steering:
        low = axes[1, 0]
        low.plot(run['t'], run['steer_cmd'], color='k', linestyle=':',
                 label='input: the steering ego sent')
        low.plot(run['t'], run['steer'], color='C0', label='what happened (measured)')
        low.set_ylabel('steering (rad)')
        low.grid(alpha=0.3)
        low.legend(loc='best', fontsize=8)
    axes[-1, 0].set_xlabel('time (s)')
    fig.suptitle(stem)
    fig.tight_layout()
    path = os.path.join(out, f'{stem}.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('paths', nargs='*', help='sessions or run folders, one per line style')
    parser.add_argument('--hallway', default=None,
                        help='a car-test --hallway session: each pass, input, measured, models')
    parser.add_argument('--label', nargs='+', default=None,
                        help='a name for each path (default: its folder name)')
    parser.add_argument('--out', required=True, help='where the PNGs go')
    args = parser.parse_args(argv)
    if args.hallway:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        passes = hallway_passes(args.hallway)
        if not passes:
            print(f'no overlays/ in {args.hallway}')
            return 1
        os.makedirs(args.out, exist_ok=True)
        for stem, overlay, log in passes:
            print(f'{stem}: {plot_pass(plt, stem, overlay, log, args.out)}')
        return 0
    if not args.paths:
        parser.error('give sessions to draw, or --hallway SESSION')
    labels = args.label or [os.path.basename(os.path.normpath(p)) for p in args.paths]
    if len(labels) != len(args.paths):
        parser.error('give one --label per path')
    if len(args.paths) > len(STYLES):
        parser.error(f'at most {len(STYLES)} paths')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    tests = {}
    for path, label in zip(args.paths, labels):
        for test, folders in find_runs(path).items():
            # The newest run of each test stands for that path.
            tests.setdefault(test, []).append((label, read_run(folders[-1])))
    if not tests:
        print('no runs (folders with log.csv) under ' + ', '.join(args.paths))
        return 1
    os.makedirs(args.out, exist_ok=True)
    for test, runs in sorted(tests.items()):
        path = plot_test(plt, test, runs, args.out)
        print(f'{test}: {path or "nothing that moved"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
