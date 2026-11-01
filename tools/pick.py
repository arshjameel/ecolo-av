#!/usr/bin/env python3
"""
Pick the simulated car's switches from a menu, so nobody has to remember them.

Purpose: Ben, 29 Oct: "Like just tacking them on the command line, I am going to forget
them tomorrow." Each question lists its choices in plain words; the default is first.
It prints the whole command (to learn it, or paste it), saves it, and runs it.

    pixi run pick            # the questions, then run
    pixi run pick last       # run the last pick again
    pixi run pick list       # every switch and its choices, then stop

Every switch here is a launch argument of sim.launch.py (tools/tests/test_pick.py checks).
The cheat sheet: docs/switches.md. Standard library only.
"""
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parent.parent
LAST = Path.home() / '.config' / 'aav' / 'last-pick'
NDT_MAP = 'maps/ndt/test_lot.pcd'

# What to start: the pixi task, and what it is.
PROGRAMS = [
    ('debugdemo', 'Gazebo, RViz, the HUD, the map window and the debug HUD (cubes, NDT)'),
    ('demo', 'Gazebo, RViz, the HUD and the map window'),
    ('sim', 'Gazebo and the car only (add windows yourself)'),
]

# Who steers along Nav2's path: Nav2's own controller, or one of our trial trackers.
# (value, plain words, the launch arguments it sets)
FOLLOWERS = [
    ('mppi', "Nav2's MPPI: tries 1,000 wiggles, keeps the good ones (default)",
     ['controller:=mppi', 'tracker:=none']),
    ('rpp', "Nav2's Regulated Pure Pursuit: steer at a point ahead, slow in bends",
     ['controller:=rpp', 'tracker:=none']),
    ('pure_pursuit', 'ours: steer at a point ahead (finished 12 of 12 legs)',
     ['tracker:=pure_pursuit', 'reversing:=false']),
    ('stanley', 'ours: fix the distance off the path and the heading (12 of 12)',
     ['tracker:=stanley', 'reversing:=false']),
    ('lqr', 'ours: a formula for the best steering each moment',
     ['tracker:=lqr', 'reversing:=false']),
    ('mpc', 'ours: try the next 1.5 s of steering, keep the best (12 of 12)',
     ['tracker:=mpc', 'reversing:=false']),
    ('mpc_full', 'ours: MPC that plans the speed too',
     ['tracker:=mpc_full', 'reversing:=false']),
]

# Who draws the route: Nav2's planner (planner:=). Gazebo, 29 Oct, the lot tour, MPPI.
PLANNERS = [
    ('hybrid', 'Hybrid-A*: routes a car can drive (default; 4 of 4 legs)', ['planner:=hybrid']),
    ('lattice', "State Lattice: our car's own moves, fast (2 of 4: lost the U-turn)",
     ['planner:=lattice']),
    ('smac2d', 'grid A*: ignores the turning circle (2 of 4)', ['planner:=smac2d']),
    ('navfn', 'NavFn, the classic: ignores the turning circle (not tried in Gazebo)',
     ['planner:=navfn']),
    ('theta_star', 'Theta*: straight lines, ignores the turning circle (not tried)',
     ['planner:=theta_star']),
]

# Where-am-I finders that run beside the EKF and are scored (they never steer yet).
# Every row sets each switch, on or off: debugdemo turns some on, and the last value wins.
LOCALIZERS = [
    ('ekf', 'the EKFs only: wheels, IMU and GPS (default)', ['ndt:=false', 'amcl:=false']),
    ('ndt', 'add NDT: the lidar matched to a saved map (1-3 cm)',
     ['ndt:=true', f'ndt_map:={NDT_MAP}', 'amcl:=false']),
    ('amcl', 'add AMCL: guesses that match the walls (6-11 cm)', ['ndt:=false', 'amcl:=true']),
    ('both', 'add NDT and AMCL', ['ndt:=true', f'ndt_map:={NDT_MAP}', 'amcl:=true']),
]

SPEED_LOOPS = [
    ('default', "today's speed loop (PI)", ['speed_loop:=default']),
    ('lqi', 'LQI: no overshoot, settles in 4-5 s', ['speed_loop:=lqi']),
    ('smooth', 'the PI with a smooth stop', ['speed_loop:=smooth']),
]

# Only asked for `sim`: demo and debugdemo always use the test lot.
WORLDS = [
    ('test_lot', 'the walled lot with a building (NDT and AMCL need it)', ['world:=test_lot']),
    ('road', 'a road with a stop sign', ['world:=road']),
    ('empty', 'an empty plane', []),
]


def ask(title, choices, read=input, say=print):
    """Ask one question; Enter takes the first choice. Return the chosen row."""
    say(f'\n{title}')
    for i, row in enumerate(choices, 1):
        say(f'  {i}  {row[0]:<13} {row[1]}')
    while True:
        answer = read('  number [1]: ').strip()
        if not answer:
            return choices[0]
        if answer.isdigit() and 1 <= int(answer) <= len(choices):
            return choices[int(answer) - 1]
        say(f'  type 1 to {len(choices)}, or Enter for 1')


def command(program, rows):
    """Return the pixi command for a program and the chosen rows' launch arguments."""
    args = []
    for row in rows:
        for arg in row[2]:
            if arg not in args:
                args.append(arg)
    return ['pixi', 'run', program] + args


def questions(read=input, say=print):
    """Ask every question; return the command to run."""
    program = ask('What to start?', PROGRAMS, read, say)[0]
    rows = []
    if program == 'sim':
        rows.append(ask('Which world?', WORLDS, read, say))
    rows.append(ask('Who draws the route?', PLANNERS, read, say))
    rows.append(ask('Who steers along the path?', FOLLOWERS, read, say))
    where = ask('Where-am-I finders beside the EKF?', LOCALIZERS, read, say)
    if 'ndt:=true' in where[2] and not (ROOT / NDT_MAP).is_file():
        say(f'  note: no {NDT_MAP} yet: make it once with pixi run ndt-map drive')
    if program == 'sim' and where[0] != 'ekf' and rows[0][0] != 'test_lot':
        say('  note: NDT and AMCL need the test lot; switching the world to it')
        rows[0] = WORLDS[0]
    rows.append(where)
    rows.append(ask('Which speed loop?', SPEED_LOOPS, read, say))
    return command(program, rows)


def listing():
    """Return every question and its choices, for `pick list` and the docs."""
    lines = []
    for title, choices in [('What to start', PROGRAMS), ('World (sim only)', WORLDS),
                           ('Who draws the route', PLANNERS),
                           ('Who steers', FOLLOWERS), ('Where am I', LOCALIZERS),
                           ('Speed loop', SPEED_LOOPS)]:
        lines.append(title)
        for row in choices:
            sets = ' '.join(row[2]) if len(row) > 2 else f'pixi run {row[0]}'
            lines.append(f'  {row[0]:<13} {sets:<42} {row[1]}')
    return '\n'.join(lines)


def main(argv):
    """Ask, or rerun the last pick, or list; then run the command."""
    if argv[1:] == ['list']:
        print(listing())
        return 0
    if argv[1:] == ['last']:
        if not LAST.is_file():
            print('no last pick yet: run pixi run pick', file=sys.stderr)
            return 1
        cmd = shlex.split(LAST.read_text())
    elif len(argv) == 1:
        cmd = questions()
        LAST.parent.mkdir(parents=True, exist_ok=True)
        LAST.write_text(shlex.join(cmd) + '\n')
    else:
        print(__doc__.strip().split('\n\n')[1], file=sys.stderr)
        return 2
    print(f'\nRunning:  {shlex.join(cmd)}\n(saved: pixi run pick last runs it again)\n')
    os.chdir(ROOT)
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
