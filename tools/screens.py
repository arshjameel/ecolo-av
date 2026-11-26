#!/usr/bin/env python3
"""
The screens, for whichever car is running (wrap-up step 5; Ben, 25 Nov: "eight become four").

    pixi run screens                    the three windows, for the car that announces itself
    pixi run screens --sim              for Ravensim on this computer (pixi run sim)
    pixi run screens --car              for the real car, on this network: listen only
    pixi run screens --only world       one window: world, hud or map
    pixi run screens --print            say what it would start, and start nothing
    Ctrl-C, or closing a window         stops every window it started

  Window      Program                      Virtual car (--sim)        Real car (--car)
  world       the world view (Bevy)        dev_window.sh --live       dev_window.sh --watch
  hud         the HUD (raylib)             hud.launch.py, here only   not opened (below)
  map         the map + HARDWARE (F2;      map.launch.py, here only   map.launch.py
              its LADDER tab: the tests)                              hardware:=false

Which car: --sim or --car; with neither, it asks /system/source (tools/jacks.py's check),
first on this computer, then on the network. SIM here: the virtual car. REAL: the real car.
Nothing heard, or a simulator on another computer: it says so and starts nothing.

With the real car it only watches:
  - the world view is --watch: no publishers at all;
  - the HUD is not opened: the sensor watchdog trusts /hud/heartbeat, the car screen's.
    A laptop's HUD would beat too, and hide a frozen car screen;
  - the map has no HARDWARE page (hardware:=false): the car's own map screen runs the
    node hw_panel, and two of them on one network would both be called hw_panel. A goal
    still needs DRIVE HERE, then CONFIRM, as on the car's screen.

Every window is in a process group of its own (tools/dry_run.py's start and stop): Ctrl-C,
or any window closing, stops them all (SIGINT, SIGTERM after 20 s, then SIGKILL) and it
says if anything is left. For the virtual car every window is on this computer only
(tools/ros.sh --local), so it never reaches a real car on the same network.

No ROS in this file: it starts programs. Its plan is tested in tools/tests/test_screens.py.
"""
import argparse
from dataclasses import dataclass
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import dry_run  # noqa: E402  (its start and stop: one way to run process groups)

WINDOWS = ('world', 'hud', 'map')
SIM, CAR = 'sim', 'car'
LOCAL = ('tools/ros.sh', '--local')
NETWORK = ('tools/ros.sh',)
HUD_ON_CAR = ('the HUD is not opened beside the real car: the sensor watchdog trusts '
              "/hud/heartbeat, and a laptop's HUD would hide a frozen car screen")


@dataclass(frozen=True)
class Window:
    name: str
    title: str
    cmd: tuple


def parse(argv):
    parser = argparse.ArgumentParser(
        prog='pixi run screens',
        description='The world view, the HUD and the map window, for whichever car runs.')
    which = parser.add_mutually_exclusive_group()
    which.add_argument('--sim', action='store_true',
                       help='the virtual car: Ravensim on this computer (pixi run sim)')
    which.add_argument('--car', action='store_true',
                       help='the real car, on this network: listen only')
    parser.add_argument('--only', choices=WINDOWS, action='append', default=[],
                        help='open this window only (again for two)')
    parser.add_argument('--print', action='store_true', dest='dry',
                        help='print the commands and start nothing')
    return parser.parse_args(argv)


def plan(car, only=()):
    """Return (the windows to start, the notes) for SIM or CAR; only: some of WINDOWS."""
    wanted = [w for w in WINDOWS if not only or w in only]
    windows, notes = [], []
    for name in wanted:
        if name == 'world':
            mode = '--live' if car == SIM else '--watch'
            windows.append(Window('world', 'the world view', ('tools/dev_window.sh', mode)))
        elif name == 'hud':
            if car == CAR:
                notes.append(HUD_ON_CAR)
                continue
            windows.append(Window('hud', 'the HUD',
                                  LOCAL + ('ros2', 'launch', 'aav_hud', 'hud.launch.py')))
        elif name == 'map':
            launch = ('ros2', 'launch', 'aav_hud', 'map.launch.py')
            windows.append(Window('map', 'the map window', LOCAL + launch if car == SIM
                                  else NETWORK + launch + ('hardware:=false',)))
    return windows, notes


def heard(where):
    """Ask tools/jacks.py what announces on /system/source; its answer (a dict) or None."""
    done = subprocess.run([*where, 'python3', 'tools/jacks.py', 'status', '--sim', '--json',
                           '--wait', '2'], cwd=ROOT, capture_output=True, text=True)
    for line in reversed(done.stdout.splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def says_real(answer):
    return bool(answer) and 'says REAL' in answer.get('message', '')


def which_car(here, network):
    """
    Return (SIM or CAR or None, why) from jacks.py's answers here and on the network.

    here: the answer on this computer only; network: on the whole network (None: not
    asked). One REAL anywhere is the real car, even beside a simulator.
    """
    if here is None:
        return None, 'tools/jacks.py gave no answer: is the workspace built (pixi run build)?'
    if says_real(here) or says_real(network):
        return CAR, 'the real car (/system/source says REAL)'
    if here.get('simulator'):
        return SIM, 'the virtual car: a simulator on this computer'
    if network and network.get('simulator'):
        return None, ('a simulator on another computer: its screens belong there. Or say '
                      '--car to watch it from here')
    return None, ('nothing announces on /system/source: start pixi run sim (or pixi run '
                  'car <computer>), or say --sim or --car')


def check_sim(here):
    """Return why --sim must not open here, or None (a real car here: never)."""
    if says_real(here):
        return 'REFUSED: --sim, but /system/source says REAL here: not a simulator'
    return None


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    if args.sim:
        car, why = SIM, 'the virtual car (--sim)'
    elif args.car:
        car, why = CAR, 'the real car (--car): listen only'
    elif args.dry:
        car, why = SIM, 'the virtual car (--print with neither --sim nor --car)'
    else:
        here = heard(LOCAL)
        network = None if here is None or here.get('simulator') else heard(NETWORK)
        car, why = which_car(here, network)
        if car is None:
            print(f'Nothing opened: {why}', file=sys.stderr)
            return 2
    if args.sim and not args.dry:
        refused = check_sim(heard(LOCAL))
        if refused:
            print(refused, file=sys.stderr)
            return 2
    windows, notes = plan(car, args.only)
    print(f'The screens for {why}:')
    for note in notes:
        print(f'  ({note})')
    for number, w in enumerate(windows, 1):
        print(f'{number}/{len(windows)}  {w.title}: {" ".join(w.cmd)}', flush=True)
    if args.dry or not windows:
        return 0 if windows or args.dry else 2

    procs, stopping = [], []
    old = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    signal.signal(signal.SIGINT, lambda *_: stopping.append('Ctrl-C'))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append('asked to stop'))
    env = dict(os.environ)
    try:
        for w in windows:
            if stopping:
                break
            part = dry_run.Part(w.title, w.cmd, needed=True)
            procs.append((w.title, dry_run.start(part, env)))
        if not stopping:
            print('Ctrl-C here, or close any window, to stop them all.', flush=True)
        while not stopping:
            ended = [name for name, p in procs if p.poll() is not None]
            if ended:
                stopping.append(f'{ended[0]} ended')
            time.sleep(0.5)
        print(f'\n{stopping[0]}: stopping every window ...', flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        left = dry_run.stop(procs)
        if left:
            print(f'STILL RUNNING: {", ".join(left)}', file=sys.stderr)
        elif procs:
            print('Stopped: nothing is left running.')
        for sig, handler in old.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    sys.exit(main())
