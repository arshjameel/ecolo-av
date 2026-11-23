#!/usr/bin/env python3
"""
The test ladder: which rung has passed, which is next, and the progress kept between days.

    pixi run ladder                         the rungs: the simulator's side and the car's
    pixi run ladder next [--car]            the next open rung, and its command
    pixi run ladder run <rung>              run a rung's simulator twin, and record it
    pixi run ladder record <rung> pass|fail [--car] [--report DIR] [--note TEXT]
                                            [--override WHY]
    pixi run ladder record --from-report ~/aav-tests/<session>   (car-test's report.yaml)
    pixi run ladder run <rung> --only <test>   one of its tests alone, in the simulator;
                                            not recorded (a rung passes whole)
    pixi run ladder session <rung> [--json] the newest simulator session of a rung: each
                                            step, its numbers, its vehicle.yaml lines
                                            (--recorded: the one the ladder's result names)
    pixi run ladder --json                  the same table, for a screen

The rungs are tools/ladder/rungs.yaml (docs/proving-it.md). Progress is one file outside
git, ~/aav-tests/ladder.yaml (AAV_LADDER_FILE): a line a result, never rewritten, so the
history stays. The simulator's passes and the car's are kept apart: a pass in Ravensim
never marks the car's rung. A rung opens when the rungs under it have passed in the same
place; a car rung also needs its own simulator twin. This tool never moves the car: for a
car rung it prints the command, and a person runs it.

No ROS. The rules are functions here, tested in tools/tests/test_ladder.py.
"""
import argparse
import datetime
import getpass
import json
import os
import subprocess
import sys
import time

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
RUNGS = os.path.join(ROOT, 'tools', 'ladder', 'rungs.yaml')
PLACES = ('sim', 'car')
# A report folder's name -> its rung.
STEPS = {'stands': 'stands', 'hallway': 'hallway', 'lot': 'lot'}


def progress_file():
    """Return the progress file's path (AAV_LADDER_FILE, or ~/aav-tests/ladder.yaml)."""
    return os.environ.get('AAV_LADDER_FILE',
                          os.path.expanduser('~/aav-tests/ladder.yaml'))


def load_rungs(path=RUNGS):
    """Return the rungs, bottom first; raise ValueError on an unknown `needs`."""
    with open(path) as f:
        rungs = yaml.safe_load(f)['rungs']
    seen = set()
    for r in rungs:
        missing = [n for n in r['needs'] if n not in seen]
        if missing:
            raise ValueError(f"{r['id']} needs {missing}, not below it")
        seen.add(r['id'])
    return rungs


def load_progress(path=None):
    """Return every result recorded, oldest first."""
    path = path or progress_file()
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get('results', [])


def latest(results, rung, place):
    """Return the newest result for a rung in a place, or None."""
    mine = [r for r in results if r['rung'] == rung and r['place'] == place]
    return mine[-1] if mine else None


def passed(results, rung, place):
    """Return True if the rung's newest result in that place is a pass."""
    last = latest(results, rung, place)
    return last is not None and last['result'] == 'pass'


def done(rungs, results, rung, place):
    """Return True if the rung counts as passed there: one with no car side, by its twin."""
    here = next(r for r in rungs if r['id'] == rung)
    return passed(results, rung, place if here.get('on_car', True) else 'sim')


def blockers(rungs, results, rung, place):
    """Return what keeps a rung shut in a place: text a line, empty when it is open."""
    by_id = {r['id']: r for r in rungs}
    out = [f'{n} has not passed ({place})' for n in by_id[rung]['needs']
           if not done(rungs, results, n, place)]
    if place == 'car' and by_id[rung]['sim'] and not passed(results, rung, 'sim'):
        out.append(f'{rung} has not passed in the simulator')
    return out


def next_rung(rungs, results, place):
    """Return the lowest rung not passed in that place, or None when all have."""
    return next((r for r in rungs if not done(rungs, results, r['id'], place)), None)


def table(rungs, results):
    """Return a row a rung: its state in each place, and what blocks it."""
    rows = []
    for r in rungs:
        row = {'id': r['id'], 'title': r['title'], 'where': r['where'],
               'measures': r['measures'], 'passes': r['passes'], 'how': r['how'],
               'after': r.get('after', ''), 'tests': single_tests(r)}
        for place in PLACES:
            last = latest(results, r['id'], place if r.get('on_car', True) else 'sim')
            row[place] = {
                'state': last['result'] if last else 'not run',
                'when': last['when'] if last else '',
                'commit': last.get('commit', '') if last else '',
                'report': last.get('report', '') if last else '',
                'override': last.get('override', '') if last else '',
                'command': r[place],
                'blocked_by': blockers(rungs, results, r['id'], place),
            }
        rows.append(row)
    return rows


def single_tests(rung):
    """
    Return a rung's single tests, each with the command that runs it alone in the simulator.

    The names come from the runner's own lists (tools/ladder_report.py); the command is
    rungs.yaml's `sim_one`, with {test}. A rung without `tests:` has none.
    """
    if not rung.get('tests'):
        return []
    import ladder_report
    return [{'name': name, 'number': number, 'title': title,
             'command': rung['sim_one'].format(test=name)}
            for name, number, title in ladder_report.single_tests(rung['tests'])]


def commit():
    """Return the repository's short commit, with + when files are changed; '' outside git."""
    def git(*args):
        return subprocess.run(['git', '-C', ROOT, *args], capture_output=True, text=True).stdout
    head = git('rev-parse', '--short', 'HEAD').strip()
    return head + ('+' if head and git('status', '--porcelain').strip() else '')


def record(rung, place, result, path=None, report='', note='', override='', when=None):
    """Add one result to the progress file, and return it."""
    path = path or progress_file()
    entry = {'rung': rung, 'place': place, 'result': result,
             'when': when or datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
             'commit': commit(), 'who': getpass.getuser()}
    entry.update({k: v for k, v in (('report', report), ('note', note),
                                    ('override', override)) if v})
    results = load_progress(path) + [entry]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + '.new', 'w') as f:
        f.write('# The test ladder so far (tools/ladder.py): a line a result, never rewritten.\n')
        yaml.safe_dump({'results': results}, f, sort_keys=False, allow_unicode=True)
    os.replace(path + '.new', path)
    return entry


def from_report(folder):
    """Return (rung, place, result) from a car-test session folder's report.yaml."""
    folder = os.path.abspath(os.path.expanduser(folder))
    with open(os.path.join(folder, 'report.yaml')) as f:
        counts = yaml.safe_load(f)['counts']
    name = os.path.basename(folder.rstrip('/'))
    rung = next((r for key, r in STEPS.items() if key in name), None)
    if rung is None:
        raise ValueError(f'{name}: not a stands, hallway or lot session')
    # Nothing passed is not a pass: a run refused before its first step has no FAIL either.
    clean = (counts.get('FAIL', 0) == 0 and counts.get('NOT RUN', 0) == 0
             and counts.get('PASS', 0) > 0)
    return rung, 'sim' if name.endswith('-sim') else 'car', 'pass' if clean else 'fail'


def tests_dir():
    """Return where the test runs keep their report folders (AAV_TESTS_DIR, ~/aav-tests)."""
    return os.path.expanduser(os.environ.get('AAV_TESTS_DIR', '~/aav-tests'))


def newest_session(rung, since, base=None):
    """
    Return the newest `<time>-<rung>-sim` report folder made at or after `since`, or ''.

    since: seconds (time.time()) when the run began; None takes any. The folder's name
    carries its start, to the second: that, not the file system's times, decides.
    """
    base = base or tests_dir()
    if not os.path.isdir(base):
        return ''
    floor = ('' if since is None else
             datetime.datetime.fromtimestamp(since).strftime('%Y-%m-%d-%H%M%S'))
    tail = f'-{rung}-sim'
    found = sorted(d for d in os.listdir(base)
                   if d.endswith(tail) and d[:17] >= floor
                   and os.path.isfile(os.path.join(base, d, 'report.yaml')))
    return os.path.join(base, found[-1]) if found else ''


MARK = {'pass': 'PASS', 'fail': 'FAIL', 'not run': '  - '}


def show(rows):
    """Print the ladder, top rung first."""
    print(f"{'rung':<9}{'simulator':<24}{'the car':<24}what it measures")
    for row in reversed(rows):
        cells = []
        for place in PLACES:
            p = row[place]
            text = MARK[p['state']] + (f" {p['when'][:10]}" if p['when'] else '')
            if p['state'] != 'pass' and p['blocked_by']:
                text += ' (shut)'
            cells.append(f'{text:<24}')
        print(f"{row['id']:<9}{''.join(cells)}{row['measures']}")


def main(argv=None):
    """Run the command line; return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--json', action='store_true')
    sub = parser.add_subparsers(dest='do')
    nxt = sub.add_parser('next')
    nxt.add_argument('--car', action='store_true')
    run = sub.add_parser('run')
    run.add_argument('rung')
    run.add_argument('--only', default='', metavar='TEST',
                     help="one of the rung's tests alone, in the simulator; not recorded")
    ses = sub.add_parser('session')
    ses.add_argument('rung')
    ses.add_argument('--json', action='store_true', dest='session_json')
    ses.add_argument('--recorded', action='store_true',
                     help="the session the ladder's newest result names, not the newest "
                          'run (a single test since then is the newest run)')
    rec = sub.add_parser('record')
    rec.add_argument('rung', nargs='?')
    rec.add_argument('result', nargs='?', choices=('pass', 'fail'))
    rec.add_argument('--car', action='store_true')
    rec.add_argument('--report', default='')
    rec.add_argument('--note', default='')
    rec.add_argument('--override', default='')
    rec.add_argument('--from-report', default='')
    args = parser.parse_args(argv)
    rungs = load_rungs()
    results = load_progress()
    ids = [r['id'] for r in rungs]

    if args.do == 'next':
        place = 'car' if args.car else 'sim'
        rung = next_rung(rungs, results, place)
        if rung is None:
            print(f'Every rung has passed ({place}).')
            return 0
        shut = blockers(rungs, results, rung['id'], place)
        print(f"Next ({place}): {rung['id']}: {rung['title']}")
        print(f"  passes when: {rung['passes']}")
        print(f"  run: {rung[place] or 'by hand: ' + rung['how']}")
        for line in shut:
            print(f'  shut: {line}')
        return 0

    if args.do == 'run':
        if args.rung not in ids:
            parser.error(f'rungs: {", ".join(ids)}')
        rung = rungs[ids.index(args.rung)]
        if args.only:
            tests = {t['name']: t for t in single_tests(rung)}
            if args.only not in tests:
                names = ', '.join(tests) or 'it has none'
                print(f'{args.rung} has no single test {args.only!r}: {names}',
                      file=sys.stderr)
                return 2
            command = tests[args.only]['command']
            print(f'Running {args.only} alone in the simulator: {command}', flush=True)
            code = subprocess.run(command, shell=True, cwd=ROOT).returncode
            print(f'A single test (exit {code}): not recorded on the ladder. A rung passes '
                  f'whole: pixi run ladder run {args.rung}')
            return code
        shut = blockers(rungs, results, args.rung, 'sim')
        if shut:
            print('Shut: ' + '; '.join(shut) + '. (To record it anyway: ladder record '
                  f'{args.rung} pass --override "why")', file=sys.stderr)
            return 2
        if not rung['sim']:
            print(f"No one command for {args.rung}: by hand, {rung['how']}; then ladder "
                  f'record {args.rung} pass', file=sys.stderr)
            return 2
        print(f"Running {args.rung} in the simulator: {rung['sim']}", flush=True)
        began = time.time()
        code = subprocess.run(rung['sim'], shell=True, cwd=ROOT).returncode
        entry = record(args.rung, 'sim', 'pass' if code == 0 else 'fail',
                       report=newest_session(args.rung, began))
        print(f"Recorded: {args.rung} {entry['result']} (sim) at {entry['commit']}")
        return code

    if args.do == 'session':
        if args.rung not in ids:
            parser.error(f'rungs: {", ".join(ids)}')
        import ladder_report
        last = latest(results, args.rung, 'sim')
        named = last.get('report', '') if last else ''
        folder = named if args.recorded else newest_session(args.rung, None)
        report = ladder_report.session(folder)
        if args.recorded and not named:
            report['why'] = 'the ladder has no recorded session for this rung yet'
        report.update(rung=args.rung, recorded=bool(folder) and named == folder)
        print(json.dumps(report) if args.session_json else ladder_report.text(report))
        return 0

    if args.do == 'record':
        place = 'car' if args.car else 'sim'
        rung, result, report = args.rung, args.result, args.report
        if args.from_report:
            rung, place, result = from_report(args.from_report)
            report = os.path.abspath(os.path.expanduser(args.from_report))
        if rung not in ids or result is None:
            parser.error(f'record <rung> pass|fail; rungs: {", ".join(ids)}')
        shut = blockers(rungs, results, rung, place)
        if shut and result == 'pass' and not args.override:
            print('Shut: ' + '; '.join(shut) + '. A pass here needs --override "why".',
                  file=sys.stderr)
            return 2
        entry = record(rung, place, result, report=report, note=args.note,
                       override=args.override)
        print(f"Recorded: {rung} {entry['result']} ({place}) at {entry['commit']}")
        return 0

    rows = table(rungs, results)
    if args.json:
        print(json.dumps({'file': progress_file(), 'rungs': rows}, indent=1))
    else:
        show(rows)
        print(f'\nProgress: {progress_file()}    Next: pixi run ladder next')
    return 0


if __name__ == '__main__':
    sys.exit(main())
