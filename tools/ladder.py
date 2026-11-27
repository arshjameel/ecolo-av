#!/usr/bin/env python3
"""
The test ladder: ten rungs named for what the car can do, and the progress kept between days.

    pixi run ladder                         the rungs: the simulator's side and the car's
    pixi run ladder next [--car]            the next open rung, and its command
    pixi run ladder run <rung>              run a rung in the simulator, record it, and save
                                            its report (the question, the verdict, the graphs)
    pixi run ladder run <rung> --only <test>   one of its tests alone, in the simulator;
                                            not recorded (a rung passes whole)
    pixi run ladder run code|stack|bench    a "before you start" check (it moves no car)
    pixi run ladder record <rung> pass|fail [--car] [--report DIR] [--note TEXT]
                                            [--override WHY]
    pixi run ladder record --from-report ~/aav-tests/<session>   (car-test's report.yaml)
    pixi run ladder session <rung> [--json] the newest simulator run of a rung: its answer
                                            (verdict, graphs, numbers) and each step
                                            (--recorded: the run the ladder's result names)
    pixi run ladder report <rung> [--open]  write that run's report page, print where it
                                            is; --open: in the browser (--recorded as above)
    pixi run ladder all --sim               the full suite on the virtual car: every built
                                            rung in turn, each recorded, one report
                                            (--car is refused: a person runs the car's)
    pixi run ladder --json                  the same table, for a screen

A rung is named by its id or its number: `run stands` is `run 1`.

The rungs are tools/ladder/rungs.yaml (docs/wrap-up.md, docs/proving-it.md). A rung that
is not built yet is listed, and is never run, recorded, passed or failed. Progress is one
file outside git, ~/aav-tests/ladder.yaml (AAV_LADDER_FILE): a line a result, never
rewritten, so the history stays. The simulator's passes and the car's are kept apart: a
pass in Ravensim never marks the car's rung. A rung opens when the rungs under it have
passed in the same place; a car rung also needs its own simulator run. This tool never
moves the car: for a car rung it prints the command, and a person runs it.

Results saved before 23 Nov, under the old rungs' names, are read like this (the file is
never rewritten): `stands` is rung 1 (the same run, the same pass mark); `code`, `stack`
and `bench` are the checks; `hallway`, `lot` and `route` are not shown (old_results()).

`all --sim` (wrap-up step 5) runs every built rung bottom first, in one session: each as
`run <rung>` runs it (recorded, its report page saved), under one lock, so nothing else
runs between them. A rung not built yet is skipped and says so; one shut by a rung under
it that failed is skipped too. Ctrl-C stops the rung that runs (its own command gets the
Ctrl-C and stops its simulator; it is waited for), records nothing for it, and skips the
rest. The session's one report is `<time>-all-sim/ladder-all.yaml` and `ladder-all.html`
in ~/aav-tests: every rung, what happened, how long, its own report page.

No ROS. The rules are functions here, tested in tools/tests/test_ladder.py; a rung's
answer and its report page are tools/rung_answer.py.
"""
import argparse
import contextlib
import datetime
import fcntl
import getpass
import json
import os
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
RUNGS = os.path.join(ROOT, 'tools', 'ladder', 'rungs.yaml')
PLACES = ('sim', 'car')
# Names that were rungs before 23 Nov and are neither a rung nor a check now: their lines
# stay in the progress file, and are not shown (rungs 2 to 4 get their own pass marks).
RETIRED = ('hallway', 'lot', 'route')
NOT_BUILT = 'not built'


def progress_file():
    """Return the progress file's path (AAV_LADDER_FILE, or ~/aav-tests/ladder.yaml)."""
    return os.environ.get('AAV_LADDER_FILE',
                          os.path.expanduser('~/aav-tests/ladder.yaml'))


def load_file(path=RUNGS):
    """Return rungs.yaml whole: `rungs` and `checks`."""
    with open(path) as f:
        return yaml.safe_load(f)


def load_rungs(path=RUNGS):
    """Return the rungs, bottom first; raise ValueError on an unknown `needs`."""
    rungs = load_file(path)['rungs']
    seen = set()
    for r in rungs:
        missing = [n for n in r['needs'] if n not in seen]
        if missing:
            raise ValueError(f"{r['id']} needs {missing}, not below it")
        if not r.get('built', True) and (r['sim'] or r['car']):
            raise ValueError(f"{r['id']} is not built: it can have no command")
        seen.add(r['id'])
    return rungs


def load_checks(path=RUNGS):
    """Return the "before you start" checks: they move no car, and shut no rung."""
    return load_file(path).get('checks', [])


def find(rungs, name):
    """Return the rung with that id or number (`stands`, `1`, `s`), or None."""
    name = str(name).lower()
    return next((r for r in rungs if name in (r['id'], str(r['number']).lower())), None)


def built(rung):
    """Return True for a rung that exists; False: listed only, as "not built yet"."""
    return rung.get('built', True)


def load_progress(path=None):
    """Return every result recorded, oldest first."""
    path = path or progress_file()
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get('results', [])


def old_results(results):
    """
    Return a line of words about results under names that are not rungs any more, or ''.

    They are ignored, not mapped forward: a hallway, lot or route pass was judged by that
    session's own pass mark, which is not the pass mark of rung 2, 3 or 4.
    """
    counts = {name: sum(r['rung'] == name for r in results) for name in RETIRED}
    found = [f'{name} ({n})' for name, n in counts.items() if n]
    if not found:
        return ''
    return ('Older results kept in the file and not shown, from rungs that are gone: '
            + ', '.join(found) + '. The new rungs 2 to 4 will be judged by their own graphs.')


def latest(results, rung, place):
    """Return the newest result for a rung in a place, or None."""
    mine = [r for r in results if r['rung'] == rung and r['place'] == place]
    return mine[-1] if mine else None


def passed(results, rung, place):
    """Return True if the rung's newest result in that place is a pass."""
    last = latest(results, rung, place)
    return last is not None and last['result'] == 'pass'


def blockers(rungs, results, rung, place):
    """Return what keeps a rung shut in a place: text a line, empty when it is open."""
    by_id = {r['id']: r for r in rungs}
    out = [f"rung {by_id[n]['number']} ({n}) has not passed ({place})"
           for n in by_id[rung]['needs'] if not passed(results, n, place)]
    if place == 'car' and by_id[rung]['sim'] and not passed(results, rung, 'sim'):
        out.append(f"rung {by_id[rung]['number']} ({rung}) has not passed in the simulator")
    return out


def next_rung(rungs, results, place):
    """Return the lowest built rung not passed in that place, or None when all have."""
    return next((r for r in rungs if built(r) and not passed(results, r['id'], place)),
                None)


def side(entry, command, shut):
    """Return one rung (or check) in one place, as the table and the window read it."""
    return {'state': entry['result'] if entry else 'not run',
            'when': entry['when'] if entry else '',
            'commit': entry.get('commit', '') if entry else '',
            'report': entry.get('report', '') if entry else '',
            'report_file': entry.get('report_file', '') if entry else '',
            'override': entry.get('override', '') if entry else '',
            'command': command, 'blocked_by': shut}


def table(rungs, results):
    """Return a row a rung: its question, its state in each place, and what blocks it."""
    rows = []
    for r in rungs:
        row = {'id': r['id'], 'number': str(r['number']), 'title': r['title'],
               'question': r['question'].strip(), 'graphs': r['graphs'],
               'passes': r['passes'].strip(), 'chapter': r['chapter'], 'built': built(r),
               'today': r.get('today', ''), 'how': r['how'],
               'after': r.get('after', '').strip(), 'tests': single_tests(r)}
        for place in PLACES:
            if built(r):
                row[place] = side(latest(results, r['id'], place), r[place],
                                  blockers(rungs, results, r['id'], place))
            else:       # whatever the file holds: never passed, never failed
                row[place] = dict(side(None, '', []), state=NOT_BUILT)
        rows.append(row)
    return rows


def checks_table(checks, results):
    """Return a row a check: its newest result (they run on this computer: `sim`)."""
    return [dict(side(latest(results, c['id'], 'sim'), c['command'], []),
                 id=c['id'], title=c['title']) for c in checks]


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


def record(rung, place, result, path=None, report='', note='', override='', when=None,
           report_file=''):
    """Add one result to the progress file, and return it."""
    path = path or progress_file()
    entry = {'rung': rung, 'place': place, 'result': result,
             'when': when or datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
             'commit': commit(), 'who': getpass.getuser()}
    entry.update({k: v for k, v in (('report', report), ('report_file', report_file),
                                    ('note', note), ('override', override)) if v})
    results = load_progress(path) + [entry]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + '.new', 'w') as f:
        f.write('# The test ladder so far (tools/ladder.py): a line a result, never rewritten.\n')
        yaml.safe_dump({'results': results}, f, sort_keys=False, allow_unicode=True)
    os.replace(path + '.new', path)
    return entry


def from_report(folder, rungs=None):
    """Return (rung, place, result) from a car-test session folder's report.yaml."""
    folder = os.path.abspath(os.path.expanduser(folder))
    with open(os.path.join(folder, 'report.yaml')) as f:
        counts = yaml.safe_load(f)['counts']
    name = os.path.basename(folder.rstrip('/'))
    kinds = [r['tests'] for r in (rungs or load_rungs()) if r.get('tests')]
    rung = next((r for r in kinds if f'-{r}' in name), None)
    if rung is None:
        raise ValueError(f'{name}: not the run of a rung that is built ({", ".join(kinds)})')
    # Nothing passed is not a pass: a run refused before its first step has no FAIL either.
    clean = (counts.get('FAIL', 0) == 0 and counts.get('NOT RUN', 0) == 0
             and counts.get('PASS', 0) > 0)
    return rung, 'sim' if name.endswith('-sim') else 'car', 'pass' if clean else 'fail'


def tests_dir():
    """Return where the test runs keep their report folders (AAV_TESTS_DIR, ~/aav-tests)."""
    return os.path.expanduser(os.environ.get('AAV_TESTS_DIR', '~/aav-tests'))


def newest_session(rung, since, base=None, first=False):
    """
    Return the newest `<time>-<rung>-sim` report folder made at or after `since`, or ''.

    first: the first run since then, not the newest. The run a person just started is the
    first: one started over it, from another window, is newer and is not theirs.

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
    return os.path.join(base, found[0 if first else -1]) if found else ''


@contextlib.contextmanager
def only_run():
    """
    Hold the one place a run may have on this computer's ROS domain; give False if taken.

    Two runs on one simulated car fight over it, and both are stopped. The lock is a file's
    (flock), so it goes with the process that held it, however that ended.
    """
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    path = os.path.join(tempfile.gettempdir(), f'aav-ladder-run-{os.getuid()}-{domain}.lock')
    with open(path, 'w') as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            yield False
            return
        yield True


BUSY = ('Another run of the ladder is going on this computer (another window, or a '
        'terminal): one car, one run at a time. Wait for it, or stop it there. Nothing ran '
        'and nothing was recorded.')


def entry_for(results, rung, folder):
    """Return the ladder's own line for that run's folder (its commit, its date), or None."""
    return next((r for r in reversed(results)
                 if r['rung'] == rung and r.get('report') == folder), None)


def session_of(rung, results, recorded):
    """Return (folder, the ladder's line for it or None) for a rung's run to show."""
    last = latest(results, rung['id'], 'sim')
    named = last.get('report', '') if last else ''
    folder = named if recorded else newest_session(rung['id'], None)
    return folder, entry_for(results, rung['id'], folder) if folder else None


def save_report(rung, folder, entry):
    """Write the run's report page into its folder; return its path, '' when it has none."""
    import rung_answer
    if not folder or not os.path.isfile(os.path.join(folder, 'report.yaml')):
        return ''
    return rung_answer.save(rung, folder, entry) or ''


def run_rung_in_sim(rung, run=None):
    """
    Run a built rung's simulator command, record it, save its page: (code, entry, page).

    The caller holds only_run(). run: how to run the shell command (its exit code back);
    the default is subprocess.run.
    """
    run = run or (lambda command: subprocess.run(command, shell=True, cwd=ROOT).returncode)
    print(f"Running rung {rung['number']} in the simulator: {rung['sim']}", flush=True)
    began = time.time()
    code = run(rung['sim'])
    folder = newest_session(rung['id'], began, first=True)
    entry = {'rung': rung['id'], 'commit': commit(), 'report': folder,
             'when': datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}
    page = save_report(rung, folder, entry)
    entry = record(rung['id'], 'sim', 'pass' if code == 0 else 'fail', report=folder,
                   report_file=page, when=entry['when'])
    print(f"Recorded: rung {rung['number']} ({rung['id']}) {entry['result']} (sim) at "
          f"{entry['commit']}")
    if page:
        print(f'Its report: {page}')
    return code, entry, page


class Stopped(Exception):
    """Ctrl-C during a rung of `all`: its command has ended; the session stops."""


STOP_WAIT = 60.0    # s for a rung's command to stop its simulator after Ctrl-C


def run_waiting(command, wait=STOP_WAIT):
    """
    Run a shell command; on Ctrl-C wait for it to stop itself, then raise Stopped.

    Ctrl-C reaches the command too (the same terminal): car-test stops its own simulator.
    subprocess.run would kill it after a quarter of a second, and leave that running.
    """
    proc = subprocess.Popen(command, shell=True, cwd=ROOT)
    try:
        return proc.wait()
    except KeyboardInterrupt:
        try:
            proc.wait(timeout=wait)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            proc.terminate()
            proc.wait()
        raise Stopped() from None


def all_in_sim(rungs, run=None, load=load_progress, run_one=run_rung_in_sim):
    """
    Run every built rung in the simulator, bottom first; return the session's lines.

    Each line: number, id, title, state (pass, fail, not built yet, shut, stopped, not
    run), why, seconds, report (the run's folder), report_file (its page).
    """
    run = run or run_waiting
    lines, stopped = [], False
    for rung in rungs:
        line = {'number': str(rung['number']), 'id': rung['id'], 'title': rung['title'],
                'state': 'not run', 'why': '', 'seconds': 0.0, 'report': '',
                'report_file': ''}
        lines.append(line)
        if stopped:
            line['why'] = 'stopped before it (Ctrl-C)'
        elif not built(rung):
            line.update(state=NOT_BUILT + ' yet', why=str(rung.get('today', '')).strip())
        elif not rung['sim']:
            line['why'] = 'it has no simulator command'
        else:
            shut = blockers(rungs, load(), rung['id'], 'sim')
            if shut:
                line.update(state='shut', why='; '.join(shut))
                continue
            began = time.monotonic()
            try:
                code, entry, page = run_one(rung, run)
                line.update(state=entry['result'], report=entry.get('report', ''),
                            report_file=page, why=f'exit {code}' if code else '')
            except Stopped:
                stopped = True
                line.update(state='stopped', why='Ctrl-C: not recorded')
            line['seconds'] = round(time.monotonic() - began, 1)
    return lines


def write_session(lines, base=None, now=None):
    """Write the session's one report (YAML and an HTML page); return the HTML's path."""
    import html as html_lib
    now = now or datetime.datetime.now()
    folder = os.path.join(base or tests_dir(), now.strftime('%Y-%m-%d-%H%M%S') + '-all-sim')
    os.makedirs(folder, exist_ok=True)
    session = {'what': 'pixi run ladder all --sim: every built rung in the simulator',
               'when': now.strftime('%Y-%m-%d %H:%M'), 'commit': commit(),
               'seconds': round(sum(x['seconds'] for x in lines), 1), 'rungs': lines}
    with open(os.path.join(folder, 'ladder-all.yaml'), 'w') as f:
        yaml.safe_dump(session, f, sort_keys=False, allow_unicode=True)
    esc = html_lib.escape
    rows = ''.join(
        f"<tr><td>{esc(x['number'])}</td><td>{esc(x['title'])}</td>"
        f"<td><b>{esc(x['state'].upper())}</b></td><td>{x['seconds']:g} s</td>"
        f"<td>{esc(x['why'])}</td><td>"
        + (f'<a href="file://{esc(x["report_file"])}">its report</a>'
           if x['report_file'] else '') + '</td></tr>'
        for x in lines)
    page = os.path.join(folder, 'ladder-all.html')
    with open(page, 'w') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>The ladder, every rung</title>'
                '<style>body{font:16px sans-serif;margin:2em}td,th{padding:4px 12px;'
                'text-align:left}tr:nth-child(even){background:#eee}</style>'
                '<h1>The ladder in the simulator, every rung</h1>'
                f"<p>{esc(session['when'])}, commit {esc(session['commit'])}, "
                f"{session['seconds']:g} s.</p>"
                '<table><tr><th>Rung</th><th>What</th><th>Result</th><th>Time</th>'
                f'<th>Why</th><th></th></tr>{rows}</table>')
    return page


MARK = {'pass': 'PASS', 'fail': 'FAIL', 'not run': '  - ', NOT_BUILT: 'not built yet'}


def day(side):
    return f" {side['when'][:10]}" if side['when'] else ''


def show(rows, checks, note):
    """Print the ladder, top rung first; then the checks, on one line."""
    print(f"{'':<3}{'rung':<45}{'simulator':<19}{'the car':<19}")
    beside = [r for r in rows if r['number'] == 'S']
    for row in [r for r in reversed(rows) if r['number'] != 'S'] + beside:
        if row['number'] == 'S':
            print('beside every rung:')
        cells = []
        for place in PLACES:
            p = row[place]
            text = MARK[p['state']] + day(p)
            if p['state'] in ('fail', 'not run') and p['blocked_by']:
                text += ' (shut)'
            cells.append(f'{text:<19}')
        print(f"{row['number']:<3}{row['title']:<45}{''.join(cells)}")
    print('\nBefore you start (they move no car): ' + '; '.join(
        f"{c['title']}: {MARK[c['state']].strip().replace('-', 'not run')}{day(c)}"
        for c in checks))
    if note:
        print(note)


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
    rep = sub.add_parser('report')
    rep.add_argument('rung')
    rep.add_argument('--open', action='store_true', help='open the page in the browser')
    for which in (ses, rep):
        which.add_argument('--recorded', action='store_true',
                           help="the run the ladder's newest result names, not the newest "
                                'run (a single test since then is the newest run)')
    every = sub.add_parser('all', help='every built rung in the simulator, one report')
    every.add_argument('--sim', action='store_true',
                       help='in the simulator (the only place: also with no flag)')
    every.add_argument('--car', action='store_true', help='refused: a person runs the car')
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
    checks = load_checks()
    results = load_progress()
    names = ', '.join(f"{r['number']} {r['id']}" for r in rungs)

    def rung_named(name, with_checks=False):
        found = find(rungs, name)
        if found is None and with_checks:
            found = next((c for c in checks if c['id'] == name), None)
        if found is None:
            also = '; the checks: ' + ', '.join(c['id'] for c in checks)
            parser.error(f'rungs: {names}{also if with_checks else ""}')
        return found

    def not_built(rung):
        print(f"Rung {rung['number']} ({rung['title']}) is not built yet: it cannot be "
              f"run or recorded. Today: {rung.get('today', '')}", file=sys.stderr)
        return 2

    if args.do == 'next':
        place = 'car' if args.car else 'sim'
        rung = next_rung(rungs, results, place)
        if rung is None:
            waiting = next((r for r in rungs if not built(r)), None)
            print(f'Every rung that is built has passed ({place}).' + (
                f" Next to build: rung {waiting['number']}, {waiting['title']}."
                if waiting else ''))
            return 0
        shut = blockers(rungs, results, rung['id'], place)
        print(f"Next ({place}): rung {rung['number']}: {rung['title']}")
        print(f"  the question: {rung['question'].strip()}")
        print(f"  passes when: {rung['passes'].strip()}")
        print(f"  run: {rung[place] or 'by hand: ' + rung['how']}")
        for line in shut:
            print(f'  shut: {line}')
        return 0

    if args.do == 'run':
        rung = rung_named(args.rung, with_checks=True)
        if 'command' in rung:       # a check: it moves no car and shuts nothing
            print(f"Running the check {rung['id']}: {rung['command']}", flush=True)
            code = subprocess.run(rung['command'], shell=True, cwd=ROOT).returncode
            entry = record(rung['id'], 'sim', 'pass' if code == 0 else 'fail')
            print(f"Recorded: {rung['id']} {entry['result']} at {entry['commit']}")
            return code
        if not built(rung):
            return not_built(rung)
        if args.only:
            tests = {t['name']: t for t in single_tests(rung)}
            if args.only not in tests:
                listed = ', '.join(tests) or 'it has none'
                print(f"{rung['id']} has no single test {args.only!r}: {listed}",
                      file=sys.stderr)
                return 2
            command = tests[args.only]['command']
            with only_run() as free:
                if not free:
                    print(BUSY, file=sys.stderr)
                    return 2
                print(f'Running {args.only} alone in the simulator: {command}', flush=True)
                code = subprocess.run(command, shell=True, cwd=ROOT).returncode
            print(f'A single test (exit {code}): not recorded on the ladder. A rung passes '
                  f"whole: pixi run ladder run {rung['id']}")
            return code
        shut = blockers(rungs, results, rung['id'], 'sim')
        if shut:
            print('Shut: ' + '; '.join(shut) + '. (To record it anyway: ladder record '
                  f"{rung['id']} pass --override \"why\")", file=sys.stderr)
            return 2
        with only_run() as free:
            if not free:
                print(BUSY, file=sys.stderr)
                return 2
            code, _, _ = run_rung_in_sim(rung)
        return code

    if args.do == 'all':
        if args.car:
            print('REFUSED: the ladder never moves the car. Its rungs on the car are run by '
                  'a person, one at a time: pixi run ladder next --car', file=sys.stderr)
            return 2
        with only_run() as free:
            if not free:
                print(BUSY, file=sys.stderr)
                return 2
            print('The full suite on the virtual car: every built rung, bottom first.',
                  flush=True)
            lines = all_in_sim(rungs)
        page = write_session(lines)
        print('\nThe session:')
        for x in lines:
            print(f"  {x['number']:<3}{x['title']:<45}{x['state'].upper():<16}"
                  f"{x['seconds']:>7g} s  {x['why']}")
        print(f'Its report: {page}')
        states = {x['state'] for x in lines}
        return 130 if 'stopped' in states else 1 if 'fail' in states else 0

    if args.do in ('session', 'report'):
        rung = rung_named(args.rung)
        folder, entry = session_of(rung, results, args.recorded)
        if args.do == 'report':
            if not folder:
                print(f"No run of rung {rung['number']} to report yet.", file=sys.stderr)
                return 2
            import rung_answer
            page = os.path.join(folder, rung_answer.file_name(rung))
            if not os.path.isfile(page):      # a run from a terminal, or a single test
                page = save_report(rung, folder, entry)
            if not page:
                print(f"Rung {rung['number']} has no report page yet.", file=sys.stderr)
                return 2
            print(page)
            if args.open:
                import webbrowser
                webbrowser.open('file://' + page)
            return 0
        import ladder_report
        import rung_answer
        report = ladder_report.session(folder)
        if args.recorded and not folder:
            report['why'] = 'the ladder has no recorded run for this rung yet'
        last = latest(results, rung['id'], 'sim')
        report.update(rung=rung['id'],
                      recorded=bool(folder) and bool(last) and last.get('report') == folder,
                      answer=rung_answer.answer(rung, folder, entry))
        print(json.dumps(report) if args.session_json else ladder_report.text(report))
        return 0

    if args.do == 'record':
        place = 'car' if args.car else 'sim'
        name, result, report = args.rung, args.result, args.report
        if args.from_report:
            name, place, result = from_report(args.from_report, rungs)
            report = os.path.abspath(os.path.expanduser(args.from_report))
        if name is None or result is None:
            parser.error(f'record <rung> pass|fail; rungs: {names}')
        rung = rung_named(name)
        if not built(rung):
            return not_built(rung)
        shut = blockers(rungs, results, rung['id'], place)
        if shut and result == 'pass' and not args.override:
            print('Shut: ' + '; '.join(shut) + '. A pass here needs --override "why".',
                  file=sys.stderr)
            return 2
        entry = record(rung['id'], place, result, report=report, note=args.note,
                       override=args.override)
        print(f"Recorded: rung {rung['number']} ({rung['id']}) {entry['result']} ({place}) "
              f"at {entry['commit']}")
        return 0

    rows = table(rungs, results)
    before = checks_table(checks, results)
    note = old_results(results)
    if args.json:
        print(json.dumps({'file': progress_file(), 'rungs': rows, 'checks': before,
                          'note': note}, indent=1))
    else:
        show(rows, before, note)
        print(f'\nProgress: {progress_file()}    Next: pixi run ladder next')
    return 0


if __name__ == '__main__':
    sys.exit(main())
