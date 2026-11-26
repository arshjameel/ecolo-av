#!/usr/bin/env python3
"""
A rung's answer, from one run's own files: the verdict, the graphs, the numbers, the page.

    answer(rung, folder, entry)   one dict, for the tests screen (tools/ladder.py's
                                  `session --json` carries it as `answer`)
    save(rung, folder, entry)     the same as one HTML page in the run's folder
                                  (rung-1-report.html), graphs inline; returns its path

Built for rung 1 (`stands`, a car-test --stands run): docs/wrap-up.md. A rung with no
builder here has no answer yet (None).

Where every number comes from, for rung 1:

    the verdict      report.yaml: each step's PASS / FAIL / SKIPPED / NOT RUN
    the wheels       step wheel-steps, log.csv: cmd_speed (asked), speed (measured)
    the steering     step steer-sweep, log.csv: cmd_steer (asked), steer (measured)
    the brake        step wheel-steps, log.csv: brake_cmd (asked), brake (measured)
    the faults       report.yaml: each fault step's result, and the times in the runner's
                     own sentence (car_test.judge_fault writes it; fault_times reads it)
    delay, rise, off tools/step_response.py, on the log's samples as they are

A number the files do not hold is the words "not measured", with what is missing. Nothing
is invented, fitted or smoothed. A graph draws every k-th sample (never an average).

The car's side is a slot: `car` says "no car run yet" until a later step fills it from a
car run. No ROS. Tested in tools/tests/test_rung_answer.py.
"""
import csv
import html as html_lib
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'ros', 'drivers', 'aav_actuator_tests'))

import step_response as sr  # noqa: E402
import yaml  # noqa: E402

SCREEN_POINTS = 300     # at most this many points a line on the tests screen
PAGE_POINTS = 1500      # and on the saved page
NOT_MEASURED = 'not measured'
NO_CAR = 'no car run yet'
STILL = 0.02            # m/s: under this the wheels count as still
ASKED_WORDS = 'asked for (dashed line)'
MEASURED_WORDS = 'measured (solid line)'
WHEELS, STEERING = ('wheel-steps',), ('steer-sweep', 'steer-rate', 'steer-centre')
HOW_TIMED = ('delay: the time until the measured line has moved 10 % of the way to the '
             'new value. 10 to 90 %: the time it then takes to get 90 % of the way. off at '
             'the end: the mean of the last quarter of the step, less the value asked for.')


def file_name(rung):
    """Return the report page's name in a run's folder: rung-1-report.html."""
    return f"rung-{str(rung['number']).lower()}-report.html"


def finite(text):
    """Return a log cell as a number, or None when it is empty or not finite."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def read_log(folder, step):
    """Return (rows, '') of a step's log.csv, or ([], why) when it has none."""
    if step is None:
        return [], ''
    log = os.path.join(folder, step.get('files') or '', 'log.csv')
    if not step.get('files') or not os.path.isfile(log):
        return [], f"step {step['number']} ({step['step']}) kept no log"
    with open(log, newline='') as f:
        return list(csv.DictReader(f)), ''


def series(rows, asked, measured):
    """Return (t, asked, measured) from log rows: only the rows where all three are numbers."""
    points = [(finite(r.get('t')), finite(r.get(asked)), finite(r.get(measured)))
              for r in rows]
    points = [p for p in points if None not in p]
    return tuple([p[i] for p in points] for i in range(3))


def thin(lines, most):
    """Return the lines with every k-th sample kept (and the last), at most about `most`."""
    n = len(lines[0])
    keep = max(1, math.ceil(n / most))
    picked = list(range(0, n, keep))
    if n and picked[-1] != n - 1:
        picked.append(n - 1)
    return [[round(line[i], 4) for i in picked] for line in lines]


def seconds(value):
    return NOT_MEASURED if value is None else f'{value:.2f} s'


def amount(value, unit, signed=False):
    return f'{value:+.3f} {unit}' if signed else f'{value:.3f} {unit}'


def empty_graph(key, title, asks, y_label, unit, missing):
    return {'id': key, 'title': title, 'asks': asks, 'from': '', 'x_label': 'time (s)',
            'y_label': y_label, 'unit': unit, 'asked_words': ASKED_WORDS,
            'measured_words': MEASURED_WORDS, 't': [], 'asked': [], 'measured': [],
            'columns': [], 'rows': [], 'facts': [], 'notes': [], 'judged': '',
            'missing': missing, 'car': NO_CAR}


def step_rows(found, unit, name=lambda r: f'{r.step.asked_to:g}'):
    """Return a table row a step (asked, delay, rise, end, off), and notes on what is missing."""
    rows, notes = [], []
    for r in found:
        rows.append([name(r), seconds(r.delay), seconds(r.rise), amount(r.end, unit),
                     amount(r.off, unit, signed=True)])
        if r.why:
            notes.append(f'{name(r)}: {NOT_MEASURED}, because {r.why}.')
    return rows, notes


def sampled(t):
    gap = sr.period(t)
    return (f'The log has a sample every {gap:.2f} s: no time here is finer than that.'
            if gap else '')


def judged(steps, names):
    """Return the run's own word on some steps: `4 wheel-steps PASS: detail`, a line each."""
    return ' '.join(f"Step {s['number']} ({s['step']}): {s['status']}." for s in steps
                    if s['step'] in names)


def graph_of(folder, steps, key, title, asks, step_name, columns, y_label, unit, points):
    """Return (graph, t, asked, measured, rows): the lines of one log, or why there are none."""
    asked_col, measured_col = columns
    step = next((s for s in steps if s['step'] == step_name), None)
    out = empty_graph(key, title, asks, y_label, unit, '')
    if step is None:
        out['missing'] = f'{NOT_MEASURED}: this run did not include the step {step_name}'
        return out, [], [], [], []
    rows, why = read_log(folder, step)
    t, asked, measured = series(rows, asked_col, measured_col) if rows else ([], [], [])
    if len(t) < 2:
        out['missing'] = f'{NOT_MEASURED}: ' + (why or (
            f"the log of step {step['number']} ({step_name}) has no numbers in "
            f'{asked_col} and {measured_col}'))
        return out, [], [], [], rows
    out['t'], out['asked'], out['measured'] = thin((t, asked, measured), points)
    out['from'] = (f"step {step['number']} ({step_name}), its log.csv: {asked_col} (asked "
                   f'for), {measured_col} (measured); {len(t)} samples')
    return out, t, asked, measured, rows


def wheels_graph(folder, steps, points):
    """Return the wheels' graph: their speed following the speed steps (step wheel-steps)."""
    g, t, asked, measured, _ = graph_of(
        folder, steps, 'wheels', 'The wheels: the speed asked for, and the speed measured',
        'Do the wheels follow a speed command?', 'wheel-steps', ('cmd_speed', 'speed'),
        'wheel speed (m/s)', 'm/s', points)
    if not t:
        return g
    found = sr.responses(t, asked, measured, least=0.02)
    g['columns'] = ['speed asked for (m/s)', 'delay', '10 to 90 %', 'measured at the end',
                    'off at the end']
    g['rows'], notes = step_rows(found, 'm/s')
    g['notes'] = notes + [HOW_TIMED, sampled(t), 'Asked for: what the test asks of the car '
                          '(a speed). Measured: the speed the wheels report.']
    g['judged'] = judged(steps, WHEELS)
    return g


def steering_graph(folder, steps, points):
    """Return the steering's graph: the angle following the sweep (step steer-sweep)."""
    g, t, asked, measured, _ = graph_of(
        folder, steps, 'steering', 'The steering: the angle asked for, and the angle measured',
        'Does the steering follow an angle command, the right way?', 'steer-sweep',
        ('cmd_steer', 'steer'), 'steering angle (rad, + is left)', 'rad', points)
    if not t:
        return g
    found = sr.responses(t, asked, measured, least=0.02)
    g['columns'] = [f"over the sweep's {len(found)} steps", 'the middle one', 'smallest',
                    'largest']
    rows = []
    for what, values, show in (
            ('delay', [r.delay for r in found], seconds),
            ('10 to 90 %', [r.rise for r in found], seconds),
            ('off at the end, either way', [abs(r.off) for r in found],
             lambda v: amount(v, 'rad'))):
        got = sr.spread(values)
        rows.append([what] + ([NOT_MEASURED] * 3 if got is None else
                              [show(got.median), show(got.low), show(got.high)]))
    g['rows'] = rows
    left = [r for r in found if r.step.asked_to > r.step.asked_from]
    same_way = all((r.end - r.before) > 0 for r in left) if left else None
    g['facts'] = [] if same_way is None else [
        ['asked to the left (+), the measured angle went',
         'to the left (+): the right way' if same_way else 'the WRONG way on some step']]
    untimed = [f'{r.step.asked_to:+.3f} rad: {r.why}' for r in found if r.why]
    g['notes'] = ([f'{NOT_MEASURED} for {len(untimed)} of the steps ({"; ".join(untimed)}).']
                  if untimed else []) + [HOW_TIMED, sampled(t),
                                         'Asked for: the angle the test asks of the car.']
    g['judged'] = judged(steps, STEERING)
    return g


def brake_graph(folder, steps, points):
    """Return the brake's graph: let go and pressed again, in the wheels' test."""
    g, t, asked, measured, rows = graph_of(
        folder, steps, 'brake', 'The brake: how hard it was asked to press, and how hard it did',
        'Does the brake follow its command?', 'wheel-steps', ('brake_cmd', 'brake'),
        'brake (0 = off, 1 = full)', 'of full', points)
    if not t:
        return g
    found = sr.responses(t, asked, measured, least=0.05)
    g['columns'] = ['brake asked for (0 to 1)', 'delay', '10 to 90 %', 'measured at the end',
                    'off at the end']
    g['rows'], notes = step_rows(found, 'of full', lambda r: (
        f"{r.step.asked_to:g} ({'pressed' if r.step.asked_to > r.step.asked_from else 'let go'})"))
    ts, asked_speed, speed = series(rows, 'cmd_speed', 'speed')
    stops = [s for s in sr.steps(asked_speed, 0.02) if s.asked_to == 0]
    if stops:
        took = sr.first_below(ts, speed, stops[-1].first, STILL)
        was = speed[stops[-1].first - 1]
        g['facts'] = [[f'asked to stop from {was:.2f} m/s, the wheels were still after',
                       seconds(took) if took is not None else
                       f'{NOT_MEASURED}: the log ends with the wheels at '
                       f'{abs(speed[-1]):.2f} m/s']]
    g['notes'] = notes + [
        HOW_TIMED, sampled(t),
        "Asked for: what the car's own software asks of the brake. The test asks for a "
        "speed; pressing the brake is the software's answer to 'stop'."]
    g['judged'] = 'The run has no pass mark for the brake alone: shown, not judged.'
    return g


# The runner's own sentences (car_test.judge_fault, judge_heartbeat_car), read back.
AFTER_WORD = re.compile(r'still (?P<total>[\d.]+) s after (?P<since>.+?), '
                        r'(?P<took>[\d.]+) s after the word \(limit (?P<limit>[\d.]+) s\)')
AFTER_FAULT = re.compile(r'still (?P<took>[\d.]+) s after (?P<since>.+?)'
                         r'(?: \(limit (?P<limit>[\d.]+) s\)|;|$)')
SAID = re.compile(r'said why (?P<said>[\d.]+) s after ')
# The test itself stopped the car, on the watchdog's word (car_test_node.sensor_fault's note).
BY_TEST = 'the runner stopped on its word'
BY_ITSELF, BY_THE_TEST, NOT_SHOWN = 'yes', 'caught; the test stopped it', 'not shown'


def began_before(name, detail):
    """
    Return how long the sensor had been frozen when the watchdog named it, if too long, or None.

    The watchdog says how long the sensor has sent the same thing. Longer than since its
    last message: it was frozen before the fault was made, so its word is not about the
    fault (Ravensim's cameras on stands send one picture again and again, TODO 274).
    """
    frozen = re.search(rf'\b{re.escape(name)} FROZEN \(the same for ([\d.]+) s', detail)
    said = SAID.search(detail)
    if frozen and said and float(frozen[1]) > float(said['said']):
        return float(frozen[1])
    return None


def fault_times(detail):
    """
    Return what a fault step's sentence says, as a dict.

    stopped (True, False, or None: it does not say), seconds (or None), limit (or None),
    and `from`: what the clock counts from, in words.
    """
    out = {'stopped': None, 'seconds': None, 'limit': None, 'from': ''}
    if 'NEVER still' in detail or "wasn't seen still" in detail:
        out['stopped'] = False
        return out
    found = AFTER_WORD.search(detail)
    if found:
        said = SAID.search(detail)
        since = found['since'].replace('its last message', "the sensor's last message")
        out.update(stopped=True, seconds=float(found['took']), limit=float(found['limit']),
                   **{'from': 'the moment something said why' + (
                       f". It said why {float(said['said']):.2f} s after "
                       f"{since}; the car was still {float(found['total']):.2f} s after "
                       f'{since}' if said else '')})
        return out
    found = AFTER_FAULT.search(detail)
    if found:
        out.update(stopped=True, seconds=float(found['took']),
                   limit=float(found['limit']) if found['limit'] else None,
                   **{'from': found['since']})
    return out


def faults_table(steps):
    """Return the faults on purpose: a row each, did it stop the car, and in how long."""
    from aav_actuator_tests import car_test as ct
    known = {s.name: s for s in ct.STEPS if s.how is ct.How.FAULT}
    rows = []
    for s in steps:
        if s['step'] not in known:
            continue
        row = {'name': s['step'], 'number': str(s['number']),
               'fault': known[s['step']].title, 'status': s['status'], 'stopped': '',
               'seconds': NOT_MEASURED, 'limit': '', 'from': '', 'car': NO_CAR}
        if s['status'] in ('SKIPPED', 'NOT RUN'):
            row.update(stopped='not tried', seconds='', **{'from': s.get('detail', '')})
        else:
            seen = fault_times(s.get('detail', ''))
            row['stopped'] = {True: 'yes', False: 'NO', None: NOT_MEASURED}[seen['stopped']]
            if seen['seconds'] is not None:
                row['seconds'] = f"{seen['seconds']:.2f} s"
            if seen['limit'] is not None:
                row['limit'] = f"{seen['limit']:g} s"
            row['from'] = seen['from'] or (
                "the run's sentence gives no time: see the full table")
            detail = s.get('detail', '')
            if seen['stopped'] and BY_TEST in detail:
                row['stopped'] = BY_THE_TEST
            frozen = began_before(s['step'], detail)
            if frozen is not None:
                row.update(stopped=NOT_SHOWN, **{'from': (
                    f'the watchdog called it frozen for {frozen:g} s already, from before '
                    'the fault was made: this does not show that losing it is caught '
                    '(TODO 274). ' + row['from'])})
            if seen['stopped'] and s['status'] == 'FAIL':
                row['stopped'] = 'yes, too late or unexplained (FAIL)'
        rows.append(row)
    return {'columns': ['fault on purpose', 'was the car stopped?', 'in how long', 'allowed',
                        'counted from'],
            'rows': rows,
            'missing': '' if rows else f'{NOT_MEASURED}: this run tried no fault',
            'car': NO_CAR}


def verdict(steps, expected, faults):
    """
    Return the verdict: `state` (PASS, FAIL, NOT FINISHED, ONE TEST ONLY) and one line of words.

    The run's own results decide, by the ladder's rule: no step failed, none left not
    run, at least one passed. `expected`: the steps a whole run has, by name.
    """
    by = {s['step']: s['status'] for s in steps}
    names = {s['step']: f"{s['number']} {s['step']}" for s in steps}
    failed = [names[n] for n, status in by.items() if status == 'FAIL']
    not_run = [names[n] for n, status in by.items() if status == 'NOT RUN']
    absent = [n for n in expected if n not in by]
    tried = [r for r in faults['rows'] if r['stopped'] != 'not tried']
    stopped = [r for r in tried if r['status'] == 'PASS']
    skipped = [r['fault'] for r in faults['rows'] if r['status'] == 'SKIPPED']

    def follows(part, group):
        mine = [by[n] for n in group if n in by]
        if not mine:
            return f'{part}: not tested'
        good = ('the wheels follow each speed step up (a step down is not judged)'
                if part == 'the wheels' else 'the steering follows its commands')
        return good if all(m == 'PASS' for m in mine) else f'{part}: NOT passed'

    parts = [follows('the wheels', WHEELS), follows('the steering', STEERING)]
    if tried:
        unshown = [r for r in tried if r['stopped'] == NOT_SHOWN]
        caught = [r for r in stopped if r['stopped'] == BY_THE_TEST]
        if len(stopped) < len(tried):
            parts.append(f'only {len(stopped)} of {len(tried)} faults tried passed')
        else:
            parts.append(f'{len(tried) - len(unshown)} of {len(tried)} faults tried were '
                         'caught in time' + (
                             f' ({len(tried) - len(unshown) - len(caught)} stopped the car '
                             f'by themselves; for {len(caught)} the watchdog named the lost '
                             'sensor and the test stopped the car)' if caught else ''))
        if unshown:
            parts.append('NOT SHOWN: ' + ', '.join(r['fault'] for r in unshown))
    else:
        parts.append('no fault was tried')
    words = '; '.join(parts) + '.'
    if skipped:
        words += f" Not tried here: {'; '.join(skipped)}."
    if failed:
        state, words = 'FAIL', f"failed: {', '.join(failed)}. " + words[0].upper() + words[1:]
    elif absent:
        state = 'ONE TEST ONLY'
        words = (f"this run was {', '.join(names.values())} alone, not the whole rung "
                 f'({len(absent)} steps not in it): it does not count. ' + words)
    elif not_run or 'PASS' not in by.values():
        state = 'NOT FINISHED'
        words = f"{len(not_run)} steps did not run ({', '.join(not_run)}). " + words
    else:
        state = 'PASS'
    return {'state': state, 'words': f'{state}: {words}'}


def answer(rung, folder, entry=None, points=SCREEN_POINTS):
    """
    Return a rung's answer from one run's folder, or None for a rung with no builder.

    entry: the ladder's own line for this run (its commit and date), or None: the run was
    not recorded (a single test, or a run from a terminal).
    """
    if rung.get('tests') != 'stands':
        return None
    out = {'rung': rung['id'], 'number': str(rung['number']), 'title': rung['title'],
           'question': rung['question'].strip(), 'passes': rung['passes'].strip(),
           'chapter': rung['chapter'], 'found': False, 'why': '', 'folder': folder or '',
           'started': '', 'place': '', 'where': '', 'commit': '', 'command': '',
           'verdict': {'state': '', 'words': ''}, 'graphs': [], 'faults': None, 'steps': [],
           'car': NO_CAR, 'report_file': ''}
    path = os.path.join(folder, 'report.yaml') if folder else ''
    if not path or not os.path.isfile(path):
        out['why'] = 'No run yet: press Run, and the answer is drawn here when it ends.'
        return out
    with open(path) as f:
        report = yaml.safe_load(f) or {}
    steps = report.get('steps', [])
    sim = os.path.basename(folder.rstrip('/')).endswith('-sim')
    import ladder_report
    expected = [name for name, _, _ in ladder_report.single_tests('stands')] if sim else []
    faults = faults_table(steps)
    page = os.path.join(folder, file_name(rung))
    out.update(
        found=True, started=str(report.get('started', '')),
        place='the simulator' if sim else 'the car', where=report.get('where', ''),
        commit=(entry or {}).get('commit', '') or
        'not recorded: the ladder did not start this run, so nothing wrote its commit down',
        command=rung['sim' if sim else 'car'] if entry else
        'not recorded: the ladder did not start this run',
        verdict=verdict(steps, expected, faults),
        graphs=[wheels_graph(folder, steps, points), steering_graph(folder, steps, points),
                brake_graph(folder, steps, points)],
        faults=faults,
        steps=[{'number': str(s.get('number', '')), 'name': s['step'], 'status': s['status'],
                'detail': s.get('detail', '')} for s in steps],
        report_file=page if os.path.isfile(page) else '')
    return out


def save(rung, folder, entry=None):
    """Write the run's report page into its folder; return its path, None with no answer."""
    found = answer(rung, folder, entry, points=PAGE_POINTS)
    if found is None or not found['found']:
        return None
    path = os.path.join(folder, file_name(rung))
    found['report_file'] = path
    with open(path, 'w') as f:
        f.write(page(found))
    return path


# --- the page ------------------------------------------------------------------------

# The charts' light palette: series 1 (blue) and 2 (orange); the lines differ by their
# dashes too, and the legend names them in words.
BLUE, ORANGE, INK, MUTED, GRID, SURFACE = ('#2a78d6', '#eb6834', '#0b0b0b', '#52514e',
                                           '#dddcd6', '#fcfcfb')
W, H, LEFT, RIGHT, TOP, BOTTOM = 760, 300, 62, 16, 34, 46


def ticks(low, high, wanted):
    """Return round numbers across a range, about `wanted` of them."""
    width = high - low
    if width <= 0:
        return [low]
    rough = width / wanted
    power = 10 ** math.floor(math.log10(rough))
    step = next(m * power for m in (1, 2, 5, 10) if m * power >= rough)
    first = math.ceil(low / step - 1e-9)
    return [round(i * step, 10) for i in range(first, math.floor(high / step + 1e-9) + 1)]


def svg(g):
    """Return one graph as an SVG picture: asked for (dashed) against measured (solid)."""
    values = g['asked'] + g['measured'] + [0.0]
    low, high = min(values), max(values)
    room = max((high - low) * 0.08, 0.01)
    low, high = (low - room if low < 0 else 0.0), high + room
    t0, t1 = g['t'][0], g['t'][-1]

    def x(t):
        return LEFT + (W - LEFT - RIGHT) * (t - t0) / ((t1 - t0) or 1.0)

    def y(v):
        return H - BOTTOM - (H - TOP - BOTTOM) * (v - low) / (high - low)

    def line(vals):
        return ' '.join(f'{x(t):.1f},{y(v):.1f}' for t, v in zip(g['t'], vals))

    esc = html_lib.escape
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" '
           f'height="{H}" role="img" aria-label="{esc(g["title"])}" '
           f'font-family="system-ui, sans-serif" font-size="13">',
           f'<rect width="{W}" height="{H}" fill="{SURFACE}"/>']
    for v in ticks(low, high, 4):
        out.append(f'<line x1="{LEFT}" x2="{W - RIGHT}" y1="{y(v):.1f}" y2="{y(v):.1f}" '
                   f'stroke="{GRID}"/><text x="{LEFT - 8}" y="{y(v) + 4:.1f}" '
                   f'text-anchor="end" fill="{MUTED}">{v:g}</text>')
    for t in ticks(t0, t1, 8):
        out.append(f'<line x1="{x(t):.1f}" x2="{x(t):.1f}" y1="{H - BOTTOM}" '
                   f'y2="{H - BOTTOM + 5}" stroke="{MUTED}"/><text x="{x(t):.1f}" '
                   f'y="{H - BOTTOM + 19}" text-anchor="middle" fill="{MUTED}">{t:g}</text>')
    out += [f'<line x1="{LEFT}" x2="{W - RIGHT}" y1="{H - BOTTOM}" y2="{H - BOTTOM}" '
            f'stroke="{MUTED}"/>',
            f'<text x="4" y="14" fill="{INK}">{esc(g["y_label"])}</text>',
            f'<text x="{(LEFT + W - RIGHT) / 2}" y="{H - 6}" text-anchor="middle" '
            f'fill="{INK}">{esc(g["x_label"])}</text>',
            # the legend: a sample of each line, then its name in ink
            f'<line x1="{W - 400}" x2="{W - 364}" y1="10" y2="10" stroke="{ORANGE}" '
            f'stroke-width="2" stroke-dasharray="8 5"/><text x="{W - 358}" y="14" '
            f'fill="{INK}">{esc(g["asked_words"])}</text>',
            f'<line x1="{W - 200}" x2="{W - 164}" y1="10" y2="10" stroke="{BLUE}" '
            f'stroke-width="2"/><text x="{W - 158}" y="14" fill="{INK}">'
            f'{esc(g["measured_words"])}</text>',
            f'<polyline fill="none" stroke="{ORANGE}" stroke-width="2" '
            f'stroke-dasharray="8 5" points="{line(g["asked"])}"/>',
            f'<polyline fill="none" stroke="{BLUE}" stroke-width="2" '
            f'stroke-linejoin="round" points="{line(g["measured"])}"/>', '</svg>']
    return '\n'.join(out)


def table_html(columns, rows):
    esc = html_lib.escape
    head = ''.join(f'<th>{esc(c)}</th>' for c in columns)
    head = f'<thead><tr>{head}</tr></thead>' if any(columns) else ''
    body = ''.join('<tr>' + ''.join(f'<td>{esc(str(c))}</td>' for c in row) + '</tr>'
                   for row in rows)
    return f'<table>{head}<tbody>{body}</tbody></table>'


CSS = """
body { font: 17px/1.5 system-ui, sans-serif; color: #0b0b0b; background: #fff;
       max-width: 1180px; margin: 2em auto; padding: 0 1em; }
h1 { font-size: 1.7em; margin-bottom: 0.2em; } h2 { margin-top: 1.8em; }
.question { font-size: 1.25em; }
.verdict { font-size: 1.2em; font-weight: 700; padding: 0.7em 1em; border: 3px solid #0b0b0b;
           border-radius: 6px; }
.verdict.PASS { border-color: #0ca30c; } .verdict.FAIL { border-color: #d03b3b; }
.muted { color: #52514e; }
table { border-collapse: collapse; margin: 0.6em 0; }
th, td { border: 1px solid #cfcec8; padding: 0.3em 0.7em; text-align: left;
         vertical-align: top; }
th { background: #f3f2ee; }
.pair { display: grid; grid-template-columns: minmax(0, 3fr) minmax(12em, 1fr); gap: 1.2em;
        align-items: start; }
.place { border: 1px solid #cfcec8; border-radius: 6px; padding: 0.6em 0.9em; }
.place h3 { margin: 0 0 0.4em; font-size: 1em; }
svg { max-width: 100%; height: auto; }
ul.notes { color: #52514e; font-size: 0.9em; padding-left: 1.2em; }
code { background: #f3f2ee; padding: 0 0.25em; }
"""


def pair(simulator, place):
    """Return the two columns: this run's side filled, the other side's slot."""
    car = (f'<p class="muted">{NO_CAR.capitalize()}. The car&#x27;s graph and numbers go '
           'here, beside the simulator&#x27;s, once this rung has run on the car.</p>')
    mine, other = ('In the simulator', 'On the car') if place != 'the car' else (
        'On the car', 'In the simulator')
    return (f'<div class="pair"><div class="place"><h3>{mine}</h3>{simulator}</div>'
            f'<div class="place"><h3>{other}</h3>'
            f'{car if other == "On the car" else "<p class=muted>Not on this page.</p>"}'
            '</div></div>')


def page(a):
    """Return the answer as one HTML page: plain headings, tables and pictures, no script."""
    esc = html_lib.escape
    out = ['<!doctype html><html lang="en"><head><meta charset="utf-8">',
           f'<title>Rung {esc(a["number"])}: {esc(a["title"])}</title>',
           f'<style>{CSS}</style></head><body>',
           f'<h1>Rung {esc(a["number"])}: {esc(a["title"])}</h1>',
           f'<p class="question"><b>The question:</b> {esc(a["question"])}</p>',
           f'<p class="verdict {esc(a["verdict"]["state"])}">{esc(a["verdict"]["words"])}</p>',
           f'<p><b>Judged against (the pass mark):</b> {esc(a["passes"])}</p>',
           table_html(['', ''], [
               ['When', a['started']], ['Where', f"{a['place']}: {a['where']}"],
               ['Commit', a['commit']], ['Command', a['command']],
               ["The run's folder", a['folder']],
               ['For the final report, chapter', a['chapter']]])]
    for g in a['graphs']:
        out.append(f'<h2>{esc(g["title"])}</h2><p>{esc(g["asks"])}</p>')
        if g['missing']:
            inner = f'<p><b>{esc(g["missing"])}</b></p>'
        else:
            inner = svg(g) + table_html(g['columns'], g['rows'])
            if g['facts']:
                inner += table_html(['', ''], g['facts'])
            inner += f'<p>{esc(g["judged"])}</p>'
            notes = [n for n in g['notes'] if n] + [f'From {g["from"]}.']
            inner += '<ul class="notes">' + ''.join(f'<li>{esc(n)}</li>' for n in notes) + '</ul>'
        out.append(pair(inner, a['place']))
    faults = a['faults']
    out.append('<h2>Faults on purpose: did each stop the car?</h2>'
               '<p>The run breaks one thing at a time while the wheels turn, and times '
               'how long the car takes to stand still.</p>')
    if faults['missing']:
        inner = f'<p><b>{esc(faults["missing"])}</b></p>'
    else:
        inner = table_html(faults['columns'], [
            [f"{r['number']} {r['fault']}", r['stopped'], r['seconds'], r['limit'], r['from']]
            for r in faults['rows']])
        inner += ("<ul class=\"notes\"><li>From report.yaml: each fault step's result, and "
                  "the times in the runner's own sentence.</li></ul>")
    out.append(pair(inner, a['place']))
    out.append('<h2>Every step of the run</h2><details><summary>The full table '
               f'({len(a["steps"])} steps)</summary>')
    out.append(table_html(['#', 'step', 'result', 'what happened'], [
        [s['number'], s['name'], s['status'], s['detail']] for s in a['steps']]))
    out.append('</details></body></html>')
    return '\n'.join(out) + '\n'
