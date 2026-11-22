"""
Draw the barriers' runs as one HTML page of graphs (research, separate from the capstone).

    AAV_BARRIER_OUT=/tmp/barrier-traces cargo test --release -p mpc-lag --test barrier \
        the_traces -- --ignored          (in ros/drivers/aav_mpc_lag: writes the CSVs)
    (the same with the_sweep: 420 runs into sweep.csv, a few minutes on every core)
    python3 tools/barrier_graphs.py /tmp/barrier-traces out.html

No ROS, the standard library only. The page draws itself (inline SVG, a hover read-out, a
table under each graph); the colours are the fixed first four of our chart palette.
"""
import csv
import json
import sys
from pathlib import Path

# name in the CSVs -> (label, light colour, dark colour): a fixed order, never reused.
SERIES = [
    ('plain', 'No barrier', '#2a78d6', '#3987e5'),
    ('limit', 'Distance limit only (a step)', '#eb6834', '#d95926'),
    ('g005', 'Barrier, share 0.05', '#1baf7a', '#199e70'),
    ('g002', 'Barrier, share 0.02', '#eda100', '#c98500'),
]
OBJECT_X, OBJECT_RADIUS = 15.0, 0.5


def load(folder, scene, every=2):
    """Return {series: rows} for a scene, each row a dict of floats, thinned."""
    runs = {}
    for name, *_ in SERIES:
        path = folder / f'{scene}_{name}.csv'
        if path.exists():
            with path.open() as f:
                rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
            runs[name] = rows[::every] + rows[-1:]
    return runs


def series(runs, x, y, keep=lambda r: True):
    """Return each run as a line: column y against column x, the rows `keep` passes."""
    return [{'key': k, 'points': [[round(r[x], 3), round(r[y], 3)] for r in rows if keep(r)]}
            for k, rows in runs.items()]


def summary(runs):
    """One table row a series: the least gap, the top speed, where it ended."""
    out = []
    for key, rows in runs.items():
        gap = min(r['gap'] for r in rows)
        out.append([key, f'{gap:.2f}', f"{max(r['v'] for r in rows):.2f}",
                    f"{max(abs(r['cross']) for r in rows):.2f}", f"{rows[-1]['x']:.1f}",
                    'touched' if gap < 0 else ('stopped short' if rows[-1]['x'] < 30 else
                                               'reached the end')])
    return out


def build(folder):
    """Return the page's graphs, from the CSVs in `folder`."""
    left, slow, fast = (load(folder, s) for s in ('left_fast', 'centre_slow', 'centre_fast'))
    step = load(folder, 'step', every=1)
    near = lambda r: 4.0 <= r['x'] <= 28.0  # noqa: E731
    head = ['Run', 'Least gap (m)', 'Top speed (m/s)', 'Widest off the path (m)',
            'Ended at x (m)', 'Outcome']
    charts = [
        {'title': 'Going round: the path each run drove',
         'note': 'Object 0.4 m left of the path, cruise 3 m/s. The side-to-side scale is '
                 'stretched, so the round object looks tall.',
         'x': 'along the path (m)', 'y': 'left of the path (m)',
         'series': series(left, 'x', 'y', near), 'object': [OBJECT_X, 0.4, OBJECT_RADIUS],
         'head': head, 'rows': summary(left)},
        {'title': 'Slowing down: speed against position, head on at 1 m/s',
         'note': 'The object is dead on the path; its near edge is at 14.5 m. The car is '
                 '1.8 m long ahead of the point plotted, so it touches at 12.7 m.',
         'x': 'along the path (m)', 'y': 'speed (m/s)',
         'series': series(slow, 'x', 'v', lambda r: 8.0 <= r['x'] <= 16.0), 'mark': 12.7,
         'mark_label': 'touching', 'head': head, 'rows': summary(slow)},
        {'title': 'Room left: the gap to the object over time, head on at 1 m/s',
         'note': 'Below zero the car is inside the object.',
         'x': 'time (s)', 'y': 'gap to the object (m)',
         'series': series(slow, 't', 'gap', lambda r: 8.0 <= r['t'] <= 30.0), 'zero': True,
         'head': head, 'rows': summary(slow)},
        {'title': 'Head on at 3 m/s: speed against position',
         'note': 'Fast enough to see far ahead: the barrier runs go round and carry on; '
                 'the distance limit alone brakes late and stops.',
         'x': 'along the path (m)', 'y': 'speed (m/s)',
         'series': series(fast, 'x', 'v', lambda r: 4.0 <= r['x'] <= 28.0),
         'head': head, 'rows': summary(fast)},
    ]
    if len(step) == 2:
        a, b = step['plain'], step['g005']
        worst = max(abs(p['cross'] - q['cross']) for p, q in zip(a, b))
        calm = [abs(r['cross']) < 0.05 for r in a]
        settle = next((r['t'] for i, r in enumerate(a) if all(calm[i:])), None)
        charts.append(
            {'title': 'A step: the car starts 1 m off the path, nothing near',
             'note': f'With barriers on and no object near, the answer to a step is the '
                     f"plain tracker's: the two differ by at most {worst * 1000:.0f} mm. "
                     f'Within 5 cm of the path after {settle:.1f} s.',
             'x': 'time (s)', 'y': 'off the path (m)',
             'series': series({'plain': a}, 't', 'cross', lambda r: r['t'] <= 15.0),
             'zero': True, 'head': ['Run', 'Largest difference (mm)', 'Settled within 5 cm (s)'],
             'rows': [['plain', '0', f'{settle:.1f}'], ['g005', f'{worst * 1000:.0f}', '']]})
    return charts


# set in sweep.csv -> (the column swept, title, note, x label)
SWEEPS = [
    ('speed', 'speed', 'Every cruise speed: the least gap, object dead on the path',
     'Straight road, no dead time. Below zero the car touched the object.', 'cruise speed (m/s)'),
    ('bend', 'speed', 'The same on a bend (25 m radius)',
     'The object stands on the path, 15 m along a left bend.', 'cruise speed (m/s)'),
    ('offset_fast', 'offset', 'Every place the object can stand, at 3 m/s',
     'From 1.6 m right of the path to 1.6 m left of it.', 'object left of the path (m)'),
    ('offset_slow', 'offset', 'Every place the object can stand, at 1 m/s',
     'Slow: it stops for an object within 1.1 m of the path, and passes one further out.',
     'object left of the path (m)'),
    ('dead_time', 'dead_time', 'A slower car: dead time on steering and pedals, 3 m/s',
     'Object dead on the path. The tracker is told the dead time.', 'dead time (s)'),
]
ENDED = ['touched', 'stopped', 'moving', 'reached']


def sweep(folder):
    """Return (graphs, tally) from sweep.csv: the least gap across each sweep."""
    path = folder / 'sweep.csv'
    if not path.exists():
        return [], ''
    with path.open() as f:
        rows = list(csv.DictReader(f))
    keys = [k for k, *_ in SERIES]
    charts = []
    for name, column, title, note, label in SWEEPS:
        mine = [r for r in rows if r['set'] == name]
        lines = [{'key': k, 'points': [[float(r[column]), round(float(r['gap']), 3)]
                                       for r in mine if r['case'] == k]} for k in keys]
        table = [[k] + [f"{sum(r['case'] == k and r['ended'] == e for r in mine)}"
                        for e in ENDED]
                 + [f"{min(float(r['gap']) for r in mine if r['case'] == k):.2f}"]
                 for k in keys]
        charts.append({'title': title, 'note': note, 'x': label,
                       'y': 'least gap to the object (m)', 'series': lines, 'zero': True,
                       'head': ['Run'] + [e.capitalize() for e in ENDED] + ['Least gap (m)'],
                       'rows': table})
    names = {k: n for k, n, _, _ in SERIES}
    body = ''
    for k in keys:
        mine = [r for r in rows if r['case'] == k]
        late = sum(int(r['unsolved']) for r in mine) / sum(int(r['ticks']) for r in mine)
        body += (f'<tr><td>{names[k]}</td><td>{len(mine)}</td>'
                 + ''.join(f"<td>{sum(r['ended'] == e for r in mine)}</td>" for e in ENDED)
                 + f"<td>{min(float(r['gap']) for r in mine):.2f}</td>"
                 f'<td>{late * 100:.0f}%</td></tr>')
    tally = ('<figure><h2>All the runs of the sweep, counted</h2><p class="note">Touched: '
             'the body met the object. Stopped: safe, and stuck short of the end. Moving: '
             'still creeping when the time ran out. Reached: got to the end of the road. '
             'Last column: ticks where the solver ran out of turns before it settled.</p>'
             '<table><tr><th>Run</th><th>Runs</th>'
             + ''.join(f'<th>{e.capitalize()}</th>' for e in ENDED)
             + '<th>Least gap (m)</th><th>Unsettled ticks</th></tr>' + body
             + '</table></figure>')
    return charts, tally


W, H, L, R, T, B = 860, 340, 56, 16, 14, 44


def ticks(lo, hi, n):
    """Return round tick values from lo to hi, about n of them."""
    import math
    raw = (hi - lo) / n
    base = 10 ** math.floor(math.log10(raw))
    step = next((m * base for m in (1, 2, 5, 10) if (hi - lo) / (m * base) <= n), base * 10)
    first = math.ceil(lo / step - 1e-9) * step
    out, v = [], first
    while v <= hi + 1e-9:
        out.append(round(v, 6))
        v += step
    return out


def draw(c, names):
    """Return one graph as HTML: its title, legend, an SVG and its table."""
    pts = [p for s in c['series'] for p in s['points']]
    x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
    y0, y1 = min(0.0, *(p[1] for p in pts)), max(p[1] for p in pts)
    if 'object' in c:
        y1 = max(y1, c['object'][1] + c['object'][2])
        y0 = min(y0, c['object'][1] - c['object'][2])
    pad = (y1 - y0) * 0.08 or 0.1
    y1 += pad
    if y0 < 0:
        y0 -= pad

    def X(v):
        return L + (v - x0) / (x1 - x0) * (W - L - R)

    def Y(v):
        return H - B - (v - y0) / (y1 - y0) * (H - T - B)

    g = []
    for v in ticks(y0, y1, 5):
        g.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" '
                 f'stroke="var(--grid)"/><text x="{L - 8}" y="{Y(v) + 4:.1f}" '
                 f'text-anchor="end">{v:g}</text>')
    for v in ticks(x0, x1, 8):
        g.append(f'<text x="{X(v):.1f}" y="{H - B + 18}" text-anchor="middle">{v:g}</text>')
    mid = (T + H - B) / 2
    g.append(f'<text x="{(L + W - R) / 2}" y="{H - 6}" text-anchor="middle">{c["x"]}</text>')
    g.append(f'<text x="14" y="{mid}" text-anchor="middle" '
             f'transform="rotate(-90 14 {mid})">{c["y"]}</text>')
    if c.get('zero') and y0 < 0:
        g.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(0):.1f}" y2="{Y(0):.1f}" '
                 f'stroke="var(--ink2)" stroke-dasharray="4 4"/>')
    if 'object' in c:
        ox, oy, r = c['object']
        g.append(f'<ellipse cx="{X(ox):.1f}" cy="{Y(oy):.1f}" rx="{X(ox + r) - X(ox):.1f}" '
                 f'ry="{Y(oy) - Y(oy + r):.1f}" fill="var(--grid)" stroke="var(--ink2)"/>'
                 f'<text x="{X(ox):.1f}" y="{Y(oy + r) - 6:.1f}" text-anchor="middle">'
                 f'the object</text>')
    if 'mark' in c:
        g.append(f'<line x1="{X(c["mark"]):.1f}" x2="{X(c["mark"]):.1f}" y1="{T}" '
                 f'y2="{H - B}" stroke="var(--ink2)" stroke-dasharray="4 4"/>'
                 f'<text x="{X(c["mark"]) + 6:.1f}" y="{T + 12}">{c["mark_label"]}</text>')
    for s in c['series']:
        d = ''.join(f'{"L" if i else "M"}{X(x):.1f} {Y(y):.1f}'
                    for i, (x, y) in enumerate(s['points']))
        ex, ey = s['points'][-1]
        g.append(f'<path d="{d}" fill="none" stroke="var(--{s["key"]})" stroke-width="2" '
                 f'stroke-linejoin="round"/><circle cx="{X(ex):.1f}" cy="{Y(ey):.1f}" r="4" '
                 f'fill="var(--{s["key"]})" stroke="var(--surface)" stroke-width="2"/>')
    g.append(f'<line class="hair" y1="{T}" y2="{H - B}" stroke="var(--ink2)" '
             f'visibility="hidden"/>')
    legend = ''
    if len(c['series']) > 1:
        legend = '<div class="legend">' + ''.join(
            f'<span><i style="background:var(--{s["key"]})"></i>{names[s["key"]]}</span>'
            for s in c['series']) + '</div>'
    table = ('<details><summary>The numbers</summary><table><tr>'
             + ''.join(f'<th>{h}</th>' for h in c['head']) + '</tr>'
             + ''.join('<tr>' + ''.join(f'<td>{names[v] if i == 0 else v}</td>'
                                        for i, v in enumerate(r)) + '</tr>'
                       for r in c['rows']) + '</table></details>')
    scale = json.dumps({'x0': x0, 'x1': x1, 'x': c['x'], 'series': c['series']})
    return (f'<figure><h2>{c["title"]}</h2><p class="note">{c["note"]}</p>{legend}'
            f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{c["title"]}" '
            f"data-chart='{scale}'>{''.join(g)}</svg>{table}</figure>")


PAGE = """<!doctype html><meta charset="utf-8"><title>Barriers: the graphs</title>
<style>
:root{--surface:#fcfcfb;--page:#f9f9f7;--ink:#0b0b0b;--ink2:#52514e;--grid:#e4e3de;__LIGHT__}
@media (prefers-color-scheme:dark){:root{--surface:#1a1a19;--page:#0d0d0d;--ink:#f0efec;
--ink2:#c3c2b7;--grid:#383835;__DARK__}}
body{background:var(--page);color:var(--ink);font:17px/1.6 -apple-system,Verdana,sans-serif;
max-width:60em;margin:2em auto;padding:0 1em}
figure{background:var(--surface);margin:2em 0;padding:1em 1.2em;border-radius:8px}
h2{font-size:1.15em;margin:0 0 .2em}p.note{color:var(--ink2);margin:.2em 0 .6em}
.legend{display:flex;flex-wrap:wrap;gap:.3em 1.4em;margin:.4em 0}
.legend i{display:inline-block;width:1.4em;height:3px;vertical-align:middle;margin-right:.4em}
svg{width:100%;height:auto;display:block}text{fill:var(--ink2);font-size:13px}
.tip{position:absolute;background:var(--surface);border:1px solid var(--grid);padding:.3em .6em;
border-radius:6px;font-size:14px;pointer-events:none;display:none;white-space:nowrap}
.key{display:inline-block;width:.8em;height:.8em;margin-right:.4em}
table{border-collapse:collapse;margin:.6em 0}td,th{border:1px solid var(--grid);
padding:.3em .7em;text-align:left}details{margin-top:.4em}
</style>
<h1>Barriers in the MPC: the graphs</h1>
<p>A pretend car (our model of the car, not the car), a straight 40 m path, a round object
0.5 m in radius at 15 m. Each line is one run. Move the pointer over a graph to read values.</p>
__FIGURES__<div class="tip" id="tip"></div>
<script>
const NAMES=__NAMES__, W=860, L=56, R=16, tip=document.getElementById('tip');
document.querySelectorAll('svg[data-chart]').forEach(svg=>{
  const c=JSON.parse(svg.dataset.chart), hair=svg.querySelector('.hair');
  const off=()=>{hair.setAttribute('visibility','hidden');tip.style.display='none'};
  svg.addEventListener('mousemove',e=>{const b=svg.getBoundingClientRect();
    const px=(e.clientX-b.left)/b.width*W, v=c.x0+(px-L)/(W-L-R)*(c.x1-c.x0);
    if(v<c.x0||v>c.x1)return off();
    hair.setAttribute('x1',px);hair.setAttribute('x2',px);hair.setAttribute('visibility','visible');
    let h='<b>'+c.x+': '+v.toFixed(1)+'</b>';
    c.series.forEach(s=>{const p=s.points.reduce((a,q)=>Math.abs(q[0]-v)<Math.abs(a[0]-v)?q:a);
      if(Math.abs(p[0]-v)<(c.x1-c.x0)/40)
        h+='<br><i class="key" style="background:var(--'+s.key+')"></i>'
          +NAMES[s.key]+': '+p[1].toFixed(2)});
    tip.innerHTML=h;tip.style.display='block';tip.style.left=(e.pageX+14)+'px';tip.style.top=(e.pageY+10)+'px'});
  svg.addEventListener('mouseleave',off)});
</script>
"""


def main(argv):
    """Write the page: `barrier_graphs.py <the CSVs' folder> <out.html>`."""
    if len(argv) != 3:
        print(__doc__)
        return 2
    charts = build(Path(argv[1]))
    swept, tally = sweep(Path(argv[1]))
    charts += swept
    names = {k: n for k, n, _, _ in SERIES}
    page = (PAGE.replace('__LIGHT__', ''.join(f'--{k}:{c};' for k, _, c, _ in SERIES))
            .replace('__DARK__', ''.join(f'--{k}:{c};' for k, _, _, c in SERIES))
            .replace('__NAMES__', json.dumps(names))
            .replace('__FIGURES__', ''.join(draw(c, names) for c in charts) + tally))
    Path(argv[2]).write_text(page)
    print(f'{len(charts)} graphs in {argv[2]}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
