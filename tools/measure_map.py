#!/usr/bin/env python3
r"""
The measuring map: a tape-measure route round the car, and its pictures.

    pixi run measure-map                         draw docs/img/measure-map-*.png, and
                                                 write the route table into
                                                 docs/measure-the-car.md
    pixi run measure-map --sheet                 print the route table (markdown)
    pixi run measure-map --blank > readings.yaml a readings file to fill in at the car
    pixi run measure-map --readings readings.yaml
                                                 turn the readings into vehicle.yaml's numbers

The route starts at base_link (the X on the floor under the middle of the rear axle) and
walks round the car. Each step is measured from an earlier mark, so nobody has to
measure the roof from the floor. This script adds the steps up into the numbers
vehicle.yaml wants: x forward, y left, z up, metres, from base_link (REP-103, REP-105).

Readings can be in metres or feet and inches: '1.57', '157 cm', "5 ft 2 in", "5' 2\"".
Page: docs/measure-the-car.md, "The measuring map". No ROS needed: PyYAML, and
matplotlib for the pictures (in Pixi's default environment).
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
VEHICLE_YAML = ROOT / 'ros/bringup/aav_description/config/vehicle.yaml'
DOC = ROOT / 'docs' / 'measure-the-car.md'
IMG = ROOT / 'docs' / 'img'
START = '<!-- measure-map:start (pixi run measure-map writes this table) -->'
END = '<!-- measure-map:end -->'

AXES = ('forward', 'left', 'up')          # x, y, z: + forward, + left, + up
BACK = {'forward': 'back', 'left': 'right', 'up': 'down'}


@dataclass(frozen=True)
class Mark:
    """A spot on or round the car that the tape reaches."""

    name: str
    what: str                 # what to put the tape on


@dataclass(frozen=True)
class Step:
    """One tape reading: from an earlier mark to a new one, along the named axes."""

    number: int
    start: str                # the mark it starts from
    end: str                  # the mark it reaches
    axes: tuple               # which of forward, left, up the tape measures
    gives: str                # the vehicle.yaml key it fills in
    how: str                  # a hint for the person at the car


MARKS = {m.name: m for m in (
    Mark('X', 'the X on the floor: base_link, under the middle of the rear axle'),
    Mark('rear-left wheel', 'the floor under the rear-left hub centre'),
    Mark('rear-left tyre top', 'the top of the rear-left tyre'),
    Mark('left side', "the body's widest point on the left, plumbed to the floor"),
    Mark('front-left wheel', 'the floor under the front-left hub centre'),
    Mark('front-right wheel', 'the floor under the front-right hub centre'),
    Mark('rear bumper', 'the floor under the rear bumper, on the x line'),
    Mark('rear camera', "the rear camera's lens centre"),
    Mark('front bumper', 'the floor under the front bumper, on the x line'),
    Mark('radar', "the middle of the radar's flat front face"),
    Mark('IMU', "the IMU board's centre"),
    Mark('roof', 'the top of the roof, straight above the X (plumb line onto the X)'),
    Mark('GPS', "the GPS antenna puck's centre (not the box)"),
    Mark('lidar', "the middle of the lidar's window"),
    Mark('left camera', "the front-left camera's lens centre"),
    Mark('right camera', "the front-right camera's lens centre"),
)}

ROUTE = (
    Step(1, 'X', 'rear-left wheel', ('left',), 'vehicle.track',
         'along the rear axle line; the yaml takes 2 x this'),
    Step(2, 'rear-left wheel', 'rear-left tyre top', ('up',), 'vehicle.wheel_diameter',
         "the car's weight on the tyre"),
    Step(3, 'rear-left wheel', 'rear-left wheel', (), 'vehicle.wheel_width',
         'across the tread: a size, not a move'),
    Step(4, 'rear-left wheel', 'left side', ('left',), 'vehicle.body.width',
         'tyre centre out to the body; the yaml takes 2 x (track/2 + this)'),
    Step(5, 'rear-left wheel', 'front-left wheel', ('forward',), 'vehicle.wheelbase',
         'hub to hub, same side'),
    Step(6, 'front-left wheel', 'front-right wheel', ('left',), 'check: vehicle.track',
         'the front track: a check on step 1, should be 2 x step 1'),
    Step(7, 'X', 'rear bumper', ('forward',), 'vehicle.body.rear_overhang',
         'along the x line, backwards'),
    Step(8, 'rear bumper', 'rear camera', AXES, 'sensors.camera_rear_link.xyz',
         'plumb line from the lens'),
    Step(9, 'rear bumper', 'front bumper', ('forward',), 'vehicle.body.length',
         'bumper to bumper'),
    Step(10, 'front bumper', 'radar', AXES, 'sensors.radar_link.xyz',
         'plumb line from the radar face'),
    Step(11, 'X', 'IMU', AXES, 'sensors.imu_link.xyz', 'plumb line from the board'),
    Step(12, 'X', 'roof', ('up',), 'vehicle.body.height',
         'the roof is flat? if not, also note its highest point'),
    Step(13, 'roof', 'GPS', AXES, 'sensors.gps_link.xyz', 'on the roof, from the roof mark'),
    Step(14, 'roof', 'lidar', AXES, 'sensors.lidar_link.xyz', 'on the roof, from the roof mark'),
    Step(15, 'roof', 'left camera', AXES, 'sensors.camera_left_link.xyz',
         'on the roof, from the roof mark'),
    Step(16, 'left camera', 'right camera', AXES, 'sensors.camera_right_link.xyz',
         'lens to lens (about the stereo baseline)'),
)

# Each step's row on the sheet below the map (sections 3 and 5 of measure-the-car.md).
SHEET_ROW = {1: 'M2', 2: 'M3', 3: 'M3b', 4: 'M4c', 5: 'M1', 6: 'M2b', 7: 'M4b', 8: 'S6',
             9: 'M4', 10: 'S3', 11: 'S2', 12: 'M4d', 13: 'S1', 14: 'S7', 15: 'S4', 16: 'S5'}

# The rest of vehicle.yaml is not a tape-measure job: where each is measured instead.
NOT_ON_THE_ROUTE = {
    'vehicle.max_steering_angle': 'M5: full lock, the two front wheel angles',
    'vehicle.max_reverse_speed': 'a choice, not a measurement',
    'vehicle.mass': 'M7: four scales',
    'vehicle.centre_of_mass': 'M7 and M7b: the scales, then the rear lifted',
    'vehicle.tyre_grip': 'test 4.3 (docs/tests.md)',
    'vehicle.encoder_counts_per_rev': 'count the spokes on the encoder wheel',
    'vehicle.speed_wheels': 'look: which wheels carry the speed sensors (TODO 49b)',
    'actuators': 'tests 3.10-3.12 and 4.1-4.5 (docs/tests.md)',
    'sensors.*.rpy': 'section 5: pointing, tilt, lean',
}


# --- reading vehicle.yaml ------------------------------------------------------------

def load(path=VEHICLE_YAML):
    """Return vehicle.yaml as a dict."""
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def get(cfg, key):
    """Return cfg['a']['b'] for key 'a.b'."""
    for part in key.split('.'):
        cfg = cfg[part]
    return cfg


def guesses(path=VEHICLE_YAML):
    """
    Return the dotted keys whose line (or the comment lines under it) says TODO or GUESS.

    A key's parent counts too: `imu_link:  # TODO` makes `sensors.imu_link.xyz` a guess.
    """
    out, stack, last = set(), [], None
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        body = line.split('#', 1)[0]
        comment = line.split('#', 1)[1] if '#' in line else ''
        flagged = bool(re.search(r'\b(TODO|GUESS)\b', comment))
        if not body.strip():                     # a comment line: belongs to the last key
            if flagged and last and line.startswith(' ') and stack:
                out.add(last)
            if not line.startswith(' '):
                last = None
            continue
        indent = len(body) - len(body.lstrip())
        name = body.strip().split(':', 1)[0]
        stack = [(i, n) for i, n in stack if i < indent] + [(indent, name)]
        last = '.'.join(n for _, n in stack)
        if flagged:
            out.add(last)
    return out


def is_guess(key, flagged):
    """Return True if key, or a key above it, is marked TODO or GUESS."""
    key = key.replace('check: ', '')
    parts = key.split('.')
    return any('.'.join(parts[:i]) in flagged for i in range(1, len(parts) + 1))


# --- the marks' positions, and the steps between them --------------------------------

def positions(cfg):
    """Return every mark's (x, y, z) from base_link, as vehicle.yaml has them today."""
    v, b, s = cfg['vehicle'], cfg['vehicle']['body'], cfg['sensors']
    half = v['track'] / 2
    front = b['length'] - b['rear_overhang']
    return {
        'X': (0.0, 0.0, 0.0),
        'rear-left wheel': (0.0, half, 0.0),
        'rear-left tyre top': (0.0, half, v['wheel_diameter']),
        'left side': (0.0, b['width'] / 2, 0.0),
        'front-left wheel': (v['wheelbase'], half, 0.0),
        'front-right wheel': (v['wheelbase'], -half, 0.0),
        'rear bumper': (-b['rear_overhang'], 0.0, 0.0),
        'rear camera': tuple(s['camera_rear_link']['xyz']),
        'front bumper': (front, 0.0, 0.0),
        'radar': tuple(s['radar_link']['xyz']),
        'IMU': tuple(s['imu_link']['xyz']),
        'roof': (0.0, 0.0, b['height']),
        'GPS': tuple(s['gps_link']['xyz']),
        'lidar': tuple(s['lidar_link']['xyz']),
        'left camera': tuple(s['camera_left_link']['xyz']),
        'right camera': tuple(s['camera_right_link']['xyz']),
    }


def offset(pos, step):
    """Return a step's (forward, left, up) from its start mark to its end mark."""
    return tuple(e - s for s, e in zip(pos[step.start], pos[step.end]))


def readings_today(cfg):
    """Return each step's readings from today's vehicle.yaml: {step: {axis: metres}}."""
    pos = positions(cfg)
    out = {}
    for step in ROUTE:
        if step.axes:
            d = offset(pos, step)
            out[step.number] = {a: d[AXES.index(a)] for a in step.axes}
        else:
            out[step.number] = {'size': get(cfg, step.gives)}
    return out


def chain(readings):
    """
    Add the readings up, step by step, into each mark's (x, y, z) from base_link.

    A step not in readings (or from a mark not reached) leaves its mark out.
    """
    pos = {'X': (0.0, 0.0, 0.0)}
    for step in ROUTE:
        if not step.axes or step.number not in readings or step.start not in pos:
            continue
        got = readings[step.number]
        d = [got.get(a, 0.0) if a in step.axes else 0.0 for a in AXES]
        pos[step.end] = tuple(round(p + q, 6) for p, q in zip(pos[step.start], d))
    return pos


SENSOR_MARKS = (('rear camera', 'camera_rear_link'), ('radar', 'radar_link'),
                ('IMU', 'imu_link'), ('GPS', 'gps_link'), ('lidar', 'lidar_link'),
                ('left camera', 'camera_left_link'), ('right camera', 'camera_right_link'))


def to_yaml_values(readings):
    """
    Turn chained readings into vehicle.yaml's numbers: {dotted key: value}.

    Keys whose marks weren't all reached are left out.
    """
    pos = chain(readings)
    rules = {
        'vehicle.track': lambda: 2 * pos['rear-left wheel'][1],
        'check: vehicle.track (front)':
            lambda: pos['front-left wheel'][1] - pos['front-right wheel'][1],
        'vehicle.wheel_diameter':
            lambda: pos['rear-left tyre top'][2] - pos['rear-left wheel'][2],
        'vehicle.wheelbase': lambda: pos['front-left wheel'][0] - pos['rear-left wheel'][0],
        'vehicle.body.width': lambda: 2 * pos['left side'][1],
        'vehicle.body.rear_overhang': lambda: -pos['rear bumper'][0],
        'vehicle.body.length': lambda: pos['front bumper'][0] - pos['rear bumper'][0],
        'vehicle.body.height': lambda: pos['roof'][2],
    }
    for mark, frame in SENSOR_MARKS:
        rules[f'sensors.{frame}.xyz'] = lambda mark=mark: pos[mark]
    out = {}
    for step in ROUTE:
        if not step.axes and 'size' in readings.get(step.number, {}):
            out[step.gives] = round(readings[step.number]['size'], 3)
    for key, rule in rules.items():
        try:
            value = rule()
        except KeyError:
            continue
        out[key] = ([round(c, 3) + 0.0 for c in value] if isinstance(value, tuple)
                    else round(value, 3) + 0.0)
    return out


# --- feet and inches -----------------------------------------------------------------

UNITS = {'m': 1.0, 'cm': 0.01, 'mm': 0.001, 'ft': 0.3048, "'": 0.3048, 'feet': 0.3048,
         'foot': 0.3048, 'in': 0.0254, '"': 0.0254, 'inch': 0.0254, 'inches': 0.0254}
_PART = re.compile(r'(\d+(?:\.\d+)?)\s*(mm|cm|m|feet|foot|ft|\'|inches|inch|in|")?')


def metres(reading, default='m'):
    r"""Return a reading in metres: 1.57, '157 cm', '5 ft 2 in', "5' 2\"", '-3 in'."""
    if isinstance(reading, (int, float)):
        return float(reading) * UNITS[default]
    text = str(reading).strip().lower()
    sign = -1.0 if text.startswith('-') else 1.0
    text = text.lstrip('+-').strip()
    total, at = 0.0, 0
    for match in _PART.finditer(text):
        if text[at:match.start()].strip():
            raise ValueError(f'not a length: {reading!r}')
        total += float(match.group(1)) * UNITS[match.group(2) or default]
        at = match.end()
    if at == 0 or text[at:].strip():
        raise ValueError(f'not a length: {reading!r}')
    return sign * total


def parse_readings(data):
    """
    Turn a readings file's dict into {step: {axis: metres}}.

    A one-axis step takes a plain reading; a three-axis step a dict of forward, left, up
    (or back, right, down: those count as minus). `units:` sets bare numbers' unit.
    """
    default = str(data.get('units', 'm'))
    steps = {s.number: s for s in ROUTE}
    out = {}
    for key, value in data.items():
        if key == 'units' or value is None:
            continue
        step = steps[int(key)]
        if not isinstance(value, dict):
            axis = step.axes[0] if step.axes else 'size'
            value = {axis: value}
        got = {}
        for axis, reading in value.items():
            axis = str(axis)
            if reading is None:
                continue
            sign = 1.0
            if axis in BACK.values():
                axis, sign = next(a for a, b in BACK.items() if b == axis), -1.0
            if axis not in step.axes and axis != 'size':
                raise ValueError(f'step {step.number} does not measure {axis}')
            got[axis] = sign * metres(reading, default)
        if got:
            out[step.number] = got
    return out


# --- the sheet -----------------------------------------------------------------------

def direction(step, values):
    """Return 'left 0.39 m' or 'forward 0.90 m, up 0.30 m' for a step's readings."""
    if not step.axes:
        return f"across: {values['size']:.2f} m"
    parts = []
    for axis in step.axes:
        v = values[axis]
        word = axis if v >= 0 else BACK[axis]
        parts.append(f'{word} {abs(v):.2f}')
    return ', '.join(parts) + ' m'


def yaml_value(cfg, key):
    """Return today's vehicle.yaml value for a key, as text."""
    value = get(cfg, key.replace('check: ', ''))
    if isinstance(value, list):
        return '[' + ', '.join(f'{c:.2f}' for c in value) + ']'
    return f'{value:.2f}'


def sheet(cfg=None, flagged=None):
    """Return the route as a markdown table, from today's vehicle.yaml."""
    cfg = load() if cfg is None else cfg
    flagged = guesses() if flagged is None else flagged
    today = readings_today(cfg)
    rows = ["| Step | From | To | Tape: direction and today's guess | Hint | Sheet | "
            'Gives (vehicle.yaml) | Today |',
            '|---|---|---|---|---|---|---|---|']
    for step in ROUTE:
        guess = ' GUESS' if is_guess(step.gives, flagged) else ''
        to = step.end if step.axes else f'{step.end} (tread)'
        cells = (step.number, step.start, to, direction(step, today[step.number]), step.how,
                 SHEET_ROW[step.number], f'`{step.gives}`', yaml_value(cfg, step.gives) + guess)
        rows.append('| ' + ' | '.join(str(c) for c in cells) + ' |')
    return '\n'.join(rows) + '\n'


def blank():
    """Return a readings file to fill in, with today's guesses as comments."""
    cfg = load()
    today = readings_today(cfg)
    lines = ['# Readings for pixi run measure-map --readings <this file>.',
             '# Write each reading after its colon: 1.57, 157 cm, 5 ft 2 in, or 5\' 2".',
             "# Back, right or down: write 'back: 3 in' or a minus. Leave blank if not measured.",
             'units: m   # the unit for bare numbers: m, cm, ft or in', '']
    for step in ROUTE:
        lines.append(f'# {step.number}. {step.start} -> {step.end}: {step.how}. '
                     f'Today: {direction(step, today[step.number])}')
        if len(step.axes) > 1:
            lines.append(f'{step.number}:')
            lines += [f'  {a}:' for a in step.axes]
        else:
            lines.append(f'{step.number}:')
    return '\n'.join(lines) + '\n'


def write_doc(table, doc=DOC):
    """Put the table between the markers in docs/measure-the-car.md."""
    text = doc.read_text(encoding='utf-8')
    head, rest = text.split(START, 1)
    _, tail = rest.split(END, 1)
    doc.write_text(f'{head}{START}\n\n{table}\n{END}{tail}', encoding='utf-8')


# --- the pictures --------------------------------------------------------------------

SENSORS = {'IMU': 'imu_link', 'GPS': 'gps_link', 'radar': 'radar_link',
           'left camera': 'camera_left_link', 'right camera': 'camera_right_link',
           'rear camera': 'camera_rear_link', 'lidar': 'lidar_link'}
VIEWS = {   # name: (horizontal axis, vertical axis, horizontal flipped, title)
    'top': (0, 1, False, 'From above: front to the right, left to the top'),
    'side': (0, 2, False, 'From the left side: front to the right'),
    'rear': (1, 2, True, "From behind: the car's left on your left"),
}


def draw(cfg, flagged, out_dir=IMG):
    """Draw the top, side and rear views; return the files written."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import patches
    import matplotlib.pyplot as plt

    ink, muted, body_c, wheel_c = '#1f2328', '#6b7280', '#e7ecf2', '#3b4046'
    route_c, sensor_c, start_c = '#d9622b', '#2a6fb0', '#1a7f37'
    v, b = cfg['vehicle'], cfg['vehicle']['body']
    pos = positions(cfg)
    half, rad, ww = v['track'] / 2, v['wheel_diameter'] / 2, v['wheel_width']
    x0, x1 = -b['rear_overhang'], b['length'] - b['rear_overhang']
    paths = []
    for name, (h, k, flip, title) in VIEWS.items():
        fig, ax = plt.subplots(figsize=(12, 7) if name != 'rear' else (10, 9))

        def p(xyz, h=h, k=k):
            return xyz[h], xyz[k]

        # The body and the wheels.
        if name == 'top':
            ax.add_patch(patches.Rectangle((x0, -b['width'] / 2), b['length'], b['width'],
                                           fc=body_c, ec=muted, lw=1.5))
            for wx in (0.0, v['wheelbase']):
                for wy in (half, -half):
                    ax.add_patch(patches.Rectangle((wx - rad, wy - ww / 2), 2 * rad, ww,
                                                   fc=wheel_c, ec=wheel_c))
        elif name == 'side':
            ax.add_patch(patches.Rectangle((x0, rad), b['length'], b['height'] - rad,
                                           fc=body_c, ec=muted, lw=1.5))
            for wx in (0.0, v['wheelbase']):
                ax.add_patch(patches.Circle((wx, rad), rad, fc=wheel_c, ec=wheel_c))
        else:
            ax.add_patch(patches.Rectangle((-b['width'] / 2, rad), b['width'],
                                           b['height'] - rad, fc=body_c, ec=muted, lw=1.5))
            for wy in (half, -half):
                ax.add_patch(patches.Rectangle((wy - ww / 2, 0), ww, 2 * rad,
                                               fc=wheel_c, ec=wheel_c))
        if name != 'top':
            ax.axhline(0, color=ink, lw=1.2)

        # The frame first, so the labels can be placed in screen space.
        every = [p(xyz) for xyz in pos.values()]
        hs, vs = [e[0] for e in every], [e[1] for e in every]
        if name == 'top':
            hs += [x0, x1]
            vs += [-b['width'] / 2, b['width'] / 2]
        elif name == 'side':
            hs += [x0, x1]
            vs += [b['height']]
        else:
            hs += [-b['width'] / 2, b['width'] / 2]
            vs += [b['height']]
        pad = 0.45 if name == 'rear' else 0.35
        ax.set_xlim(min(hs) - pad - 0.4, max(hs) + pad + 0.6)
        ax.set_ylim(min(min(vs), -0.1) - pad, max(vs) + pad)
        if flip:                                  # seen from behind: +y (left) on the left
            ax.invert_xaxis()
        wheel_text = (f"wheelbase {v['wheelbase']:.2f}, track {v['track']:.2f}, "
                      f"wheel {v['wheel_diameter']:.2f} x {ww:.2f}")
        body_text = (f"body {b['length']:.2f} long, {b['width']:.2f} wide, "
                     f"{b['height']:.2f} high, rear overhang {b['rear_overhang']:.2f}")
        guess = ' (GUESS)' if is_guess('vehicle.body.length', flagged) else ''
        ax.set_title(f'{title}\n{body_text}{guess}\n{wheel_text}', fontsize=11, color=ink)
        axis_names = ['x forward (m)', 'y left (m)', 'z up (m)']
        ax.set_xlabel(axis_names[h] + (': left is on the left' if flip else ''), color=muted)
        ax.set_ylabel(axis_names[k], color=muted)
        ax.set_aspect('equal')
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        ax.grid(alpha=0.25)
        ax.text(0.01, 0.01, 'orange circles: the route steps, in order; '
                'blue dots: sensors [x, y, z from base_link, m]',
                transform=ax.transAxes, fontsize=8.5, color=muted)
        fig.tight_layout()
        fig.canvas.draw()                         # fixes the aspect, so screen spots hold
        renderer = fig.canvas.get_renderer()
        screen = ax.transData.transform

        # base_link, and the sensors' dots.
        taken = []                                # screen boxes already used

        def dot(xy, size=9):
            (u, w), r = screen(xy), size
            taken.append((u - r, w - r, u + r, w + r))

        sx, sy = p(pos['X'])
        ax.plot(sx, sy, marker='X', ms=16, color=start_c, mec='white', mew=1.5, zorder=5)
        dot((sx, sy), 12)
        for label in SENSORS:
            ax.plot(*p(pos[label]), 'o', ms=9, color=sensor_c, mec='white', mew=1.5, zorder=6)
            dot(p(pos[label]))

        def free(box):
            return sum(max(0, min(box[2], t[2]) - max(box[0], t[0]))
                       * max(0, min(box[3], t[3]) - max(box[1], t[1])) for t in taken)

        # The route: one numbered arrow per step; the number where it hits nothing.
        for step in ROUTE:
            if not step.axes:
                continue
            (a, c), (e, f) = p(pos[step.start]), p(pos[step.end])
            if abs(e - a) + abs(f - c) < 0.02:
                continue
            ax.annotate('', xy=(e, f), xytext=(a, c),
                        arrowprops={'arrowstyle': '-|>', 'color': route_c, 'lw': 1.8,
                                    'shrinkA': 3, 'shrinkB': 3})
            best = None
            for t in (0.5, 0.35, 0.65, 0.25, 0.75, 0.15, 0.85):
                u, w = screen((a + t * (e - a), c + t * (f - c)))
                box = (u - 11, w - 11, u + 11, w + 11)
                if best is None or free(box) < best[0]:
                    best = (free(box), t, box)
                if best[0] == 0:
                    break
            _, t, box = best
            taken.append(box)
            ax.text(a + t * (e - a), c + t * (f - c), str(step.number), color='white',
                    fontsize=9, fontweight='bold', ha='center', va='center', zorder=8,
                    bbox={'boxstyle': 'circle,pad=0.25', 'fc': route_c, 'ec': 'white',
                          'lw': 1.5})

        # The labels: each tries spots round its dot, and takes the first that's free.
        def label(text, xy, colour=ink, weight='normal', size=8.5, first=()):
            spots = [*first, (8, 6), (8, -14), (-8, 6), (-8, -14), (8, 20), (-8, 20), (8, -28),
                     (-8, -28), (8, 34), (-8, 34), (8, -42), (-8, -42), (8, 48), (-8, 48)]
            best = None
            for dx, dy in spots:
                note = ax.annotate(text, xy, xytext=(dx, dy), textcoords='offset points',
                                   fontsize=size, color=colour, fontweight=weight,
                                   ha='left' if dx > 0 else 'right', zorder=9)
                bb = note.get_window_extent(renderer)
                box = (bb.x0, bb.y0, bb.x1, bb.y1)
                inside = ax.bbox.x0 <= bb.x0 and bb.x1 <= ax.bbox.x1
                cost = free(box) + (0 if inside else 1e6)
                if best is None or cost < best[0]:
                    if best is not None:
                        best[2].remove()
                    best = (cost, box, note)
                else:
                    note.remove()
                if cost == 0:
                    break
            _, box, note = best
            taken.append(box)
            if abs(note.xyann[1]) > 15:
                ax.annotate('', xy, xytext=note.xyann, textcoords='offset points',
                            arrowprops={'arrowstyle': '-', 'color': muted, 'lw': 0.6})

        if name != 'top':
            label('roof mark', p(pos['roof']), muted, 'bold', 8.5, [(-8, 6), (-8, -14)])
        label('START: base_link (the X)', (sx, sy), start_c, 'bold', 10, [(10, -24)])
        for name_, frame in SENSORS.items():
            key = f'sensors.{frame}.xyz'
            text = f'{name_} [{yaml_value(cfg, key)[1:-1]}]'
            if is_guess(key, flagged):
                text += ' GUESS'
            label(text, p(pos[name_]))

        path = out_dir / f'measure-map-{name}.png'
        path.parent.mkdir(parents=True, exist_ok=True)
        # No date in the file: the same numbers draw the same picture.
        fig.savefig(path, dpi=110, metadata={'Software': None, 'Creation Time': None})
        plt.close(fig)
        paths.append(path)
    return paths


# --- the command ---------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--sheet', action='store_true', help='print the route table only')
    parser.add_argument('--blank', action='store_true', help='print a readings file')
    parser.add_argument('--readings', type=Path,
                        help="a readings file: print vehicle.yaml's numbers from it")
    parser.add_argument('--no-figures', action='store_true', help="don't draw the pictures")
    args = parser.parse_args(argv)
    if args.blank:
        print(blank(), end='')
        return 0
    if args.readings:
        data = yaml.safe_load(args.readings.read_text(encoding='utf-8')) or {}
        readings = parse_readings(data)
        values = to_yaml_values(readings)
        missing = [s.number for s in ROUTE if s.number not in readings]
        print('# For vehicle.yaml (metres, from base_link). Delete each TODO you replace.')
        for key, value in values.items():
            print(f'{key}: {value}')
        if missing:
            print(f'# Not measured yet: steps {missing} (and what is measured from them)')
        return 0
    table = sheet()
    if args.sheet:
        print(table, end='')
        return 0
    write_doc(table)
    print(f'wrote the route table into {DOC.relative_to(ROOT)}')
    if not args.no_figures:
        for path in draw(load(), guesses()):
            print(f'drew {path.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
