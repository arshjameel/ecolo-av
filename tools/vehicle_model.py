#!/usr/bin/env python3
"""
The car's linear model: print its poles, step responses and margins, and draw them.

    pixi run vehicle-model                      today's numbers (vehicle.yaml)
    pixi run vehicle-model --set throttle_lag=0.25 --set steer_lag=0.15
                                                the car day's numbers, from the actuator tests
    pixi run vehicle-model --no-figures         the tables only
    pixi run vehicle-model --speed-loop         TODO 27: the speed loop's designs against the
                                                throttle's 5x uncertainty (and its figure)
    pixi run vehicle-model --lqi                TODO 3: the speed LQI's gains, as the Rust
                                                constants (paste into speed_loop.rs)

The maths: ros/ego/aav_ego/aav_ego/vehicle_model.py. The page: docs/vehicle-model.md.
The figures go to docs/img/vehicle-model-*.png, drawn with matplotlib (in Pixi's default
environment, through ROS's rqt); without it, the tables only. No ROS needed.
"""

import argparse
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros' / 'ego' / 'aav_ego'))
sys.path.insert(0, str(ROOT / 'ros' / 'sim' / 'aav_gazebo'))

from aav_ego import vehicle_model as vm  # noqa: E402, I100
from aav_gazebo import actuator_model as am  # noqa: E402
import numpy as np  # noqa: E402

VEHICLE_YAML = ROOT / 'ros/bringup/aav_description/config/vehicle.yaml'
GAZEBO_XACRO = ROOT / 'ros/bringup/aav_description/urdf/aav.gazebo.xacro'
IMG = ROOT / 'docs' / 'img'


def table(title, rows):
    """Return a table as aligned text."""
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    line = '  '.join('-' * w for w in widths)
    out = [title, line]
    for n, r in enumerate(rows):
        out.append('  '.join(c.ljust(w) for c, w in zip(r, widths)).rstrip())
        if n == 0:
            out.append(line)
    return '\n'.join(out)


def gazebo_steer_gain(path=GAZEBO_XACRO):
    """Gazebo's AckermannSteering steer_p_gain: its joints turn at gain x error (1/s)."""
    found = re.search(r'<steer_p_gain>([\d.]+)</steer_p_gain>', Path(path).read_text())
    return float(found.group(1)) if found else 1.0      # gz-sim's own default


# s: gazebo_adapter sends the model's angle 50 times a second (rate_hz); holding each for
# 20 ms delays it by half that, on average. Measured: 1/gain + 11-12 ms
# (tools/gazebo_steering.py, TODO 52c).
ADAPTER_HOLD = 0.5 / 50.0


def gazebo_extra_lag(gain=None):
    """Return how much later than the model Gazebo's wheels steer (s): 1/gain + the hold."""
    return 1.0 / (gain or gazebo_steer_gain()) + ADAPTER_HOLD


def gazebo_rows(model):
    """Gazebo's car against the model: its steering has one more lag (gazebo_extra_lag)."""
    gain = gazebo_steer_gain()
    extra = gazebo_extra_lag(gain)
    rows = [['v m/s', 'the car (model)', f'Gazebo (+{extra * 1000:.0f} ms lag)',
             'the car: delay left', 'Gazebo: delay left']]
    for v in vm.lateral_speeds():
        ol = model.lateral_open_loop(v)
        gz = vm.series(ol, (np.array([1.0]), np.array([extra, 1.0])))
        T = model.lateral_delay('typical')
        a, b = vm.margins(ol, T), vm.margins(gz, T)
        rows.append([f'{v:g}', f'PM {a.phase_margin_deg:.0f} deg',
                     f'PM {b.phase_margin_deg:.0f} deg',
                     f'{a.max_delay * 1000:.0f} ms', f'{b.max_delay * 1000:.0f} ms'])
    return gain, rows


def figures(model, out_dir):
    """Draw the step responses, the Bode plots and the pole map. Return the files."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.3,
                         'axes.spines.top': False, 'axes.spines.right': False})
    blue, orange, grey, green = '#2a6fb0', '#d9822b', '#7a7a7a', '#3a9a5b'
    files = []

    # 1. Step responses: the speed loop, the steering, the lateral recovery.
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    ax = axes[0]
    t, y = vm.step(model.speed_closed_loop(), t_end=25.0)
    ax.plot(t, y, color=blue, label="spine's PI + throttle lag")
    t2, y2 = vm.step(model.speed_closed_loop(vm.SpeedLoopGains(kp=0.3, ki=0.0)), t_end=25.0)
    ax.plot(t2, y2, color=orange, ls='--', label='P only (ki = 0)')
    t3, y3 = vm.step(model.speed_closed_loop(vm.SMOOTH), t_end=25.0)
    ax.plot(t3, y3, color=green, label="'smooth' profile (TODO 27)")
    ax.axhline(1.0, color=grey, lw=0.8)
    ax.set(title='Speed: a 1 m/s step in the set speed', xlabel='s', ylabel='m/s')
    ax.legend(frameon=False, fontsize=9)
    ax = axes[1]
    t, y = vm.step(model.steer_tf(), t_end=0.8)
    ax.plot(t, model.steer_step * y, color=blue, label='second-order model')
    ramp = np.minimum(t * model.steer_rate, model.steer_step)
    lag = np.zeros_like(t)
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        lag[i] = lag[i - 1] + (ramp[i] - lag[i - 1]) * (1 - math.exp(-dt / model.steer_lag))
    ax.plot(t, lag, color=orange, ls='--', label='rate limit, then lag')
    ax.set(title=f'Steering: a {model.steer_step:g} rad step', xlabel='s', ylabel='rad')
    ax.legend(frameon=False, fontsize=9)
    ax = axes[2]
    for v, c in ((0.5, orange), (1.0, blue), (3.0, green)):
        acl = model.lateral_closed_loop(v)
        ts = np.linspace(0, 12, 1201)
        x0 = np.array([0.5, 0.0, 0.0])
        ys = [(np.real(_expm(acl * tt) @ x0))[0] for tt in ts]
        ax.plot(ts, ys, color=c, label=f'{v:g} m/s')
    ax.axhline(0.0, color=grey, lw=0.8)
    ax.set(title='Lateral: 0.5 m off the path, pure pursuit', xlabel='s',
           ylabel='lateral error, m')
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    files.append(save(fig, out_dir / 'vehicle-model-steps.png'))

    # 2. Bode: the speed loop and the lateral loop at 1 m/s, with and without the delays.
    w = np.logspace(-2, 2, 2000)
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex='col')
    loops = (('Speed loop L(s)', model.speed_open_loop(),
              model.speed_delay),
             ('Lateral loop L(s), 1 m/s', model.lateral_open_loop(1.0), model.lateral_delay))
    for col, (title, ol, delay) in enumerate(loops):
        for case, c, ls in (('none', blue, '-'), ('worst', orange, '--')):
            T = delay(case)
            mag, ph = vm.freq_response(ol, w, T)
            m = vm.margins(ol, T)
            axes[0, col].semilogx(w, 20 * np.log10(mag), color=c, ls=ls,
                                  label=f'delay {T * 1000:.0f} ms')
            axes[1, col].semilogx(w, ph, color=c, ls=ls,
                                  label=f'PM {m.phase_margin_deg:.0f} deg')
            axes[1, col].plot([m.gain_crossover], [m.phase_margin_deg - 180], 'o', color=c)
        axes[0, col].axhline(0, color=grey, lw=0.8)
        axes[1, col].axhline(-180, color=grey, lw=0.8)
        axes[0, col].set(title=title, ylabel='gain, dB')
        axes[1, col].set(xlabel='rad/s', ylabel='phase, deg', ylim=(-360, 0))
        axes[0, col].legend(frameon=False, fontsize=9)
        axes[1, col].legend(frameon=False, fontsize=9)
    fig.tight_layout()
    files.append(save(fig, out_dir / 'vehicle-model-bode.png'))

    # 3. Pole map: the speed loop, the steering, the lateral loop at each speed.
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    ax = axes[0]
    sp = vm.poles(model.speed_closed_loop())
    ax.plot(sp.real, sp.imag, 'x', color=blue, ms=9, mew=2, label='speed loop (spine PI)')
    st = vm.poles(model.steer_tf())
    ax.plot(st.real, st.imag, 'x', color=green, ms=9, mew=2, label='steering')
    for v, c in ((0.5, orange), (1.0, blue), (3.0, green)):
        p = np.linalg.eigvals(model.lateral_closed_loop(v))
        ax.plot(p.real, p.imag, 'o', mfc='none', color=c, ms=8, label=f'lateral, {v:g} m/s')
    ax.axvline(0, color=grey, lw=0.8)
    ax.set(title='Closed-loop poles (s-plane)', xlabel='real, 1/s', ylabel='imaginary, rad/s')
    ax.legend(frameon=False, fontsize=8, loc='upper left')
    ax = axes[1]
    ax.plot(sp.real, sp.imag, 'x', color=blue, ms=9, mew=2)
    for v, c in ((0.5, orange), (1.0, blue), (3.0, green)):
        p = np.linalg.eigvals(model.lateral_closed_loop(v))
        ax.plot(p.real, p.imag, 'o', mfc='none', color=c, ms=8)
    ax.axvline(0, color=grey, lw=0.8)
    ax.set(title='Zoomed in: the slow poles', xlabel='real, 1/s', xlim=(-1.2, 0.1),
           ylim=(-1.1, 1.1))
    fig.tight_layout()
    files.append(save(fig, out_dir / 'vehicle-model-poles.png'))
    plt.close('all')
    return files


# --- TODO 27: the speed loop against the throttle's uncertainty --------------------------
#
# The nonlinear loop (Gazebo's port of speed_loop.rs: clamps, the coast band, the brake;
# the shared examples keep it equal to the Rust) on the actuator model, with ego's 1 m/s^2
# ramp and the wheel speed's delay. The plant's gain K = a_max / drag is the guess two
# sources disagree on by 5x (TODO 52b).

PLANTS = (                       # name, a_max x, drag x (of vehicle.yaml's, or --set's)
    ('K (vehicle.yaml)', 1.0, 1.0),
    ('K/5 by drag (5x)', 1.0, 5.0),
    ('K/5 by a weak throttle (a_max / 5)', 0.2, 1.0),
    ('2K by a strong throttle (a_max x 2)', 2.0, 1.0),
)
LAGS = (('the lag, 30 ms late', 0.1, 0.03),        # vehicle.yaml's lag; the typical delay
        ('the lag x 2, 110 ms late', 0.2, 0.11))   # the lag doubled; the worst delay
# (Each lag here is for vehicle.yaml's 0.1 s, and scales with the model's throttle_lag.)
TARGETS = (0.5, 1.0, 2.0)                    # m/s; reversing is the same law (0.5 checked)
RAMP = 1.0                                   # m/s^2: ego's max_acceleration
DT = 0.01                                    # s: the spine's tick


class FeedForward(am.SpeedLoop):
    """Add throttle for the ramp's acceleration to today's PI (the throttle map's guess)."""

    def __init__(self, per_accel):
        super().__init__(am.SpeedLoopConfig())
        self.per_accel = per_accel        # throttle per m/s^2 (1 / a_max guessed)
        self.last = 0.0

    def update(self, target, measured, dt):
        d = super().update(target, measured, dt)
        slope, self.last = (abs(target) - abs(self.last)) / dt, target
        if d.brake > 0.0 or slope <= 0.0:
            return d
        return am.Demand(min(1.0, d.throttle + self.per_accel * slope), 0.0, d.reverse)


def designs():
    """Return the candidates: (name, make a loop, its linear gains for the margins)."""
    pp = vm.VehicleModel().place_speed_gains(0.9, 1.0)
    today = am.SPEED_LOOP_PROFILES['default']
    weighted = {'integral_limit': 1.0, 'anti_windup': 'clamp'}
    return (
        ('today (kp 0.3, ki 0.1)', lambda: am.SpeedLoop(today), vm.SpeedLoopGains()),
        ('today + clamping', lambda: am.SpeedLoop(replace_cfg(today, anti_windup='clamp')),
         vm.SpeedLoopGains()),
        ('today + setpoint weight 0', lambda: am.SpeedLoop(replace_cfg(
            today, setpoint_weight=0.0, **weighted)), vm.SpeedLoopGains()),
        ('today + feed-forward (1/1.5 per m/s^2)', lambda: FeedForward(1 / 1.5),
         vm.SpeedLoopGains()),
        (f'pole placement (kp {pp.kp:.2f}, ki {pp.ki:.2f})', lambda: am.SpeedLoop(replace_cfg(
            today, kp=round(pp.kp, 2), ki=round(pp.ki, 2))), pp),
        ('pole placement + weight 0.3', lambda: am.SpeedLoop(replace_cfg(
            today, kp=round(pp.kp, 2), ki=round(pp.ki, 2), setpoint_weight=0.3, **weighted)),
         pp),
        ("'smooth' (kp 1.0, ki 0.25, weight 0.75)",
         lambda: am.SpeedLoop(am.SPEED_LOOP_PROFILES['smooth']), vm.SMOOTH),
        ("'lqi' (TODO 3: throttle and brake, one law)",
         lambda: am.SpeedLoop(am.SPEED_LOOP_PROFILES['lqi']), LQI),
    )


LQI = 'lqi'   # designs()' marker: the margins come from speed_lqi_open_loop


def lqi_gains_of(cfg):
    """Return spine-core's LQI constants as the model's SpeedLqiGains (for its margins)."""
    return vm.SpeedLqiGains(k_speed=cfg.k_speed, k_accel=cfg.k_accel,
                            k_integral=cfg.k_integral, setpoint_weight=cfg.setpoint_weight,
                            design_lag=cfg.brake_lag, tick=DT)


def open_loop_of(plant_model, gains):
    """Return a design's loop on a plant: the PI's, or the LQI's with spine-core's model."""
    if gains is LQI:
        q = am.LQI_MODEL
        return plant_model.speed_lqi_open_loop(lqi_gains_of(q), q.max_accel, q.throttle_lag)
    return plant_model.speed_open_loop(gains)


def lqi_constants(model, weights=None):
    """Return the LQI's gains for this model, as spine-core's and the port's constants."""
    w = weights or vm.SpeedLqiWeights()
    g = model.speed_lqi(w)
    numbers = {'k_speed': g.k_speed, 'k_accel': g.k_accel, 'k_integral': g.k_integral,
               'setpoint_weight': g.setpoint_weight, 'integral_limit': model.max_accel,
               'dead_band': 0.05, 'max_accel': model.max_accel, 'max_brake': model.max_brake,
               'drag': model.drag, 'throttle_lag': model.throttle_lag,
               'brake_lag': model.brake_lag}
    numbers = {k: float(f'{v:.4g}') for k, v in numbers.items()}
    rust = ['    pub const MODEL: LqiConfig = LqiConfig {']
    rust += [f'        {k}: {v!r},' for k, v in numbers.items()]
    rust += ['    };']
    python = ', '.join(f'{k}={v!r}' for k, v in numbers.items())
    return (f'// LQI for {w} (tau {g.design_lag:g} s, tick {g.tick:g} s)\n'
            + '\n'.join(rust) + f'\n\n# actuator_model.py\nLQI_MODEL = LqiConfig({python})')


def replace_cfg(cfg, **changes):
    from dataclasses import replace
    return replace(cfg, **changes)


def ramp_response(make_loop, actuators, target, delay, seconds=60.0):
    """Return the speeds of one run: ego's ramp to target, the wheel speed `delay` s old."""
    car = am.Actuators(actuators)
    loop = make_loop()
    late = [0.0] * (round(delay / DT) + 1)
    speeds, command = [], 0.0
    for _ in range(round(seconds / DT)):
        command = math.copysign(min(abs(target), abs(command) + RAMP * DT), target)
        s = car.drive(loop.update(command, late[0], DT), 0.0, DT)
        late = late[1:] + [s.speed]
        speeds.append(s.speed)
    return np.array(speeds)


def response_numbers(speeds, target):
    """Return (overshoot, time to 90 %, time to stay within 5 %, end error), as fractions."""
    v, a = np.abs(speeds), abs(target)
    t = np.arange(1, len(v) + 1) * DT
    reached = np.nonzero(v >= 0.9 * a)[0]
    outside = np.nonzero(np.abs(v - a) > 0.05 * a)[0]
    settle = t[outside[-1]] if len(outside) else 0.0
    return (max(0.0, v.max() / a - 1.0), t[reached[0]] if len(reached) else math.inf,
            settle if settle < t[-1] else math.inf, v[-1] / a - 1.0)


def plant_numbers(model, accel_x, drag_x):
    """Return the plant's (a_max, drag): the model's, times the uncertainty's factors."""
    return {'max_accel': model.max_accel * accel_x, 'drag': model.drag * drag_x}


def speed_loop_rows(model):
    """Return the study's table: each design against each plant, the worst of its runs."""
    rows = [['design', 'plant', 'overshoot', 'to 90 %', 'within 5 %', 'end',
             'phase margin (worst)']]
    for name, make, gains in designs():
        for plant, accel_x, drag_x in PLANTS:
            act = plant_numbers(model, accel_x, drag_x)
            lin = act
            worst = [0.0, 0.0, 0.0, 0.0]
            for _, lag, delay in LAGS:
                cfg = replace_cfg(am.ActuatorConfig(), throttle_lag=lag * model.throttle_lag
                                  / 0.1, **act)
                for target in TARGETS:
                    o, r, w, e = response_numbers(ramp_response(make, cfg, target, delay),
                                                  target)
                    worst = [max(worst[0], o), max(worst[1], r), max(worst[2], w),
                             min(worst[3], e)]
            worst_lag = LAGS[-1][1] * model.throttle_lag / 0.1
            plant_model = vm.VehicleModel(throttle_lag=worst_lag, **lin)
            pm = vm.margins(open_loop_of(plant_model, gains), LAGS[-1][2]).phase_margin_deg
            end = 'reached' if worst[3] > -0.02 else f'{-100 * worst[3]:.0f} % short'
            rows.append([name, plant, f'{100 * worst[0]:.0f} %', seconds(worst[1]),
                         seconds(worst[2]), end, f'{pm:.0f} deg'])
            name = ''
    return rows


def seconds(t):
    """Return a time for the table: never, if the run didn't get there."""
    return f'{t:.1f} s' if math.isfinite(t) else 'never'


def speed_loop_figure(model, out_dir):
    """Draw today's loop, 'smooth' and 'lqi' on each plant: 1 m/s with ego's ramp."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.3,
                         'axes.spines.top': False, 'axes.spines.right': False})
    blue, orange, grey, green = '#2a6fb0', '#d9822b', '#7a7a7a', '#3a9a5b'
    loops = {name: make for name, make, _ in designs()}
    today, smooth, lqi = list(loops)[0], list(loops)[-2], list(loops)[-1]
    fig, axes = plt.subplots(1, len(PLANTS), figsize=(15, 3.6), sharey=True)
    for ax, (plant, accel_x, drag_x) in zip(axes, PLANTS):
        cfg = replace_cfg(am.ActuatorConfig(), throttle_lag=model.throttle_lag,
                          **plant_numbers(model, accel_x, drag_x))
        t = np.arange(1, round(30.0 / DT) + 1) * DT
        for name, colour, style in ((today, orange, '--'), (smooth, blue, '-'),
                                    (lqi, green, ':')):
            ax.plot(t, ramp_response(loops[name], cfg, 1.0, LAGS[0][2], 30.0), color=colour,
                    ls=style, label=name.split(' (')[0])
        ax.axhline(1.0, color=grey, lw=0.8)
        ax.set(title=plant.replace(' (', '\n('), xlabel='s')
    axes[0].set(ylabel='m/s')
    axes[0].legend(frameon=False, fontsize=9, loc='lower right')
    fig.suptitle('The speed loop: ego asks for 1 m/s (ramped at 1 m/s²)')
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = save(fig, out_dir / 'vehicle-model-speed-loop.png')
    plt.close('all')
    return path


def _expm(a):
    from scipy.linalg import expm
    return expm(a)


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    # No date in the file: the same numbers draw the same picture.
    fig.savefig(path, dpi=110, metadata={'Software': None, 'Creation Time': None})
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[1].strip(),
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vehicle', default=str(VEHICLE_YAML), help='vehicle.yaml to read')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='a measured number: throttle_lag=0.25, actuators.steer_lag=0.15, '
                         'steer_overshoot=0.1 (repeat it)')
    ap.add_argument('--no-figures', action='store_true', help='print the tables only')
    ap.add_argument('--speed-loop', action='store_true',
                    help="TODO 27: the speed loop's designs against the throttle's 5x "
                         'uncertainty, then its figure (about a minute)')
    ap.add_argument('--lqi', action='store_true',
                    help="TODO 3: print the speed LQI's gains for this model, as spine-core's "
                         'LqiConfig::MODEL, and as LQI_MODEL for the Python port')
    ap.add_argument('--out', default=str(IMG), help='where the figures go')
    args = ap.parse_args(argv)

    measured = {}
    for item in args.set:
        key, _, value = item.partition('=')
        measured[key.strip()] = float(value)
    model = vm.from_vehicle_yaml(args.vehicle).with_measured(measured)
    if args.lqi:
        print(lqi_constants(model))
        return 0
    if args.speed_loop:
        print(table('The speed loop against the uncertainty (TODO 27): the worst of '
                    f'{len(LAGS) * len(TARGETS)} runs each ({", ".join(n for n, _, _ in LAGS)}; '
                    f'{", ".join(f"{v:g}" for v in TARGETS)} m/s); K from a_max '
                    f'{model.max_accel:g} m/s^2, drag {model.drag:g} /s', speed_loop_rows(model)))
        print()
        if not args.no_figures:
            f = speed_loop_figure(model, Path(args.out))
            print(f'wrote {f.relative_to(ROOT) if f.is_relative_to(ROOT) else f}')
        return 0
    print(f"The car's linear model, from {Path(args.vehicle).name}"
          + (f' with {", ".join(args.set)}' if args.set else ' (every actuator number a GUESS)'))
    print()
    for title, rows in vm.summary(model):
        print(table(title, rows))
        print()
    gain, rows = gazebo_rows(model)
    print(table(f"Gazebo's car: steer_p_gain {gain:g} and the adapter's hold add a "
                f'{1000 * gazebo_extra_lag(gain):.0f} ms lag after the actuator model '
                '(typical delays)', rows))
    print()
    if args.no_figures:
        return 0
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        print('no matplotlib here: the tables only (pixi run vehicle-model has it)')
        return 0
    for f in figures(model, Path(args.out)):
        print(f'wrote {f.relative_to(ROOT) if f.is_relative_to(ROOT) else f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
