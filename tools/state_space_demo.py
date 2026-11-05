#!/usr/bin/env python3
"""
State space, from a drawing to a car simulator: the teaching example.

    pixi run python3 tools/state_space_demo.py               print the numbers, draw
                                                             docs/img/learn-ss-*.png
    pixi run python3 tools/state_space_demo.py --no-figures  the numbers only

The page that walks through it: docs/learn-state-space.md. Each part is one step of the
process: build A, B, C, D; simulate (Euler, RK4, exact); poles = eigenvalues = the
transfer function's roots; Bode with a delay; the quarter-car; roll in a turn.

The car's numbers come from vehicle.yaml. The suspension's are GUESSES (SUSPENSION below):
nobody has measured a spring on the car yet. No ROS needed: numpy, scipy, matplotlib.
"""

import argparse
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros' / 'ego' / 'aav_ego'))

from aav_ego import vehicle_model as vm  # noqa: E402
import numpy as np  # noqa: E402
from scipy import linalg, signal  # noqa: E402
import yaml  # noqa: E402

VEHICLE_YAML = ROOT / 'ros/bringup/aav_description/config/vehicle.yaml'
IMG = ROOT / 'docs' / 'img'
G = 9.81  # m/s^2

# GUESSES for a small electric car of about 181 kg (TODO measure: the M-list).
SUSPENSION = {
    'unsprung': 12.0,        # kg per corner: wheel, hub, half an axle
    'spring': 8000.0,        # N/m per corner, at the wheel
    'zeta': 0.3,             # the shocks' damping ratio (a soft, bouncy car is 0.2-0.3)
    'tyre': 60000.0,         # N/m: the tyre as a spring
    'roll_centre': 0.20,     # m above the ground
    'roll_gyration': 0.35,   # m: the body's radius of gyration about its own roll axis
    'cornering': 8.0,        # per rad: an axle's cornering stiffness / the load on it
}


def load_car(path=VEHICLE_YAML):
    """Return vehicle.yaml's vehicle: section (mass, track, centre of mass...)."""
    with open(path) as f:
        return yaml.safe_load(f)['vehicle']


# --- Step 4: the models as x' = A x + B u, y = C x + D u ------------------------------

def mass_spring_damper(m, c, k):
    """Return A, B, C, D of m x'' + c x' + k x = F: states [x, x'], two energy stores."""
    a = np.array([[0.0, 1.0],
                  [-k / m, -c / m]])
    b = np.array([[0.0], [1.0 / m]])
    return a, b, np.array([[1.0, 0.0]]), np.zeros((1, 1))


def speed_loop(car):
    """
    Return A, B, C, D of the car's speed with the spine's PI: x = [v, u_a, z], u = v_set.

    v' = a_max u_a - d v;  u_a' = (u - u_a) / tau_t;  z' = v_set - v (the integral)
    u = kp (v_set - v) + ki z
    """
    d, a_max, tt = car.drag, car.max_accel, car.throttle_lag
    kp, ki = car.gains.kp, car.gains.ki
    a = np.array([[-d, a_max, 0.0],
                  [-kp / tt, -1.0 / tt, ki / tt],
                  [-1.0, 0.0, 0.0]])
    b = np.array([[0.0], [kp / tt], [1.0]])
    return a, b, np.array([[1.0, 0.0, 0.0]]), np.zeros((1, 1))


def bicycle(x, u, v, wheelbase, tau_s):
    """Return x' of the kinematic bicycle, nonlinear: x = [y, theta, delta], u = delta_cmd."""
    _, theta, delta = x
    return np.array([v * math.sin(theta),
                     v * math.tan(delta) / wheelbase,
                     (u - delta) / tau_s])


def jacobian(f, x0, u0, eps=1e-6):
    """Linearise x' = f(x, u) about (x0, u0) by finite differences: return A, B."""
    x0 = np.asarray(x0, dtype=float)
    f0 = f(x0, u0)
    a = np.column_stack([(f(x0 + eps * e, u0) - f0) / eps for e in np.eye(len(x0))])
    b = ((f(x0, u0 + eps) - f0) / eps).reshape(-1, 1)
    return a, b


def dynamic_bicycle(v, car, s=SUSPENSION):
    """
    Return A, B of the bicycle with tyres that slip: x = [v_y, r] (sideways, yaw), u = delta.

    Each axle pushes sideways C x its slip angle. The tyres' own poles sit near
    -(C_f + C_r) / (m v): fast at low speed, slower as the car speeds up.
    """
    m, wheelbase = float(car['mass']), float(car['wheelbase'])
    b_ = float(car['centre_of_mass']['x'])          # rear axle to the centre of mass
    a_ = wheelbase - b_                              # ... to the front axle
    cf = s['cornering'] * m * G * b_ / wheelbase     # the load on the front axle x 8 /rad
    cr = s['cornering'] * m * G * a_ / wheelbase
    iz = m * (float(car['body']['length']) ** 2 + float(car['body']['width']) ** 2) / 12
    a = np.array([[-(cf + cr) / (m * v), (b_ * cr - a_ * cf) / (m * v) - v],
                  [(b_ * cr - a_ * cf) / (iz * v), -(a_ * a_ * cf + b_ * b_ * cr) / (iz * v)]])
    b = np.array([[cf / m], [a_ * cf / iz]])
    return a, b


def quarter_car(m_s, m_u, k_s, c_s, k_t):
    """
    Return A, B, C, D of one corner: the body (m_s) on its spring and shock, the wheel (m_u).

    States [z_s, z_s', z_u, z_u'] (heights, up). Inputs [road height z_r, gravity 1].
    """
    a = np.array([[0.0, 1.0, 0.0, 0.0],
                  [-k_s / m_s, -c_s / m_s, k_s / m_s, c_s / m_s],
                  [0.0, 0.0, 0.0, 1.0],
                  [k_s / m_u, c_s / m_u, -(k_s + k_t) / m_u, -c_s / m_u]])
    b = np.array([[0.0, 0.0], [0.0, -G], [0.0, 0.0], [k_t / m_u, -G]])
    return a, b, np.array([[1.0, 0.0, 0.0, 0.0]]), np.zeros((1, 2))


def roll_model(m_s, arm, k_roll, inertia, zeta):
    """Return A, B, C, D of I phi'' + c phi' + (K - m g h') phi = m h' a_y; u = a_y."""
    k_net = k_roll - m_s * G * arm          # gravity helps the body lean further
    c = 2.0 * zeta * math.sqrt(k_net * inertia)
    a = np.array([[0.0, 1.0], [-k_net / inertia, -c / inertia]])
    b = np.array([[0.0], [m_s * arm / inertia]])
    return a, b, np.array([[1.0, 0.0]]), np.zeros((1, 1))


def car_corners(car, s=SUSPENSION):
    """Return the suspension numbers for this car: sprung mass, a corner's, roll's."""
    m = float(car['mass'])
    m_s = m - 4 * s['unsprung']
    corner = m_s / 4
    c_s = 2 * s['zeta'] * math.sqrt(s['spring'] * corner)
    arm = float(car['centre_of_mass']['height']) - s['roll_centre']
    k_roll = s['spring'] * float(car['track']) ** 2      # two axles, k t^2 / 2 each
    inertia = m_s * (s['roll_gyration'] ** 2 + arm ** 2)  # parallel axis, to the roll axis
    return {'m_s': m_s, 'corner': corner, 'c_s': c_s, 'arm': arm, 'k_roll': k_roll,
            'inertia': inertia}


def steady_roll(a_y, m_s, arm, k_roll):
    """Return the roll angle (rad) in a steady turn at a lateral acceleration a_y."""
    return m_s * arm * a_y / (k_roll - m_s * G * arm)


def load_transfer(a_y, mass, cg_height, track):
    """Return the weight (N) that moves from the inside wheels to the outside ones."""
    return mass * a_y * cg_height / track


def rollover_limit(cg_height, track):
    """Return the lateral acceleration (m/s^2) at which the inside wheels lift: g t / 2h."""
    return G * track / (2.0 * cg_height)


# --- Step 5: simulate: step the state forward in time ---------------------------------

def euler(f, x0, dt, n):
    """Step x[k+1] = x[k] + dt f(x[k]): the simplest simulator there is."""
    xs = [np.asarray(x0, dtype=float)]
    for _ in range(n):
        xs.append(xs[-1] + dt * f(xs[-1]))
    return np.array(xs)


def rk4(f, x0, dt, n):
    """Step with Runge-Kutta 4: four slopes per step, averaged. Error shrinks as dt^4."""
    xs = [np.asarray(x0, dtype=float)]
    for _ in range(n):
        x = xs[-1]
        k1 = f(x)
        k2 = f(x + dt / 2 * k1)
        k3 = f(x + dt / 2 * k2)
        k4 = f(x + dt * k3)
        xs.append(x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
    return np.array(xs)


def exact(a, b, u, x0, dt, n):
    """Step x[k+1] = Ad x[k] + Bd u, Ad = e^(A dt): exact for a linear model, u held."""
    ad, bd, *_ = signal.cont2discrete((a, b, np.eye(len(a)), 0 * b), dt, method='zoh')
    xs = [np.asarray(x0, dtype=float)]
    for _ in range(n):
        xs.append(ad @ xs[-1] + (bd @ np.atleast_1d(u)))
    return np.array(xs)


# --- Steps 6-7: transfer function and poles -------------------------------------------

def poles_two_ways(a, b, c, d):
    """Return (eigenvalues of A, roots of G(s)'s denominator), both sorted."""
    _, den = signal.ss2tf(a, b, c, d)
    key = (lambda p: (round(p.real, 9), round(p.imag, 9)))
    return (np.array(sorted(np.linalg.eigvals(a), key=key)),
            np.array(sorted(np.roots(den), key=key)))


def report(car_yaml, model):
    """Print each step's numbers."""
    print('1. Mass-spring-damper, m = 1, c = 0.4, k = 4 (wn 2 rad/s, zeta 0.1)')
    a, b, c, d = mass_spring_damper(1.0, 0.4, 4.0)
    eig, roots = poles_two_ways(a, b, c, d)
    print(f'   A = {a.tolist()}  eig(A) = {np.round(eig, 3)}  TF roots = {np.round(roots, 3)}')
    print('2. The speed loop (the spine PI around the car), states [v, u_a, z]')
    eig, roots = poles_two_ways(*speed_loop(model))
    print(f'   poles = {np.round(eig, 3)}  (vehicle-model.md: -0.28 +/- 0.28j, -9.5)')
    print('3. The bicycle, linearised about straight at 1 m/s (finite-difference Jacobian)')
    f = (lambda x, u: bicycle(x, u, 1.0, model.wheelbase, model.steer_tau))
    a, _ = jacobian(f, [0, 0, 0], 0.0)
    print(f'   A = {np.round(a, 3).tolist()}  eig = {np.round(np.linalg.eigvals(a), 3)}')
    print('4. Euler vs exact on the mass-spring-damper: biggest error over 10 s')
    a, b, *_ = mass_spring_damper(1.0, 0.4, 4.0)
    for dt in (0.1, 0.01, 0.001):
        n = int(round(10 / dt))
        lin = (lambda x: a @ x + b[:, 0])
        ref = exact(a, b, 1.0, [0, 0], dt, n)[:, 0]
        print(f'   dt {dt:<6g} Euler {np.abs(euler(lin, [0, 0], dt, n)[:, 0] - ref).max():.2e}'
              f'   RK4 {np.abs(rk4(lin, [0, 0], dt, n)[:, 0] - ref).max():.2e}')
    s = car_corners(car_yaml)
    mass, h, t = float(car_yaml['mass']), float(car_yaml['centre_of_mass']['height']), \
        float(car_yaml['track'])
    print('5. Quarter-car (GUESSED springs): poles, static sag')
    qa, qb, *_ = quarter_car(s['corner'], SUSPENSION['unsprung'], SUSPENSION['spring'],
                             s['c_s'], SUSPENSION['tyre'])
    for p in np.linalg.eigvals(qa):
        if p.imag > 0:
            wn, z = vm.wn_zeta(p)
            print(f'   {p:.1f}: {wn / (2 * math.pi):.1f} Hz, zeta {z:.2f}')
    x_static = -linalg.solve(qa, qb[:, 1])
    print(f'   spring sag {(x_static[2] - x_static[0]) * 100:.1f} cm '
          f'(m g / k = {s["corner"] * G / SUSPENSION["spring"] * 100:.1f} cm)')
    print(f'6. Roll and load transfer (m {mass:g} kg, h {h} m GUESS, track {t} m)')
    print(f'   rollover limit g t / 2h = {rollover_limit(h, t):.2f} m/s^2 '
          f'({rollover_limit(h, t) / G:.2f} g)')
    tightest = model.wheelbase / math.tan(model.max_steering)
    for v in (1.0, 2.0, 3.0):
        for r in (tightest, 5.0, 10.0):
            ay = v * v / r
            phi = steady_roll(ay, s['m_s'], s['arm'], s['k_roll'])
            dfz = load_transfer(ay, mass, h, t)
            print(f'   v {v:g} m/s, R {r:4.2f} m: a_y {ay:4.2f} m/s^2 ({ay / G:.2f} g), '
                  f'roll {math.degrees(phi):4.2f} deg, {dfz:4.0f} N moves out '
                  f'({dfz / (mass * G / 2) * 100:3.0f} % of a side)')
    ra, rb, rc, rd = roll_model(s['m_s'], s['arm'], s['k_roll'], s['inertia'],
                                SUSPENSION['zeta'])
    wn, _ = vm.wn_zeta(np.linalg.eigvals(ra)[0])
    num, den = signal.ss2tf(ra, rb, rc, rd)
    print(f'   the roll model: {wn / (2 * math.pi):.1f} Hz; G(0) = '
          f'{math.degrees(vm.dc_gain((num[0], den))):.2f} deg per m/s^2 sideways')
    print('7. The dynamic bicycle (tyres that slip, GUESSED grip): its poles')
    for v in (1.0, 3.0, 10.0):
        eig = np.linalg.eigvals(dynamic_bicycle(v, car_yaml)[0])
        print(f'   v {v:g} m/s: {np.round(eig, 1)}')


# --- The figures ----------------------------------------------------------------------

BLUE, ORANGE, PURPLE, GREEN, GREY = '#2a6fb0', '#c26a17', '#8a5cb8', '#2f8a4f', '#7a7a7a'


def figures(car_yaml, model, out=IMG):
    """Draw docs/img/learn-ss-*.png. Return the files."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.3,
                         'axes.spines.top': False, 'axes.spines.right': False})
    files = []

    def save(fig, name):
        fig.tight_layout()
        path = out / f'learn-ss-{name}.png'
        fig.savefig(path, dpi=110)
        plt.close(fig)
        files.append(path)

    # Damping: the same mass and spring, four shocks.
    fig, ax = plt.subplots(figsize=(7, 3.6))
    t = np.linspace(0, 8, 801)
    for zeta, col, ls in ((0.1, ORANGE, '-'), (0.4, PURPLE, '--'), (1.0, BLUE, '-'),
                          (2.0, GREEN, ':')):
        _, y = signal.step(signal.StateSpace(*mass_spring_damper(1, 2 * zeta * 2, 4)), T=t)
        ax.plot(t, 4 * y, color=col, ls=ls, lw=2, label=f'ζ = {zeta}')
    ax.set(xlabel='time (s)', ylabel='position ÷ final',
           title='A push on a mass-spring-damper: more damping, less wobble')
    ax.legend(loc='lower right')
    save(fig, 'damping')

    # Simulators: Euler drifts, RK4 and the exact form agree.
    a, b, *_ = mass_spring_damper(1.0, 0.4, 4.0)
    lin = (lambda x: a @ x + b[:, 0])
    fig, ax = plt.subplots(figsize=(7, 3.6))
    dt = 0.1
    n = 100
    tt = np.arange(n + 1) * dt
    ax.plot(tt, exact(a, b, 1.0, [0, 0], dt, n)[:, 0] * 4, color=GREY, lw=5, alpha=0.5,
            label='exact, Ad = e^(A dt)')
    ax.plot(tt, rk4(lin, [0, 0], dt, n)[:, 0] * 4, color=BLUE, lw=2, ls='--', label='RK4')
    ax.plot(tt, euler(lin, [0, 0], dt, n)[:, 0] * 4, color=ORANGE, lw=2, label='Euler')
    ax.set(xlabel='time (s)', ylabel='position ÷ final',
           title='Three simulators, one model, dt = 0.1 s: Euler adds energy',
           ylim=(-0.1, 2.5))
    ax.legend(loc='upper center', ncol=3)
    save(fig, 'simulators')

    # The s-plane: where a pole sits, and what it does.
    pts = [('A', -0.5 + 0j, BLUE, 'slow, smooth'), ('B', -2 + 0j, GREEN, 'fast, smooth'),
           ('C', -0.4 + 3j, PURPLE, 'wobbles, dies out'), ('D', 0.2 + 0j, ORANGE, 'blows up'),
           ('E', 0 + 2j, GREY, 'wobbles for ever')]
    fig = plt.figure(figsize=(12, 4.6))
    grid = fig.add_gridspec(2, 5)
    ax = fig.add_subplot(grid[:, :2])
    ax.axvline(0, color='#222', lw=1)
    ax.axhline(0, color='#222', lw=1)
    ax.axvspan(-3, 0, color=GREEN, alpha=0.06)
    ax.text(-2.9, 3.5, 'left: dies out', color='#222')
    ax.text(0.05, 3.5, 'right:\ngrows', color='#222', va='center')
    t = np.linspace(0, 8, 400)
    for i, (name, p, col, what) in enumerate(pts):
        ax.plot([p.real, p.real], [p.imag, -p.imag], 'x', color=col, ms=11, mew=3)
        ax.annotate(name, (p.real, p.imag), xytext=(7, 5), textcoords='offset points',
                    fontweight='bold')
        bx = fig.add_subplot(grid[i // 3, 2 + i % 3])
        bx.plot(t, np.exp(p.real * t) * np.cos(p.imag * t), color=col, lw=2)
        bx.axhline(0, color='#222', lw=0.8)
        bx.set(ylim=(-1.2, 2.5), title=f'{name}: {what}', xlabel='time (s)')
        bx.title.set_fontsize(10)
    ax.set(xlim=(-3, 1), ylim=(-4, 4), xlabel='real part (1/s): how fast',
           ylabel='imaginary part (rad/s): wobble', title='Poles on the s-plane')
    save(fig, 'splane')

    # Bode: the lateral loop at 1 m/s, without and with the worst delay.
    ol = model.lateral_open_loop(1.0)
    w = np.logspace(-1, 1.5, 600)
    worst = model.delays.lateral('worst')
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for delay, col, ls, label in ((0.0, BLUE, '-', 'no delay'),
                                  (worst, ORANGE, '--', f'{worst * 1000:.0f} ms delay')):
        mag, ph = vm.freq_response(ol, w, delay)
        m = vm.margins(ol, delay)
        ax.semilogx(w, 20 * np.log10(mag), color=col, ls=ls, lw=2, label=label)
        bx.semilogx(w, ph, color=col, ls=ls, lw=2,
                    label=f'{label}: phase margin {m.phase_margin_deg:.0f}°')
        bx.plot([m.gain_crossover] * 2, [-180, -180 + m.phase_margin_deg], color=col, lw=4)
    ax.axhline(0, color='#222', lw=1)
    bx.axhline(-180, color='#222', lw=1)
    ax.set(ylabel='gain (dB)', title='The path loop at 1 m/s: a delay costs phase, not gain')
    bx.set(xlabel='how fast the path wiggles (rad/s)', ylabel='phase (°)', ylim=(-300, -60))
    ax.legend()
    bx.legend(loc='lower left')
    save(fig, 'bode')

    # The quarter-car over a 5 cm step (a kerb ramp).
    s = car_corners(car_yaml)
    qa, qb, *_ = quarter_car(s['corner'], SUSPENSION['unsprung'], SUSPENSION['spring'],
                             s['c_s'], SUSPENSION['tyre'])
    x0 = -linalg.solve(qa, qb[:, 1])          # sitting still, on its springs
    dt, n = 0.001, 1500
    xs = exact(qa, qb, np.array([0.05, 1.0]), x0, dt, n)
    t = np.concatenate([[-0.1, 0.0], np.arange(n + 1) * dt])   # still, then the step
    body = np.concatenate([[0, 0], (xs[:, 0] - x0[0]) * 100])
    wheel = np.concatenate([[0, 0], (xs[:, 2] - x0[2]) * 100])
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot([-0.1, 0, 0, t[-1]], [0, 0, 5, 5], color=GREY, lw=2, ls=':', label='the road')
    ax.plot(t, wheel, color=ORANGE, lw=2, label='the wheel')
    ax.plot(t, body, color=BLUE, lw=2, label='the body')
    ax.set(xlabel='time (s)', ylabel='rise (cm)',
           title='One corner over a 5 cm step (springs and shocks GUESSED)')
    ax.legend(loc='lower right')
    save(fig, 'quarter-car')

    # Roll and load transfer against speed, for three turns.
    h, track = float(car_yaml['centre_of_mass']['height']), float(car_yaml['track'])
    tightest = model.wheelbase / math.tan(model.max_steering)
    v = np.linspace(0, 4, 200)
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11, 3.8))
    for r, col, ls in ((tightest, ORANGE, '-'), (5.0, PURPLE, '--'), (10.0, BLUE, ':')):
        ay = v ** 2 / r
        ax.plot(v, np.degrees(steady_roll(ay, s['m_s'], s['arm'], s['k_roll'])), color=col,
                ls=ls, lw=2)
        bx.plot(v, ay / G, color=col, ls=ls, lw=2, label=f'R = {r:.1f} m')
        ax.annotate(f'R {r:.1f} m', (4, math.degrees(steady_roll(16 / r, s['m_s'], s['arm'],
                                                                 s['k_roll']))),
                    xytext=(4, 0), textcoords='offset points', va='center')
    bx.axhline(rollover_limit(h, track) / G, color='#222', lw=1.5)
    bx.text(0.1, rollover_limit(h, track) / G + 0.02, 'inside wheels lift (g·t/2h)')
    bx.axhline(0.5 * float(car_yaml['tyre_grip']), color=GREY, lw=1.5, ls='--')
    bx.text(0.1, 0.5 * float(car_yaml['tyre_grip']) + 0.02, 'kinematic model good below')
    for a_ in (ax, bx):
        a_.axvline(1.0, color=GREY, lw=1)
        a_.axvline(3.0, color=GREY, lw=1)
    ax.set(xlabel='speed (m/s)', ylabel='body roll (°)', xlim=(0, 4.7),
           title='Roll in a steady turn (springs GUESSED)')
    bx.set(xlabel='speed (m/s)', ylabel='sideways acceleration (g)', ylim=(0, 0.8),
           title='How hard the turn pushes')
    bx.legend(loc='upper left', bbox_to_anchor=(0.0, 0.88))
    save(fig, 'roll')
    return files


def main(argv=None):
    """Print the numbers, and draw the figures unless --no-figures."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    p.add_argument('--no-figures', action='store_true', help='print the numbers only')
    args = p.parse_args(argv)
    car_yaml = load_car()
    model = vm.from_vehicle_yaml(str(VEHICLE_YAML))
    report(car_yaml, model)
    if not args.no_figures:
        for f in figures(car_yaml, model):
            print(f'wrote {f.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
