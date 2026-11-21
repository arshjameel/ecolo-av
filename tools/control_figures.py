#!/usr/bin/env python3
"""
Draw the figures of docs/control-explained.md into docs/img/control/.

Each figure is worked from the same small models as docs/vehicle-model.md (vehicle.yaml's
guesses: wheelbase 1.561 m, the steering as one 0.16 s lag), so the pictures show what our
controllers do, not a textbook's car.

Run: pixi run python tools/control_figures.py
"""

import math
import os

import matplotlib

matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'aav'   # the same file each run
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.linalg import solve_continuous_are  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), '..', 'docs', 'img', 'control')
L = 1.561       # m, vehicle.yaml's wheelbase
TAU_S = 0.16    # s, the steering as one lag (docs/vehicle-model.md, section 2)
V = 1.0         # m/s
BLUE, ORANGE, GREEN, GREY, RED = '#2a6fdb', '#e8871e', '#2e9e5b', '#8a8a8a', '#c8402f'


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


def lateral(v=V):
    """Return A, B of the lateral model x = [e_y, e_theta, delta], u = delta_cmd."""
    a = np.array([[0.0, v, 0.0], [0.0, 0.0, v / L], [0.0, 0.0, -1.0 / TAU_S]])
    b = np.array([[0.0], [0.0], [1.0 / TAU_S]])
    return a, b


def simulate(gain, kappa, feed_forward, t_end=8.0, dt=0.002, x0=(0.0, 0.0, 0.0)):
    """Drive the lateral model with u = -K x (+ the bend's own steering), kappa(t)."""
    a, b = lateral()
    x = np.array(x0, dtype=float)
    rows = []
    for t in np.arange(0.0, t_end, dt):
        k = kappa(t)
        u = float(-gain @ x)
        if feed_forward:
            u += math.atan(L * k)
        dx = a @ x + b[:, 0] * u
        dx[1] -= V * k              # the path turns under the car
        x = x + dt * dx
        rows.append((t, x[0], x[2], u))
    return np.array(rows)


def pure_pursuit_gain(ld=1.5):
    """Return pure pursuit's K for small errors (docs/vehicle-model.md, section 3)."""
    return np.array([2 * L / ld ** 2, 2 * L / ld, 0.0])


def feed_forward_figure():
    def bend(t):
        return 0.2 if t >= 1.0 else 0.0     # a 5 m radius bend from t = 1 s
    k = pure_pursuit_gain()
    fb = simulate(k, bend, feed_forward=False)
    ff = simulate(k, bend, feed_forward=True)
    fig, (top, low) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    top.plot(fb[:, 0], fb[:, 1] * 100, color=ORANGE, label='feedback only')
    top.plot(ff[:, 0], ff[:, 1] * 100, color=BLUE, label='feed-forward + feedback')
    top.axvline(1.0, color=GREY, ls=':')
    top.set_ylabel('off the path (cm)')
    top.legend()
    top.set_title('A bend starts at 1 s (radius 5 m, 1 m/s)')
    low.plot(fb[:, 0], np.degrees(fb[:, 2]), color=ORANGE)
    low.plot(ff[:, 0], np.degrees(ff[:, 2]), color=BLUE)
    low.axhline(math.degrees(math.atan(L * 0.2)), color=GREY, ls='--', lw=1)
    low.text(5.2, math.degrees(math.atan(L * 0.2)) + 0.6, "the bend's own steering",
             color=GREY)
    low.set_ylabel('wheel angle (deg)')
    low.set_xlabel('time (s)')
    save(fig, 'feed-forward.png')


def lqr_figure():
    a, b = lateral()
    fig, (top, low) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    for r, colour in ((0.1, RED), (1.0, BLUE), (10.0, GREEN)):
        q = np.diag([1.0, 1.0, 0.0])
        p = solve_continuous_are(a, b, q, np.array([[r]]))
        k = (np.linalg.solve(np.array([[r]]), b.T @ p))[0]
        run = simulate(k, lambda t: 0.0, feed_forward=False, x0=(0.5, 0.0, 0.0))
        label = f'R = {r:g}: K = [{k[0]:.2f}, {k[1]:.2f}, {k[2]:.2f}]'
        top.plot(run[:, 0], run[:, 1], color=colour, label=label)
        low.plot(run[:, 0], np.degrees(run[:, 2]), color=colour)
    pp = simulate(pure_pursuit_gain(), lambda t: 0.0, feed_forward=False, x0=(0.5, 0, 0))
    top.plot(pp[:, 0], pp[:, 1], color=GREY, ls='--', label='pure pursuit (ld 1.5 m)')
    low.plot(pp[:, 0], np.degrees(pp[:, 2]), color=GREY, ls='--')
    top.set_ylabel('off the path (m)')
    top.set_title('LQR from 0.5 m off: Q = diag(1, 1, 0), three R')
    top.legend(fontsize=8)
    low.set_ylabel('wheel angle (deg)')
    low.set_xlabel('time (s)')
    save(fig, 'lqr.png')


def tube_figure():
    e = np.linspace(-0.6, 0.6, 400)
    tube = 0.3          # vehicle.MPC_LAG_TUBE, tuned 21 Nov
    plain = e ** 2
    beyond = np.maximum(np.abs(e) - tube, 0.0) ** 2
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(e, plain, color=ORANGE, label='today: q e_y²')
    ax.plot(e, beyond, color=BLUE, lw=2.5, label='the tube: q (e_y - clip(e_y, ±0.3))²')
    ax.axvspan(-tube, tube, color=BLUE, alpha=0.08)
    ax.text(0, 0.30, 'free: no pull\nto the centre', ha='center', color=BLUE)
    ax.set_xlabel('e_y: off the path (m)')
    ax.set_ylabel('cost')
    ax.legend(loc='upper center', fontsize=8)
    save(fig, 'tube.png')


def horizon_figure():
    s = np.linspace(0, 12, 200)
    path = 0.6 * np.sin(s / 3.0)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(s, path, color=GREY, lw=6, alpha=0.35, label='the path')
    t = np.linspace(0, 1.5, 31)
    for start, alpha in ((0.0, 1.0), (1.0, 0.45)):
        xs = start + t * 3.0
        ys = 0.6 * np.sin(xs / 3.0) + 0.35 * np.exp(-2.2 * t) * (1 if start == 0 else 0.4)
        ax.plot(xs, ys, color=BLUE, alpha=alpha, marker='o', ms=2.5,
                label='the plan, 30 ticks (1.5 s)' if start == 0 else "the next tick's plan")
        ax.plot(xs[0], ys[0], 'o', color=RED, ms=7)
    ax.annotate('only the first command\nis sent; then plan again', (0.0, 0.35),
                (1.8, 1.05), arrowprops={'arrowstyle': '->', 'color': RED}, color=RED)
    ax.set_xlabel('along the road (m)')
    ax.set_ylabel('sideways (m)')
    ax.set_ylim(-1.1, 1.5)
    ax.legend(loc='lower right', fontsize=8)
    save(fig, 'horizon.png')


def mppi_figure():
    rng = np.random.default_rng(3)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    x = np.linspace(0, 6, 40)
    costs, paths = [], []
    for _ in range(60):
        noise = np.cumsum(rng.normal(0, 0.05, x.size))     # correlated: smooth wiggles
        y = 0.15 + noise * x / 6
        paths.append(y)
        costs.append(float(np.mean(y ** 2)) * 50)
    costs = np.array(costs)
    weights = np.exp(-(costs - costs.min()) / 0.5)
    weights /= weights.sum()
    for y, w in zip(paths, weights):
        ax.plot(x, y, color=ORANGE, alpha=0.15 + 0.85 * w / weights.max(), lw=0.8)
    mean = np.sum(np.array(paths) * weights[:, None], axis=0)
    ax.plot(x, np.zeros_like(x), color=GREY, lw=6, alpha=0.35, label='the path')
    ax.plot(x, mean, color=BLUE, lw=2.5, label='the weighted mean: what it sends')
    ax.plot([], [], color=ORANGE, label='samples: darker = cheaper = more weight')
    ax.set_xlabel('ahead (m)')
    ax.set_ylabel('sideways (m)')
    ax.legend(fontsize=8, loc='lower left')
    save(fig, 'mppi.png')


def gauss(x, mu, sigma):
    return np.exp(-0.5 * ((x - mu) / sigma) ** 2) / (sigma * math.sqrt(2 * math.pi))


def kalman_figure():
    fig, (left, right) = plt.subplots(1, 2, figsize=(9, 3.6))
    x = np.linspace(0, 10, 500)
    prior, p_sig = 4.0, 1.0
    meas, m_sig = 6.0, 0.8
    k = p_sig ** 2 / (p_sig ** 2 + m_sig ** 2)
    post = prior + k * (meas - prior)
    post_sig = math.sqrt((1 - k) * p_sig ** 2)
    left.plot(x, gauss(x, prior, p_sig), color=ORANGE, label='predict (the model)')
    left.plot(x, gauss(x, meas, m_sig), color=GREEN, label='measure (GPS)')
    left.plot(x, gauss(x, post, post_sig), color=BLUE, lw=2.5,
              label=f'fused: K = {k:.2f}, narrower')
    left.set_xlabel('position (m)')
    left.set_yticks([])
    left.legend(fontsize=8)
    left.set_title('One update')
    # Over time: a car at 1 m/s, the model 3% fast, GPS every 1 s.
    rng = np.random.default_rng(1)
    dt, q, r = 0.1, 0.02, 0.6 ** 2
    est, p = 0.0, 0.01
    rows = []
    for i in range(200):
        t = i * dt
        est += 1.03 * dt
        p += q
        if i % 10 == 9:
            z = t + dt + rng.normal(0, 0.6)
            gain = p / (p + r)
            est += gain * (z - est)
            p *= 1 - gain
        rows.append((t + dt, est, math.sqrt(p)))
    rows = np.array(rows)
    right.plot(rows[:, 0], rows[:, 1] - rows[:, 0], color=BLUE, label='estimate - truth')
    right.fill_between(rows[:, 0], -2 * rows[:, 2], 2 * rows[:, 2], color=BLUE, alpha=0.15,
                       label='±2σ: how sure it is')
    right.axhline(0, color=GREY, lw=1)
    right.set_xlabel('time (s)')
    right.set_ylabel('error (m)')
    right.set_title('Predict grows it, each GPS fix shrinks it')
    right.legend(fontsize=8)
    save(fig, 'kalman.png')


def pi_figure():
    """Draw the speed loop on docs/vehicle-model.md's guesses: P, PI, PI with a weight."""
    a_max, drag, tau_t = 1.5, 0.1, 0.1
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for name, kp, ki, b, colour in (('P only (kp 0.3)', 0.3, 0.0, 1.0, ORANGE),
                                    ('PI today (0.3, 0.1)', 0.3, 0.1, 1.0, RED),
                                    ('smooth (1.0, 0.25, weight 0.75)', 1.0, 0.25, 0.75,
                                     BLUE)):
        v = ua = integral = 0.0
        dt = 0.01
        ts, vs = [], []
        for i in range(int(20 / dt)):
            target = 1.0
            e = target - v
            integral += ki * e * dt
            u = min(max(kp * (b * target - v) + integral, 0.0), 1.0)
            ua += (u - ua) * dt / tau_t
            v += (a_max * ua - drag * v) * dt
            ts.append(i * dt)
            vs.append(v)
        ax.plot(ts, vs, color=colour, label=name)
    ax.axhline(1.0, color=GREY, ls='--', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('speed (m/s)')
    ax.set_title('Asked for 1 m/s (the guesses: a_max 1.5, drag 0.1)')
    ax.legend(fontsize=8)
    save(fig, 'pi.png')


# --- the formulas, drawn (WeasyPrint draws no maths markup, and we have no LaTeX) ---------

EQ_SIZE = 15        # pt
EQ_DPI = 100


def _measure(renderer, scratch, tex):
    t = scratch.text(0, 0, f'${tex}$', fontsize=EQ_SIZE)
    box = t.get_window_extent(renderer)
    t.remove()
    return box.width, box.height


def _item_size(renderer, scratch, item):
    """Return (width, height, layout) of a formula piece or a matrix (a list of rows)."""
    if isinstance(item, str):
        w, h = _measure(renderer, scratch, item)
        return w, h, None
    cells = [[_measure(renderer, scratch, c) for c in row] for row in item]
    cols = [max(cells[r][c][0] for r in range(len(item))) for c in range(len(item[0]))]
    row_h = max(h for row in cells for _, h in row) + 8
    gap, bracket = 16, 8
    width = sum(cols) + gap * (len(cols) - 1) + 2 * bracket + 8
    return width, row_h * len(item), (cols, row_h, gap, bracket)


def equation(name, rows):
    """Draw rows of pieces (formula strings, or matrices as lists of rows) as an SVG."""
    scratch = plt.figure(dpi=EQ_DPI)
    renderer = scratch.canvas.get_renderer()
    laid = []
    for row in rows:
        sizes = [_item_size(renderer, scratch, item) for item in row]
        laid.append((row, sizes, max(h for _, h, _ in sizes)))
    plt.close(scratch)
    pad, item_gap, row_gap = 10, 8, 14
    width = max(sum(w for w, _, _ in sizes) + item_gap * (len(sizes) - 1)
                for _, sizes, _ in laid) + 2 * pad
    height = sum(h for _, _, h in laid) + row_gap * (len(laid) - 1) + 2 * pad
    fig = plt.figure(figsize=(width / EQ_DPI, height / EQ_DPI), dpi=EQ_DPI)
    y_top = height - pad
    for row, sizes, row_h in laid:
        y_mid = y_top - row_h / 2
        x = pad
        for item, (w, h, layout) in zip(row, sizes):
            if layout is None:
                fig.text(x / width, y_mid / height, f'${item}$', fontsize=EQ_SIZE,
                         va='center', ha='left')
            else:
                cols, cell_h, gap, bracket = layout
                top, bottom = y_mid + h / 2, y_mid - h / 2
                for left, sign in ((x + 2, 1), (x + w - 2, -1)):
                    xs = [left + sign * bracket, left, left, left + sign * bracket]
                    ys = [top, top, bottom, bottom]
                    fig.add_artist(plt.Line2D([v / width for v in xs],
                                              [v / height for v in ys], lw=1.3,
                                              color='black'))
                for r, cells in enumerate(item):
                    cy = top - cell_h * (r + 0.5)
                    cx = x + 4 + bracket
                    for c, cell in enumerate(cells):
                        fig.text((cx + cols[c] / 2) / width, cy / height, f'${cell}$',
                                 fontsize=EQ_SIZE, va='center', ha='center')
                        cx += cols[c] + gap
            x += w + item_gap
        y_top -= row_h + row_gap
    fig.savefig(os.path.join(OUT, f'eq-{name}.svg'), transparent=True,
                metadata={'Date': None})
    plt.close(fig)


EQUATIONS = {
    'bicycle': [[r'\dot{x} = v\,\cos\theta'], [r'\dot{y} = v\,\sin\theta'],
                [r'\dot{\theta} = \frac{v\,\tan\delta}{L}']],
    'lag': [[r'\tau\,\dot{a} + a = u']],
    'state-space': [[r'\dot{\mathbf{x}} = A\,\mathbf{x} + B\,u'], [r'y = C\,\mathbf{x}']],
    'lateral': [[r'\mathbf{x} =', [[r'e_y'], [r'e_\theta'], [r'\delta']],
                 r',\quad u = \delta_{cmd}'],
                [r'A =', [['0', 'v', '0'], ['0', '0', r'v/L'], ['0', '0', r'-1/\tau_s']],
                 r'\qquad B =', [['0'], ['0'], [r'1/\tau_s']]]],
    'feed-forward': [[r'\delta_{ff} = \arctan(L\,\kappa)'],
                     [r'\delta_{cmd} = \delta_{ff} - K\,\mathbf{x}']],
    'pi': [[r'e = v_{set} - v'],
           [r'\mathrm{throttle} = k_p\,e + k_i \int e\,dt']],
    'speed-plant': [[r'P(s) = \frac{K}{(\tau_v s + 1)(\tau_t s + 1)}', r'\qquad',
                     r'C(s) = k_p + \frac{k_i}{s}']],
    'final-value': [[r'e(\infty) = \frac{1}{1 + C(0)\,P(0)}', r'\qquad',
                     r'k_i > 0 \;\Rightarrow\; C(0) = \infty \;\Rightarrow\; e(\infty) = 0']],
    'setpoint-weight': [[r'\mathrm{throttle} = k_p\,(b\,v_{set} - v) + k_i \int e\,dt',
                         r'\qquad b = 0.75']],
    'lqr-cost': [[r'J = \int_0^\infty \left( \mathbf{x}^T Q\,\mathbf{x} + u^T R\,u \right) dt']],
    'riccati': [[r'A^T P + P A - P B R^{-1} B^T P + Q = 0'],
                [r'K = R^{-1} B^T P', r'\qquad', r'u = -K\,\mathbf{x}']],
    'lqi': [[r'\dot{z} = v_{set} - v', r'\qquad',
             r'\mathbf{x}_{aug} =', [['v'], [r'a_{throttle}'], ['z']]],
            [r'u = -K\,\mathbf{x}_{aug} = -k_v v - k_a a_{throttle} - k_z z']],
    'mpc-cost': [[r'J = \sum_{k=1}^{N} \left[ q_1 \left(e_y - \mathrm{clip}(e_y, \pm w)\right)^2'
                  r' + q_2 e_\theta^2 + q_3 (\delta - \delta_{ff})^2'
                  r' + q_v (v - v_{lim})^2 \right]'],
                 [r'\quad + \sum_{k=1}^{N} \left[ r\,(u - \delta_{ff})^2 + r_\Delta\,\Delta u^2'
                  r' + r_c\,\Delta c^2 \right] + w_s\,s^2'],
                 [r'\mathrm{within}\quad |u| \leq \delta_{max},\quad'
                  r' |\Delta u| \leq \dot{\delta}_{max}\,\Delta t,\quad'
                  r' 0 \leq c \leq v_{cruise}']],
    'qp': [[r'X = \bar{X} + M\,\Delta U'],
           [r'\min_{\Delta U}\; \frac{1}{2}\,\Delta U^T P\,\Delta U + q^T \Delta U'
            r'\quad \mathrm{within\ the\ limits}']],
    'tube': [[r'\mathrm{cost}(e_y) = q_1 \left(e_y - \mathrm{clip}(e_y, -w, w)\right)^2',
              r'\qquad w = 0.3\ \mathrm{m}']],
    'mppi': [[r'w_k = \frac{e^{-c_k/\lambda}}{\sum_j e^{-c_j/\lambda}}', r'\qquad',
              r'u = u_{last} + \sum_k w_k\,\epsilon_k']],
    'closed-loop': [[r'\dot{\mathbf{x}} = (A - B K)\,\mathbf{x}']],
    'lyapunov': [[r'V(\mathbf{x}) > 0,\quad \dot{V}(\mathbf{x}) < 0'
                  r'\;\Rightarrow\; \mathbf{x} \rightarrow 0', r'\qquad',
                  r'\mathrm{e.g.}\ V = \mathbf{x}^T P\,\mathbf{x}']],
    'ekf-predict': [[r'\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, u)'],
                    [r'P^- = F\,P\,F^T + Q']],
    'ekf-update': [[r'K = P^- H^T \left(H P^- H^T + R\right)^{-1}'],
                   [r'\hat{\mathbf{x}} = \hat{\mathbf{x}}^-'
                    r' + K\left(z - h(\hat{\mathbf{x}}^-)\right)'],
                   [r'P = (I - K H)\,P^-']],
    'speed-car': [[r'\tau_v\,\dot{v} + v = K\,u_a', r'\qquad',
                   r'\tau_v = 1/d,\quad K = a_{max}/d']],
}


def main():
    os.makedirs(OUT, exist_ok=True)
    feed_forward_figure()
    lqr_figure()
    tube_figure()
    horizon_figure()
    mppi_figure()
    kalman_figure()
    pi_figure()
    for name, rows in EQUATIONS.items():
        equation(name, rows)
    print('wrote', ', '.join(sorted(os.listdir(OUT))))


if __name__ == '__main__':
    main()
