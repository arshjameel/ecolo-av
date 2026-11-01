"""
LQR steering about the path's own bend, with a preview of the bends ahead.

No ROS here. The model is aav_ego's VehicleModel (the steering as one lag tau_s), but
linearised about driving the path's curvature kappa, not about driving straight. The bend
needs the steady steering r = atan(L kappa); the state is the error from that, and the
input is each tick's change of command c (MPC's cost, without its bounds):

    x = [e_y, e_theta, delta - r, c - r]   e_y + left of the path, e_theta car - path
    u = c[k] - c[k-1]

    e_y'     = v e_theta
    e_theta' = v / (L cos^2 r) (delta - r) - v kappa^2 e_y      (tan's slope at r, and
    delta'   = (c - delta) / tau_s                               the bend's 1 / (1 - kappa e_y))

At full lock the steering turns the car 1 / cos^2(0.5) = 1.3 times faster than about
straight, so the gains differ; they come from a table by speed and by |r| (linear in
between), each the discrete Riccati solution at the controller's tick (scipy).
cost = sum x' diag(q, r) x + r_rate u^2: r_rate is what keeps the steering near its rate.

The bend ahead is known: the path. It enters the model as the steady steering's change,
x[k+1] = Phi x[k] + Gam u[k] - (e3 + e4) (r[k+1] - r[k]), and LQR with preview
(Tomizuka) adds the optimal answer to it, over `preview` s:

    u = -K x - sum_j g_j (r[j] - r[j+1]),   g_j = S^-1 Gam' (Phi - Gam K)'^j P (e3 + e4)
    S = r_rate + Gam' P Gam

so the car starts to steer into a bend (or out of one) before it gets there: the steering
needs about 1.7 s at 0.6 rad/s to swing from one lock to the other. The old LQR (29 Oct)
was linearised about straight, with atan(L kappa) at the car as its only feed-forward; it
steered only once the bend had arrived, and swung wide of the U-turn.
Below `min_speed` the table's first row.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Sequence, Tuple

from aav_track.path import wrap
from aav_track.trackers import Command, Limits, State, SteerLag, Tracker
import numpy as np
from scipy import signal
from scipy.linalg import solve_discrete_are


class LQR(Tracker):
    name = 'lqr'

    def __init__(self, limits: Limits, model, q: Sequence[float] = (1.0, 2.0, 0.05),
                 r: float = 0.2, r_rate: float = 100.0, min_speed: float = 0.5,
                 max_speed: float = 6.0, speed_step: float = 0.25, period: float = 0.05,
                 preview: float = 2.0, bend_step: float = 0.1):
        super().__init__(limits)
        if min_speed <= 0 or max_speed <= min_speed or speed_step <= 0 or r <= 0 or \
                r_rate <= 0 or period <= 0 or preview < 0 or bend_step <= 0:
            raise ValueError('need 0 < min_speed < max_speed; speed_step, r, r_rate, '
                             'period, bend_step above 0; preview >= 0')
        self.model = replace(model, wheelbase=limits.wheelbase, max_steering=limits.max_steer)
        self.lag = SteerLag(self.model.steer_tau)
        self.period = period
        self.n_preview = int(round(preview / period))
        self.min_speed = min_speed
        self.speeds = np.arange(min_speed, max_speed + 1e-9, speed_step)
        self.bends = np.append(np.arange(0.0, limits.max_steer, bend_step), limits.max_steer)
        self.q = np.diag([*q, r])
        self.r = np.array([[r_rate]])
        # One row of [K (4), g_0 .. g_N-1] for each (speed, |r|).
        self.table = np.array([[self.design(float(v), float(b)) for b in self.bends]
                               for v in self.speeds])

    def matrices(self, v: float, bend: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return (Phi, Gam) about steering `bend` rad at v m/s, one tick.

        The state is [e_y, e_theta, delta - r, c - r], c the last command; the input is
        this tick's change of command, so the cost on it (r_rate) keeps the steering
        within its rate, and the preview starts a swing in time.
        """
        L, ts = self.limits.wheelbase, self.model.steer_tau
        kappa = math.tan(bend) / L
        a = np.array([[0.0, v, 0.0],
                      [-v * kappa * kappa, 0.0, v / (L * math.cos(bend) ** 2)],
                      [0.0, 0.0, -1.0 / ts]])
        b = np.array([[0.0], [0.0], [1.0 / ts]])
        phi3, gam3, _, _, _ = signal.cont2discrete(
            (a, b, np.eye(3), np.zeros((3, 1))), self.period, method='zoh')
        phi = np.block([[phi3, gam3], [np.zeros((1, 3)), np.ones((1, 1))]])
        gam = np.vstack([gam3, [[1.0]]])
        return phi, gam

    def design(self, v: float, bend: float) -> np.ndarray:
        """Return [K, g]: the feedback and the preview gains at (v, bend)."""
        phi, gam = self.matrices(v, bend)
        p = solve_discrete_are(phi, gam, self.q, self.r)
        s = self.r + gam.T @ p @ gam
        k = np.linalg.solve(s, gam.T @ p @ phi)
        closed = (phi - gam @ k).T
        # The bend's change moves both delta - r and c - r: E = e3 + e4.
        g, w = np.empty(self.n_preview), p[:, 2] + p[:, 3]     # P E, then (Phi_cl')^j P E
        for j in range(self.n_preview):
            g[j] = float(np.linalg.solve(s, gam.T @ w)[0])
            w = closed @ w
        return np.concatenate([k[0], g])

    def row(self, v: float, bend: float) -> np.ndarray:
        """Return the table's row at (v, |bend|), linear in between, its ends beyond."""
        v = float(np.clip(v, self.speeds[0], self.speeds[-1]))
        b = float(np.clip(abs(bend), self.bends[0], self.bends[-1]))
        i = min(int(np.searchsorted(self.speeds, v, side='right')) - 1, len(self.speeds) - 2)
        j = min(int(np.searchsorted(self.bends, b, side='right')) - 1, len(self.bends) - 2)
        tv = (v - self.speeds[i]) / (self.speeds[i + 1] - self.speeds[i])
        tb = (b - self.bends[j]) / (self.bends[j + 1] - self.bends[j])
        t = self.table
        return ((1 - tv) * ((1 - tb) * t[i, j] + tb * t[i, j + 1]) +
                tv * ((1 - tb) * t[i + 1, j] + tb * t[i + 1, j + 1]))

    def gains(self, v: float, bend: float = 0.0) -> np.ndarray:
        """Return K on [e_y, e_theta, delta - r, c - r] at v, about steering `bend` rad."""
        return self.row(v, bend)[:4]

    def preview_gains(self, v: float, bend: float = 0.0) -> np.ndarray:
        """Return g_0 .. g_N-1, the weights on each tick's change of the bend ahead."""
        return self.row(v, bend)[4:]

    def bend_steering(self, s: float) -> float:
        """Return r = atan(L kappa) at s m along the path, within full lock."""
        lim = self.limits
        return max(-lim.max_steer, min(lim.max_steer,
                                       math.atan(lim.wheelbase * self.path.curvature_at(s))))

    def step(self, state: State, dt: float) -> Command:
        foot = self.foot(state.x, state.y)
        delta = self.lag.update(state.steer, dt)
        v = max(state.v, 0.0)
        ahead = foot.s + max(v, self.min_speed) * self.period * np.arange(self.n_preview + 1)
        r = np.array([self.bend_steering(float(s)) for s in ahead])
        row = self.row(v, r[0])
        k, g = row[:4], row[4:]
        x = np.array([foot.cross, wrap(state.yaw - foot.heading), delta - r[0],
                      state.steer - r[0]])
        preview = -float(g @ (r[:-1] - r[1:]))
        steer = self.limited(state.steer - float(k @ x) + preview, state.steer, dt)
        return Command(steer, debug={'feed_forward_rad': float(r[0]),
                                     'preview_rad': preview})
