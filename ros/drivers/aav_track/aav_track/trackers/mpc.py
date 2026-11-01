"""
MPC steering: plan the next `horizon` steering commands, within the steering's limits.

No ROS here; numpy and scipy only. The model is LQR's (aav_ego VehicleModel.lateral(v),
x = [e_y, e_theta, delta]) with the path's bend as a known input, held for a tick at a
time (zero-order hold, scipy.signal.cont2discrete), at the controller's 20 Hz:

    x[k+1] = Phi x[k] + Gam u[k] + Gam_k kappa[k]      kappa: the path's curvature ahead,
                                                       read where the car will be
    cost   = sum (x' Q x + r (u - L kappa)^2 + r_rate (u[k] - u[k-1])^2)  + x[N]' P x[N]

P is the discrete Riccati solution (the cost beyond the horizon, as LQR would steer it).
Each tick it solves for u[0..N-1] with |u| <= full lock and |u[k] - u[k-1]| <= rate dt as
hard bounds (scipy SLSQP), warm-started from the last plan shifted by one tick; when
the unconstrained answer already fits, it skips the solver. It sends u[0].

The speed is held through the horizon (the speed policy sets it); `min_speed` as in LQR.
"""

from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Dict, Sequence, Tuple

from aav_track.path import wrap
from aav_track.trackers import Command, Limits, State, SteerLag, Tracker
import numpy as np
from scipy import signal
from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize


class MPC(Tracker):
    name = 'mpc'

    def __init__(self, limits: Limits, model, period: float = 0.05, horizon: int = 30,
                 q: Sequence[float] = (1.0, 2.0, 0.05), r: float = 2.0, r_rate: float = 1.0,
                 min_speed: float = 0.5, speed_step: float = 0.1):
        super().__init__(limits)
        if period <= 0 or horizon < 2 or r <= 0 or r_rate < 0 or min_speed <= 0:
            raise ValueError('need period, r, min_speed above 0, horizon >= 2, r_rate >= 0')
        self.model = replace(model, wheelbase=limits.wheelbase, max_steering=limits.max_steer)
        self.lag = SteerLag(self.model.steer_tau)
        self.period, self.n = period, horizon
        self.q, self.r, self.r_rate = np.diag(q), r, r_rate
        self.min_speed, self.speed_step = min_speed, speed_step
        self.cache: Dict[float, Tuple[np.ndarray, ...]] = {}
        self.plan = None              # the last solution, for the warm start
        n = horizon
        # D U: the rate of change, u[k] - u[k-1]; the first row's u[-1] comes from outside.
        self.diff = np.eye(n) - np.eye(n, k=-1)

    def reset(self, path):
        super().reset(path)
        # Keep the plan: the new path starts where the car is, as the old one did.

    def matrices(self, v: float) -> Tuple[np.ndarray, ...]:
        """Return (S, M, G, H, Qbar) for speed v: X = S x0 + M U + G kappa."""
        key = round(max(v, self.min_speed) / self.speed_step) * self.speed_step
        if key in self.cache:
            return self.cache[key]
        a, b, _, _ = self.model.lateral(key)
        bk = np.array([[0.0], [-key], [0.0]])        # the bend turns the path, not the car
        both = np.hstack([b, bk])
        phi, gam2, _, _, _ = signal.cont2discrete(
            (a, both, np.eye(3), np.zeros((3, 2))), self.period, method='zoh')
        gam, gk = gam2[:, :1], gam2[:, 1:]
        n, nx = self.n, 3
        s = np.zeros((n * nx, nx))
        m = np.zeros((n * nx, n))
        g = np.zeros((n * nx, n))
        power = np.eye(nx)
        powers = [power]
        for _ in range(n):
            power = phi @ power
            powers.append(power)
        for k in range(n):                          # row block k holds x[k+1]
            s[k * nx:(k + 1) * nx] = powers[k + 1]
            for j in range(k + 1):
                m[k * nx:(k + 1) * nx, j] = (powers[k - j] @ gam)[:, 0]
                g[k * nx:(k + 1) * nx, j] = (powers[k - j] @ gk)[:, 0]
        p = solve_discrete_are(phi, gam, self.q, np.array([[self.r]]))
        qbar = np.kron(np.eye(n), self.q)
        qbar[-nx:, -nx:] = p
        h = m.T @ qbar @ m + self.r * np.eye(n) + self.r_rate * self.diff.T @ self.diff
        out = (s, m, g, h, qbar)
        self.cache[key] = out
        return out

    def solve(self, x0, kappa, previous) -> Tuple[np.ndarray, int]:
        """Return (the plan u[0..N-1], solver iterations: 0 when no solver was needed)."""
        lim, n = self.limits, self.n
        s, m, g, h, qbar = self.matrices(self.speed)
        free = s @ x0 + g @ kappa
        ff = lim.wheelbase * kappa                  # the linear model's steady steering
        e0 = np.zeros(n)
        e0[0] = previous
        f = m.T @ qbar @ free - self.r * ff - self.r_rate * self.diff.T @ e0
        step = lim.max_steer_rate * self.period
        u = np.linalg.solve(h, -f)
        rate = np.diff(np.concatenate([[previous], u]))
        if np.all(np.abs(u) <= lim.max_steer) and np.all(np.abs(rate) <= step + 1e-12):
            return u, 0
        if self.plan is not None:
            start = np.concatenate([self.plan[1:], self.plan[-1:]])
        else:
            start = np.full(n, previous)
        start = self.feasible(start, previous, step)
        lower = np.vstack([self.diff, -self.diff])            # D U - e0 within +-step
        bound = np.concatenate([step + e0, step - e0])
        res = minimize(lambda z: 0.5 * z @ h @ z + f @ z, start, jac=lambda z: h @ z + f,
                       method='SLSQP', bounds=[(-lim.max_steer, lim.max_steer)] * n,
                       constraints=[{'type': 'ineq', 'fun': lambda z: bound - lower @ z,
                                     'jac': lambda z: -lower}],
                       options={'maxiter': 50, 'ftol': 1e-9})
        return self.feasible(res.x, previous, step), int(res.nit)

    def feasible(self, u, previous, step):
        """Return u within full lock and the rate, walked from `previous`."""
        out, last = np.empty_like(u), previous
        for k, value in enumerate(u):
            last = float(np.clip(np.clip(value, last - step, last + step),
                                 -self.limits.max_steer, self.limits.max_steer))
            out[k] = last
        return out

    def step(self, state: State, dt: float) -> Command:
        t0 = time.perf_counter()
        foot = self.foot(state.x, state.y)
        delta = self.lag.update(state.steer, dt)
        self.speed = max(state.v, self.min_speed)
        along = foot.s + self.speed * self.period * np.arange(self.n)
        kappa = np.array([self.path.curvature_at(float(a)) for a in along])
        x0 = np.array([foot.cross, wrap(state.yaw - foot.heading), delta])
        plan, iterations = self.solve(x0, kappa, state.steer)
        self.plan = plan
        # The plan's first command, within this tick's own dt (the node's tick may be late).
        steer = self.limited(float(plan[0]), state.steer, max(dt, self.period))
        return Command(steer, debug={'solve_ms': 1000.0 * (time.perf_counter() - t0),
                                     'iterations': float(iterations),
                                     'feed_forward_rad': math.atan(
                                         self.limits.wheelbase * kappa[0])})
