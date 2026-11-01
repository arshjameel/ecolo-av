"""
MPC for the whole drive: plan the speed and the steering together, within the car's limits.

No ROS here; numpy and scipy (the QP: aav_track.qp). Unlike the other trackers it sets the
speed itself (`Command.speed`): the shared policy isn't used. Its limits are the same:
the cruise speed, the throttle (the lower of vehicle.yaml's max_accel and the smoother's),
the planned braking (half of max_brake, as the policy's) and the sideways `lateral_accel`.

The plan is over distance, not time: `horizon` steps of `ds` m along the path from the
car's foot. Over distance the path errors don't depend on the speed (the kinematic
bicycle); the speed enters through the steering (its rate per metre is rate / v, its lag
over a step exp(-ds / (v tau_s))). The speed's square w = v^2 is linear in distance:

    w[k+1] - w[k] = 2 a ds                          a: the throttle's or the brake's
    x[k+1] = F_k x[k] + G_k u[k] + h_k              x = [e_y, e_theta, delta], u: command
             (linearised about the last plan's steering and speed)

    cost = sum q.(x - [0, 0, r])^2 + r_u (u - r)^2 + r_rate (u[k] - u[k-1])^2
           + q_speed (w / cruise^2 - 1)^2              r = atan(L kappa), the bend's

subject to, at every step,
    |u| <= full lock
    |u[k] - u[k-1]| <= rate ds / v                 the steering's rate, over the metres
                                                   the step takes: so slower steers more
                                                   per metre (linearised in w: the tangent,
                                                   on the safe side as 1/sqrt(w) is convex)
    -2 b_full ds <= w[k+1] - w[k] <= 2 a ds        the brake (full), the throttle
    w <= the speed limit: cruise^2; lateral_accel / |kappa| at a bend; 0 past the path's
         end; each eased back at the planned braking (so it slows before, not in, a bend,
         and stops at the goal as the policy does)

So it slows for the U-turn when the steering couldn't swing in time at speed, not only
when the bend's sideways acceleration demands it. One QP a tick (real-time iteration:
linearised about the last plan, warm-started from it). It sends u[0], and the speed the
plan has one tick ahead.
"""

from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Sequence

from aav_drive_to_point.approach import planned_deceleration
from aav_track import qp
from aav_track.path import wrap
from aav_track.trackers import Command, Limits, State, SteerLag, Tracker
import numpy as np


class MPCFull(Tracker):
    name = 'mpc_full'
    plans_speed = True            # Command.speed, within the limits given here

    def __init__(self, limits: Limits, model, cruise: float = 1.0, accel: float = 1.0,
                 max_brake: float = 3.0, lateral_accel: float = 1.0, period: float = 0.05,
                 ds: float = 0.3, horizon: int = 30, q: Sequence[float] = (1.0, 2.0, 0.05),
                 r: float = 0.2, r_rate: float = 1.0, q_speed: float = 0.2,
                 min_speed: float = 0.2):
        super().__init__(limits)
        if cruise <= 0 or accel <= 0 or max_brake <= 0 or lateral_accel <= 0 or \
                period <= 0 or ds <= 0 or horizon < 2 or r <= 0 or r_rate < 0 or \
                q_speed < 0 or min_speed <= 0:
            raise ValueError('need cruise, accel, max_brake, lateral_accel, period, ds, r, '
                             'min_speed above 0; horizon >= 2; r_rate, q_speed >= 0')
        self.model = replace(model, wheelbase=limits.wheelbase, max_steering=limits.max_steer)
        self.lag = SteerLag(self.model.steer_tau)
        self.cruise, self.period, self.ds, self.n = cruise, period, ds, horizon
        self.throttle = min(accel, self.model.max_accel)
        self.brake = planned_deceleration(max_brake)      # planned: the policy's half
        self.full_brake = max_brake
        self.lateral_accel = lateral_accel
        self.q, self.r_u, self.r_rate, self.q_speed = np.array(q, float), r, r_rate, q_speed
        self.min_speed = min_speed
        self.asked = None             # m/s: the last speed sent (the plan starts from it)
        self.warm = None              # (z, y) of the last QP
        self.last = None              # (delta per step, w per point) of the last plan
        n = horizon
        self.diff = np.eye(n) - np.eye(n, k=-1)           # u[k] - u[k-1]

    # --- the path ahead -------------------------------------------------------------------

    def limits_ahead(self, s0: float):
        """Return (kappa per step, r per step, w's upper bound per point 0..N), from s0."""
        n, ds, path = self.n, self.ds, self.path
        lim = self.limits
        w_top = self.cruise ** 2
        # Past the horizon too, as far as it takes to stop from cruise: the bound eases back.
        extra = int(math.ceil(w_top / (2.0 * self.brake * ds))) + 1
        points = s0 + ds * np.arange(n + 1 + extra)
        kappa = np.array([path.curvature_at(float(s + 0.5 * ds)) for s in points[:-1]])
        bound = np.full(len(points), w_top)
        bend = np.abs(kappa)
        for k, kk in enumerate(bend):          # a bend bounds both ends of its step
            if kk > 1e-9:
                cap = self.lateral_accel / kk
                bound[k] = min(bound[k], cap)
                bound[k + 1] = min(bound[k + 1], cap)
        # Stop at the path's end: within the planned braking of it, exactly at each point.
        bound = np.minimum(bound, 2.0 * self.brake * np.maximum(path.length - points, 0.0))
        for k in range(len(points) - 2, -1, -1):           # eased back at planned braking
            bound[k] = min(bound[k], bound[k + 1] + 2.0 * self.brake * ds)
        r = np.clip(np.arctan(lim.wheelbase * kappa[:n]), -lim.max_steer, lim.max_steer)
        return kappa[:n], r, bound[:n + 1]

    # --- the model over the plan ----------------------------------------------------------

    def condensed(self, x0, kappa, delta_bar, v_bar):
        """Return (S x0 + H, M): X = S x0 + M U + H, X the states at points 1..N."""
        n, ds, L = self.n, self.ds, self.limits.wheelbase
        tau = self.model.steer_tau
        free = np.zeros((n, 3))
        m = np.zeros((n, 3, n))
        x, mk = np.array(x0, float), np.zeros((3, n))
        for k in range(n):
            a = math.exp(-ds / (max(v_bar[k], self.min_speed) * tau))
            c = 1.0 / (L * math.cos(delta_bar[k]) ** 2)
            d = math.tan(delta_bar[k]) / L - c * delta_bar[k] - kappa[k]
            k2 = kappa[k] ** 2
            half = 0.5 * ds * ds
            f = np.array([[1.0 - half * k2, ds, half * c * (1 + a) / 2],
                          [-ds * k2, 1.0, ds * c * (1 + a) / 2],
                          [0.0, 0.0, a]])
            g = np.array([half * c * (1 - a) / 2, ds * c * (1 - a) / 2, 1.0 - a])
            h = np.array([half * d, ds * d, 0.0])
            x = f @ x + h
            mk = f @ mk
            mk[:, k] += g
            free[k], m[k] = x, mk
        return free, m

    # --- one tick -------------------------------------------------------------------------

    def step(self, state: State, dt: float) -> Command:
        t0 = time.perf_counter()
        lim, n, ds = self.limits, self.n, self.ds
        foot = self.foot(state.x, state.y)
        delta = self.lag.update(state.steer, dt)
        v0 = max(state.v, 0.0) if self.asked is None else self.asked
        top = self.cruise ** 2
        w0 = v0 * v0 / top                                  # scaled: 1 is cruise
        kappa, r, bound = self.limits_ahead(foot.s)
        bound = bound / top

        if self.last is None:
            delta_bar, w_bar = r.copy(), np.full(n + 1, w0)
        else:
            delta_bar, w_bar = self.last
        w_floor = (self.min_speed ** 2) / top
        w_bar = np.maximum(w_bar, w_floor)
        v_bar = np.sqrt(top * 0.5 * (w_bar[:-1] + w_bar[1:]))
        x0 = np.array([foot.cross, wrap(state.yaw - foot.heading), delta])
        free, m = self.condensed(x0, kappa, delta_bar, v_bar)

        # The cost, in z = [u_0..u_N-1, w_1..w_N] (w scaled by cruise^2).
        qd = np.tile(self.q, n)
        mf = m.reshape(3 * n, n)
        target = np.zeros((n, 3))
        target[:, 2] = r
        err = (free - target).reshape(-1)
        e0 = np.zeros(n)
        e0[0] = state.steer
        pu = mf.T @ (qd[:, None] * mf) + self.r_u * np.eye(n) + \
            self.r_rate * self.diff.T @ self.diff
        qu = mf.T @ (qd * err) - self.r_u * r - self.r_rate * self.diff.T @ e0
        p = np.zeros((2 * n, 2 * n))
        p[:n, :n] = pu
        p[n:, n:] = self.q_speed * np.eye(n)
        qv = np.concatenate([qu, -self.q_speed * np.ones(n)])

        # The constraints: rows of A z within [lo, hi].
        rows, lo, hi = [], [], []
        eye = np.eye(n)
        rows.append(np.hstack([eye, np.zeros((n, n))]))          # full lock
        lo.append(np.full(n, -lim.max_steer))
        hi.append(np.full(n, lim.max_steer))
        # The rate over each step: |du| <= g(w_mid), g(w) = rate ds / sqrt(top w), by its
        # tangent at the last plan's w_mid.
        wm_bar = 0.5 * (w_bar[:-1] + w_bar[1:])
        g = lim.max_steer_rate * ds / np.sqrt(top * wm_bar)
        slope = -0.5 * g / wm_bar                                # dg/dw_mid
        mid = 0.5 * (np.eye(n) + np.eye(n, k=-1))                # w_mid[k] from w_1..w_N
        constant = np.zeros(n)
        constant[0] = 0.5 * w0                                   # w_0 isn't a variable
        beta = -slope
        gamma = g - slope * wm_bar - beta * constant
        for sign in (1.0, -1.0):
            rows.append(np.hstack([sign * self.diff, beta[:, None] * mid]))
            lo.append(np.full(n, -np.inf))
            hi.append(gamma + sign * e0)
        # The throttle and the brake, between points.
        dw = np.hstack([np.zeros((n, n)), self.diff])
        start = np.zeros(n)
        start[0] = w0
        rows.append(dw)
        lo.append(-2.0 * self.full_brake * ds / top + start)
        hi.append(2.0 * self.throttle * ds / top + start)
        # The speed limit at each point.
        rows.append(np.hstack([np.zeros((n, n)), eye]))
        lo.append(np.zeros(n))
        hi.append(bound[1:])
        a_mat = np.vstack(rows)
        lo_v, hi_v = np.concatenate(lo), np.concatenate(hi)
        hi_v = np.maximum(hi_v, lo_v)

        z0, y0 = (None, None) if self.warm is None else self.warm
        res = qp.solve(p, qv, a_mat, lo_v, hi_v, z0=z0, y0=y0, eps=1e-4, max_iter=300)
        self.warm = (res.z, res.y)
        u = np.clip(res.z[:n], -lim.max_steer, lim.max_steer)
        w = np.concatenate([[w0], np.clip(res.z[n:], 0.0, None)])

        # The plan's states, for the next tick's linearisation.
        x = free + (m @ u)
        deltas = np.concatenate([[delta], x[:, 2]])
        self.last = (0.5 * (deltas[:-1] + deltas[1:]), w)

        # The QP holds its bounds to its tolerance: the throttle and brake exactly here.
        tick = max(dt, self.period)
        steer = self.limited(float(u[0]), state.steer, tick)
        a0 = float(np.clip(top * (w[1] - w[0]) / (2.0 * ds), -self.full_brake, self.throttle))
        speed = max(0.0, v0 + a0 * tick)
        if a0 > 0:
            speed = min(speed, math.sqrt(top * w[1]))
        # Never above the limit where the car is: the car lags its commands, so a plan
        # that starts from the last command can start past the limit (the goal's braking).
        speed = max(min(speed, math.sqrt(top * max(bound[0], 0.0))),
                    v0 - self.full_brake * tick, 0.0)
        self.asked = speed
        return Command(steer, speed=speed, debug={
            'solve_ms': 1000.0 * (time.perf_counter() - t0),
            'iterations': float(res.iterations), 'solved': float(res.solved),
            'planned_min_speed_mps': float(math.sqrt(top * float(np.min(w)))),
            'feed_forward_rad': float(r[0])})
