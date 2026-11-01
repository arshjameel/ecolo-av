"""
A small dense QP solver, the ADMM of OSQP (Stellato et al. 2020), in numpy.

No ROS here. For mpc_full's plan, a few dozen numbers a tick:

    minimise   1/2 z' P z + q' z
    subject to lo <= A z <= hi            (lo = hi for an equality; +-inf for none)

The stock solver would be OSQP itself (Apache-2.0, conda-forge `osqp`); this file stands
in for it so the car's environment needs nothing new. Same steps as OSQP's (its paper,
algorithm 1): one factorisation of P + sigma I + rho A'A per problem, then cheap
iterations; warm-started from the last answer. It stops when both residuals are under
`eps` (absolute, infinity norm), or after `max_iter` with its best (still within the
bounds of A z only to that tolerance: callers clip what must hold exactly).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.linalg import cho_factor, cho_solve


@dataclass
class Result:
    z: np.ndarray            # the answer
    y: np.ndarray            # the constraints' multipliers (for a warm start)
    iterations: int
    solved: bool             # both residuals under eps


def solve(p: np.ndarray, q: np.ndarray, a: np.ndarray, lo: np.ndarray, hi: np.ndarray,
          z0: Optional[np.ndarray] = None, y0: Optional[np.ndarray] = None,
          rho: float = 0.1, sigma: float = 1e-6, alpha: float = 1.6, eps: float = 1e-4,
          max_iter: int = 400) -> Result:
    """Return the QP's answer (P symmetric positive semidefinite; lo <= hi)."""
    n, m = p.shape[0], a.shape[0]
    if np.any(lo > hi):
        raise ValueError('a constraint with lo above hi')
    # Equality rows get a stiffer rho, as OSQP does.
    rhos = np.where(np.isclose(lo, hi), 1e3 * rho, rho)
    rhos = np.where(np.isinf(lo) & np.isinf(hi), 1e-6, rhos)
    factor = cho_factor(p + sigma * np.eye(n) + a.T @ (rhos[:, None] * a))
    x = np.zeros(n) if z0 is None else np.array(z0, dtype=float)
    y = np.zeros(m) if y0 is None else np.array(y0, dtype=float)
    z = np.clip(a @ x, lo, hi)
    for k in range(1, max_iter + 1):
        xt = cho_solve(factor, sigma * x - q + a.T @ (rhos * z - y))
        zt = a @ xt
        x = alpha * xt + (1.0 - alpha) * x
        mixed = alpha * zt + (1.0 - alpha) * z
        z_new = np.clip(mixed + y / rhos, lo, hi)
        y = y + rhos * (mixed - z_new)
        z = z_new
        if k % 10 == 0 or k == max_iter:
            primal = np.max(np.abs(a @ x - z), initial=0.0)
            dual = np.max(np.abs(p @ x + q + a.T @ y), initial=0.0)
            if primal < eps and dual < eps:
                return Result(x, y, k, True)
    return Result(x, y, max_iter, False)
