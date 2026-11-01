"""The small QP solver (qp.py) against answers known by hand and by scipy's SLSQP."""

from aav_track import qp
import numpy as np
import pytest
from scipy.optimize import minimize


def test_a_box_bound_that_binds():
    # min (z - 2)^2 with z <= 1: z = 1.
    r = qp.solve(np.array([[2.0]]), np.array([-4.0]), np.eye(1), np.array([-np.inf]),
                 np.array([1.0]))
    assert r.solved and r.z == pytest.approx([1.0], abs=1e-3)


def test_an_equality():
    # min z1^2 + z2^2 with z1 + z2 = 1: both 0.5.
    r = qp.solve(2 * np.eye(2), np.zeros(2), np.ones((1, 2)), np.array([1.0]),
                 np.array([1.0]))
    assert r.solved and r.z == pytest.approx([0.5, 0.5], abs=1e-3)


@pytest.mark.parametrize('seed', range(5))
def test_random_problems_match_slsqp(seed):
    rng = np.random.default_rng(seed)
    n, m = 12, 20
    g = rng.normal(size=(n, n))
    p = g @ g.T + 0.1 * np.eye(n)
    q = rng.normal(size=n)
    a = rng.normal(size=(m, n))
    lo, hi = -rng.uniform(0.1, 1.0, m), rng.uniform(0.1, 1.0, m)
    r = qp.solve(p, q, a, lo, hi, eps=1e-6, max_iter=4000)
    ref = minimize(lambda z: 0.5 * z @ p @ z + q @ z, np.zeros(n), jac=lambda z: p @ z + q,
                   method='SLSQP', constraints=[
                       {'type': 'ineq', 'fun': lambda z: hi - a @ z, 'jac': lambda z: -a},
                       {'type': 'ineq', 'fun': lambda z: a @ z - lo, 'jac': lambda z: a}],
                   options={'ftol': 1e-12, 'maxiter': 500})
    assert r.solved
    assert np.all(a @ r.z <= hi + 1e-4) and np.all(a @ r.z >= lo - 1e-4)
    cost = 0.5 * r.z @ p @ r.z + q @ r.z
    assert cost == pytest.approx(ref.fun, abs=1e-4)


def test_a_warm_start_takes_fewer_iterations():
    rng = np.random.default_rng(7)
    n = 10
    g = rng.normal(size=(n, n))
    p, q = g @ g.T + np.eye(n), rng.normal(size=n)
    a, lo, hi = np.eye(n), -0.2 * np.ones(n), 0.2 * np.ones(n)
    cold = qp.solve(p, q, a, lo, hi)
    warm = qp.solve(p, q, a, lo, hi, z0=cold.z, y0=cold.y)
    assert warm.iterations < cold.iterations


def test_bounds_the_wrong_way_round_are_refused():
    with pytest.raises(ValueError):
        qp.solve(np.eye(1), np.zeros(1), np.eye(1), np.array([1.0]), np.array([0.0]))
