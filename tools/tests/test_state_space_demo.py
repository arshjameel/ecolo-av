"""state_space_demo: the teaching simulator agrees with the textbook and with hand sums."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import numpy as np
    import state_space_demo as ss  # noqa: E402
    HAVE_SCIPY = True
except ImportError:          # no numpy or scipy here: pixi run test has them
    HAVE_SCIPY = False

pytestmark = pytest.mark.skipif(not HAVE_SCIPY, reason='needs numpy and scipy')


def sort(p):
    return np.array(sorted(p, key=lambda z: (round(z.real, 9), round(z.imag, 9))))


@pytest.mark.parametrize('m, c, k', [(1.0, 0.4, 4.0), (2.0, 10.0, 3.0), (45.0, 300.0, 8000.0)])
def test_poles_are_eigenvalues_are_the_transfer_function_s_roots(m, c, k):
    eig, roots = ss.poles_two_ways(*ss.mass_spring_damper(m, c, k))
    by_hand = sort(np.roots([m, c, k]))         # m s^2 + c s + k = 0
    assert np.allclose(eig, roots) and np.allclose(eig, by_hand)


def test_the_speed_loop_s_poles_match_vehicle_model():
    model = ss.vm.VehicleModel()
    eig, roots = ss.poles_two_ways(*ss.speed_loop(model))
    from_tf = sort(ss.vm.poles(model.speed_closed_loop()))
    assert np.allclose(eig, roots, atol=1e-6) and np.allclose(eig, from_tf, atol=1e-6)


def test_the_quarter_car_s_poles_two_ways():
    a, b, c, d = ss.quarter_car(33.0, 12.0, 8000.0, 300.0, 60000.0)
    eig, roots = ss.poles_two_ways(a, b[:, :1], c, d[:, :1])
    assert np.allclose(eig, roots, rtol=1e-6)
    assert np.all(eig.real < 0)


@pytest.mark.parametrize('v', [0.5, 1.0, 3.0])
def test_the_bicycle_s_jacobian_is_vehicle_model_s_a_and_b(v):
    model = ss.vm.VehicleModel()
    f = (lambda x, u: ss.bicycle(x, u, v, model.wheelbase, model.steer_tau))
    a, b = ss.jacobian(f, [0.0, 0.0, 0.0], 0.0)
    a_ref, b_ref, _, _ = model.lateral(v)
    assert np.allclose(a, a_ref, atol=1e-5) and np.allclose(b, b_ref, atol=1e-5)


def test_euler_converges_to_the_exact_answer_and_rk4_faster():
    a, b, _, _ = ss.mass_spring_damper(1.0, 0.4, 4.0)
    f = (lambda x: a @ x + b[:, 0])
    errors = {}
    for dt in (0.01, 0.001):
        n = int(round(5 / dt))
        ref = ss.exact(a, b, 1.0, [0, 0], dt, n)
        errors[dt] = (np.abs(ss.euler(f, [0, 0], dt, n) - ref).max(),
                      np.abs(ss.rk4(f, [0, 0], dt, n) - ref).max())
    euler_ratio = errors[0.01][0] / errors[0.001][0]
    assert 8 < euler_ratio < 12                  # first order: 10 x smaller dt, 10 x less
    assert errors[0.01][1] < errors[0.001][0]    # RK4 at dt 0.01 beats Euler at 0.001


def test_the_exact_form_is_the_textbook_step_response():
    # m = 1, k = 4, c = 0.4: wn = 2, zeta = 0.1; a unit force settles at x = 1/k.
    a, b, _, _ = ss.mass_spring_damper(1.0, 0.4, 4.0)
    xs = ss.exact(a, b, 1.0, [0, 0], 0.05, 200)
    t = np.arange(201) * 0.05
    wn, z = 2.0, 0.1
    wd = wn * math.sqrt(1 - z * z)
    textbook = (1 - np.exp(-z * wn * t) * (np.cos(wd * t) + z / math.sqrt(1 - z * z)
                                           * np.sin(wd * t))) / 4.0
    assert np.allclose(xs[:, 0], textbook, atol=1e-9)


def test_the_quarter_car_sags_by_m_g_over_k():
    m_s, m_u, k_s, k_t = 33.0, 12.0, 8000.0, 60000.0
    a, b, _, _ = ss.quarter_car(m_s, m_u, k_s, 300.0, k_t)
    x = -np.linalg.solve(a, b[:, 1])             # x' = 0 with gravity alone
    assert x[2] - x[0] == pytest.approx(m_s * 9.81 / k_s)        # the spring: 4.05 cm
    assert -x[2] == pytest.approx((m_s + m_u) * 9.81 / k_t)      # the tyre: 7.4 mm


def test_roll_against_a_hand_calculation():
    # 133 kg sprung, 0.35 m above the roll centre, springs 8000 N/m, track 0.77 m:
    # K = 8000 x 0.77^2 = 4743.2 N m/rad; gravity takes 133 x 9.81 x 0.35 = 456.6 back.
    k_roll = 8000.0 * 0.77 ** 2
    phi = ss.steady_roll(3.0, 133.0, 0.35, k_roll)
    assert phi == pytest.approx(133 * 0.35 * 3.0 / (4743.2 - 456.65), rel=1e-4)
    assert math.degrees(phi) == pytest.approx(1.87, abs=0.01)
    # The roll model's G(0) is the same number per m/s^2.
    a, b, c, d = ss.roll_model(133.0, 0.35, k_roll, 30.0, 0.3)
    assert (-c @ np.linalg.solve(a, b))[0, 0] == pytest.approx(phi / 3.0)


def test_load_transfer_and_rollover_by_hand():
    assert ss.load_transfer(3.0, 181.0, 0.55, 0.77) == pytest.approx(181 * 3 * 0.55 / 0.77)
    assert ss.rollover_limit(0.55, 0.77) == pytest.approx(9.81 * 0.77 / 1.1)   # 0.70 g


def test_the_car_s_corners_from_vehicle_yaml():
    s = ss.car_corners(ss.load_car())
    assert s['m_s'] == pytest.approx(181.0 - 4 * ss.SUSPENSION['unsprung'])
    assert s['k_roll'] == pytest.approx(ss.SUSPENSION['spring'] * 0.75 ** 2)


def test_the_tyres_are_far_faster_than_the_steering_at_our_speeds():
    # Why the kinematic bicycle is enough at <= 3 m/s: the tyres' poles sit far left of
    # the steering's (-1/tau_s, about -6) until the car goes much faster.
    car = ss.load_car()
    steer = -1.0 / ss.vm.VehicleModel().steer_tau
    for v in (1.0, 3.0):
        a, _ = ss.dynamic_bicycle(v, car)
        assert np.all(np.linalg.eigvals(a).real < 3 * steer)


def test_the_numbers_print(capsys):
    assert ss.main(['--no-figures']) == 0
    out = capsys.readouterr().out
    for text in ('Mass-spring-damper', 'speed loop', 'Euler', 'Quarter-car', 'rollover limit',
                 'dynamic bicycle'):
        assert text in out
