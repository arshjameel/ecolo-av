"""The linear model's maths against textbook cases, the nonlinear car, and the firmware."""

from dataclasses import replace
import math
from pathlib import Path
import re

import pytest

try:
    import numpy as np
    from scipy import signal
    from aav_ego import vehicle_model as vm
    HAVE_SCIPY = True
except ImportError:          # plain Python without numpy and scipy: pixi run test has them
    HAVE_SCIPY = False

pytestmark = pytest.mark.skipif(not HAVE_SCIPY, reason='needs numpy and scipy')

ROOT = Path(__file__).resolve().parents[4]
VEHICLE_YAML = ROOT / 'ros/bringup/aav_description/config/vehicle.yaml'


@pytest.fixture
def car():
    return vm.from_vehicle_yaml(str(VEHICLE_YAML))


# --- textbook cases -----------------------------------------------------------------------

def test_a_first_order_step_reaches_63_percent_at_tau(car):
    tf = car.car_tf()
    t, y = vm.step(tf)
    tau = car.speed_tau
    assert np.interp(tau, t, y) == pytest.approx((1 - math.exp(-1)) * car.speed_gain, rel=1e-3)
    info = vm.step_info(t, y)
    assert info.final == pytest.approx(car.speed_gain, rel=1e-3)     # K, the DC gain
    assert info.rise_time == pytest.approx(math.log(9) * tau, rel=1e-3)   # 2.197 tau
    assert info.overshoot == 0.0


@pytest.mark.parametrize('zeta', [0.2, 0.5, 0.7])
def test_second_order_overshoot_matches_the_formula(zeta):
    wn = 3.0
    t, y = vm.step(vm.second_order(wn, zeta))
    info = vm.step_info(t, y, final=1.0)
    assert info.overshoot == pytest.approx(vm.second_order_overshoot(zeta), rel=1e-3)
    # The peak comes at pi / w_d.
    assert info.peak_time == pytest.approx(math.pi / (wn * math.sqrt(1 - zeta ** 2)), rel=1e-3)
    assert vm.zeta_from_overshoot(info.overshoot) == pytest.approx(zeta, rel=1e-3)


def test_the_steering_is_second_order_from_its_two_lags(car):
    wn, zeta = car.steer_wn_zeta()
    tm, tl = car.motor_tau, car.steer_lag
    assert tm == pytest.approx(car.steer_step / (2 * car.steer_rate))
    # (tau_m s + 1)(tau_l s + 1) / (tau_m tau_l) = s^2 + 2 zeta w_n s + w_n^2.
    np.testing.assert_allclose(vm.second_order(wn, zeta)[1],
                               np.polymul([tm, 1], [tl, 1]) / (tm * tl))
    assert zeta >= 1.0                      # two real lags never overshoot
    measured = car.with_measured(steer_overshoot=0.10)
    _, z = measured.steer_wn_zeta()
    t, y = vm.step(measured.steer_tf())
    assert vm.step_info(t, y).overshoot == pytest.approx(0.10, rel=1e-3)
    assert z == pytest.approx(0.5912, abs=1e-4)


def test_pade_matches_the_delay_at_low_frequency():
    T = 0.1
    for order in (1, 2, 3):
        # Up to w T = 0.1 for the first order, 0.3 for the higher ones.
        w = np.array([0.1, 1.0] if order == 1 else [0.1, 1.0, 3.0])
        mag, ph = vm.freq_response(vm.pade(T, order), w)
        np.testing.assert_allclose(mag, 1.0, atol=1e-9)      # all-pass
        np.testing.assert_allclose(np.radians(ph), -w * T, atol=2e-3)


# --- the speed loop -----------------------------------------------------------------------

def test_the_spine_pi_closed_loop_poles(car):
    # No throttle lag: s^2 + (d + a kp) s + a ki.
    g, a, d = car.gains, car.max_accel, car.drag
    cl = car.speed_closed_loop(throttle_lag=False)
    np.testing.assert_allclose(cl[1] / cl[1][0], [1, d + a * g.kp, a * g.ki])
    assert all(p.real < 0 for p in vm.poles(car.speed_closed_loop()))


def test_pole_placement_puts_the_poles_where_asked(car):
    g = car.place_speed_gains(0.9, 1.0)
    p = vm.poles(car.speed_closed_loop(g, throttle_lag=False))
    wn, zeta = vm.wn_zeta(p[0])
    assert (wn, zeta) == (pytest.approx(1.0), pytest.approx(0.9))
    assert g.kp == pytest.approx((1.8 - car.drag) / car.max_accel)
    # The throttle's lag, left out of the design, moves them only a little.
    p = [q for q in vm.poles(car.speed_closed_loop(g)) if q.imag > 0][0]
    assert vm.wn_zeta(p)[1] == pytest.approx(0.9, abs=0.1)


def test_final_value_theorem_against_a_long_simulation(car):
    # P only: e_ss = 1 / (1 + kp K). Simulate the closed loop for a long time.
    p_only = vm.SpeedLoopGains(kp=0.3, ki=0.0)
    ol = car.speed_open_loop(p_only)
    t, y = vm.step(vm.feedback(ol), t_end=200.0)
    e_ss = vm.final_value_step_error(ol)
    assert e_ss == pytest.approx(1 / (1 + 0.3 * car.speed_gain))
    assert 1 - y[-1] == pytest.approx(e_ss, rel=1e-4)
    # The spine's PI: no error left.
    t, y = vm.step(car.speed_closed_loop(), t_end=200.0)
    assert vm.final_value_step_error(car.speed_open_loop()) == 0.0
    assert y[-1] == pytest.approx(1.0, abs=1e-4)


def test_a_ramp_leaves_r_over_kv(car):
    ol = car.speed_open_loop()
    kv = vm.velocity_constant(ol)
    assert kv == pytest.approx(car.gains.ki * car.speed_gain)
    cl = signal.lti(*vm.feedback(ol))
    t = np.linspace(0, 300, 30001)
    _, y, _ = signal.lsim(cl, U=t, T=t)
    assert t[-1] - y[-1] == pytest.approx(1 / kv, rel=1e-3)


def test_the_margins_of_a_textbook_loop():
    # L = 1 / (s (s + 1)): crossover at 0.786 rad/s, phase margin 51.8 deg, no gain margin.
    ol = (np.array([1.0]), np.array([1.0, 1.0, 0.0]))
    m = vm.margins(ol)
    assert m.gain_crossover == pytest.approx(0.7862, abs=1e-3)
    assert m.phase_margin_deg == pytest.approx(51.83, abs=0.05)
    assert math.isinf(m.gain_margin_db)
    # A delay costs w_c T of phase, and the margin runs out at PM / w_c.
    d = vm.margins(ol, 0.2)
    assert d.phase_margin_deg == pytest.approx(51.83 - math.degrees(0.7862 * 0.2), abs=0.05)
    assert m.max_delay == pytest.approx(math.radians(51.83) / 0.7862, rel=1e-3)
    # The delay that uses it up makes the Pade loop marginally stable.
    near = vm.feedback(vm.with_delay(ol, m.max_delay, order=3))
    assert max(p.real for p in vm.poles(near)) == pytest.approx(0.0, abs=2e-3)


def test_the_spine_s_numbers_match_the_rust():
    src = (ROOT / 'firmware/spine/crates/spine-core/src/speed_loop.rs').read_text()
    block = src[src.index('impl Default for SpeedLoopConfig'):]
    g = vm.SpeedLoopGains()
    for name in ('kp', 'ki', 'integral_limit'):
        assert float(re.search(rf'{name}: ([\d.]+)', block).group(1)) == getattr(g, name)
    spine = (ROOT / 'firmware/spine/crates/spine-core/src/spine.rs').read_text()
    assert float(re.search(r'max_accel: ([\d.]+)', spine).group(1)) == g.accel_slew
    board = (ROOT / 'firmware/spine/boards/esp32-umod/src/main.rs').read_text()
    tick = re.search(r'const TICK: Duration = Duration::from_millis\((\d+)\)', board)
    assert int(tick.group(1)) / 1000 == g.tick
    full = re.search(r'const FULL_THROTTLE_SPEED: f32 = ([\d.]+);', board)
    assert float(full.group(1)) == vm.FULL_THROTTLE_SPEED


def test_the_smooth_profile_matches_the_rust():
    src = (ROOT / 'firmware/spine/crates/spine-core/src/speed_loop.rs').read_text()
    block = src[src.index('pub fn smooth() -> Self'):]
    block = block[:block.index('}')]
    for name in ('kp', 'ki', 'integral_limit', 'setpoint_weight'):
        assert float(re.search(rf'\b{name}: ([\d.]+)', block).group(1)) == \
            getattr(vm.SMOOTH, name)
    assert vm.SpeedLoopGains().setpoint_weight == 1.0


def test_a_setpoint_weight_moves_only_the_zero(car):
    plain = car.speed_closed_loop()
    np.testing.assert_allclose(vm.poles(car.speed_closed_loop(vm.SMOOTH)),
                               vm.poles(car.speed_closed_loop(replace(vm.SMOOTH,
                                                                      setpoint_weight=1.0))))
    # Weight 1 is the plain PI's T = L / (1 + L), zero at -ki / kp.
    same = car.speed_closed_loop(replace(car.gains, setpoint_weight=1.0))
    np.testing.assert_allclose(same[0], plain[0])
    np.testing.assert_allclose(same[1], plain[1])
    g = vm.SMOOTH
    np.testing.assert_allclose(np.roots(car.speed_closed_loop(g)[0]),
                               [-g.ki / (g.setpoint_weight * g.kp)])
    assert vm.dc_gain(car.speed_closed_loop(g)) == pytest.approx(1.0)


def test_the_smooth_profile_does_not_overshoot_today_s_does(car):
    t, y = vm.step(car.speed_closed_loop(), t_end=60.0)
    assert vm.step_info(t, y).overshoot == pytest.approx(0.148, abs=0.005)   # TODO 27
    t, y = vm.step(car.speed_closed_loop(vm.SMOOTH), t_end=60.0)
    info = vm.step_info(t, y)
    assert info.overshoot < 0.001
    assert info.settling_time < 9.0                     # today's: 12.7 s
    assert all(abs(p.imag) < 1e-9 for p in vm.poles(car.speed_closed_loop(vm.SMOOTH)))


def test_the_controller_numbers_match_nav2():
    import yaml
    nav2 = yaml.safe_load((ROOT / 'ros/drivers/aav_navigation/config/nav2.yaml').read_text())
    cs = nav2['controller_server']['ros__parameters']
    assert 1 / cs['controller_frequency'] == vm.Delays().control_period
    rpp = cs['FollowPath']
    la = vm.Lookahead()
    assert (rpp['lookahead_time'], rpp['min_lookahead_dist'], rpp['max_lookahead_dist']) == \
        (la.time, la.shortest, la.longest)


# --- the lateral model --------------------------------------------------------------------

def bicycle(state, delta_cmd, v, L, tau):
    """Return the nonlinear car's rates about the path y = 0: [y, theta, delta]'."""
    y, theta, delta = state
    return np.array([v * math.sin(theta), v * math.tan(delta) / L, (delta_cmd - delta) / tau])


@pytest.mark.parametrize('v', [0.5, 1.0, 3.0, -0.5])
def test_the_linear_a_is_the_jacobian_of_the_bicycle(car, v):
    a, b, _, _ = car.lateral(v)
    h = 1e-6
    x0 = np.zeros(3)
    jac = np.column_stack([(bicycle(x0 + h * e, 0.0, v, car.wheelbase, car.steer_tau)
                            - bicycle(x0 - h * e, 0.0, v, car.wheelbase, car.steer_tau)) / (2 * h)
                           for e in np.eye(3)])
    np.testing.assert_allclose(a, jac, atol=1e-8)
    du = (bicycle(x0, h, v, car.wheelbase, car.steer_tau)
          - bicycle(x0, -h, v, car.wheelbase, car.steer_tau)) / (2 * h)
    np.testing.assert_allclose(b[:, 0], du, atol=1e-8)


def test_the_open_loop_eigenvalues_dont_move_with_speed(car):
    for v in (0.5, 1.0, 3.0):
        eig = np.sort(np.linalg.eigvals(car.lateral(v)[0]).real)
        np.testing.assert_allclose(eig, [-1 / car.steer_tau, 0, 0], atol=1e-12)


def test_pure_pursuit_is_damped_at_0_707_and_scales_with_v_over_ld():
    # With no steering lag, the closed loop is s^2 + (2 v / ld) s + 2 v^2 / ld^2.
    car = vm.VehicleModel(steer_lag=1e-6, steer_rate=1e6)
    for v, ld in ((1.0, 1.5), (3.0, 4.0)):
        cl = np.linalg.eigvals(car.lateral_closed_loop(v, ld))
        pair = [p for p in cl if p.imag > 1e-6][0]
        wn, zeta = vm.wn_zeta(pair)
        assert zeta == pytest.approx(1 / math.sqrt(2), rel=1e-3)
        assert wn == pytest.approx(math.sqrt(2) * v / ld, rel=1e-3)


def open_loop(car, v, order):
    a, b, _, _ = car.lateral(v, order)
    k = np.zeros((1, a.shape[0]))
    k[0, :2] = car.pursuit_gains(v)[0, :2]
    num, den = signal.ss2tf(a, b, k, np.zeros((1, 1)))
    return (vm.trim(num[0]), den), a - b @ k


def test_the_four_state_model_agrees_with_the_three_state_one(car):
    v = 1.0
    (l1, a1), (l2, a2) = open_loop(car, v, 1), open_loop(car, v, 2)
    np.testing.assert_allclose(l1[1], car.lateral_open_loop(v)[1])
    # One lag with the same mean delay: the same response well below the steering's w_n.
    w = np.array([0.05, 0.5, 1.5])
    m1, p1 = vm.freq_response(l1, w)
    m2, p2 = vm.freq_response(l2, w)
    np.testing.assert_allclose(m1, m2, rtol=0.02)
    np.testing.assert_allclose(p1, p2, atol=0.5)
    assert max(np.linalg.eigvals(a1).real) < 0 and max(np.linalg.eigvals(a2).real) < 0


# --- discrete time ------------------------------------------------------------------------

def test_the_discrete_model_matches_the_continuous_one_at_the_samples(car):
    a, b, c, d = car.lateral(1.0)
    T = 0.05
    phi, gam, _, _, _ = signal.cont2discrete((a, b, c, d), T, method='zoh')
    u = 0.1                                   # a held steering step: what ZOH assumes
    n = 40
    x = np.zeros((3, 1))
    xs = []
    for _ in range(n):
        x = phi @ x + gam * u
        xs.append(x[:, 0].copy())
    t = np.arange(n + 1) * T
    _, _, xc = signal.lsim(signal.StateSpace(a, b, np.eye(3), np.zeros((3, 1))),
                           U=np.full(len(t), u), T=t)
    np.testing.assert_allclose(np.array(xs), xc[1:], atol=1e-9)


def test_the_spine_s_discrete_loop_is_its_continuous_one(car):
    z = np.linalg.eigvals(car.speed_loop_discrete())
    s = vm.poles(car.speed_closed_loop())
    np.testing.assert_allclose(np.sort(np.abs(z)), np.sort(np.abs(np.exp(s * car.gains.tick))),
                               rtol=2e-3)


def test_the_spine_s_discrete_loop_is_the_spine_s_code(car):
    # Run the update exactly as speed_loop.rs does, on the ZOH plant, from v = 0 to 1 m/s.
    g = car.gains
    a, b, c, d = car.speed_state_space()
    phi, gam, _, _, _ = signal.cont2discrete((a, b, c, d), g.tick, method='zoh')
    x, integral = np.zeros((2, 1)), 0.0
    big = car.speed_loop_discrete()
    z = np.zeros((3, 1))
    r = 1.0
    ref = np.zeros((3, 1))
    ref[:2, :] = gam * (g.kp + g.ki * g.tick) * r
    ref[2, 0] = g.ki * g.tick * r
    for _ in range(500):
        e = r - (c @ x).item()
        integral = integral + g.ki * e * g.tick       # no clamp: stays inside 0..0.3 here
        u = g.kp * e + integral
        x = phi @ x + gam * u
        z = big @ z + ref
        np.testing.assert_allclose(z[:2], x, atol=1e-12)
    assert integral < g.integral_limit


def test_a_late_command_still_leaves_the_lateral_loop_stable(car):
    now = max(abs(np.linalg.eigvals(car.lateral_discrete(1.0, delay_steps=0))))
    late = max(abs(np.linalg.eigvals(car.lateral_discrete(1.0, delay_steps=1))))
    assert now < 1 and late < 1
    # The discrete loop with no delay is the continuous one, sampled.
    cont = max(abs(np.exp(np.linalg.eigvals(car.lateral_closed_loop(1.0)) * 0.05)))
    assert now == pytest.approx(cont, rel=0.01)


# --- numbers in, numbers out --------------------------------------------------------------

def test_from_vehicle_yaml_reads_the_guesses(car):
    import yaml
    cfg = yaml.safe_load(VEHICLE_YAML.read_text())
    assert car.wheelbase == cfg['vehicle']['wheelbase']
    assert car.max_steering == cfg['vehicle']['max_steering_angle']
    for key in ('max_accel', 'throttle_lag', 'max_brake', 'brake_lag', 'drag', 'steer_rate',
                'steer_lag'):
        assert getattr(car, key) == cfg['actuators'][key]


def test_with_measured_takes_the_actuator_tests_report(car):
    report = {'actuators.throttle_lag': (0.25, 'test 4.1: command vs measured level'),
              'actuators.steer_lag': (0.15, 'test 3.11'),
              'actuators.steer_offset': (0.01, 'test 3.12'),     # not linear: skipped
              'actuators.drag': 0.2}
    m = car.with_measured(report, steer_rate=0.5)
    assert (m.throttle_lag, m.steer_lag, m.drag, m.steer_rate) == (0.25, 0.15, 0.2, 0.5)
    assert m.max_accel == car.max_accel
    assert car.throttle_lag == 0.1                   # the original is untouched
    assert m.speed_gain == pytest.approx(1.5 / 0.2)
    with pytest.raises(KeyError):
        car.with_measured({'actuators.throttle_lagg': 0.2})


def test_the_summary_has_every_table(car):
    titles = [title for title, _ in vm.summary(car)]
    assert len(titles) == 7
    for _, rows in vm.summary(car):
        assert all(len(r) == len(rows[0]) for r in rows)


# --- dead times: the actuator tests' FOPDT fits, in the model ----------------------------

def test_dead_times_default_to_none_and_come_from_the_report(car):
    assert (car.throttle_dead_time, car.brake_dead_time, car.steer_dead_time) == (0, 0, 0)
    m = car.with_measured({'actuators.throttle_dead_time': (0.05, 'test 4.1'),
                           'actuators.steer_dead_time': 0.03, 'brake_dead_time': 0.08})
    assert (m.throttle_dead_time, m.steer_dead_time, m.brake_dead_time) == (0.05, 0.03, 0.08)
    cfg = {'vehicle': {'wheelbase': 1.57, 'max_steering_angle': 0.5},
           'actuators': {'throttle_dead_time': 0.04}}
    assert vm.from_config(cfg).throttle_dead_time == 0.04


def test_a_dead_time_adds_to_the_loops_delay_and_costs_phase(car):
    m = car.with_measured(throttle_dead_time=0.1, steer_dead_time=0.05)
    for case in vm.CASES:
        assert m.speed_delay(case) == pytest.approx(car.speed_delay(case) + 0.1)
        assert m.lateral_delay(case) == pytest.approx(car.lateral_delay(case) + 0.05)
    before = vm.margins(car.lateral_open_loop(1.0), car.lateral_delay('typical'))
    after = vm.margins(m.lateral_open_loop(1.0), m.lateral_delay('typical'))
    # Phase lost = crossover x the extra delay.
    lost = math.degrees(before.gain_crossover * 0.05)
    assert before.phase_margin_deg - after.phase_margin_deg == pytest.approx(lost, rel=0.02)


def test_the_plants_with_their_dead_times_as_pade(car):
    m = car.with_measured(throttle_dead_time=0.1, brake_dead_time=0.1)
    assert vm.dc_gain(m.speed_plant_tf(pade_order=2)) == pytest.approx(
        vm.dc_gain(car.speed_plant_tf()))
    assert len(m.brake_plant_tf(pade_order=2)[1]) == len(car.brake_plant_tf()[1]) + 2
    assert m.speed_plant_tf()[1].tolist() == car.speed_plant_tf()[1].tolist()


# --- the speed LQI (TODO 3) ---------------------------------------------------------------

def test_the_speed_lqi_is_stable_and_pushes_the_right_way(car):
    g = car.speed_lqi()
    assert g.k_speed > 0 and g.k_integral > 0 and g.k_accel > 0
    assert g.design_lag == max(car.throttle_lag, car.brake_lag)
    pl = car.speed_lqi_open_loop(g)
    assert all(p.real < 0 for p in vm.poles(vm.feedback(pl)))
    assert vm.margins(pl, car.speed_delay('worst')).phase_margin_deg > 45


def test_heavier_error_weights_give_stiffer_gains(car):
    soft = car.speed_lqi(vm.SpeedLqiWeights(q_speed=1.0))
    stiff = car.speed_lqi(vm.SpeedLqiWeights(q_speed=4.0))
    assert stiff.k_speed > soft.k_speed


def test_the_speed_lqi_gains_are_spine_core_s():
    src = (ROOT / 'firmware/spine/crates/spine-core/src/speed_loop.rs').read_text()
    block = src[src.index('pub const MODEL: LqiConfig'):]
    block = block[:block.index('};')]
    g = vm.VehicleModel().speed_lqi()
    for name in ('k_speed', 'k_accel', 'k_integral', 'setpoint_weight'):
        rust = float(re.search(rf'\b{name}: ([\d.]+)', block).group(1))
        assert rust == pytest.approx(getattr(g, name), rel=1e-3), name
