"""The report: true times, plant boundaries, what shaped the commands, the fallbacks."""

import math

from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_sim_models.actuator_model import ActuatorConfig
import fake_car
import pytest
import synthetic as sy

CFG = ActuatorConfig()
CTX = reports.Context(current={f'actuators.{k}': v for k, v in vars(CFG).items()},
                      command_lead=fake_car.DT)
SETTINGS = pr.Settings(cap=3.0)
NAN = float('nan')


def row(**values):
    r = dict.fromkeys(reports.COLUMNS, NAN)
    r['phase'] = 'x'
    r.update(values)
    return r


# --- true times ---------------------------------------------------------------------------

def test_signal_uses_each_message_once_at_its_arrival():
    rows = [row(t=0.00, speed=1.0, t_speed=-0.005), row(t=0.02, speed=1.0, t_speed=-0.005),
            row(t=0.04, speed=1.5, t_speed=0.031), row(t=0.06, speed=1.5, t_speed=0.031)]
    assert reports.signal(rows, 'speed') == ([-0.005, 0.031], [1.0, 1.5])


def test_signal_without_stamps_reads_the_rows():
    # An older log: no t_* columns, so the rows' own times.
    rows = [row(t=0.0, speed=1.0), row(t=0.02, speed=1.0), row(t=0.04, speed=1.5)]
    assert reports.signal(rows, 'speed') == ([0.0, 0.02, 0.04], [1.0, 1.0, 1.5])
    assert reports.at_times(rows, 'speed', [0.03]) == pytest.approx([1.25])


def test_an_old_csv_still_parses_and_analyses():
    p = pr.plan('coast', SETTINGS)
    rows, _ = fake_car.drive(p.steps)
    old_columns = reports.COLUMNS[:reports.COLUMNS.index('ego_speed')]
    records = [{k: (r[k] if k == 'phase' else reports.number(r[k])) for k in old_columns}
               for r in rows]
    back = reports.parse_rows(records)
    assert math.isnan(back[0]['t_speed']) and math.isnan(back[0]['ego_speed'])
    rep = reports.analyse('coast', back, CTX)
    assert rep.yaml['actuators.drag'][0] == pytest.approx(CFG.drag, rel=0.05)


# --- plant boundaries ---------------------------------------------------------------------

def test_every_fit_says_where_it_goes_from_and_to():
    p = pr.plan('speed-up', SETTINGS)
    rows, _ = fake_car.drive(p.steps)
    rep = reports.analyse('speed-up', rows, CTX)
    text = reports.table(rep)
    assert 'PATH [1]:' in text
    lines = reports.yaml_lines(rep)
    for key in ('max_accel', 'throttle_lag'):
        line = next(x for x in lines.splitlines() if x.strip().startswith(key + ':'))
        assert '->' in line and 'rms' in line
    assert 'throttle level (/vehicle/throttle measured) -> wheel speed' in \
        rep.yaml['actuators.max_accel'][1]
    assert 'throttle demand' in rep.yaml['actuators.throttle_lag'][1]
    # Every suggestion names its path.
    assert all('->' in comment or 'where' in comment for _, comment in rep.yaml.values())


# --- what shaped the commands -------------------------------------------------------------

def test_shaping_on_the_legacy_car():
    s = reports.shaping(['legacy_uart_bridge', 'spine_bridge'],
                        {'legacy_uart_bridge': {'full_throttle_speed': 3.0}},
                        {'max_speed': 1.0, 'max_acceleration': 1.0})
    assert s['backend'] == 'legacy' and s['speed_loop'].startswith('none: open loop')
    assert s['full_throttle_speed'] == 3.0
    assert s['ego']['max_acceleration'] == 1.0 and s['ego']['max_deceleration'] == 'unknown'
    ctx = reports.with_shaping(reports.Context(), s)
    assert (ctx.backend, ctx.full_throttle_speed, ctx.command_lead) == ('legacy', 3.0, 0.0)


def test_shaping_in_gazebo_and_on_the_spine():
    gz = reports.shaping(['sim_adapter'], {'sim_adapter': {
        'speed_loop': 'smooth', 'spine': False, 'actuator_model': True, 'rate_hz': 50.0}}, {})
    assert (gz['backend'], gz['speed_loop'], gz['command_lead']) == ('gazebo', 'smooth', 0.02)
    assert gz['ego'].startswith('unknown')
    spine = reports.shaping(['spine_bridge'], {}, {})
    assert spine['backend'] == 'spine' and spine['speed_loop'].startswith('unknown')
    nobody = reports.shaping([], {}, {})
    assert nobody['backend'] == nobody['speed_loop'] == 'unknown'


def test_an_old_meta_without_shaping():
    meta = {'wheelbase': 1.57, 'max_steering': 0.5, 'vehicle_yaml': {'actuators.drag': 0.1}}
    ctx = reports.context_from_meta(meta)
    assert ctx.backend == 'unknown' and ctx.command_lead == 0.0
    meta['shaping'] = reports.shaping(['sim_adapter'], {}, {})
    assert reports.context_from_meta(meta).command_lead == pytest.approx(0.02)


# --- the fallbacks: no brake level, no throttle level -------------------------------------

def brake_rows():
    p = pr.plan('brake', SETTINGS)
    rows, _ = fake_car.drive(p.steps)
    for r in rows:
        r['brake'] = NAN                          # no brake level reported
    return rows


def test_brake_with_no_level_reads_the_imu():
    rep = reports.analyse('brake', brake_rows(), CTX)
    assert rep.yaml['actuators.max_brake'][0] == pytest.approx(CFG.max_brake, rel=0.05)
    assert rep.yaml['actuators.brake_lag'][0] == pytest.approx(CFG.brake_lag, abs=0.03)
    assert 'IMU ax' in rep.yaml['actuators.max_brake'][1]


def test_brake_with_no_level_and_no_demand_times_the_whole_stop():
    rows = brake_rows()
    for r in rows:
        r['brake_cmd'] = NAN
    rep = reports.analyse('brake', rows, CTX)
    assert 'actuators.max_brake' not in rep.yaml and 'actuators.brake_lag' not in rep.yaml
    assert any(r.what == 'stop: dead time' for r in rep.rows)
    assert any('/vehicle/brake command' in n for n in rep.notes)


# m/s: the ESP32's full_throttle_speed here, the speed full throttle holds (max_accel /
# drag), so its open loop reaches about the speed asked for.
FTS = 15.0


def legacy_rows(top):
    """Return a legacy speed-up: ego ramps to `top`, the ESP32 drives speed / FTS open loop."""
    t = sy.times(15)
    ego = [min(top, max(0.0, 1.0 * (x - 0.5))) for x in t]
    throttle = [min(1.0, v / FTS) if v >= 0.05 else 0.0 for v in ego]
    tf, v, _ = sy.drive(t, throttle, 1.5, 0.1, 0.05, 0.1)
    ht, hv = sy.hall(tf, v)
    out = []
    for ti, e in zip(t, ego):
        k = max(i for i, x in enumerate(ht) if x <= ti + 1e-9)
        out.append(row(t=ti, phase=f'speed up to {top:g}', cmd_speed=top, ego_speed=e,
                       t_ego=ti, speed=hv[k], t_speed=ht[k]))
    return out


def test_the_legacy_open_loop_throttle_gives_max_accel_through_hall_pulses():
    ctx = reports.Context(current={'actuators.drag': 0.1}, backend='legacy',
                          full_throttle_speed=FTS)
    rep = reports.analyse('speed-up', legacy_rows(3.0), ctx)
    value, comment = rep.yaml['actuators.max_accel']
    assert value == pytest.approx(1.5, rel=0.05)
    assert "ego's speed / full_throttle_speed" in comment
    assert rep.yaml['actuators.throttle_lag'][0] == pytest.approx(0.15, abs=0.05)
    # At walking pace the pulses are too far apart for the throttle's 0.15 s delay.
    slow = reports.analyse('speed-up', legacy_rows(1.0), ctx)
    assert 'actuators.throttle_lag' not in slow.yaml
    assert any('not measured' in n for n in slow.notes)
    # Without the backend's full_throttle_speed: no fit, and it says what's missing.
    none = reports.analyse('speed-up', legacy_rows(1.0), reports.Context())
    assert 'actuators.max_accel' not in none.yaml
    assert any('full_throttle_speed' in n for n in none.notes)
