"""safety_filter_report: the sums, on made-up drives (no ROS)."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import safety_filter_report as sfr  # noqa: E402


def row(t, v_in, v_out, stop_m=1.0, rules=(), driver='ai', **more):
    values = {'v_in': str(v_in), 'v_out': str(v_out), 'stop_m': str(stop_m),
              'h_speed': 'NaN', 'h_left': 'NaN', 'h_right': 'NaN'}
    values.update({k: str(v) for k, v in more.items()})
    return sfr.Row(t, driver, values, list(rules))


def braking(v0, a, dt=0.01, start=0.0, until=10.0):
    """Return (t, v) for a car at v0 that brakes at a from `start`."""
    out, t, v = [], 0.0, v0
    while t <= until:
        out.append((t, v))
        if t >= start:
            v = max(0.0, v - a * dt)
        t += dt
    return out


def test_summary_counts_rules_per_driver():
    rows = [row(0, 1, 1), row(0.1, 2, 1, rules=['max_speed']),
            row(0.2, 1, 0.5, rules=['speed_barrier', 'lane_left']), row(0.3, 1, 1, driver='nav')]
    s = sfr.summarize(rows)
    assert s['ai']['commands'] == 3 and s['ai']['trimmed'] == 2
    assert s['ai']['rules'] == {'max_speed': 1, 'speed_barrier': 1, 'lane_left': 1}
    assert s['nav'] == {'commands': 1, 'trimmed': 0, 'rules': {}}


def test_closest_ignores_nan():
    rows = [row(0, 1, 1, h_speed=2.0, h_left=0.4), row(1, 1, 1, h_speed=-0.1)]
    w = sfr.closest(rows)
    assert w['h_speed'] == -0.1 and w['h_left'] == 0.4
    assert w['h_right'] != w['h_right']  # NaN: never checked


def test_a_stop_is_measured_against_its_prediction():
    # 1 m/s, braking at 1 m/s² from t = 2: it rolls v²/(2a) = 0.5 m.
    speeds = braking(1.0, 1.0, start=2.0)
    rows = [row(1.0, 1.0, 1.0), row(2.0, 1.0, 0.0, stop_m=0.6, rules=['speed_barrier']),
            row(2.5, 1.0, 0.0, stop_m=0.3)]
    found = sfr.stops(rows, speeds)
    assert len(found) == 1, found
    t, driver, v, pred, meas = found[0]
    assert t == 2.0 and driver == 'ai' and abs(v - 1.0) < 1e-9 and pred == 0.6
    assert abs(meas - 0.5) < 0.01


def test_a_stop_that_never_ends_says_so():
    speeds = [(t / 10, 1.0) for t in range(50)]
    found = sfr.stops([row(1.0, 1.0, 0.0)], speeds)
    assert found[0][4] is None
    assert 'never stopped' in '\n'.join(sfr.report([row(1.0, 1.0, 0.0)], speeds))


def test_slow_cars_and_driver_stops_are_not_filter_stops():
    speeds = braking(0.1, 1.0)
    assert sfr.stops([row(0.5, 1.0, 0.0)], speeds) == []          # too slow to judge
    assert sfr.stops([row(0.5, 0.0, 0.0)], braking(1.0, 1.0)) == []  # the driver stopped
