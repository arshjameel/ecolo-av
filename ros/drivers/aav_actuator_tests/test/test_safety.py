"""The safety rules: what may run where, how fast, and when it must stop."""

from aav_actuator_tests import safety
from aav_actuator_tests.safety import Kind
import pytest

REAL, SIM, BENCH, REPLAY = (safety.SOURCE_REAL, safety.SOURCE_SIM, safety.SOURCE_BENCH,
                            safety.SOURCE_REPLAY)


def test_source_kind_errs_towards_real():
    assert safety.source_kind({}) is Kind.UNKNOWN
    assert safety.source_kind({'gazebo_adapter': [SIM, SIM]}) is Kind.SIM
    assert safety.source_kind({'spine_bridge': [BENCH], 'mock_imu': [BENCH]}) is Kind.BENCH
    assert safety.source_kind({'legacy_uart_bridge': [REAL]}) is Kind.REAL
    # One real actuator beside a simulator is the real car.
    assert safety.source_kind({'gazebo_adapter': [SIM], 'legacy_uart_bridge': [REAL]}) \
        is Kind.REAL
    assert safety.source_kind({'replay_source': [REPLAY]}) is Kind.REPLAY


@pytest.mark.parametrize('kind', [Kind.REAL, Kind.UNKNOWN])
def test_the_real_car(kind):
    d = safety.decide(kind, sim=False, stands=False, stage=4)
    assert d.allowed and d.real and d.confirm
    assert d.cap == safety.REAL_CAP
    # Lower is fine; above 1 m/s is refused, not quietly clamped.
    assert safety.decide(kind, False, False, 4, 0.5).cap == 0.5
    assert not safety.decide(kind, False, False, 4, 1.5).allowed


def test_stage_three_needs_stands_on_the_real_car():
    assert not safety.decide(Kind.REAL, False, stands=False, stage=3).allowed
    d = safety.decide(Kind.REAL, False, stands=True, stage=3)
    assert d.allowed and d.cap == safety.STANDS_CAP and d.confirm
    # And stage 4 can't run on stands.
    assert not safety.decide(Kind.REAL, False, stands=True, stage=4).allowed


def test_sim_flag_must_match_the_source():
    assert not safety.decide(Kind.REAL, sim=True, stands=False, stage=4).allowed
    assert not safety.decide(Kind.UNKNOWN, sim=True, stands=False, stage=4).allowed
    assert not safety.decide(Kind.SIM, sim=False, stands=False, stage=4).allowed
    d = safety.decide(Kind.SIM, sim=True, stands=False, stage=4)
    assert d.allowed and not d.confirm and not d.real
    assert d.cap == safety.SIM_DEFAULT_CAP
    assert safety.decide(Kind.SIM, True, False, 4, 5.0).cap == 5.0
    assert not safety.decide(Kind.SIM, True, False, 4, 6.0).allowed
    # Stage 3 runs in Gazebo without stands; with them, the stands' cap.
    assert safety.decide(Kind.SIM, True, False, 3).allowed
    assert safety.decide(Kind.SIM, True, True, 3).cap == safety.STANDS_CAP


def test_bench_and_replay():
    d = safety.decide(Kind.BENCH, False, False, 4)
    assert d.allowed and not d.confirm and d.cap == safety.REAL_CAP
    assert not safety.decide(Kind.BENCH, False, False, 4, 2.0).allowed
    assert not safety.decide(Kind.REPLAY, False, False, 4).allowed


def test_bad_caps():
    for cap in (0.0, -1.0, float('nan'), float('inf')):
        assert not safety.decide(Kind.SIM, True, False, 4, cap).allowed


def test_confirmation_is_the_exact_name():
    assert safety.confirmed('speed-up', 'speed-up')
    assert safety.confirmed('  speed-up\n', 'speed-up')
    for typed in ('', 'y', 'yes', 'Speed-up', 'speed', None):
        assert not safety.confirmed(typed, 'speed-up')


def test_envelope():
    env = safety.Envelope(max_speed=1.3, max_distance=20.0, max_time=60.0)
    assert env.breach(1.0, 10.0, 30.0) is None
    assert 'speed' in env.breach(1.4, 10.0, 30.0)
    assert 'speed' in env.breach(-1.4, 10.0, 30.0)                # reverse counts too
    assert 'drove' in env.breach(1.0, 20.5, 30.0)
    assert 'ran' in env.breach(1.0, 10.0, 61.0)
    assert env.breach(float('nan'), 0.0, 0.0) is not None        # unknown speed: stop
    assert safety.speed_limit(1.0) == pytest.approx(1.3)


def test_taken_over():
    ours = [(0.5, 0.0), (0.5, 0.1)]
    assert not safety.taken_over(ours, (0.5, 0.1))
    assert safety.taken_over(ours, (0.0, 0.0))     # the joystick's deadman: stop
    assert safety.taken_over([], (0.5, 0.0))


def test_the_geofence_stops_a_test_on_the_ground():
    stop = safety.geofence_stop
    for state in ('NEAR: 0.9 m from the edge', 'OUTSIDE: ...', 'LOST: no GPS', 'JUMP: 4 m'):
        assert stop(state, 0.1, stands=False, real=True)
        assert stop(state, 0.1, stands=False, real=False)       # Gazebo too
        assert stop(state, 0.1, stands=True, real=True) is None  # wheels off the ground
    assert stop('OK: 20 m from the edge', 0.1, stands=False, real=True) is None
    assert stop('COASTING: ...', 0.1, stands=False, real=True) is None   # a person drives
    # STARTING only keeps a test from starting.
    assert stop('STARTING: settling', 0.1, False, True) is None
    assert stop('STARTING: settling', 0.1, False, True, starting=True)
    # On the real car the geofence must be heard, and not in stands mode.
    assert 'geofence must run' in stop(None, None, stands=False, real=True)
    assert stop('OK: fine', 3.0, stands=False, real=True)
    assert stop(None, None, stands=False, real=False) is None
    assert 'stands mode' in stop('STANDS: NO FENCE', 0.1, stands=False, real=True)


def test_checklists_come_from_the_stage():
    assert any('OFF the ground' in line for line in safety.checklist(stands=True))
    assert any('safety driver' in line for line in safety.checklist(stands=False))


def test_bench_needs_bench_and_bench_needs_bench_mode():
    assert safety.decide(Kind.BENCH, False, True, 3, bench=True).allowed
    assert not safety.decide(Kind.BENCH, False, True, 3, bench=False).allowed
    assert not safety.decide(Kind.REAL, False, True, 3, bench=True).allowed
    assert not safety.decide(Kind.SIM, True, True, 3, bench=True).allowed


def test_unseen_speed_stops_after_four_seconds():
    spinning = [(t / 10, 0.3, 0.0) for t in range(0, 45)]
    assert safety.unseen(spinning[:35]) is None          # 3.4 s: not yet (Hall clicks)
    assert "can't be seen" in safety.unseen(spinning)
    moving = [(t / 10, 0.3, 0.2) for t in range(0, 25)]
    assert safety.unseen(moving) is None
    parked = [(t / 10, 0.0, 0.0) for t in range(0, 25)]
    assert safety.unseen(parked) is None


def test_runaway_speed_rising_while_the_command_falls():
    stuck = [(t / 10, max(0.0, 0.3 - t / 10), 0.3 + t / 20) for t in range(0, 15)]
    assert 'stuck throttle' in safety.runaway(stuck)
    braking = [(t / 10, 0.0, max(0.0, 0.3 - t / 10)) for t in range(0, 15)]
    assert safety.runaway(braking) is None
    speeding_up = [(t / 10, 0.3, t / 50) for t in range(0, 15)]
    assert safety.runaway(speeding_up) is None


def test_the_preflight_refuses_the_real_car_without_a_driven_wheel_speed():
    why = safety.preflight_refusal(Kind.REAL, True, 'front', False, 'open')
    assert why and '49b' in why and '--no-drive' in why
    assert safety.preflight_refusal(Kind.UNKNOWN, True, None, None, None)
    assert safety.preflight_refusal(Kind.REAL, False, 'front', True, 'pi') is None
    assert safety.preflight_refusal(Kind.SIM, True, 'front', None, None) is None
    assert 'no-timeout' in safety.preflight_refusal(Kind.REAL, True, 'rear', True, 'open')
    assert 'pi' in safety.preflight_refusal(Kind.REAL, True, 'rear', False, 'pi')
    assert safety.preflight_refusal(Kind.REAL, True, 'rear', False, 'open') is None
