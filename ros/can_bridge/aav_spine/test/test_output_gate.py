"""The safety gate for /spine/arm (Ben, 24 Oct): the cases of spine-bridge's tests/gate.rs."""

from aav_spine.output_gate import GATE_SHUT, OutputGate, TARGETS, try_again
import pytest

OPEN = 'OPEN - car: every source REAL'
WAIT = 'WAIT - car: all REAL, settling for 2 s'
STOP = ('STOP - target car needs REAL - camera_front_left is SIM (sim_adapter) '
        '- latched: /safety/output_mux/clear')


def test_refused_before_the_output_mux_is_heard():
    why = OutputGate('car').refusal(0.0)
    assert why.startswith(GATE_SHUT)
    assert '/safety/output_mux/state' in why


@pytest.mark.parametrize('line', [WAIT, STOP])
def test_refused_while_it_waits_or_stops_with_its_reason(line):
    gate = OutputGate('car')
    gate.heard(line, 10.0)
    assert gate.refusal(10.0) == f'{GATE_SHUT}: {line}; arm once it says OPEN'


def test_accepted_once_open():
    gate = OutputGate('car')
    gate.heard(STOP, 10.0)
    assert gate.refusal(10.0)
    gate.heard(OPEN, 11.0)
    assert gate.refusal(11.0) is None


def test_a_quiet_output_mux_counts_as_shut():
    gate = OutputGate('car')
    gate.heard(OPEN, 10.0)
    assert gate.refusal(11.5) is None
    assert gate.refusal(12.0) == (f'{GATE_SHUT}: the output mux has been quiet for 2.0 s; '
                                  'arm once it says OPEN')


def test_open_for_another_target_is_not_this_bridges_gate():
    gate = OutputGate('car')
    gate.heard('OPEN - sim: every source SIM', 0.0)
    assert gate.refusal(0.0) == (f'{GATE_SHUT}: the output mux is for another target '
                                 '(OPEN - sim: every source SIM); this bridge needs '
                                 'target car')
    for target, line in (('bench', 'OPEN - bench: every source BENCH'),
                         ('sim', 'OPEN - sim: every source SIM')):
        own = OutputGate(target)
        own.heard(line, 0.0)
        assert own.refusal(0.0) is None
        own.heard(OPEN, 0.0)
        assert 'another target' in own.refusal(0.0)


def test_arm_when_open_retries_only_the_gate():
    shut = OutputGate('car').refusal(0.0)
    assert try_again(False, shut, waited=5.0, give_up=120.0)
    assert not try_again(False, shut, waited=120.0, give_up=120.0)   # gives up
    assert not try_again(True, 'armed', waited=5.0, give_up=120.0)
    assert not try_again(False, 'e-stop is active; clear /safety/estop first', 5.0, 120.0)


def test_targets_by_source():
    assert TARGETS == {'real': 'car', 'bench': 'bench', 'sim': 'sim'}
    with pytest.raises(ValueError, match='turtle'):
        OutputGate('turtle')


# The boot self-test (TODO 255): the same cases as spine-bridge's tests/ready.rs.
def test_the_ready_gate_refuses_until_the_self_test_says_ready():
    from aav_spine.output_gate import NOT_READY, ReadyGate
    gate = ReadyGate(timeout=1.0)
    why = gate.refusal(0.0)
    assert why.startswith(NOT_READY) and '/safety/self_test/status' in why
    gate.heard(False, 'NOT READY - sensors (STOP: lidar STALE)', 0.0)
    assert gate.refusal(0.0) == f'{NOT_READY}: NOT READY - sensors (STOP: lidar STALE)'
    gate.heard(True, 'READY', 0.0)
    assert gate.refusal(1.0) is None
    assert 'quiet for 1.5 s' in gate.refusal(1.5)
    # Not the gate's words: arm_when_open never retries a self-test's refusal.
    assert not try_again(False, gate.refusal(1.5), 0.0, 10.0)
