"""
The safety gate, as spine_bridge hears it. No ROS imports.

The output mux (aav_io_mux) says its state on /safety/output_mux/state, one line:
"OPEN - car: every source REAL", "WAIT - car: ...", "STOP - target car needs REAL - ...".
spine_bridge refuses /spine/arm unless that line says OPEN for its own target, and came
within `timeout` s (Ben, 24 Oct). Armed while the gate is shut, the spine would get no
commands, and its watchdog would only latch it again.

| The bridge's source | Its target | Its commands |
|---|---|---|
| real | car | /control/car/actuator_cmd |
| bench | bench | /control/bench/actuator_cmd |
| sim | sim | /control/sim/actuator_cmd |

Times come from a steady clock (time.monotonic), never the node's: in the simulator that one is
simulated, and a paused simulation must not keep an old OPEN fresh.

The Rust twin is the spine-bridge crate's gate.rs; both say the same words.
"""

from typing import Optional

# Every refusal starts with this: arm_when_open retries only these.
GATE_SHUT = 'the safety gate is shut'
# The bridge's source:= -> the output mux's target:= it reads commands from.
TARGETS = {'real': 'car', 'bench': 'bench', 'sim': 'sim'}
TIMEOUT = 1.5   # s: the output mux says its state twice a second


class OutputGate:
    """The output mux's last state line, and when it came."""

    def __init__(self, target: str, timeout: float = TIMEOUT):
        if target not in TARGETS.values():
            raise ValueError(f'target {target}: use car, bench or sim')
        self.target = target
        self.timeout = timeout
        self.line: Optional[str] = None
        self.time = 0.0

    def heard(self, line: str, now: float) -> None:
        """Keep one line from /safety/output_mux/state, heard at `now` (steady s)."""
        self.line, self.time = line, now

    def refusal(self, now: float) -> Optional[str]:
        """None if the gate is OPEN for this target and fresh; else why not, for people."""
        if self.line is None:
            why = ('no word from the output mux yet (/safety/output_mux/state); '
                   'arm once it says OPEN')
        elif now - self.time > self.timeout:
            why = (f'the output mux has been quiet for {now - self.time:.1f} s; '
                   'arm once it says OPEN')
        elif not self.line.startswith('OPEN - '):
            why = f'{self.line}; arm once it says OPEN'
        elif self.line[len('OPEN - '):].partition(':')[:2] != (self.target, ':'):
            why = (f'the output mux is for another target ({self.line}); '
                   f'this bridge needs target {self.target}')
        else:
            return None
        return f'{GATE_SHUT}: {why}'


# The boot self-test (TODO 255): spine_bridge with require_ready (the car) refuses to arm
# until /safety/self_test/status says READY (level OK) within READY_TIMEOUT s. The same
# words as spine-bridge's ready.rs.
NOT_READY = 'the boot self-test says NOT READY'
READY_TIMEOUT = 2.0   # s: the self-test speaks twice a second


class ReadyGate:
    """The boot self-test's last word (ready, its message), and when it came."""

    def __init__(self, timeout: float = READY_TIMEOUT):
        self.timeout = timeout
        self.ready: Optional[bool] = None
        self.message = ''
        self.time = 0.0

    def heard(self, ready: bool, message: str, now: float) -> None:
        """Keep one /safety/self_test/status: ready if its level is OK; steady s."""
        self.ready, self.message, self.time = ready, message, now

    def refusal(self, now: float) -> Optional[str]:
        """None if the self-test said READY within the timeout; else why not, for people."""
        if self.ready is None:
            why = ('nothing heard from it yet (/safety/self_test/status); '
                   'arm once it says READY')
        elif now - self.time > self.timeout:
            why = f'it has been quiet for {now - self.time:.1f} s; arm once it says READY'
        elif not self.ready:
            why = self.message
        else:
            return None
        return f'{NOT_READY}: {why}'


def try_again(success: bool, message: str, waited: float, give_up: float) -> bool:
    """
    arm_when_open: ask /spine/arm again? Only while the gate alone refused, for give_up s.

    An e-stop's refusal is final: a person pressed it, and a launch must not keep arming
    behind them.
    """
    return not success and message.startswith(GATE_SHUT) and waited < give_up
