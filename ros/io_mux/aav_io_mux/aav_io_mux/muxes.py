"""
The two muxes' node logic, without ROS: what each does with each message and each tick.

sensor_mux.py and output_mux.py only turn messages into calls here, and the answers
back into messages and log lines. So the Rust twins (aav_io_mux_rs, io-mux-core) can be
checked against this module, step by step (aav_io_mux_rs/gen_vectors.py writes what it
answers; test_vectors.py checks it still does).

Times are seconds on a steady clock. `alive` is the set of nodes the ROS graph shows
publishing /system/source now. A log line is (level, text), level 'error', 'warning'
or 'info'; None when nothing is logged.
"""

from aav_io_mux import heartbeat, provenance


def screen_names(text):
    """Return the output mux's `screens` parameter's names: blanks dropped."""
    return tuple(n.strip() for n in text.split(',') if n.strip())


class SensorMuxLogic:
    """sensor_mux.py's work: the announcements heard, and the verdict each tick."""

    def __init__(self, source, now, startup=2.0, hold=5.0, grace=30.0):
        self.expected = provenance.source_of(source)
        self.ready_at = now + startup
        self.hold = provenance.Hold(hold, grace_until=now + grace)
        self.announcers = provenance.Announcers()
        self.last_verdict = None

    def hear(self, announcer, entries, now):
        """Keep one /system/source message: its announcer and its entries."""
        self.announcers.hear(announcer, entries, now)

    def tick(self, now, alive, publishers):
        """
        Return (check, log) for this tick; (None, None) while discovery settles.

        publishers: each watched sensor topic -> the nodes publishing it now.
        """
        if now < self.ready_at:
            return None, None
        found = provenance.check_sensors(
            self.expected, self.announcers.entries(alive, now), publishers,
            unheard=self.announcers.unheard(alive))
        grace = self.hold.in_grace(now, found)
        check = self.hold.judge(now, found)
        verdict = (check.problems, check.doubts, grace)
        if verdict == self.last_verdict:
            return check, None
        self.last_verdict = verdict
        want = provenance.kind_name(self.expected)
        if check.problems:
            return check, ('error', f'NOT {want}: ' + '; '.join(check.problems))
        if check.doubts and grace:
            left = self.hold.grace_until - now
            return check, ('warning',
                           f'unsure, waiting for /system/source (the output mux holds WAIT; '
                           f'a missing announcement counts in {left:.0f} s): '
                           + '; '.join(check.doubts))
        if check.doubts:
            return check, ('warning', f'unsure (for {self.hold.hold:g} s): '
                           + '; '.join(check.doubts))
        found = ', '.join(f'{s.name} {provenance.kind_name(s.kind)}'
                          for s in check.sensors) or 'no sensors yet'
        return check, ('info', f'all {want}: {found}')


class OutputMuxLogic:
    """
    output_mux.py's work: the gate, the announcements, the heartbeat, the state line.

    Each call that may change the state returns what to publish on
    /safety/output_mux/state, as (line, log), or None when nothing is published.
    """

    def __init__(self, target, settle=2.0, stale_after=3.0, screens=()):
        self.target = target
        self.gate = provenance.Gate(target, settle=settle, stale_after=stale_after,
                                    screens=tuple(screens))
        self.announcers = provenance.Announcers()
        self.counter = heartbeat.Counter()
        self.last_state = None

    def tick(self, now, alive):
        """Re-judge: the live announcements, any not heard yet, the sensor mux's age."""
        self.gate.entries = self.announcers.entries(alive, now)
        self.gate.unheard = self.announcers.unheard(alive)
        self.gate.update(now)

    def check(self, now, alive, source, ok, problems, doubts):
        """Take one /system/sensor_mux/state message."""
        self.gate.report = provenance.Report(now, source, ok, tuple(problems), tuple(doubts))
        self.tick(now, alive)
        return self.publish_state(now, changes_only=True)

    def source(self, now, alive, announcer, entries):
        """Take one /system/source message."""
        self.announcers.hear(announcer, entries, now)
        self.tick(now, alive)
        return self.publish_state(now, changes_only=True)

    def beat(self, screen, now):
        """Note one beat from a screen in `screens`."""
        self.gate.beat(screen, now)

    def timer(self, now, alive):
        """Re-judge, and say the state: the 2 Hz timer."""
        self.tick(now, alive)
        return self.publish_state(now)

    def command(self, now):
        """Judge one of ego's commands: the heartbeat's counter if it goes on, else None."""
        if self.gate.is_open(now):
            return self.counter.beat()
        return None

    def clear(self, now, alive):
        """Answer a person's /safety/output_mux/clear: (success, message, state to publish)."""
        self.tick(now, alive)
        success, message = self.gate.clear(now)
        return success, message, self.publish_state(now, changes_only=True)

    def publish_state(self, now, changes_only=False):
        """Return (line, log) to publish now, or None: only changes, if changes_only."""
        state = self.gate.state(now)
        if changes_only and state == self.last_state:
            return None
        log = None
        if state != self.last_state:
            log = ('error' if state.startswith('STOP') else 'info', state)
            self.last_state = state
        return state, log
