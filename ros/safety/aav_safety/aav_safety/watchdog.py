"""
The sensor watchdog's logic: is each sensor alive, sane and moving? No ROS here.

sensor_watchdog (the node) tells this module what arrived and when; this module says, for
each input, OK or what is wrong, and whether the automatic drivers must stop (twist_mux's
sensors lock). docs/full-simulation.md step 6; docs/simulation.md, "Sensor failures
on purpose".

What counts as wrong, per input (each check only where its rule asks for it):

  NEVER   nothing yet
  STALE   nothing good for max_age seconds (a cable pulled, a driver dead)
  NAN     the newest message has NaN or inf in it (a driver's bad parse)
  NO FIX  a GPS fix that says it has none, or NaN: the GPS counts as lost (STALE after
          max_age), as navsat_transform ignores those fixes too
  FROZEN  the same values again and again for frozen_after seconds (a driver stuck,
          sending its last message with fresh stamps). For a camera or a cloud (the lidar)
          only while the car moves: a still car in a still world sees the same picture,
          and a noiseless lidar (the bench's) the same cloud. For a heartbeat: its count
          stopped rising
  JUMP    two GPS fixes further apart than max_jump_speed allows in the time between
          their stamps, plus jump_margin for the fixes' own noise. Latched: the position
          after a jump can't be trusted until a person says so (clear())
  DISAGREE  the wheels' speed or turn against the IMU's and the GPS's, where the odom
          rule has a cross_check (wheel_check.py): a wheel speed frozen while the car
          speeds up, brakes, turns or drives on faster. Latched, as a jump
  STUCK   an actuator input (kind actuator, ActuatorState: what it was told, what it did)
          whose measured value stayed more than follow_tolerance off its command for
          follow_persist seconds: steering stuck in a bend (TODO 207; found by the
          actuator faults, TODO 159). Latched, as a jump. NaN on either side is not a
          fault here (the spine fills a side it doesn't know with NaN): only no check
  OVER    a speed input (kind speed: ego's commanded speed, the wheels' measured speed)
          whose measured speed has been more than over_margin above the command for
          over_warn seconds while the car was not slowing: a warning only, for people
  OVERSPEED  the same for over_persist seconds: a throttle stuck open, or a speed loop
          that runs away (TODO 207). Latched, as a jump. "Not slowing": the measured speed
          fell by less than over_slowing m/s^2 over the last SLOWING_WINDOW s, so a
          healthy stop, which lags its command by up to 0.7 m/s for a second (the car
          model), never counts. Speeds compare as sizes, so reversing counts the same.
          Only fresh numbers count (16 Nov): a wheel speed older than the speed input's
          max_age, or a command after a gap of more than max_age, is no check (the
          watched odom input says STALE for the wheels; this input for ego)
  FAULT   a status input (kind status) whose newest message says min_level or worse
          (1 WARN, 2 ERROR, 3 STALE): one of our nodes says it is in trouble, in its own
          words (the costmap waiting for odometry, the planner finding no path)
  LOST    where the rule says latch (the HUD's heartbeat): the input went wrong (STALE,
          FROZEN...) after its start-up. Latched: a program that died or froze once may
          do it again, so a person looks before Nav2 drives again (clear())

The latch rule (Ben, 24 Oct: a HUD that keeps crashing holds Nav2 until a person clears
it): one dropout is forgiven while the input starts, within latch_grace seconds of its
first OK (a window opening, its first slow frames): it only holds the drivers
while it lasts, as before. Any other dropout latches LOST, even once it is back. update(),
once a tick, makes that call. After a stall of the watchdog itself (a busy computer,
SIGSTOP), seen on a steady clock, it decides nothing for `settle` s: the silence it sees
then is its own, while the queued messages (and, in a simulator, the clock) catch up.

A heartbeat input (kind heartbeat) is a program saying it is alive, not a sensor: the
HUD's /hud/heartbeat, the number of frames its screen has drawn. Silence (STALE, or NEVER
before its first) means the screen froze or died; a count that stops rising is FROZEN.

A node input (kind node) is one of our own nodes (Ben, 14 Nov: the watchdog warns about
our nodes, warn only, no lock): its topic is the node's full name (/planner), and it
"arrives" each tick the node is in the ROS graph. Gone for max_age (it died, and its 1 s
respawn didn't bring it back, or it hangs before it joins): STALE, a warning. A node input
never stops the car (stop must be false): the costmap's and odometry's age checks and
twist_mux's timeout already stop it while a node it needs is down.

A status input (kind status) watches one of our nodes through its own status topic
(diagnostic_msgs/DiagnosticStatus; Ben, 18 Nov: warn only by default, TODO 171). Only the
messages whose name is the rule's status_name count (one topic can carry two: 'costmap
local' and 'costmap global'; empty: all). Silence for max_age is STALE (a node that hangs
in the graph still says nothing), unless the rule says quiet_ok: the node speaks only while
it works (the planner, one a plan; the path follower, while a path runs), so silence is
quiet, OK, and an old complaint no longer counts. Its stop switch works as any input's:
false by default (respawn, mppi_lag's costmap age and twist_mux's timeout already stop
the car), true to hold Nav2 and the AI slot while it isn't OK.

A detections input (kind detections) watches a camera through its detector's output: the
car's camera_stream nodes (aav_camera_stream) keep their pictures inside, and send a
Detection2DArray for every picture, boxes or none, all on one topic. Only the messages
whose header.frame_id is the rule's frame_id count, so one input watches one camera.
Silence means its stream, decoder or detector stopped; NaN, a broken detector. No FROZEN:
an empty list repeats on a clear road (a stuck Pi camera stops the stream instead).

Times are seconds on the node's clock (Gazebo's, in the simulator).
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from aav_safety.wheel_check import CrossCheck, DISAGREE, WheelCheck

OK = 'OK'
NEVER = 'NEVER'
STALE = 'STALE'
NAN = 'NAN'
FROZEN = 'FROZEN'
JUMP = 'JUMP'
LOST = 'LOST'
STUCK = 'STUCK'
OVER = 'OVER'
OVERSPEED = 'OVERSPEED'
FAULT = 'FAULT'
WARN_ONLY = (OVER,)         # states that warn but never stop the drivers

KINDS = ('odom', 'imu', 'gps', 'image', 'cloud', 'heartbeat', 'detections', 'node',
         'actuator', 'speed', 'status')
LEVELS = {1: 'WARN', 2: 'ERROR', 3: 'STALE'}    # diagnostic_msgs/DiagnosticStatus's
EARTH_RADIUS = 6371000.0    # m
MOVING = 0.2                # m/s: faster than this, a camera's picture or a cloud must change
MOVING_KINDS = ('image', 'cloud')    # frozen only while the car moves
SLOWING_WINDOW = 0.5        # s: the overspeed check's look back, for "is it slowing?"


@dataclass(frozen=True)
class Rule:
    """How to watch one input. 0 turns a check off."""

    name: str
    topic: str
    kind: str                   # one of KINDS: which message, and what is in it
    max_age: float              # s without a good message: STALE
    stop: bool = True           # stop the automatic drivers while it isn't OK
    frozen_after: float = 0.0   # s of identical values: FROZEN
    max_jump_speed: float = 0.0  # m/s between two GPS fixes: JUMP
    jump_margin: float = 5.0    # m more, for the noise of two fixes (0.5 m each, in Gazebo)
    cross_check: Optional[CrossCheck] = None    # odom only: the wheels against the IMU
    frame_id: str = ''          # detections only: the camera whose messages count
    latch: bool = False         # a dropout after its start-up latches LOST, until clear()
    latch_grace: float = 5.0    # s after its first OK: one dropout forgiven
    follow_tolerance: float = 0.0   # actuator only: |command - measured| above it ...
    follow_persist: float = 1.0     # ... for this many s: STUCK (latched)
    over_margin: float = 0.0    # speed only: m/s over the command (0 off) ...
    over_warn: float = 1.0      # ... for this many s while not slowing: OVER (a warning)
    over_persist: float = 2.0   # ... for this many s: OVERSPEED (latched)
    over_slowing: float = 0.3   # m/s^2: slowing at least this fast doesn't count
    status_name: str = ''       # status only: the messages of this name count ('' all)
    min_level: int = 2          # status only: this level or worse is a FAULT (1 WARN, 2 ERROR)
    quiet_ok: bool = False      # status only: silence is quiet, not STALE

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f'{self.name}: kind {self.kind!r} is not one of {KINDS}')
        if not self.max_age > 0.0:
            raise ValueError(f'{self.name}: max_age must be a positive number of seconds')
        if self.cross_check is not None and self.kind != 'odom':
            raise ValueError(f'{self.name}: only an odom input has a cross check')
        if (self.kind == 'detections') != bool(self.frame_id):
            raise ValueError(f'{self.name}: a detections input, and only one, names its '
                             "camera's frame_id")
        if self.kind == 'detections' and self.frozen_after > 0.0:
            raise ValueError(f'{self.name}: detections have no frozen check (an empty list '
                             'repeats on a clear road)')
        if self.kind == 'node' and self.stop:
            raise ValueError(f'{self.name}: a node input only warns (stop: false; Ben, '
                             '14 Nov)')
        if self.kind == 'node' and not self.topic.startswith('/'):
            raise ValueError(f"{self.name}: a node input's topic is the node's full name, "
                             'such as /planner')
        if self.follow_tolerance > 0.0 and self.kind != 'actuator':
            raise ValueError(f'{self.name}: only an actuator input has a follow check')
        if not (self.follow_tolerance >= 0.0 and self.follow_persist > 0.0):
            raise ValueError(f'{self.name}: follow_tolerance must be 0 or more, '
                             'follow_persist above 0 s')
        if self.over_margin > 0.0 and self.kind != 'speed':
            raise ValueError(f'{self.name}: only a speed input has an overspeed check')
        if not (self.over_margin >= 0.0 and 0.0 < self.over_warn <= self.over_persist
                and self.over_slowing >= 0.0):
            raise ValueError(f'{self.name}: over_margin and over_slowing must be 0 or more, '
                             'and 0 < over_warn <= over_persist')
        if not self.latch_grace >= 0.0:
            raise ValueError(f'{self.name}: latch_grace must be 0 or more seconds')
        if self.kind != 'status' and (self.status_name or self.quiet_ok):
            raise ValueError(f'{self.name}: only a status input has status_name or quiet_ok')
        if self.min_level not in LEVELS:
            raise ValueError(f'{self.name}: min_level is 1 (WARN), 2 (ERROR) or 3 (STALE)')

    def counts(self, frame_id: str) -> bool:
        """Report whether a message from this frame is this input's (detections: its camera)."""
        return self.kind != 'detections' or frame_id == self.frame_id

    def hears(self, status_name: str) -> bool:
        """Report whether a status message of this name is this input's."""
        return self.kind != 'status' or not self.status_name or status_name == self.status_name


def finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def ground_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Return the metres between two (latitude, longitude) points, in degrees: short spans."""
    lat = math.radians((a[0] + b[0]) / 2.0)
    dy = math.radians(b[0] - a[0]) * EARTH_RADIUS
    dx = math.radians(b[1] - a[1]) * EARTH_RADIUS * math.cos(lat)
    return math.hypot(dx, dy)


class Watch:
    """One input's state, from what arrived."""

    def __init__(self, rule: Rule):
        self.rule = rule
        self.heard = None          # when anything last arrived
        self.good = None           # when a good message last arrived
        self.bad = None            # what was wrong with the newest message, or None
        self.digest = None         # the newest values, to spot a frozen sensor
        self.same_since = None     # when the values stopped changing
        self.fix = None            # the newest good GPS fix: (stamp, latitude, longitude)
        self.jumped = None         # the jump that latched, as text, until clear()
        self.off_since = None      # actuator: when it went more than follow_tolerance off
        self.stuck = None          # actuator: what latched STUCK, as text, until clear()
        self.over_since = None     # speed: when it went over (and not slowing), or None
        self.over_words = ''       # speed: what is over, as text, while over_since is set
        self.speeds = []           # speed: (now, |measured|) over the last SLOWING_WINDOW
        self.overspeed = None      # speed: what latched OVERSPEED, as text, until clear()
        self.said = None           # status: (level, words) of a newest message at min_level+
        # Latch rules (rule.latch), kept by update().
        self.first_good = None     # the first update() it was OK at: the grace starts
        self.was_ok = False        # OK at the last update()
        self.forgiven = False      # the one dropout of its start-up is spent
        self.lost = None           # (what it went, when): latched until clear()
        # The wheels against the IMU (odom only): fed by arrived() and imu().
        self.cross = WheelCheck(rule.cross_check) if rule.cross_check is not None else None
        # Its traffic, for people (diagnostics: Lichtblick's "Sensors health" layout).
        self.count = 0             # messages since the window began
        self.window = None         # when the window began: the first message, then traffic()
        self.gap = 0.0             # the longest gap between two messages in the window
        self.worst_gap = 0.0       # the longest since the first message

    def arrived(self, now: float, values: Sequence[float] = (), digest=None,
                fix: Optional[Tuple[float, float]] = None, has_fix: bool = True,
                speed: Optional[float] = None, stamp: Optional[float] = None,
                words: str = ''):
        """
        Note a message: its numbers (NaN check), a digest of it (frozen check), a GPS fix.

        speed is the car's newest wheel speed (m/s), for a camera's or cloud's frozen check. stamp
        is the fix's own time (s): fixes can arrive two at once, so their arrival times
        say nothing about how far apart they were taken. A status: values (its level,)
        and words, its message.
        """
        rule = self.rule
        gap_before = None if self.heard is None else now - self.heard
        if self.heard is None:
            self.window = now
        else:
            self.gap = max(self.gap, now - self.heard)
            self.worst_gap = max(self.worst_gap, self.gap)
        self.count += 1
        self.heard = now
        if rule.kind == 'gps' and (not has_fix or fix is None or not finite(fix)):
            self.bad = 'NO FIX'
            return
        if rule.kind in ('actuator', 'speed'):
            if rule.kind == 'speed' and gap_before is not None and gap_before > rule.max_age:
                self.over_since, self.speeds = None, []     # a stale command: start afresh
            self.bad = None
            self.good = now
            if rule.kind == 'actuator':
                self.follow(now, values)
            else:
                self.over(now, values)
            return
        if not finite(values):
            self.bad = NAN
            return
        self.bad = None
        self.good = now
        if rule.kind == 'status':
            level = int(values[0]) if values else 0
            self.said = (level, words) if level >= rule.min_level else None
        if self.cross is not None and len(values) >= 3:
            self.cross.wheels(now, values[0], values[2])    # forward speed, turn rate
        if rule.frozen_after > 0.0 and digest is not None:
            moving = rule.kind not in MOVING_KINDS or (speed is not None and abs(speed) > MOVING)
            if digest != self.digest or not moving:
                self.same_since = now
            self.digest = digest
        if rule.kind == 'gps':
            taken = now if stamp is None else stamp
            if self.fix is not None and rule.max_jump_speed > 0.0 and self.jumped is None:
                dt = taken - self.fix[0]
                metres = ground_distance(self.fix[1:], fix)
                if dt > 0.0 and metres > rule.max_jump_speed * dt + rule.jump_margin:
                    self.jumped = f'{metres:.1f} m in {dt:.2f} s'
            if self.fix is None or taken > self.fix[0]:     # not an old fix, late
                self.fix = (taken, fix[0], fix[1])

    def follow(self, now: float, values: Sequence[float]):
        """Actuator: note (command, measured); latch STUCK once off too long."""
        rule = self.rule
        if rule.follow_tolerance <= 0.0 or len(values) < 2 or not finite(values[:2]):
            self.off_since = None
            return
        command, measured = values[0], values[1]
        if abs(command - measured) <= rule.follow_tolerance:
            self.off_since = None
            return
        if self.off_since is None:
            self.off_since = now
        if self.stuck is None and now - self.off_since >= rule.follow_persist:
            self.stuck = (f'asked {command:+.2f}, did {measured:+.2f} for '
                          f'{now - self.off_since:.1f} s')

    def over(self, now: float, values: Sequence[float]):
        """
        Speed: note (command, measured[, age]) in m/s; latch OVERSPEED once over too long.

        age: the measured speed's age (s). Older than max_age, it is no check: the wheels
        went quiet (odometry's own fault), and their last speed says nothing now.
        """
        rule = self.rule
        stale = len(values) >= 3 and not values[2] <= rule.max_age
        if rule.over_margin <= 0.0 or len(values) < 2 or not finite(values[:2]) or stale:
            self.over_since, self.speeds = None, []
            return
        command, measured = abs(values[0]), abs(values[1])
        self.speeds = [(t, v) for t, v in self.speeds if now - t <= SLOWING_WINDOW]
        oldest = self.speeds[0] if self.speeds else None
        self.speeds.append((now, measured))
        # Only a full window says whether it is slowing: a shorter one is noise.
        slowing = (oldest is not None and now - oldest[0] >= 0.8 * SLOWING_WINDOW
                   and (oldest[1] - measured) / (now - oldest[0]) >= rule.over_slowing)
        if measured <= command + rule.over_margin or slowing:
            self.over_since = None
            return
        if self.over_since is None:
            self.over_since = now
        self.over_words = (f'asked {command:.2f} m/s, did {measured:.2f} for '
                           f'{now - self.over_since:.1f} s')
        if self.overspeed is None and now - self.over_since >= rule.over_persist:
            self.overspeed = self.over_words

    def state(self, now: float) -> Tuple[str, str]:
        """Return (OK or what is wrong, a few words for a person)."""
        if self.lost is not None:
            went, since = self.lost
            current, _ = self.current(now)
            return LOST, (f'went {went} {now - since:.0f} s ago, {current} now: clear it '
                          'once it runs right')
        return self.current(now)

    def current(self, now: float) -> Tuple[str, str]:
        """Return (state, words) as the input is now, without a latched LOST."""
        rule = self.rule
        if self.jumped is not None:
            return JUMP, f'jumped {self.jumped}: clear it once the position is right'
        if self.stuck is not None:
            return STUCK, f'{self.stuck}: clear it once the actuator follows again'
        if self.overspeed is not None:
            return OVERSPEED, f'{self.overspeed}: clear it once the throttle obeys again'
        why = self.cross.update(now) if self.cross is not None else None
        if why is not None:
            return DISAGREE, why + ': clear it once the wheels are right'
        if self.heard is None:
            return (OK, 'quiet: nothing yet') if rule.quiet_ok else (NEVER, 'nothing yet')
        if self.bad == NAN:
            return NAN, 'NaN or inf in the newest message'
        age = now - self.good if self.good is not None else math.inf
        if age > rule.max_age and rule.quiet_ok:
            return OK, f'quiet for {age:.1f} s'
        if age > rule.max_age:
            why = 'no fix' if self.bad == 'NO FIX' else 'nothing'
            since = f'for {age:.1f} s' if math.isfinite(age) else 'yet'
            return STALE, f'{why} {since} (more than {rule.max_age:g} s)'
        if (rule.frozen_after > 0.0 and self.same_since is not None
                and now - self.same_since > rule.frozen_after):
            return FROZEN, f'the same for {now - self.same_since:.1f} s'
        if self.said is not None:
            level, words = self.said
            return FAULT, f'says {LEVELS.get(level, LEVELS[3])}: {words}'
        if self.over_since is not None and self.good - self.over_since >= rule.over_warn:
            return OVER, self.over_words
        return OK, f'{age:.1f} s old'

    def traffic(self, now: float) -> Tuple[float, float, float]:
        """
        Return (messages a second, longest gap, longest gap ever) in s, and start a new window.

        The window runs from the last call (or the first message) to now. Its longest gap
        counts the silence since the newest message too, so a sensor that just stopped
        shows at once. All zero before the first message.
        """
        if self.heard is None:
            return 0.0, 0.0, 0.0
        span = now - self.window
        rate = self.count / span if span > 0.0 else 0.0
        gap = max(self.gap, now - self.heard)
        self.worst_gap = max(self.worst_gap, gap)
        out = (rate, gap, self.worst_gap)
        self.count, self.window, self.gap = 0, now, 0.0
        return out

    def update(self, now: float):
        """Latch rules: latch a dropout, unless it is the one of its start-up. Once a tick."""
        if not self.rule.latch or self.lost is not None:
            return
        state, _ = self.current(now)
        if state == OK:
            self.was_ok = True
            if self.first_good is None:
                self.first_good = now
        elif self.was_ok:
            self.was_ok = False
            if not self.forgiven and now - self.first_good <= self.rule.latch_grace:
                self.forgiven = True        # starting up: holds only while it lasts
            else:
                self.lost = (state, now)

    def clear(self):
        """Forget what latched; a latch rule starts afresh, with its grace again."""
        self.jumped = None
        self.stuck, self.off_since = None, None
        self.overspeed, self.over_since, self.speeds = None, None, []
        if self.cross is not None:
            self.cross.clear()
        self.lost, self.first_good, self.was_ok, self.forgiven = None, None, False, False


class Watchdog:
    """Every input's Watch, and the verdict: whether the automatic drivers may drive."""

    def __init__(self, rules: Sequence[Rule], stall: float = 0.5, settle: float = 3.0):
        """
        stall: steady s between two update()s that mean the watchdog itself was stopped.

        settle: steady s after such a stall in which nothing latches.
        """
        self.watches = {r.name: Watch(r) for r in rules}
        self.stall, self.settle = stall, settle
        self.ticked = None          # the last update()'s steady time
        self.quiet_until = None     # steady time: no latching before it

    def update(self, now: float, steady: float):
        """
        Note the time, once a tick: where a rule says latch, a dropout latches here.

        now is the node's clock (Gazebo's, in a simulator); steady a steady clock's
        seconds (time.monotonic()). After more than `stall` steady s since the last tick,
        the watchdog itself was stopped (a busy computer, SIGSTOP): the inputs look silent
        only because it didn't listen. Nothing latches for `settle` s, while their queued
        messages arrive. Only a steady clock sees the stall: a simulator's clock catches
        up in small steps, from its queued messages too.
        """
        if self.ticked is not None and steady - self.ticked > self.stall:
            self.quiet_until = steady + self.settle
        self.ticked = steady
        if self.quiet_until is not None and steady < self.quiet_until:
            return
        for watch in self.watches.values():
            watch.update(now)

    def imu(self, now: float, forward: float, yaw_rate: float):
        """Note an IMU sample for every cross check: forward m/s^2, yaw rate rad/s."""
        for watch in self.watches.values():
            if watch.cross is not None:
                watch.cross.imu(now, forward, yaw_rate)

    def gps(self, stamp: float, fix: Tuple[float, float]):
        """Note a good GPS fix for every cross check: its stamp (s), (latitude, longitude)."""
        for watch in self.watches.values():
            if watch.cross is not None:
                watch.cross.gps(stamp, *fix)

    def cross_checked(self) -> bool:
        return any(w.cross is not None for w in self.watches.values())

    def states(self, now: float):
        """Return [(name, state, detail, stops)] for every input, in the rules' order."""
        out = []
        for name, watch in self.watches.items():
            state, detail = watch.state(now)
            out.append((name, state, detail,
                        watch.rule.stop and state != OK and state not in WARN_ONLY))
        return out

    def locked(self, now: float) -> bool:
        """Return True while any input whose rule says stop isn't OK."""
        return any(stops for _, _, _, stops in self.states(now))

    def summary(self, now: float) -> str:
        """Return one line for the HUD and the log: OK, or what is wrong and what it does."""
        wrong = [(n, s, d, stops) for n, s, d, stops in self.states(now) if s != OK]
        if not wrong:
            return 'OK'
        wrong.sort(key=lambda w: not w[3])      # what stops the drivers first: the HUD's reason
        verdict = 'STOP' if any(stops for *_, stops in wrong) else 'WARN'
        return verdict + ': ' + '; '.join(f'{n} {s} ({d})' for n, s, d, _ in wrong)

    def clear(self):
        """Forget latched jumps, disagreements, stuck actuators, overspeed, lost inputs."""
        for watch in self.watches.values():
            watch.clear()
