"""
The numbers behind `pixi run barrier-watch` (tools/barrier_watch.py). No ROS here.

Outside the capstone's scope (Ben's own research, toward his MASc thesis): the barriers in
the MPC (docs/barriers.md). One drive past a round object gives a list of samples; this
turns them into one line: touched, stopped or went round; the least gap; who acted first.

Everything is in the run's own frame: x along the car's heading at the start, y to its
left, the origin where base_link stood (REP-103: metres, seconds, radians). The poses are
the simulator's truth, and the ghost stands where its map coordinates are in the world, as
a real thing would: a car that is lost by half a metre can touch what it believes it
missed. How far navigation's belief (the global EKF) was from the truth is kept beside
each sample (`off`), so a run where the car was lost shows.
"""

from dataclasses import dataclass
import math
import statistics

MODES = ('plain', 'step', 'barrier')
# The launch switches of each mode (sim.launch.py), after tracker:=mpc_lag_rs ghost:=true.
#   plain    the follower alone
#   step     the barrier's rule with gamma 1: only "never inside", a plain distance limit
#   barrier  the rule with the tracker's own gamma (0.05)
MODE_SWITCHES = {
    'plain': (),
    'step': ('mpc_barriers:=true', 'mpc_barrier_gamma:=1'),
    'barrier': ('mpc_barriers:=true',),
}
SWERVED = 1.0       # m off the start's line, before the object: the car steered round
STILL = 0.02        # m/s: standing
MOVING = 0.3        # m/s: the drive has begun


@dataclass(frozen=True)
class Body:
    """The car's body in base_link: the bumpers' x (m) and half its width (m)."""

    rear: float
    front: float
    half_width: float

    @classmethod
    def from_vehicle(cls, car):
        """Return the body of vehicle.yaml (`car`: vehicle.load())."""
        body = car['vehicle']['body']
        return cls(rear=-float(body['rear_overhang']),
                   front=float(body['length']) - float(body['rear_overhang']),
                   half_width=float(body['width']) / 2.0)


@dataclass(frozen=True)
class Sample:
    """One moment of the drive, in the run's frame."""

    t: float                # s, the simulator's clock
    x: float                # m: base_link
    y: float
    yaw: float              # rad
    v: float                # m/s, forward
    barrier_stop: bool = False      # the tracker's stop rule asks a stop (its status)
    zone_stop: bool = False         # the stop zone is stopping the car
    off: float = 0.0                # m between the believed place and the simulator's truth


def to_run_frame(x, y, yaw, start):
    """Return a world pose in the run's frame; start: the car's (x, y, yaw) at the start."""
    x0, y0, h = start
    c, s = math.cos(h), math.sin(h)
    dx, dy = x - x0, y - y0
    return dx * c + dy * s, -dx * s + dy * c, yaw - h


def from_run_frame(x, y, start):
    """Return a point of the run's frame in the world's; the opposite of to_run_frame."""
    x0, y0, h = start
    c, s = math.cos(h), math.sin(h)
    return x0 + x * c - y * s, y0 + x * s + y * c


def object_centre(body, ahead, offset, radius):
    """
    Return the object's centre in the run's frame.

    Its near edge is `ahead` m past the front bumper (as `pixi run ghost ahead`), and its
    centre `offset` m left of the car's line (negative: right).
    """
    return body.front + ahead + radius, offset


def gap(body, sample, centre, radius):
    """
    Return the m between the car's body and a round object; negative: that far inside.

    The body is a rectangle; the distance is from the circle's centre to it, less the radius.
    """
    dx, dy = centre[0] - sample.x, centre[1] - sample.y
    c, s = math.cos(sample.yaw), math.sin(sample.yaw)
    bx, by = dx * c + dy * s, -dx * s + dy * c      # the centre, in base_link
    out_x = max(body.rear - bx, 0.0, bx - body.front)
    out_y = max(abs(by) - body.half_width, 0.0)
    if out_x > 0.0 or out_y > 0.0:
        return math.hypot(out_x, out_y) - radius
    # The centre is inside the body: how deep, to the nearest side.
    return -min(bx - body.rear, body.front - bx, body.half_width - abs(by)) - radius


@dataclass(frozen=True)
class Verdict:
    """What one drive did."""

    outcome: str            # touched, went round, stopped, or still moving
    least_gap: float        # m (nan with no samples)
    first: str              # who acted first, in words
    passed: bool            # the rear bumper went past the object
    top_speed: float        # m/s
    stood_x: float          # m: where base_link stood at the end (the run's frame)
    widest: float           # m: the furthest the car went off its line
    zone_stopped: bool      # the stop zone stopped the car at some point
    rule_stopped: bool      # the tracker's stop rule asked a stop at some point
    took: float = 0.0       # s from the first move to the last sample
    backed: float = 0.0     # m driven backwards
    lost: float = 0.0       # m: the believed place from the truth, the drive's median

    def line(self):
        """Return the one line the watch command prints."""
        asked = ' and '.join(n for n, on in (('the stop rule', self.rule_stopped),
                                             ('the stop zone', self.zone_stopped)) if on)
        return (f'{self.outcome.upper()}: least gap {self.least_gap:.2f} m; first to act: '
                f'{self.first}; stops asked by: {asked or "nobody"}; top speed '
                f'{self.top_speed:.2f} m/s; widest off the line {self.widest:.2f} m; '
                f'ended at x = {self.stood_x:.2f} m after {self.took:.0f} s'
                + (f'; backed up {self.backed:.1f} m' if self.backed >= 0.1 else '')
                + f'; localization off by {self.lost:.2f} m (median)')


def driving(samples):
    """
    Return the samples from when the car first drove (MOVING m/s) on.

    Before that the stop zone may say stop for a moment as its sensors start: not this drive's.
    """
    start = next((i for i, s in enumerate(samples) if s.v > MOVING), len(samples))
    return samples[start:]


def first_to_act(samples, centre, radius, body):
    """
    Return who asked a stop first, in words: the tracker's stop rule or the stop zone.

    Only what happens once the car drives, and before its front is past the object,
    counts. With no stop
    asked: "the steering alone" when the car left its line by SWERVED or more, else
    "nobody". (Leaving the line is not timed against the stops: the follower sways a
    little on its own, so a small move says nothing about who acted.)
    """
    before = [s for s in driving(samples) if s.x + body.front < centre[0] + radius]
    events = []
    for name, when in (("the tracker's stop rule", lambda s: s.barrier_stop),
                       ('the stop zone', lambda s: s.zone_stop)):
        hit = next((s for s in before if when(s)), None)
        if hit is not None:
            events.append((hit.t, name, hit))
    if not events:
        swerved = any(abs(s.y) >= SWERVED for s in before)
        return 'the steering alone (no stop asked)' if swerved else 'nobody'
    _, name, s = min(events, key=lambda e: e[0])
    return f'{name}, at {s.v:.2f} m/s, {gap(body, s, centre, radius):.2f} m from it'


def verdict(samples, body, centre, radius):
    """Return the Verdict of a drive: `samples` in time order, the object where it stood."""
    if not samples:
        return Verdict('no data', math.nan, 'nobody', False, 0.0, math.nan, 0.0,
                       zone_stopped=False, rule_stopped=False)
    least = min(gap(body, s, centre, radius) for s in samples)
    last = samples[-1]
    passed = last.x + body.rear > centre[0] + radius
    if least <= 0.0:
        outcome = 'touched'
    elif passed:
        outcome = 'went round'
    elif abs(last.v) < STILL:
        outcome = 'stopped'
    else:
        outcome = 'still moving'
    return Verdict(outcome=outcome, least_gap=least,
                   first=first_to_act(samples, centre, radius, body), passed=passed,
                   top_speed=max(s.v for s in samples), stood_x=last.x,
                   widest=max(abs(s.y) for s in samples),
                   zone_stopped=any(s.zone_stop for s in driving(samples)),
                   rule_stopped=any(s.barrier_stop for s in driving(samples)),
                   took=last.t - driving(samples)[0].t if driving(samples) else 0.0,
                   backed=sum(-b.v * (b.t - a.t) for a, b in zip(samples, samples[1:])
                              if b.v < 0.0),
                   lost=statistics.median(s.off for s in (driving(samples) or samples)))


def launch_switches(mode, speed, gamma=None):
    """
    Return sim.launch.py's switches for one watch run, as name:=value strings.

    mode: one of MODES. speed: m/s (1.0 is navigation's own: no switch). gamma: another
    share for `barrier` (None: the tracker's 0.05). Every safety lock stays as the
    launch has it: no switch here takes the stop zone or the ghost's costmap layer out.
    """
    if mode not in MODES:
        raise ValueError(f'mode {mode}: pick one of {", ".join(MODES)}')
    if not 0.0 < speed <= 5.0:
        raise ValueError(f'speed {speed}: above 0, up to 5 m/s (sim_speed:=)')
    out = ['world:=road', 'rviz:=false', 'tracker:=mpc_lag_rs', 'ghost:=true',
           *MODE_SWITCHES[mode]]
    if gamma is not None:
        if mode != 'barrier':
            raise ValueError('--gamma goes with barrier (step is gamma 1, plain has none)')
        if not 0.0 < gamma <= 1.0:
            raise ValueError(f'gamma {gamma}: above 0, up to 1')
        out.append(f'mpc_barrier_gamma:={gamma:g}')
    if speed != 1.0:
        out.append(f'sim_speed:={speed:g}')
    return out
