"""
The trial trackers: one small class each, no ROS (CLAUDE.md: keep the maths out of ROS).

```python
tracker = make('lqr', limits, model)   # model: aav_ego VehicleModel (lqr, mpc)
tracker.reset(path)                    # a new path, in odom (aav_track.path.Path)
cmd = tracker.step(state, dt)          # Command(steer, speed_cap=None, debug={})
```

| Name | Module | Steers by |
|---|---|---|
| `pure_pursuit` | `pure_pursuit.py` | an arc to a carrot ahead on the path (the control) |
| `stanley` | `stanley.py` | heading error + atan(k e / v), at the front axle |
| `lqr` | `lqr.py` | -K x on [e_y, e_theta, delta], gains scheduled by speed, + feed-forward |
| `mpc` | `mpc.py` | a 1.5 s plan of steering commands, within the steering's limits |
| `mpc_full` | `mpc_full.py` | a 9 m plan of speed and steering together (its own speed) |

Every tracker keeps its command within full lock and the steering rate (`Limits`); the
node then turns it into (v, w = v tan(steer) / L) with the one speed policy (`speed.py`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Optional

from aav_track.path import Foot, Path


@dataclass(frozen=True)
class State:
    """The car now: base_link in odom, its speed, yaw rate, and the last steering asked."""

    x: float
    y: float
    yaw: float
    v: float                 # m/s, + forward
    yaw_rate: float = 0.0    # rad/s
    steer: float = 0.0       # rad: the last command this tracker gave (+ left)


@dataclass(frozen=True)
class Limits:
    wheelbase: float         # m (vehicle.yaml)
    max_steer: float         # rad, full lock either side (vehicle.yaml)
    max_steer_rate: float    # rad/s (the tighter of ego's ramp and the motor)

    def __post_init__(self):
        if self.wheelbase <= 0 or not 0 < self.max_steer < math.pi / 2 or \
                self.max_steer_rate <= 0:
            raise ValueError('wheelbase and steering rate must be above 0, '
                             'full lock in (0, pi/2)')


@dataclass
class Command:
    steer: float                         # rad, + left
    speed_cap: Optional[float] = None    # m/s: a tracker that wants it slower
    # m/s: a tracker that plans its own speed (mpc_full); the shared policy is then skipped
    speed: Optional[float] = None
    debug: Dict[str, float] = field(default_factory=dict)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Tracker:
    """The interface: reset(path), step(state, dt)."""

    name = ''

    def __init__(self, limits: Limits):
        self.limits = limits
        self.path: Optional[Path] = None
        self.hint = 0

    def reset(self, path: Path):
        """Take a new path (in odom). The steering carries on from the last command."""
        self.path = path
        self.hint = 0

    def foot(self, x: float, y: float) -> Foot:
        """Return where (x, y) is on the path, searching on from the last foot."""
        if self.path is None:
            raise RuntimeError('step() before reset(path)')
        f = self.path.project((x, y), self.hint)
        self.hint = f.index
        return f

    def limited(self, steer: float, previous: float, dt: float) -> float:
        """Return the steering within full lock, and within the rate of `previous`."""
        lim = self.limits
        step = lim.max_steer_rate * max(dt, 0.0)
        steer = clamp(steer, previous - step, previous + step)
        return clamp(steer, -lim.max_steer, lim.max_steer)

    def step(self, state: State, dt: float) -> Command:   # pragma: no cover - interface
        raise NotImplementedError


class SteerLag:
    """The wheels' angle, estimated: the commands through one lag tau (the model's tau_s)."""

    def __init__(self, tau: float):
        self.tau = tau
        self.angle = 0.0

    def update(self, command: float, dt: float) -> float:
        a = 1.0 - math.exp(-max(dt, 0.0) / self.tau) if self.tau > 0 else 1.0
        self.angle += a * (command - self.angle)
        return self.angle


# The trackers that need the vehicle model.
MODEL_TRACKERS = ('lqr', 'mpc', 'mpc_full')


def make(name: str, limits: Limits, model=None, **params) -> Tracker:
    """Return the tracker called `name`; MODEL_TRACKERS need `model` (aav_ego VehicleModel)."""
    if name == 'pure_pursuit':
        from aav_track.trackers.pure_pursuit import PurePursuit
        return PurePursuit(limits, **params)
    if name == 'stanley':
        from aav_track.trackers.stanley import Stanley
        return Stanley(limits, **params)
    if name in MODEL_TRACKERS:
        if model is None:
            raise ValueError(f'{name} needs the vehicle model')
        if name == 'lqr':
            from aav_track.trackers.lqr import LQR
            return LQR(limits, model, **params)
        if name == 'mpc_full':
            from aav_track.trackers.mpc_full import MPCFull
            return MPCFull(limits, model, **params)
        from aav_track.trackers.mpc import MPC
        return MPC(limits, model, **params)
    raise ValueError(f'no tracker called {name!r}: pure_pursuit, stanley, lqr, mpc or '
                     'mpc_full')
