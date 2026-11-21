"""
Error bars on every fitted number: mean ± spread over the runs, and a trust flag. No ROS.

TODO 248 (and TODO 104's "knows itself"): a number for vehicle.yaml comes with how sure
we are of it. Each run (each pass down the hallway) gives its own fit; their spread is the
error bar:

| Field | Is |
|---|---|
| mean, sd | the runs' mean and sample standard deviation |
| se | sd / sqrt(n): how well the mean is known |
| lo, hi | a bootstrap 90 % interval of the mean (resampled runs, seeded) |
| resolution | the shortest time the data can tell (analysis.time_resolution), for times |
| floor | the least error bar, a share of the mean: what the spread can't show (the wheel's size) |
| bar | max(sd, resolution, floor x mean): what covers() judges by |
| trusted, why | False when the data can't tell: one run, a fit under its resolution, |
|   | or a spread over SPREAD_LIMIT of the mean |

One run has no spread: it is never trusted on its own (run it again).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Callable, List, Optional, Sequence

SPREAD_LIMIT = 0.25   # sd over this share of |mean| (and over the resolution): not trusted
BOOTSTRAP = 400       # resamples
NAN = float('nan')


def finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Value:
    """One run's fitted number: the value, what the data could resolve, why not."""

    value: float
    resolution: float = 0.0     # s for times; 0 for gains and drags
    problem: str = ''           # why this run's value can't be trusted ('' if it can)


@dataclass(frozen=True)
class Estimate:
    """A number from several runs, with its error bar."""

    mean: float
    sd: float
    n: int
    lo: float
    hi: float
    resolution: float
    trusted: bool
    why: str = ''
    floor: float = 0.0     # the least error bar, a share of |mean|: what no spread shows

    @property
    def se(self) -> float:
        return self.sd / math.sqrt(self.n) if self.n > 0 else NAN

    @property
    def bar(self) -> float:
        """Return the error bar used to judge: the spread, at least the resolution."""
        return max(self.sd if finite(self.sd) else 0.0, self.resolution,
                   self.floor * abs(self.mean) if finite(self.mean) else 0.0)

    def covers(self, true: float, k: float = 2.0) -> bool:
        """Whether `true` lies within k error bars of the mean (the recovery check)."""
        return finite(self.mean, true) and abs(self.mean - true) <= k * self.bar + 1e-9

    def text(self, unit: str = '', digits: int = 3) -> str:
        """Return '0.312 ± 0.021 s (n 6)'."""
        if not finite(self.mean):
            return '-'
        u = f' {unit}' if unit else ''
        return f'{self.mean:.{digits}f} ± {self.sd:.{digits}f}{u} (n {self.n})'


def bootstrap(values: Sequence[float], stat: Callable[[Sequence[float]], float] = None,
              resamples: int = BOOTSTRAP, seed: int = 1, level: float = 0.9):
    """Return (lo, hi): the `level` interval of `stat` (the mean) over resampled runs."""
    xs = [v for v in values if finite(v)]
    if not xs:
        return NAN, NAN
    stat = stat or (lambda s: sum(s) / len(s))
    if len(xs) == 1:
        return xs[0], xs[0]
    rng = random.Random(seed)
    stats = sorted(stat([rng.choice(xs) for _ in xs]) for _ in range(resamples))
    cut = (1.0 - level) / 2.0
    return (stats[int(math.floor(cut * (resamples - 1)))],
            stats[int(math.ceil((1.0 - cut) * (resamples - 1)))])


def estimate(values: Sequence[Value], spread_limit: float = SPREAD_LIMIT,
             floor: float = 0.0) -> Optional[Estimate]:
    """
    Return the runs' estimate, with its trust flag; None with no finite value.

    floor: the least error bar as a share of the mean, for what the runs' spread can't
    show: a gain read through the wheel speed is only as good as the wheel's size
    (hallway.SCALE).
    """
    vs = [v for v in values if finite(v.value)]
    if not vs:
        return None
    xs = [v.value for v in vs]
    n = len(xs)
    mean = sum(xs) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / (n - 1)) if n > 1 else 0.0
    lo, hi = bootstrap(xs)
    res = max((v.resolution for v in vs if finite(v.resolution)), default=0.0)
    why: List[str] = []
    if n < 2:
        why.append('one run: no spread (run it again)')
    problems = sorted({v.problem for v in vs if v.problem})
    if problems:
        why.append(f'{sum(1 for v in vs if v.problem)} of {n} runs: ' + '; '.join(problems))
    if n > 1 and sd > max(spread_limit * abs(mean), res):
        why.append(f'the runs disagree: sd {sd:.3g} is over {spread_limit:.0%} of the mean '
                   f'{mean:.3g}' + (f' and the resolution {res:.3g} s' if res else ''))
    return Estimate(mean, sd, n, lo, hi, res, not why, '; '.join(why), floor)
