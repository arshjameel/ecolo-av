r"""
The road ahead's speed limit on the automatic drivers' commands: the stopping filter's first form.

No ROS here. The design: docs/designs/2026-11-14_the-whole-car.md (the road ahead; R1:
the filter's first form in ego, speed only, only for the automatic drivers).

```text
 /control/cmd_vel (v, w) ──► whose is it? ──► a person, a test, a script: unchanged
                                   │
                                   └─► an automatic driver (road_ahead_drivers):
                                       v ≤ what the road ahead allows here, w scaled with it
```

| Case | Allowed forward speed |
|---|---|
| A strip, a fresh pose | `limit_at(strip, x, y, curve, base)`: falls to 0 at the strip's end |
| No strip yet | 0: no road granted |
| The pose is stale | 0: the car can't tell how much road is left |

Reversing is not limited: the strip covers only the road ahead. A person is never trimmed.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from aav_description.road_ahead import limit_at, Strip
from aav_description.stop_law import StopCurve


def allowed(strip: Optional[Strip], pose: Optional[Tuple[float, float]], pose_age: float,
            pose_timeout: float, curve: StopCurve, base: float) -> float:
    """Return the forward speed the road ahead allows now (m/s, 0 or more)."""
    if strip is None or pose is None or not (pose_age <= pose_timeout):
        return 0.0
    return max(0.0, limit_at(strip, pose[0], pose[1], curve, base))


def cap(cmd: Tuple[float, float], limit: float) -> Tuple[float, float]:
    """
    Return (v, w) with v at most `limit` going forward, keeping the curvature w / v.

    A reverse command and a stop pass unchanged.
    """
    v, w = cmd
    if v <= limit or v <= 0.0:
        return cmd
    if limit <= 0.0 or not math.isfinite(limit):
        return (0.0, 0.0) if limit <= 0.0 else cmd
    return (limit, w * limit / v)
