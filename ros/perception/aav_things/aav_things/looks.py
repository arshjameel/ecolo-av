"""
How a thing looks on a screen: its words and its colour. No ROS here.

TODO 192 step 3 (Ben, 15 Nov: "object deteciton and display like a stop sign"). The
markers (RViz, Foxglove) and the Rust screens (hud_core::things) use the same colours:

| Kind | Colour | Why |
|---|---|---|
| person | yellow | the one to watch: high-visibility vest |
| car, truck, bus, motorcycle | blue | another vehicle |
| stop sign | red | the sign's own red |
| any other name (bicycle...) | green | named, not one of the above |
| unknown | orange | no camera has named it (the colour before names) |
| coasting (unseen now) | grey | moved on its speed, not seen in the last cloud |
"""

import math
from typing import Tuple

UNKNOWN = 'unknown'
VEHICLES = ('car', 'truck', 'bus', 'motorcycle')

RGB = Tuple[float, float, float]
PERSON: RGB = (1.0, 0.85, 0.1)
VEHICLE: RGB = (0.25, 0.55, 1.0)
STOP_SIGN: RGB = (0.9, 0.1, 0.1)
OTHER: RGB = (0.3, 0.85, 0.4)
UNNAMED: RGB = (1.0, 0.6, 0.0)
COASTING: RGB = (0.6, 0.6, 0.6)


def colour(kind: str, coasting: bool = False) -> RGB:
    """Return a thing's colour (r, g, b, 0..1) by its kind; grey while coasting."""
    if coasting:
        return COASTING
    name = (kind or UNKNOWN).strip().lower()
    if name == 'person':
        return PERSON
    if name in VEHICLES:
        return VEHICLE
    if name == 'stop sign':
        return STOP_SIGN
    if name == UNKNOWN:
        return UNNAMED
    return OTHER


def caption(thing_id: int, kind: str, vx: float, vy: float, radar: bool) -> str:
    """
    Return a thing's words: its name and id, then its speed: 'person 3 1.2 m/s'.

    An unknown thing shows only its id ('3 0.0 m/s'), as before names; R: the radar
    gave its speed.
    """
    name = (kind or UNKNOWN).strip()
    who = str(thing_id) if name.lower() == UNKNOWN else f'{name} {thing_id}'
    return f'{who} {math.hypot(vx, vy):.1f} m/s' + (' R' if radar else '')
