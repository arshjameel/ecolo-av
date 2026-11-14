"""
The road ahead as a ROS message, and back: aav_interfaces/RoadAhead <-> road_ahead.Strip.

The maths is road_ahead.py (no ROS). A reader does:

    strip = from_msg(msg)
    v = limit_at(strip, x, y, curve, base)     # x, y in msg.header.frame_id (map)
"""

from aav_description.road_ahead import StopOnLine, Strip
from aav_interfaces.msg import RoadAhead, StopPoint


def to_msg(strip: Strip, header) -> RoadAhead:
    """Return the message for `strip`, stamped with `header` (frame map)."""
    return RoadAhead(
        header=header, spacing=float(strip.spacing),
        x=[float(v) for v in strip.x], y=[float(v) for v in strip.y],
        heading=[float(v) for v in strip.heading],
        speed_limit=[float(v) for v in strip.speed_limit],
        left_hard=[float(v) for v in strip.left_hard],
        right_hard=[float(v) for v in strip.right_hard],
        left_soft=[float(v) for v in strip.left_soft],
        right_soft=[float(v) for v in strip.right_soft],
        stops=[StopPoint(along=float(s.along), reason=s.reason, kind=int(s.kind))
               for s in strip.stops],
        end=float(strip.end), state=strip.state, reason=strip.reason)


def from_msg(msg: RoadAhead) -> Strip:
    """Return the Strip a RoadAhead message carries."""
    return Strip(
        spacing=msg.spacing, x=tuple(msg.x), y=tuple(msg.y), heading=tuple(msg.heading),
        speed_limit=tuple(msg.speed_limit),
        stops=tuple(StopOnLine(s.along, s.reason, s.kind) for s in msg.stops),
        end=msg.end, state=msg.state, reason=msg.reason,
        left_hard=tuple(msg.left_hard), right_hard=tuple(msg.right_hard),
        left_soft=tuple(msg.left_soft), right_soft=tuple(msg.right_soft))
