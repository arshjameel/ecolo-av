"""
When to stop for a stop sign, from the detector's boxes. No ROS imports.

A sign counts once it is close enough and its score good enough. How close comes from
its box's height and the camera's lens, a pinhole camera:

    distance = focal_px x sign_height_m / box_height_px

    a 0.75 m sign, Gazebo's and the bench's lens (343 px at 640x360): 55 px tall at 4.7 m
    the same sign, a lens with 950 px (1280x720, say):                152 px at 4.7 m

So the rule is a distance (stop_distance_m), and each camera's focal length comes from
its CameraInfo (its calibration), and carries over to any camera. Then the lock goes on
for `hold_s` seconds: twist_mux holds the autonomous drivers, and ego stops the car.
After that the lock goes off, and signs are ignored for `ignore_s` seconds, so the car
can drive past the one it stopped for.

    CLEAR --a close sign--> STOPPING --hold_s--> PASSING --ignore_s--> CLEAR
              (lock on)                  (lock off, signs ignored)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from aav_detector.boxes import normal_label

CLEAR, STOPPING, PASSING = 'clear', 'stopping', 'passing'

# A stop sign's height, flat to flat: Ontario's and the US's standard road sign (750 mm),
# and Gazebo's (aav_gazebo models/stop_sign).
SIGN_HEIGHT_M = 0.75
# Gazebo's front camera and the bench camera: 640x360, focal length 343 px (one lens,
# aav.sensors.gazebo.xacro). For a camera with no CameraInfo yet.
GAZEBO_FOCAL_PX = 343.0


def sign_distance_m(height_px: float, focal_px: float,
                    sign_height_m: float = SIGN_HEIGHT_M) -> float:
    """Return how far a sign is (m, along the camera's axis) from its box's height (px)."""
    return focal_px * sign_height_m / height_px if height_px > 0 else float('inf')


@dataclass
class StopRule:
    """Thresholds, from the watcher's parameters."""

    # 4.7 m: a sign 2 m right of the lane is still in Gazebo's picture then, with the car
    # 0.7 m off its line and turned 5 degrees, as Nav2 on GPS leaves it. At 3.5 m (72 px
    # with Gazebo's lens) it had often left the picture. 55 px with that lens.
    stop_distance_m: float = 4.7
    focal_px: float = GAZEBO_FOCAL_PX     # a camera's lens until its CameraInfo comes
    sign_height_m: float = SIGN_HEIGHT_M
    min_score: float = 0.5
    hold_s: float = 4.0           # about 1 s to stop, then a 3 s wait
    ignore_s: float = 10.0

    def min_height_px(self, focal_px: Optional[float] = None) -> float:
        """Return the box height (px) of a sign at stop_distance_m, with this lens."""
        return (focal_px or self.focal_px) * self.sign_height_m / self.stop_distance_m


class StopSignWatcher:

    def __init__(self, rule: StopRule):
        self.rule = rule
        self.state = CLEAR
        self.since: Optional[float] = None

    def close_sign(self, boxes: Iterable[Tuple[str, float, float]],
                   focal_px: Optional[float] = None) -> bool:
        """
        Report whether any (label, score, height_px) box is a stop sign close enough.

        focal_px: the lens of the camera that saw them (its CameraInfo's fy, in the
        boxes' picture); None: the rule's focal_px.
        """
        least = self.rule.min_height_px(focal_px)
        return any(normal_label(label) == 'stop sign' and score >= self.rule.min_score
                   and height >= least for label, score, height in boxes)

    def update(self, boxes: Iterable[Tuple[str, float, float]], now: float,
               focal_px: Optional[float] = None) -> bool:
        """Take the newest boxes, and their camera's focal_px; return True while stopped."""
        if self.state == CLEAR and self.close_sign(boxes, focal_px):
            self.state, self.since = STOPPING, now
        elif self.state == STOPPING and now - self.since >= self.rule.hold_s:
            self.state, self.since = PASSING, now
        elif self.state == PASSING and now - self.since >= self.rule.ignore_s:
            self.state, self.since = CLEAR, None
        return self.state == STOPPING

    def tick(self, now: float) -> bool:
        """Advance the timers without new boxes (the detector can go quiet)."""
        return self.update((), now)


def sign_point(centre_px: float, height_px: float, focal_px: float,
               fx: Optional[float] = None, cx: Optional[float] = None,
               optical: bool = False,
               sign_height_m: float = SIGN_HEIGHT_M) -> Tuple[float, float, float]:
    """
    Return where a sign's box puts the sign, (x, y, z) in its camera's frame (m).

    The range along the camera's axis is sign_distance_m; the bearing comes from the box's
    centre column (centre_px) and the lens (fx, cx: the CameraInfo's K[0] and K[2]); with
    no lens yet, straight ahead. optical: the frame is z forward, x right (REP-103's
    _optical); else x forward, y left, as camera_left_link. The height isn't known: 0.
    """
    ahead = sign_distance_m(height_px, focal_px, sign_height_m)
    right = ahead * (centre_px - cx) / fx if fx and cx is not None else 0.0
    return (right, 0.0, ahead) if optical else (ahead, -right, 0.0)


def biggest_sign(boxes: Iterable[Tuple[str, float, float, float]],
                 min_score: float) -> Optional[Tuple[float, float]]:
    """Return (centre_px, height_px) of the tallest stop-sign box, or None."""
    best = None
    for label, score, height, centre in boxes:
        if normal_label(label) == 'stop sign' and score >= min_score and height > 0 and (
                best is None or height > best[1]):
            best = (centre, height)
    return best


def lidar_sign(camera_xy: Tuple[float, float],
               things: Iterable[Tuple[str, float, float]],
               gate_m: float = 3.0) -> Optional[Tuple[float, float]]:
    """
    Return the lidar's place for the sign the camera sees, or None (TODO 192 step 2).

    camera_xy: where the box's height puts the sign; things: (kind, x, y) from the things
    node with names on, in the same frame. The thing named stop sign nearest the camera's
    guess, within gate_m, wins: the lidar measures its distance; the pixel height only
    guesses it (a sign 10 % smaller than 0.75 m reads 10 % nearer).
    """
    best, best_d = None, gate_m
    for kind, x, y in things:
        if normal_label(kind) != 'stop sign':
            continue
        d = ((x - camera_xy[0]) ** 2 + (y - camera_xy[1]) ** 2) ** 0.5
        if d <= best_d:
            best, best_d = (x, y), d
    return best
