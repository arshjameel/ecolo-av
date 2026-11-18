"""
Names for the things: the camera's boxes matched to the lidar's things. No ROS here.

TODO 192 (Ben, 15 Nov: "How the camera perception informs the lidar distance stuff"):

    a thing (lidar: where, how big) ─► its box drawn into each camera's picture
    the detector's boxes (camera: what) ─► the one overlapping it most (IoU ≥ min_iou)
    ─► the thing takes that box's label: person, car, stop sign...; kept `hold` s

The lidar's cubes stop at a ceiling (aav_voxels' max_height, 2 m): a thing reaching it has
an unknown top. A stop sign's plate (1.6 to 2.4 m) sits mostly above it, so its lidar
thing is the pole: thin, and short of the camera's box. Such a thing is matched on its
bearing instead: the boxes overlap left to right by at least `min_side` of the narrower,
and at all top to bottom. A thing seen whole keeps the strict IoU, so a person in front
of a car still can't rename the car.

The lidar says where and how far, the camera says what. A thing no camera sees keeps
its last name for `hold` s, then goes back to unknown: never guessed. The pinhole maths is
aav_detector's (ground.py: Lens, Mount), the same the camera's road points use. On the car
the cameras' mounts need calibrating first (Q22); in the simulator they are exact.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

from aav_detector.ground import Lens, Mount

Box = Tuple[float, float, float, float]      # u0, v0, u1, v1: pixels, left top to right bottom
UNKNOWN = 'unknown'


@dataclass(frozen=True)
class Seen:
    """One detector box: its label, score and pixels."""

    label: str
    score: float
    box: Box


def to_camera(mount: Mount, p) -> Tuple[float, float, float]:
    """Return a base_link point in the camera's own frame (the mount's rotation undone)."""
    d = [p[i] - mount.position[i] for i in range(3)]
    r = mount.rotation
    return tuple(sum(r[row][col] * d[row] for row in range(3)) for col in range(3))


def project(lens: Lens, mount: Mount, p) -> Optional[Tuple[float, float]]:
    """Return the pixel (u, v) a base_link point falls on; None behind the camera."""
    x, y, z = to_camera(mount, p)
    if mount.optical:
        forward, right, down = z, x, y
    else:
        forward, right, down = x, -y, -z
    if forward <= 0.05:
        return None
    return lens.cx + lens.fx * right / forward, lens.cy + lens.fy * down / forward


def thing_box(lens: Lens, mount: Mount, x: float, y: float, length: float, width: float,
              height: float) -> Optional[Box]:
    """
    Return the picture's box round a thing standing at (x, y) in base_link, or None.

    The thing is an upright box from the road (z = 0) to `height`, `length` along x and
    `width` along y. Its corners in front of the camera are drawn; the box round them is
    clipped to the picture. None if no corner is in front, or it falls off the picture.
    """
    corners = [(x + sx * length / 2.0, y + sy * width / 2.0, z)
               for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, height)]
    pixels = [p for p in (project(lens, mount, c) for c in corners) if p is not None]
    if not pixels:
        return None
    u0 = max(0.0, min(u for u, _ in pixels))
    u1 = min(float(lens.width), max(u for u, _ in pixels))
    v0 = max(0.0, min(v for _, v in pixels))
    v1 = min(float(lens.height), max(v for _, v in pixels))
    if u1 <= u0 or v1 <= v0:
        return None
    return u0, v0, u1, v1


def iou(a: Box, b: Box) -> float:
    """Return the overlap of two boxes over their union, 0..1."""
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0.0 or h <= 0.0:
        return 0.0
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def side_overlap(a: Box, b: Box) -> float:
    """Return how much two boxes overlap left to right, over the narrower's width, 0..1."""
    w = min(a[2], b[2]) - max(a[0], b[0])
    narrower = min(a[2] - a[0], b[2] - b[0])
    return w / narrower if w > 0.0 and narrower > 0.0 else 0.0


def best_label(box: Box, seen: Sequence[Seen], min_iou: float,
               min_side: Optional[float] = None) -> Optional[str]:
    """
    Return the label of the detector box overlapping `box` most, if any matches.

    A match has an IoU of at least min_iou; with min_side (a thing whose top the lidar
    can't see), also a box overlapping it top to bottom and by min_side left to right.
    The highest IoU wins.
    """
    best, best_score = None, -1.0
    for s in seen:
        overlap = iou(box, s.box)
        tall = (min_side is not None and side_overlap(box, s.box) >= min_side
                and min(box[3], s.box[3]) > max(box[1], s.box[1]))
        if (overlap >= min_iou or tall) and overlap > best_score:
            best, best_score = s.label, overlap
    return best


@dataclass
class Camera:
    """One camera: its lens and mount, and its newest boxes with their time (s)."""

    lens: Lens
    mount: Mount
    seen: List[Seen] = field(default_factory=list)
    stamp: float = -math.inf


@dataclass
class Namer:
    """
    Names things from every camera's newest boxes; a name is held `hold` s unseen.

    `name(id, x, y, length, width, height, now)` takes a thing in base_link; the node keeps
    the cameras' boxes fresh (boxes older than max_age s don't count).
    """

    min_iou: float = 0.3
    min_side: float = 0.5
    ceiling: float = 2.0          # m: the lidar's cubes stop here (aav_voxels' max_height)
    hold: float = 3.0
    max_age: float = 0.5
    cameras: Dict[str, Camera] = field(default_factory=dict)
    named: Dict[int, Tuple[str, float]] = field(default_factory=dict)   # id: (name, when)

    def name(self, thing_id: int, x: float, y: float, length: float, width: float,
             height: float, now: float) -> str:
        for camera in self.cameras.values():
            if now - camera.stamp > self.max_age:
                continue
            open_top = height >= self.ceiling
            box = thing_box(camera.lens, camera.mount, x, y, length, width, height)
            label = (best_label(box, camera.seen, self.min_iou,
                                self.min_side if open_top else None) if box else None)
            if label is not None:
                self.named[thing_id] = (label, now)
                return label
        kept = self.named.get(thing_id)
        if kept is not None and now - kept[1] <= self.hold:
            return kept[0]
        self.named.pop(thing_id, None)
        return UNKNOWN
