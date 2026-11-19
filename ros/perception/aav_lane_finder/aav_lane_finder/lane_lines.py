"""
Lane lines in the bird's-eye view: find the paint, follow each line, say where the car is.

No ROS imports, so the tests run anywhere. For each camera picture:

1. `paint_mask`: paint is brighter than the road on both sides of it, and narrow.
2. `follow`: sliding windows climb each line from where it starts near the car.
3. `fit_line`: a second-order fit through the paint they found, y = a x^2 + b x + c, in
   base_link (metres), and how sure the finder is of it (0 to 1).
4. `lane_from`: the car's offset from the lane's centre, the lane's width, each line's
   radius and the way the road bends: aav_lane_geometry/LaneGeometry's fields.
5. `paint_of`: each line's paint, white or yellow, from its pixels' colour (TODO 264):
   the costmap prices a yellow centre line apart from a white edge line.

Signs follow REP-103: x forward, y left. The offset is + when the car is left of the lane's
centre (the 2025-26 lane follower steered right by it). It is measured `lookahead`
metres ahead of base_link, because a PID on the offset at the rear axle weaves; a few
metres ahead, the offset also says which way the car is pointing.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .birds_eye import BirdsEye, Grid

# Each line's paint: LaneGeometry's PAINT_* constants.
PAINT_UNKNOWN = 0
PAINT_WHITE = 1
PAINT_YELLOW = 2

MAX_RADIUS = 9999.0   # m: what a straight line, or a line not seen, reports as its radius
STRAIGHT = 500.0      # m: a bigger radius counts as straight (curve_direction 0)


@dataclass(frozen=True)
class Settings:
    """What the finder looks for. The node makes each one a parameter of the same name."""

    paint_contrast: float = 40.0   # grey levels the paint is brighter than the road, both sides
    paint_width: float = 0.30      # m: the widest paint it looks for
    windows: int = 12              # sliding windows per line, near to far
    window_margin: float = 0.5     # m either side of a window's centre
    min_pixels: int = 8            # paint pixels for a window to count
    min_span: float = 2.0          # m: a line must be seen over this much road
    fit_scale: float = 0.15        # m: the fit's scatter that halves (about) the confidence
    min_confidence: float = 0.3    # below this, a line isn't detected
    lane_width: float = 3.5        # m: assumed until both lines are seen
    min_lane_width: float = 2.5    # m: two "lines" closer than this are one line
    max_lane_width: float = 5.0    # m
    lookahead: float = 3.0         # m ahead of base_link: where the offset is measured
    wheelbase: float = 1.57        # m, for steering_hint (vehicle.yaml)
    # Paint is yellow when its blue is below this share of the lower of its red and green.
    yellow_ratio: float = 0.75


@dataclass(frozen=True)
class Line:
    """One lane line: y = a x^2 + b x + c in base_link (m), and the finder's confidence."""

    a: float
    b: float
    c: float
    confidence: float

    def y(self, x: float) -> float:
        return (self.a * x + self.b) * x + self.c

    def slope(self, x: float) -> float:
        return 2.0 * self.a * x + self.b

    def curvature(self, x: float) -> float:
        """Return 1 / radius (1/m) at x: + when the line bends left."""
        return 2.0 * self.a / (1.0 + self.slope(x) ** 2) ** 1.5


@dataclass(frozen=True)
class Lane:
    """What the finder reports, in LaneGeometry's units and signs."""

    left: Optional[Line]          # None: not detected
    right: Optional[Line]
    left_confidence: float        # 0 to 1, also for a line found but not trusted
    right_confidence: float
    offset: float                 # m: the car from the lane's centre, + = left of it
    width: float                  # m
    left_radius: float            # m; MAX_RADIUS when straight or not seen
    right_radius: float
    mean_radius: float
    curve_direction: float        # +1 bends left, -1 right, 0 straight (or no lane)
    steering_hint: float          # rad, + = left: pure pursuit to the lane's centre
    left_paint: int = PAINT_UNKNOWN    # PAINT_*: unknown when not detected
    right_paint: int = PAINT_UNKNOWN


@dataclass(frozen=True)
class Window:
    """One sliding window, in bird's-eye pixels, and whether it found paint."""

    top: int
    bottom: int
    left: float
    right: float
    hit: bool


@dataclass
class View:
    """The last picture's working, for the debug picture."""

    bev: np.ndarray
    mask: np.ndarray
    windows: List[Window] = field(default_factory=list)


def paint_mask(bev: np.ndarray, resolution: float, s: Settings) -> np.ndarray:
    """
    Return a mask of the paint: pixels brighter than the road on both sides, nearby.

    A pixel counts when it is paint_contrast grey levels brighter than the pixels just
    beyond paint_width / 2 on its left and on its right. A step from road to grass, or to
    a shadow, has a dark side, so it never counts; nor do wide bright patches.
    """
    grey = bev.max(axis=2) if bev.ndim == 3 else bev   # white and yellow paint both stand out
    g = grey.astype(np.int16)
    w = int(math.ceil(s.paint_width / resolution / 2.0)) + 1
    left = np.full_like(g, 255)   # off the edge: as bright as can be, so the edges never count
    right = np.full_like(g, 255)
    left[:, w:] = g[:, :-w]
    right[:, :-w] = g[:, w:]
    return np.minimum(g - left, g - right) >= s.paint_contrast


def starts(rows: np.ndarray, cols: np.ndarray, grid: Grid, s: Settings
           ) -> Tuple[Optional[float], Optional[float]]:
    """Return the columns where the left and right lines start: the paint nearest the car."""
    split = int(round(float(grid.col_of_y(0.0)) + 0.5))   # the car's centre line
    found = []
    for lo, hi in ((0, split), (split, grid.cols)):
        best = None
        # The nearest third of the view first, then all of it, for a line that starts
        # further out (the car is turned, or the line is hidden near the car).
        for near in (rows >= grid.rows * 2 // 3, np.ones(rows.shape, bool)):
            hist = np.bincount(cols[near], minlength=grid.cols)[lo:hi].astype(float)
            if hist.size == 0:
                break
            k = max(1, int(round(s.paint_width / grid.resolution)))
            hist = np.convolve(hist, np.ones(k), 'same')
            if hist.max() >= s.min_pixels:
                best = lo + float(np.argmax(hist))
                break
        found.append(best)
    return found[0], found[1]


def follow(rows: np.ndarray, cols: np.ndarray, start: float, grid: Grid, s: Settings
           ) -> Tuple[np.ndarray, List[Window]]:
    """Climb one line from `start` with sliding windows; return its paint pixels' indices."""
    edges = np.linspace(grid.rows, 0, s.windows + 1).round().astype(int)
    margin = s.window_margin / grid.resolution
    centre, shift, last = float(start), 0.0, None
    picked, windows = [], []
    for k, (bottom, top) in enumerate(zip(edges[:-1], edges[1:])):
        inside = np.flatnonzero((rows >= top) & (rows < bottom)
                                & (np.abs(cols - centre) <= margin))
        hit = inside.size >= s.min_pixels
        windows.append(Window(int(top), int(bottom), centre - margin, centre + margin, hit))
        if hit:
            now = float(cols[inside].mean())
            if last is not None:
                shift = (now - last[1]) / (k - last[0])   # the line's drift per window
            last = (k, now)
            picked.append(inside)
            centre = now + shift
        else:
            centre += shift   # carry on the way the line was going, through a gap
    found = np.concatenate(picked) if picked else np.empty(0, int)
    return found, windows


def fit_line(x: np.ndarray, y: np.ndarray, hits: int, s: Settings) -> Optional[Line]:
    """Fit y = a x^2 + b x + c through one line's paint (m); None if too little was seen."""
    if hits < 2 or x.size < 3 or x.max() - x.min() < s.min_span:
        return None
    # A short stretch can't tell a bend from noise: fit it straight.
    degree = 2 if x.max() - x.min() >= 2.0 * s.min_span else 1
    coefficients = np.polyfit(x, y, degree)
    if degree == 1:
        coefficients = np.concatenate([[0.0], coefficients])
    scatter = float(np.sqrt(np.mean((y - np.polyval(coefficients, x)) ** 2)))
    confidence = (hits / s.windows) * math.exp(-(scatter / s.fit_scale) ** 2)
    a, b, c = (float(v) for v in coefficients)
    return Line(a, b, c, min(1.0, confidence))


def paint_of(bgr: np.ndarray, s: Settings) -> int:
    """
    Return a line's paint from its pixels' colours (n x 3, blue, green, red).

    Yellow paint is red and green with little blue; white has as much blue as the others.
    A grey picture has no colour: every line in it is white.
    """
    if bgr.size == 0:
        return PAINT_UNKNOWN
    bgr = bgr.reshape(-1, 3).astype(float)
    blue = float(bgr[:, 0].mean())
    red_green = float(np.minimum(bgr[:, 1], bgr[:, 2]).mean())
    return PAINT_YELLOW if blue < s.yellow_ratio * red_green else PAINT_WHITE


def radius(curvature: float) -> float:
    return MAX_RADIUS if abs(curvature) < 1.0 / MAX_RADIUS else 1.0 / abs(curvature)


def centre_y(left: Optional[Line], right: Optional[Line], width: float, x: float) -> float:
    """
    Return the lane's centre at x (m, + = left), from the lines detected.

    With one line, the centre is half of `width` beside it, square to the line.
    """
    if left and right:
        return (left.y(x) + right.y(x)) / 2.0
    if left:
        return left.y(x) - width / 2.0 * math.sqrt(1.0 + left.slope(x) ** 2)
    if right:
        return right.y(x) + width / 2.0 * math.sqrt(1.0 + right.slope(x) ** 2)
    raise ValueError('no line to find the centre from')


def lane_from(left: Optional[Line], right: Optional[Line], width: float, s: Settings,
              left_confidence: float = 0.0, right_confidence: float = 0.0) -> Lane:
    """Work out the car's place in the lane from the lines detected (None = not detected)."""
    if not (left or right):
        return Lane(None, None, left_confidence, right_confidence, 0.0, width,
                    MAX_RADIUS, MAX_RADIUS, MAX_RADIUS, 0.0, 0.0)
    x = s.lookahead
    centre = centre_y(left, right, width, x)
    if left and right:
        slope = (left.slope(x) + right.slope(x)) / 2.0
        width = (left.y(x) - right.y(x)) / math.sqrt(1.0 + slope ** 2)
    bends = [line.curvature(x) for line in (left, right) if line]
    mean = sum(bends) / len(bends)
    mean_radius = radius(mean)
    direction = 0.0 if mean_radius > STRAIGHT else math.copysign(1.0, mean)
    # Pure pursuit: the front-wheel angle whose arc reaches the lane's centre at x.
    reach = math.hypot(x, centre)
    hint = math.atan(2.0 * s.wheelbase * math.sin(math.atan2(centre, x)) / reach)
    return Lane(left, right, left_confidence, right_confidence, -centre, width,
                radius(left.curvature(x)) if left else MAX_RADIUS,
                radius(right.curvature(x)) if right else MAX_RADIUS,
                mean_radius, direction, hint)


class LaneFinder:
    """Find the lane in camera pictures, one at a time. Remembers the lane's width."""

    def __init__(self, view: BirdsEye, settings: Settings = Settings()):
        self.view, self.s = view, settings
        self.width = settings.lane_width
        self.last: Optional[View] = None

    def find(self, image: np.ndarray) -> Lane:
        """Return the lane in one picture (BGR or grey, the lens's size)."""
        grid, s = self.view.grid, self.s
        bev = self.view.warp(image)
        mask = paint_mask(bev, grid.resolution, s) & self.view.valid
        rows, cols = np.nonzero(mask)
        self.last = View(bev, mask)
        lines, paints = [], []
        colour = bev if bev.ndim == 3 else np.repeat(bev[:, :, None], 3, axis=2)
        for start in starts(rows, cols, grid, s):
            line, paint = None, PAINT_UNKNOWN
            if start is not None:
                found, windows = follow(rows, cols, start, grid, s)
                self.last.windows += windows
                line = fit_line(grid.x_of_row(rows[found]), grid.y_of_col(cols[found]),
                                sum(w.hit for w in windows), s)
                paint = paint_of(colour[rows[found], cols[found]], s)
            lines.append(line)
            paints.append(paint)
        confidence = [line.confidence if line else 0.0 for line in lines]
        left, right = (line if line and line.confidence >= s.min_confidence else None
                       for line in lines)
        if left and right:
            x = s.lookahead
            apart = left.y(x) - right.y(x)
            if not s.min_lane_width <= apart <= s.max_lane_width:
                # One "line" is something else, or both found the same line: keep the
                # surer one.
                if left.confidence >= right.confidence:
                    right, confidence[1] = None, 0.0
                else:
                    left, confidence[0] = None, 0.0
        lane = lane_from(left, right, self.width, s, confidence[0], confidence[1])
        lane = replace(lane, left_paint=paints[0] if left else PAINT_UNKNOWN,
                       right_paint=paints[1] if right else PAINT_UNKNOWN)
        if left and right:
            self.width += 0.2 * (lane.width - self.width)   # smooth: one frame can be off
        return lane


def draw(view: View, lane: Lane, grid: Grid, scale: int = 2) -> np.ndarray:
    """Return the bird's-eye view with the paint, the windows and the lines drawn on (BGR)."""
    bev = view.bev if view.bev.ndim == 3 else cv2.cvtColor(view.bev, cv2.COLOR_GRAY2BGR)
    picture = (bev * 0.6).astype(np.uint8)
    picture[view.mask] = (0, 220, 255)   # the paint found: yellow
    picture = cv2.resize(picture, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for w in view.windows:
        colour = (0, 200, 0) if w.hit else (90, 90, 90)
        cv2.rectangle(picture, (int(w.left * scale), w.top * scale),
                      (int(w.right * scale), w.bottom * scale - 1), colour, 1)
    xs = np.linspace(grid.x_min, grid.x_max, 40)

    def polyline(ys, colour, thick=2):
        pts = np.stack([(grid.col_of_y(ys) + 0.5) * scale, (grid.row_of_x(xs) + 0.5) * scale],
                       axis=1)
        cv2.polylines(picture, [pts.round().astype(np.int32)], False, colour, thick, cv2.LINE_AA)

    for line, colour in ((lane.left, (255, 120, 0)), (lane.right, (0, 0, 255))):
        if line:
            polyline(np.array([line.y(x) for x in xs]), colour)
    # The car's own centre line (white), and the lane's centre (green).
    polyline(np.zeros_like(xs), (255, 255, 255), 1)
    if lane.left or lane.right:
        polyline(np.array([centre_y(lane.left, lane.right, lane.width, x) for x in xs]),
                 (0, 255, 0), 1)
    text = (f'offset {lane.offset:+.2f} m  width {lane.width:.2f} m'
            if lane.left or lane.right else 'no lane')
    cv2.putText(picture, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return picture
