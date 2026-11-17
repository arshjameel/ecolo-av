"""
Each camera's newest obstacle points, together: what camera_obstacles publishes.

No ROS imports, so the tests run anywhere.

camera_obstacles gets one detection message per picture, from whichever camera it came.
Published one camera at a time, a person behind the car would come and go with every
front picture, and the stop zone, which keeps only a source's newest cloud, would
stop and start. So it keeps each camera's newest points and publishes them all, every
time; a camera silent for more than max_age seconds drops out (a stale picture shows
where things were, not where they are).
"""

from typing import Dict, List, Tuple

Point = Tuple[float, float]


class Recent:
    """The newest points from each camera, by its frame."""

    def __init__(self, max_age: float = 1.0):
        self.max_age = max_age
        self._latest: Dict[str, Tuple[float, List[Point], List[Point]]] = {}

    def update(self, camera: str, stamp: float, obstacles: List[Point], view: List[Point]):
        """Keep this camera's newest obstacles and view edge (s, base_link points)."""
        self._latest[camera] = (stamp, list(obstacles), list(view))

    def cameras(self, now: float) -> List[str]:
        """Return the cameras heard from within max_age of now (s)."""
        return sorted(c for c, (t, _, _) in self._latest.items() if now - t <= self.max_age)

    def obstacles(self, now: float) -> List[Point]:
        """Return every fresh camera's obstacle points."""
        return [p for c in self.cameras(now) for p in self._latest[c][1]]

    def view(self, now: float) -> List[Point]:
        """Return every fresh camera's view edge: the costmap clears out to it."""
        return [p for c in self.cameras(now) for p in self._latest[c][2]]
