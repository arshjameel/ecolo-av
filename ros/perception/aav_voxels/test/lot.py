"""A made-up 16-beam lidar turn round a car on a flat lot, for the tests and the timing. No ROS."""

import math

import numpy

LIDAR = (0.9, 0.0, 1.8)      # vehicle.yaml's lidar_link, in base_link
ELEVATIONS = numpy.radians(numpy.arange(-15.0, 16.0, 2.0))
AZIMUTHS = numpy.linspace(-math.pi, math.pi, 1800, endpoint=False)


def rays():
    """Return (N, 3) unit directions: 16 beams x 1800 steps, as Gazebo's lidar."""
    e, a = numpy.meshgrid(ELEVATIONS, AZIMUTHS, indexing='ij')
    return numpy.column_stack([(numpy.cos(e) * numpy.cos(a)).ravel(),
                               (numpy.cos(e) * numpy.sin(a)).ravel(), numpy.sin(e).ravel()])


def lean(roll, pitch):
    """Return the 3 x 3 turn from base_link into the road's frame: the body's roll, pitch."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return numpy.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]) @ numpy.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])


def turn(boxes=(), roll=0.0, pitch=0.0, max_range=100.0):
    """
    Return one turn's points in base_link: the road, and upright boxes on it.

    boxes: (x0, x1, y0, y1, height) standing on the road, in the road's frame. roll,
    pitch: the body's lean (rad, REP-103: + roll = the right side down) about base_link.
    Rays that hit nothing are inf, as Gazebo's.
    """
    body = lean(roll, pitch)
    dr = rays() @ body.T
    origin = body @ numpy.array(LIDAR, dtype=float)
    t = numpy.full(len(dr), numpy.inf)
    down = dr[:, 2] < 0
    t[down] = -origin[2] / dr[down, 2]
    with numpy.errstate(divide='ignore', invalid='ignore'):
        for x0, x1, y0, y1, h in boxes:
            sides = ((0, (x0, x1), 1, (y0, y1)), (1, (y0, y1), 0, (x0, x1)))
            for axis, faces, other, (lo, hi) in sides:
                for face in faces:
                    s = (face - origin[axis]) / dr[:, axis]
                    p = origin + s[:, None] * dr
                    hit = ((s > 0) & (p[:, other] >= lo) & (p[:, other] <= hi)
                           & (p[:, 2] >= 0) & (p[:, 2] <= h))
                    t = numpy.where(hit & (s < t), s, t)
        t[t > max_range] = numpy.inf
        points = origin + t[:, None] * dr
    return points @ body      # back into base_link
