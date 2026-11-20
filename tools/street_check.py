"""
The three-point turn's numbers (sim_check --three-point, TODO 174), without ROS.

A track is a list of (t, x, y, yaw, v): the simulator's truth, base_link on the ground
under the rear axle, v signed (minus: backing).
"""

import math

from drive_metrics import cusp_pieces


def turns(track, flips=0.1):
    """Return how often a track changes between forward and back (moving faster than flips)."""
    signs = [1 if v > 0 else -1 for *_, v in track if abs(v) > flips]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def plan_cusps(paths):
    """
    Return each new plan's changes of direction, in order: '3' or '3, 2, 2'.

    paths: every /plan seen, each [(x, y)]; a plan the same as the one before is counted
    once (the navigator republishes it). '' if there were none.
    """
    counts, last = [], None
    for path in paths:
        if path and path != last:
            counts.append(len(cusp_pieces(path)) - 1)
        last = path
    return ', '.join(str(n) for n in counts)


def clearance(track, x0, y0, heading, half_street, body):
    """
    Return the least room (m) between the car's corners and a street's walls, over a track.

    The street runs along `heading` through (x0, y0), `half_street` m to each wall's face.
    body: vehicle.yaml's (length, width, rear_overhang). Below 0: a corner went through.
    """
    front, rear = body['length'] - body['rear_overhang'], -body['rear_overhang']
    side = body['width'] / 2.0
    nx, ny = -math.sin(heading), math.cos(heading)       # across the street, to the left
    least = math.inf
    for _, x, y, yaw, _ in track:
        c, s = math.cos(yaw), math.sin(yaw)
        for along, across in ((front, side), (front, -side), (rear, side), (rear, -side)):
            px, py = x + along * c - across * s, y + along * s + across * c
            least = min(least, half_street - abs((px - x0) * nx + (py - y0) * ny))
    return least


WALL_THICK = 0.3    # m: aav_sim_models's wall model (10 x 0.3 x 2 m)


def block(x0, y0, heading, along, name='wall_block'):
    """
    Return a wall across the street (name, x, y, yaw), its near face `along` m ahead.

    TODO 174: the narrow street blocked ahead (sim_check --blocked-street). The wall model
    is 10 m long, wider than any street here, so it closes it.
    """
    c, s = math.cos(heading), math.sin(heading)
    centre = along + WALL_THICK / 2.0
    return name, x0 + centre * c, y0 + centre * s, heading + math.pi / 2.0


def walls(x0, y0, heading, half_street, alongs):
    """
    Return where a street's walls go: (name, x, y), one a side for each centre in `alongs`.

    The street runs along `heading` through (x0, y0); each wall's face is `half_street` m
    from that line, so its centre is half a wall's thickness further out. alongs: each
    wall's centre, m along the street from (x0, y0). Names: wall_l0, wall_l1... (left),
    wall_r0... (right).
    """
    c, s = math.cos(heading), math.sin(heading)
    offset = half_street + WALL_THICK / 2.0
    out = []
    for side, tag in ((1.0, 'l'), (-1.0, 'r')):
        for k, along in enumerate(alongs):
            out.append((f'wall_{tag}{k}', x0 + along * c - side * offset * s,
                        y0 + along * s + side * offset * c))
    return out
