"""
The three-point turn's numbers (sim_check --three-point, TODO 174), without ROS.

A track is a list of (t, x, y, yaw, v): the simulator's truth, base_link on the ground
under the rear axle, v signed (minus: backing).
"""

import math


def turns(track, flips=0.1):
    """Return how often a track changes between forward and back (moving faster than flips)."""
    signs = [1 if v > 0 else -1 for *_, v in track if abs(v) > flips]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


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
