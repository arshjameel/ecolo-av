"""
Pictures of a flat road, for the lane finder's tests, drawn from first principles.

Each pixel's ray is followed down to the ground, and the ground point is coloured by where
it lies in the lane. None of this comes from the finder's own code, so a mistake in its
geometry can't hide behind the same mistake in the picture.
"""

import math

import numpy as np

SKY = (205, 175, 125)   # BGR, as the bench camera draws them
GRASS = (70, 130, 80)
ROAD = (95, 95, 95)
PAINT = (235, 235, 235)
YELLOW = (40, 200, 230)   # a yellow centre line, BGR

# The bench camera: 640 x 360, 343 px focal length, vehicle.yaml's camera_left_link.
BENCH_CAMERA = (1.80, 0.29, 1.20)


def picture(offset=0.0, heading=0.0, radius=math.inf, lines=('left', 'right'),
            lane_width=3.5, line_width=0.15, width=640, height=360, focal=343.0,
            camera=BENCH_CAMERA, pitch=0.0, dash=None, yellow=()):
    """
    Draw what the camera sees of a lane, in BGR.

    offset   m: the car (base_link) is this far left of the lane's centre
    heading  rad: the car is turned this far left of the lane's direction
    radius   m: + bends left, - bends right, inf is straight. The bend starts at the car.
    lines    which of the lane's two lines are painted
    camera   m: where the camera is in base_link; pitch, rad: + looks down
    dash     (period, painted) m: dashed lines, painted the first `painted` m of each
             `period` along the lane; None: solid
    yellow   which of the lines are painted yellow (the rest white)
    """
    v, u = np.mgrid[0:height, 0:width].astype(float)
    # A ray through each pixel's centre, in the camera's axes: x forward, y left, z up.
    left = -(u - width / 2.0) / focal
    up = -(v - height / 2.0) / focal
    # Tip it down with the camera.
    forward = math.cos(pitch) + math.sin(pitch) * up
    rise = -math.sin(pitch) + math.cos(pitch) * up
    ground = rise < -1e-9
    t = -camera[2] / np.where(ground, rise, -1.0)
    x = camera[0] + t * forward
    y = camera[1] + t * left
    # From base_link into the lane's frame: the lane runs along its x axis.
    c, s = math.cos(heading), math.sin(heading)
    lx, ly = c * x - s * y, offset + s * x + c * y
    if math.isinf(radius):
        across = ly
    else:
        across = math.copysign(1.0, radius) * (abs(radius) - np.hypot(lx, ly - radius))
    image = np.empty((height, width, 3), np.uint8)
    image[:] = SKY
    image[ground] = GRASS
    image[ground & (np.abs(across) < lane_width / 2.0 + 0.5)] = ROAD
    painted = ground
    if dash is not None:
        period, length = dash
        painted = ground & (np.mod(lx, period) < length)
    for side in lines:
        middle = lane_width / 2.0 if side == 'left' else -lane_width / 2.0
        image[painted & (np.abs(across - middle) < line_width / 2.0)] = (
            YELLOW if side in yellow else PAINT)
    return image
