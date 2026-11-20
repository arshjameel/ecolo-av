# Every thing near the car, one list (aav_things_rs, on /world/things, 10 Hz).
# docs/designs/2026-11-14_the-whole-car.md, "Seeing: things, not just points".
# A thing stays in the list for a few seconds after it leaves view, moved on its speed,
# so what went into the blind zone before the bumper is still known.

std_msgs/Header header      # frame_id odom; stamp: the time the places are for
Thing[] things
