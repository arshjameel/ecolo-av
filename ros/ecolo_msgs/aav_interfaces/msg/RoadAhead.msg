# The road ahead: where the car may be, how fast, and where it must stop.
# One writer (the behaviour planner), every driver and screen reads it.
# docs/designs/2026-11-14_the-whole-car.md, "The road ahead". Published on /world/road_ahead.
#
#        left hard edge (never cross) ══════════════════════╗
#        left soft edge (cross only when the boss allows) ─ ║ ◄ the END: the car
#   car ● ── the line, a point every `spacing` m ─────────► ║   may not pass it
#        right edges ═══════════════════════════════════════╝
#        speed limit:  1.0 … 1.0 … 0.6 … 0.3 … 0
#
# The END is a movement authority, as on a railway: the car may run only as far as it
# has been given. The boss keeps extending it. If the boss goes silent, nothing new
# comes, and the car stops at the end of what it was last given: silence never means
# "carry on". The speed limit at the end is always 0.

std_msgs/Header header      # frame_id map; stamp: when it was made

# The line to follow: one point every `spacing` m, the first at the car's foot on it.
float32 spacing             # m between points
float32[] x                 # m, in header.frame_id
float32[] y                 # m
float32[] heading           # rad, the line's direction at each point
float32[] speed_limit       # m/s at each point: never above it there; 0 at and past `end`

# The edges, sideways from each point (m, always positive). NaN: not known yet (they
# come with the lane map, W5).
float32[] left_hard
float32[] right_hard
float32[] left_soft
float32[] right_soft

StopPoint[] stops           # every stop on the line, nearest first
float32 end                 # m along the line: the car may not pass it

string state                # the boss's state (DRIVE, APPROACH, STOPPED, WAIT, GO, ASK)
string reason               # why: the binding stop's reason, a lock, or empty
