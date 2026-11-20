# One thing near the car: a group of lidar cubes, followed over time.
# Part of Things, on /world/things (docs/designs/2026-11-14_the-whole-car.md,
# "Seeing: things, not just points"; aav_things_rs).
#
#          ▲ vx, vy (m/s, odom)
#      ┌───┼───┐
#      │   ● x, y (m, odom): the middle of what was seen
#      └───────┘  length along odom x, width along odom y (an upright box)

string UNKNOWN = "unknown"  # no name yet: names need camera-to-lidar calibration (Q22)

uint32 id                   # the same thing keeps its id while it is in the list
string kind                 # UNKNOWN until the camera can name it
float32 x                   # m, in Things' header.frame_id (odom)
float32 y                   # m
float32 vx                  # m/s, over the ground (not relative to the car)
float32 vy                  # m/s
float32 length              # m along x
float32 width               # m along y
float32 height              # m above the road (the highest cube's top)
float32 position_sigma      # m: the filter's one-sigma doubt about x and y
float32 confidence          # 0..1: how often it was seen, less while unseen
builtin_interfaces/Time last_seen   # when the lidar last saw it
uint32 seen                 # how many clouds it was seen in
bool radar                  # its speed came from the radar too
bool coasting               # not seen in the last cloud: moved on its speed (the blind zone)
