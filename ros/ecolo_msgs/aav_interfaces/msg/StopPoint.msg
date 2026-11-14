# One place on the road ahead where the car must stop, and why.
# Part of RoadAhead (docs/designs/2026-11-14_the-whole-car.md, "The road ahead").

uint8 WAIT = 0      # stop, then go on once the reason clears (a sign, a person, a lock)
uint8 FINAL = 1     # stop and stay: the goal, the fence

float32 along       # m along RoadAhead's line, from its first point
string reason       # sign, obstacle, fence, goal
uint8 kind          # WAIT or FINAL
