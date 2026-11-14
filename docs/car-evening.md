# Tonight: drive as is, and record everything

!!! note "A record: this was the plan for 6 November"
    What happened that evening: [the 6 Nov audit](history/audit-2026-11-06-car.md) (a fuse
    and a circuit board, then the car moved; no bag was recorded). Use this page again
    only to drive the 2025-26 stack as is. To put our software on: [Rolling out](rollout.md).

Ben, 6 Nov: we drive the car with nothing flashed. The ESP32 keeps the 2025-26 C++ and the car
runs the 2025-26 stack (Humble, domain 0). Our software drives nothing tonight. **Every minute of
driving is data we don't have yet.**

> **The short version**
> - Their stack drives. Ours only gets the recordings, at home.
> - **Test the Kar-Tech stop first**, and keep it in someone's hand.
> - Record on each computer, to its own disk.
> - Write the time and what you did on one sheet.
> - Copy the bags off before anyone goes home.

## Safety first

| Fact | So |
|---|---|
| The C++ has **no timeout**: if the Pi, the bridge or the wire dies, the last throttle stays | The stop is a person: the Kar-Tech stop, in hand, whenever the car has power |
| The RC transmitter has no timeout either | Turning it off does not stop the car |
| The runbook says the C++ is "never the joystick or the ground" ([car day](car-day.md)) | Tonight is the **hardware lead's and Ben's decision**. Write down who decided |
| The throttle's strength is a guess | Empty lot, walking pace at first, a safety driver in the seat |

Before moving on the ground: ask S1-S3 (what the stops cut, the brake without power, what stops a
frozen Pi), then test the Kar-Tech stop with the wheels off the ground.

**Never two drive paths.** Don't start `car motor_pi` or our bridge beside theirs.

## Before you drive (15 min)

| ☐ | Do | Why |
|---|---|---|
| ☐ | `df -h` on the Orin and LOCALIZATION | The lidar is about 5 GB per 10 min; stop at 2 GB left |
| ☐ | `date` on every computer, at the same moment; a photo of the answers | Their clocks may differ: this lines the bags up at home |
| ☐ | `ros2 topic list > ~/aav-bags/topics.txt` and `ros2 topic hz` on the ones below | What their stack really publishes tonight |
| ☐ | A photo of the wiring, the Jrks and the ESP32 | Reference, and for the hardware lead's answers |
| ☐ | A phone on the dash, filming the road and a clock | Shows what the car did, and proves the times |

## Record

Each computer records its own sensors, so a weak network can't lose data. Log in with the logins
from the vault. Run each in its own `tmux`, so a dropped ssh doesn't stop it.

| Computer | Run | Gives |
|---|---|---|
| LOCALIZATION (.113) | `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/<name> /imu/data_raw /imu/mag /aav/gps/fix` | The real IMU and GPS |
| The Orin (.140) | `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/<name> /velodyne_points /scan` | The real lidar |
| CORTEX (.120) | `source /opt/ros/humble/setup.bash; ros2 bag record -o ~/aav-bags/<name> <the command topics from topics.txt>` | What the car was told to do |

Keep one bag per drive below, each with its own `<name>`. Ctrl-C ends a bag cleanly.

## What to drive

| # | Name | Do | Gives us |
|---|---|---|---|
| 1 | `parked` | Parked, nobody touching it, 5 min | The IMU's bias, the GPS's noise |
| 2 | `walk` | Parked: someone walks past at 5 m, then 10 m | The lidar on a person, for the tracker and ghost obstacles |
| 3 | `tape` | 10 m by tape, straight, slowly; start and end on tape marks | The wheel diameter (TODO 60) |
| 4 | `straight` | A long straight, wheel centred, slowly | How much it pulls right (TODO 67) |
| 5 | `lock` | One slow full circle at full lock, each way; mark the circle's edge and measure it | The real turning radius |
| 6 | `coast` | Up to a steady slow speed, let go, roll to a stop; 3 times | The drag, which settles the 5-times throttle guess (TODO 52b) |
| 7 | `brake` | Same speed, brake firmly, measure the distance; 3 times | The real stopping distance |
| 8 | `lot` | 2 slow laps, a figure 8, a stop by each cone | NDT's map, and real scores for every algorithm |
| 9 | `smooth` | Your smoothest drive round a bend or two | The sway a person gets: the number the followers must beat |
| 10 | `buildings` | If there's time and it's allowed: slowly past known buildings on campus | The first real drive to fit the campus map to (TODO 120) |

Do the first eight even if nothing else happens. Note the lot's corner from a phone (latitude and
longitude): it becomes `navsat.yaml`'s `datum`.

## The sheet

One line per drive: bag name, start time, who drove, what happened, anything odd (a stop, a
stumble, a sensor dropping out). A bag nobody can explain is half a bag.

## Before you go home

| ☐ | Do |
|---|---|
| ☐ | Copy to the encrypted drive: `rsync -a <user>@192.168.1.140:aav-bags/ ~/aav-bags/orin/`, the same for .113 and .120 |
| ☐ | Check the sizes match, and `ros2 bag info` opens one from each computer |
| ☐ | Photograph the sheet |
| ☐ | Leave their stack as you found it. Delete nothing from the car |

Bags stay out of git. Ben's Mac has only 18 GB free, so bags go on the drive, not the Mac.

## At home, next

Their topics have the old names, so the bags need renaming before our tools read them (TODO 125).
Then: the wheel diameter and drag into `vehicle.yaml`, NDT's map with `pixi run ndt-map build`, and
every algorithm scored on real data. **Untested:** nothing has read a Humble bag from this car yet.
