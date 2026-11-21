# Drive by hand and record everything

Ben, 20 Nov: "make the joystick and the remote control drive the car and the hardware
screen ... provide the heartbeat", and record "the lidar, camera, stopping, steering,
acceleration, imu data, etc. Especially how it all fits together."

[[untested]] on the car: both machine files are new (20 Nov), tested only by the launch
tests.

## Who does what

```text
 laptop (laptop_orin)                       Orin (orin_sensors)
   joystick ─► mux ─► ego ─► output mux       lidar, front cameras, the EKFs, /tf
   hardware screen ─┘        (the heartbeat)  records the bag
            │ Ethernet cable                          ▲ Ethernet
            ▼                                         │ every topic
 Motor Pi (motor_pi): bridge ─UART─► ESP32 ◄─iBUS─ RC transmitter
 localization Pi (localization): IMU, GPS
```

| Computer | Runs | Command (`export ROS_DOMAIN_ID=42` first) |
|---|---|---|
| Motor Pi | the bridge to the ESP32 | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi` |
| localization Pi | IMU, GPS | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=localization` |
| Orin | lidar, cameras, where the car is | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=orin_sensors` |
| laptop, joystick plugged in | mux, ego, output mux: the heartbeat | `pixi run tools/ros.sh ros2 launch aav_bringup car.launch.py machine:=laptop_orin max_speed:=0.3` |
| laptop, second terminal | the hardware screen | `pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true` |

No Orin, or it doesn't run our stack yet? Skip its row: the joystick and the screen still
work, but no lidar or cameras.

**Plug the laptop into the car's switch with the Ethernet cable**, not WiFi: the heartbeat
then goes over the wire, and the lidar's 9 MB/s fits. Give the laptop an address on the
car's network (192.168.1.x) and run `pixi run doctor`.

**Then turn the laptop's WiFi off.** ROS (Cyclone DDS) uses one network card, and with both
on it may pick the WiFi. If the WiFi must stay on, name the cable's card (`ifconfig`: the
`en` with the 192.168.1.x address) in `~/.config/aav/cyclonedds.xml`:

```xml
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces>
  <NetworkInterface name="en7"/>
</Interfaces></General></Domain></CycloneDDS>
```

**Never** run `car orin` or `car motor_pi_alone` beside these: two muxes and two egos
would both drive.

## Who drives

| Driver | How | Needs |
|---|---|---|
| RC transmitter, CH6 up | the sticks, straight into the ESP32 | nothing else: no laptop, no Pi |
| joystick | through the laptop's mux and ego, as **a pedal**: push and hold to hold a speed, let go to slow at 1 m/s², pull back to brake, hold back at a stop for 1 s to reverse. The centre 10 % is let go | the laptop, the Motor Pi |
| hardware screen | STOP, E-STOP, steering; **speed off on the real car** (TODO 49b) | the same |
| scripted drive ("3 m and stop") | `ros2 action send_goal /scripted_drive aav_interfaces/action/ScriptedDrive "{target_speed: 0.5, acceleration: 0.3, deceleration: 0.3, hold_distance: 3.0, curvature: 0.0}" --feedback` | the Orin's EKFs (the wheels come by its CAN); the geofence's good position |

CH6 is the RC's override switch (the 2025-26 `RC_EN_CH`). Which switch sends it is set in
the transmitter's menu: flip each and watch the hardware screen's GRAPHS page. Two or three
positions is the hardware lead's question (H10).

## Which firmware

| On the ESP32 | The heartbeat | Where |
|---|---|---|
| the C++ (on the car today) | **none**: frames stop, the last throttle stays | **stands only** |
| the Rust, with `frame_version: 2` on the Motor Pi | 100 ms without a new counter: full brake, latched. Over WiFi a hiccup brakes too: use the Ethernet cable | the ground, at `max_speed:=0.3` |

Flashing the Rust is Ben's call ([the bench](esp32-bench.md)).

## Record

On the Orin (or the laptop, if there's no Orin), before the first move:

```sh
pixi run tools/record.sh --lidar handdrive      # bags/<date>_handdrive/, MCAP
```

About 9 MB/s with the lidar: 10 minutes is about 5 GB. Without `--lidar`, a few hundred
KB/s.

| In the bag | Shows |
|---|---|
| the joystick, the screen's buttons, `/control/cmd_vel`, the mux's state | who drove, and what they asked |
| `/ego/actuator_cmd`, `/control/car/actuator_cmd`, `/control/car/heartbeat` | what ego sent, what passed the gate, the heartbeat |
| `/control/car/link_frame`, `/vehicle/esp32/uplink_health` | what the Pi wrote; what the ESP32 says back (with the uplink) |
| `/vehicle/steering`, `/throttle`, `/brake`, `/wheels`, `/odom` | what the car did |
| the IMU, the GPS, the EKFs, `/tf` | how it moved, and where it was |
| the lidar, the front camera (JPEG) | what it saw |

## What to do, in order

Each move 3 times, a few seconds still between moves, so each one stands out in the bag.

| # | Move | Measures |
|---|---|---|
| 1 | on stands: steering full left, centre, full right, slowly then quickly | the steering's lag and rate |
| 2 | on stands: throttle steps, 0 to a little and back | the throttle's lag |
| 3 | on stands: brake steps | the brake's lag |
| 4 | E-STOP on the screen; then the joystick let go; then the RC switched off | each stop, and how fast |
| 5 | on the ground (Rust firmware only): straight at 0.3 m/s, then a stop | the stopping distance |
| 6 | a slow circle each way, full lock | the turning radius, the IMU's turn rate |
| 7 | one lap of the lot by joystick | everything together: the lidar map, the EKFs, the camera |

## After

| Command | Gives |
|---|---|
| `pixi run tools/ros.sh python3 tools/score_drive.py bags/<drive>` | one scorecard: speed, distance, stops, who drove |
| `pixi run link-check bags/<drive>` | did the Pi's frames reach the ESP32 |
| `pixi run map-drive bags/<drive>` | a map of the lot from the lap (with `--lidar`) |
| `pixi run -e rerun view-bag bags/<drive>/<drive>_0.mcap` | watch it all again |

Steps 1-3 fit the lags in `vehicle.yaml` ([the actuator tests](actuator-tests.md)): every
lag there is still a guess.
