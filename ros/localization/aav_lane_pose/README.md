# aav_lane_pose

## Overview

A trial (TODO 155), **off by default**: the lane finder as a pose sensor.

The GPS says where the car is to a metre or so, and it jitters. The painted lines say
where the car is **across** the road to a few centimetres. With the lane map, that is a
position in the map: sure across the lane, nothing along it.

```text
 lane_finder ──lines──┐
                      ├─► lane_pose ──► /localization/lane/pose ──► ekf_global
 lane map ──centre────┤    (gates)       sure across, 50 m along     (x, y)
 TF map → base_link ──┘
```

The pose's covariance is an ellipse turned to the lane's heading: thin across the lane,
very long along it. So the global EKF takes only the sideways part.

## Interface

### Node `lane_pose`

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/lane/geometry` | `aav_lane_geometry/LaneGeometry`: the lines in base_link (lane_finder) |
| in | TF `map → base_link` | where the filter has the car, at the picture's stamp |
| out | `/localization/lane/pose` | `geometry_msgs/PoseWithCovarianceStamped`, map frame, the picture's stamp |

| Parameter | Default | Means |
|---|---|---|
| `lane_map` | (needed) | a `lanes.yaml` (`aav_description` `lane_map.py`) |
| `min_confidence` | 0.5 | the surest line must reach it |
| `max_heading` | 0.3 rad | the car turned further from the lane: refused |
| `width_tolerance` | 0.6 m | both lines' gap against the map lane's width |
| `max_jump`, `jump_window` | 0.4 m, 1 s | a jump in the offset this soon: refused |
| `max_innovation` | 1.5 m | further than this from the filter's own: refused |
| `behind`, `ahead`, `step` | 4, 12, 4 m | no junction or road end over this stretch |
| `sigma_both`, `sigma_one` | 0.15, 0.30 m | across the lane, two lines or one |
| `sigma_along` | 50 m | along the lane: no information |
| `sigma_yaw` | 0.05 rad | the yaw's (in the message, not fused) |

Every 10 s it logs how many pictures each gate refused (`ok 120, junction 14, ...`).

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `lane_fix.py` | yes | `seen_from` (lines → offset, heading, width), `LanePose.update` (the gates, the fix), `across_covariance`, `Settings` |
| `lane_pose.py` | no | the node |

### Switch it on

```sh
pixi run sim world:=road lane_pose:=true            # the road world's lanes.yaml
pixi run sim world:=campus lane_pose:=true          # maps/campus/lanes.yaml
```

`sim.launch.py lane_pose:=true` starts `lane_finder` (without the lane follower) and
passes `lane_pose` and the lane map to `aav_localization`'s launch file, which starts
this node and adds `config/ekf_global_lane.yaml` to `ekf_global`.

## Invariants

- Off by default; the global EKF on GPS alone stays the champion until this wins.
- Only x and y are fused; the covariance across the lane only.
- No fix near a junction, a road's end, off the map, or in the other lane.
- `lane_fix.py` has no ROS imports, and its own tests.
- REP-103: `d` + when the car is left of its lane's centre. Metres, radians.

## Side Effects

- Publishes `/localization/lane/pose`. Reads the lane map once at start.

## Verification

```sh
cd ros/localization/aav_lane_pose && python3 -m pytest -q   # the maths and gates (tools/test.sh)
pixi run build && colcon test --packages-select aav_lane_pose aav_bringup
```

In the simulator, against the truth: `tools/lane_drive.py --lane-pose` (results in
`docs/trials.md`, "The lane finder as a pose sensor").
