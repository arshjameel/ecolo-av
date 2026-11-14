# Provenance

The `costmap` crate ports the parts of Nav2's `nav2_costmap_2d` that our nav2.yaml runs.
It is rewritten in Rust's own style, not transliterated: plain structs and functions, no
plugins, no ROS.

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Package | `nav2_costmap_2d` |
| Commit | the Jazzy commit in `vendor/ros2-src.repos` (the same as `aav_mppi_lag`'s) |
| Licence | Apache-2.0 (the package's licence; Willow Garage, Samsung Research and the Nav2 authors, from ROS 1's costmap_2d, BSD) |
| Read 8 Nov 2026 | `include/nav2_costmap_2d/costmap_2d.hpp` (`raytraceLine`, `bresenham2D`), `src/costmap_2d.cpp` (`updateOrigin`, `cellDistance`, `worldToMap`), `plugins/obstacle_layer.cpp` (`raytraceFreespace`, marking, `updateFootprint`), `plugins/static_layer.cpp` (`interpretValue`, the rolling copy), `plugins/inflation_layer.cpp` and its header (`computeCost`, the update rule), `src/footprint.cpp` (`padFootprint`, `calculateMinAndMaxDistances`), `src/costmap_2d_publisher.cpp` (the cost table) |

## What was taken

| Ours | Nav2's |
|---|---|
| `cost.rs` | the cost codes; `Costmap2DPublisher`'s table to `OccupancyGrid` values |
| `grid.rs` | `Costmap2D`: `worldToMap`, `mapToWorld`, `cellDistance`, `updateOrigin`, `raytraceLine` and `bresenham2D` |
| `footprint.rs` | `padFootprint`, `calculateMinAndMaxDistances`, `setConvexPolygonCost` (the cells under the car) |
| `static_layer.rs` | `StaticLayer::interpretValue` (trinary, lethal at 100), and its rolling-window copy cell by cell through map -> odom |
| `obstacle.rs` | `ObstacleLayer`: the observation buffer's height band, `raytraceFreespace` (rays cut at the grid's edge), marking by range, footprint clearing, `updateWithMax`; the scan's `inf_is_valid` |
| `inflation.rs` | `InflationLayer::computeCost` and its update rule (an unknown cell becomes only inscribed or lethal) |
| `layered.rs` | `LayeredCostmap::updateMap`: the rolling origin (`getSizeInMetersX`'s half cell included), the reset, the layers in order |

## What changed, and why

| Change | Why |
|---|---|
| The inflation finds each cell's nearest lethal cell exactly (Felzenszwalb and Huttenlocher's distance transform), not by Nav2's brushfire | simpler and linear in the cells; the brushfire can, rarely, give a cell a slightly further obstacle. Otherwise the same costs |
| The whole grid is rebuilt each update, not only the bounds the layers touched | always the same answer as Nav2's with `always_send_full_costmap`; simpler; fast enough (0.4 ms local, `examples/timing.rs`) |
| The cells under the car are its outline's edges and every cell whose centre is inside | Nav2 fills between the outline's cells column by column; they differ by a cell at the edge |
| No `VoxelLayer` (3D), no range sensor, denoise or filter layers | nav2.yaml runs none of them |
| Sensor messages are placed with the odometry at their stamp (interpolated, kept 2 s) and `/tf_static`'s mounts, not tf2 | rclrs has no tf2 yet; our frames are only odom -> base_link (moving) and fixed mounts |
| The node reads one JSON block per costmap (`vehicle.costmap_params`) | the launch file makes it from the same tuned nav2.yaml Nav2 gets, so both see the same layers |
| 14 Nov: odometry (and map -> odom, for the global costmap) that last arrived over `transform_tolerance` ago (0.3 s, Nav2's default) stops the updates and the grid's publishing; the status says why. A scan further than that from any odometry stamp is left out (`age.rs`: `PoseHistory`, `Arrival`) | Nav2 skips an update when it can't find the car's pose, and tf2 drops a scan it can't place; ours placed scans at the last odometry pose for ever (the gap list, 14 Nov). Not publishing lets the readers' own age checks stop the car |
