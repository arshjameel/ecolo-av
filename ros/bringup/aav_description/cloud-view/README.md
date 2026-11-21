# cloud-view

## Overview

A `sensor_msgs/PointCloud2`'s bytes, read by field name. A Rust crate with no ROS and no
dependencies (TODO 176): **one view for every Rust crate that reads a cloud**, in place
of the four copies they had.

```text
 PointCloud2 ─► CloudView ─► column("x", FLOATS) ─► Column ─┐
  (the node)    (no ROS)     a name, a type, an offset      ├─► each point's value
                             points() ─► each point's bytes ┘
```

| Who | Reads |
|---|---|
| `voxels` (`aav_voxels_rs`), and through it `lidar-scan`, `kinematic-icp` | x, y, z as floats |
| `ndt-core` (`aav_ndt`) | x, y, z as floats |
| `radar-clustering` (`aav_radar_clusters`), and through it `debug-core` (`aav_hud`) and the dev window | x, y, z float32; intensity and velocity if there |
| `things-core` (`aav_things_rs`) | named columns, any number type |

## Interface

| Item | What |
|---|---|
| `CloudView`, `Field` | the message's layout and bytes, copied by the node; `Field::new` for a cloud we write |
| `Datatype` (`code`, `from_code`, `size`, `ALL`, `FLOATS`) | PointField's eight number types |
| `CloudView::column(name, wanted)`, `optional_column` | a field as a `Column`: one value of a wanted type, inside a point |
| `CloudView::points()` | each point's bytes, row by row (padded rows too); the length checked once |
| `Column::read(point)` | the value as f64 (exact for every type); NaN if the bytes are too short |
| `xyz(cloud)`, `read_columns(cloud, names)` | x, y, z as floats; any named columns |
| `XYZ32_FIELDS`, `XYZ32_STEP`, `write_xyz32` | an x y z float32 cloud, written |
| `CloudError` | `MissingField`, `WrongType`, `OutsidePoint`, `TooShort` |

## Not on it yet

Three nodes read x, y (z) inline from the message: `costmap-node`, `behaviour-node` and
`stop-zone-node`. They skip a short point instead of refusing the cloud; the stop zone is
a safety lock, so moving it is a reviewed change.

## Testing

`cargo test` here (`tools/test.sh` runs it, with `cargo fmt` and `cargo clippy`). Each
crate that uses it keeps its own tests of what it reads.
