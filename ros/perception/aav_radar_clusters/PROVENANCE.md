# Provenance

Written for `aav` from 2026-10-11 to 15: a Rust rewrite of the 2025-26 team's C++ radar
clustering node, `dbscan` in `aav_object_detection` (Ben, 10 Oct: our C++ moves to Rust).
On 2026-10-15 it replaced it everywhere, and the C++ package was removed.

| Part | From the C++ node |
|---|---|
| `crates/radar-clustering/src/lib.rs` | DBSCAN and the boxes, from `include/object_detection/cluster/dbscan.hpp` and `common/boundingbox.hpp`: the same clusters, in the same order, and the same box sums |
| `crates/radar-clustering/src/cloud.rs` | new: reads and writes `PointCloud2` bytes (the C++ used PCL) |
| `crates/clusters-node` | the node: `example/dbscan.cpp`'s parameters `eps` and `minimum_points`. Where it differs, on purpose, is listed at the top of `src/main.rs` |
| `launch/radar_clusters.launch.py` | the C++ package's launch file of the same name (ours, 10 Oct) |
| `crates/radar-clustering/tests/cpp_vectors.*` | what the C++ node published for 94 clouds, recorded just before it was removed |

## The C++ package it replaced

`aav_object_detection`: copied on 2026-10-09 from
github.com/AAV-2025-2026/Nav_Radar @ main (6242631882), folder `src/object_detection`.
Original author: Gyro (2025-26 team). Removed from `aav` on 2026-10-15. To find it:

- on GitHub: that repo and commit, `src/object_detection`;
- in our git: `git log --oneline -- ros/perception/aav_object_detection`.

What it held, and what became of it:

| Part | Lines | Now |
|---|---|---|
| `example/dbscan.cpp` + `dbscan.hpp` + `boundingbox.hpp`: the one node we launched | ~540 | this package |
| filters (passthrough, voxel grid, outlier removal, difference of normals, box), ICP, the Euclidean cluster extractor, octomap map | ~1,100 | not ported: never launched in `aav`. Nav2's obstacle layer and collision monitor read the radar's points directly |
| 2025-26 launch and RViz files | ~1,300 | not ported: they ran the parts above, with the 2025-26 topic names |

What `aav` changed in it, while it ran (its own PROVENANCE.md, now in git history):

- 2026-10-10: CMake 3.10 and C++17; `PointTI.h`'s `EIGEN_ALIGN16` moved for today's
  Eigen; a new `package.xml`; and `launch/radar_clusters.launch.py`, the node on our topics.
- 2026-10-15: the topics renamed by area (`/sensors/radar/points` in;
  `/perception/radar/clusters`, `/perception/radar/cluster_boxes` out).

## How the two compared, before the C++ went

`test/capture_cpp_vectors.py` played the same clouds into both nodes, through ROS, on
Ben's MacBook, 15 Oct: 47 clouds with the car's settings (eps 0.8, minimum_points 15) and
47 with the bench's (minimum_points 5). Shapes (two objects, a wall, a border point both
clusters reach, one point many times, noise only, exactly minimum_points) and 40 random
scenes each.

| Check | Result |
|---|---|
| Clusters: the same points, the same ids | 94 of 94 clouds (118 clusters) |
| Boxes: centre and size, bit for bit | 94 of 94 |
| An empty cloud clears the clusters and boxes | Rust yes; the C++ sent the last ones again |
| Ctrl-C | Rust exits 0; the C++ had to be killed |

`tests/cpp_vectors.rs` keeps the first two checks, with no ROS: `cargo test -p
radar-clustering`, in `tools/test.sh`. `capture_cpp_vectors.py --node rust` repeats them
through ROS.

## Behaviour that changed, on purpose

| | C++ | Rust |
|---|---|---|
| `eps` | compared with the squared distance: 0.8 meant 0.89 m | a distance in metres: 0.8 means 0.8 m. The vectors give the Rust sqrt(eps) |
| Noise points | one point at (0, 0, 0) each, in the clusters cloud | left out |
| Old boxes | stayed in RViz | a DELETEALL first, each time |
| Header | boxes in `target_frame` (a parameter); the cloud's time wrong (whole seconds, which PCL reads as microseconds) | the radar cloud's own frame and time, on both |
| Colours | random, and red never set | ten fixed ones, easy to tell apart |
| Intensity | lost (a typo in the field check) | kept |
| Cloud fields | x, y, z, intensity, velocity, rgb | the same, and `cluster`: its box's id |

## Changes

- 2026-10-11: written (Ben's request, 10 Oct), not yet tested.
- 2026-10-15: finished. Topic names by area; the C++ vectors recorded and checked; wired
  into car, bench and sim launch files; `aav_object_detection` removed.
- 2026-11-15 (TODO 176): the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour.
- 2026-11-20 (TODO 176): `cloud.rs` reads through the shared `cloud-view` crate (`ros/bringup/aav_description/cloud-view`); its `Datatype` is now that crate's (all eight PointField types); the C++ vectors still pass bit for bit.
