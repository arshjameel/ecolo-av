# Provenance

A port of **nav2_amcl**'s behaviour to Rust, written for `ecolo-av` from 2026-11-18
(TODO 231: the last Nav2 node). Rewritten in Rust's own style, not transliterated: no
file was copied. Because it follows nav2_amcl's code closely, it keeps its licence,
**LGPL-2.1-or-later**.

| What | From | Licence |
|---|---|---|
| The filter: Gaussian start, weights, the running averages, resampling with KLD's count, clusters, the estimate (heaviest cluster's mean, the set's covariance, yaw's -2 ln R), the converged test | `nav2_amcl/src/pf/pf.c`, `pf_kdtree.c` (Andrew Howard, Player) | LGPL-2.1-or-later |
| The motion model (rot1, trans, rot2; reversing as no turn) | `nav2_amcl/src/motion_model/differential_motion_model.cpp` | LGPL-2.1-or-later |
| The likelihood field (`z_hit`, `z_rand`, `sigma_hit`, the beam step, `1 + Σ p³`) and the distance table's cap | `nav2_amcl/src/sensors/laser/likelihood_field_model.cpp`, `src/map/map_cspace.cpp` | LGPL-2.1-or-later |
| When to update, resample and publish; the initial pose and its odometry correction; the scan's range limits; parameter names and defaults; topics and QoS | `nav2_amcl/src/amcl_node.cpp` | LGPL-2.1-or-later |

Source read: `vendor/ros2-src/ecosystem/navigation2` at commit
`645abd95f2be02a13ca539b29c2ddc065db34f89` (nav2_amcl 1.3.13, Jazzy).

Ideas, not code:

| Idea | From |
|---|---|
| Monte Carlo localization; KLD sampling | D. Fox, W. Burgard, F. Dellaert, S. Thrun, "Monte Carlo Localization", AAAI 1999; D. Fox, "Adapting the Sample Size in Particle Filters Through KLD-Sampling", IJRR 2003 |
| The odometry motion model, low-variance resampling, Augmented MCL's averages | S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press 2005 (tables 4.4, 5.6, 8.3) |
| The exact distance transform | P. Felzenszwalb, D. Huttenlocher, "Distance Transforms of Sampled Functions", Theory of Computing 2012 |
| The random numbers (`rng.rs`) | ours: `aav_mppi_lag`'s `mppi-lag/src/rng.rs` (xoshiro256++, public domain) |

Where ours differs from nav2_amcl, on purpose (each explained in the code):

- Resampling is low-variance; its picks are shuffled so KLD can stop at any count.
- Yaw stays in (-π, π]; KLD's yaw bins wrap round, so a cluster across ±π is one.
- Recovery's averages are of the weighted mean likelihood (nav2 also divided by the
  particle count), and after an injection only the fast one starts again (nav2 zeroed
  both and forgot what a good fit scores).
- A random particle lands anywhere in its free cell, not at its centre.
- Odometry from `ekf_local`'s messages, not TF; never TF out; not a lifecycle node.

## Changes since

| Date | Change |
|---|---|
| 2026-11-18 | First version (Ben Woodhead) |
