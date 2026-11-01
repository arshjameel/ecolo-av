# Provenance

Our own code, written for `ecolo-av` from 2026-10-29 (Ben, 28 Oct: "take ideas from that
and rebuild them in rust", "lets write our own version of it all"). Not a port: no file
was copied or translated. What we followed, and from where:

| Idea | From | Licence |
|---|---|---|
| NDT: cubes with a mean and covariance, a scan scored on them, Newton | P. Biber and W. Straßer, "The Normal Distributions Transform: A New Approach to Laser Scan Matching", IROS 2003 | paper |
| The score's outlier mix (d1, d2, eq. 6.8), its gradient and Hessian (eq. 6.12-6.13), the eigenvalue floor, scoring against nearby cubes | M. Magnusson, "The Three-Dimensional Normal-Distributions Transform", PhD thesis, Örebro University, 2009, chapter 6 | thesis |
| The outlier ratio 0.55, a step cap, the score per point as the fit's quality, coarse-to-fine, initial pose from the EKF, the map as PCD | Autoware Universe `localization/autoware_ndt_scan_matcher` and its `ndt_omp` (its design page and defaults) | Apache-2.0 |
| The 2D idea, the eigenvalue floor's 1 % | `tools/ndt_demo.py` (ours, 28 Oct) | ours |

Where ours differs from Autoware's, on purpose: x, y and yaw only (a flat lot; design
N1); the ground taken out; a backtracking line search instead of More-Thuente; the 6
face neighbours, not a k-d tree radius search; no multithreading yet.

## Changes since

None yet.
