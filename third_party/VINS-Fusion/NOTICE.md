# VINS-Fusion algorithm adaptation

Upstream: HKUST-Aerial-Robotics/VINS-Fusion
Pinned revision: be55a937a57436548ddfb1bd324bc1e9a9e828e0
Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology.
Upstream feature tracker author: Qin Tong (qintonguav@gmail.com).
License: GNU General Public License v3.0 (see LICENSE in this directory).

Modified by the MOHISM project on 2026-10-28:

- `vins_estimator/src/featureTracker/feature_tracker.cpp` → `src/feature_tracker.cpp`:
  temporal/stereo forward-backward LK, age-prioritized spatial masking and corner replenishment.
  ROS/camodocal/Eigen interfaces replaced by bounded OpenCV tracks, IDs and rectified metric stereo depth.
- `vins_estimator/src/factor/integration_base.h` → `include/preintegration.hpp`, `src/preintegration.cpp`:
  two-endpoint midpoint propagation, 15-state Jacobian, 18-dimensional noise injection, buffered bias replay.
  Eigen/ROS/Ceres removed; rotations use SO(3) exponential; noise arguments are continuous densities,
  scaled per interval. Added finite/range/monotonicity checks and a 4096-step buffer limit.

This is a modified adaptation, not an upstream VINS-Fusion distribution or endorsement.
The MOHISM userspace estimator and its adaptations are distributed under GPL-3.0-only;
root LICENSE contains the license text. Independent kernel/SDK/dependency components retain
 their own licenses (the kernel driver declares GPL). No upstream authorship is claimed
for MOHISM-specific runtime, USB or MAVLink code.

Upstream source is fetched reproducibly into ignored `vendor/VINS-Fusion` by scripts/bootstrap.sh.
The adapted implementation is tracked in this repository and builds without that checkout.
The 0.5.0 adaptation additionally includes the upstream IMU and three projection
factors, pose local parameterization and utility code in `ported/`. ROS includes
and debug methods were removed. Noise, gravity and projection weights are instance
state; covariance whitening uses LLT; invalid projections are rejected.

`src/tight_backend.cpp`, `src/marginal_prior.cpp` and `src/initialization.cpp`
adapt the upstream estimator assembly, square-root Schur marginalization and
metric stereo visual-inertial alignment designs. They use bounded RAII storage,
validated input and the project runtime instead of ROS. `src/loop_closure.cpp`
implements a bounded ORB retrieval database, stereo geometric verification,
4-DoF graph and atomic maps; it does not include the upstream BRIEF/DBoW
vocabulary or ROS pose-graph application. This is not a verbatim full upstream
application. Dependencies keep their own licenses (Ceres BSD-3-Clause).
