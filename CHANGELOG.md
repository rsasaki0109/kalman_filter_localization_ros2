# Changelog

## 0.3.0 — 2026-10-24

Release target: ROS 2 Jazzy, from the `devel` branch. All three packages
(`kalman_filter_localization_core`, `kalman_filter_localization_msgs`, and
`kalman_filter_localization`) now share version 0.3.0.

### Localization and interfaces

- ROS-independent 15-error-state inertial ESKF with position, velocity,
  quaternion attitude, gyro/accelerometer biases and configurable vehicle models.
- GNSS Pose/NavSatFix and velocity/course aiding, wheel-speed fusion, optional
  vehicle constraints, robust observations, fault isolation and calibration tools.
- Correlated full 6DoF pose observations and full-covariance initialization,
  including timestamped frame conversion, atomic rejection and NIS gating.
- Selectable source-frame linear-Twist axes, covariance/origin transport and
  delayed pose/twist replay without constraining unobserved velocity axes.
- Consistent Pose/Twist/Odometry outputs with transformed full covariance,
  input freshness monitoring and typed status/measurement diagnostics.
- Session reset and SetBool activation; stale queued observations and dynamic
  TF history are cleared while static calibration is retained.
- Autoware topic/service integration launches and Nav2 map-to-odom composition
  using local odometry TF at matching sensor timestamps.
- Optional offline fixed-lag RTS smoothing oracle and repeatable evaluation tools.

### Demos and evaluation

- README hero using real UrbanNav Odaiba GNSS/IMU/wheel measurements, with
  attribution, pinned RTKLIB processing, source hashes and reproducible replay.
- Independently reproducible synthetic 6DoF climbing/banked scenario with
  delayed poses and a 12-second absolute-pose outage; planar baseline retained.
- Dataset profiles, regression protocols, deterministic receipt/output checks
  and documented separation of regression data from the blind holdout.

### Migration and known limitations

- Rebuild all packages and downstream consumers of the changed custom messages.
  `EstimatorStatus` has new pose-freshness, measurement-count and activation fields.
- Standalone nodes remain active by default. The Autoware launch starts inactive;
  provide a valid seed, activate via SetBool, and deliver a new IMU sample.
- Full-covariance initialization is strict in integration profiles. Legacy
  diagonal covariance fallback remains the standalone default.
- Autoware/Nav2 full-stack navigation and exact parameter/diagnostic/QoS parity
  are not validated. Yaw-bias-specific outputs and joint angular-Twist fusion
  remain pending. Repeated activation is intentionally idempotent.
- Odaiba is a regression/demo sequence with reference-seeded initialization and
  incomplete sensor extrinsics. Published errors include those assumptions;
  they do not establish calibrated accuracy or independent holdout performance.
- ROS 2 Humble is not newly validated by this release; Jazzy is the tested target.

See [stack launch documentation](docs/stack_launch.md),
[real-data reproduction and results](demos/real_data_hero/README.md), and
[synthetic 6DoF reproduction](demos/hero_6dof/README.md).
