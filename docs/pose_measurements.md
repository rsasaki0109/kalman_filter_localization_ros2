# Joint 6DoF pose measurements

The optional `geometry_msgs/PoseWithCovarianceStamped` input fuses position and
orientation in one update, preserving position/attitude covariance. It is useful
for sources such as NDT, visual localization or another independent absolute
body-pose sensor. It is separate from the position-only GNSS `PoseStamped` input.

## Configure

```yaml
ekf_localization:
  ros__parameters:
    reference_frame_id: map
    robot_frame_id: base_link
    pose_measurement_topic: measured_pose_with_covariance
    max_pose_nis: 16.812
    enable_measurement_replay: true
    measurement_history_duration_sec: 2.0
    max_future_measurement_wait_sec: 0.2
    use_flat_ground: false
    use_imu_orientation: false
```

The topic defaults to an empty string, so existing configurations do not acquire
an additional subscription. `max_pose_nis` is nonnegative; zero disables gating.
The example threshold is the approximate 99th percentile of chi-square with
**six** degrees of freedom. It must be reviewed alongside sensor calibration
and covariance consistency, rather than treated as a universal tuning constant.

The [Autoware pose-input development profile](../kalman_filter_localization_ros2/param/profiles/autoware_6dof_pose.yaml)
also selects IMU-time output, the fast propagation model, and turns off the
position-only GNSS/wheel/odometry paths. Provide the actual sensor topics,
extrinsics and calibrated IMU noise before use. This profile is not yet a full
Autoware replacement: twist-input fusion and activation/reset service parity
remain separate work.

An initial pose must be supplied first. This measurement stream does not
implicitly reset or activate the estimator. The existing covariance-bearing
initialization path defaults to diagonal fallback for compatibility. Set
`initial_pose_covariance_mode: full` for [correlated initialization/reset](initialization_reset.md).

## Pose and frame contract

The pose must describe `robot_frame_id`, including any upstream sensor-to-body
extrinsic/lever-arm correction. The message has no `child_frame_id`, so the
estimator cannot infer that it describes a camera, LiDAR or GNSS antenna instead
of the body origin. `header.frame_id` names the **reference frame of the pose**,
not the sensor body being tracked.

If the message reference differs from `reference_frame_id`, the estimator looks
up its transform at the measurement's sensor timestamp. It transforms position,
quaternion, and the **entire** covariance using `diag(R, R)`. This coordinate
conversion assumes the reference-frame transform is known; it does not add
uncertainty for a stochastic transform. Missing/empty reference frames are
rejected; an identity transform is never substituted for a missing TF. Avoid
reference transforms derived from this same estimate, which would feed its own
information back into the filter.

Input covariance is row-major `[x, y, z, rotation_x, rotation_y, rotation_z]`,
with rotation perturbations about fixed reference-frame axes (ROS convention).
It is not covariance of Euler-angle coordinates. Position variances have units
m², rotation variances rad², and their cross terms m·rad. The full matrix must
be finite, symmetric and positive definite; unknown/zero variances are rejected
rather than replaced with confident measurements. Non-finite positions and
zero, non-finite or unnormalizable quaternions are also rejected.

## Update model

For nominal world-from-body orientation `q`, the residual is
`[p_measured - p, Log(q.inverse() * q_measured)]`. The quaternion uses its shortest
arc, so `q` and `-q` have identical effects. Rotation residuals use the body's
right tangent; the position residual stays in the reference frame.

Before the joint update, covariance is transformed with
`B = diag(I, R_world_body.transpose())`: `R_tangent = B * R_input * B.transpose()`.
Both position-attitude cross blocks are retained. The observation Jacobian
selects position and attitude error states. The existing Joseph covariance
update and SO(3) error-injection/reset machinery are shared with the other
measurements. This is a local EKF linearization, not a globally convergent pose
solver; initialization must put the filter in a useful local neighborhood.

The NIS gate uses the full 6×6 innovation covariance. An invalid measurement or
a gated outlier leaves the state and covariance unchanged. Gating rejection is
a successful no-op for replay, so an older rejected event does not invalidate
later IMU processing.

## Timing and diagnostics

Use sensor timestamps and the same clock for IMU, localization and TF. With
measurement replay enabled, delayed poses rewind the filter within its retained
history and repropagate to the latest IMU. Slightly future poses can queue until
the IMU catches up; farther-future and too-old poses follow the existing replay
rejection rules. Frame TF must already be available at that sensor timestamp.
With replay disabled the measurement applies to the current state without
latency compensation.

Startup-queued observations older than the first IMU anchor are rejected once
that anchor is known; queuing before IMU arrival does not bypass the history
boundary. Observations at the first anchor are still valid.

The typed measurement-quality topic identifies this input as `source_id=8`,
`source="pose_6d"`. NIS has six dimensions; `innovation_magnitude` reports only
the position-residual norm in metres (angles are not added to metres). Quality
is reported once at the event's first evaluation, including a future-queued
event when it is eventually evaluated. Later replays re-evaluate the update but
do not multiply its diagnostic counter. The initial quality decision is not a
claim that every subsequent historical re-evaluation made the same decision.

Typed replay-timing messages use the same source ID. A replay status of
`applied` describes processing; inspect measurement quality to distinguish
accepted fusion from a NIS-gated no-op.

## Freshness watchdog

When the pose topic is enabled, a 100 ms wall timer publishes status even if
IMU callbacks stop. Age is measured in the node's **ROS clock** from the last
accepted pose sensor stamp. Use `use_sim_time` and `/clock` consistently during
simulation/bag replay; pausing simulated time also pauses age progression.
Backward clock jumps require reinitialization rather than silently accepting
data on the previous timeline.

Status/diagnostic headers carry the assessment's current ROS time; filter and
IMU sample times remain explicit status fields. A long IMU gap exceeding
`max_imu_dt_sec` is still rejected by the existing propagation/replay policy.
Reinitialize before resuming after such a gap: the watchdog cannot reconstruct
missing inertial data. A Pose-only outage with continuous valid IMU resumes
normal mode on a fresh accepted pose without reinitialization.

`pose_measurement_timeout_sec` defaults to 1.0 and must be finite and positive.
`EstimatorStatus` exposes `pose_input_enabled`, `pose_available`, `pose_age_sec`
and `received_pose_measurements`. Age is NaN before the first accepted pose;
disabled inputs do not trigger this watchdog. Receive counters include rejected
and pre-initialization callbacks.

Before the first accepted pose, initialized filters report `waiting_for_pose_input`.
Stale accepted data reports `pose_input_stale`, `HEALTH_DEGRADED` and `MODE_OUTAGE`.
Pose outage overrides the stationary mode, so stopping the vehicle cannot hide
it; an existing sensor/numerical FAULT keeps its more severe status and reason.
A fresh accepted pose restores normal mode selection. Invalid,
gated or duplicate observations do not advance freshness; old replayed stamps
cannot move the latest accepted stamp forward. Freshness is committed only
after the corresponding filter processing succeeds, not on receipt alone.
Reinitialization clears the accepted-pose timestamp.

This is freshness monitoring, not bounded sensor quarantine or a complete
deployment health policy. Those integration requirements remain separate.
The new status fields change the custom ROS message type hash; rebuild clients
of `kalman_filter_localization_msgs` together with the estimator.

## Verification

`test_pose_update` checks the analytic 6D gain with correlated covariance,
world-to-body tangent conversion under tilt, quaternion sign equivalence,
rejection without state mutation, and delayed replay against chronological
full-state/covariance results. It also checks future-pose queuing against the
chronological result. `test_pose_input_ros` launches the actual node
twice, converts a tilted/raised body pose from a rotated and translated reference
frame, checks preserved covariance cross terms, and compares chronological and
delayed delivery. It also exercises a gated outlier, invalid covariance,
unavailable reference TF and zero quaternion.

After building and sourcing the workspace:

```bash
colcon test --packages-select kalman_filter_localization_core \
  --ctest-args -R test_pose_update
colcon test --packages-select kalman_filter_localization \
  --ctest-args -R test_pose_input_ros
colcon test-result --verbose
```

Validation on 2026-10-24 (ROS 2 Jazzy): 365 tests, 0 errors, 0 failures and
32 skipped. All six pose-update core tests and the extended ROS pose-input test
executed successfully, including freshness expiration without IMU callbacks,
duplicate rejection, reset, and Pose-only outage recovery. Skipped checks are
not counted as functional coverage.
