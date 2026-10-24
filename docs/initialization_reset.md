# Initialization and reset

Initialization describes the pose of `robot_frame_id` in the message's reference
frame. It does not infer a camera/LiDAR/antenna lever arm from a pose message
without a child-frame ID. The complete request is validated before changing
the active state, covariance, prediction history, sensor latches or freshness.

## Covariance modes

```yaml
ekf_localization:
  ros__parameters:
    initial_pose_covariance_topic: /initialpose
    initial_pose_covariance_mode: full
```

`initial_pose_covariance_mode` has two values:

| Mode | Behavior |
|---|---|
| `diagonal` (default) | Keep the six diagonal entries; replace non-positive/non-finite entries with configured initial variances. Cross terms are ignored, preserving the legacy input policy. |
| `full` | Require a finite symmetric, positive-definite, well-conditioned 6×6 covariance and preserve all position/attitude cross terms. Invalid covariance rejects the whole request. |

Both modes reject invalid positions, zero/non-finite/unnormalizable quaternions,
empty frames and missing reference transforms. When the message reference
differs from `reference_frame_id`, the mean and covariance are transformed at
the seed's timestamp. The deterministic coordinate conversion uses `diag(R,R)`;
extrinsic/reference-transform uncertainty is not inferred.

Covariance ordering is `[x,y,z,rotation_x,rotation_y,rotation_z]` about fixed
reference-frame axes, not Euler-angle coordinates. The core converts the
complete covariance to its right/body attitude tangent using the initialized
orientation. The world-pose covariance exported at the first primed IMU therefore
recovers the transformed input, including its correlations.

Legacy `PoseStamped` initialization remains available at
`/<node>/initial_pose`. It uses the configured initial pose variances in
`reference_frame_id`, while its pose mean is still transformed if necessary.
The diagonal fallback for covariance-bearing messages is interpreted in the
message's reference axes before coordinate conversion. Explicit configured zero
variances remain supported in the legacy path; `full` does not silently treat
unknown zero covariance as confidence.

A valid seed sets nominal velocity and IMU biases to zero, with their configured
initial variances. It clears the old IMU integration/sample cache, main replay
and smoother history, relative-odometry baseline, stationarity/fault windows,
wheel-scale sample window, TF output queue, and accepted-Pose freshness. Sensor
noise/model parameters, the configured map/datum and current wheel scale factor
are retained. The state waits for a post-initialization IMU before advertising
initialization or publishing pose/twist/odometry. New absolute-pose aiding must
also arrive to restore the configured Pose freshness health condition.

An invalid request leaves a running estimator intact; it does not partially
reset the covariance or erase its time base. The main and smoothing tracks are
prepared before either is committed.

## Explicit reset service

```bash
ros2 service call /ekf_localization/reset std_srvs/srv/Trigger '{}'
```

This is this package's reset API. It preserves the current activation gate; see
[stack launch/activation](stack_launch.md) for the separate SetBool interface.
The node name determines the prefix. A successful response deactivates filtering/output until a new
valid seed and IMU arrive. Status/diagnostics continue when their existing timer
is active. The service clears internal reorder, replay and TF output queues,
dynamic TF lookup history (retaining static calibration transforms), the reorder
time barrier/session counters, sensor latches and freshness. Lifetime received
and measurement-quality counters remain cumulative.

While waiting for the seed after an explicit reset, sensor packets cannot
establish a new reorder watermark. A valid seed is processed directly in this
state, even when reordering is enabled. Later sensor packets older than the last
initialization stamp are ignored. This allows a coordinated ROS clock rewind
to establish a new epoch without reusing internally queued observations from
the previous epoch. It does not flush external publishers or repair inconsistent
clocks; coordinate publishers and `/clock` when restarting a dataset.

A normal initialization message in an active buffered stream respects sensor
ordering and retains post-seed buffered inputs. Use the explicit reset service
when a full internal queue flush or clock-epoch change is intended.

## Verification

`test_partial_initialization` checks covariance round trips under tilt, cleared
prediction history, rejection without mutation, and the configured-zero legacy
policy. `test_initialization_ros` launches the real node in full and diagonal
modes. It checks transformed covariance, invalid request rejection during active
output, accepted-Pose freshness reset, a nonempty reorder-queue flush, no output
while deactivated, ignored pre-seed traffic and a new earlier clock epoch with
different dynamic reference TF data.

Validation on 2026-10-24 (ROS 2 Jazzy): all three packages built; 365 tests,
0 errors, 0 failures and 32 skipped. All seven partial-velocity/initialization
core tests and both full/diagonal ROS initialization/reset cases executed.
Skipped checks are not counted as functional coverage.
