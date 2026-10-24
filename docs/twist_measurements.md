# Linear Twist observations

The optional `geometry_msgs/TwistWithCovarianceStamped` input fuses selected
components of linear velocity with their correlated covariance. Angular
propagation uses the IMU; Twist angular rates are not fused as additional gyro
observations. This avoids directly reusing an IMU-derived angular rate as an
independent angular measurement. The error-state filter still estimates full
3D position, velocity, attitude and configured IMU biases.

## Configure

```yaml
ekf_localization:
  ros__parameters:
    robot_frame_id: base_link
    twist_measurement_topic: measured_twist_with_covariance
    twist_linear_axes: [x, y, z]
    max_twist_nis: 11.345
    enable_measurement_replay: true
    measurement_history_duration_sec: 2.0
```

The topic is disabled by default (`""`). `max_twist_nis` is nonnegative; zero
disables gating. The example is the approximate chi-square 99% threshold with
**three** degrees of freedom. Use the selected linear-axis count, not six:

| Observed source axes | NIS dimensions | Approximate 99% threshold |
|---|---|---|
| `[x]` | 1 | 6.635 |
| `[x, z]` | 2 | 9.210 |
| `[x, y, z]` (default) | 3 | 11.345 |

Empty lists, duplicate names and names outside `x/y/z` reject startup. The
[pose + Twist development profile](../kalman_filter_localization_ros2/param/profiles/autoware_6dof.yaml)
combines these inputs and IMU propagation. It requires calibrated sensor noise,
correct extrinsics and stack-specific remappings; activation/reset compatibility
and complete Autoware/Nav2 stack validation remain work in progress.

Only selected axes are observed. Unobserved linear values/covariance entries
may be unavailable (including NaN); they do not become zero-velocity constraints.
The active covariance must be finite, symmetric and positive definite. For a
forward-speed-only source, select `[x]` and a one-dimensional gate, as in the
[forward profile](../kalman_filter_localization_ros2/param/profiles/autoware_6dof_forward.yaml).
The existing `/wheel_speed` scalar model also remains available.

## Origin, axes and covariance

`header.frame_id` identifies a co-moving body/sensor frame whose **origin's
velocity** is described by the Twist. Inputs must be independent observations
of motion, not output from this same estimator. For a coincident source/body
origin, only selected linear values and covariance rows/columns are used. Unused
angular values and covariance may be unavailable and are not read as velocity
measurements.

For an offset source, TF is looked up as `body <- source` at the sensor stamp.
Let its rotation be `R` and its translation (source origin in body coordinates)
be `r`. Rigid-body transport is:

```text
v_body = R v_source + r × (R omega_source)
A      = [ R, skew(r) R ]
Cov(v_body) = A Cov(twist_source) A.transpose()
```

For partial axes, the equivalent observation is kept in source coordinates:
`S(v_source + R.transpose()*skew(r)*R*omega_source)`, predicted by
`S*R.transpose()*v_body`, where `S` selects the observed axes. This avoids
rotating an incomplete vector into invented body-axis measurements.

Offset transport uses the Twist's own angular values and covariance cross
terms. Only angular components that affect an observed linear component are
required; a zero transport coefficient has a `1e-12` numerical tolerance.
The covariance submatrix for selected linear and required angular components
must be finite, symmetric and positive definite. For example, a source-X linear
observation never needs source-X spin for rigid-body origin transport. Angular
components required for transport do not increase the NIS dimensions. Their values
are used to relocate the velocity observation; they do not directly update
gyro bias or attitude as independent gyro measurements.

The reference transform/extrinsics are treated as deterministic. This contract
does not describe a body-origin velocity merely expressed in arbitrary world
axes; convert that upstream instead of labeling it as the world origin's Twist.
It also does not account for unknown correlations between a Twist source and
the IMU propagation data. In particular, a source already fusing the same IMU
is not automatically an independent observation just because it has a different
topic name. Offset transport requires valid angular uncertainty, not unknown
zeros substituted as exact angular rates.

Missing TF, empty frames, invalid selected data/covariance and NIS-gated outliers
are rejected. No identity fallback or confident covariance replacement is used.
The standard body-velocity observation Jacobian accounts for attitude as well
as world velocity, so the correction preserves the state correlations. Joint
angular-rate fusion would need an explicit model of propagation/measurement
noise correlation and remains outside this input model.

## Timing and diagnostics

Replay handles delayed, duplicate, too-old and slightly future data using the
same machinery as the pose input. With replay disabled, the observation updates
the current state without latency compensation. Keep source timestamps and TF
on the same clock. Input QoS is reliable with `input_qos_depth` history.

Before the first IMU, observations may queue while the time anchor is unknown.
Once the first IMU establishes that anchor, queued observations older than it
are rejected, matching the normal too-old history rule. They are not silently
applied at the first IMU's timestamp.

Measurement quality identifies `source_id=9`, `source="twist_linear"`. Its NIS
dimension is the number of selected axes; `innovation_magnitude` is the selected residual norm in
m/s. The first main-filter evaluation is reported once. Replays still recompute
the update without multiplying the quality counters. A replay `applied` status
can include a NIS-gated no-op; inspect measurement quality for acceptance.

`EstimatorStatus.received_twist_measurements` counts subscription callbacks,
including inputs subsequently ignored or rejected. A dedicated Twist freshness
and quarantine policy is not implemented by this change. Pose freshness is
handled separately as documented in [pose measurements](pose_measurements.md).

## Verification

`test_twist_update` checks the correlated 3D gain under tilt, rigid-body origin
transport, covariance cross terms, rejection without mutation and delayed
replay. `test_twist_input_ros` launches the real node for chronological and
delayed delivery in two cases: coincident origins with unavailable angular
fields, and a rotated/offset sensor during three-axis rotation. It checks all
three recovered world-velocity components, unchanged IMU angular rates, state
and covariance equivalence, outlier rejection and invalid frame/covariance data.
Forward-only cases additionally check acceptance of unavailable unobserved data,
retained uncertainty in two unobserved directions, and delayed replay under
rotated sensor axes.

Validation on 2026-10-24 (ROS 2 Jazzy): 365 tests, 0 errors, 0 failures and
32 skipped. All five Twist core tests, all four ROS Twist cases, the extended Pose
watchdog/recovery test, partial-axis core tests, full/diagonal initialization
and reset cases, and the startup replay-boundary regression executed.
Skipped checks are not counted as functional coverage.
