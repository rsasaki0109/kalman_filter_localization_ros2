# Autoware and Nav2 launch integration

These launch files start the real 6DoF estimator and configure its interfaces.
They do not start Autoware, Nav2, sensor drivers, a map server, or a local
odometry provider. Full stack navigation is still an integration gate.

## Autoware

```bash
ros2 launch kalman_filter_localization autoware.launch.py \
  namespace:=localization/pose_twist_fusion_filter \
  imu_topic:=/sensing/imu/imu_data
```

The topic/service argument names and defaults follow
[the pinned Autoware EKF launch](https://github.com/autowarefoundation/autoware_core/blob/e9ce57bb47ebad82642f09594b8e97712d1893c8/localization/autoware_ekf_localizer/launch/ekf_localizer.launch.xml).
Relative names resolve under `namespace`; absolute names remain absolute.
Frame IDs are independent of the ROS namespace.

| Argument | Default | Message/service type |
|---|---|---|
| `input_initial_pose_name` | `initialpose3d` | `PoseWithCovarianceStamped` |
| `input_pose_with_cov_name` | `in_pose_with_covariance` | `PoseWithCovarianceStamped` |
| `input_twist_with_cov_name` | `in_twist_with_covariance` | `TwistWithCovarianceStamped` |
| `input_trigger_node_service_name` | `trigger_node` | `std_srvs/SetBool` |
| `output_pose_name` | `ekf_pose` | `PoseStamped` |
| `output_pose_with_covariance_name` | `ekf_pose_with_covariance` | `PoseWithCovarianceStamped` |
| `output_twist_name` | `ekf_twist` | `TwistStamped` |
| `output_twist_with_covariance_name` | `ekf_twist_with_covariance` | `TwistWithCovarianceStamped` |
| `output_odom_name` | `ekf_odom` | `nav_msgs/Odometry` |
| `diagnostics_topic` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

The node starts **inactive**. Supply a valid full-covariance initial pose and
activate it, in either order, then deliver a new IMU sample:

```bash
ros2 service call /localization/pose_twist_fusion_filter/trigger_node \
  std_srvs/srv/SetBool '{data: true}'
```

The input IMU is an additional requirement of this inertial replacement.
The default profile selects only source-frame Twist `x`, appropriate for a
forward-speed observation; unknown lateral/vertical values are not zero-speed
constraints. See [Twist measurement semantics](twist_measurements.md).

This covers the principal Pose/Twist/Odometry and activation interfaces.
Autoware's yaw-bias/biased-pose/debug outputs, diagnostic names and thresholds,
planar tuning parameters, QoS parity and full pipeline operation are not yet
compatible. Angular Twist values are used for offset-origin transport, not
independent angular-rate fusion. Do not substitute the Autoware parameter file
for this package's `param_file`.

## Activation and reset semantics

Without these launches the service is `<namespace>/ekf_localization/trigger_node_srv`
and the `start_active` parameter defaults to `true` for existing callers.
`EstimatorStatus.active` reports the gate separately from `initialized`.

- `false`: stop sensor processing and state/TF publication, discard the seed,
  clear replay/reorder/output-TF queues and dynamic TF lookup history, retain
  static calibration transforms, and report `summary: inactive`.
- While inactive, valid initialization messages are accepted directly. Other
  sensor packets cannot establish a replay/reorder watermark or become pending
  measurements. Status/diagnostics remain available through their configured timers.
- `true`: enable processing. A valid seed received while inactive is preserved;
  initialization still requires a new IMU. With no seed, pre-seed sensors are ignored.
  Repeated `true` requests preserve the active filter and pending work. This
  idempotent behavior intentionally differs from the pinned Autoware implementation,
  which clears its measurement queues and prediction time on every `true` request.
- The existing `std_srvs/Trigger` reset clears the session and waits for a new
  seed/IMU without changing `active`. Resetting an inactive node does not activate it.

Requests run in the estimator's existing callback group. A successful response
means the gate changed; it does not certify initialization, freshness or health.
Old messages already delivered to downstream DDS subscribers cannot be retracted.
See [initialization/reset](initialization_reset.md) for clock-epoch coordination.

## Nav2 global localization

```bash
ros2 launch kalman_filter_localization nav2.launch.py \
  input_initial_pose_name:=/initialpose \
  input_pose_with_cov_name:=/global_pose \
  input_twist_with_cov_name:=/local_velocity \
  imu_topic:=/imu/data
```

This starts active and publishes **map → odom**. An independent continuous local
odometry provider must own **odom → base_link** and supply Nav2 controller
odometry. The global `ekf_odom` output has frame `map`; it must not replace the
controller's continuous local odometry input. Stop AMCL or any other competing
map-to-odom publisher. Configure Nav2 to consume the same frames and local
odometry topic; this launch does not modify Nav2 parameters or lifecycle state.

Global TF is composed only when the local TF is available at the global pose's
sensor timestamp. Missing local TF suppresses global TF output while global
Pose/Odometry messages can still publish. This launch creates no static TF:
provide calibrated sensor mounting transforms from the robot description or
external broadcasters. ROS clock, sensor timestamps and local TF must share an epoch.

Both launches support:

- `param_file`: this package's `ekf_localization.ros__parameters` profile, layered
  over `param/ekf.yaml` and loaded independently of the runtime namespace.
  Default: `param/profiles/autoware_6dof_forward.yaml`. Use the full-vector
  `autoware_6dof.yaml` only if all three linear velocity axes are observed.
- `map_frame`, `odom_frame`, `base_frame`: default `map`, `odom`, `base_link`.
- `use_sim_time`: default `false`.
- `start_active`: `auto` (Autoware false, Nav2 true), `true`, or `false`.
- `publish_tf`: `auto` (Autoware false, Nav2 true), `true`, or `false`.
  Autoware's enabled mode publishes map-to-base; ensure exclusive ownership.

Topic/frame/clock/activation/TF launch arguments override the selected profile.
Sensor noise, extrinsics and observation covariance still require calibration.
A 6DoF estimate does not make Nav2's standard costmaps/planners operate in 3D.

## Verification

`test_integration_launch_ros` starts both installed launch entry points in a
namespace, with custom input/output names and frame IDs. It checks applied
profile values, all five output types/timestamps, tilted initialization,
activation before/after seed, ignored inactive IMU traffic, queued-input flushing,
invalid reinitialization, new-seed recovery and idempotent activation. The Nav2
case delays local TF and reconstructs the full body pose from map-to-odom and
odom-to-base, checking that no competing map-to-base edge is published.

Validation on 2026-10-24 (ROS 2 Jazzy): all three packages built; `colcon
test-result` reported 365 tests, 0 errors, 0 failures and 32 skipped. Both stack
launch cases executed successfully; skipped checks are not functional coverage.
