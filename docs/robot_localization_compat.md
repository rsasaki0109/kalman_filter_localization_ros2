# robot_localization compatibility

The ROS 2 component can consume the two standard interfaces used by a
`robot_localization`/`navsat_transform_node` pipeline:

- `nav_msgs/msg/Odometry` as an absolute position observation;
- `geometry_msgs/msg/PoseWithCovarianceStamped` as an initial pose.

## Configuration

Start with
[`robot_localization_compat.yaml`](../kalman_filter_localization_ros2/param/profiles/robot_localization_compat.yaml)
and set the topic names and frames to match the upstream pipeline.

```yaml
odom_input_mode: "absolute"
odom_topic: "/odometry/gps"
use_odom: true
initial_pose_covariance_topic: "/initialpose"
```

`odom_input_mode: "relative"` remains the default and preserves the historical
behavior: consecutive odometry poses are converted to a relative displacement.
In absolute mode, the x/y/z block of `Odometry.pose.covariance` is used when it
is finite and positive-definite. `var_odom_xyz` is the fallback for missing,
zero, or invalid covariance.

By default, the covariance-bearing initial pose uses the diagonal entries for x/y/z and
roll/pitch/yaw. Non-positive or non-finite entries fall back independently to
`initial_position_variance_*` and `initial_attitude_variance_*`. The existing
`PoseStamped` initial-pose subscription remains available for legacy users.

Select `initial_pose_covariance_mode: full` to preserve a valid correlated 6×6
initial covariance. Initial poses are transformed to `reference_frame_id` at
their timestamps; invalid full covariance or missing TF rejects the request
without resetting a running estimator. See [initialization/reset](initialization_reset.md).

The odometry pose must already be expressed in `reference_frame_id`; kf_ws does
not silently reinterpret a robot_localization frame ID. Verify the upstream TF
tree and covariance semantics before deploying a profile.
