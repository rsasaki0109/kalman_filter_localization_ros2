# Synthetic 6DoF hero

The README now uses the [real-data hero](../real_data_hero/README.md).
This synthetic scenario remains reproducible at `images/hero_localization.gif`.

A **seeded synthetic illustration** using the installed Autoware integration
launch and the actual ROS 2 ESKF output. This is not a real-data benchmark,
production accuracy guarantee, or full Autoware/Nav2 navigation demonstration.
The previous [planar baseline](../hero/README.md) remains reproducible.

## Scenario

One 48 s climbing, banked lap, with a stationary tilted start and smooth
acceleration. Its height spans 15.59 m, bank reaches 15.88°, and pitch reaches
14.93°. All three coordinates share the same visual scale and a fixed camera.
The road/reference is synthetic ground truth; the mint trail and rigid vehicle
use recorded estimator position **and quaternion**, including roll and pitch.

- 100 Hz IMU with independent sample noise, small nonzero gyro/accelerometer
  biases and a non-identity 3-axis mounting rotation.
- 20 Hz source-frame forward speed. Lateral/vertical values and unobserved
  covariance entries are NaN, exercising partial-axis fusion without introducing
  zero-velocity constraints.
- 5 Hz full-covariance absolute poses with independent position and right/body
  tangent orientation perturbations. Their covariance is converted to the ROS
  fixed-world-axis convention.
- Absolute-pose samples are withheld over `[18, 30)` s. Pose messages otherwise
  arrive 150 ms late and exercise the real rewind/replay path. The final delayed
  samples arrive after the final IMU sample.
- Flat-ground, IMU orientation, nonholonomic, wheel and legacy GNSS/odometry
  observation modes are disabled. Height/attitude are not clamped for rendering.

The absolute-pose badge describes input availability, not the node's health.
Recent purple dots are only past observations (not future measurements).
Ground-truth reference curves may show the full course as context. Error is
unaligned 3D Euclidean position error and SO(3) geodesic attitude error, using
all 4,801 output samples at their exact sensor timestamps. Renderer output
never substitutes ground truth for an estimate or smooths a measured error.

## Reproduce

From the repository root, on a Jazzy host with NumPy, Pillow, FFmpeg and DejaVu
fonts, or using the same pinned image as the planar demo:

```bash
BUILDX_CONFIG=/tmp/kfl-buildx docker build \
  -f demos/hero/Dockerfile -t kfl-hero:jazzy demos/hero

docker run --rm --mount "type=bind,src=$PWD,dst=/repo" \
  -e ROS_DOMAIN_ID=46 kfl-hero:jazzy \
  bash demos/hero_6dof/run.sh .data/hero-6dof
```

On an existing development machine:

```bash
bash demos/hero_6dof/run.sh .data/hero-6dof
```

Use a **new output directory** for every run. No GPU, display, external dataset
or Autoware/Nav2 installation is needed. The script builds the workspace, checks
analytic kinematics by independent finite differences, starts `autoware.launch.py`,
seeds/activates the node, streams the inputs, validates actual receipt/output and
renders only after regression gates pass. Small differences from DDS callback
interleaving are possible; seeded sensor inputs are reproducible, while outputs
are validated by numerical tolerances rather than byte equality.

## Artifacts

| File | Content |
|---|---|
| `recording/` | ROS bag of published inputs, clock/static calibration and actual Odometry output |
| `reference.csv` | Timestamped reference position/quaternion/Euler angles |
| `estimate.csv` | Actual ROS output at the same timestamps |
| `observations.csv` | Noisy absolute-pose positions for the plot |
| `measurement_quality.csv` | Actual first-evaluation acceptance/NIS/reason per measurement |
| `parameters.yaml` | Selected profile with scenario overrides |
| `runtime_parameters.json` | Effective parameters queried from the running node |
| `scenario.json` | Counts, seed/noise/delay, replay checks, launch command, source/config hashes, Git provenance |
| `estimator.log` | Process log, complete receipt and replay counters |
| `render/metrics.json` | Evaluated errors and source/renderer checksums |
| `render/hero.gif`, `render/hero.mp4` | 1120 × 700, 20 fps, 14 s animation |
| `render/poster.png`, `render/frames/` | A banked-turn still and all source frames |

The rosbag retains arrival/recording times separately from sensor header stamps,
including delayed pose delivery. The driver supplies transient-local static TF
and initializes/activates the estimator; replay the scenario with the driver,
rather than expecting a bare `ros2 bag play` to perform those service calls.
The last 0.4 s of the animation dissolves to the opening frame for a loop. This
transition is an editorial effect, not estimator motion. Source data and metrics
stay unchanged; the `t` label shows scenario time during the replay.

## Validation and published run

Readiness requires complete IMU/Pose/Twist receipt, exactly 4,801 timestamp-matched
finite normalized output poses without a missing sample, at least one rewind,
zero numerical failures, 3D RMSE ≤ 1 m, maximum 3D error ≤ 3 m and maximum
geodesic attitude error ≤ 2°. These are regression limits for this synthetic
scenario, not navigation acceptance criteria. The physics tests independently
check acceleration, body gyro, forward speed, quaternion/rotation agreement,
stationary start, spatial excitation and lap closure.

The published run's measured results are recorded in
[published_metrics.json](published_metrics.json), with hashes for the source
CSV/manifest and final GIF/MP4. Full run artifacts stay in ignored `.data/`.

| Metric | Published synthetic run |
|---|---|
| Position 3D RMSE | 0.542 m |
| Maximum position 3D error (including outage) | 2.255 m |
| Maximum geodesic attitude error | 0.629° |
| Complete output poses | 4,801 / 4,801 |
| Actual rewind/replay operations | 1,142 |
| Numerical failures | 0 |
The 48 s replay is compressed to 14 s for the README; runtime speed depends on
DDS/executor interleaving and is not inferred from animation speed.

Inspect the stills and animation before publishing another run:

```bash
cp .data/hero-6dof/render/hero.gif images/hero_localization.gif
```

The shared estimator/integration package suite last reported 365 tests with
0 errors, 0 failures and 32 skips. This demo adds two physical-scenario tests
and a complete real-node scenario run; skipped checks are not functional
coverage. Full navigation stack operation and real-data holdouts remain separate
requirements before general localization performance claims.
