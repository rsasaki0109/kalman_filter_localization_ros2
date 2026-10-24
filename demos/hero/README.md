# Planar baseline (previous README hero)

The current README uses the [6DoF hero](../hero_6dof/README.md). This
planar baseline remains reproducible at `images/hero_localization_planar.gif`.

A seeded **synthetic illustration**, driven by the actual ROS 2 ESKF node.
It is not a real-world benchmark or an accuracy guarantee. The renderer reads
the recorded estimate CSV; it never draws ground truth as the estimator output.

The 52 s scenario contains a stationary start, smooth acceleration, an elliptical
lap, and a 16 s GNSS blackout (21–37 s). Inputs are 100 Hz IMU, 20 Hz wheel speed,
and 5 Hz noisy GNSS, with small nonzero IMU biases. Ground truth is used only for
the dashed reference and measured error. The 12 s animation compresses time;
the `t` label always shows scenario time. The last half-second dissolves back to
the opening frame for a loop. This visual transition is not estimator behavior.

## Reproduce with Docker

From the repository root, using the existing checkout:

```bash
# BUILDX_CONFIG is writable even in cloud machines with a read-only home.
BUILDX_CONFIG=/tmp/kfl-buildx docker build \
  -f demos/hero/Dockerfile -t kfl-hero:jazzy demos/hero

docker run --rm --mount "type=bind,src=$PWD,dst=/repo" \
  -e ROS_DOMAIN_ID=31 kfl-hero:jazzy \
  bash demos/hero/run.sh .data/hero
```

The image pins the official ROS 2 Jazzy base by digest. It adds Pillow, NumPy,
DejaVu fonts and FFmpeg using signed Ubuntu packages. No GPU, display server,
external dataset, access token, or running ROS service is required.
In managed cloud environments, preserve Docker's supplied proxy configuration.
The Dockerfile only needs the Ubuntu APT repositories and the image registry.

On an existing Jazzy development machine with these dependencies installed:

```bash
bash demos/hero/run.sh .data/hero
```

Output directories must be new. To keep an old run, choose another path, such as
`.data/hero-2`. The script refuses to overwrite one and incrementally rebuilds
the existing colcon workspace. It also checks the simulated motion using finite
differences before replaying inputs through the estimator.

## Artifacts and validation

Each run retains these artifacts beneath its output directory:

| Path | Content |
|---|---|
| `scenario.json` | Seed, rates, outage interval, biases, expected input counts |
| `input_bag/`, `reference.csv`, `gnss.csv` | Reproducible synthetic inputs and reference |
| `results/configs/` | Exact merged estimator configuration |
| `results/wheel_nhc_fixed/` | Recorded estimate bag/CSV, input counts and process logs |
| `results/manifest.json` | Repository/runtime provenance and artifact checksums |
| `evaluation.json` | Existing evaluator's unaligned 3D/horizontal/yaw error results |
| `render/metrics.json` | Measured outage errors and checksums of render source data |
| `render/hero.gif`, `render/hero.mp4` | 960 × 560, 16 fps, 12 s animation |
| `render/poster.png`, `render/frames/` | Still image and individual frames |

Readiness checks require complete ROS input receipt, at least 99% reference
matching, overall 3D RMSE below 3 m, recorded output spanning the entire lap with
no gap over 50 ms, and maximum horizontal outage error below 3 m. These are
demo regression limits, not production localization requirements. Timestamp
interpolation is used for drawing and error evaluation; axes stay fixed across
the entire clip. GNSS dots show only observations already received in the last
6 s. The sensor badge describes input availability, not the node's health topic.

To run the repository's existing suite independently:

```bash
docker run --rm --mount "type=bind,src=$PWD,dst=/repo" kfl-hero:jazzy \
  bash -lc 'set -eo pipefail;
    source /opt/ros/jazzy/setup.bash; source install/setup.bash;
    colcon test --parallel-workers 2 --event-handlers console_direct+ &&
    colcon test-result --verbose'
```

Review the still and the animation before updating the README asset. Publishing
the checked run is an explicit copy, so an experimental rerun cannot silently
replace it:

```bash
cp .data/hero/render/hero.gif images/hero_localization_planar.gif
```

The README caption keeps the synthetic-data disclosure visible without requiring
the reader to open this document. The MP4 and intermediate frames stay in
ignored `.data/` rather than increasing repository size.
