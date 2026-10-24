# Real-data hero: UrbanNav Odaiba

Actual public **UrbanNav-TK-20181219 Odaiba** IMU, wheel-speed and GNSS
observations, replayed through this repository's real ROS 2 ESKF. The GIF shows
recorded output, not a simulated route. This is a regression/demo sequence,
not the registered Shinjuku blind holdout or a full Autoware/Nav2 navigation run.

Source: [UrbanNav official dataset](https://github.com/IPNL-POLYU/UrbanNavDataset#tokyo-dataset).
The dataset and linked downloads are MIT licensed according to the official
README and LICENSE at commit `e53717a40c94803cd479a140433f6d803b1a0876`.
[The copyright and permission notice](URBANNAV_LICENSE.txt) is retained here.
Raw sensor files are not redistributed in the repository.

## Fixed experiment

The interval is the **first 180 seconds from the initial IMU timestamp**, chosen
before inspecting estimator errors. No warmup is cut away to improve the score.
The actual final IMU timestamp is slightly before the 180 s boundary.

- TAG264 IMU at approximately 50 Hz and measured wheel velocity at approximately
  50 Hz, converted from source FRD to ROS FLU as `x, -y, -z`.
- GNSS positions calculated from the measured Trimble rover/base RINEX files and
  navigation messages, using RTKLIB commit `71db0ffa0d9735697c6adfd06fdf766d0e5ce807`.
  Processing is GPS-only, L1+L2, forward kinematic, 15° elevation mask, continuous
  ambiguity resolution with threshold 3, sampled at 0.2 s intervals. Base ECEF
  coordinates come from the supplied base RINEX header. `run.sh` records the
  exact command and configuration; these are not reference-derived GNSS positions.
- Applanix reference at 10 Hz. Initial attitude is reference seeded; position is
  zero at the first reference's ENU origin. The reference supplies **no recurring
  estimator observations**. Absolute position is compared without translation,
  yaw or trajectory alignment. Sensor lever arms are not corrected.
- The existing `urbannav_tokyo_tuned.yaml` and `research_wheel.yaml` regression
  settings are used with [explicit overrides](profile.yaml): flat ground, NHC
  and IMU-orientation observations are disabled, forward wheel speed is enabled.
  Existing GNSS-derived velocity/course-yaw aids remain enabled. This is not a
  claim of GNSS-independent attitude estimation.
- GPS week/TOW timestamps are kept on a shared continuous GPS-seconds axis;
  they are not represented as Unix UTC timestamps. Reference yaw is
  `90° - heading`, pitch changes sign and roll retains its sign in ENU/FLU.

The Tokyo release does not supply complete inter-sensor extrinsic calibration.
The demo assumes identity IMU-to-body mounting after axis conversion. The
reported position and attitude errors therefore include possible lever-arm,
mounting and reference-frame effects. This experiment does not establish
calibrated 6DoF accuracy, production performance or independent holdout results.

## What the animation means

The fixed-scale ENU view uses a plain metric grid; it does not invent streets
or display an unlicensed basemap. The gray reference shows the full route for
context. Mint is recorded estimator output, purple points are previously
available GNSS solutions, and the navigation marker uses recorded yaw. The
attitude instrument uses recorded roll/pitch; the height plot uses recorded z.

No artificial observation outage is injected. RTKLIB produced solution gaps of
5.0, 4.0 and 3.8 seconds in this interval. The amber bands mark inter-solution
gaps longer than 0.5 s. The GNSS badge becomes GAP when no position solution has
arrived for more than 0.5 s; it is **not** a receiver-hardware outage or estimator
health diagnosis. Solutions can be absent even while raw receiver observations
exist. The error chart draws ESKF in amber and GNSS samples in purple.

Position error is 3D Euclidean error. Quaternion error is the SO(3) geodesic angle,
using normalized interpolation along the shortest quaternion sign path between
reference samples. The first five outputs cannot be bracketed by the reference
and are explicitly excluded from error evaluation; they are retained in the bag
and CSV. Values are not clamped to make missing reference coverage appear valid.

The 180 s replay is compressed into 16 s. The final 0.4 s dissolves to the opening
frame for looping; this is an editorial transition, not estimator motion.

## Reproduce

Use a new, writable output directory and the existing `kfl-hero:jazzy` image
([image setup](../hero_6dof/README.md)). No GPU or display is needed.
From the repository root, download only the Odaiba navigation members of the
4.45 GB official ZIP. The range reader requires exact HTTP 206 bounds and ZIP
CRC verification, and records each downloaded member's SHA-256. It does not
extract or inspect Shinjuku sensor/reference content or LiDAR bags.

```bash
real_data_root="$PWD/.data/odaiba_real"
python3 demos/real_data_hero/fetch_odaiba.py --output "$real_data_root/source"
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/tomojitakasu/RTKLIB.git "$real_data_root/RTKLIB"
git -C "$real_data_root/RTKLIB" sparse-checkout set src app/rnx2rtkp
git -C "$real_data_root/RTKLIB" checkout 71db0ffa0d9735697c6adfd06fdf766d0e5ce807

docker run --rm --mount "type=bind,src=$PWD,dst=/repo" \
  --mount "type=bind,src=$real_data_root,dst=/realdata" \
  -e ROS_DOMAIN_ID=48 kfl-hero:jazzy \
  bash demos/real_data_hero/run.sh /realdata /realdata/RTKLIB
```

If the remote default branch has advanced, fetch the pinned commit before
checkout; do not silently use a newer RTKLIB revision. In managed cloud machines
use a writable directory such as `/workspace/hero-setup/odaiba-real` when a
restored `.data/` directory is owned by another user. The supplied image already
contains build/runtime dependencies; the container replay step is offline.

`run.sh` builds the pinned RTKLIB utility and this workspace, runs two focused
integrity/interpolation tests, computes GNSS solutions, converts raw measurements,
launches/records/drains the real estimator with complete-receipt checks, exports
full output quaternions and renders the animation. It refuses to overwrite
prepared, result or render directories. For rendering an existing checked run:

```bash
python3 demos/real_data_hero/render.py --data /path/to/run \
  --output /path/to/new-render
```

## Evidence retained per run

| Path | Evidence |
|---|---|
| `source/download_manifest.json` | Official ZIP URL/size, transferred bytes, member CRCs and SHA-256 |
| `rtklib.conf`, `rtklib.log`, `gnss.pos` | GNSS solver settings, progress and measured-data solutions |
| `prepared/scenario.json` | Fixed interval, frame/initialization assumptions, counts and input hashes |
| `prepared/input_bag` | Real IMU/wheel/GNSS inputs plus explicitly reference-seeded initialization |
| `prepared/reference.csv`, `observations.csv`, `estimate.csv` | Raw-converted reference, GNSS positions and actual ROS output |
| `results/configs/wheel.yaml` | Exact merged estimator parameters |
| `results/manifest.json`, `results/wheel/*.log` | Runtime/repository provenance, complete receipt/drain checks |
| `results/wheel/estimate_bag` | Original recorded PoseStamped output, including full quaternion |
| `render/metrics.json` | All evaluated errors, coverage/exclusions, gap intervals and source hashes |
| `render/hero.gif`, `hero.mp4`, `poster.png`, `frames/` | 1120×700, 20 fps, 16 s media and inspectable frames |

The published run contains 8,999 IMU inputs, 8,999 wheel inputs, 838 GNSS
solutions and 8,999 output poses, with zero late/buffered events at completion.
Two independent replays produced identical output CSV hashes.

| Metric | First-180-second real-data regression |
|---|---|
| Position 3D RMSE | 1.194 m |
| Maximum position 3D error | 5.024 m |
| Attitude geodesic RMSE | 2.750° |
| Maximum attitude geodesic error | 10.771° |
| Reference-matched outputs | 8,994 / 8,999 (five endpoint samples excluded) |
| Longest GNSS solution gap | 5.0 s |

Two focused range-integrity/quaternion-interpolation tests passed, and all three
packages rebuilt. The production estimator code is unchanged from the previous
365-test suite result (0 errors, 0 failures, 32 skipped); that full suite was not
rerun merely for this demo. The actual complete-input/output replay is additional
functional validation.

The published [measured results](published_metrics.json) include all error
statistics, including attitude error, and final-media checksums. GNSS RMSE uses
GNSS solution epochs; ESKF RMSE uses IMU output epochs. Those scalar metrics are
not a common-time ranking or a claimed improvement percentage.

Inspect the run and its attribution before updating the real-data asset:

```bash
cp /path/to/run/render/hero.gif images/hero_localization_real.gif
```

The [synthetic 6DoF scenario](../hero_6dof/README.md) and
[planar baseline](../hero/README.md) remain separately reproducible.
