#!/usr/bin/env bash
# Run from the ROS-enabled host or the documented Docker image. No display needed.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if ! command -v colcon >/dev/null || ! command -v ffmpeg >/dev/null; then
  echo 'ROS 2 Jazzy, colcon, Pillow, NumPy, DejaVu fonts and FFmpeg are required; see demos/hero/README.md.' >&2
  exit 1
fi
data_dir="$(realpath -m "${1:-.data/hero}")"
if [[ -e "$data_dir" ]]; then
  echo "Refusing to overwrite an existing run: $data_dir" >&2
  exit 1
fi
CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
  colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
python3 -m unittest discover -s demos/hero -p 'test_*.py' -v
python3 demos/hero/generate.py --output "$data_dir"
python3 kalman_filter_localization_ros2/scripts/run_urbannav_ablation.py \
  --input-bag "$data_dir/input_bag" \
  --reference-csv "$data_dir/reference.csv" \
  --output-dir "$data_dir/results" \
  --base-profile demos/hero/profile.yaml \
  --profiles-dir kalman_filter_localization_ros2/param/profiles \
  --profile wheel_nhc_fixed --rate 5 --startup-delay 2 \
  --drain-timeout-sec 60 --require-complete-input
python3 kalman_filter_localization_ros2/scripts/evaluate_localization.py \
  --estimate-csv "$data_dir/results/wheel_nhc_fixed/estimate.csv" \
  --reference-csv "$data_dir/reference.csv" \
  --output-json "$data_dir/evaluation.json" \
  --max-rmse-3d 3.0 --min-match-ratio 0.99
python3 demos/hero/render.py --data "$data_dir" --output "$data_dir/render"
echo "Hero GIF: $data_dir/render/hero.gif"
echo "MP4, poster and source metrics: $data_dir/render/"
