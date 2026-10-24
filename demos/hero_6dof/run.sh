#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
data_dir="$(realpath -m "${1:-.data/hero-6dof}")"
if [[ -e "$data_dir" ]]; then
  echo "Refusing to overwrite an existing run: $data_dir" >&2
  exit 1
fi
CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
  colcon build --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
python3 -m unittest discover -s demos/hero_6dof -p 'test_*.py' -v
python3 demos/hero_6dof/simulate.py --output "$data_dir"
python3 demos/hero_6dof/render.py --data "$data_dir" --output "$data_dir/render"
echo "Verified 6DoF GIF: $data_dir/render/hero.gif"
echo "Source bag, CSVs, effective parameters, metrics and MP4: $data_dir"
