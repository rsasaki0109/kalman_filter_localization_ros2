#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
# Offline replay after fetch_odaiba.py and the pinned RTKLIB source checkout.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
source /opt/ros/jazzy/setup.bash
data_root="$(realpath "${1:?Provide a data directory containing source/}")"
rtklib_source="$(realpath "${2:?Provide the pinned RTKLIB source checkout}")"
for entry in prepared results render; do
  if [[ -e "$data_root/$entry" ]]; then
    echo "Refusing to overwrite $data_root/$entry" >&2
    exit 1
  fi
done
[[ "$(git -c safe.directory="$rtklib_source" -C "$rtklib_source" rev-parse HEAD)" == 71db0ffa0d9735697c6adfd06fdf766d0e5ce807 ]]
git config --global --add safe.directory "$repo_root"
make -C "$rtklib_source/app/rnx2rtkp/gcc" -j2
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --parallel-workers 2
source install/setup.bash
python3 -m unittest discover -s demos/real_data_hero -p 'test_*.py' -v
cat > "$data_root/rtklib.conf" <<'EOF'
pos1-posmode       =kinematic
pos1-frequency     =l1+l2
pos1-soltype       =forward
pos1-elmask        =15
pos1-navsys        =1
pos2-armode        =continuous
pos2-arthres       =3
EOF
"$rtklib_source/app/rnx2rtkp/gcc/rnx2rtkp" -k "$data_root/rtklib.conf" \
  -ts 2018/12/19 03:56:15 -te 2018/12/19 03:59:15 -ti 0.2 -t -s , \
  -r -3961904.4341 3348994.2660 3698211.7067 \
  -o "$data_root/gnss.pos" "$data_root/source/rover_trimble.obs" \
  "$data_root/source/base_trimble.obs" "$data_root/source/base.nav" \
  > "$data_root/rtklib.log" 2>&1
python3 demos/real_data_hero/prepare.py --source "$data_root/source" \
  --position "$data_root/gnss.pos" --output "$data_root/prepared"
python3 - "$data_root" <<'PY'
from pathlib import Path
import sys
import yaml
base=yaml.safe_load(Path('kalman_filter_localization_ros2/param/profiles/urbannav_tokyo_tuned.yaml').read_text())
extra=yaml.safe_load(Path('demos/real_data_hero/profile.yaml').read_text())
base['ekf_localization']['ros__parameters'].update(extra['ekf_localization']['ros__parameters'])
Path(sys.argv[1],'parameters.yaml').write_text(yaml.safe_dump(base))
PY
python3 kalman_filter_localization_ros2/scripts/run_urbannav_ablation.py \
  --input-bag "$data_root/prepared/input_bag" \
  --reference-csv "$data_root/prepared/reference.csv" --output-dir "$data_root/results" \
  --base-profile "$data_root/parameters.yaml" \
  --profiles-dir kalman_filter_localization_ros2/param/profiles --profile wheel \
  --rate 10 --startup-delay 2 --drain-timeout-sec 120 --require-complete-input
python3 demos/real_data_hero/extract.py --bag "$data_root/results/wheel/estimate_bag" \
  --output "$data_root/prepared/estimate.csv"
python3 demos/real_data_hero/render.py --data "$data_root" --output "$data_root/render"
