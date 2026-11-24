#!/usr/bin/env bash
# 导航段对比的批量运行（逐个串行，每次约 10 min 墙钟）。
# 用法：source install/setup.bash && [STRESS=3] bash runs_nav.sh <配置名> <种子> [<种子> ...]
#   配置名只用于结果目录 runs/<配置名>_s<种子>；Nav2 参数取当时的 nav2_params.yaml。
set -u
cd "$(dirname "$0")"
cfg="$1"; shift
for seed in "$@"; do
  out="runs/${cfg}_s${seed}"
  if [ -f "${out}/legs.csv" ]; then echo "skip ${out}"; continue; fi
  python3 nav_legs.py --out "${out}" --seed "${seed}" --rounds 6 --stress "${STRESS:-0}"
done
