#!/usr/bin/env bash
# 沿墙精度实验的全部运行（每个配置打滑种子 1、2 各一次；约 7 min / 次）。
# 用法：source install/setup.bash && bash run_matrix.sh [配置名 ...]
set -u
cd "$(dirname "$0")"
CONFIGS=("$@")
if [ ${#CONFIGS[@]} -eq 0 ]; then
  CONFIGS=(single_off single_on tags_0.6 scale_single tags_1.2 tags_0.3 scale_tags_1.2
           realign_0.3 tags_0.6_err3 single_on_noobs scale_tags_0.6)
fi
for c in "${CONFIGS[@]}"; do
  for s in 1 2; do
    if [ -f "runs/${c}_s${s}/summary.yaml" ]; then
      echo "skip ${c}_s${s}"
      continue
    fi
    python3 run_experiment.py "$c" --seed "$s"
  done
done
