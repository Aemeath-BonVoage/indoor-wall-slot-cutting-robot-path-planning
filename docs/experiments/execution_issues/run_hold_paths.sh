#!/usr/bin/env bash
# 第 6.4 节：定点保持控制律 p / dob 在单站任务、独立精对位与示教零位上的配对运行（种子 1–3）。
# 用法：source install/setup.bash && bash run_hold_paths.sh [运行名 ...]
# 运行名为 hold_<task|align>_<p|dob>_s<种子>；已有 summary.yaml 的运行跳过。
set -u
cd "$(dirname "$0")"
RUNS=("$@")
if [ ${#RUNS[@]} -eq 0 ]; then
  for s in 1 2 3; do
    RUNS+=(hold_task_p_s$s hold_task_dob_s$s hold_align_p_s$s hold_align_dob_s$s)
  done
fi
for r in "${RUNS[@]}"; do
  if [ -f "runs/$r/summary.yaml" ]; then
    echo "skip $r"
    continue
  fi
  IFS=_ read -r _ path mode seed <<< "$r"
  python3 hold_paths.py "$path" --hold "$mode" --seed "${seed#s}" --out "runs/$r"
done
