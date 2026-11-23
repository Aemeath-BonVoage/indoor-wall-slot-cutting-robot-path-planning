#!/usr/bin/env bash
# 带切割力的整墙切割试验的批量运行（逐个串行）。
# 用法：source install/setup.bash && bash run_matrix.sh <规划: joint|rule> <种子> <配置名> [<配置名> ...]
#   BAG=1 时同时录制 rosbag2 到 ~/scr_bags/cutting_force/<运行名>（仓库之外）
# 运行名为 <配置名>_<规划>_s<种子>，结果在 runs/<运行名>/；已有 summary.yaml 的运行跳过。
set -u
cd "$(dirname "$0")"
declare -A PLAN=(
  [joint]=../plan_validation/plans/plan_joint.yaml
  [rule]=../plan_validation/plans/plan_rule.yaml
)
# 配置名 → run_station_test.py 的参数。烧结砖；进给 5.5 mm/s 是摩擦约束给出的上限
# （μ = 0.3，安全系数 1.5），8.8 mm/s 是安全系数 1.0 的上限；下刀 2.5 mm/s（主轴功率的 67 %）
F55="--feed 0.0055 --plunge 0.0025"
F88="--feed 0.0088 --plunge 0.0025"
declare -A SETS=(
  [dry_f5.5]="--no-force --hold p $F55"
  [p_f5.5]="--hold p $F55"
  [pid_f5.5]="--hold pid $F55"
  [dob_f5.5]="--hold dob $F55"
  [dobff_f5.5]="--hold dob --ff 0.7 $F55"
  [dob_f8.8]="--hold dob $F88"
  [dobff_f8.8]="--hold dob --ff 0.7 $F88"
  [dob_f5.5_step]="--hold dob $F55 --ramp 0 --no-split"
  [dobff_f5.5_step]="--hold dob --ff 0.7 $F55 --ramp 0 --no-split"
  [dob_f5.5_tool]="--hold dob $F55 --apply-to tool"
  [dobff_f5.5_tool]="--hold dob --ff 0.7 $F55 --apply-to tool"
  [dobff_limit]="--hold dob --ff 0.7 --force-limit"
)
plan="$1"; seed="$2"; shift 2
for c in "$@"; do
  name="${c}_${plan}_s${seed}"
  if [ -f "runs/${name}/summary.yaml" ]; then
    echo "skip ${name}"
    continue
  fi
  args=(${SETS[$c]})
  if [ "${BAG:-0}" = 1 ]; then args+=(--bag "$HOME/scr_bags/cutting_force/${name}"); fi
  python3 run_station_test.py "${PLAN[$plan]}" "runs/${name}" --seed "$seed" "${args[@]}"
  [ -f "runs/${name}/tool_trace.csv" ] && gzip -f "runs/${name}/tool_trace.csv"
  [ -f "runs/${name}/wrench.csv" ] && gzip -f "runs/${name}/wrench.csv"
  [ -f "runs/${name}/arm_state.csv" ] && gzip -f "runs/${name}/arm_state.csv"
  [ -f "runs/${name}/ekf_trace.csv" ] && gzip -f "runs/${name}/ekf_trace.csv"
done
