#!/usr/bin/env bash
# 机械臂轨迹实验的批量运行（逐个串行；每次约 4–6 min 墙钟）。
# 用法：source install/setup.bash && bash run_matrix.sh <规划: joint|rule|dp11> <种子> <配置名> [<配置名> ...]
#   BAG=1 时同时录制 rosbag2 到 ~/scr_bags/arm_trajectory/<运行名>（仓库之外）
# 运行名为 <配置名>_<规划>_s<种子>，结果在 runs/<运行名>/；已有 summary.yaml 的运行跳过。
set -u
cd "$(dirname "$0")"
declare -A PLAN=(
  [joint]=../plan_validation/plans/plan_joint.yaml
  [rule]=../plan_validation/plans/plan_rule.yaml
  [dp11]=../slot_execution/runs/dp/slot_plan.yaml
)
# 配置名 → 相对默认值的 arm_motion.* 覆盖（一次只改一类参数）
# 配置名 → 相对默认值的 arm_motion.* 覆盖。参数扫描（进给、加速度、空行程速度、加加速度、
# 沿墙轴加速度）在确定下刀后等待时间之前完成，都在 dwell = 0 下进行，一次只改一类参数
declare -A SETS=(
  [p2p]="mode=point_to_point"
  [cont]=""
  [dwell_0.0]="dwell=0.0"
  [dwell_0.1]="dwell=0.1"
  [dwell_0.2]="dwell=0.2"
  [dwell_0.4]="dwell=0.4"
  [feed_0.02]="dwell=0.0 feed_rate=0.02"
  [feed_0.10]="dwell=0.0 feed_rate=0.1"
  [feed_0.20]="dwell=0.0 feed_rate=0.2"
  [acc_x0.5]="dwell=0.0 rapid_accel=0.5 feed_accel=0.25 rot_accel=6.0"
  [acc_x2]="dwell=0.0 rapid_accel=2.0 feed_accel=1.0 rot_accel=24.0"
  [acc_x4]="dwell=0.0 rapid_accel=4.0 feed_accel=2.0 rot_accel=48.0"
  [rapid_0.10]="dwell=0.0 rapid_speed=0.1"
  [rapid_0.50]="dwell=0.0 rapid_speed=0.5"
  [trapezoid]="dwell=0.0 jerk_time=0.0"
  [jerk_0.10]="dwell=0.0 jerk_time=0.1"
  [lat_0.5]="dwell=0.0 rapid_accel_lateral=0.5"
  [lat_0.25]="dwell=0.0 rapid_accel_lateral=0.25"
  [fast]="dwell=0.0 feed_rate=0.2 rapid_speed=0.5 rapid_accel=4.0 feed_accel=2.0 rot_accel=48.0"
)
plan="$1"; seed="$2"; shift 2
for c in "$@"; do
  name="${c}_${plan}_s${seed}"
  if [ -f "runs/${name}/summary.yaml" ]; then
    echo "skip ${name}"
    continue
  fi
  args=()
  for kv in ${SETS[$c]}; do args+=(--set "$kv"); done
  if [ "${BAG:-0}" = 1 ]; then args+=(--bag "$HOME/scr_bags/arm_trajectory/${name}"); fi
  python3 run_experiment.py "${PLAN[$plan]}" "runs/${name}" --seed "$seed" "${args[@]}"
  [ -f "runs/${name}/tool_trace.csv" ] && gzip -f "runs/${name}/tool_trace.csv"
done
