#!/usr/bin/env bash
# 第 8 节的 Gazebo 验证（G1–G8 × 种子 1、2，另加两个标记的 G10、G11）；约 5 min / 次。
# 用法：source install/setup.bash && bash run_gazebo.sh [运行名 ...]
# 运行名为 <配置>_s<种子> 或 <配置>_pass2_s<种子>；数据包写到 ~/scr_bags/tag_map_val/。
# 同一台机器上只允许一个 Gazebo：每次运行前结束遗留的 gzserver。
# 一个停靠站都没有执行的运行（精对位阶段技能服务器收不到激光 / 里程计，见报告“Gazebo 验证”
# 中的剔除记录）移到 gazebo_runs_discarded/<运行名>_noexec<k> 后重跑，最多 MAX_TRY 次。
set -u
cd "$(dirname "$0")"
EXP=../along_wall_accuracy/run_experiment.py
OUT=gazebo_runs
DISC=gazebo_runs_discarded
BAGS=~/scr_bags/tag_map_val
ISO=(--domain 49 --gazebo-port 11353)
MAX_TRY=3
RUNS=("$@")
if [ ${#RUNS[@]} -eq 0 ]; then
  RUNS=()
  for s in 1 2; do
    RUNS+=(slam_err5_s$s slam_err5_pass2_s$s known_err5_s$s slam_ok_s$s slam_err3_s$s
           slam_err3_pass2_s$s slam_err5_noscale_s$s tags_0.6_noodomerr_s$s
           slam2_err5_s$s slam2_err5_noscale_s$s)
  done
fi
mkdir -p "$OUT" "$DISC" "$BAGS"
for r in "${RUNS[@]}"; do
  seed=${r##*_s}
  cfg=${r%_s*}
  extra=()
  if [[ $cfg == *_pass2 ]]; then
    cfg=${cfg%_pass2}
    prior="$OUT/${cfg}_s${seed}/tag_map_estimate.yaml"
    if [ ! -f "$prior" ]; then
      echo "skip $r: no first-pass estimate $prior"
      continue
    fi
    extra+=(--prior-map "$prior")
  fi
  if [[ $cfg == tags_0.6_noodomerr ]]; then
    extra+=(--launch-arg odom_error:=false)
  fi
  for k in $(seq 1 $MAX_TRY); do
    if [ -f "$OUT/$r/summary.yaml" ]; then
      echo "skip $r"
      break
    fi
    pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; sleep 2
    python3 "$EXP" "$cfg" --seed "$seed" --out "$OUT/$r" --bag "$BAGS/$r" "${ISO[@]}" "${extra[@]}"
    if [ -f "$OUT/$r/summary.yaml" ] && ! grep -q '^stations_done: 0$' "$OUT/$r/summary.yaml"; then
      break
    fi
    n=1
    while [ -e "$DISC/${r}_noexec$n" ]; do n=$((n + 1)); done
    rm -f "$OUT/$r"/{tool_trace.csv,arm_state.csv,launch.log,bag.log}
    mv "$OUT/$r" "$DISC/${r}_noexec$n"
    rm -rf "$BAGS/$r"
    echo "$r: no station executed, moved to $DISC/${r}_noexec$n"
  done
done
