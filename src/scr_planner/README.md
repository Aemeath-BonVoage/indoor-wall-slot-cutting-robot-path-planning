# scr_planner

墙面开槽（暗敷电线管）路径规划：C++ 方向感知、按整段直线搜索的增强 A\*，施工规范作为硬约束；以及多条槽的停靠站划分、访问顺序优化、槽线与停靠站的任务级联合规划，和多站执行的实验记录。底盘导航不使用本规划器，由 Nav2 完成。

## 功能

- **`OrientedAStar`（C++，不依赖 ROS）**：每一步走一整段横 / 竖直线，下一段必须转 90°，状态为 `(x, y, 上一段方向)`。目标函数为 Σ(轴向权重 + 代价权重 × 格代价) + `turn_penalty` × 弯数，在下列约束下可证最优：
  - 禁用某个方向（承重墙不得开横槽）；
  - 单段直线最大长度（非承重墙横槽 ≤ 50 cm）；
  - 弯曲半径 r：与弯相邻的直线段 ≥ r（两弯之间 ≥ 2r），且半径 r 的弯管圆弧不碰障碍。
- **接口**：C 接口 `include/scr_planner/c_api.h`，Python ctypes 封装 `python/scr_planner_py.py`。
- **`python/wall_rules.py`**：开槽规范常量（逐条标注来源：国标 / 参照 / 行业 / 假设）、规则 → 栅格与搜索参数、独立的合规检查、随机墙面场景生成。
- **`python/slot_plan.py`**：开槽任务规划。槽线限制在机械臂可切高度带内；按机械臂每站 12 cm 的可切宽度划分停靠站（每条槽单独贪心 / 合并贪心 / 合并动态规划）；访问顺序（逐条槽 / 一维最优扫掠）；每站的机械臂切割点（每刀只动一个轴，刀片 0° / 90°）；由 Gazebo 执行记录标定的时间模型，标定结果保存为命名预设 `TIME_MODELS`（见下文）。
- **`python/task_plan.py`**：任务级联合规划。在分段 A\* 中加入按停靠站窗口划分的列代价，使拐点落在窗口边界上，搜索多个窗口偏移并保留规范代价最优的路线作候选，以预测执行时间为目标选择；多条槽互相阻挡时尝试其他规划顺序；时间下界证书；Lee 迷宫布线基线与独立的规范检查。
- **`python/wall_gen.py`**：随机住宅墙面生成器（承重 / 非承重、门窗、梁柱、常见点位高度、强弱电间距、1–4 条槽），由整数种子完全确定。

## 搜索参数（`SearchParams`，Python 关键字参数同名）

| 参数 | 默认 | 说明 |
|---|---|---|
| `turn_penalty` | 4.0 | 每个弯折算的格数 |
| `weight_x` / `weight_y` | 1.0 | 横 / 竖每格代价（非承重墙取 `weight_x = 2`，竖槽优先） |
| `allow_x` / `allow_y` | true | 是否允许横 / 竖向（承重墙 `allow_x = false`） |
| `max_run_x` / `max_run_y` | 0 | 单段最大长度（格），0 为不限 |
| `bend_radius` | 0 | 弯曲半径（格），0 为不考虑 |
| `cost_weight` | 0.0 | 每格附加 `cost_weight × cost / 252`（用于“电上水下”等软偏好） |
| `allow_unknown` / `obstacle_threshold` | true / 253 | 255 是否可通行；≥ 阈值的格不可通行 |
| `goal_tolerance_cells` | 0 | 终点被占时在此半径内找最近可通行格 |
| `timeout_s` / `max_expansions` | 0 / 0 | 超时 / 扩展上限，0 为不限 |

## 用法：离线规划 → 逐站执行

```bash
# 离线规划（不需要 ROS）：槽线、停靠站（按执行顺序）、每站机械臂点、预测时间
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_planner/config/wall_task_room3_south.yaml /tmp/plan.yaml \
    [--routes=rule|joint] [--partition=dp|greedy|per_slot] [--order=sweep|slot_by_slot] \
    [--time-model=traj_v1|align_v3|align_v2|align_v1]
# --routes=joint：槽线与停靠站联合规划（每面墙规划耗时中位约 0.4 s，最长约 5.5 s），输出格式不变
# --time-model：预测时间所用的预设，默认 traj_v1；也可在任务文件 options.time_model 中指定

# 执行（行为树执行器 /execute_slot_plan，需先 ros2 launch scr_bringup sim_nav.launch.py）；--dry-run-gap 为空切
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01

# 实验记录器：同一 action 的客户端 + Gazebo 真值评估，输出 stations.csv / tool_trace.csv / ekf_trace.csv / summary.yaml
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run --dry-run-gap=0.01 \
    [--no-compensation] [--max-strafe=1.5] [--bt=<tree.xml>] [--realign-distance=0.3] \
    [--marker-x=8.42 --marker-y=-3.9245]

# ROS 节点：规划并在 /wall_slot_plan/markers 发布 RViz 显示
ros2 run scr_planner wall_slot_planner_node.py --ros-args \
    -p task_file:=$(ros2 pkg prefix scr_planner)/share/scr_planner/config/wall_task_room3_south.yaml \
    -p output_file:=/tmp/slot_plan.yaml
```

墙面任务文件的格式见 `config/wall_task_room3_south.yaml` 和 [使用指南](../../docs/USER_GUIDE.md#7-多站开槽离线规划--逐站执行)。线盒高度须在机械臂可切范围内（离地约 0.27–0.80 m）。

`execute_slot_plan.py` 不含执行逻辑：执行由 `scr_task` 的行为树完成，记录器发送规划文件，订阅 Gazebo 真值、技能服务器的阶段话题 `/task_skills/status` 与墙面状态估计 `/task_skills/wall_estimate`，把刀尖轨迹、到站误差、底盘漂移与规划槽线比较。`summary.yaml` 与 `stations.csv` 的字段与早期版本相同，另增每站恢复次数 `recoveries`、是否按行程重新对位 `realigned`、到站时的无标记行程 `since_fix_m`、本站接受过更新的标记 `tags_used` / `tag_updates`，以及 `summary.yaml` 中的 `realignments`、`final_lateral_scale`、`final_sigma_scale`。`ekf_trace.csv` 逐周期记录估计（`d, u, ψ`、σ、横移比例、画面中的标记）与同一时刻的真值（以参考标记为原点，参考标记的 map 坐标由 `--marker-x/--marker-y` 给出）。`--realign-distance=<m>` 填写目标的按行程重新对位阈值（不给出时用执行器参数，负值关闭）。

## 时间模型

每站耗时 = `t_settle` + 横移距离 / `v_strafe` + 点数 × `t_point` + 切割长度 / `v_feed`（`slot_plan.TimeModel`）。机械臂逐点执行的预设中 `v_feed` 为无穷大，切割耗时已含在 `t_point` 内。标定结果保存为命名预设 `slot_plan.TIME_MODELS`，用 `TimeModel.preset(name)` 取用：

| 预设 | 标定数据 | 方法 | `t_settle` | `v_strafe` | `t_point` |
|---|---|---|---|---|---|
| `traj_v1`（默认，`TimeModel()`） | 当前默认执行路径：同 `align_v3`，机械臂为连续轨迹，`v_feed` = 0.05 m/s，4 次运行 44 站（[报告](../../docs/experiments/arm_trajectory/README.md)） | 分阶段，切割阶段对点数回归、进给取指令值（`traj`） | 4.77 s | 0.1036 m/s | 0.501 s |
| `align_v3` | 行为树 + 标记地图 + 横移比例估计 + 横移前回观测位姿，机械臂逐点执行，4 次运行 44 站 | 分阶段，切割阶段回归（`phases-reg`） | 4.70 s | 0.0971 m/s | 2.030 s |
| `align_v2` | 行为树改造之前、精对位第二版，3 次运行 33 站 | 分阶段估计（`phases`） | 2.83 s | 0.0984 m/s | 2.049 s |
| `align_v1` | 第一版精对位，4 次运行 54 站 | 整体最小二乘 | 4.40 s | 0.0945 m/s | 2.034 s |

`plan_wall_slots.py`、`task_planning_benchmark.py`、`task_planning_report.py` 接受 `--time-model=<预设>`。标定：

```bash
python3 src/scr_planner/scripts/calibrate_time_model.py <run_dir> [<run_dir> ...] \
    [--method=joint|phases|phases-reg|traj] [--feed=<m/s>] [--compare=align_v1,align_v2,align_v3] [--loo]
```

- `--method=joint`（默认）：对每站总耗时 `station_s` 做整体最小二乘，三个参数同时拟合；
- `--method=phases`：分阶段估计——横移阶段 `move_s = a + d / v_strafe` 回归，`t_point = Σ cut_s / Σ points`，固定开销取均值，`t_settle = a + 固定开销`；少数站上的偶发停顿只影响 `t_point` 的均值，而不会被归到横移距离上；
- `--method=phases-reg`：同上，但切割阶段也回归 `cut_s = c_cut + 点数 × t_point`，截距计入 `t_settle`。横移前机械臂回到观测位姿后，每站切割阶段多出约 1.0 s 与点数无关的固定开销（到站残差测量），比值估计会把它摊进 `t_point`（2.152 s，每站 RMSE 0.55 s），回归后为 2.030 s、0.22 s；
- `--compare`：在同一批数据上评估已有预设的每站 RMSE、平均偏差与各站耗时之和的误差；
- `--loo`：逐次运行留一交叉验证。

`align_v1`、`align_v2` 的数据来自早期配置（`/odom` 为真值），在当前默认路径上 `align_v2` 每次运行低估 18–21 s，只用于复现此前的报告；`align_v3` 的数据来自沿墙精度实验中默认配置的 4 次运行，在 5 次未参与标定的运行（含联合规划与规则路线）上各站之和的误差为 −1.7 … +0.7 s。执行路径或仿真配置改变后应用新的执行记录重新标定。标定、样本外检验与联合规划的 Gazebo 执行见 [plan_validation](../../docs/experiments/plan_validation/README.md)，标定方法的演变见 [技术报告](../../docs/report/README.md#64-执行时间模型的标定) 6.4 节。

其他脚本：`wall_benchmark.py`（规划器与施工规范的随机墙面对比）、`station_order_benchmark.py`（停靠站划分与顺序的离线对比）、`task_planning_benchmark.py` / `task_planning_report.py`（联合规划的批量基准与统计，见 [报告](../../docs/experiments/task_planning/README.md)）、`plot_execution.py` / `plot_station_order.py`（作图）。

## 测试

```bash
colcon build --packages-select scr_planner
colcon test --packages-select scr_planner && colcon test-result --verbose
```

- `test/test_oriented_astar.cpp`（gtest）：基本行为、各项规范约束，以及在 400 张随机栅格、随机规则组合上与独立实现的逐格 Dijkstra 比较最优值；
- `test/test_slot_plan.py`（pytest）：停靠站覆盖、动态规划划分不劣于贪心、扫掠顺序与暴力枚举一致、单轴切割、规划文件读写、规划文件的解析与补偿公式（`scr_arm_controller/slot_execution.py`）、时间模型预设。

实验报告：[开槽规划算法与施工规范](../../docs/experiments/cpp_planner/README.md)、[多站执行与停靠站顺序优化](../../docs/experiments/slot_execution/README.md)。
