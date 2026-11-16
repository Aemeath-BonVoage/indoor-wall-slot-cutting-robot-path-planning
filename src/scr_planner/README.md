# scr_planner

墙面开槽（暗敷电线管）路径规划：方向感知、按整段直线搜索的增强 A*。

- 搜索的每一步是“一整段横 / 竖直线 + 一次转弯”，状态为 `(x, y, 上一段方向)`。
- 目标函数：Σ(轴向权重 + 代价权重 × 格代价) + `turn_penalty` × 弯数。在下列约束下**可证最优**：
  - 某个方向整体禁止（承重墙不得开横槽）；
  - 单段直线最大长度（非承重墙横槽 ≤ 50 cm）；
  - 弯曲半径 r：与弯相邻的直线段 ≥ r（两弯之间 ≥ 2r），并且半径 r 的弯管圆弧不碰障碍。
- C++ 核心不依赖 ROS；提供 C 接口（`include/scr_planner/c_api.h`）和 Python ctypes 封装（`python/scr_planner_py.py`）。
- `python/wall_rules.py`：开槽规范常量（逐条标注国标 / 参照 / 行业 / 假设）、规则 → 栅格与参数、独立的合规检查、随机墙面场景生成。

## 参数（`SearchParams` / Python 关键字参数同名）

| 参数 | 默认 | 说明 |
|---|---|---|
| `turn_penalty` | 4.0 | 每个弯折算的格数 |
| `weight_x` / `weight_y` | 1.0 | 横 / 竖每格代价（非承重墙取 `weight_x = 2`，竖槽优先） |
| `allow_x` / `allow_y` | true | 是否允许横 / 竖向（承重墙 `allow_x = false`） |
| `max_run_x` / `max_run_y` | 0 | 单段最大长度（格），0 = 不限 |
| `bend_radius` | 0 | 弯曲半径（格），0 = 不考虑 |
| `cost_weight` | 0.0 | 每格附加 `cost_weight × cost / 252`（用于“电上水下”等软偏好） |
| `allow_unknown` / `obstacle_threshold` | true / 253 | 255 是否可通行；≥ 阈值的格不可通行 |
| `goal_tolerance_cells` | 0 | 终点被占时在此半径内找最近可通行格 |
| `timeout_s` / `max_expansions` | 0 / 0 | 超时 / 扩展上限，0 = 不限 |

## 开槽任务：离线规划 → 逐站执行

```bash
# 离线规划（不需要 ROS）：槽线、停靠站（按执行顺序）、每站机械臂点、预测时间
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_planner/config/wall_task_room3_south.yaml /tmp/plan.yaml
# Gazebo 中执行（只读规划文件；--dry-run-gap 为空切，仿真墙是刚体）
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run --dry-run-gap=0.01
```

- 切割只沿直角折线：每刀只动一个轴，刀片 0° / 90°；弯曲半径只作规划约束。
- 线盒高度须在机械臂可切范围内（离地约 0.27–0.80 m，任务约束）。
- 停靠站划分 `--partition=dp|greedy|per_slot`，访问顺序 `--order=sweep|slot_by_slot`。

## 测试与实验

```bash
colcon build --packages-select scr_planner && colcon test --packages-select scr_planner
PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/wall_benchmark.py          # 2a：规范对比
PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/station_order_benchmark.py  # 2b：停靠站顺序优化
```

报告：[`docs/experiments/cpp_planner`](../../docs/experiments/cpp_planner/README.md)（规划算法与规范）、
[`docs/experiments/slot_execution`](../../docs/experiments/slot_execution/README.md)（离线规划、多站执行、顺序优化）。
