# scr_planner

方向感知增强 A* —— Nav2 全局规划器插件 `scr_planner/EnhancedAStarPlanner`。

- 搜索状态 `(x, y, θ)`，4 邻域，代价 = 步数（可按代价地图加权）+ `turn_penalty` × 转弯次数，启发 = 曼哈顿 + `turn_penalty` × 剩余最少转弯数 → 路径横平竖直且该代价**可证最优**。
- 输出：正交拐点按代价地图分辨率加密的稠密路径，每段严格轴向，中间位姿朝向 = 行进方向，末位姿 = 目标朝向。
- 核心 `OrientedAStar`（`include/scr_planner/oriented_astar.hpp`）不依赖 ROS，可单独测试与基准。

## 参数（`<planner>.xxx`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `turn_penalty` | 2.5 | 每次转弯折合的栅格步数 |
| `use_cost_weight` | true | 是否按代价地图加权 |
| `cost_weight` | 1.0 | 步长代价 = 1 + `cost_weight` × cost / 252 |
| `allow_unknown` | true | 未知格是否可通行 |
| `tolerance` | 0.5 | 目标在障碍内时在此半径 (m) 内找最近可通行格 |
| `max_planning_time` | 2.0 | 超时 (s)，≤ 0 不限 |
| `max_iterations` | 0 | 扩展状态数上限，0 不限 |
| `use_final_approach_orientation` | false | true 时末位姿朝向取最后一段行进方向 |

致命（254）与内切膨胀（253）格不可通行；起点格本身豁免。所有参数支持运行时动态修改。

## 使用

```bash
ros2 launch scr_bringup sim_nav.launch.py \
    params_file:=$(ros2 pkg prefix scr_navigation)/share/scr_navigation/config/nav2_params_scr_planner.yaml
```

测试与对比实验见 [`docs/experiments/cpp_planner`](../../docs/experiments/cpp_planner/README.md)。
