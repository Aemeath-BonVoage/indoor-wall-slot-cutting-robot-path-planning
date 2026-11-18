# scr_task

行为树任务执行器（BehaviorTree.CPP v3）。结构与 Nav2 的 `bt_navigator` 相同：每种任务是一个由行为树执行的 action（`nav2_behavior_tree::BtActionServer`），树中的技能节点是技能服务器 action 的客户端（`nav2_behavior_tree::BtActionNode`），导航直接使用 Nav2 的 `NavigateToPose` 节点。

| 任务 action | 行为树 | 流程 |
|---|---|---|
| `/execute_slot_plan` | [`behavior_trees/slot_plan.xml`](behavior_trees/slot_plan.xml) | 读规划 → 导航到定位标记前 → 精对位（确定墙面原点）→ 逐站：（按行程重新对位到期时先回参考标记重新精对位）→ 到站（远站先 Nav2）→ 定点保持下切割 → 记录 → 机械臂缩回 |
| `/execute_cutting_task` | [`behavior_trees/cutting_task.xml`](behavior_trees/cutting_task.xml) | 读任务 → 导航 → 精对位 → 定点保持下扫描与切割 |

恢复策略：导航失败重试一次；精对位失败则缩回机械臂、重新导航到标记前再对位；到站失败（未收敛且残差超出补偿范围）则回到标记前重新对位、重新建立墙面原点后再到站；切割失败重试一次。仍失败则任务失败，结果中的 `error` 给出失败的停靠站与节点（例如 `station S2 failed (CutStation aborted: arm motion failed)`）。

## 节点

| 节点 | 类型 | 说明 |
|---|---|---|
| `FineAlign` / `MoveToStation` / `CutStation` / `HoldBase` / `ScanMarker` / `RetractArm` | 动作 | 技能 action 客户端，端口与 `scr_interfaces` 的目标 / 结果字段对应 |
| `LoadSlotPlan` / `LoadCuttingTask` | 同步动作 | 读任务文件（`task_files.cpp`，规则与 Python 侧 `slot_execution.py` 相同） |
| `ForEachStation` | 装饰 | 逐站执行子树，每站开始前写出该站参数（`station_u`、`points`、`nav_leg` 等） |
| `RecordStation` | 同步动作 | 每站结束时追加 `StationRecord` |
| `CountRecovery` | 同步动作 | 恢复路径计数（本站 / 全程） |
| `CheckFlag` | 条件 | 布尔端口为 true 时成功 |
| `RealignDue` | 条件 | 按行程重新对位：`since_fix + |move_dist| > threshold`（`threshold ≤ 0` 关闭）时成功，判定结果写到 `due` |

`slot_plan.xml` 中的子树：`AlignToMarker`（精对位，失败时重新导航再对位）、`ReachStation`（到站）、`ScheduledRealign`（横移回参考标记前 → `AlignToMarker`）与 `ExecuteStation`（每站开始前由 `RealignDue` 决定是否先执行 `ScheduledRealign`）。黑板上的 `since_fix` 是自上次接受标记更新以来的沿墙里程计行程：`FineAlign` 成功时清零，`MoveToStation` 成功时取结果中的 `odom_since_fix`；`move_dist` 由 `ForEachStation` 给出。计划内的重新对位记入 `StationRecord.realigned`，不计入恢复次数。

## 用法

执行器由 `scr_bringup/sim_nav.launch.py` 启动（`task:=true`，默认），也可单独启动任务层：

```bash
ros2 launch scr_task task.launch.py [alignment_params:=<yaml>] [odom_topic:=/wheel/odom] [executor_params:=<yaml>]
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `alignment_params` | `scr_arm_controller/config/fine_alignment.yaml` | 技能服务器的精对位参数（`alignment.*`） |
| `odom_topic` | `/odom` | 技能局部闭环使用的里程计；融合定位时应为 `/wheel/odom`（`sim_nav.launch.py` 自动设置） |
| `executor_params` | `config/task_executor.yaml` | 行为树执行器参数 |

执行器先等待 Nav2（`bt_navigator` 激活）与技能服务器就绪，再加载行为树并激活。发送任务用命令行入口：

```bash
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
ros2 run scr_arm_controller navigation_cutting_task <plan.yaml> --dry-run-gap=0.01 [--bt=<tree.xml>]
```

执行器参数见 [`config/task_executor.yaml`](config/task_executor.yaml)：tick 周期与超时、行为树插件库、定位标记在 map 中的坐标 `marker_x / marker_y`、Nav2 目标离墙距离 `nav_standoff`（1.2 m）、按行程重新对位的阈值 `realign_distance`（米，默认 0 = 关闭；只在配置阶段读取）。默认行为树由参数 `slot_plan_bt`、`cutting_task_bt` 指定（缺省为本包 `behavior_trees/` 下的两个文件，未写在该 YAML 中）；单个任务可以用客户端的 `--bt=<tree.xml>` 换用其他树。多站任务的 `max_strafe` 为 0 时取 1.5 m，空切间隙为 0 时取 0.01 m，`realign_distance` 为 0 时取执行器参数、为负时本次关闭（实验记录器 `execute_slot_plan.py` 的 `--realign-distance=<m>` 填写该字段）。

按行程重新对位只消除随行程累积的随机误差；对里程计方向固定的横移比例误差无效（阈值 0.3 m 时最大沿墙误差 17.5 mm，不重新对位为 14.4 mm，耗时 +48 s），因此默认关闭，沿墙精度由技能服务器的标记地图与横移比例估计保证，见 [沿墙精度报告](../../docs/experiments/along_wall_accuracy/README.md)。

每站结束时执行器在 `/task_executor/station_records`（`scr_interfaces/StationRecord`）发布该站记录；任务 action 的反馈给出当前停靠站、阶段与恢复次数。

## 测试

```bash
colcon test --packages-select scr_task
```

- `test_task_files`：对照实际的 11 站规划文件检查停靠位姿、空切点与单站任务解析；
- `test_behavior_trees`：用同名、同端口的模拟技能节点运行两棵行为树，检查逐站执行、重新对位（恢复路径与按行程触发）、切割重试、Nav2 段与失败报告（不需要 ROS 通信）。
