# scr_interfaces

任务层的消息与 action 定义：技能服务器（`scr_arm_controller/skill_server.py`）提供的技能 action，以及行为树执行器（`scr_task`）对外的任务 action。

## Action

| 名称 | 服务器 | 作用 |
|---|---|---|
| `FineAlign` | `/task_skills/fine_align` | 精对位到定位标记正前方 `standoff` 处，确定墙面坐标原点；结果含估计误差与原点残差 |
| `MoveToStation` | `/task_skills/move_to_station` | 墙面坐标闭环横移 / 接近到沿墙 `lateral_offset`、离墙 `standoff`；收敛或残差可补偿时成功，结束后进入定点保持 |
| `CutStation` | `/task_skills/cut_station` | 测到站残差、补偿机械臂点、逐点执行；单站任务（`station = -1`）另做可达性检查、缩回与空切后移 |
| `HoldBase` | `/task_skills/hold_base` | 定点保持，直到目标被取消 |
| `ScanMarker` | `/task_skills/scan_marker` | 相机射线与激光墙面求交检测定位标记 |
| `RetractArm` | `/task_skills/retract_arm` | 机械臂缩回到接近位姿 |
| `ExecuteSlotPlan` | `/execute_slot_plan` | 执行离线开槽规划文件（行为树）；结果含每站记录 `StationRecord[]` |
| `ExecuteCuttingTask` | `/execute_cutting_task` | 执行单站任务文件（行为树） |

## 消息

- `ArmPoint`：base_link 系机械臂点 `(x, y, z, rotation)` 与是否处于切深 `at_depth`。
- `TaskStatus`：技能服务器的当前阶段（`align / move / arrived / cut / ...`）与停靠站序号，发布在 `/task_skills/status`，实验记录器据此标注真值轨迹。
- `StationRecord`：一个停靠站的执行记录（耗时、到站残差、补偿钳制点数、恢复次数等），由执行器在每站结束时发布在 `/task_executor/station_records`。

字段含义见各 `.msg` / `.action` 文件中的注释。
