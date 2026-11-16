# 墙面开槽任务规划与多站执行（2b，进行中）

![执行结果](execution.png)

## 做了什么

- **规划**（`src/scr_planner/python/slot_plan.py`，节点 `wall_slot_planner_node.py`）：
  1. 读入墙面任务 YAML；
  2. 按开槽规范规划正交槽线（2a 的 C++ 核心），并把路线限制在机械臂够得到的高度带内；
  3. 在拐角处用 R12 cm 圆弧连接（15° 弦逼近）；
  4. 按机械臂横向行程划分最少数量的底盘停靠站（一维区间覆盖，贪心最优，pytest 中给出最优性证书）；
  5. 生成每站的机械臂切割点，格式与 `navigation_cutting_task` 的 `points` 相同；
  6. 写出 YAML，并在 `/wall_slot_plan/markers` 上发布 RViz 可视化。
- **执行**（`execute_slot_plan.py`，复用 `NavigationCuttingTask`，未修改它）：
  1. Nav2 导航到定位标记前；
  2. 视觉伺服对位，作为墙面坐标的参考原点；
  3. 沿墙横移到各站（横向用里程计，离墙距离与朝向用激光拟合墙面 + 指数平滑）；
  4. 每站切割期间用里程计**定点保持**；
  5. 用 Gazebo `link_states` 真值记录刀具轨迹。
- **测试**：19 个 gtest 和 11 个 pytest 全部通过。pytest 会校验机器人常数与 `cartesian_cutting.py`、`visual_servo_ros.py`、`task_at_target.yaml`、URDF 一致，并检查覆盖完整且最优、每个点都在关节限位内、点能反算回槽线。

## 关键发现

| 项目 | 数值 | 含义 |
|---|---|---|
| 机械臂可达高度（离地） | 0.27–0.80 m | 开关（1.3 m）、空调插座、从顶部引下的管线都够不到 → 需要更大的升降行程 |
| 每站可切宽度 | 12 cm（Y 行程 14 cm 减去对位余量） | 示例槽长 1.37 m、横向跨度 1.10 m，**需要 10 个停靠站**（理论最少） |
| 最大切深 | 41 mm | 目标槽深 35 mm（Φ20 + 15 mm）可以达到 |
| 刚体墙 + 下刀 | 机器人被顶开约 35 mm | Gazebo 无法仿真切削，所以采用**空切**：刀尖停在墙前 10 mm |
| 机械臂运动时底盘漂移 | 不保持时单站最多 11 cm；加里程计定点保持后最多 1.7 mm | 轮子无摩擦、底盘由 planar_move 驱动；**现有 `navigation_cutting_task` 很可能也受影响** |

## 结果（空切 + 定点保持，`summary.yaml`）

- 10 / 10 站全部完成；仿真时间 406 s；每站到位 5–70 s。
- 到位真值误差：
  - 横向平均 5.1 mm，基本是**系统偏差**，来自定位标记对位本身，即路线图中已有的“感知横向偏差 / 相机外参”问题；
  - 离墙距离 0.8 mm；
  - 朝向 0.52°。
- 刀具轨迹与规划槽线的偏差：平均 6.8 mm，95 分位 20 mm，覆盖率 51 %。
  - 只有横向直线的站（S1、S5、S9）偏差 0.1 mm；
  - 偏差集中在含竖段或圆弧的站（S2、S3、S7，沿墙方向 −4 到 −12 mm）。

## 未解决（下一步）

- **沿墙方向约 −13 mm 的残余偏移**：每一站的实测刀具轨迹都比窗口整体偏左约 18 mm，其中约 5 mm 是到位误差。
  - 剩下的部分怀疑是 `cartesian_cutting` 的 (x0, y0, z0) 参考点与 link4 原点在 Y 向不重合，或刀具相对 link4 有横向偏置。
  - 需要用 TF 查 base_link → link4 在 j2 = 0 时的 y 值来核对，再修正 `RobotModel.y0` 或加一个工具偏置。
  - 修正前，覆盖率与轨迹误差这两项数字**不可作为最终结果**。
- 圆弧段刀片角度的正负号是从 URDF（joint4 轴为 −x）推出来的，还没有实际验证。
- 停靠站位姿还没有与占用地图做碰撞检查。
- 每站耗时主要花在横移收敛上（最长 70 s），可以调参优化。

## 复现

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false
ros2 run scr_planner execute_slot_plan.py \
    $(ros2 pkg prefix scr_planner)/share/scr_planner/config/wall_task_room3_south.yaml /tmp/run --dry-run-gap=0.01
```
