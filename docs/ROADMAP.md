# 路线图

本文只列计划与进行中的工作。已完成的工作按阶段记录在 [项目演进](EVOLUTION.md)，详细数据见各实验报告。

每项工作的完成标准：在 Gazebo / RViz 中实际运行，并留下可复现的命令、原始数据和报告。

## 进行中与计划

### 1. ROS 2 系统层现代化

**动机**：任务流程目前由 `navigation_cutting_task.py` 中的一个顺序程序实现，单站任务、多站执行、重试与失败处理都写在同一个类里；导航使用 NavFn + DWB，定位只用里程计与 AMCL；底盘绕过 ros2_control 直接由平面运动插件驱动；Gazebo Classic 已于 2025 年 1 月停止维护。

- [ ] 任务编排改用 BehaviorTree.CPP，替换顺序执行的 `navigation_cutting_task.py`（与 Nav2 bt_navigator 相同的模式）
- [ ] Nav2：局部控制器由 DWB 换为 MPPI（全向运动模型），全局规划器由 NavFn 换为 Smac Planner；在 Gazebo 中评估
- [ ] 定位：仿真中加入 IMU，用 `robot_localization` EKF 融合轮式里程计与 IMU
- [ ] 底盘改由 ros2_control（麦克纳姆轮驱动控制器）控制，与机械臂一致
- [ ] 仿真器从 Gazebo Classic 迁移到新版 Gazebo（最后一项，视机器资源而定）

### 2. 任务级规划的后续工作

联合规划已完成（[报告](experiments/task_planning/README.md)），留下的问题：

- [ ] 在 Gazebo 中执行联合规划生成的示例任务，验证预测的 225.5 s
- [ ] 按精对位第二版的执行路径重新标定时间模型（每站移动与稳定约 4.5 s → 2.8 s）
- [ ] 106 面墙（63 面为 4 条槽）没有任何方法给出可行方案：多方案决策，路线弯数过多或无解时给出改走吊顶 / 地面等替代方案
- [ ] 停靠站位姿与占用地图的碰撞检查
- [ ] 小规模实例用整数规划求精确最优解，核对下界证书未能证明最优的墙面

### 3. Gazebo 演示录制

- [ ] 在 Gazebo 中验证墙面实体的运行时生成、演示相机出图和运行中的刀尖轨迹
- [ ] 录制完整多站开槽演示（GIF / MP4），放入 README

### 4. 技术报告

- [ ] 汇总系统设计、方法与实验结果，形成一份完整的技术报告

## 待处理

- [ ] 单条槽执行精度的数据（[多站执行报告](experiments/slot_execution/README.md)）在仿真配置 v1 上测得，待在 v2 上重测
- [ ] `mecanum_controller.py` 关闭时未处理中断信号，launch 报告进程非零退出（不影响实验）

## 已完成（摘要）

| 工作 | 记录 |
|---|---|
| 一键启动、轻量模式、依赖修复、CI | [演进 2](EVOLUTION.md#2-工程化与可测量) |
| 相机 + 激光 PBVS 精对位；对位鲁棒性实验 | [演进 3](EVOLUTION.md#3-精对位从里程计到视觉伺服)、[报告](experiments/alignment_robustness/README.md) |
| 主点偏差修正、刀尖安全余量、制动距离防碰撞守护 | [演进 3](EVOLUTION.md#3-精对位从里程计到视觉伺服) |
| 底盘旋转失真修复（仿真配置 v2）、定点保持、空切 | [演进 4](EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真) |
| C++ 方向感知增强 A\* 与施工规范约束 | [演进 5](EVOLUTION.md#5-墙面开槽规划)、[报告](experiments/cpp_planner/README.md) |
| 离线开槽规划、多站执行、停靠站顺序优化、执行路径统一 | [演进 6](EVOLUTION.md#6-从规划到执行多站开槽)、[报告](experiments/slot_execution/README.md) |
| 精对位第二版：AprilTag + 激光 EKF + 饱和控制律、示教零位、多站原点修正 | [演进 8](EVOLUTION.md#8-精对位第二版检测估计与控制分离)、[设计](experiments/fine_alignment/README.md)、[对比](experiments/alignment_robustness/README.md) |
| 任务级联合规划：随机住宅墙生成器、站感知选线、下界证书、Lee 布线基线 | [演进 9](EVOLUTION.md#9-任务级联合规划)、[报告](experiments/task_planning/README.md) |
| PBVS / IBVS / 2.5D 混合伺服离线对比 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[报告](experiments/ibvs_vs_pbvs/README.md) |
| RViz 开槽可视化、墙面实体、演示录制工具 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[说明](experiments/demo/README.md) |

## 不再计划

| 项目 | 原因 |
|---|---|
| 与 Theta\* / JPS 等任意角度规划器对比 | 开槽只允许横平竖直的线段，任意角度规划器不是合适的基线；改为与逐条规划、第一版规划器对比 |
| 单独的 Nav2 参数调优 | 导航只负责把底盘送到墙前约 1–2 m 处，精度由精对位保证；当前参数下多站任务的导航段全部成功。Nav2 方面的计划是换用 MPPI / Smac（见第 2 项），而不是调参 |
