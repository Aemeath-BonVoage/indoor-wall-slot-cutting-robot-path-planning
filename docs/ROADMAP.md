# 路线图

本文只列计划与进行中的工作。已完成的工作按阶段记录在 [项目演进](EVOLUTION.md)，详细数据见各实验报告。

每项工作的完成标准：在 Gazebo / RViz 中实际运行，并留下可复现的命令、原始数据和报告。

## 进行中与计划

### 1. 沿墙精度的后续工作

多标记地图 + 横移比例在线估计已把 11 站开槽的最大沿墙误差降到 1.5 mm（[报告](experiments/along_wall_accuracy/README.md)），留下的问题：

- [ ] 标记位置误差几乎一比一进入结果（±3 mm → 5.2 mm）：标记贴好后测量其实际位置，或在执行中用相邻标记的已知间距在线修正标记地图
- [ ] 不估计横移比例时残留 1–2 mm 偏差，与约 0.1° 的朝向估计误差同步，推测是未建模的横移误差泄漏到朝向估计中，需要单独的对照实验
- [ ] 无标记方案：墙角、门框等稀疏特征或 BIM 墙面模型配准，作为标记的补充（目前只有分析）
- [ ] 默认配置每站多出约 2 s（横移前回到观测位姿），按新执行路径重新标定时间模型

### 2. 规划结果的 Gazebo 验证与演示

- [ ] 在 Gazebo 中执行联合规划生成的示例任务，与时间模型的预测对比（`align_v2` 预测 208.9 s；默认配置改变后需先重新标定）
- [ ] 在 Gazebo 中验证墙面实体的运行时生成、演示相机出图和运行中的刀尖轨迹
- [ ] 录制完整多站开槽演示（GIF / MP4），放入 README

### 3. 任务级规划的后续工作

- [ ] 106 面墙（63 面为 4 条槽）没有任何方法给出可行方案：多方案决策，路线弯数过多或无解时给出改走吊顶 / 地面等替代方案
- [ ] 停靠站位姿与占用地图的碰撞检查
- [ ] 小规模实例用整数规划求精确最优解，核对下界证书未能证明最优的墙面

### 4. 仿真与系统

- [ ] robot_localization EKF 在 mm/s 级慢速运动下输出偏小的原因（目前局部闭环改用原始轮式里程计规避）
- [ ] 轮子驱动的底盘被控对象：速度跟踪 92–97 %、慢速横移仅 40 %，需要更好的轮地接触模型后才能替代力驱动被控对象
- [ ] 仿真器从 Gazebo Classic 迁移到新版 Gazebo（视机器资源而定）

### 5. 技术报告

- [ ] 随上述工作更新 [技术报告](report/README.md) 并重新生成 PDF

## 待处理

- [ ] 单条槽执行精度的数据（[多站执行报告](experiments/slot_execution/README.md)）在仿真配置 v1 上测得，待在 v2 上重测

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
| 系统层现代化：行为树任务编排、Nav2 Smac 2D + MPPI、编码器误差模型 + 陀螺 + EKF、底盘 ros2_control | [演进 10](EVOLUTION.md#10-系统层现代化)、[报告](experiments/system_modernization/README.md) |
| 多站沿墙精度：多标记地图、横移比例在线估计、按行程重新对位的对比 | [演进 11](EVOLUTION.md#11-多站沿墙精度标记地图与横移比例估计)、[报告](experiments/along_wall_accuracy/README.md) |
| 时间模型按精对位第二版重新标定（`align_v2`）；技术报告与英文摘要 | [演进 10](EVOLUTION.md#10-系统层现代化)、[技术报告](report/README.md) |
| PBVS / IBVS / 2.5D 混合伺服离线对比 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[报告](experiments/ibvs_vs_pbvs/README.md) |
| RViz 开槽可视化、墙面实体、演示录制工具 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[说明](experiments/demo/README.md) |

## 不再计划

| 项目 | 原因 |
|---|---|
| 与 Theta\* / JPS 等任意角度规划器对比 | 开槽只允许横平竖直的线段，任意角度规划器不是合适的基线；改为与逐条规划、第一版规划器对比 |
| 单独的 Nav2 参数调优 | 导航只负责把底盘送到墙前约 1–2 m 处，精度由精对位保证；当前参数下多站任务的导航段全部成功。Nav2 已换用 Smac 2D + MPPI，见 [系统层报告](experiments/system_modernization/README.md) |
