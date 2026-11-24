# 路线图

本文只列计划与进行中的工作。已完成的工作按阶段记录在 [项目演进](EVOLUTION.md)，详细数据见各实验报告。

每项工作的完成标准：在 Gazebo / RViz 中实际运行，并留下可复现的命令、原始数据和报告。

## 进行中与计划

### 0. 切割过程

机械臂连续轨迹（[报告](experiments/arm_trajectory/README.md)）、切割力模型及其 Gazebo 验证、按力模型限制的进给（[报告](experiments/cutting_force/README.md)）已完成，留下的问题：

- [ ] 扰动观测器在默认空切任务上的对照已完成（4 对运行：耗时 +0.3 %，轨迹误差最大值下降，[报告](experiments/execution_issues/README.md#6-扰动观测器定点保持的空切对照)）；把 `hold.mode` 的默认值改为 `dob` 之前还缺单站任务、精对位与示教路径下的检查
- [ ] 放宽后的保持限幅（`base_max_ang_accel` = 75 rad/s²）对应 μ = 0.6，与进给上限所用的 μ = 0.3 不一致，应按同一个摩擦系数推出
- [ ] 带力切割时横槽刀尖偏移峰值 5–6 mm，约为离线预测的两倍：把切削力与底盘运动的耦合加入离线模型，并处理下刀（近零进给）时力模型对速度符号敏感的问题
- [ ] 一刀切完、切削力消失后的瞬态（峰值最大 14 mm），1 s 的等待没有完全消除
- [ ] 定点保持使用相对墙面的测量（激光 + 标记 + 陀螺）：轮子打滑时里程计看不到底盘的偏移
- [ ] 时间模型 `cut_brick_v1` 只在一面墙的两份规划上标定和检验，需要在另一面墙上核对
- [ ] 横向换位时机械臂的反作用使底盘偏航 0.4°，目前靠下刀后等待 0.2 s 衰减
- [ ] 单站任务仍是逐点执行；`joint_state_broadcaster` 实际以 1 kHz 发布（配置为 50 Hz）
- [ ] 切割力的数值多为假设（比切削能、法向 / 切向力比），需要对照文献全文或实测数据

### 1. 沿墙精度的后续工作

多标记地图 + 横移比例在线估计已把 11 站开槽的最大沿墙误差降到 1.5 mm（[报告](experiments/along_wall_accuracy/README.md)）；标记位置的在线估计已完成离线设计与评估（[报告](experiments/tag_map_estimation/README.md)），留下的问题：

- [ ] 在 Gazebo 中验证标记地图在线估计（`alignment.estimate_tag_map`）：运行表与预期值见报告第 8 节（16 次运行）
- [ ] 墙角作为地图原点的锚点：先用已录制数据包中的激光数据离线拟合墙角，再决定是否做 Gazebo 试验
- [ ] 回放所需的记录：估计器内部时间戳与原始检测话题（现有估计话题的时间戳只有 0.1 s 分辨率）
- [ ] 相机尺度误差与已知标记间距的利用（相机看到的间距偏短约 0.06 %）

### 2. 执行层的遗留问题

四个遗留问题已排查（[报告](experiments/execution_issues/README.md)）：导航到达停滞与 EKF 慢速偏差已修正，其余留下的问题：

- [ ] 切割阶段偶发的 1.4–3.0 s 停顿：动作响应样本丢失后在下一次 DDS 心跳时才补发。丢失的原因未查明（接收缓冲、图像话题的流量）；缩短心跳周期可以消除停顿，但对流量与处理器的影响还没有测
- [ ] Nav2 组件启动失败（155 次启动中 1 次）：组件容器没有处理加载请求，而加载等待没有超时；30 次冷启动未复现，未修正
- [ ] 导航到达停滞的修正依据的是日志证据，停滞本身出现频率约 1 %，修正后的频率需要在后续运行中累计
- [ ] 贴墙原地旋转时 AMCL 偶发约 0.45 m 的定位跳变（一次运行中观察到）
- [ ] `/clock` 只有 10 Hz 的副作用：按仿真时间计时的短超时、定时器成批触发

### 3. 任务级规划的后续工作

房间级规划与无合规方案墙面的替代方案已完成离线部分（[报告](experiments/room_planning/README.md)），留下的问题：

- [ ] 在 Gazebo 中执行一个房间任务：room3 的东墙、北墙需要布置定位标记并加入标记地图，任务层需要按墙切换入口标记并把多面墙串成一次任务
- [ ] 转场时间模型目前由 18 段导航记录拟合：补充 1–6 m 导航段与每面墙首次精对位的执行记录后重新标定
- [ ] 每面墙约 15 s 的首次精对位是转场中最大的一项：利用已有定位缩短它
- [ ] 跨墙角回路的引线竖槽可能使原本可行的墙变为无解（数据中有 3 面单槽墙），需要在分配时检查
- [ ] 仍无解的 4 面墙与多槽的完备性：协商式拥塞消解或整数规划
- [ ] 穿过门洞的多房间任务
- [ ] 停靠站位姿与占用地图的碰撞检查（房间级规划只检查了墙前站位是否可通行）
- [ ] 小规模实例用整数规划求精确最优解，核对下界证书未能证明最优的墙面

### 4. 仿真与系统

- [ ] 局部闭环仍用原始轮式里程计：EKF 慢速偏差已修正，但它的输出在仿真中只有 10 Hz，不适合 50 Hz 的定点保持
- [ ] 轮子驱动的底盘被控对象：速度跟踪 92–97 %、慢速横移仅 40 %，零指令、零外力下自身以约 25 mm/s 漂移，需要更好的轮地接触模型后才能替代力驱动被控对象
- [ ] 仿真器从 Gazebo Classic 迁移到新版 Gazebo（视机器资源而定）

### 5. 技术报告

- [ ] 随上述工作更新 [技术报告](report/README.md) 并重新生成 PDF

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
| 时间模型按当前执行路径重新标定（`align_v3`，样本外误差 ≤ 0.7 %）；联合规划在 Gazebo 中实测节省 7.3 %；多站开槽演示 GIF | [演进 12](EVOLUTION.md#12-规划预测的-gazebo-验证与演示)、[规划验证](experiments/plan_validation/README.md)、[演示](experiments/demo/README.md) |
| 机械臂连续轨迹（11 站 228 → 160 s）；时间模型增加切割长度项（`traj_v1`） | [演进 13](EVOLUTION.md#13-机械臂连续轨迹)、[报告](experiments/arm_trajectory/README.md) |
| 切割力模型、平台可行性、定点保持控制律的离线对比；切割力插件与抗扰保持（默认关闭） | [演进 14](EVOLUTION.md#14-切割力与抗扰定点保持离线分析)、[报告](experiments/cutting_force/README.md) |
| 切割力插件与定点保持控制律的 Gazebo 验证；按力模型限制的进给与时间模型 `cut_brick_v1`；400 面墙的规划对比 | [演进 15](EVOLUTION.md#15-切割力的-gazebo-验证与进给上限)、[报告](experiments/cutting_force/README.md#10-gazebo-验证) |
| 无合规方案墙面的诊断与替代方案（102 / 106）；房间级规划（子集动态规划） | [演进 16](EVOLUTION.md#16-从一面墙到一个房间)、[报告](experiments/room_planning/README.md) |
| 执行层问题排查：导航到达停滞（确认超时）与 EKF 慢速偏差（twist 取平均）修正；切割停顿定位；扰动观测器空切对照；单条槽精度复测 | [演进 17](EVOLUTION.md#17-执行层遗留问题的排查)、[报告](experiments/execution_issues/README.md) |
| 标记地图在线估计（EKF-SLAM，默认关闭）的离线设计与评估；剩余同号偏差的来源；墙角等自然特征的精度估算 | [演进 18](EVOLUTION.md#18-标记地图在线估计离线)、[报告](experiments/tag_map_estimation/README.md) |
| 英文 README 与技术概览；Docker 镜像与镜像检查工作流 | [English README](../README.en.md)、[使用指南 11](USER_GUIDE.md#11-docker-复现) |
| 时间模型按精对位第二版重新标定（`align_v2`）；技术报告与英文摘要 | [演进 10](EVOLUTION.md#10-系统层现代化)、[技术报告](report/README.md) |
| PBVS / IBVS / 2.5D 混合伺服离线对比 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[报告](experiments/ibvs_vs_pbvs/README.md) |
| RViz 开槽可视化、墙面实体、演示录制工具 | [演进 7](EVOLUTION.md#7-方法对比与可视化)、[说明](experiments/demo/README.md) |

## 不再计划

| 项目 | 原因 |
|---|---|
| 与 Theta\* / JPS 等任意角度规划器对比 | 开槽只允许横平竖直的线段，任意角度规划器不是合适的基线；改为与逐条规划、第一版规划器对比 |
| 单独的 Nav2 参数调优 | 导航只负责把底盘送到墙前约 1–2 m 处，精度由精对位保证；当前参数下多站任务的导航段全部成功。Nav2 已换用 Smac 2D + MPPI，见 [系统层报告](experiments/system_modernization/README.md) |
