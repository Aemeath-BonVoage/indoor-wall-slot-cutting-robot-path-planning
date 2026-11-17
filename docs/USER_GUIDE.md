# 使用指南

本文档说明如何编译、启动和使用 SCR 仿真系统：从环境准备、建图、导航，到单站开槽任务、多站开槽规划的执行和可视化。各模块的设计与实验结果见 [项目演进](EVOLUTION.md) 和 [实验报告](../README.md#项目演进与文档)。

- [1. 环境与编译](#1-环境与编译)
- [2. 启动仿真](#2-启动仿真)
- [3. 底盘与机械臂的手动控制](#3-底盘与机械臂的手动控制)
- [4. SLAM 建图](#4-slam-建图)
- [5. 导航](#5-导航)
- [6. 单站任务：导航 → 精对位 → 扫描 → 切割](#6-单站任务导航--精对位--扫描--切割)
- [7. 多站开槽：离线规划 → 逐站执行](#7-多站开槽离线规划--逐站执行)
- [8. 可视化与演示录制](#8-可视化与演示录制)
- [9. 测试与实验复现](#9-测试与实验复现)
- [10. 故障排查](#10-故障排查)
- [附录：坐标系、话题与机器人参数](#附录坐标系话题与机器人参数)

---

## 1. 环境与编译

### 1.1 系统要求

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo 11（Gazebo Classic，`ros-humble-gazebo-ros-pkgs`）
- Nav2、SLAM Toolbox、ros2_control

参考配置：4 核 CPU / 4 GB 内存的虚拟机可以运行完整仿真（不开 Gazebo 窗口时）。

### 1.2 安装依赖并编译

```bash
git clone https://github.com/Aemeath-BonVoage/indoor-wall-slot-cutting-robot-path-planning.git ~/colcon_ws
cd ~/colcon_ws
sudo rosdep init   # 首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

也可以手动安装主要依赖：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-teleop-twist-keyboard \
  ros-humble-robot-localization ros-humble-cv-bridge
```

`--symlink-install` 下修改 URDF、world、launch 和 YAML 后无需重新编译；修改 C++ 代码（`scr_planner`、`scr_description` 中的 Gazebo 插件）后需要重新编译对应的包。

### 1.3 Docker（可选）

`docker/` 中提供基于 `osrf/ros:humble-desktop-full` 的开发镜像，`src/` 挂载到容器内的 `/workspace/src`：

```bash
cd docker
./build.sh
./start.sh
docker exec -it ros2-humble-container bash
```

在容器内：

```bash
cd /workspace
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

容器内 RViz 出现 GLSL 着色器报错时见 [10.6](#106-docker-中-rviz-报-glsl-错误)。

---

## 2. 启动仿真

### 2.1 一键启动（推荐）

`scr_bringup` 用一条命令启动 Gazebo 仿真、Nav2 导航和 RViz：

```bash
ros2 launch scr_bringup sim_nav.launch.py                        # 完整界面
ros2 launch scr_bringup sim_nav.launch.py gui:=false             # 不开 Gazebo 窗口
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false # 纯后台，内存占用最小
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `gui` | `true` | 是否启动 Gazebo 窗口（gzclient） |
| `rviz` | `true` | 是否启动 RViz |
| `rtf` | `1.0` | 仿真速度上限（相对实时的倍数），`0` 为不限速 |
| `world` | `scr_description/worlds/room.world` | Gazebo 世界文件 |
| `map` | `scr_navigation/maps/my_map.yaml` | Nav2 使用的地图 |
| `params_file` | `scr_navigation/config/nav2_params.yaml` | Nav2 参数文件 |
| `nav_delay` | `8.0` | Gazebo 启动后等待多少秒再启动 Nav2 |

Nav2 在启动后约 20–30 s 就绪。初始位姿由 `set_initial_pose.py` 自动发布（默认 `(0, 0, 0)`，与机器人在 `room.world` 中的出生位置一致），不需要在 RViz 中手动设置。纯后台模式下，仿真加完整 Nav2 约额外占用 400 MB 内存，仿真实时率约 1.0（4 核虚拟机）。

### 2.2 只启动仿真

不需要导航时（例如单独测试机械臂、运行对位实验）：

```bash
ros2 launch scr_description gazebo.launch.py                     # 带 Gazebo 窗口
ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5 # 批量实验：最多 2.5 倍实时
```

该 launch 文件启动 gzserver（及可选的 gzclient）、`robot_state_publisher`、生成机器人实体、底盘运动学节点 `mecanum_controller.py`，并在 5 s 后加载 `joint_state_broadcaster`、`arm_controller`、`wheel_controller` 三个控制器。

**关于 `rtf`**：所有控制循环按仿真时钟计时，加速不改变控制律；但感知与控制代码的计算延迟按仿真时间计会被放大 `rtf` 倍。批量实验前应先用同一随机种子与 `rtf:=1.0` 对比，确认结果一致。带 Nav2 的流程建议使用 `rtf:=1.0`。

### 2.3 只查看机器人模型

```bash
ros2 launch scr_description display.launch.py
```

启动 RViz 和 `joint_state_publisher_gui`，可以拖动滑条查看各关节的运动方向。

---

## 3. 底盘与机械臂的手动控制

### 3.1 底盘

底盘是麦克纳姆轮全向底盘，接受标准的 `/cmd_vel`（`linear.x` 前后、`linear.y` 左右、`angular.z` 转向）。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘按住 `Shift` 时 `u / o / m / .` 等键为横移和斜移。也可以直接发布速度指令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

仿真中的速度链路：`/cmd_vel` → `mecanum_controller.py`（同时计算四轮转速，驱动轮子转动作为显示）→ `/cmd_vel_direct` → 力控平面运动插件 `scr_planar_move`（对底盘施加力 / 力矩做速度闭环）→ 发布 `/odom` 与 `odom → base_link` TF。

### 3.2 机械臂

机械臂是直角坐标式 4 自由度结构，由 `arm_controller`（`FollowJointTrajectory`）驱动：

| 关节 | 类型 | 范围 | 作用 |
|---|---|---|---|
| `joint1` | 移动 | −0.35 ~ 0.20 m | Z 升降（竖切） |
| `joint2` | 移动 | −0.08 ~ 0.06 m | Y 横移（横切），关节值增大时末端向 −Y |
| `joint3` | 移动 | −0.045 ~ 0.02 m | X 伸缩（切深），关节值增大时末端向后 |
| `joint4` | 转动 | −90° ~ 90° | 刀片方向 |

笛卡尔工作空间（相对 `base_link`，TF 实测）：X 0.517–0.582 m，Y −0.060–0.080 m，Z 0.157–0.707 m；所有关节为 0 时末端位于 (0.537, 0.000, 0.507) m。逆运动学为解耦的闭式解：`x = x0 − joint3`，`y = y0 − joint2`，`z = z0 + joint1`。

示例程序（需先启动仿真）：

```bash
ros2 run scr_arm_controller square_demo         # 关节空间：用 joint1 / joint2 画 5 cm 正方形
ros2 run scr_arm_controller cartesian_cutting   # 笛卡尔空间：在初始位置附近走 5 cm 矩形
```

用 YAML 定义任意点序列（至少 2 个点，`rotation` 单位为度）：

```bash
ros2 run scr_arm_controller custom_cutting wall_cutting.yaml          # 包内 config/ 下的文件
ros2 run scr_arm_controller custom_cutting /path/to/my_points.yaml    # 或给出完整路径
```

```yaml
points:
  - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.532, rotation: 0}
motion:
  duration: 3.0      # 每段运动时间（s）
  return_home: true  # 结束后回到关节零位
```

`config/` 下的示例：`wall_cutting.yaml`（墙面内 YZ 矩形）、`horizontal_cutting.yaml`（水平面 XY 矩形，刀片 90°）、`retract_rotate_extend.yaml`（伸缩—旋转—伸出）、`complex_sequence.yaml`、`cutting_task.yaml`（模板）。执行前会逐点检查可达性，任何一点超出关节限位则整段不执行。

也可以直接发送关节目标：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4], points: [{positions: [0.1, 0.0, 0.0, 0.5], time_from_start: {sec: 2}}]}}"
```

---

## 4. SLAM 建图

仓库中已有 `room.world` 的地图 `src/scr_navigation/maps/my_map.yaml`，直接导航时可跳过本节。换用新的世界文件时按以下步骤建图：

```bash
# 终端 1：仿真
ros2 launch scr_description gazebo.launch.py
# 终端 2：SLAM Toolbox（异步建图）+ RViz
ros2 launch scr_navigation slam.launch.py
# 终端 3：键盘遥控，覆盖需要建图的区域
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 建图完成后保存（生成 .yaml 与 .pgm）
ros2 run nav2_map_server map_saver_cli -f src/scr_navigation/maps/my_map
```

建图时保持较低速度（0.2–0.3 m/s），转向不要太快，尽量回到起点形成闭环。SLAM 参数在 `src/scr_navigation/config/slam_toolbox.yaml`（分辨率 0.05 m）。

---

## 5. 导航

分步启动（一键启动见 [2.1](#21-一键启动推荐)）：

```bash
ros2 launch scr_description gazebo.launch.py
ros2 launch scr_navigation navigation.launch.py                                   # 默认地图 my_map.yaml
ros2 launch scr_navigation navigation.launch.py map:=/path/to/other_map.yaml rviz:=false
```

`navigation.launch.py` 的参数：`map`、`params_file`、`use_sim_time`（默认 `true`）、`rviz`、`initial_pose_x / initial_pose_y / initial_pose_yaw`（默认 0，启动 6 s 后自动发布）。

发送导航目标：

```bash
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0     # x y yaw（map 坐标，米 / 弧度）
```

或在 RViz 中用 “Nav2 Goal” 点选。若机器人实际不在默认初始位姿，先用 “2D Pose Estimate” 设置初始位姿。

Nav2 配置要点（`src/scr_navigation/config/nav2_params.yaml`）：AMCL 使用全向运动模型 `OmniMotionModel`；全局规划器 NavFn；局部控制器 DWB，速度上限 `vx = vy = 0.4 m/s`、`ωz = 0.8 rad/s`；代价地图机器人半径 0.4 m、膨胀半径 0.7 m。可以通过 `params_file:=` 换用其他参数文件。

---

## 6. 单站任务：导航 → 精对位 → 扫描 → 切割

### 6.1 运行

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false        # 终端 1
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml   # 终端 2
```

相对路径的任务文件先在当前目录查找，再到 `scr_arm_controller` 包的 `config/` 中查找。完整任务约 100 s。

流程：

1. **导航**：Nav2 把底盘送到任务文件中的 `navigation` 位姿（墙前约 1.9 m）。仿真刚启动时 Nav2 可能拒绝第一个目标，程序会每 3 s 重试，最多 10 次。
2. **精对位**：机械臂先移到观测位姿（滑台降 0.15 m，避免机械臂遮挡相机），伸缩轴完全缩回；然后驱动全向底盘停在墙面定位标记正前方 `distance_from_marker` 处：相机检测 AprilTag 定位标记、激光拟合墙面，二者经 EKF 融合为墙面相对位姿估计，饱和控制律闭环，满足收敛判据（误差在容差内、底盘静止、估计不确定度足够小）后转入定点保持。参数见 `config/fine_alignment.yaml`，设计见 [精对位报告](experiments/fine_alignment/README.md)。
3. **扫描**：在当前位姿检测定位标记（5 帧平均）；未检测到时，机械臂末端在 Z = 0.30–0.70 m（`base_link` 坐标）内以 5 cm 步长逐级检测。完成后回到缩回位姿。
4. **切割**：按 `points` 逐点执行。从对位结束到切割完成，底盘处于定点保持状态。

运行中输出两类真值误差，用于评估（不参与控制）：对位结束时底盘相对目标的真值误差，以及扫描 + 切割期间底盘的最大真值漂移。

### 6.2 任务文件

`src/scr_arm_controller/config/task_at_target.yaml`：

```yaml
navigation:              # Nav2 目标（map 坐标）
  x: 8.61277
  y: -2.01879
  yaw: -1.64894

precise_positioning:
  enabled: true
  mode: visual                # visual = AprilTag + 激光 EKF；visual_v1 = 第一版色块 PBVS；odom = 里程计 + 已知标记坐标
  distance_from_marker: 0.70  # 对位后底盘中心离墙距离（米）
  position_tolerance: 0.01    # 收敛容差（米）
  angle_tolerance_deg: 1.0    # 收敛容差（度）
  observe_joint1: -0.15       # 对位时滑台高度（米）

base_hold: true          # 扫描 / 切割期间的定点保持；false 时只测量漂移

scanning:
  enabled: true          # 扫描高度范围在程序中固定为 0.30–0.70 m

points:                  # 机械臂切割点（base_link 坐标，rotation 单位为度）
  - {x: 0.520, y: 0.000, z: 0.507, rotation: 0}
  # ...
motion:
  duration: 2.0          # 每段运动时间（s）
  return_home: true
  dry_run_gap: 0.005     # 空切间隙（米），见下文
```

**空切**：Gazebo 中的墙是刚体，刀具按切割点压入墙面会把整台机器人顶开（约 9 mm、3–4°）。设置 `motion.dry_run_gap` 后，处于切深的点按激光实测墙距整体后移，刀尖最深处停在墙前该距离，只验证轨迹位置。真机上删去此项。

**安全措施**：接近墙面时机械臂伸缩轴完全缩回（0.70 m 对位时刀尖离墙约 24 mm）；激光防碰撞守护按制动距离限制向墙速度，允许速度 = (刀尖间隙 − 5 mm) / 0.5 s。

---

## 7. 多站开槽：离线规划 → 逐站执行

机械臂每站沿墙方向只能切约 12 cm，一条槽要分多个停靠站完成。系统先离线规划整面墙的槽线和停靠站，再由主任务逐站执行，执行中不重新规划。

### 7.1 墙面任务文件

示例 `src/scr_planner/config/wall_task_room3_south.yaml`（`room.world` 中 room3 南墙）：

```yaml
wall:
  frame_id: map
  left_end: [9.925, -3.925]     # 面对墙面时的左端（map 坐标）
  right_end: [5.075, -3.925]
  height: 2.0
  load_bearing: false           # 承重墙只允许竖槽
obstacles:                      # 墙面坐标 (u, v)：u 沿墙向右，v 离地高度；(u, v) 为左下角
  - {kind: box, u: 1.480, v: 0.575, w: 0.05, h: 0.05, label: 黄色定位标记}
tasks:                          # 每条槽的起止线盒中心 (u, v)
  - {start: [1.20, 0.34], start_label: 已有插座A, goal: [2.30, 0.34], goal_label: 新插座A}
  - {start: [2.45, 0.72], start_label: 已有插座B, goal: [1.75, 0.72], goal_label: 书桌插座B}
options:
  restrict_to_reach: true       # 只在机械臂可切高度带（离地约 0.27–0.80 m）内规划
  retract_between_turns: true   # 改变刀片角度前先退刀
  duration: 2.0
  partition: dp                 # dp / greedy / per_slot
  order: sweep                  # sweep / slot_by_slot
```

`obstacles.kind` 可取 `door / window / beam / column / box / weak_line / pier`，各类规范边距定义在 `src/scr_planner/python/wall_rules.py`。线盒高度须在可切高度带内。

### 7.2 规划

```bash
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_planner/config/wall_task_room3_south.yaml /tmp/plan.yaml
# 基线方案：每条槽单独分站、逐条执行
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_planner/config/wall_task_room3_south.yaml /tmp/plan_naive.yaml \
    --partition=per_slot --order=slot_by_slot
```

不需要 ROS。输出包含每条槽的直角折线、按执行顺序排列的停靠站、每站底盘位姿与机械臂切割点、预测执行时间。也可以运行 ROS 节点 `wall_slot_planner_node.py`（参数 `task_file`、`output_file`），它额外在 `/wall_slot_plan/markers` 发布 RViz 显示。

### 7.3 执行

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01
```

主任务识别到规划文件（含 `wall` 和 `stations`）就走多站流程：Nav2 到定位标记前 1.2 m → 精对位到 0.70 m（此时的里程计位姿作为墙面坐标原点）→ 逐站：沿墙横移到站 → 定点保持 → 用激光 / 里程计测量残余误差并补偿机械臂点 → 切割。

| 选项 | 默认 | 说明 |
|---|---|---|
| `--dry-run-gap=<m>` | 不设 | 空切：切深处的点改为刀尖停在墙前该距离 |
| `--max-strafe=<m>` | 1.5 | 站间距离超过该值时，先缩回机械臂、退到离墙 1.2 m 用 Nav2 过去 |
| `--no-compensation` | — | 关闭到站后的残差补偿 |

需要真值评估时使用实验记录器，它调用同一执行路径，额外记录刀尖轨迹、到站误差和底盘漂移：

```bash
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run --dry-run-gap=0.01
```

输出 `stations.csv`、`tool_trace.csv`、`summary.yaml`。示例任务（两条槽、11 站）约 250 s 仿真时间。

---

## 8. 可视化与演示录制

### 8.1 RViz 开槽场景

```bash
# 离线（不开 Gazebo）：静态 TF map → odom，载入 scr_viz/config/ 下的示例任务与规划
ros2 launch scr_viz plan_viz.launch.py static_tf:=true
# 与仿真一起：显示新的规划，并从 /gazebo/link_states 推算实际刀尖轨迹
ros2 launch scr_viz plan_viz.launch.py plan_file:=/tmp/plan.yaml use_sim_time:=true
```

显示墙面、可切高度带、障碍与规范边距、槽线（线宽 = 槽宽）与导管中心线、停靠站序号与可达窗口、访问顺序、计划刀尖路径与实际刀尖轨迹（红 = 切割，青 = 移动）。规划或任务文件改写后自动重新加载。清空轨迹：

```bash
ros2 service call /scr_plan_viz/clear_trace std_srvs/srv/Empty
```

不启动 ROS 时可以生成同一场景的 PNG：

```bash
ros2 run scr_viz plan_preview --task src/scr_viz/config/wall_task_room3_south_demo.yaml \
    --plan src/scr_viz/config/plan_room3_south_demo.yaml --out /tmp/preview.png
```

### 8.2 Gazebo 墙面实体与演示相机

```bash
# 在 Gazebo 中生成 / 删除插座、开关、配电箱等墙面实体（默认只有外观、没有碰撞，不影响激光与切割）
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --spawn
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --delete
# 固定视角演示相机：按仿真时间存帧，Ctrl-C 结束后合成视频
ros2 run scr_viz demo_record --out /tmp/demo_frames --preset oblique --rate 2 --video /tmp/demo.mp4
ros2 run scr_viz make_demo_video /tmp/demo_frames /tmp/demo.gif --fps 12 --width 480
```

视角预设：`oblique`、`front`、`side`、`top`，或用 `--eye X Y Z --target X Y Z` 指定。墙面实体的运行时生成和演示相机出图尚未在 Gazebo 中完整验证（见 [demo 报告](experiments/demo/README.md)）。详见 [`src/scr_viz/README.md`](../src/scr_viz/README.md)。

---

## 9. 测试与实验复现

```bash
colcon build && colcon test && colcon test-result --verbose
colcon test --packages-select scr_planner     # 只测单个包
```

测试包括 C++ gtest、pytest 单元测试和 flake8 / pep257 代码风格检查；GitHub Actions 在 `ros:humble` 容器中执行同样的编译与测试。

各实验的复现命令在对应报告的“复现”一节：

| 实验 | 是否需要 Gazebo | 报告 |
|---|---|---|
| 对位鲁棒性（第一版 vs 第二版精对位） | 是 | [alignment_robustness](experiments/alignment_robustness/README.md) |
| 开槽规划算法与施工规范 | 否 | [cpp_planner](experiments/cpp_planner/README.md) |
| 多站执行与停靠站顺序优化 | 是（离线对比部分否） | [slot_execution](experiments/slot_execution/README.md) |
| 任务级联合规划（槽线 + 停靠站） | 否 | [task_planning](experiments/task_planning/README.md) |
| 精对位设计与控制分析 | 离线部分否 | [fine_alignment](experiments/fine_alignment/README.md) |
| PBVS / IBVS / 2.5D 混合伺服对比 | 否 | [ibvs_vs_pbvs](experiments/ibvs_vs_pbvs/README.md) |
| 可视化与演示 | 部分 | [demo](experiments/demo/README.md) |

第一版规划器（Python 原型）的对比脚本不需要 ROS：

```bash
pip install numpy matplotlib pillow
cd src/path_planning
python3 improved_astar.py         # 基于 SLAM 地图的标准 / 改进 / 增强 A* 对比
python3 wall_cutting_planner.py   # 墙面开槽轨迹的三算法对比
```

---

## 10. 故障排查

### 10.1 RViz 中没有地图，报 `frame "map" does not exist`

Nav2 尚未就绪或地图未加载。确认地图文件存在（`ls src/scr_navigation/maps/`），等待约 20–30 s；仍无地图时检查 `map:=` 参数是否为完整路径。机器人不在默认初始位姿时，用 RViz 的 “2D Pose Estimate” 设置初始位姿。

### 10.2 机器人不响应 `/cmd_vel`

```bash
ros2 control list_controllers          # 应有 joint_state_broadcaster / arm_controller / wheel_controller，均为 active
ros2 topic info /cmd_vel_direct        # 应有 mecanum_controller 发布、planar_move 订阅
```

控制器在仿真启动 5 s 后才加载。若控制器加载失败，查看 gzserver 输出中的 ros2_control 报错，并确认已安装 `ros-humble-gazebo-ros2-control`。

### 10.3 机械臂不动或报“不可达”

检查点是否在笛卡尔工作空间内（见 [3.2](#32-机械臂)）。`custom_cutting` 与任务程序都在执行前做可达性检查，任何一点越界整段不执行。

### 10.4 对位不收敛或检测不到定位标记

```bash
ros2 topic hz /scan
ros2 topic hz /front_camera/image_raw
ros2 run rqt_image_view rqt_image_view /front_camera/image_raw
```

对位需要激光看到墙面、相机看到定位标记（相机水平视场 80°）。导航目标离标记太远或偏角太大时，标记可能不在视野内；此时调整任务文件中的 `navigation` 位姿，使机器人停在标记正前方 1–2 m 处。对位在 50 s 仿真时间内未收敛时如实报告失败。

### 10.5 TF 查询失败

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

正常的 TF 树为 `map → odom → base_link → link1 → … / camera_link → camera_optical_link / lidar_link`。`map → odom` 由 AMCL 发布（只启动仿真时没有），`odom → base_link` 由 `scr_planar_move` 发布，其余由 `robot_state_publisher` 发布。对位和扫描只需要 `base_link` 以下的 TF，不依赖 AMCL。

### 10.6 Docker 中 RViz 报 GLSL 错误

容器内 OpenGL 兼容性问题，使用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

`docker/start.sh` 已默认设置该变量。

### 10.7 多个仿真互相干扰

同一台机器上同时运行多套仿真或残留进程时，话题会互相串扰。关闭后确认没有残留进程（`pgrep -af "gzserver|robot_state_publisher|mecanum_controller"`），或为每套仿真设置不同的 `ROS_DOMAIN_ID`。

### 10.8 关闭仿真时 `mecanum_controller` 报错退出

按 Ctrl-C 关闭仿真时，launch 会显示 `mecanum_controller` 以非零码退出。这是该节点未处理关闭信号导致的退出报错，不影响仿真和实验结果，已列入 [路线图](ROADMAP.md)。

---

## 附录：坐标系、话题与机器人参数

**坐标系**：`base_link` 为 X 向前、Y 向左、Z 向上；`base_link` 离地 0.1025 m。墙面坐标 `(u, v)`：面向墙面时 `u` 沿墙向右，`v` 为离地高度。

**主要话题与接口**

| 名称 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令 |
| `/odom` | `nav_msgs/Odometry` | 底盘里程计（`scr_planar_move`，50 Hz） |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达（360 线，测距噪声 σ = 1 cm） |
| `/front_camera/image_raw`、`/front_camera/camera_info` | `sensor_msgs/Image`、`CameraInfo` | 前视相机，640 × 480，水平视场 80°，30 Hz |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 机械臂轨迹 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 导航 |
| `/gazebo/model_states`、`/gazebo/link_states` | `gazebo_msgs` | 仿真真值，只用于评估 |
| `/scr_plan_viz/markers`、`/scr_plan_viz/tool_trace` | `visualization_msgs/MarkerArray` | 开槽场景与刀尖轨迹 |

**机器人参数**

| 项目 | 参数 |
|---|---|
| 底盘 | 麦克纳姆轮，轮径 100 mm，轮距 593.16 × 493.80 mm |
| 机械臂工作空间 | X 0.517–0.582 m，Y −0.060–0.080 m，Z 0.157–0.707 m（相对 `base_link`） |
| 可切高度 | 离地约 0.27–0.80 m |
| 刀尖前伸 | 伸缩轴完全缩回时离底盘中心 0.676 m，完全伸出时 0.741 m |
| 传感器 | 2D 激光雷达、前视 RGB 相机（安装在升降滑台上） |
| 定位标记 | AprilTag tag36h11 ID 0（黑框 0.10 m，中心离地 0.70 m）；第一版使用的 5 cm 黄色色块（中心离地 0.60 m）保留作对比。均位于 room3 南墙 x = 8.42 m |
