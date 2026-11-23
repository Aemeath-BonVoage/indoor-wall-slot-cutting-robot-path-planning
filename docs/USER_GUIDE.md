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
- [11. Docker 复现](#11-docker-复现)
- [附录：坐标系、话题与机器人参数](#附录坐标系话题与机器人参数)

---

## 1. 环境与编译

### 1.1 系统要求

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo 11（Gazebo Classic，`ros-humble-gazebo-ros-pkgs`）
- Nav2（含 MPPI 控制器、Smac 规划器与 BehaviorTree.CPP v3）、SLAM Toolbox、ros2_control、`gazebo_ros2_control`、`robot_localization`

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

`--symlink-install` 下修改 URDF、world、launch、YAML 和行为树 XML 后无需重新编译；修改 C++ 代码（`scr_planner`、`scr_task`、`scr_base_control`，以及 `scr_description` 中的 Gazebo 插件）或 `scr_interfaces` 中的消息 / action 定义后需要重新编译对应的包。

### 1.3 Docker（可选）

不在本机安装 ROS 2 与 Gazebo 时，可以用 `docker/` 中的镜像：依赖由 rosdep 安装，工作空间在镜像内编译好，无界面或带窗口运行整个系统。构建、运行多站开槽任务和测试的命令见 [第 11 节](#11-docker-复现)。

---

## 2. 启动仿真

### 2.1 一键启动（推荐）

`scr_bringup` 用一条命令启动 Gazebo 仿真、Nav2 导航、任务层（技能服务器 + 行为树执行器）和 RViz：

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
| `params_file` | `scr_navigation/config/nav2_params.yaml` | Nav2 参数文件（默认 Smac 2D + MPPI；基线 `nav2_params_dwb.yaml` 为 NavFn + DWB） |
| `nav_delay` | `8.0` | Gazebo 启动后等待多少秒再启动 Nav2 与任务层 |
| `use_composition` | `True` | Nav2 各服务器放在一个容器进程中；`False` 时每个服务器一个进程（便于分别统计 CPU） |
| `base_control` | `ros2_control` | 底盘实现：`ros2_control`（麦克纳姆控制器 + 仿真硬件接口）或 `plugin`（早期力控插件，`/odom` 为真值） |
| `plant` | `force` | ros2_control 底盘的被控对象：`force`（力驱动）或 `wheels`（轮子驱动，实验） |
| `odom_error` | `true` | 编码器误差模型开关（只作用于 `force`）；`false` 时轮式里程计等于真值积分 |
| `odom_seed` | `1` | 编码器打滑过程的随机种子 |
| `localization` | `ekf` | `ekf`：轮式里程计 + 陀螺经 `robot_localization` 融合为 `/odom`；`wheel`：控制器直接发布 `/odom` |
| `task` | `true` | 是否启动任务层；只做导航实验时设为 `false` |
| `alignment_params` | `scr_arm_controller/config/fine_alignment.yaml` | 技能服务器的精对位参数文件（`alignment.*`） |

Nav2 在启动后约 20–30 s 就绪。初始位姿由 `set_initial_pose.py` 自动发布（默认 `(0, 0, 0)`，与机器人在 `room.world` 中的出生位置一致），不需要在 RViz 中手动设置。任务层的行为树执行器先等待 Nav2（`bt_navigator` 激活）与技能服务器就绪，再加载行为树。纯后台模式下，仿真加完整 Nav2 约额外占用 400 MB 内存，仿真实时率约 1.0（4 核虚拟机）。

### 2.2 只启动仿真

不需要导航时（例如单独测试机械臂、运行对位实验、底盘特性测试）：

```bash
ros2 launch scr_description gazebo.launch.py                     # 带 Gazebo 窗口
ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5 # 批量实验：最多 2.5 倍实时
```

该 launch 文件启动 gzserver（及可选的 gzclient）、`robot_state_publisher`、生成机器人实体，并在 5 s 后加载控制器。`gazebo.launch.py` 接受 `world`、`gui`、`rtf`、`camera_yaw_error_deg`（相机安装偏角，只注入 Gazebo 中的模型，用于验证示教零位）以及与 2.1 相同的 `base_control`、`plant`、`odom_error`、`odom_seed`、`localization`。按底盘实现不同，启动的节点为：

| `base_control` | 底盘相关节点 | 控制器 |
|---|---|---|
| `ros2_control`（默认） | 仿真硬件接口 `ScrMecanumBaseSystem`（在 `gazebo_ros2_control` 中加载）；`localization:=ekf` 时另启动 `robot_localization` 的 `ekf_filter_node` | `joint_state_broadcaster`、`arm_controller`、`base_controller` |
| `plugin` | 力控平面运动插件 `scr_planar_move` + 运动学节点 `mecanum_controller.py` | `joint_state_broadcaster`、`arm_controller`、`wheel_controller` |

真值始终发布在 `/ground_truth/odom`（`gazebo_ros_p3d`）与 `/gazebo/model_states`、`/gazebo/link_states`，只用于评估。

**关于 `rtf`**：所有控制循环按仿真时钟计时，加速不改变控制律；但感知与控制代码的计算延迟按仿真时间计会被放大 `rtf` 倍。批量实验前应先用同一随机种子与 `rtf:=1.0` 对比，确认结果一致。带 Nav2 的流程建议使用 `rtf:=1.0`。

### 2.3 只查看机器人模型

```bash
ros2 launch scr_description display.launch.py
```

启动 RViz 和 `joint_state_publisher_gui`，可以拖动滑条查看各关节的运动方向。

### 2.4 底盘、里程计与定位

默认配置（`base_control:=ros2_control plant:=force odom_error:=true localization:=ekf`）中：

- 底盘由 ros2_control 控制：`base_controller`（`scr_base_control/MecanumDriveController`）把 `/cmd_vel` 经逆运动学分配为四个轮的速度命令，并由四个轮的速度状态积分出轮式里程计 `/wheel/odom`；
- 仿真硬件接口 `ScrMecanumBaseSystem` 用力 / 力矩驱动底盘，编码器读数由底盘实际运动加误差模型（每轮轮径误差、随机打滑、横移打滑 3 %、转向打滑 2 %）得到，同时以 100 Hz 在 `/imu` 发布模拟陀螺；
- `robot_localization` 的 EKF 融合轮式里程计的 `(vx, vy)` 与陀螺的 `wz`，发布 `/odom` 与 `odom → base_link`，供 Nav2 与 AMCL 使用；
- 技能服务器的局部闭环（精对位预测、站间横移、定点保持）使用 `/wheel/odom`，由 `sim_nav.launch.py` 自动设置。原因是 EKF 输出在 mm/s 级的慢速运动下明显偏小，见 [系统层现代化报告](experiments/system_modernization/README.md#45-真实里程计下的精对位与多站执行) 4.5 节。

常用组合：

```bash
ros2 launch scr_bringup sim_nav.launch.py localization:=wheel   # 只用轮式里程计
ros2 launch scr_bringup sim_nav.launch.py odom_error:=false     # 理想编码器
ros2 launch scr_bringup sim_nav.launch.py plant:=wheels         # 轮子驱动底盘（实验，跟踪与里程计明显变差）
ros2 launch scr_bringup sim_nav.launch.py base_control:=plugin  # 早期配置：/odom 为真值，用于复现早期结果
```

本仓库中早于系统层现代化的实验数据（精对位、多站执行、时间模型标定）都在早期配置下取得（仿真配置 v2，`/odom` 为真值，NavFn + DWB）。复现这些数据时使用 `base_control:=plugin`，需要时再加 `params_file:=` 指向 `nav2_params_dwb.yaml`；多站执行的数据还需要早期的站间语义（单标记、站间只用里程计），见 [6.4](#64-定位标记地图与横移比例估计) 末尾。

编码器的横移打滑（3 %）使里程计横移距离偏大。默认的精对位参数用墙上的多标记地图和 EKF 中在线估计的横移比例补偿这一项，11 站开槽的最大沿墙误差为 1.5 mm（只用一个标记、站间只靠里程计时为 24.7 mm），见 [6.4](#64-定位标记地图与横移比例估计)。

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

仿真中的速度链路（默认 `base_control:=ros2_control`）：`/cmd_vel` → `base_controller`（逆运动学 → 四个轮的速度命令）→ `ScrMecanumBaseSystem`（对底盘施加力 / 力矩做速度闭环；轮子只按命令转动作为外观）→ 编码器读数 → `base_controller` 积分为 `/wheel/odom` → EKF 融合陀螺后发布 `/odom` 与 `odom → base_link` TF。`/cmd_vel` 超过 0.5 s 没有新指令时底盘停车。

早期配置（`base_control:=plugin`）的链路为：`/cmd_vel` → `mecanum_controller.py`（同时计算四轮转速作为显示）→ `/cmd_vel_direct` → 力控平面运动插件 `scr_planar_move` → 以真值位姿发布 `/odom` 与 TF。

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

`navigation.launch.py` 的参数：`map`、`params_file`、`use_sim_time`（默认 `true`）、`rviz`、`use_composition`（默认 `True`）、`initial_pose_x / initial_pose_y / initial_pose_yaw`（默认 0，启动 6 s 后自动发布）。分步启动时不包含任务层，需要时另开终端运行 `ros2 launch scr_task task.launch.py`（技能服务器的 `odom_topic` 默认为 `/odom`，融合定位时应加 `odom_topic:=/wheel/odom`）。

发送导航目标：

```bash
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0     # x y yaw（map 坐标，米 / 弧度）
```

或在 RViz 中用 “Nav2 Goal” 点选。若机器人实际不在默认初始位姿，先用 “2D Pose Estimate” 设置初始位姿。

Nav2 配置要点（`src/scr_navigation/config/nav2_params.yaml`）：AMCL 使用全向运动模型 `OmniMotionModel`；全局规划器 Smac 2D（`SmacPlanner2D`，`cost_travel_multiplier = 2.0`，路径远离障碍）；局部控制器 MPPI，运动模型 `Omni`，1000 条采样、40 步 × 50 ms 预测时域、20 Hz；速度上限 `vx = vy = 0.4 m/s`、`ωz = 0.8 rad/s`；代价地图机器人半径 0.4 m、膨胀半径 0.7 m。

基线配置 `nav2_params_dwb.yaml`（NavFn + DWB）与默认配置只有 `FollowPath` 与 `GridBased` 两项不同，用 `params_file:=` 切换：

```bash
ros2 launch scr_bringup sim_nav.launch.py \
    params_file:=$(ros2 pkg prefix scr_navigation)/share/scr_navigation/config/nav2_params_dwb.yaml
```

两者的对比（到达率、离障间隙、平滑性、CPU）见 [系统层现代化报告](experiments/system_modernization/README.md#42-nav2smac-2d--mppi-与-navfn--dwb) 4.2 节。导航巡回基准脚本：

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false task:=false use_composition:=False
ros2 run scr_navigation nav_benchmark.py --out /tmp/nav_mppi --rounds 3 --label mppi_smac2d
```

---

## 6. 单站任务：导航 → 精对位 → 扫描 → 切割

### 6.1 运行

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false        # 终端 1：仿真 + Nav2 + 任务层
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml   # 终端 2
```

任务由行为树执行。`navigation_cutting_task` 是命令行入口（action 客户端）：它判断任务文件的类型，单站任务发给行为树执行器的 `/execute_cutting_task`，多站开槽规划发给 `/execute_slot_plan`（第 7 节），然后打印执行阶段与结果；Ctrl-C 会取消任务。相对路径的任务文件先在当前目录查找，再到 `scr_arm_controller` 包的 `config/` 中查找。任务层没有就绪时，客户端每 10 s 打印一次“等待行为树执行器”。完整任务约 85 s 仿真时间。

单站任务的行为树为 [`cutting_task.xml`](../src/scr_task/behavior_trees/cutting_task.xml)，各步骤是技能服务器 `task_skill_server` 的 action：

1. **导航**（`NavigateToPose`）：Nav2 把底盘送到任务文件中的 `navigation` 位姿（墙前约 1.9 m），失败时重试一次。
2. **精对位**（`FineAlign`）：机械臂先移到观测位姿（滑台降 0.15 m，避免机械臂遮挡相机），伸缩轴完全缩回；然后驱动全向底盘停在墙面定位标记正前方 `distance_from_marker` 处：相机检测 AprilTag 定位标记、激光拟合墙面，二者经 EKF 融合为墙面相对位姿估计，饱和控制律闭环，满足收敛判据（误差在容差内、底盘静止、估计不确定度足够小）后到位。失败时缩回机械臂、重新导航并再对位一次。参数见 `config/fine_alignment.yaml`（[6.3](#63-精对位参数)），设计见 [精对位报告](experiments/fine_alignment/README.md)。
3. **扫描**（`ScanMarker`，与定点保持 `HoldBase` 并行）：在当前位姿检测定位标记（5 帧平均）；未检测到时，机械臂末端在 Z = 0.30–0.70 m（`base_link` 坐标）内以 5 cm 步长逐级检测。扫描失败不影响切割。
4. **切割**（`CutStation`，与 `HoldBase` 并行）：可达性检查后按 `points` 逐点执行。从对位结束到切割完成，底盘处于定点保持状态。

技能服务器的日志（`sim_nav.launch.py` 所在终端）中输出两类真值误差，用于评估（不参与控制）：对位结束时底盘相对目标的真值误差，以及扫描 + 切割期间底盘的最大真值漂移。

需要换用其他行为树时（例如修改恢复策略），用 `--bt=<tree.xml>` 指定 XML 文件；树中可用的节点见 [`scr_task`](../src/scr_task/README.md)。

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

行为树执行器读取上面列出的字段。示例文件中另有 `position_tolerance`、`angle_tolerance_deg`、`observe_joint1`、`base_hold` 与 `scanning.z_*` 等字段，它们来自早期的顺序程序，当前执行路径不读取：第一版对位的容差（1 cm / 1°）、观测滑台高度（−0.15 m）与扫描范围由技能服务器内置，扫描与切割期间始终定点保持。

**空切**：Gazebo 中的墙是刚体，刀具按切割点压入墙面会把整台机器人顶开（约 9 mm、3–4°）。设置 `motion.dry_run_gap` 后，处于切深的点按激光实测墙距整体后移，刀尖最深处停在墙前该距离，只验证轨迹位置。真机上删去此项。

**安全措施**：接近墙面时机械臂伸缩轴完全缩回（0.70 m 对位时刀尖离墙约 24 mm）；激光防碰撞守护按制动距离限制向墙速度，允许速度 = (刀尖间隙 − 5 mm) / 0.5 s。

### 6.3 精对位参数

精对位参数（`alignment.*`）属于技能服务器，启动时用 `alignment_params:=` 指定参数文件，默认 [`config/fine_alignment.yaml`](../src/scr_arm_controller/config/fine_alignment.yaml)。参数文件的节点名用通配 `/**`，同一文件也可以直接传给对位实验脚本。

```bash
# 示教零位（只需仿真，不需要 Nav2）：输出参数文件
ros2 run scr_arm_controller teach_alignment_offset --out /tmp/fine_alignment_taught.yaml --seconds 30
# 用示教结果执行任务
ros2 launch scr_bringup sim_nav.launch.py gui:=false alignment_params:=/tmp/fine_alignment_taught.yaml
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `alignment.detector` | `apriltag` | `apriltag` 或 `blob`（第一版色块检测器，基线） |
| `alignment.tolerance_normal / _lateral / _yaw_deg` | 2 mm / 2 mm / 0.3° | 收敛容差 |
| `alignment.offset_normal / _lateral / _yaw_deg` | 0 | 示教零位偏置 |
| `alignment.tag_family / tag_id / tag_size` | `tag36h11` / 0 / 0.10 m | 只有一个标记时（`tag_map` 为空串）使用；设置了标记地图时以地图中的 `family`、`size`、`reference_id` 为准 |
| `alignment.fold_residual` | `true` | 多站执行以 EKF 估计的标记位置为墙面原点；`false` 为第一版语义（以停车位姿为原点），用于消融 |
| `alignment.tag_map` | `tag_map.yaml` | 标记地图文件（[6.4](#64-定位标记地图与横移比例估计)）。相对路径按 `scr_arm_controller` 包的 `config/` 解析，也可用绝对路径或 `~`；空串表示只有 `tag_id` 一个标记 |
| `alignment.strafe_tag_updates` | `true` | 站间横移与到站残差测量时，画面中的地图标记继续更新沿墙位置；`false` 时站间沿墙位置只由里程计外推 |
| `alignment.observe_between_stations` | `true` | 横移前机械臂回到观测位姿（滑台 −0.15 m、伸缩轴缩回），使墙上的标记处在画面内；只在 `strafe_tag_updates` 时生效，每站约多 2 s |
| `alignment.estimate_scale` | `true` | 把里程计横移比例修正 `s`（真实横移 = (1 + s) × 里程计横移）作为 EKF 的第四个状态在线估计 |
| `alignment.scale_init / scale_sigma` | 0.0 / 0.05 | `s` 的先验均值与标准差；有离线标定值时可填入 `scale_init` |

早期版本中通过 `navigation_cutting_task ... --ros-args --params-file <yaml>` 或 `-p alignment.*:=` 传参；任务改由行为树执行后，这些参数需在启动任务层时传给技能服务器，对客户端设置不再生效。

### 6.4 定位标记地图与横移比例估计

激光只能观测离墙距离与朝向，沿墙位置只有看到定位标记时才可观测；站间横移时，沿墙位置由轮式里程计外推。默认配置用两种办法限制外推误差：

- **标记地图**：墙上布置多个已知位置的 AprilTag，画面中的任一地图内标记都更新沿墙位置。默认地图 [`config/tag_map.yaml`](../src/scr_arm_controller/config/tag_map.yaml) 使用 room3 南墙上的 ID 0、2、4（间距 0.6 m）；
- **横移比例估计**：在 EKF 中估计里程计横移的比例误差，看不到标记时按修正后的里程计外推。带标记的横移（同一个标记在画面内移动，或在两个已知间距的标记之间移动）提供估计所需的信息；往返同一位置不提供。

地图格式（解析见 [`tag_map.py`](../src/scr_arm_controller/scr_arm_controller/tag_map.py)）：

```yaml
tag_map:
  family: tag36h11
  size: 0.10                 # 黑框边长（米）
  reference_id: 0            # 墙面坐标原点，精对位以它为目标；其 u 必须为 0
  sigma_position: 0.002      # 标记相对参考标记的位置误差（米，1σ），计入测量噪声
  tags:                      # u：相对参考标记的沿墙位置，面向墙面向右为正（米）；v：中心离地高度，只作记录
    - {id: 0, u: 0.0, v: 0.70}
    - {id: 2, u: 0.6, v: 0.70}
    - {id: 4, u: 1.2, v: 0.70, sigma: 0.003}   # 可逐个标记给出 sigma
```

地图之外的标记 ID 被检测器忽略，因此可以在同一个世界中只换地图来比较不同的布置。`room.world` 中 ID 0 两侧另有 ID 1–5（间距 0.3 m，见 [`scr_description`](../src/scr_description/README.md)）；实验用的其他地图在 [`along_wall_accuracy/maps/`](experiments/along_wall_accuracy/maps/)。

布置标记的依据（[沿墙精度报告](experiments/along_wall_accuracy/README.md)）：

- **间距不超过相机的可见窗口**：离墙 0.70 m、观测位姿下，标记在底盘相对它 −0.39 … +0.44 m 内可用（约 0.83 m 宽）。间距 0.6 m 时每个停靠站都有可见标记；0.3 m 没有可区分的改善；1.2 m 留下 0.37 m 的无标记区，区内误差回到只用里程计时的水平；
- **记录实际粘贴位置**：地图位置误差几乎一比一进入结果（±3 mm 的记录偏差使最大沿墙误差 2.8 → 5.2 mm），标记贴好后应测量其位置，`sigma_position` 填测量的不确定度；
- **高度与观测位姿一致**：标记中心离地 0.70 m，配合 `observe_between_stations`。切割位姿下滑台较低，标记会落到画面上沿之外；
- **比例估计需要带标记的横移**：只有一个标记时，前几站在其视野内的净横移（本例约 0.66 m）把 `s` 估计到 σ ≈ 0.3 %，之后的外推误差按 σ_s × 外推距离增长（每米约 3 mm）。多标记 + 比例估计（默认）不依赖这一条件。

| 配置（11 站，各 2 次运行） | 最大到站沿墙误差 | 总耗时 |
|---|---|---|
| 一个标记，站间只用里程计 | 24.7 mm | 272 s |
| 一个标记，横移时保留标记更新 | 14.4 mm | 297 s |
| 标记间距 0.6 m | 2.8 mm | 296 s |
| 一个标记 + 横移比例估计 | 1.3 mm | 294 s |
| 标记间距 0.6 m + 横移比例估计（默认） | 1.5 mm（平均 0.7 mm） | 295 s |

按行程回到标记重新对位（行为树执行器参数 `realign_distance`，[7.3](#73-执行)）对这种方向固定的比例误差无效（阈值 0.3 m 时 17.5 mm、+48 s），默认关闭。

在世界中加新的标记：用 `src/scr_description/scripts/generate_apriltag_model.py --id <N>` 生成模型（默认输出到 `scr_description/models/`），在世界文件中按 ID 0 的姿态（`roll = π/2`、`yaw = π`）摆放，再把它的 `u` 写进地图。

**早期的站间语义**（单标记、站间只用里程计外推；复现系统层现代化之前的多站数据时使用）：

```bash
sed -e 's/tag_map: tag_map.yaml/tag_map: ""/' -e 's/estimate_scale: true/estimate_scale: false/' \
    -e 's/strafe_tag_updates: true/strafe_tag_updates: false/' \
    $(ros2 pkg prefix scr_arm_controller)/share/scr_arm_controller/config/fine_alignment.yaml \
    > /tmp/fine_alignment_single_tag.yaml
ros2 launch scr_bringup sim_nav.launch.py gui:=false alignment_params:=/tmp/fine_alignment_single_tag.yaml
```

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

不需要 ROS。输出包含每条槽的直角折线、按执行顺序排列的停靠站、每站底盘位姿与机械臂切割点、预测执行时间。

| 选项 | 默认 | 说明 |
|---|---|---|
| `--routes=rule\|joint` | `rule` | `rule`：逐条槽按规范代价选线后划分停靠站；`joint`：槽线与停靠站联合规划（[task_planning](experiments/task_planning/README.md)） |
| `--partition=dp\|greedy\|per_slot` | 任务文件 `options.partition`，未设时 `dp` | 停靠站划分 |
| `--order=sweep\|slot_by_slot` | 任务文件 `options.order`，未设时 `sweep` | 访问顺序 |
| `--time-model=align_v3\|align_v2\|align_v1` | `align_v3` | 预测执行时间所用的时间模型预设（[7.4](#74-执行时间模型)）；也可在任务文件 `options.time_model` 中指定 |

也可以运行 ROS 节点 `wall_slot_planner_node.py`（参数 `task_file`、`output_file`），它额外在 `/wall_slot_plan/markers` 发布 RViz 显示。

### 7.3 执行

```bash
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01
```

客户端识别到规划文件（含 `wall` 和 `stations`）就把它发给行为树执行器的 `/execute_slot_plan`。行为树 [`slot_plan.xml`](../src/scr_task/behavior_trees/slot_plan.xml) 的流程：Nav2 到定位标记前 1.2 m → 精对位到 0.70 m（EKF 估计的标记位置作为墙面坐标原点）→ 逐站：（按行程重新对位开启且到期时，先横移回参考标记前重新精对位）→ 机械臂回到观测位姿，沿墙横移到站（站间距离超过 `--max-strafe` 时先缩回机械臂、用 Nav2 过去）→ 定点保持下测量残余误差、补偿机械臂点并切割 → 记录本站 → 全部完成后缩回机械臂。横移与到站测量中，画面里的地图标记持续更新沿墙位置（[6.4](#64-定位标记地图与横移比例估计)）。

恢复策略：导航失败重试一次；精对位失败时缩回机械臂、回到标记前再对位；到站失败（未收敛且残差超出机械臂补偿范围）时回到标记前重新对位、重新建立墙面原点后再到站；切割失败重试一次。仍失败时任务失败，结果给出失败的停靠站与节点。客户端在结束时打印每站耗时、到站残差与恢复次数。

| 选项 | 默认 | 说明 |
|---|---|---|
| `--dry-run-gap=<m>` | 不设 | 空切：切深处的点改为刀尖停在墙前该距离（仿真中统一用 0.01） |
| `--max-strafe=<m>` | 1.5 | 站间距离超过该值时，先缩回机械臂、退到离墙 1.2 m 用 Nav2 过去 |
| `--no-compensation` | — | 关闭到站后的残差补偿 |
| `--bt=<tree.xml>` | `slot_plan.xml` | 换用其他行为树 |

**按行程重新对位**：行为树在每站开始前检查“自上次接受标记更新以来的沿墙里程计行程 + 到下一站的横移”是否超过阈值，超过时先回参考标记前重新精对位（`RealignDue` 条件与 `ScheduledRealign` 子树）。阈值是行为树执行器的参数 `realign_distance`（米，默认 0 = 关闭，[`task_executor.yaml`](../src/scr_task/config/task_executor.yaml)），也可在目标的 `realign_distance` 字段中逐次给出（0 = 用执行器参数，负值 = 本次关闭）。执行器只在配置阶段读取该参数，运行中用 `ros2 param set` 修改不生效；`navigation_cutting_task` 没有对应选项，逐次设置用下面的实验记录器。计划内的重新对位记入 `StationRecord.realigned`，不计入恢复次数。它只消除随行程累积的随机误差，对方向固定的横移比例误差无效（[6.4](#64-定位标记地图与横移比例估计)），默认配置不需要开启。

需要真值评估时使用实验记录器。它是同一 action 的客户端，另外订阅 Gazebo 真值、技能服务器的阶段话题 `/task_skills/status` 与墙面状态估计 `/task_skills/wall_estimate`，记录刀尖轨迹、到站误差、底盘漂移与估计过程。选项与上表相同，另有：

| 选项 | 默认 | 说明 |
|---|---|---|
| `--realign-distance=<m>` | 不设（用执行器参数） | 填入目标的 `realign_distance`；负值关闭 |
| `--marker-x=<m>`、`--marker-y=<m>` | 8.42、−3.9245 | 参考标记的 map 坐标，用于把真值换算到以标记为原点的墙面坐标 |

```bash
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run --dry-run-gap=0.01
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run_realign --dry-run-gap=0.01 --realign-distance=0.3
```

输出：

- `stations.csv`：逐站耗时、到站真值误差、传感器残差、恢复次数、是否计划内重新对位（`realigned`）、到站时的无标记行程（`since_fix_m`）、本站接受过更新的标记（`tags_used`、`tag_updates`）；
- `ekf_trace.csv`：每个估计周期的墙面相对状态 `(d, u, ψ)`、σ、横移比例、画面中的标记，以及同一时刻的真值；
- `tool_trace.csv`（刀尖真值轨迹）与 `summary.yaml`（含重新对位次数、最终的横移比例估计）。

示例任务（两条槽、11 站）在默认配置下约 295 s 仿真时间（横移前回观测位姿每站约 2 s）。

**精度与里程计**：激光只观测离墙距离与朝向，站间的沿墙位置靠标记与里程计。默认配置（编码器误差模型 + EKF，标记间距 0.6 m + 横移比例估计）下 11 站的最大到站沿墙误差为 1.5 mm（平均 0.7 mm），补偿后刀尖沿墙误差最大 1.1 mm，刀尖轨迹误差 95 分位 0.23 mm；只用一个标记、站间只靠里程计外推时，误差随离标记的距离线性增长到 24.7 mm。早期配置（`base_control:=plugin`，`/odom` 为真值）下为 0.2–0.3 mm 量级。详见 [沿墙精度报告](experiments/along_wall_accuracy/README.md)。

**录制数据包**：需要事后作图或回放估计过程时，在任务开始前另开终端录制 rosbag2（录到仓库之外，每次 11 站运行约 150–180 MiB）：

```bash
ros2 bag record -o ~/scr_bags/run1 --use-sim-time \
    /odom /wheel/odom /ground_truth/odom /task_skills/wall_estimate /task_skills/status \
    /task_executor/station_records /cmd_vel /joint_states /imu /scan /tf /tf_static /clock
```

读取包中的 `scr_interfaces` 消息需要先 `source install/setup.bash`。相机图像约 27 MB/s，一般不录；行为树 action 的反馈属于隐藏话题，逐站结果从 `station_records` 读取。沿墙精度实验的 `run_experiment.py <配置> --bag <目录>` 与规划验证的 `run_validation.py ... --bag <目录>`（[8.3](#83-演示录制与数据包回放)）录制同样的话题，后者另录 `/gazebo/link_states`、`/plan`、`/map` 与 `/robot_description`，可在 RViz 中回放。

### 7.4 执行时间模型

规划器用时间模型预测执行时间：每站耗时 = `t_settle` + 横移距离 / `v_strafe` + 点数 × `t_point`。模型由 Gazebo 执行记录标定，标定结果保存为命名预设（`slot_plan.TIME_MODELS`）：

| 预设 | 对应的执行路径 | 标定方法 | `t_settle` | `v_strafe` | `t_point` |
|---|---|---|---|---|---|
| `align_v3`（默认） | 当前默认：行为树任务层，标记地图 + 横移比例估计，横移前回观测位姿（44 站，每站 RMSE 0.22 s） | `phases-reg` | 4.70 s | 0.0971 m/s | 2.030 s |
| `align_v2` | 行为树改造之前、精对位第二版、单标记，`/odom` 为真值 | `phases` | 2.83 s | 0.0984 m/s | 2.049 s |
| `align_v1` | 第一版精对位，用于复现早期报告的数字 | `joint` | 4.40 s | 0.0945 m/s | 2.034 s |

`plan_wall_slots.py`、`task_planning_benchmark.py` 与 `task_planning_report.py` 都接受 `--time-model`，默认 `align_v3`。在当前默认路径上，`align_v3` 对 5 次未参与标定的运行（联合规划、规则路线与 11 站规划）预测各站耗时之和的误差为 −1.7 … +0.7 s（≤ 0.7 %）；`align_v2` 每次运行低估 18–21 s，只用于复现此前的报告（[plan_validation](experiments/plan_validation/README.md)）。

用新的执行记录（实验记录器的结果目录）重标定：

```bash
python3 src/scr_planner/scripts/calibrate_time_model.py /tmp/run_a /tmp/run_b \
    --method=phases-reg --compare=align_v1,align_v2,align_v3 --loo
```

| 选项 | 说明 |
|---|---|
| `--method=joint`（默认） | 三个参数对每站总耗时整体最小二乘 |
| `--method=phases` | 分阶段：横移阶段回归，`t_point` = 切割总时间 / 总点数，其余为固定开销均值；对少数站上的偶发停顿不敏感（`align_v2` 用此法） |
| `--method=phases-reg` | 同 `phases`，但切割阶段也回归 `cut_s = c_cut + 点数 × t_point`，截距计入每站固定开销（`align_v3` 用此法） |
| `--compare=<预设,...>` | 在同一批数据上评估已有预设的每站 RMSE、平均偏差与各站之和的误差 |
| `--loo` | 逐次运行留一交叉验证的每站 RMSE |

当前路径改用 `phases-reg` 的原因：横移前机械臂回到观测位姿后，每站切割阶段先要从观测位姿移到第一个切割点，这段约 1.0 s 的运动与点数无关。比值估计把它摊进 `t_point`（2.152 s，偏大 6 %），会高估“每少一个点省多少时间”，每站 RMSE 0.55 s；回归后 `t_point` = 2.030 s，每站 RMSE 0.22 s，与整体最小二乘一致。执行路径或仿真配置改变后应重新标定。

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

- 墙面实体由 Gazebo 的 `gazebo_ros_factory` 生成与删除，服务名为 `/spawn_entity`、`/delete_entity`（根命名空间，没有 `/gazebo` 前缀）。
- 演示相机的图像话题为 `/<name>/<name>/image_raw`（`gazebo_ros_camera` 发布在“插件命名空间 / 相机名”下），默认相机名 `demo_camera`，即 `/demo_camera/demo_camera/image_raw`；`demo_record` 默认订阅它，`--no-spawn --topic` 可改用已有相机。
- 视角预设：`oblique`（斜后方俯视墙面与机械臂，机器人在横移范围两端时刀具与标记都不被立柱遮挡）、`overview`（7.5 m 高的近似俯视，覆盖起点与 room3 南墙，看导航全程）、`front`、`side`、`top`，或用 `--eye X Y Z --target X Y Z` 指定。同时开多台相机时用 `--name` 区分。

二者都已在运行中的 Gazebo 里验证（6 个墙面实体与两台相机，执行不受影响），见 [demo 报告](experiments/demo/README.md)。详见 [`src/scr_viz/README.md`](../src/scr_viz/README.md)。

### 8.3 演示录制与数据包回放

完整的多站开槽演示（[`slot_demo.gif`](experiments/demo/slot_demo.gif)）由三步生成：执行并录制、合成动画、在 RViz 中回放。在 4 GB 内存的机器上，Gazebo + Nav2 + 任务层 + 两台演示相机 + rosbag2 录制可以同时运行，但再开 RViz 内存不够；此时不要让 RViz 与仿真同时运行，而是录制数据包，事后回放到 RViz。

```bash
# 1. 执行一份规划：无界面仿真 + 任务层 + 墙面实体 + 两台演示相机 + 数据包 + 真值记录（约 6 min 墙钟）
python3 docs/experiments/plan_validation/run_validation.py \
    docs/experiments/plan_validation/plans/plan_joint.yaml /tmp/demo_run --seed 1 \
    --fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml \
    --camera ~/scr_media/demo_frames --presets overview oblique \
    --bag ~/scr_bags/demo_run
# 2. 合成 GIF / MP4 与截图：上为演示相机（导航阶段 overview，到墙前后 oblique），下为墙面立面图
python3 docs/experiments/demo/make_slot_demo.py /tmp/demo_run ~/scr_media/demo_frames \
    --task src/scr_viz/config/wall_task_room3_south_demo.yaml --plan /tmp/demo_run/slot_plan.yaml \
    --gif /tmp/slot_demo.gif --mp4 /tmp/slot_demo.mp4 --stills /tmp/demo_stills
# 3. 回放数据包到 RViz：plan_viz 从回放的 /gazebo/link_states 推算刀尖轨迹，按仿真时间截图
python3 docs/experiments/demo/replay_rviz.py ~/scr_bags/demo_run \
    --task src/scr_viz/config/wall_task_room3_south_demo.yaml --plan /tmp/demo_run/slot_plan.yaml \
    --out /tmp/rviz --at 60 170 --rate 2
```

- `run_validation.py`：仿真与规划之外的设置全部取默认值，`--seed` 为编码器打滑种子；`--fixtures` 在执行前生成墙面实体；`--camera <帧目录> --presets ...` 每个预设开一台演示相机，帧存到 `<帧目录>/<预设>/`（`--rate` 每仿真秒帧数，默认 2）；`--bag` 同时录制数据包。仿真隔离在 `ROS_DOMAIN_ID = 48`，结束时清理本次启动的全部进程与 Fast DDS 共享内存。Nav2 组件偶尔加载超时时，启动看门狗（`--startup-timeout`，默认 240 s）判为启动失败并整次重跑（`--retries`，默认 2 次），记录写入 `startup_failures.txt`。
- `make_slot_demo.py`：读取结果目录中的 `tool_trace.csv[.gz]`、`summary.yaml` 与相机帧，按阶段以不同倍速抽帧（`--speed-nav` / `--speed-align` / `--speed-cut`，默认 10 / 4 / 10 倍），需要 ffmpeg。
- `replay_rviz.py`：用包内的 `/clock` 回放，`plan_viz` 与 RViz 都用仿真时间；`--at` 为相对包内第一条 `/clock` 的仿真秒数，到达时抓取 RViz 窗口存为 `<目录>/rviz_<t>.png`，结束时另存 `rviz_end.png`。需要桌面会话（`DISPLAY`，默认 `:0`）与 `xwd`、ImageMagick `convert`；Wayland 下 RViz 走 XWayland，可以直接抓取。

不用脚本时手动回放（包内已录 `/clock`，不需要 `--clock`）：

```bash
ros2 launch scr_viz plan_viz.launch.py task_file:=<任务.yaml> plan_file:=<plan.yaml> use_sim_time:=true   # 含 RViz
ros2 bag play ~/scr_bags/demo_run
```

---

## 9. 测试与实验复现

```bash
colcon build && colcon test && colcon test-result --verbose
colcon test --packages-select scr_planner     # 只测单个包
```

测试包括 C++ gtest（规划器、麦克纳姆运动学与编码器误差模型、行为树结构与任务文件解析）、pytest 单元测试和 flake8 / pep257 代码风格检查；GitHub Actions 在 `ros:humble` 容器中执行同样的编译与测试。行为树结构测试用模拟技能节点运行两棵树，不需要 Gazebo。

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
| 行为树编排、Nav2 MPPI / Smac、定位融合、底盘 ros2_control | 是 | [system_modernization](experiments/system_modernization/README.md) |
| 多站沿墙精度：标记地图、横移比例估计、按行程重新对位 | 是 | [along_wall_accuracy](experiments/along_wall_accuracy/README.md) |
| 时间模型 `align_v3` 标定；联合规划与规则路线的 Gazebo 执行 | 标定部分否 | [plan_validation](experiments/plan_validation/README.md) |

注意各报告所用的仿真配置：系统层现代化之前的实验（精对位、多站执行、时间模型）在早期配置（`base_control:=plugin`，`/odom` 为真值，NavFn + DWB）下取得，复现时按 [2.4](#24-底盘里程计与定位) 设置启动参数；沿墙精度实验之前的多站数据还要用早期的站间语义（单标记、站间只用里程计，[6.4](#64-定位标记地图与横移比例估计)）。各报告的“复现”一节给出了对应的命令。

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
ros2 control list_controllers          # 应有 joint_state_broadcaster / arm_controller / base_controller，均为 active
ros2 topic hz /wheel/odom              # 底盘控制器的轮式里程计，约 50 Hz
ros2 topic hz /odom                    # EKF 输出（localization:=ekf）
```

控制器在仿真启动 5 s 后才加载。若控制器加载失败，查看 gzserver 输出中的 ros2_control 报错，确认已安装 `ros-humble-gazebo-ros2-control`，并确认 `scr_base_control` 已编译（`base_controller` 与仿真硬件接口都由它的插件提供）。`/cmd_vel` 超过 0.5 s 没有新指令时底盘会停车，`ros2 topic pub --once` 只能让底盘动一下，持续运动用 `--rate 10`。

早期配置（`base_control:=plugin`）下控制器列表为 `joint_state_broadcaster / arm_controller / wheel_controller`，`/cmd_vel_direct` 应有 `mecanum_controller` 发布、`planar_move` 订阅。

### 10.3 机械臂不动或报“不可达”

检查点是否在笛卡尔工作空间内（见 [3.2](#32-机械臂)）。`custom_cutting` 与任务程序都在执行前做可达性检查，任何一点越界整段不执行。

### 10.4 对位不收敛或检测不到定位标记

```bash
ros2 topic hz /scan
ros2 topic hz /front_camera/image_raw
ros2 run rqt_image_view rqt_image_view /front_camera/image_raw
```

对位需要激光看到墙面、相机看到定位标记（相机水平视场 80°）。导航目标离标记太远或偏角太大时，标记可能不在视野内；此时调整任务文件中的 `navigation` 位姿，使机器人停在标记正前方 1–2 m 处。对位在 50 s 仿真时间内未收敛时如实报告失败。

多站执行中，技能服务器的日志或 `/task_skills/wall_estimate` 的 `tag_ids` 为空、`tag_state` 报告未检测到标记时：确认 `alignment.observe_between_stations` 为 `true`（切割位姿下滑台较低，0.70 m 高的标记落在画面上沿之外），并确认画面中的标记 ID 在标记地图中（地图之外的 ID 被忽略）。

### 10.5 TF 查询失败

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

正常的 TF 树为 `map → odom → base_link → link1 → … / camera_link → camera_optical_link / lidar_link`。`map → odom` 由 AMCL 发布（只启动仿真时没有）；`odom → base_link` 默认由 EKF（`ekf_filter_node`）发布，`localization:=wheel` 时由 `base_controller` 发布，`base_control:=plugin` 时由 `scr_planar_move` 发布，同一时刻只能有一个发布者；其余由 `robot_state_publisher` 发布。对位和扫描只需要 `base_link` 以下的 TF，不依赖 AMCL。

### 10.6 Docker 中 RViz 报 GLSL 错误

容器内 OpenGL 兼容性问题，使用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

`docker/docker-compose.yml` 的 `gui` 服务已默认设置该变量（[11.5](#115-图形界面可选)）。

### 10.7 多个仿真互相干扰

同一台机器上同时运行多套仿真或残留进程时，话题会互相串扰。关闭后确认没有残留进程（`pgrep -af "gzserver|robot_state_publisher|mecanum_controller"`），或为每套仿真设置不同的 `ROS_DOMAIN_ID`。

### 10.8 关闭仿真时 `mecanum_controller` 报错退出

只出现在早期配置（`base_control:=plugin`）下：按 Ctrl-C 关闭仿真时，launch 会显示 `mecanum_controller` 以非零码退出。这是该节点未处理关闭信号导致的退出报错，不影响仿真和实验结果，已列入 [路线图](ROADMAP.md)。

### 10.9 客户端一直显示“等待行为树执行器”

行为树执行器在 `nav_delay`（默认 8 s）后启动，并等待 Nav2 的 `bt_navigator` 激活与技能服务器就绪后才提供 `/execute_slot_plan`、`/execute_cutting_task`。

```bash
ros2 action list | grep -E "execute_|task_skills"   # 应有 2 个任务 action 与 6 个技能 action
ros2 lifecycle get /bt_navigator                    # 应为 active
```

- 启动时设置了 `task:=false`：任务层没有启动，改为 `task:=true` 或另行运行 `ros2 launch scr_task task.launch.py`；
- Nav2 没有激活：见 [10.11](#1011-nav2-停在生命周期配置阶段)；
- 技能服务器没有就绪：查看 `task_skills` 节点的日志，并确认控制器均为 active（[10.2](#102-机器人不响应-cmd_vel)）；
- 目标被拒绝（`目标被行为树执行器拒绝`）：执行器无法加载行为树，常见原因是 `--bt=` 给出的 XML 路径不存在或树中使用了未注册的节点，原因见执行器日志。

### 10.10 多站执行中沿墙误差逐站增大、触发重新对位

沿墙误差随离参考标记的距离线性增长，说明看不到标记的那几站在用未修正的里程计外推：编码器的横移打滑约 3 %，只用一个标记、不估计横移比例时最远站约 25 mm（[6.4](#64-定位标记地图与横移比例估计)）。到站残差超出机械臂的补偿范围（横向 20 mm、离墙 10 mm、朝向 1.5°）时，行为树会回到标记前重新对位，任务耗时随之增加。排查：

- 确认用的是默认精对位参数：启动日志中技能服务器应报告加载了标记地图（标记个数与最大间距）；`/task_skills/wall_estimate` 的 `scale_estimated` 为 `true`，`lateral_scale` 在几站之后收敛到约 −3 %（`sigma_scale` 约 0.3 %）；
- 看 `stations.csv` 的 `tags_used` 与 `since_fix_m`：某些站没有用到标记、无标记行程大，说明这一段没有可见标记，应缩小标记间距（[6.4](#64-定位标记地图与横移比例估计)）；
- 确认技能服务器的局部闭环用的是 `/wheel/odom`（启动日志中技能服务器的 `odom_topic`）。若用 EKF 的 `/odom`，定点保持会随 EKF 在慢速运动下的偏差一起漂移；
- 需要排除里程计误差时用 `odom_error:=false` 或早期配置 `base_control:=plugin`。

按行程重新对位（`realign_distance`）不能修正这种误差，只增加耗时。

### 10.11 Nav2 停在生命周期配置阶段

`use_composition:=False`（每个 Nav2 服务器一个进程）时偶尔停在生命周期配置阶段，同时出现 Fast DDS 共享内存端口错误。关闭全部进程后清理共享内存再重新启动：

```bash
pgrep -af "gzserver|robot_state_publisher|component_container|ekf_node|task_executor"   # 确认没有残留进程
fastdds shm clean
```

### 10.12 机械臂动作时底盘缓慢转动或漂移（早期配置）

Gazebo 中底盘有一部分转动（零速度指令下约 0.08°/s）只出现在位姿里、不出现在连杆速度里。默认的 ros2_control 底盘用位姿差分得到的实际速度做速度闭环、生成编码器与陀螺读数，因而不受影响；早期配置（`base_control:=plugin`）仍有这一现象，由定点保持在位置环中修正。排查过程见 [系统层现代化报告](experiments/system_modernization/README.md#24-轮式里程计陀螺与-ekf) 2.4 节。

---

## 11. Docker 复现

不在本机安装 ROS 2 与 Gazebo 时，用 `docker/` 中的镜像复现。需要 Docker Engine 与 Compose 插件（`docker compose`）；以下命令都在仓库根目录执行。

### 11.1 镜像内容

| 文件 | 作用 |
|---|---|
| [`docker/Dockerfile`](../docker/Dockerfile) | 基于官方镜像 `ros:humble`。依赖由各包的 `package.xml` 经 rosdep 安装（与 GitHub Actions 中相同的一组），随后 `colcon build`。依赖层只依赖 `package.xml`，修改源码后重新构建时不会重装依赖 |
| [`docker/entrypoint.sh`](../docker/entrypoint.sh) | 容器入口：加载 ROS 2 与工作空间的环境；没有 `DISPLAY` 时启动虚拟显示 Xvfb |
| [`docker/docker-compose.yml`](../docker/docker-compose.yml) | 两个服务：`sim`（无界面）与 `gui`（Gazebo 与 RViz 窗口） |
| `.dockerignore` | 构建上下文只包含 `src/`、`docs/` 与入口脚本 |

工作空间在容器内的 `/ws`：源码 `/ws/src`，编译结果 `/ws/build`、`/ws/install`，实验数据与脚本 `/ws/docs`（部分单元测试读取 `docs/experiments` 中的执行记录）。容器的工作目录是 `/ws`，本指南中以 `src/...`、`docs/...` 开头的相对路径在容器内可以直接使用。

rosdep 之外只安装了 `xvfb`：Gazebo Classic 用 OGRE 渲染相机图像，即使不开 Gazebo 窗口也需要一个 X 显示；容器里没有显示时，前视相机不发布图像，精对位看不到定位标记。入口脚本因此在没有 `DISPLAY` 时启动 Xvfb，由 Mesa 软件渲染。环境变量 `SCR_XVFB=0` 可关闭这一行为。

### 11.2 构建

```bash
docker compose -f docker/docker-compose.yml build
```

得到镜像 `scr-sim:humble`。等价的命令是 `docker build -f docker/Dockerfile -t scr-sim:humble .`（构建上下文必须是仓库根目录）。首次构建需要下载 Gazebo、Nav2、RViz 等依赖；修改 `src/` 后重新构建只重新执行 `colcon build`。

### 11.3 无界面运行多站开槽任务

```bash
docker compose -f docker/docker-compose.yml up -d sim        # 仿真 + Nav2 + 任务层，无窗口
docker compose -f docker/docker-compose.yml exec sim bash    # 进入容器，环境已加载
```

`sim` 服务执行的是 `ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false`（[2.1](#21-一键启动推荐)）。在容器内执行与 [第 7 节](#7-多站开槽离线规划--逐站执行) 相同的命令：

```bash
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_planner/config/wall_task_room3_south.yaml /tmp/plan.yaml
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01
```

客户端先等待行为树执行器就绪（Nav2 激活之后），再执行 11 个停靠站并打印每站记录；任务成功时退出码为 0。单站任务同理：`ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml`。

查看仿真侧的日志与结束：

```bash
docker compose -f docker/docker-compose.yml logs -f sim
docker compose -f docker/docker-compose.yml down
```

不进入容器、直接执行单条命令时，在命令前加入口脚本以加载环境：

```bash
docker compose -f docker/docker-compose.yml exec sim /ros_entrypoint.sh ros2 action list
```

### 11.4 运行测试

```bash
docker compose -f docker/docker-compose.yml run --rm sim bash -c "colcon test && colcon test-result --verbose"
```

与 [第 9 节](#9-测试与实验复现) 相同的全部测试，在镜像内已编译的工作空间上运行，不启动 Gazebo。

### 11.5 图形界面（可选）

`gui` 服务启动完整界面（`ros2 launch scr_bringup sim_nav.launch.py`），把 Gazebo 与 RViz 的窗口显示到宿主机的 X 服务器上：

```bash
xhost +local:docker                                                   # 允许容器连接 X 服务器
docker compose -f docker/docker-compose.yml --profile gui up gui
docker compose -f docker/docker-compose.yml --profile gui down        # 结束后
xhost -local:docker
```

- **X11**：服务挂载 `/tmp/.X11-unix` 并传入宿主机的 `DISPLAY`（未设置时取 `:0`）。
- **Wayland**：Gazebo Classic 与 RViz 都是 X11 程序，在 Wayland 桌面上经 XWayland 显示；`DISPLAY` 由 XWayland 设置，命令与 X11 相同。
- **渲染**：默认 `LIBGL_ALWAYS_SOFTWARE=1`（软件渲染，任何宿主机都可用）。要用宿主机的显卡时，以 `LIBGL_ALWAYS_SOFTWARE=0` 启动，并取消 compose 文件中 `/dev/dri` 一段的注释。
- 在该服务中执行任务：`docker compose -f docker/docker-compose.yml exec gui bash`，命令与 11.3 相同。
- `sim` 与 `gui` 各自都是完整的系统，一次只启动其中一个；`gui` 使用 `ROS_DOMAIN_ID=1`，两者同时运行时话题也不会互相串扰。

### 11.6 持续集成中的镜像检查

GitHub Actions 的 `Docker` 工作流（`.github/workflows/docker.yml`）在 `src/` 或 `docker/` 变化时构建镜像（各层缓存在 Actions 缓存中），并在容器内依次执行：compose 文件检查、11.4 的全部测试、一次无界面启动（等到前视相机发布一帧图像、任务 action `/execute_slot_plan` 出现）。手动触发该工作流并勾选 `demo` 时，另外执行 11.3 的完整多站开槽任务。

---

## 附录：坐标系、话题与机器人参数

**坐标系**：`base_link` 为 X 向前、Y 向左、Z 向上；`base_link` 离地 0.1025 m。墙面坐标 `(u, v)`：面向墙面时 `u` 沿墙向右，`v` 为离地高度。

**主要话题与接口**

| 名称 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（Nav2 与技能服务器） |
| `/odom` | `nav_msgs/Odometry` | 导航用里程计：默认为 EKF 输出（50 Hz）；`localization:=wheel` 时为轮式里程计；`base_control:=plugin` 时为真值 |
| `/wheel/odom` | `nav_msgs/Odometry` | 轮式里程计（`base_controller`，50 Hz，含编码器误差），技能的局部闭环使用 |
| `/imu` | `sensor_msgs/Imu` | 模拟陀螺（只有角速度，100 Hz） |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 底盘真值（`gazebo_ros_p3d`），只用于评估 |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达（360 线，测距噪声 σ = 1 cm） |
| `/front_camera/image_raw`、`/front_camera/camera_info` | `sensor_msgs/Image`、`CameraInfo` | 前视相机，640 × 480，水平视场 80°，30 Hz |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 机械臂轨迹 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 导航 |
| `/execute_cutting_task`、`/execute_slot_plan` | `scr_interfaces/action/ExecuteCuttingTask`、`ExecuteSlotPlan` | 行为树执行器的任务 action |
| `/task_skills/fine_align` 等 6 个 | `scr_interfaces/action/*` | 技能 action（`fine_align`、`move_to_station`、`cut_station`、`hold_base`、`scan_marker`、`retract_arm`） |
| `/task_skills/status` | `scr_interfaces/msg/TaskStatus` | 技能服务器的当前阶段与停靠站序号 |
| `/task_skills/wall_estimate` | `scr_interfaces/msg/WallEstimate` | 技能服务器的墙面相对状态估计（每个估计周期一次）：`(d, u, ψ)`、σ、横移比例、无标记行程、画面中的标记 |
| `/task_executor/station_records` | `scr_interfaces/msg/StationRecord` | 每站执行记录 |
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
| 定位标记 | room3 南墙（y = −3.925 m）上的 AprilTag tag36h11 ID 0–5，黑框 0.10 m，中心离地 0.70 m：参考标记 ID 0 在 x = 8.42 m，ID 5、1、2、3、4 在沿墙 u = −0.3、0.3、0.6、0.9、1.2 m（面向墙面向右为正，即 x 减小方向）；默认地图使用 ID 0、2、4。第一版使用的 5 cm 黄色色块（x = 8.42 m，中心离地 0.60 m）保留作对比 |
