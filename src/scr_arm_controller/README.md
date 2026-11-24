# scr_arm_controller

机械臂运动学与控制、墙面精对位（AprilTag + 激光 EKF + 饱和控制律）、底盘定点保持，以及切割任务的技能服务器 `task_skill_server`：精对位、墙面坐标横移、定点保持、残差补偿与切割等步骤以 ROS 2 action 的形式提供，由 [`scr_task`](../scr_task/README.md) 的行为树编排。命令行入口 `navigation_cutting_task` 把任务文件发给行为树执行器。

## 模块

| 模块 | 内容 | 依赖 ROS |
|---|---|---|
| `cartesian_cutting.py` | `CartesianCuttingController`：闭式正 / 逆运动学、可达性检查、笛卡尔 / 关节空间运动 | 是 |
| `task_skills.py` | 技能库 `CuttingSkills`：精对位（三种模式）、墙面坐标横移 / 接近、到站残差测量、定点保持下的机械臂点执行、标记扫描、激光防碰撞守护、空切；可由多线程执行器驱动，也可被实验脚本顺序调用 | 是 |
| `skill_server.py` | `task_skill_server`：把 `CuttingSkills` 的技能包装成 6 个 action 服务器，发布阶段话题 `/task_skills/status` | 是 |
| `slot_execution.py` | 纯函数：识别与解析多站开槽规划文件、每站机械臂点、到站残差补偿、残差是否可补偿的判据 | 否 |
| `navigation_cutting_task.py` | 命令行入口（action 客户端）：单站任务 → `/execute_cutting_task`，规划文件 → `/execute_slot_plan`；打印反馈与每站记录 | 是 |
| `fiducial.py` | 纯算法：AprilTag（tag36h11）检测，边缘直线求交的亚像素角点，IPPE PnP 求标记中心与协方差 | 否 |
| `wall_ekf.py` | 纯算法：墙面相对状态 (d, u, ψ) 的 EKF，里程计增量预测，激光与标记更新（标记地图中的沿墙偏移 `u_i`，位置误差计入测量噪声），χ² 门限；可选第四个状态：里程计横移比例修正 `s`（真实横移 = (1 + s) × 里程计横移），重新对位时保留 | 否 |
| `tag_map_slam.py` | 纯算法：标记地图在线估计 `TagMapEKF`（非参考标记的沿墙位置作为 EKF 状态，地图与横移比例在重新对位时保留），带协方差的地图读写，批量最小二乘的交叉检查；任务结束与每次残差测量后写出地图；由 `alignment.estimate_tag_map` 打开，默认关闭（[报告](../../docs/experiments/tag_map_estimation/README.md)） | 否 |
| `tag_map.py` | 纯算法：标记地图 `TagMap`（标记 ID → 沿墙位置 `u` 与位置误差），YAML 读写与校验、子地图、加扰动（实验用）、相对路径解析 | 否 |
| `wall_estimator_ros.py` | EKF 的 ROS 封装：按时间戳经里程计历史补偿测量延迟（估计横移比例时按修正后的增量），画面中每个地图内标记都参与更新，累计无标记更新的沿墙行程 `odom_since_fix`，每个估计周期发布 `/task_skills/wall_estimate` | 是 |
| `docking_control.py` | 纯算法：平方根整形饱和比例律、速度 / 加速度限幅、收敛判据（误差 + 静止 + 估计不确定度） | 否 |
| `teach_alignment_offset.py` | 示教零位标定：在真值参考位姿处求估计器的静态偏置，输出 ROS 参数文件 | 是 |
| `visual_servo.py` | 纯算法（第一版精对位，保留为基线）：HSV 定位标记检测、激光墙面拟合（总体最小二乘 + 离群点剔除）、像素反投影、射线与墙面求交、PBVS 误差与全向底盘速度指令 | 否 |
| `visual_servo_ros.py` | `WallMarkerServo`：订阅激光、图像、相机内参，查询 TF，输出滤波后的墙面与标记测量（检测画面中全部地图内 ID）；包含主点修正 | 是 |
| `base_hold.py` | `BaseHold`：定点保持，每收到一帧里程计（50 Hz）计算一次指令，并用真值记录保持期间的最大漂移；参数 `hold.mode` 选择控制律 `p`（默认）/ `pi` / `pid` / `dob` / `lqg`，`hold.feedforward_topic` 接收预计的切削力旋量作前馈（Gazebo 中的验证见[报告](../../docs/experiments/cutting_force/README.md)第 10 节） | 是 |
| `arm_trajectory.py` | 整站机械臂连续轨迹的时间参数化：各直线段为静止到静止的 S 形（或梯形）速度曲线，切割段按进给、空行程按各轴限值，下刀后等待（[报告](../../docs/experiments/arm_trajectory/README.md)） | 否 |
| `cutting_force.py` | 切割力模型：刀片受力、底盘力旋量、轮压与所需摩擦系数，以及由主轴功率、轮地摩擦与定点保持能力给出的进给上限 `feed_limit`（[报告](../../docs/experiments/cutting_force/README.md)） | 否 |
| `cut_schedule.py` | 一个停靠站的切削状态时间表：由连续轨迹给出每个时刻的切深、进给与预计的底盘力旋量；下刀段在接触点的拆分 | 否 |
| `disturbance_observer.py` | 定点保持的控制律：P / PI / PID / 扰动观测器 / LQG（`PlanarHold`）及增益设计 | 否 |
| `ibvs.py` | IBVS 与 2.5D 混合视觉伺服（交互矩阵、`IbvsController`、`HybridController`），用于方法对比 | 否 |
| `alignment_benchmark.py` | 对位鲁棒性实验：随机起点传送、里程计漂移与传感器噪声注入、真值评分（直接调用 `CuttingSkills`，不经过任务层） | 是 |
| `custom_cutting.py`、`square_demo.py` | YAML 点序列执行、关节空间演示 | 是 |
| `marker_scanner.py`、`trajectory_visualizer.py` | 早期的独立标记扫描节点；切割结束后在 Gazebo 中画出切割框 | 是 |
| `joint_direction_check.py` | 检查各关节在 `base_link` 中的实际运动方向（需仿真运行） | 是 |

## 技能服务器 `task_skill_server`

| Action | 类型（`scr_interfaces/action`） | 作用 |
|---|---|---|
| `/task_skills/fine_align` | `FineAlign` | 精对位到定位标记正前方 `standoff` 处，确定墙面坐标原点；结果含到位时的估计误差与原点残差 |
| `/task_skills/move_to_station` | `MoveToStation` | 墙面坐标闭环横移 / 接近到沿墙 `lateral_offset`、离墙 `standoff`（站间使用标记更新时先把机械臂移到观测位姿）；收敛或残差可由机械臂补偿（横向 20 mm、离墙 10 mm、朝向 1.5° 以内）时成功，结束后保持在到站位姿；结果含到站时的无标记行程 `odom_since_fix` |
| `/task_skills/cut_station` | `CutStation` | 测到站残差 → 补偿机械臂点 → 逐点执行；`station = -1`（单站任务）时另做可达性检查、前后缩回与空切后移 |
| `/task_skills/hold_base` | `HoldBase` | 定点保持，直到目标被取消 |
| `/task_skills/scan_marker` | `ScanMarker` | 相机射线与激光墙面求交检测定位标记，必要时按高度扫描 |
| `/task_skills/retract_arm` | `RetractArm` | 机械臂缩回到接近位姿 |

六个技能在同一进程中：精对位确定的墙面原点保存在墙面相对 EKF 中，之后的横移沿用同一个估计；横移收敛时开始的定点保持一直持续到切割结束。运动类技能（除 `HoldBase` 外）互斥，同一时刻只接受一个目标。阶段变化发布在 `/task_skills/status`（`TaskStatus`：`align / move / arrived / hold / cut / arm / scan` 与停靠站序号），实验记录器据此标注真值轨迹。墙面相对状态估计在每个估计周期发布到 `/task_skills/wall_estimate`（`WallEstimate`：`(d, u, ψ)` 与 σ、横移比例 `lateral_scale` 与 `sigma_scale`、`odom_since_fix`、画面中的地图内标记及其测量位置与是否被接受）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `odom_topic` | `/odom` | 局部闭环（精对位预测、横移外推、定点保持）使用的里程计。`sim_nav.launch.py` 在 `base_control:=ros2_control localization:=ekf` 时设为 `/wheel/odom`：`robot_localization` 的输出在 mm/s 级慢速运动下明显偏小，定点保持会被它带着漂移 |
| `alignment.*` | `config/fine_alignment.yaml` | 精对位参数，见下节 |
| `arm_motion.*` | `config/arm_motion.yaml` | 切割时的机械臂运动：`mode`（`continuous` 默认 / `point_to_point`）、进给与空行程限值、下刀后等待；每次执行 `CutStation` 时读取（[使用指南 7.4](../../docs/USER_GUIDE.md#74-机械臂运动方式)） |
| `hold.*` | `mode: p` | 定点保持：控制律 `mode`、指令限幅 `max_lin` / `max_ang`、观测器带宽 `observer_bandwidth`、前馈 `feedforward_topic` / `feedforward_gain` |

连续轨迹执行期间，技能服务器按轨迹时间表发布 `cutting_force/engagement`（`std_msgs/Float64`，当前切深，供 Gazebo 切割力插件使用）与 `cutting_force/expected_wrench`（`geometry_msgs/WrenchStamped`，名义切削力折算到底盘的力旋量，可作定点保持的前馈）。带切割力的一组参数（进给由切割力模型限制、扰动观测器 + 前馈、进给缓慢建立、切完后先停再退刀）在 [`config/arm_motion_cutting.yaml`](config/arm_motion_cutting.yaml)，默认不启用。

由 `scr_task/launch/task.launch.py` 启动（`sim_nav.launch.py` 默认包含）：

```bash
ros2 launch scr_task task.launch.py alignment_params:=<yaml> odom_topic:=/wheel/odom
```

## 精对位

对位模式由任务的 `mode` 选择：`visual`（第二版，默认）、`visual_v1`（第一版色块 + PBVS，作对比基线）、`odom`（里程计 + 已知标记坐标）。第二版参数在 [`config/fine_alignment.yaml`](config/fine_alignment.yaml)（节点参数 `alignment.*`，节点名通配 `/**`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `detector` | `apriltag` | `apriltag` 或 `blob`（第一版色块检测器） |
| `tag_family` / `tag_id` / `tag_size` | `tag36h11` / 0 / 0.10 m | 只有一个标记时（`tag_map` 为空串）使用；有地图时以地图的 `family`、`size`、`reference_id` 为准（`reference_id` 与 `tag_id` 不同时日志警告） |
| `offset_normal` / `offset_lateral` / `offset_yaw_deg` | 0 | 示教零位偏置（测量模型内扣除） |
| `tolerance_normal` / `tolerance_lateral` / `tolerance_yaw_deg` / `settle_cycles` | 2 mm / 2 mm / 0.3° / 10 | 收敛判据（另要求底盘静止、σ < 2 mm） |
| `k_lin` / `k_ang`、`decel_*`、`max_*`、`control_rate` | 2 /s、0.15 m/s² / 0.5 rad/s²、0.15 m/s / 0.3 rad/s、20 Hz | 平方根整形饱和律 |
| `timeout` / `lost_timeout` / `sigma_hold` | 50 s / 10 s / 0.01 m | 超时、标记持续丢失、停车等待阈值 |
| `fold_residual` | `true` | 多站执行以 EKF 估计的标记位置为墙面原点；`false` 以停车位姿为原点（第一版语义，消融用） |
| `tag_map` | `tag_map.yaml` | 标记地图；相对路径按本包 `config/` 解析，也可用绝对路径或 `~`；空串 = 只有 `tag_id` 一个标记 |
| `strafe_tag_updates` | `true` | 站间横移与到站残差测量时，画面中的地图标记继续更新沿墙位置；`false` 时站间沿墙位置只由里程计外推 |
| `observe_between_stations` | `true` | 横移前机械臂回观测位姿（滑台 −0.15 m、伸缩轴缩回，每站约 2 s），使标记处在画面内；只在 `strafe_tag_updates` 时生效 |
| `estimate_scale` | `true` | 在线估计里程计横移比例修正 `s`（EKF 第四个状态） |
| `scale_init` / `scale_sigma` | 0.0 / 0.05 | `s` 的先验均值与标准差（可填离线标定值）；`s` 的随机游走 0.002 /√m 在 `WallEKF` 中固定 |

### 标记地图

[`config/tag_map.yaml`](config/tag_map.yaml) 是默认地图（room3 南墙 ID 0、2、4，间距 0.6 m）：

```yaml
tag_map:
  family: tag36h11
  size: 0.10                 # 黑框边长（米）
  reference_id: 0            # 墙面坐标原点，u 必须为 0；缺省时取最小 ID
  sigma_position: 0.002      # 相对参考标记的位置误差（米，1σ）；可在单个标记上用 sigma 覆盖
  tags:
    - {id: 0, u: 0.0, v: 0.70}   # u：沿墙位置，面向墙面向右为正；v：离地高度，只作记录
    - {id: 2, u: 0.6, v: 0.70}
    - {id: 4, u: 1.2, v: 0.70}
```

参考标记的位置误差恒为 0（它定义原点）；ID 重复、参考标记不在地图中或 `u ≠ 0`、σ 为负时加载报错。标记 `i` 的测量模型为 `R(−ψ)·(d, u − u_i)`，位置误差按 `σ_map²·h_u h_uᵀ` 计入测量噪声，从一个标记过渡到下一个标记时不会因几毫米的不一致被门限拒收。间距的选择与各方案的对比见 [沿墙精度报告](../../docs/experiments/along_wall_accuracy/README.md) 与 [使用指南 6.4](../../docs/USER_GUIDE.md#64-定位标记地图与横移比例估计)：0.6 m 间距 + 横移比例估计时 11 站最大沿墙误差 1.5 mm（单标记、站间只用里程计时 24.7 mm）。

参数属于技能服务器，启动时用 `alignment_params:=` 传入：

```bash
# 示教零位（需先启动仿真，不需要 Nav2）；输出的参数文件在启动任务层时加载
ros2 run scr_arm_controller teach_alignment_offset --out /tmp/fine_alignment_taught.yaml --seconds 30
ros2 launch scr_bringup sim_nav.launch.py alignment_params:=/tmp/fine_alignment_taught.yaml
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
```

对 `navigation_cutting_task` 加 `--ros-args -p alignment.*` 不起作用：它只是客户端。

设计、控制律稳定性分析与实验见 [精对位报告](../../docs/experiments/fine_alignment/README.md) 与 [对位鲁棒性对比](../../docs/experiments/alignment_robustness/README.md)。

## 接口

技能服务器使用的话题与 action：

| 名称 | 方向 | 用途 |
|---|---|---|
| `/cmd_vel` | 发布 | 精对位、横移、定点保持 |
| `/arm_controller/follow_joint_trajectory` | action 客户端 | 机械臂运动 |
| `odom_topic`（`/odom` 或 `/wheel/odom`） | 订阅 | 墙面 EKF 预测、横移外推、定点保持 |
| `/task_skills/wall_estimate` | 发布 | 墙面相对状态估计（`scr_interfaces/msg/WallEstimate`），实验记录器写入 `ekf_trace.csv` |
| `/scan` | 订阅 | 墙面拟合、防碰撞守护 |
| `/front_camera/image_raw`、`/front_camera/camera_info` | 订阅 | 定位标记检测 |
| `/gazebo/model_states` | 订阅 | 真值，只用于评估（日志中的对位真值误差与保持漂移），不参与控制 |

导航（`NavigateToPose`）由行为树直接调用 Nav2，不经过技能服务器。

单站任务文件（`config/task_at_target.yaml`）中行为树执行器读取的字段：

| 字段 | 说明 |
|---|---|
| `navigation.{x, y, yaw}` | Nav2 目标（map 坐标） |
| `precise_positioning.enabled / mode` | 是否精对位；`visual`、`visual_v1` 或 `odom` |
| `precise_positioning.distance_from_marker` | 对位后底盘中心离墙距离，默认任务为 0.70 m |
| `scanning.enabled` | 是否在对位后检测标记三维位置 |
| `points`、`motion.{duration, return_home}` | 切割点（`base_link` 坐标，`rotation` 单位为度，至少 2 个）与运动参数 |
| `motion.dry_run_gap` | 空切间隙（米）：切深处的点后移，刀尖停在墙前该距离 |

示例文件中的 `position_tolerance`、`angle_tolerance_deg`、`observe_joint1`、`base_hold`、`scanning.z_*` 来自早期的顺序程序，当前执行路径不读取（对应值由技能服务器内置，扫描与切割期间始终定点保持）。

多站开槽规划文件由 `scr_planner/scripts/plan_wall_slots.py` 生成，客户端选项 `--dry-run-gap=<m>`、`--max-strafe=<m>`（默认 1.5 m）、`--no-compensation`、`--bt=<tree.xml>`。

## 用法

需先启动仿真与任务层（`ros2 launch scr_bringup sim_nav.launch.py`；只测机械臂时可用 `scr_description gazebo.launch.py`）。

```bash
# 任务：单站任务 / 多站开槽规划（经行为树执行）
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01

# 机械臂
ros2 run scr_arm_controller square_demo
ros2 run scr_arm_controller cartesian_cutting
ros2 run scr_arm_controller custom_cutting wall_cutting.yaml

# 对位鲁棒性实验（只需仿真，不需要 Nav2 与任务层）
ros2 run scr_arm_controller alignment_benchmark --trials 8 --out /tmp/alignment
```

相对路径的配置文件先在当前目录查找，再到本包安装目录的 `config/` 中查找。`config/` 中的示例：`task_at_target.yaml`（单站任务）、`fine_alignment.yaml`（精对位参数）、`tag_map.yaml`（标记地图）、`wall_cutting.yaml`、`horizontal_cutting.yaml`、`retract_rotate_extend.yaml`、`complex_sequence.yaml`、`cutting_task.yaml`。

完整说明见 [使用指南](../../docs/USER_GUIDE.md#6-单站任务导航--精对位--扫描--切割)。

## 测试

```bash
colcon build --packages-select scr_arm_controller
colcon test --packages-select scr_arm_controller && colcon test-result --verbose
```

- `test/test_visual_servo.py`：标记检测、墙面拟合、射线求交、PBVS 误差，以及闭环运动学收敛；
- `test/test_fiducial.py`、`test/test_wall_ekf.py`、`test/test_docking_control.py`：AprilTag 检测与位姿、墙面 EKF（含地图标记的测量模型、地图误差下的门限、横移比例状态：视野外的外推修正、往返横移不可观测、重新对位后保留）、饱和控制律与收敛判据；
- `test/test_tag_map.py`：标记地图的读写、校验、子地图、扰动与路径解析；
- `test/test_ibvs.py`：用有限差分校验特征雅可比、各深度方式与混合方案的闭环收敛、弱可观方向、合成图像角点检测；
- flake8、pep257、版权声明检查。

规划文件的解析与补偿公式（`slot_execution.py`）由 `scr_planner` 的 pytest 覆盖；行为树执行器一侧的解析由 `scr_task` 的 `test_task_files` 对照同一份规划文件检查。

相关实验：[对位鲁棒性](../../docs/experiments/alignment_robustness/README.md)、[视觉伺服方法对比](../../docs/experiments/ibvs_vs_pbvs/README.md)、[系统层现代化](../../docs/experiments/system_modernization/README.md)、[多站沿墙精度](../../docs/experiments/along_wall_accuracy/README.md)。
