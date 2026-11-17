# scr_arm_controller

机械臂运动学与控制、墙面精对位（AprilTag + 激光 EKF + 饱和控制律）、底盘定点保持，以及项目的任务总流程 `navigation_cutting_task`（单站任务和多站开槽共用的执行路径）。

## 模块

| 模块 | 内容 | 依赖 ROS |
|---|---|---|
| `cartesian_cutting.py` | `CartesianCuttingController`：闭式正 / 逆运动学、可达性检查、笛卡尔 / 关节空间运动 | 是 |
| `navigation_cutting_task.py` | 任务总流程：导航 → 精对位 → 扫描 → 定点保持下切割；读取多站开槽规划文件逐站执行；激光防碰撞守护；空切 | 是 |
| `fiducial.py` | 纯算法：AprilTag（tag36h11）检测，边缘直线求交的亚像素角点，IPPE PnP 求标记中心与协方差 | 否 |
| `wall_ekf.py` | 纯算法：墙面相对状态 (d, u, ψ) 的 EKF，里程计增量预测，激光与标记更新，χ² 门限 | 否 |
| `wall_estimator_ros.py` | EKF 的 ROS 封装：按时间戳经里程计历史补偿测量延迟，标记丢失时外推 | 是 |
| `docking_control.py` | 纯算法：平方根整形饱和比例律、速度 / 加速度限幅、收敛判据（误差 + 静止 + 估计不确定度） | 否 |
| `teach_alignment_offset.py` | 示教零位标定：在真值参考位姿处求估计器的静态偏置，输出 ROS 参数文件 | 是 |
| `visual_servo.py` | 纯算法（第一版精对位，保留为基线）：HSV 定位标记检测、激光墙面拟合（总体最小二乘 + 离群点剔除）、像素反投影、射线与墙面求交、PBVS 误差与全向底盘速度指令 | 否 |
| `visual_servo_ros.py` | `WallMarkerServo`：订阅激光、图像、相机内参，查询 TF，输出滤波后的墙面与标记测量；包含主点修正 | 是 |
| `base_hold.py` | `BaseHold`：独立节点与线程，50 Hz 里程计 P 控制保持底盘位姿，并用真值记录保持期间的最大漂移 | 是 |
| `ibvs.py` | IBVS 与 2.5D 混合视觉伺服（交互矩阵、`IbvsController`、`HybridController`），用于方法对比 | 否 |
| `alignment_benchmark.py` | 对位鲁棒性实验：随机起点传送、里程计漂移与传感器噪声注入、真值评分 | 是 |
| `custom_cutting.py`、`square_demo.py` | YAML 点序列执行、关节空间演示 | 是 |
| `marker_scanner.py`、`trajectory_visualizer.py` | 早期的独立标记扫描节点；切割结束后在 Gazebo 中画出切割框 | 是 |
| `joint_direction_check.py` | 检查各关节在 `base_link` 中的实际运动方向（需仿真运行） | 是 |

## 机械臂运动学

| 关节 | 范围 | 作用 |
|---|---|---|
| `joint1` | −0.35 ~ 0.20 m | Z 升降，`z = z0 + joint1` |
| `joint2` | −0.08 ~ 0.06 m | Y 横移，`y = y0 − joint2` |
| `joint3` | −0.045 ~ 0.02 m | X 伸缩，`x = x0 − joint3` |
| `joint4` | −90° ~ 90° | 刀片方向 |

所有关节为 0 时末端位于 (x0, y0, z0) = (0.537, 0.000, 0.507) m（`base_link` 坐标）。笛卡尔工作空间（TF 实测）：X 0.517–0.582 m，Y −0.060–0.080 m，Z 0.157–0.707 m。伸缩轴完全缩回（`joint3 = +0.02`）时刀尖离底盘中心 0.676 m，完全伸出时 0.741 m。

## 精对位

对位模式由任务 YAML 的 `precise_positioning.mode` 选择：`visual`（第二版，默认）、`visual_v1`（第一版色块 + PBVS，作对比基线）、`odom`（里程计 + 已知标记坐标）。第二版参数在 [`config/fine_alignment.yaml`](config/fine_alignment.yaml)（节点参数 `alignment.*`）：检测器 `apriltag | blob`、标记尺寸与 ID、收敛容差、控制增益与限幅、示教零位偏置。

```bash
# 示教零位（需先启动仿真，不需要 Nav2）；输出的参数文件在执行任务时加载
ros2 run scr_arm_controller teach_alignment_offset --out fine_alignment_taught.yaml --seconds 30
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml \
    --ros-args --params-file fine_alignment_taught.yaml
```

设计、控制律稳定性分析与实验见 [精对位报告](../../docs/experiments/fine_alignment/README.md) 与 [对位鲁棒性对比](../../docs/experiments/alignment_robustness/README.md)。

## 接口

`navigation_cutting_task` 使用的话题与 action：

| 名称 | 方向 | 用途 |
|---|---|---|
| `/navigate_to_pose` | action 客户端 | Nav2 导航 |
| `/cmd_vel` | 发布 | 精对位、横移、定点保持 |
| `/arm_controller/follow_joint_trajectory` | action 客户端 | 机械臂运动 |
| `/odom` | 订阅 | 横移参考、定点保持 |
| `/scan` | 订阅 | 墙面拟合、防碰撞守护 |
| `/front_camera/image_raw`、`/front_camera/camera_info` | 订阅 | 定位标记检测 |
| `/gazebo/model_states` | 订阅 | 真值，只用于评估，不参与控制 |

单站任务文件（`config/task_at_target.yaml`）的主要字段：

| 字段 | 说明 |
|---|---|
| `navigation.{x, y, yaw}` | Nav2 目标 |
| `precise_positioning.enabled / mode` | 是否精对位；`visual`（PBVS）或 `odom`（里程计 + 已知标记坐标，对照基线） |
| `precise_positioning.distance_from_marker` | 对位后底盘中心离墙距离，默认任务为 0.70 m |
| `precise_positioning.position_tolerance / angle_tolerance_deg` | 收敛容差，默认 1 cm / 1° |
| `precise_positioning.observe_joint1` | 对位时滑台高度，避免机械臂遮挡相机 |
| `base_hold` | 扫描 / 切割期间是否定点保持（`false` 时只测量漂移） |
| `scanning.enabled` | 是否在对位后检测标记三维位置 |
| `points`、`motion.{duration, return_home}` | 切割点（`base_link` 坐标，`rotation` 单位为度）与运动参数 |
| `motion.dry_run_gap` | 空切间隙（米）：切深处的点后移，刀尖停在墙前该距离 |

多站开槽规划文件由 `scr_planner/scripts/plan_wall_slots.py` 生成，命令行选项 `--dry-run-gap`、`--max-strafe`（默认 1.5 m）、`--no-compensation`。

## 用法

需先启动仿真（`ros2 launch scr_bringup sim_nav.launch.py`，只测机械臂时可用 `scr_description gazebo.launch.py`）。

```bash
# 任务总流程：单站任务 / 多站开槽规划
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
ros2 run scr_arm_controller navigation_cutting_task /tmp/plan.yaml --dry-run-gap=0.01

# 机械臂
ros2 run scr_arm_controller square_demo
ros2 run scr_arm_controller cartesian_cutting
ros2 run scr_arm_controller custom_cutting wall_cutting.yaml

# 对位鲁棒性实验（只需仿真，不需要 Nav2）
ros2 run scr_arm_controller alignment_benchmark --trials 8 --out /tmp/alignment
```

相对路径的配置文件先在当前目录查找，再到本包安装目录的 `config/` 中查找。`config/` 中的示例：`task_at_target.yaml`（单站任务）、`wall_cutting.yaml`、`horizontal_cutting.yaml`、`retract_rotate_extend.yaml`、`complex_sequence.yaml`、`cutting_task.yaml`。

完整说明见 [使用指南](../../docs/USER_GUIDE.md#6-单站任务导航--精对位--扫描--切割)。

## 测试

```bash
colcon build --packages-select scr_arm_controller
colcon test --packages-select scr_arm_controller && colcon test-result --verbose
```

- `test/test_visual_servo.py`：标记检测、墙面拟合、射线求交、PBVS 误差，以及闭环运动学收敛；
- `test/test_ibvs.py`：用有限差分校验特征雅可比、各深度方式与混合方案的闭环收敛、弱可观方向、合成图像角点检测；
- flake8、pep257、版权声明检查。

多站执行的解析与补偿公式由 `scr_planner` 的 pytest 覆盖。

相关实验：[对位鲁棒性](../../docs/experiments/alignment_robustness/README.md)、[视觉伺服方法对比](../../docs/experiments/ibvs_vs_pbvs/README.md)。
