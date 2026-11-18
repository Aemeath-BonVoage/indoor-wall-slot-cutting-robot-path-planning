# scr_base_control

麦克纳姆底盘的 ros2_control 实现：一个控制器插件（仿真与真机共用）和一个 Gazebo 仿真硬件接口插件。

Humble 的 `ros2_controllers` 没有麦克纳姆轮控制器（`mecanum_drive_controller` 从 Jazzy 开始提供），因此按 `diff_drive_controller` 的结构自行实现；硬件接口按 `gazebo_ros2_control` 的 `GazeboSystemInterface` 插件机制实现，和机械臂使用的 `GazeboSystem` 并存于同一个控制器管理器。

```
/cmd_vel ─▶ base_controller (MecanumDriveController) ─velocity 命令接口─▶ ScrMecanumBaseSystem ─▶ Gazebo
/odom 或 /wheel/odom ◀─ 正运动学 + 积分 ◀─────────velocity 状态接口──┘
```

## 控制器 `scr_base_control/MecanumDriveController`

- 订阅 `cmd_vel_topic`（`geometry_msgs/Twist`，base_link 系），逆运动学得到四个轮的 `velocity` 命令；超过 `cmd_vel_timeout` 没有新指令则停车。
- 读四个轮的 `velocity` 状态，正运动学（最小二乘）得到机体速度，二阶 Runge–Kutta 积分为轮式里程计，按 `publish_rate` 发布到 `odom_topic`，`enable_odom_tf` 时同时发布 `odom → base_link`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `wheel_names` | `[joint_lf_link, joint_rf_link, joint_lb_link, joint_rb_link]` | 轮序 [lf, rf, lb, rb] |
| `wheel_radius` | 0.05 | 米 |
| `wheel_separation_x` / `_y` | 0.59316 / 0.4938 | 前后 / 左右轮距（米） |
| `cmd_vel_timeout` | 0.5 | 秒 |
| `odom_topic` / `enable_odom_tf` | `/odom` / true | 与 robot_localization 融合时为 `/wheel/odom` / false（`config/base_controller_ekf.yaml`） |
| `pose_covariance_diagonal` / `twist_covariance_diagonal` | 见源码 | 融合时的权重 |

运动学约定（`k = (wheel_separation_x + wheel_separation_y) / 2`）：

```
ω_lf = (vx − vy − k·wz) / r    ω_rf = (vx + vy + k·wz) / r
ω_lb = (vx + vy − k·wz) / r    ω_rb = (vx − vy + k·wz) / r
```

## 仿真硬件接口 `scr_base_control/ScrMecanumBaseSystem`

对上提供四个轮的 `velocity` 命令接口与 `position` / `velocity` 状态接口（与真机硬件接口相同）。对下有两种被控对象（硬件参数 `plant`）：

| `plant` | 驱动方式 | 编码器读数 |
|---|---|---|
| `force`（默认） | 命令轮速经正运动学得到机体速度指令，在底盘上施加力 / 力矩做速度闭环（与 `scr_description` 的 `scr_planar_move` 插件相同）；轮子摩擦为 0，只按命令转动作为外观 | 真实机体速度经逆运动学得到，再叠加编码器误差模型 |
| `wheels`（实验） | 轮子关节由 ODE 关节电机驱动（目标转速 + 最大力矩 `wheel_max_torque`）；底盘只受轮–地接触力：每个轮在辊子轴线方向上的滑移速度 `e` 产生 `F = −sat(c·e, μN)` 的摩擦力（`contact_damping`、`friction_mu`），作用在接地点，反作用力矩加在轮子上；垂直于辊子轴线的方向无力 | 轮子关节的实际转速 |

接触力在插件中计算，而不是用 ODE 的各向异性摩擦（`fdir1`）：Gazebo Classic 中 `fdir1` 只能在碰撞体自身坐标系中表达，轮子转动时摩擦方向跟着转，无法表达麦克纳姆辊子在底盘中的固定方向。`wheels` 的跟踪与里程计都不如 `force`（见 `docs/experiments/system_modernization`），默认仍用 `force`。

**底盘实际速度的来源**（`velocity_source`，默认 `pose`）：由相邻物理步的 base_link 位姿差分并低通（5 ms）得到，而不是 Gazebo 的连杆速度。Gazebo 中底盘有一部分转动（零速度指令下约 0.08°/s）只出现在位姿里、不出现在连杆速度里，用连杆速度做速度闭环时这部分转动不受控制，编码器与陀螺也测不到。`link` 恢复早期插件的做法。

**模拟陀螺**：硬件接口同时按 `imu_rate`（默认 100 Hz）在 `/imu` 发布 `sensor_msgs/Imu`，角速度取同一个位姿差分角速度，加白噪声 `gyro_noise`（0.001 rad/s）与零偏 `gyro_bias`（5 × 10⁻⁵ rad/s，随机符号）；不提供姿态与加速度（协方差 −1）。这是仿真专用的传感器模型，真机上由 IMU 驱动发布同一话题。

编码器误差模型（`include/scr_base_control/encoder_error_model.hpp`，只用于 `force`）：

```
ω_meas,i = (1 + scale_i + g_i) · IK(vx, (1 + lateral_slip)·vy, (1 + yaw_slip)·wz)_i
```

`scale_i` 为每个轮固定的轮径 / 标定误差，`g_i` 为一阶高斯–马尔可夫打滑过程（`slip_sigma`、`slip_tau`），`lateral_slip` / `yaw_slip` 为辊子横移 / 原地转向时的系统性打滑。全部为 0 时里程计等于真值积分。参数在 `scr_description/urdf/scr_robot_gazebo.urdf` 的 `MecanumBase` 块中，`gazebo.launch.py` 的 `odom_error:=false` 把它们置零，`odom_seed` 改变打滑过程的随机种子。

## 用法

由 `scr_description/launch/gazebo.launch.py`（`base_control:=ros2_control`，默认）加载；`sim_nav.launch.py` 透传以下参数：

```bash
ros2 launch scr_bringup sim_nav.launch.py                        # 力驱动底盘 + 编码器误差 + EKF（默认）
ros2 launch scr_bringup sim_nav.launch.py localization:=wheel    # 只用轮式里程计
ros2 launch scr_bringup sim_nav.launch.py odom_error:=false      # 理想轮式里程计
ros2 launch scr_bringup sim_nav.launch.py plant:=wheels          # 轮子驱动底盘（实验）
```

底盘特性测试（跟踪比、里程计漂移、实时率与 CPU；只启动 Gazebo 与控制器）：

```bash
ros2 launch scr_description gazebo.launch.py gui:=false
ros2 run scr_base_control base_characterization.py --out <目录> --label force
```

## 测试

```bash
colcon test --packages-select scr_base_control
```

`test_mecanum_kinematics`：逆 / 正运动学互逆、与外观轮速脚本的符号一致、里程计积分（圆周闭合、横移方向）、误差模型（零误差精确、比例误差与打滑的系统偏差、打滑过程的稳态标准差）。
