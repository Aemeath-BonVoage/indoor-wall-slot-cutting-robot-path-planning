# scr_description

SCR 机器人模型与 Gazebo 仿真：URDF、网格、世界文件、ros2_control 控制器配置，以及早期配置使用的全向底盘运动学节点与力控平面运动插件 `scr_planar_move`。默认底盘的控制器与仿真硬件接口在 [`scr_base_control`](../scr_base_control/README.md) 中。

## 机器人

- **底盘**：麦克纳姆轮全向底盘，轮径 100 mm，轮距 593.16 × 493.80 mm。
- **机械臂**：直角坐标式 4 自由度，`joint1` Z 升降（−0.35 ~ 0.20 m）、`joint2` Y 横移（−0.08 ~ 0.06 m）、`joint3` X 伸缩（−0.045 ~ 0.02 m）、`joint4` 刀片旋转（±90°）。
- **传感器**：2D 激光雷达（`lidar_link`，360 线，测距噪声 σ = 1 cm）；前视 RGB 相机（`camera_optical_link`，640 × 480，水平视场 80°，30 Hz），安装在升降滑台 `link1` 上。

`urdf/scr_robot_gazebo.urdf` 用于仿真，`urdf/scr_robot.urdf` 用于 `display.launch.py` 中的模型查看。碰撞模型使用按网格拟合的长方体和圆柱体，外观使用 STL 网格。

## 仿真中的执行器

底盘有两种实现，由 `gazebo.launch.py` 的 `base_control` 参数选择。URDF 中两种实现的块用 `<!-- BEGIN base_control=... -->` / `<!-- END base_control=... -->` 标记，launch 文件只保留所选的一块。

| 部件 | `base_control:=ros2_control`（默认） | `base_control:=plugin`（早期配置，仿真配置 v2） |
|---|---|---|
| 机械臂关节 | gazebo_ros2_control `GazeboSystem`，`position_pid` 力控，`arm_controller`（`JointTrajectoryController`） | 相同 |
| 底盘运动 | ros2_control 系统 `MecanumBase`：硬件接口 `scr_base_control/ScrMecanumBaseSystem`（与机械臂共用一个控制器管理器），控制器 `base_controller`（`MecanumDriveController`） | `scr_planar_move`（`src/scr_planar_move.cpp`）：只对底盘施加力 / 力矩，对 `cmd_vel` 做速度 P 闭环 |
| 车轮 | 硬件接口按命令转动（`plant:=force`）或由关节电机驱动（`plant:=wheels`，实验） | `wheel_controller`（`velocity_pid`），只作显示；`scripts/mecanum_controller.py` 把 `/cmd_vel` 分为 `/cmd_vel_direct` 与四轮转速 |
| 轮式里程计 | `base_controller` 由编码器读数积分；编码器读数由底盘实际运动加误差模型得到 | 无（`/odom` 为真值位姿） |
| 陀螺 | 硬件接口模拟（`/imu`，100 Hz，白噪声 0.001 rad/s、零偏 5 × 10⁻⁵ rad/s） | 无 |
| `/odom` 与 `odom → base_link` | `localization:=ekf`：`robot_localization` 融合 `/wheel/odom` 与 `/imu`；`localization:=wheel`：`base_controller` 直接发布 | `scr_planar_move` 以真值发布 |

`MecanumBase` 块中的硬件参数：`plant`、轮半径与轮距、`wheel_max_torque`，编码器误差模型 `wheel_scale_errors`（+0.4 %、−0.3 %、+0.5 %、−0.6 %）、`slip_sigma`（2 %）、`slip_tau`（0.5 s）、`lateral_slip`（3 %）、`yaw_slip`（2 %）、`seed`，陀螺 `imu_rate`、`gyro_noise`、`gyro_bias`。`gazebo.launch.py` 按 `plant`、`odom_seed` 改写对应参数，`odom_error:=false` 时把误差项置零。各参数的含义见 [`scr_base_control`](../scr_base_control/README.md)。

模型中没有 Gazebo 的 IMU 传感器：Gazebo 中底盘有一部分转动只出现在位姿里、不出现在连杆速度里，读连杆角速度的 IMU 传感器测不到它，因此陀螺改由硬件接口按位姿差分生成（排查见 [系统层现代化报告](../../docs/experiments/system_modernization/README.md#24-轮式里程计陀螺与-ekf) 2.4 节）。

`scr_planar_move` 的 SDF 参数：`k_lin` / `k_ang`（速度环增益，默认 20 1/s）、`max_accel`（5 m/s²）、`max_ang_accel`（10 rad/s²）、`cmd_timeout`（0.5 s）、`publish_rate`、`odometry_frame`、`robot_base_frame`、`publish_odom_tf`。

`gazebo_ros_planar_move` 给模型所有连杆设置同一个速度，偏心的机械臂连杆在旋转时速度不一致；配合位置接口驱动的机械臂关节时，底盘实际转速只有指令的 18–37 %。改用力控机械臂与 `scr_planar_move` 后转速比为 0.99–1.01（诊断过程见 [项目演进](../../docs/EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真)）。`ScrMecanumBaseSystem` 的 `force` 被控对象沿用同一力控算法。

真值（评估用，不参与控制）：`gazebo_ros_p3d` 在 `/ground_truth/odom` 发布 `base_link` 的世界系位姿与速度（50 Hz）；`room.world` 中的 `gazebo_ros_state` 发布 `/gazebo/model_states`、`/gazebo/link_states`。

## 话题

| 话题 | 方向 | 类型 |
|---|---|---|
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` |
| `/odom` | 发布 | `nav_msgs/Odometry`（见上表，按 `base_control` / `localization` 由不同节点发布） |
| `/wheel/odom` | 发布 | `nav_msgs/Odometry`（`base_control:=ros2_control localization:=ekf`） |
| `/imu` | 发布 | `sensor_msgs/Imu`（`base_control:=ros2_control`，只有角速度） |
| `/scan` | 发布 | `sensor_msgs/LaserScan` |
| `/front_camera/image_raw`、`/front_camera/camera_info` | 发布 | `sensor_msgs/Image`、`CameraInfo` |
| `/arm_controller/follow_joint_trajectory` | action | `control_msgs/action/FollowJointTrajectory` |
| `/ground_truth/odom` | 发布 | `nav_msgs/Odometry`，仿真真值 |
| `/gazebo/model_states`、`/gazebo/link_states` | 发布 | 仿真真值 |

## 用法

```bash
ros2 launch scr_description gazebo.launch.py                      # 仿真（带窗口）
ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5  # 无窗口，最多 2.5 倍实时
ros2 launch scr_description display.launch.py                     # RViz + 关节滑条，查看模型
ros2 run scr_description test_mecanum.py                          # 依次测试前后、左右、斜向、旋转
ros2 run scr_description arm_demo.py                              # 机械臂关节轨迹演示
```

`gazebo.launch.py` 的参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `world` | `worlds/room.world` | Gazebo 世界文件 |
| `gui` | `true` | 是否启动 gzclient |
| `rtf` | `1.0` | 仿真速度上限，`0` 为不限速 |
| `camera_yaw_error_deg` | `0` | 在相机关节上加入已知的安装偏角（只注入 Gazebo 中的模型，TF 不变），用于验证示教零位标定 |
| `base_control` | `ros2_control` | `ros2_control` 或 `plugin` |
| `plant` | `force` | `force` 或 `wheels`（只对 `ros2_control`） |
| `odom_error` | `true` | 编码器误差模型开关 |
| `odom_seed` | `1` | 打滑过程的随机种子 |
| `localization` | `ekf` | `ekf`（启动 `robot_localization`）或 `wheel` |

```bash
ros2 launch scr_description gazebo.launch.py gui:=false base_control:=plugin   # 早期配置
ros2 launch scr_description gazebo.launch.py gui:=false plant:=wheels          # 轮子驱动底盘（实验）
```

`worlds/room.world` 是多房间室内场景。room3 南墙（内表面 y = −3.925 m）上有以下定位标记：

- **AprilTag 参考标记**（tag36h11，ID 0，黑框边长 0.10 m，中心离地 0.70 m，x = 8.42 m），精对位的目标与墙面坐标原点；
- **沿墙的其他 AprilTag**（ID 1–5，同一规格与高度），沿墙间距 0.3 m，供多标记地图使用。面向墙面向右（−x 方向）为墙面坐标 `u` 的正方向：

  | ID | 5 | 0 | 1 | 2 | 3 | 4 |
  |---|---|---|---|---|---|---|
  | x（m） | 8.72 | 8.42 | 8.12 | 7.82 | 7.52 | 7.22 |
  | u（m） | −0.3 | 0 | 0.3 | 0.6 | 0.9 | 1.2 |

  哪些标记参与定位由 `scr_arm_controller` 的标记地图决定（默认 ID 0、2、4，[`config/tag_map.yaml`](../scr_arm_controller/config/tag_map.yaml)），地图之外的 ID 被忽略；
- **黄色色块标记**（5 cm，中心离地 0.60 m，x = 8.42 m），第一版精对位使用，保留作对比。

标记模型 `models/scr_apriltag_tag36h11_<id>` 由 `scripts/generate_apriltag_model.py` 生成（不随包安装，在源码目录中运行）：

```bash
python3 src/scr_description/scripts/generate_apriltag_model.py --id 6 [--size 0.10] [--family tag36h11]
```

输出 `model.config`、`model.sdf`、Ogre 材质脚本与贴图（默认写到本包的 `models/`）。在世界文件中按 ID 0 的姿态（`roll = π/2`、`yaw = π`，背面贴墙）摆放，再把它的 `u` 写进标记地图。

墙前有一盏默认关闭的点光源 `work_light`，对位鲁棒性实验通过 `/gazebo/set_light_properties` 打开它模拟强光 / 冷光照射。

## 测试

```bash
colcon build --packages-select scr_description
```

本包没有单元测试。仿真行为（转速比、机械臂跟踪、定点保持）在任务与实验中用 Gazebo 真值检验，见 [对位鲁棒性实验](../../docs/experiments/alignment_robustness/README.md)；底盘两种实现与两种被控对象的跟踪比、里程计漂移见 [系统层现代化报告](../../docs/experiments/system_modernization/README.md#43-底盘被控对象) 4.3 节（测试脚本 `scr_base_control/scripts/base_characterization.py`）。
