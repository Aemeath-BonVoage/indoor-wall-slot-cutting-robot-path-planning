# scr_description

SCR 机器人模型与 Gazebo 仿真：URDF、网格、世界文件、ros2_control 控制器配置、全向底盘运动学节点，以及力控平面运动插件 `scr_planar_move`。

## 机器人

- **底盘**：麦克纳姆轮全向底盘，轮径 100 mm，轮距 593.16 × 493.80 mm。
- **机械臂**：直角坐标式 4 自由度，`joint1` Z 升降（−0.35 ~ 0.20 m）、`joint2` Y 横移（−0.08 ~ 0.06 m）、`joint3` X 伸缩（−0.045 ~ 0.02 m）、`joint4` 刀片旋转（±90°）。
- **传感器**：2D 激光雷达（`lidar_link`，360 线，测距噪声 σ = 1 cm）；前视 RGB 相机（`camera_optical_link`，640 × 480，水平视场 80°，30 Hz），安装在升降滑台 `link1` 上。

`urdf/scr_robot_gazebo.urdf` 用于仿真，`urdf/scr_robot.urdf` 用于 `display.launch.py` 中的模型查看。碰撞模型使用按网格拟合的长方体和圆柱体，外观使用 STL 网格。

## 仿真中的执行器

| 部件 | 实现 | 说明 |
|---|---|---|
| 机械臂关节 | gazebo_ros2_control，`position_pid` 力控，控制器 1 kHz | `config/arm_controllers.yaml`；PID 按各关节驱动质量整定 |
| 车轮 | `wheel_controller`（`velocity_pid`） | 只驱动轮子转动作为显示；轮子摩擦为 0 |
| 底盘运动 | `scr_planar_move`（`src/scr_planar_move.cpp`） | 只对底盘施加力 / 力矩，对 `cmd_vel` 做速度 P 闭环；发布 `odom` 与 `odom → base_link` TF |
| 速度分发 | `scripts/mecanum_controller.py` | `/cmd_vel` → `/cmd_vel_direct`（底盘）+ `/wheel_controller/commands`（四轮转速） |

`scr_planar_move` 的 SDF 参数：`k_lin` / `k_ang`（速度环增益，默认 20 1/s）、`max_accel`（5 m/s²）、`max_ang_accel`（10 rad/s²）、`cmd_timeout`（0.5 s）、`publish_rate`、`odometry_frame`、`robot_base_frame`、`publish_odom_tf`。

`gazebo_ros_planar_move` 给模型所有连杆设置同一个速度，偏心的机械臂连杆在旋转时速度不一致；配合位置接口驱动的机械臂关节时，底盘实际转速只有指令的 18–37 %。改用力控机械臂与 `scr_planar_move` 后转速比为 0.99–1.01（诊断过程见 [项目演进](../../docs/EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真)）。

## 话题

| 话题 | 方向 | 类型 |
|---|---|---|
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` |
| `/odom` | 发布 | `nav_msgs/Odometry` |
| `/scan` | 发布 | `sensor_msgs/LaserScan` |
| `/front_camera/image_raw`、`/front_camera/camera_info` | 发布 | `sensor_msgs/Image`、`CameraInfo` |
| `/arm_controller/follow_joint_trajectory` | action | `control_msgs/action/FollowJointTrajectory` |
| `/gazebo/model_states`、`/gazebo/link_states` | 发布 | 仿真真值（`room.world` 中的 `gazebo_ros_state`） |

## 用法

```bash
ros2 launch scr_description gazebo.launch.py                      # 仿真（带窗口）
ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5  # 无窗口，最多 2.5 倍实时
ros2 launch scr_description display.launch.py                     # RViz + 关节滑条，查看模型
ros2 run scr_description test_mecanum.py                          # 依次测试前后、左右、斜向、旋转
ros2 run scr_description arm_demo.py                              # 机械臂关节轨迹演示
```

`gazebo.launch.py` 的参数：`world`、`gui`（默认 `true`）、`rtf`（默认 `1.0`）。

`worlds/room.world` 是多房间室内场景，room3 南墙（内表面 y = −3.925 m）上贴有 5 cm 黄色定位标记，中心 (8.42, −3.9245) m。

## 测试

```bash
colcon build --packages-select scr_description
```

本包没有单元测试。仿真行为（转速比、机械臂跟踪、定点保持）在任务与实验中用 Gazebo 真值检验，见 [对位鲁棒性实验](../../docs/experiments/alignment_robustness/README.md)。
