# scr_navigation

SLAM Toolbox 建图、Nav2 导航（Smac 2D + MPPI）与 `robot_localization` 定位融合的配置、地图和辅助脚本。

## 内容

| 文件 | 说明 |
|---|---|
| `launch/slam.launch.py` | SLAM Toolbox 异步建图 + RViz |
| `launch/navigation.launch.py` | Nav2 bringup + RViz + 自动发布初始位姿 |
| `config/slam_toolbox.yaml` | 建图参数（分辨率 0.05 m，`base_frame: base_link`） |
| `config/nav2_params.yaml` | Nav2 参数（默认）：Smac 2D + MPPI（Omni） |
| `config/nav2_params_dwb.yaml` | Nav2 基线参数：NavFn + DWB；与默认文件只有 `FollowPath` 与 `GridBased` 不同 |
| `config/ekf.yaml` | `robot_localization` EKF：平面模式，融合 `/wheel/odom` 的 `(vx, vy)` 与 `/imu` 的 `wz`，发布 `/odom` 与 `odom → base_link`（由 `scr_description/gazebo.launch.py` 在 `localization:=ekf` 时启动） |
| `maps/my_map.yaml` | `room.world` 的地图 |
| `scripts/send_goal.py` | 命令行发送导航目标 |
| `scripts/set_initial_pose.py` | 向 AMCL 发布初始位姿（参数 `x`、`y`、`yaw`） |
| `scripts/nav_benchmark.py` | 导航巡回基准：固定 6 个目标、按圈记录到达、用时、路径、离障间隙、平滑性、CPU 与里程计漂移 |

## Nav2 配置要点

- AMCL：全向运动模型 `nav2_amcl::OmniMotionModel`。
- 全局规划：`nav2_smac_planner/SmacPlanner2D`（代价感知的 8 邻接 A\* + 路径平滑，`cost_travel_multiplier = 2.0` 使路径远离障碍）。麦克纳姆底盘没有最小转弯半径，不使用 Hybrid-A\*。
- 局部控制：`nav2_mppi_controller::MPPIController`，运动模型 `Omni`；1000 条采样、40 步 × 50 ms（2 s 预测时域）、20 Hz；`PathAngleCritic` 不偏好前进方向，另加 `TwirlingCritic` 抑制行进中的旋转。速度上限 `vx = vy = 0.4 m/s`、`ωz = 0.8 rad/s`。
- 代价地图：机器人半径 0.4 m，膨胀半径 0.7 m。
- 两份参数文件中 `controller_server.min_y_velocity_threshold` 均为 0.001（全向底盘需要小的横向速度反馈）。

两种配置在 Gazebo 中的对比（固定 6 目标 × 3 圈）：Smac 2D + MPPI 18 / 18 到达、最小离障间隙 183 mm、控制器 CPU 9.7 % 单核；NavFn + DWB 15 / 18、5 mm、45.5 %（[系统层现代化报告](../../docs/experiments/system_modernization/README.md#42-nav2smac-2d--mppi-与-navfn--dwb) 4.2 节）。

导航只负责把底盘送到墙前约 1–2 m 处；墙前的毫米级对位由 `scr_arm_controller` 中的精对位技能完成。

## 用法

```bash
# 建图（另开终端运行仿真与键盘遥控）
ros2 launch scr_navigation slam.launch.py
ros2 run nav2_map_server map_saver_cli -f src/scr_navigation/maps/my_map

# 导航
ros2 launch scr_navigation navigation.launch.py
ros2 launch scr_navigation navigation.launch.py map:=/path/to/map.yaml params_file:=/path/to/params.yaml rviz:=false
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0     # x y yaw（map 坐标）

# 导航基准（Nav2 分进程以分别统计 CPU；--wheel-topic 额外记录轮式里程计）
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false task:=false use_composition:=False
ros2 run scr_navigation nav_benchmark.py --out /tmp/nav_mppi --rounds 3 --label mppi_smac2d \
    --wheel-topic /wheel/odom
```

`navigation.launch.py` 的参数：`map`、`params_file`、`use_sim_time`（默认 `true`）、`rviz`（默认 `true`）、`use_composition`（默认 `True`，`False` 时每个服务器一个进程）、`initial_pose_x / initial_pose_y / initial_pose_yaw`（默认 0，启动 6 s 后自动发布）。

完整步骤见 [使用指南](../../docs/USER_GUIDE.md#4-slam-建图)。

## 测试

本包没有单元测试；导航功能用 `nav_benchmark.py` 在 Gazebo 中评估（[系统层现代化报告](../../docs/experiments/system_modernization/README.md)），并在单站任务与多站执行中验证（[多站执行报告](../../docs/experiments/slot_execution/README.md)）。
