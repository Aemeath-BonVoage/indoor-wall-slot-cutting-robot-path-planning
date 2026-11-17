# scr_navigation

SLAM Toolbox 建图与 Nav2 导航的配置、地图和辅助脚本。

## 内容

| 文件 | 说明 |
|---|---|
| `launch/slam.launch.py` | SLAM Toolbox 异步建图 + RViz |
| `launch/navigation.launch.py` | Nav2 bringup + RViz + 自动发布初始位姿 |
| `config/slam_toolbox.yaml` | 建图参数（分辨率 0.05 m，`base_frame: base_link`） |
| `config/nav2_params.yaml` | Nav2 参数 |
| `maps/my_map.yaml` | `room.world` 的地图 |
| `scripts/send_goal.py` | 命令行发送导航目标 |
| `scripts/set_initial_pose.py` | 向 AMCL 发布初始位姿（参数 `x`、`y`、`yaw`） |

## Nav2 配置要点

- AMCL：全向运动模型 `nav2_amcl::OmniMotionModel`。
- 全局规划：`NavfnPlanner`；局部控制：DWB，速度上限 `vx = vy = 0.4 m/s`、`ωz = 0.8 rad/s`。
- 代价地图：机器人半径 0.4 m，膨胀半径 0.7 m。

导航只负责把底盘送到墙前约 1–2 m 处；墙前的毫米级对位由 `scr_arm_controller` 中的视觉伺服完成。

## 用法

```bash
# 建图（另开终端运行仿真与键盘遥控）
ros2 launch scr_navigation slam.launch.py
ros2 run nav2_map_server map_saver_cli -f src/scr_navigation/maps/my_map

# 导航
ros2 launch scr_navigation navigation.launch.py
ros2 launch scr_navigation navigation.launch.py map:=/path/to/map.yaml params_file:=/path/to/params.yaml rviz:=false
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0     # x y yaw（map 坐标）
```

`navigation.launch.py` 的参数：`map`、`params_file`、`use_sim_time`（默认 `true`）、`rviz`（默认 `true`）、`initial_pose_x / initial_pose_y / initial_pose_yaw`（默认 0，启动 6 s 后自动发布）。

完整步骤见 [使用指南](../../docs/USER_GUIDE.md#4-slam-建图)。

## 测试

本包没有单元测试；导航功能在单站任务与多站执行中验证（多站执行中 10 段 Nav2 导航全部成功，见 [多站执行报告](../../docs/experiments/slot_execution/README.md)）。
