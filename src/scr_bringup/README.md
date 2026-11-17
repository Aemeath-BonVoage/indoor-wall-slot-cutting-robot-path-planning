# scr_bringup

一键启动 SCR 仿真：Gazebo 仿真（`scr_description`）+ Nav2 导航（`scr_navigation`）+ RViz。本包只包含 launch 文件。

## 用法

```bash
ros2 launch scr_bringup sim_nav.launch.py                        # 完整界面
ros2 launch scr_bringup sim_nav.launch.py gui:=false             # 不开 Gazebo 窗口
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false # 纯后台，内存占用最小
ros2 launch scr_bringup sim_nav.launch.py params_file:=/path/to/nav2_params.yaml
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `gui` | `true` | 是否启动 Gazebo 窗口（gzclient） |
| `rviz` | `true` | 是否启动 RViz |
| `rtf` | `1.0` | 仿真速度上限（相对实时的倍数），`0` 为不限速 |
| `world` | `scr_description/worlds/room.world` | Gazebo 世界文件 |
| `map` | `scr_navigation/maps/my_map.yaml` | Nav2 地图 |
| `params_file` | `scr_navigation/config/nav2_params.yaml` | Nav2 参数文件 |
| `nav_delay` | `8.0` | Gazebo 启动后等待多少秒再启动 Nav2 |

Nav2 在启动后约 20–30 s 就绪，初始位姿自动发布。纯后台模式下仿真加 Nav2 约额外占用 400 MB 内存（4 核 / 4 GB 虚拟机实测）。

## 测试

本包没有单元测试；启动后可用以下命令确认各部分就绪：

```bash
ros2 control list_controllers                  # joint_state_broadcaster / arm_controller / wheel_controller 均为 active
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0   # 导航到 (1.5, 0) 应成功
```
