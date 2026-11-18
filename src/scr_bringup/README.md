# scr_bringup

一键启动 SCR 仿真：Gazebo 仿真与底盘 / 机械臂控制器（`scr_description`）+ Nav2 导航（`scr_navigation`）+ 任务层（技能服务器与行为树执行器，`scr_task`）+ RViz。本包只包含 launch 文件。

## 用法

```bash
ros2 launch scr_bringup sim_nav.launch.py                        # 完整界面
ros2 launch scr_bringup sim_nav.launch.py gui:=false             # 不开 Gazebo 窗口
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false # 纯后台，内存占用最小
ros2 launch scr_bringup sim_nav.launch.py task:=false            # 不启动任务层（只做导航实验）
ros2 launch scr_bringup sim_nav.launch.py localization:=wheel    # 只用轮式里程计（不融合陀螺）
ros2 launch scr_bringup sim_nav.launch.py odom_error:=false      # 理想编码器
ros2 launch scr_bringup sim_nav.launch.py plant:=wheels          # 轮子驱动底盘（实验）
ros2 launch scr_bringup sim_nav.launch.py base_control:=plugin   # 早期力控插件（/odom 为真值）
ros2 launch scr_bringup sim_nav.launch.py \
    params_file:=$(ros2 pkg prefix scr_navigation)/share/scr_navigation/config/nav2_params_dwb.yaml
                                                                 # NavFn + DWB 基线
```

任务层就绪后执行任务：

```bash
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
ros2 run scr_arm_controller navigation_cutting_task <plan.yaml> --dry-run-gap=0.01
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `gui` | `true` | 是否启动 Gazebo 窗口（gzclient） |
| `rviz` | `true` | 是否启动 RViz |
| `rtf` | `1.0` | 仿真速度上限（相对实时的倍数），`0` 为不限速 |
| `world` | `scr_description/worlds/room.world` | Gazebo 世界文件 |
| `map` | `scr_navigation/maps/my_map.yaml` | Nav2 地图 |
| `params_file` | `scr_navigation/config/nav2_params.yaml` | Nav2 参数文件：默认 Smac 2D + MPPI（Omni）；`nav2_params_dwb.yaml` 为 NavFn + DWB 基线 |
| `nav_delay` | `8.0` | Gazebo 启动后等待多少秒再启动 Nav2 与任务层 |
| `use_composition` | `True` | Nav2 各服务器放在一个容器进程中；`False` 时每个服务器一个进程 |
| `base_control` | `ros2_control` | `ros2_control`：`scr_base_control` 的麦克纳姆控制器 + 仿真硬件接口；`plugin`：早期力控插件 `scr_planar_move`，`/odom` 为真值（仿真配置 v2） |
| `plant` | `force` | ros2_control 底盘的被控对象：`force`（力驱动，默认）或 `wheels`（轮子驱动，实验） |
| `odom_error` | `true` | 编码器误差模型开关（只作用于 `force`） |
| `odom_seed` | `1` | 编码器打滑过程的随机种子 |
| `localization` | `ekf` | `ekf`：`/wheel/odom` + `/imu` 经 `robot_localization` 融合为 `/odom` 与 TF；`wheel`：控制器直接发布 `/odom` 与 TF（只对 `ros2_control` 有效） |
| `task` | `true` | 是否启动任务层（`scr_task/launch/task.launch.py`） |
| `alignment_params` | `scr_arm_controller/config/fine_alignment.yaml` | 技能服务器的精对位参数文件（`alignment.*`） |

技能服务器的局部闭环里程计 `odom_topic` 由本 launch 自动设置：`base_control:=ros2_control localization:=ekf` 时为 `/wheel/odom`，其余情况为 `/odom`。

Nav2 在启动后约 20–30 s 就绪，初始位姿自动发布；行为树执行器等待 Nav2 激活与技能服务器就绪后再加载行为树。纯后台模式下仿真加 Nav2 约额外占用 400 MB 内存（4 核 / 4 GB 虚拟机实测）。

## 测试

本包没有单元测试；启动后可用以下命令确认各部分就绪：

```bash
ros2 control list_controllers                  # joint_state_broadcaster / arm_controller / base_controller 均为 active
ros2 action list | grep -E "execute_|task_skills"   # 2 个任务 action、6 个技能 action
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0   # 导航到 (1.5, 0) 应成功
```

`base_control:=plugin` 时控制器为 `joint_state_broadcaster / arm_controller / wheel_controller`。各启动选项的说明与故障排查见 [使用指南](../../docs/USER_GUIDE.md#2-启动仿真)。
