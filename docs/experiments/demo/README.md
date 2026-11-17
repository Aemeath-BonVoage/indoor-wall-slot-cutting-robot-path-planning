# 可视化与演示（路线图阶段 3）

工具在 [`src/scr_viz`](../../../src/scr_viz/README.md)：RViz 场景（`plan_viz`）、离线预览图（`plan_preview`）、
Gazebo 墙面实体（`spawn_wall_fixtures`）、演示相机录制（`demo_record` / `make_demo_video`）。

## RViz 场景（离线，静态 TF）

`ros2 launch scr_viz plan_viz.launch.py static_tf:=true`，载入 `src/scr_viz/config/` 下的演示任务
（room3 南墙，两条槽 + 灯开关 + 配电箱）与其规划结果（dp / sweep，11 站）：

![RViz](rviz_plan_viz.png)

墙面上：可达高度带（绿）、每站可达窗口（彩色竖带，下方是站号）、槽线（红，线宽 = 槽宽 35 mm）、
管线中心线（白，弯曲半径 12 cm）、已有插座（白）/ 新装线盒（蓝）、障碍物与规范边距轮廓；
地面上：停靠站箭头与序号、访问顺序（黄，从定位标记前的 start 出发）、底盘轮廓。

同一份 Marker 几何的离线渲染（`plan_preview`，中文标签）：

![预览](plan_viz_preview.png)

```bash
ros2 run scr_viz plan_preview --task src/scr_viz/config/wall_task_room3_south_demo.yaml \
    --plan src/scr_viz/config/plan_room3_south_demo.yaml --out docs/experiments/demo/plan_viz_preview.png
```

### 截图方法（Wayland）

GNOME Wayland 下普通截屏工具拿不到画面，但 RViz、gzclient 这类 Qt 程序默认走 XWayland，
可以直接抓 X 窗口：

```bash
export DISPLAY=:0
xwininfo -root -tree | grep -i rviz          # 找窗口 id（标题含 “- RViz” 的那一行）
xwd -silent -id 0x1400106 | convert xwd:- rviz.png
```

上图就是这样截的（再裁掉左侧面板）。

## 验证情况

| 项目 | 状态 |
|---|---|
| `plan_viz` + RViz（离线静态 TF，`ROS_DOMAIN_ID=44`） | ✅ 111 个 Marker；RViz `Status: Ok`；截图如上 |
| 刀尖轨迹（`/scr_viz/tool_tip` 手动发点） | ✅ 按离墙距离着色（离墙 12.5 cm → 青；进入墙面 → 红）；`clear_trace` 发 DELETEALL |
| 规划 / 任务文件改动后自动重载 | ✅ 按修改时间检测 |
| 墙面实体 SDF | ✅ 6 个实体 + 相机 SDF 均通过 `gz sdf -k`；位姿单元测试（背面贴墙、朝向） |
| `/gazebo/link_states` 推算刀尖、`--spawn` 运行时生成、演示相机出图 | ⏳ 未在 Gazebo 中跑：可用内存不足 |
| 单元测试 | ✅ `colcon test --packages-select scr_viz`：53 项（flake8 / pep257 在内），0 失败 |

## 演示视频的录制流程（待 Gazebo 空闲时执行）

```bash
# 1) 启动仿真（另一终端）
ros2 launch scr_bringup sim_nav.launch.py
# 2) 墙面实体 + RViz
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --spawn
ros2 launch scr_viz plan_viz.launch.py plan_file:=/tmp/plan.yaml use_sim_time:=true
# 3) 演示相机（每仿真秒 2 帧），同时执行规划
ros2 run scr_viz demo_record --out /tmp/demo_frames --preset oblique --rate 2 --video docs/experiments/demo/slot_demo.mp4
ros2 run scr_planner execute_slot_plan.py /tmp/plan.yaml /tmp/run --dry-run-gap=0.01
# 4) GIF（README 用）
ros2 run scr_viz make_demo_video /tmp/demo_frames docs/experiments/demo/slot_demo.gif --fps 12 --width 480 --every 2
```

墙面实体默认无 collision，不会改变激光对位和切割，因此执行结果应与不生成实体时一致。
