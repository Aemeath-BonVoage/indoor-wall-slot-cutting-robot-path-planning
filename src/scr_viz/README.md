# scr_viz

墙面开槽的可视化与演示工具：RViz 场景、Gazebo 墙面实体、演示相机录制。只读 `scr_planner` 的
墙面任务 YAML 与规划结果 YAML，不依赖 `scr_planner` 的 Python 代码。

| 可执行文件 | 作用 | 需要 ROS / Gazebo |
|---|---|---|
| `plan_viz` | 墙面任务 + 规划结果 → `MarkerArray`；实时画实际刀尖轨迹 | ROS |
| `plan_preview` | 同一份 Marker 几何离线画成 PNG（立面图 + 俯视图） | 否 |
| `spawn_wall_fixtures` | 线盒 / 插座 / 开关 / 配电箱 → 贴墙薄盒 SDF；可运行时 spawn / delete | `--spawn` 时需要 Gazebo |
| `demo_record` | spawn 固定视角相机，按仿真时间抽帧存 JPEG，结束后合成视频 | Gazebo |
| `make_demo_video` | 帧序列 → MP4（ffmpeg）/ GIF（PIL，无 ffmpeg 时自动退回） | 否 |

## RViz 可视化

```bash
# 离线（不开 Gazebo）：静态 TF map→odom，默认载入 config/ 下的示例任务与规划
ros2 launch scr_viz plan_viz.launch.py static_tf:=true

# 新的规划结果
python3 src/scr_planner/scripts/plan_wall_slots.py src/scr_viz/config/wall_task_room3_south_demo.yaml /tmp/plan.yaml
ros2 launch scr_viz plan_viz.launch.py plan_file:=/tmp/plan.yaml static_tf:=true

# 与仿真一起：刀尖轨迹从 /gazebo/link_states 推算（link4 + tool_offset，与 execute_slot_plan 一致）
ros2 launch scr_viz plan_viz.launch.py plan_file:=/tmp/plan.yaml use_sim_time:=true
```

`/scr_plan_viz/markers`（transient local）中的 namespace：

| ns | 内容 |
|---|---|
| `wall` / `edge_margin` | 墙面（半透明）及墙边 10 cm 净距 |
| `reach_band` | 机械臂可达高度带（绿） |
| `obstacles` / `margins` / `obstacle_labels` | 墙面已有物体（梁柱 / 门窗 / 线盒…，按种类着色）与规范边距外扩轮廓 |
| `endpoints` / `endpoint_labels` | 开槽任务线盒：已有（白）/ 新装（蓝） |
| `slots` / `conduits` / `unreachable` | 槽线（直角折线，线宽 = 槽宽）、管线中心线（带弯曲半径）、够不到的段 |
| `station_arrows` / `station_ids` / `station_footprints` | 停靠站位姿、执行序号、底盘轮廓 |
| `station_order` / `start` | 访问顺序连线（从定位标记前的起点出发） |
| `station_windows` / `station_window_ids` / `station_pieces` | 每站机械臂可达窗口（墙面上的彩色带）及本站负责切的子折线 |
| `planned_tip_path` | 每站机械臂点换算到 map 的计划刀尖轨迹（紫），可与实际轨迹对比 |

`/scr_plan_viz/tool_trace`：实际刀尖轨迹（红 = 离墙 < `contact_threshold`，即切割；青 = 移动 / 退刀）。
来源二选一或同时：`/gazebo/link_states`（默认开）或 `tool_tip_topic`（`geometry_msgs/PointStamped`，
默认 `/scr_viz/tool_tip`）。`ros2 service call /scr_plan_viz/clear_trace std_srvs/srv/Empty` 清空。

规划 / 任务文件被改写后节点会自动重新加载（按修改时间检测）。RViz 的文字只能可靠显示 ASCII，
中文标签默认换成种类 + 序号（`ascii_labels:=false` 关闭）；`plan_preview` 中显示中文原名。

## Gazebo 墙面实体

```bash
# 只生成 SDF（离线）
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --out /tmp/fixtures
# 运行时生成 / 删除（不修改 room.world）
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --spawn
ros2 run scr_viz spawn_wall_fixtures src/scr_viz/config/wall_task_room3_south_demo.yaml --delete
```

- `obstacles` 中 `kind: box` 的物体按标签推断种类（配电箱 / 开关 / 插座 / 线盒），尺寸取 YAML；
  `tasks` 的起终点：“已有…”生成插座面板，其余生成半透明蓝色“新装位置”。
- 标签含“定位标记”的物体跳过（`room.world` 中已有定位标记）。
- 默认**只有 visual、没有 collision**：激光（ray 只看 collision）和机械臂都碰不到，不影响对位与切割；
  `--collision` 可加。盒子背面贴墙面内表面，正面带插孔 / 开关键 / 箱门缝等细节。

## 演示录制（Wayland 下替代录屏）

```bash
ros2 run scr_viz demo_record --out /tmp/demo_frames --preset oblique --rate 2 --video /tmp/demo.mp4
# Ctrl-C 结束：删除相机模型并合成视频；或之后单独合成
ros2 run scr_viz make_demo_video /tmp/demo_frames /tmp/demo.gif --fps 12 --width 480
```

预设视角（room3 南墙）：`oblique`（斜后方俯视）、`front`（机器人身后正对墙）、`side`（东侧近景看进给）、
`top`（高处俯视底盘横移）；也可 `--eye X Y Z --target X Y Z`。按图像时间戳（仿真时间）抽帧，
仿真慢于实时也不影响节奏；帧上叠加仿真时间。相机图像话题 `/<name>/image_raw`。

## 测试

```bash
colcon build --packages-select scr_viz && colcon test --packages-select scr_viz
```

纯函数（YAML → Marker / SDF、坐标变换、抽帧、图像解码、GIF / MP4）都有 pytest；其中会用 `scr_planner`
的规划结果核对：停靠站位姿 = 墙面 `u_c` 处离墙 standoff；切割点换算到 map 后刀尖进入墙面 `slot_depth`
且落在本站子折线上；规范边距与 `wall_rules.py` 一致。
