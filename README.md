# Indoor Wall Slot-Cutting Robot — Path Planning

[![CI](https://github.com/Aemeath-BonVoage/indoor-wall-slot-cutting-robot-path-planning/actions/workflows/ci.yml/badge.svg)](https://github.com/Aemeath-BonVoage/indoor-wall-slot-cutting-robot-path-planning/actions/workflows/ci.yml)

**面向室内墙体线槽开槽任务的机器人自主路径规划设计**

一个基于 **ROS 2 Humble** 的室内墙面开槽机器人（SCR, *Slot Cutting Robot*）仿真系统：麦克纳姆轮全向底盘 + 直角坐标式 4 自由度机械臂，覆盖 **SLAM 建图 → Nav2 自主导航 → 视觉伺服对位 → 横平竖直开槽轨迹规划 → 机械臂执行** 的完整作业链路。

![Gazebo 仿真场景](docs/images/gazebo_world.png)

---

## ✨ 主要功能

| 模块 | 内容 |
|---|---|
| 🛞 **全向底盘** | 麦克纳姆轮逆运动学控制器，`/cmd_vel` → 四轮速度，支持横移/斜移/原地旋转 |
| 🦾 **机械臂** | 3 移动 + 1 旋转的直角坐标机械臂，闭式正/逆运动学，YAML 声明式轨迹任务 |
| 🗺️ **SLAM 建图** | SLAM Toolbox 异步建图，2D 激光雷达 |
| 🧭 **自主导航** | Nav2（AMCL 定位 + DWB 局部规划 + 行为树恢复） |
| 🎯 **视觉伺服** | 相机 + 激光融合 PBVS：激光拟合墙面、相机检测标记并与墙面求交得三维位置，闭环驱动全向底盘对位；仿真真值误差约 2 mm |
| 📐 **增强 A\* 规划** | 面向开槽"横平竖直"工艺约束：正交方向约束 + 曼哈顿启发 + 转向惩罚 + 路径平滑 |

<table>
<tr>
<td><img src="docs/images/rviz_map.png" alt="RViz 导航"></td>
<td><img src="docs/images/costmap.png" alt="代价地图"></td>
</tr>
<tr>
<td align="center">RViz 中的地图与导航</td>
<td align="center">Nav2 多层代价地图</td>
</tr>
<tr>
<td><img src="docs/images/arm_closeup.png" alt="机械臂"></td>
<td><img src="docs/images/camera_view.png" alt="相机视角"></td>
</tr>
<tr>
<td align="center">直角坐标机械臂</td>
<td align="center">前视相机视角</td>
</tr>
</table>

---

## 🎯 视觉伺服对位鲁棒性

相机 + 激光融合 PBVS 与“里程计 + 已知标记坐标”基线的对比（每条件 8 次随机起点，Gazebo 真值评估）：

![对位鲁棒性](docs/experiments/alignment_robustness/robustness.png)

无扰动时视觉伺服 2.0 mm、里程计基线 4.0 mm；里程计漂移 3 / 10 cm 时基线误差升至 29 / 84 mm、刀具离墙间隙被压缩到个位毫米，视觉伺服保持 1.9–2.7 mm、间隙 ≥ 20 mm。接近墙面时机械臂缩回，并有按制动距离限速的激光防碰撞守护（开启时 64 次试验零碰墙）。详见 [实验报告](docs/experiments/alignment_robustness/README.md)。

---

## 📐 增强 A\* 开槽轨迹规划

水电开槽规范要求线槽**横平竖直**，而标准 A\* 偏好对角捷径，会产生大量斜线和密集拐点。增强 A\* 通过以下改造使轨迹完全正交：

- **方向约束**：重惩罚斜向移动，路径只由水平/竖直段构成
- **曼哈顿距离启发**：与正交运动的度量一致
- **转向惩罚**：在代价函数中对方向变化加罚，压低拐点数量
- **路径平滑**：后处理去除冗余共线点，只保留转折点

![墙面开槽三算法对比](src/path_planning/wall_cutting_comparison.png)

**墙面开槽场景**（3.2 m × 2.8 m 墙面，含配电箱、插座、承重梁、窗户，障碍膨胀 8 cm）：

| 指标 | 标准 A\* | 改进 A\*（转向惩罚） | **增强 A\*** |
|---|---|---|---|
| 转向次数 | 95 | 3 | **1** |
| 横平竖直比例 | 50.7 % | 50.7 % | **100 %** |
| 斜切比例 | 49.3 % | 49.3 % | **0 %** |
| 切割总长度 | 2.66 m | 2.66 m | 3.30 m |

**底盘导航场景**（SLAM 地图 `my_map.pgm`，299×260，0.05 m/格）：

| 指标 | 标准 A\* | 改进 A\* | **增强 A\*** |
|---|---|---|---|
| 路径长度 | 13.06 m | 13.06 m | 17.25 m |
| 转弯次数 | 27 | 2 | **4** |
| 横平竖直比例 | 29.2 % | 29.2 % | **100 %** |
| 搜索节点数 | 7001 | 7191 | 20779 |

增强 A\* 以约 24 %–32 % 的路径长度增加，换取完全符合工艺要求的正交轨迹。

---

## 📁 项目结构

```
.
├── src/
│   ├── scr_bringup/         # 一键启动（仿真 + 导航 + RViz，支持轻量模式）
│   ├── scr_description/      # URDF/网格模型、Gazebo 世界、控制器配置、麦轮控制器
│   ├── scr_navigation/       # SLAM Toolbox / Nav2 配置、地图、RViz 配置
│   ├── scr_arm_controller/   # 笛卡尔运动学、YAML 轨迹任务、标记扫描、导航+切割总流程
│   ├── scr_planner/          # C++ 方向感知增强 A* + 施工规范约束的墙面开槽规划、停靠站划分与顺序优化
│   └── path_planning/        # 标准/改进/增强 A* 实现与对比实验（第一版规划器纯 Python 参考实现）
├── docker/                   # Docker 环境（osrf/ros:humble-desktop-full）
├── docs/images/              # README 图片
└── *.md                      # 操作文档与专题指南
```

---

## 🚀 快速开始

### 环境

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo 11（`ros-humble-gazebo-ros-pkgs`）
- Nav2、SLAM Toolbox、ros2_control

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-teleop-twist-keyboard \
  ros-humble-robot-localization ros-humble-cv-bridge
```

### 编译

```bash
git clone git@github.com:Aemeath-BonVoage/indoor-wall-slot-cutting-robot-path-planning.git ~/colcon_ws
cd ~/colcon_ws
# 也可以用 rosdep 一键安装依赖（首次需 sudo rosdep init && rosdep update）
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 一键启动（推荐）

```bash
ros2 launch scr_bringup sim_nav.launch.py                        # Gazebo + Nav2 + RViz
ros2 launch scr_bringup sim_nav.launch.py gui:=false             # 不开 Gazebo 窗口
ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false # 纯后台，最省内存
ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5  # 批量实验：只启动仿真，约 2.3 倍速

# 导航就绪后（约 20–30 s）发送目标点
ros2 run scr_navigation send_goal.py 1.5 0.0 0.0
```

> 实测（4 核 / 4 GB 虚拟机，纯后台模式）：仿真实时率 ≈ 1.0，仿真 + 完整 Nav2 额外占用约 400 MB 内存；完整“导航 → 对位 → 扫描 → 切割”任务约 100 s。

### 分步运行

```bash
# 1. 启动 Gazebo 仿真（gui:=false 可关闭窗口）
ros2 launch scr_description gazebo.launch.py

# 2a. SLAM 建图（键盘遥控建图后保存）
ros2 launch scr_navigation slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f src/scr_navigation/maps/my_map

# 2b. 或直接使用已有地图进行导航
ros2 launch scr_navigation navigation.launch.py

# 3. 完整任务：导航 → 扫描标记 → 精确对位 → 开槽
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
```

只运行机械臂开槽：

```bash
ros2 run scr_arm_controller custom_cutting wall_cutting.yaml
```

### 路径规划对比实验（无需 ROS）

```bash
pip install numpy matplotlib pillow
cd src/path_planning
python3 improved_astar.py        # 底盘层：基于 SLAM 地图的三算法对比
python3 wall_cutting_planner.py  # 墙面层：开槽轨迹三算法对比
```

更多细节见 [操作文档.md](操作文档.md)、[NAVIGATION_CUTTING_GUIDE.md](NAVIGATION_CUTTING_GUIDE.md)、[MARKER_SCANNING_GUIDE.md](MARKER_SCANNING_GUIDE.md)。

---

## 🤖 机器人参数

| 项目 | 参数 |
|---|---|
| 底盘 | 麦克纳姆轮，轮径 100 mm，轮距 593.16 × 493.80 mm |
| 机械臂工作空间 | X 0.517–0.582 m，Y −0.060–0.080 m，Z 0.157–0.707 m（相对 `base_link`，TF 实测） |
| 关节 | joint1 Z 升降 / joint2 Y 横移 / joint3 X 伸缩 / joint4 末端旋转 ±90° |
| 传感器 | 2D 激光雷达、前视 RGB 相机 |

---

## 🗺️ Roadmap

完整路线见 [docs/ROADMAP.md](docs/ROADMAP.md)。

- [ ] 增强 A\* 用 C++ 重写为 Nav2 全局规划器插件，在 Gazebo 中直接驱动底盘
- [ ] 墙面开槽规划节点，规划结果在 RViz 可视化并驱动机械臂执行
- [ ] 按机械臂工作空间自动分段，多段开槽 + 底盘重定位的任务级规划
- [ ] 随机场景生成器与批量基准测试
- [ ] 迁移到 Gazebo Harmonic（ros_gz）

---

## 📄 关于

本项目为本科毕业设计《面向室内墙体线槽开槽任务的机器人自主路径规划设计》的配套代码。
