# Roadmap

目标：把毕设原型升级为一个**可复现、可扩展、可做研究**的移动操作机器人项目。

每个阶段完成的标准都是**在 Gazebo / RViz 中实际跑通并留下可复现的命令或数据**。

---

## 阶段 0 · 工程基础与稳定性

- [x] git 仓库、`.gitignore`、README
- [x] 修复 `package.xml` 依赖声明（Humble 下 `rosdep install` 可用）
- [x] 一键启动：`scr_bringup` 包，单条命令启动 仿真 + 导航 + RViz
- [x] 轻量模式：`gui:=false`（无 Gazebo 窗口）、`rviz:=false`，适配低内存机器
- [x] 修复导航 launch 默认地图路径（`map.yaml` → `my_map.yaml`）
- [x] 修复 Ctrl+C 退出时 `rclpy.shutdown()` 重复调用导致的报错
- [x] 记录内存占用：纯后台模式 仿真 + Nav2 约 +400 MB，Nav2 启动后约 20–30 s 就绪

## 阶段 1 · 导航跑通与调优

- [ ] 用现有地图完整跑通 `navigation_cutting_task`（导航 → 扫描 → 对位 → 切割）
- [ ] AMCL、代价地图膨胀、DWB 速度参数调优
- [ ] 自动化测量对位误差（多次重复，统计均值 / 方差）

## 阶段 2 · 增强 A\* 进入 ROS 系统

- [ ] **C++ 实现增强 A\***，作为 Nav2 全局规划器插件（`nav2_core::GlobalPlanner`）
  - 搜索状态加入方向 `(x, y, θ)`，保证转弯数最优
  - 在 `nav2_params.yaml` 中替换 `NavfnPlanner`，Gazebo 中直接驱动底盘
- [ ] Python 版保留为参考实现，用同一组场景校验 C++ 版正确性并对比耗时
- [ ] 墙面开槽规划节点：读取墙面障碍 → 规划正交轨迹 → 按机械臂工作空间分段 → 生成 `cartesian_cutting` 路径点

## 阶段 3 · 开槽执行可视化

- [ ] RViz Marker：墙面障碍、膨胀安全区、规划轨迹、刀具实际轨迹
- [ ] Gazebo 中加入配电箱 / 插座等墙面实体
- [ ] 录制完整演示视频 / GIF 放入 README

## 阶段 4 · 研究扩展

- [ ] **多段开槽任务规划**：多条线槽 + 机械臂小工作空间 → 联合优化开槽顺序、底盘停靠点与切割路径
- [ ] 随机墙面场景生成器 + 批量基准测试（数百场景统计结果）
- [ ] 与 Theta\*、JPS、分层规划等基线对比
- [ ] 视觉感知闭环：深度相机检测墙面障碍，自动生成墙面障碍地图

## 阶段 5 · 技术栈升级

- [ ] Gazebo Classic → Gazebo Harmonic（`ros_gz`）
- [ ] 机械臂与底盘统一走 `ros2_control`
- [ ] GitHub Actions CI：编译 + 单元测试 + 代码风格检查
