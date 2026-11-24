# 仿真器迁移评估：从 Gazebo Classic 11 到新版 Gazebo

**结论**：本仓库对 Gazebo Classic 的依赖集中在 `scr_description` 与 `scr_base_control` 两个包：机器人描述中 8 个 Gazebo 插件（其中 2 个是自写的模型插件），1 个自写的 ros2_control 仿真硬件接口，1 个世界文件与 6 个定位标记模型，2 个启动文件，以及约 12 个直接调用 Gazebo 话题或服务的 Python 脚本。传感器、关节状态与启动方式都有官方的直接替代；三个自写组件（力控平面运动插件 `scr_planar_move`、切割力插件 `scr_cutting_force`、麦克纳姆仿真硬件接口 `ScrMecanumBaseSystem`）必须按新版的“系统”接口重写，机械臂关节的 `position_pid` / `velocity_pid` 指令接口在 `gz_ros2_control` 中没有对应项。估计工作量约 27–42 人日，另需约 2–3 天的机器时间重做验证与时间模型标定。**建议现在不迁移**：已提交的全部 Gazebo 结果都在 Classic 上取得，路线图上仍在进行的 Gazebo 验证需要与它们可比；迁移应在这批实验完成后、与 ROS 2 Jazzy + Gazebo Harmonic 的升级一起进行，并先用一个小规模试点在本机测量资源占用。本报告是文献与代码的桌面研究，没有安装新版 Gazebo，也没有运行任何新仿真。

## 1. 问题

本项目的仿真基于 ROS 2 Humble + Gazebo Classic 11。`gazebo_ros2_control` 的仓库说明 Gazebo Classic 已于 2025 年 1 月停止维护、该包不再更新，且 Classic 没有发布到 Ubuntu 24.04（Noble），因此该包从未发布到 Jazzy [7]。本报告回答：

1. 迁移到新版 Gazebo 时，本仓库的哪些插件、传感器、ros2_control 接口、世界 / 模型文件与启动文件受影响，哪些有直接替代，哪些必须重写；
2. 目标版本怎么选：Humble 配套的版本，以及换到 Jazzy 时的变化；
3. 在本机（4 核、约 4 GB 内存、无独立显卡）上的资源占用预期；
4. 对已提交结果的可复现性有什么风险；
5. 迁移的顺序、工作量与建议。

## 2. 方法

- **清单**：从源码树中枚举全部 Gazebo 依赖：`grep` 机器人描述与世界文件中的 `<plugin>`、`<sensor>`、`<ros2_control>`，C++ 源码中的 `gazebo/` 头文件与 API 调用，Python 中对 `/gazebo/*` 话题与服务、`gazebo_msgs`、`spawn_entity` 的使用，`package.xml` 中的依赖，以及启动文件。
- **官方资料**：逐页阅读第 10 节列出的官方迁移文档与软件包说明（2026 年 11 月访问）。文中凡依据这些页面的陈述都标出来源编号；页面上没有、凭对这些软件的一般了解写出的内容标为 **[未核对]**。
- **资源占用**：现有数据取自已提交的实验报告；另在本机运行其他 Gazebo 实验时用 `ps` 抽样一次 `gzserver` 的常驻内存与处理器占用（单次快照，见第 5 节）。新版 Gazebo 的占用没有测量，只能给出待测项与判据。

## 3. 现状清单

### 3.1 机器人描述中的插件与接口（`scr_description/urdf/scr_robot_gazebo.urdf`）

| 组件 | 现在 | 作用 | 新版中的对应 | 类别 |
|---|---|---|---|---|
| `libgazebo_ros2_control.so` + `gazebo_ros2_control/GazeboSystem` | 机械臂 4 个关节 `position_pid`，4 个轮子 `velocity_pid` | 机械臂与（插件模式下）轮子的 ros2_control 接口 | `gz_ros2_control-system` + `gz_ros2_control/GazeboSimSystem`；指令接口只有 position / velocity / effort，position 由“速度 = 增益 × 误差 × 更新率”实现，没有 PID 接口 [5, 6] | 有替代，但**行为不同**（4.2 节） |
| `ScrMecanumBaseSystem`（`scr_base_control`，314 行） | 继承 `gazebo_ros2_control::GazeboSystemInterface`，用 `gazebo::physics` 读位姿、对底盘施力、对轮子 `SetForce`，内含编码器误差模型与模拟陀螺 | 默认底盘的仿真硬件接口 | 自定义硬件接口须继承 `gz_ros2_control::GazeboSimSystemInterface`，由 pluginlib 加载，官方有示例 `GazeboCustomSimSystem` [5, 6] | **重写**（4.3 节） |
| `libscr_planar_move.so`（`scr_description`，195 行） | `gazebo::ModelPlugin`：订阅 `/cmd_vel`，速度 P 环对底盘施力，发布 `/odom` 与 TF | 早期配置（`base_control:=plugin`） | Classic 模型插件对应新版的“系统”（`ISystemConfigure` 替代 `Load`，更新回调替代事件连接，通过实体组件读写状态）[3] | **重写或停用**（4.4 节） |
| `libscr_cutting_force.so`（`scr_description`，279 行） | `gazebo::ModelPlugin`：按切深与刀片相对墙面的速度计算力旋量，加在底盘或刀具上，发布 `/cutting_force/wrench` | 切割力验证（默认关闭） | 同上，重写为系统 | **重写**（4.4 节） |
| `libgazebo_ros_joint_state_publisher.so` | 发布关节状态 | 机械臂与轮子的关节状态 | `gz::sim::systems::JointStatePublisher` + 话题桥 [2]；或直接用 ros2_control 的 `joint_state_broadcaster` | 直接替代 |
| `libgazebo_ros_p3d.so` | 50 Hz 发布 `/ground_truth/odom` | 评估用真值 | 新版有位姿 / 里程计发布系统，经 `ros_gz_bridge` 转为 ROS 话题 **[未核对具体系统名与输出频率]** | 有替代 |
| `libgazebo_ros_ray_sensor.so`（`ray`，360 线，10 Hz，σ = 1 cm） | 二维激光 | 墙面拟合、AMCL、代价地图 | `ray` 不再支持，改用 `type="gpu_lidar"`，世界中需加载 `Sensors` 系统（渲染引擎 ogre2），经 `ros_gz_bridge` 桥接 [2] | 有替代，**依赖渲染** |
| `libgazebo_ros_camera.so`（640 × 480，水平视场 80°，30 Hz） | 前视相机 | AprilTag 检测 | 通用 `camera` 传感器 + `ros_gz_image` 图像桥 [2] | 有替代，**依赖渲染**；内参约定需复核（6 节） |

### 3.2 世界与模型

| 文件 | 内容 | 迁移要点 |
|---|---|---|
| `worlds/room.world` | 20 个 `<model>`；`model://sun`、`model://ground_plane`；世界插件 `libgazebo_ros_state.so`、`libgazebo_ros_properties.so`；6 个标记模型的 `<include>` | `sun` 与 `ground_plane` 改为 Fuel 地址或本地模型；一旦写了任何世界插件，默认插件不再自动加入，需要显式加 `Physics`、`UserCommands`、`SceneBroadcaster`、`Sensors`、`Imu` [2]；`GAZEBO_MODEL_PATH` 改为 `GZ_SIM_RESOURCE_PATH` [4] |
| `models/scr_apriltag_tag36h11_{0..5}` 与生成脚本 `scripts/generate_apriltag_model.py` | 贴图通过 Ogre 材质脚本（`<material><script>`）加载 | 新版不支持 Ogre 材质脚本，要改用 `ambient` / `diffuse` 与 PBR 贴图 [4]；生成脚本同步修改。标记的打印尺寸与贴图边界直接影响检测精度，迁移后要复核 |
| `scr_viz/scr_viz/sdf_gen.py`、`spawn_wall_fixtures.py` | 生成并在线生成墙面线盒 / 插座等外观实体 | 生成的 SDF 与生成服务都要改（4.5 节） |

### 3.3 启动文件与构建依赖

| 位置 | 现在 | 迁移要点 |
|---|---|---|
| `scr_description/launch/gazebo.launch.py` | 启动 `gzserver` / `gzclient`；`spawn_entity.py` 生成机器人；`rtf` 参数通过 Classic 命令行 `gz physics -u` 设置更新率；`camera_yaw_error_deg` 在生成前改写 URDF | 改用 `ros_gz_sim` 的 `gz_sim.launch.py`（无界面用 `-s`）；`spawn_entity.py` 改为 `ros_gz_sim create`（`-entity` 改为 `-name`）[2]；`rtf` 改为世界文件的物理参数或运行时服务 **[未核对]**；加 `ros_gz_bridge` 的桥接配置 [2] |
| `scr_bringup/launch/sim_nav.launch.py` | 转发 `rtf`、`base_control`、`plant`、`cutting_force` 等参数 | 参数名可以不变，内部改由新的 `gazebo.launch.py` 处理 |
| `package.xml` | `gazebo_dev`、`gazebo_ros`、`gazebo_plugins`、`gazebo_ros2_control`、`gazebo_msgs`，分布在 6 个包中 | 换为 `ros_gz_sim`、`ros_gz_bridge`、`ros_gz_image`、`gz_ros2_control` 等 [2, 5] |
| `docker/Dockerfile`、CI | 基于 `ros:humble`，经 `rosdep` 安装上述依赖 | 换依赖后镜像与 CI 同步修改；保留一个 Classic 镜像用于复现旧结果（6 节） |

### 3.4 直接使用 Gazebo 话题与服务的 Python 代码

| 接口 | 使用者 | 新版 |
|---|---|---|
| `/gazebo/set_entity_state`（传送机器人到随机起点或参考位姿） | `alignment_benchmark.py`、`teach_alignment_offset.py`、`base_characterization.py` | 经 `ros_gz` 调用世界的设置位姿服务 **[未核对接口名与消息类型]** |
| `/gazebo/model_states`、`/gazebo/link_states`（真值、刀尖） | `execute_slot_plan.py`、`plan_viz_node.py`、`trajectory_visualizer.py`、`nav_benchmark.py`、`base_hold.py`、`task_skills.py` 中的真值相关路径 | 由位姿发布系统经话题桥提供 **[未核对]**；数据包回放（`replay_rviz.py` 读回放的 `/gazebo/link_states`）的格式随之改变 |
| `/spawn_entity`、`/delete_entity`（墙面实体、演示相机） | `spawn_wall_fixtures.py`、`demo_camera.py`（经 `scr_viz/gazebo_io.py`） | `ros_gz_sim create` 与对应的删除服务 **[删除服务未核对]**；`gazebo_io.py` 已把这些调用集中在一处，改动面小 |
| `/gazebo/get_light_properties`、`/gazebo/set_light_properties`（光照扰动） | `alignment_benchmark.py` | 需要替代方案 **[未核对新版是否有等价服务]** |
| 相机主点 (W+1)/2 的补偿 | 精对位的检测器（[项目演进](../../EVOLUTION.md#34-感知系统偏差主点差-1-px) 第 3.4 节） | 新版相机发布的内参约定需要重新逐像素核对（6 节） |

## 4. 逐项分析

### 4.1 目标版本

- **ROS 2 Humble**：官方配套 Gazebo Fortress；Harmonic 可以用，但属于“谨慎使用”，需要 `packages.osrfoundation.org` 的非 ROS 官方包 `ros-humble-ros-gzharmonic`，它与 `ros-humble-ros-gz*` 冲突，并可能与仍依赖 Gazebo Classic 的 ROS 包冲突 [1]。
- **ROS 2 Jazzy**：官方配套 Gazebo Harmonic（ROS 的 vendor 包），Fortress 不兼容 [1]；Classic 没有发布到 Ubuntu 24.04，`gazebo_ros2_control` 也没有 Jazzy 版本 [7]。换到 Jazzy 等于必须迁移。
- **维护期**：Fortress 到 2027 年 5 月，Harmonic 到 2029 年 5 月 [8]。Humble 本身的维护期（2027 年 5 月）在本次读到的页面上没有出现 **[未核对]**。
- **Jazzy 带来的其他变化**（与仿真器无关，但迁移时会一起遇到）：`gz_ros2_control` 增加 `position_proportional_gain`（默认 0.1）与 `hold_joints`（默认 true）参数并支持 SDF 描述 [6]；操作系统需升级到 Ubuntu 24.04；Jazzy 的 Nav2 改用 BehaviorTree.CPP v4，而本项目的任务层基于 v3 并复用 Nav2 的 `BtActionServer`，需要同步移植 **[未核对，凭对 Nav2 发行说明的了解]**。

结论：在 Humble 上迁到 Fortress，维护期与 Humble 同时结束，之后仍要再迁一次 Humble → Jazzy 与 Fortress → Harmonic；直接迁到 Harmonic 又要在 Humble 上使用非官方包。两次迁移合并为一次（Jazzy + Harmonic）更省事。

### 4.2 机械臂关节：`position_pid` 没有对应项

仿真配置 v2 的核心改动就是把机械臂关节从位置接口改为 `position_pid` 力控：位置接口每个周期直接设定关节位置，丢掉了随底盘旋转应有的速度，是底盘转速只有指令 18–37 % 的两层根因之一（[项目演进](../../EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真) 第 4 节）。`gz_ros2_control` 只有 position / velocity / effort 三种指令接口，position 由速度指令近似实现，没有 PID 接口 [5, 6]。可选的做法：

- **effort 接口 + ros2_control 侧的 PID**：`joint_trajectory_controller` 以 effort 为指令接口时用 PID 计算力矩 **[未核对 Humble 版本的具体参数名]**。最接近现在的力控语义，控制环频率与增益需要重新整定，并重做机械臂跟踪（现为 ≤ 0.2 mm / 0.14°）与底盘转速比（现为 0.99–1.01）的受控试验。
- **position 接口**：新版通过物理引擎的关节速度指令实现，不是直接设定位置，第一层根因可能不会重现，但没有验证；它的跟踪特性（比例增益 × 更新率）与现在的 PID 不同，连续轨迹的跟踪误差与底盘反作用（第 7 章的偏航峰值）都会变。

两种做法都要求重新测量机械臂相关的全部精度与耗时数据。

### 4.3 麦克纳姆仿真硬件接口

`ScrMecanumBaseSystem` 的纯 C++ 部分（`mecanum_kinematics.hpp`、`encoder_error_model.hpp`、`MecanumDriveController`）与 Gazebo 无关，可以原样保留；与 Gazebo 交互的部分要改写为 `GazeboSimSystemInterface` 的子类 [5, 6]，涉及：

- 读底盘位姿与仿真时间（现为 `WorldPose()`、`SimTime()`）：新版用 `worldPose(entity, ecm)` 等组件访问 [3]；
- 对底盘施力（`AddForce`、`AddForceAtRelativePosition`）与对轮子关节施力矩（`SetForce`）：新版通过链接的施力接口或组件实现 **[未核对：本次读到的插件迁移页面没有描述施力方法]**；
- 速度闭环、编码器模型与陀螺改用“相邻物理步的位姿差分”是针对 Classic（ODE）中“只出现在位姿里的转动”做的修正（[system_modernization](../system_modernization/README.md) 2.4 节）。新版的默认物理引擎不同 **[未核对默认引擎]**，这一现象可能不存在，也可能以别的形式出现，需要重做 60 s 特性测试与寄生转速测量（现为 1 × 10⁻⁵ rad/s）。

### 4.4 自写模型插件

- `scr_cutting_force`：计算部分（力模型、切深时间表、进给低通）与 Gazebo 无关，可以抽成独立的库；Gazebo 侧只有读刀片位姿与速度、施加力旋量、订阅切深、发布力旋量四件事。改为系统后，需重做外力阶跃试验（现为稳态偏差 ≤ 1.3 %、14 次中 12 次峰值误差 ≤ 6 %，[cutting_force](../cutting_force/README.md) 第 10 节）。新版中系统内部没有现成的 `gazebo_ros` 节点，ROS 话题要么在系统内自建 `rclcpp` 节点，要么经 Gazebo Transport + `ros_gz_bridge` 转换 **[未核对推荐做法]**。
- `scr_planar_move`：只服务于早期配置（`base_control:=plugin`，`/odom` 为真值），默认配置不用它。建议不移植，保留在 Classic 版本中，用于复现早期数据。

### 4.5 评估与实验工具

真值、传送、实体生成与光照四类接口（3.4 节）全部要改。`scr_viz/gazebo_io.py` 已经把生成 / 删除集中在一处；真值与传送分散在约 8 个脚本中，建议先在本仓库内加一层与仿真器无关的接口（“读真值位姿”“传送到位姿”），再分别实现 Classic 与新版两种后端，使实验脚本在迁移期间两边都能运行。光照扰动是对位鲁棒性实验 9 类扰动之一，如果新版没有运行时修改光源的服务，需要改为启动时选择世界文件。

### 4.6 仿真时钟

本项目有两个缺陷来自 `/clock` 只有 10 Hz：导航确认超时与定点保持定时器成批触发（[execution_issues](../execution_issues/README.md) 第 2 节、[cutting_force](../cutting_force/README.md) 10.1 节）。新版中 `/clock` 由话题桥从 Gazebo 的时钟转发，频率与现在不同 **[未核对]**。这可能消除这类缺陷，也会改变依赖它的现有行为（例如 EKF 的输出频率只有 10 Hz 是受 `/clock` 限制）；迁移后要逐项复查执行层报告第 8 节列出的默认配置。

## 5. 资源占用

**现有数据**（Gazebo Classic，无界面，`rtf` 1）：

| 来源 | 数据 |
|---|---|
| [system_modernization](../system_modernization/README.md) 4.2 节 | 导航基准中 Gazebo 约占 45–50 % 单核；整机 4 核平均占用 36 %（MPPI）与 47 %（DWB） |
| [demo](../demo/README.md) | Gazebo + Nav2 + 任务层 + 两台演示相机 + rosbag2 录制同时运行时，可用内存约 0.8 GB；RViz 不能与仿真同时运行 |
| [cutting_force](../cutting_force/README.md) 13 节、[arm_trajectory](../arm_trajectory/README.md) 8 节 | 实际实时率约 0.8–0.85；同一台机器上同时运行其他计算任务会使任务失败 |
| 本次 `ps` 抽样（单次快照，另一组 Gazebo 实验运行中） | `gzserver` 常驻内存约 535 MB，占用约 67 % 单核 |

**新版的预期**：官方页面没有给出内存或处理器需求 [1–8]。与资源相关的两点结构性差异：

- 激光改为 `gpu_lidar`、相机改为渲染传感器，都依赖 `Sensors` 系统与 ogre2 渲染 [2]。本机没有独立显卡，渲染只能走软件实现（Mesa 的 llvmpipe 一类）**[未核对新版在纯软件渲染下的可用性与性能]**。Classic 的 `ray` 传感器不需要渲染，现在只有相机需要；迁移后激光也要经过渲染管线，30 Hz 的相机与 10 Hz、360 线的激光都在处理器上渲染。
- 新版的服务器与界面分离，无界面运行（`-s`）与现在的 `gzserver` 相当 [2]。

因此无法从资料中估计迁移后能否在 4 GB 内存、4 核上维持现在的实时率。迁移前的第一步必须是试点测量（7 节第 1 步），判据建议取：Gazebo + Nav2 + 任务层同时运行时可用内存 ≥ 0.5 GB，实时率 ≥ 0.8（与现在相当），相机与激光的实际发布频率达到设定值。达不到时，迁移需要等机器升级（更多内存或独立显卡），或者降低传感器的设定（这会改变精对位与导航的输入，属于新的仿真配置）。

## 6. 对已提交结果可复现性的风险

| 风险 | 影响的结果 | 处理 |
|---|---|---|
| 物理引擎、接触与摩擦模型不同 | 底盘转速比、零速滑移（精对位名义条件下 +0.43 mm 的解释）、定点保持漂移、切割力的对象模型核对、刚体墙“顶开”机器人的量级 | 迁移后的仿真定义为**仿真配置 v3**；v2 上的数据保留并标注，不与 v3 混在同一张表里 |
| 机械臂关节接口（4.2 节） | 连续轨迹的跟踪误差、偏航峰值、竖切起点偏差、全部时间模型预设（`traj_v1`、`cut_brick_v1` 等） | 时间模型按 v3 重标定为新预设，旧预设保留 |
| 传感器渲染与内参约定 | 精对位的感知偏差（主点补偿可能变成新的 1 px 偏差）、激光墙面拟合噪声、AprilTag 贴图 | 迁移后先重做感知精度测试（27 个已知位姿、270 帧）再看对位结果 |
| 噪声模型与随机数 | 编码器误差模型的种子配对（同种子运行之间的可比性） | 编码器模型是本仓库代码，种子语义可以保持；传感器噪声由 Gazebo 生成，序列会不同 |
| 仿真时钟频率（4.6 节） | 执行层排查中与 10 Hz 时钟有关的结论、EKF 输出频率 | 复查默认配置 |
| Classic 软件包停止更新 | 旧结果的复现环境本身 | 保留一个基于 `ros:humble` 的 Classic Docker 镜像与对应的版本标记，复现说明指向它 |

已提交的每份实验报告都在 Classic（仿真配置 v1 / v2）上取得，迁移后它们仍然有效，但不能再用当前代码直接复现，只能用保留的 Classic 版本复现。

## 7. 迁移计划与工作量

按“先测能不能跑，再换外围，最后换与精度相关的部分”的顺序。工作量是对本仓库代码量与改动面的估计（人日），不含等待仿真的机器时间；**均为估计，没有依据可核对**。

| 步骤 | 内容 | 工作量 | 完成判据 |
|---|---|---|---|
| 0 | 完成当前在 Classic 上的验证（标记地图在线估计的 16 次运行、房间任务），打上版本标记，保留 Classic Docker 镜像 | — | 路线图中依赖 Classic 可比性的条目完成 |
| 1 | **试点**：在 Docker 中装 Jazzy + Harmonic，只放房间世界与机器人（`GazeboSimSystem` 驱动机械臂、底盘先用官方示例接口），开激光与相机，测量内存、处理器与实时率 | 2–3 | 第 5 节的判据；不满足则停止迁移 |
| 2 | 构建依赖、世界文件、标记模型（材质改 PBR）、生成脚本、`GZ_SIM_RESOURCE_PATH` | 2–3 | 世界加载无警告；标记贴图在相机图像中可检测 |
| 3 | 传感器与话题桥（激光、相机、关节状态、真值），相机内参逐像素核对 | 2–3 | 感知精度测试与 Classic 相当 |
| 4 | 机械臂关节接口（4.2 节）与连续轨迹 | 3–5 | 机械臂跟踪误差、底盘转速比受控试验 |
| 5 | `ScrMecanumBaseSystem` 移植（4.3 节） | 4–6 | 底盘特性测试：速度跟踪比、寄生转速、里程计漂移 |
| 6 | 启动文件（`gazebo.launch.py`、`sim_nav.launch.py`）、`rtf`、时钟桥 | 1–2 | 一键启动；Nav2 固定 6 目标基准可运行 |
| 7 | 实验工具：真值 / 传送 / 生成 / 光照的仿真器无关接口与两种后端 | 3–5 | 对位基准、执行记录器、演示工具在新版上运行 |
| 8 | 切割力系统（4.4 节） | 2–3 | 外力阶跃试验与离线模型的核对 |
| 9 | 任务层随 Nav2 Jazzy 的移植（BehaviorTree.CPP v4）**[未核对工作量依据]** | 3–5 | 行为树结构测试与 11 站任务 |
| 10 | 重新验证与标定：精对位名义 + 2–3 个扰动条件、11 站默认配置、时间模型 `traj_v1` 的 v3 版本、沿墙精度默认配置、切割力整墙 | 3–5（另需约 2–3 天机器时间） | 各报告追加“仿真配置 v3”一节 |
| 11 | Docker、CI、使用指南、项目演进 | 2 | CI 通过；文档区分 v2 / v3 |

合计约 27–42 人日。步骤 1 的结果决定是否继续；步骤 4、5 是风险最大、最可能超出估计的两步。

## 8. 建议

**现在不迁移，以后迁移；时机是“Classic 上的验证完成之后，与 Jazzy + Harmonic 的升级一起”，前提是步骤 1 的试点在本机通过。**

理由：

1. **可比性**：路线图上仍有依赖 Gazebo 的验证（标记地图在线估计、房间任务），它们的意义在于与已提交的 Classic 结果对照。现在换仿真器，这些对照就同时混入了仿真器的变化。
2. **收益暂时不大**：Classic 在 Humble / Ubuntu 22.04 上仍可安装运行，现有 Docker 镜像可以复现全部结果；迁移本身不产生新的方法或结论，主要收益是延长可维护性。
3. **资源风险没有排除**：新版的激光也要渲染，本机没有独立显卡，能否维持现在的实时率无法从资料判断。
4. **一次迁移比两次省**：Humble + Fortress 与 Humble 同时到期 [8]，之后仍要迁到 Jazzy + Harmonic；Humble 上用 Harmonic 需要非官方包且与 Classic 依赖冲突 [1]。

触发迁移的条件：项目升级到 Ubuntu 24.04 / Jazzy；或机器升级（更多内存或独立显卡）；或需要新版才有的功能（例如更好的传感器模型）。在此之前可以先做两件与迁移无关、又能降低以后迁移成本的事：把 `scr_cutting_force` 与 `ScrMecanumBaseSystem` 中与 Gazebo 无关的计算抽成独立的库；为真值、传送与实体生成建立仿真器无关的接口（4.5 节）。

## 9. 局限

- 桌面研究：没有安装新版 Gazebo，没有任何测量；资源占用、物理行为与传感器的差异都只能列为待测项。
- 官方页面只读了第 10 节列出的 8 页，其中“最新”版本的页面对应 Gazebo Jetty，个别细节可能与 Fortress / Harmonic 不同；标 **[未核对]** 的陈述没有在这些页面中找到依据。
- 工作量估计基于代码行数与改动面，没有同类迁移的实际数据作参照。
- `gzserver` 的资源数据只有一次快照，且是在另一组实验运行时取得的。

## 10. 参考资料

（均于 2026 年 11 月访问。）

1. Gazebo 文档，Installing Gazebo with ROS（ROS 与 Gazebo 版本配套表）：https://gazebosim.org/docs/latest/ros_installation/
2. Gazebo 文档，Migrating ROS 2 packages that use Gazebo Classic（Harmonic 版）：https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/
3. gz-sim 8 API 文档，Migration from Gazebo Classic: Plugins：https://gazebosim.org/api/sim/8/migrationplugins.html
4. gz-sim 8 API 文档，Migration from Gazebo Classic: SDF：https://gazebosim.org/api/sim/8/migrationsdf.html
5. ros2_control 文档（Humble），gz_ros2_control：https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
6. ros2_control 文档（Jazzy），gz_ros2_control：https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
7. ros-controls/gazebo_ros2_control 仓库说明（Gazebo Classic 停止维护、未发布到 Jazzy）：https://github.com/ros-controls/gazebo_ros2_control
8. Gazebo 文档，Releases（各版本发布与停止维护日期）：https://gazebosim.org/docs/latest/releases/
