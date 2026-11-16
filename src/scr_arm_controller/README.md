# scr_arm_controller

SCR机器人机械臂控制功能包

## 机械臂关节说明

机械臂有4个关节，采用串联结构：

- **joint1**: Z轴上下移动（升降），范围 -0.35 ~ 0.2 m
  - parent: base_link → child: link1
  - 关节值增加 → 末端向上
- **joint2**: Y轴左右移动（横移），范围 -0.08 ~ 0.06 m
  - parent: link1 → child: link2
  - 关节值增加 → 末端向右（-Y方向）
- **joint3**: X轴前后移动（伸缩，平行于机器人运动方向），范围 -0.045 ~ 0.02 m
  - parent: link2 → child: link3
  - 关节值增加 → 末端向后（-X方向）
- **joint4**: 末端旋转，范围 -1.57 ~ 1.57 rad (-90° ~ 90°)
  - parent: link3 → child: link4

**初始位置（所有关节=0）：**
- 末端位置: X=0.537m, Y≈0m, Z=0.507m（离地面约50cm）

**笛卡尔空间工作范围（相对于base_link）：**
- X轴: [0.517, 0.582] m（范围 0.065m，前后伸缩）
- Y轴: [-0.060, 0.080] m（范围 0.140m，左右横移）
- Z轴: [0.157, 0.707] m（范围 0.550m，上下升降）

**注意**：
- base_link坐标系：X向前，Y向左，Z向上（ROS标准）
- joint1和joint2构成的平面垂直于机器人运动平面，可用于在墙面上画图
- joint3的运动方向平行于机器人前后运动方向

## 功能

- 关节空间直接控制
- 方形轨迹演示（关节空间）
- 笛卡尔空间切割控制（逆运动学）
- 自定义四点切割

## 目录结构

```
scr_arm_controller/
├── scr_arm_controller/          # Python模块
│   ├── __init__.py
│   ├── square_demo.py           # 方形演示（关节空间）
│   ├── cartesian_cutting.py     # 笛卡尔切割（固定示例）
│   ├── custom_cutting.py        # 自定义切割（命令行参数）
│   └── joint_direction_check.py # 关节方向测试（需仿真运行）
├── config/                      # 配置文件
│   └── arm_params.yaml          # 机械臂参数
└── test/                        # 测试文件
```

## 使用方法

### 1. 编译功能包

```bash
cd /workspace
colcon build --packages-select scr_arm_controller
source install/setup.bash
```

### 2. 运行方形演示（关节空间）

启动Gazebo仿真（终端1）：
```bash
ros2 launch scr_description gazebo.launch.py
```

运行方形演示（终端2）：
```bash
ros2 run scr_arm_controller square_demo
```

这个演示会用joint1（上下）和joint2（左右）画一个5cm×5cm的正方形。

### 3. 笛卡尔空间切割（固定示例）

启动Gazebo仿真后，运行：
```bash
ros2 run scr_arm_controller cartesian_cutting
```

这个演示会在初始位置附近画一个5cm×5cm的矩形，使用笛卡尔坐标控制。

### 4. 自定义四点切割（YAML配置文件）

**重要：笛卡尔空间工作范围限制**

在使用自定义切割前，请确保所有点都在工作范围内：

| 轴 | 最小值 | 最大值 | 范围 | 说明 |
|---|--------|--------|------|------|
| X | 0.517 m | 0.582 m | 0.065 m | 前后伸缩（joint3控制） |
| Y | -0.080 m | 0.060 m | 0.140 m | 左右横移（joint2控制） |
| Z | 0.157 m | 0.707 m | 0.550 m | 上下升降（joint1控制） |

初始位置（所有关节=0）：X=0.537m, Y≈0m, Z=0.507m

**使用方法：**

通过YAML配置文件指定切割任务，支持任意数量的点（至少2个）：

```bash
# 使用预定义的配置文件
ros2 run scr_arm_controller custom_cutting wall_cutting.yaml
ros2 run scr_arm_controller custom_cutting horizontal_cutting.yaml
ros2 run scr_arm_controller custom_cutting complex_sequence.yaml

# 使用自定义配置文件（完整路径）
ros2 run scr_arm_controller custom_cutting /path/to/my_task.yaml
```

**配置文件格式：**

```yaml
# 轨迹点列表（可以是任意数量的点）
# rotation 单位为度，控制 joint4
points:
  - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.532, rotation: 0}
  - {x: 0.537, y: -0.025, z: 0.532, rotation: 0}

# 运动参数
motion:
  duration: 3.0      # 每段运动的持续时间（秒）
  return_home: true  # 完成后是否回到初始位置
```

**预定义配置文件：**

1. `wall_cutting.yaml` - 墙面切割（X固定，YZ变化）
2. `horizontal_cutting.yaml` - 水平面切割（Z固定，XY变化）
3. `cutting_task.yaml` - 通用模板
4. `complex_sequence.yaml` - 复杂动作序列示例
5. `retract_rotate_extend.yaml` - 伸缩-旋转-伸出示例

配置文件位置：`install/scr_arm_controller/share/scr_arm_controller/config/`

**示例1 - 墙面切割（wall_cutting.yaml）：**
```yaml
# X固定，YZ变化，使用 joint1(Z) 和 joint2(Y)
points:
  - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.532, rotation: 0}
  - {x: 0.537, y: -0.025, z: 0.532, rotation: 0}
motion:
  duration: 3.0
  return_home: true
```

**示例2 - 水平面切割（horizontal_cutting.yaml）：**
```yaml
# Z固定，XY变化，使用 joint2(Y) 和 joint3(X)
# 每个点末端旋转90度
points:
  - {x: 0.520, y: -0.025, z: 0.507, rotation: 90}
  - {x: 0.520, y: 0.025, z: 0.507, rotation: 90}
  - {x: 0.554, y: 0.025, z: 0.507, rotation: 90}
  - {x: 0.554, y: -0.025, z: 0.507, rotation: 90}
motion:
  duration: 3.0
  return_home: true
```

**示例3 - 伸缩-旋转-伸出（retract_rotate_extend.yaml）：**
```yaml
# 演示复杂动作：伸出 -> 缩回 -> 旋转 -> 伸出
points:
  - {x: 0.545, y: 0.000, z: 0.507, rotation: 0}   # 伸出
  - {x: 0.520, y: 0.000, z: 0.507, rotation: 0}   # 缩回（只改变X）
  - {x: 0.520, y: 0.000, z: 0.507, rotation: 45}  # 旋转45度（只改变rotation）
  - {x: 0.545, y: 0.000, z: 0.507, rotation: 45}  # 再次伸出
  - {x: 0.545, y: 0.000, z: 0.507, rotation: 90}  # 继续旋转到90度
  - {x: 0.520, y: 0.000, z: 0.507, rotation: 90}  # 缩回
  - {x: 0.520, y: 0.000, z: 0.507, rotation: 0}   # 旋转回0度
  - {x: 0.545, y: 0.000, z: 0.507, rotation: 0}   # 伸出到原位
motion:
  duration: 2.5
  return_home: true
```

**关键特性：**
- 支持任意数量的点（不限于4个）
- 每个点可以独立控制 X、Y、Z 和 rotation
- 通过只改变一个参数实现单轴运动：
  - 只改变 X → 前后伸缩（joint3）
  - 只改变 Y → 左右横移（joint2）
  - 只改变 Z → 上下升降（joint1）
  - 只改变 rotation → 末端旋转（joint4）
- 可以组合多个动作实现复杂轨迹

### 5. 运动顺序

笛卡尔切割的运动顺序：
1. 回到初始位置 [0, 0, 0, 0]
2. 移动到点1
3. 移动到点2
4. 移动到点3
5. 移动到点4
6. 回到点1（闭合矩形）
7. 回到初始位置

## 逆运动学说明

笛卡尔切割使用简化的逆运动学求解：

```python
# 初始位置（所有关节=0）: X=0.537m, Y≈0m, Z=0.507m
# 目标位置: (x, y, z)

joint1 = z - 0.507  # Z轴控制
joint2 = -(y - 0.0) # Y轴控制（反向）
joint3 = -(x - 0.537) # X轴控制（反向）
joint4 = rotation   # 旋转
```

**注意**：
- 这是基于机械臂结构的解析解
- 自动检查关节限位
- 不可达的点会被拒绝

## 开发说明

### 关节空间控制

参考 `square_demo.py` 的实现方式：
- 使用 `FollowJointTrajectory` Action
- 直接指定关节位置
- 等待每个动作完成后再执行下一个

### 笛卡尔空间控制

参考 `cartesian_cutting.py` 的实现方式：
- 使用逆运动学将笛卡尔坐标转换为关节值
- 检查关节限位确保可达性
- 使用相同的 Action 接口控制
