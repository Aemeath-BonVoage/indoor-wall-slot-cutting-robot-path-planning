# 黄色方块扫描功能 - 使用指南

## 功能概述

在精确定位到黄色方块正前方后，机械臂会自动进行上下扫描，使用相机检测黄色方块的精确位置，并返回其相对于base_link的坐标。

## 工作流程

1. **导航阶段**：机器人导航到目标区域
2. **精确定位阶段**：使用里程计闭环控制到达方块正前方
3. **扫描阶段**（新增）：
   - 机械臂从下往上移动（Z轴）
   - 每个高度停留并采集相机图像
   - 使用HSV颜色空间检测黄色方块
   - 计算方块在base_link坐标系中的位置
4. **切割阶段**：执行预定义的切割任务

## 扫描原理

### 1. 颜色检测
- 使用HSV色彩空间检测黄色
- HSV范围：H(20-30), S(100-255), V(100-255)
- 形态学操作去除噪声
- 轮廓检测找到最大黄色区域

### 2. 距离估算
使用相似三角形原理：
```
distance = (real_size × focal_length) / image_size
```
- 假设方块实际大小：5cm
- 焦距从相机FOV计算：focal_length = (image_width/2) / tan(fov/2)

### 3. 坐标转换
```
相机坐标系 → camera_optical_link → base_link
```
使用TF2进行坐标变换

## 配置参数

在配置文件中添加 `scanning` 部分：

```yaml
scanning:
  enabled: true      # 是否启用扫描
  z_start: 0.3       # 扫描起始高度（米）
  z_end: 0.7         # 扫描结束高度（米）
  z_step: 0.05       # 扫描步进（米）
```

### 参数说明

- **enabled**: 是否启用扫描功能
  - `true`: 启用扫描
  - `false`: 跳过扫描，直接执行切割

- **z_start**: 扫描起始高度
  - 相对于base_link的Z坐标
  - 建议范围：0.2 - 0.4m
  - 默认：0.3m

- **z_end**: 扫描结束高度
  - 相对于base_link的Z坐标
  - 建议范围：0.6 - 0.8m
  - 默认：0.7m

- **z_step**: 扫描步进
  - 每次移动的高度增量
  - 建议范围：0.03 - 0.1m
  - 默认：0.05m（5cm）
  - 步进越小，扫描越精细，但耗时越长

## 使用方法

### 1. 完整配置示例

```yaml
# task_at_target.yaml
navigation:
  x: 8.61277
  y: -2.01879
  yaw: -1.64894

precise_positioning:
  enabled: true
  distance_from_marker: 0.6

scanning:
  enabled: true
  z_start: 0.3
  z_end: 0.7
  z_step: 0.05

points:
  - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.482, rotation: 0}
  - {x: 0.537, y: 0.025, z: 0.532, rotation: 0}
  - {x: 0.537, y: -0.025, z: 0.532, rotation: 0}

motion:
  duration: 3.0
  return_home: true
```

### 2. 运行任务

```bash
# 编译
colcon build --packages-select scr_arm_controller
source install/setup.bash

# 运行
ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
```

### 3. 单独测试扫描器

可以单独运行扫描器节点进行测试：

```bash
# 终端1：启动仿真
ros2 launch scr_description gazebo.launch.py

# 终端2：运行扫描器
ros2 run scr_arm_controller marker_scanner

# 终端3：查看相机图像
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## 输出信息

### 扫描过程日志

```
============================================================
步骤2.5: 扫描寻找黄色方块
============================================================
扫描范围: Z=0.30m 到 0.70m
扫描步进: 0.05m
固定位置: X=0.537m, Y=0.000m

扫描点 1: Z=0.300m
发送目标: [...]
等待图像采集...

扫描点 2: Z=0.350m
发送目标: [...]
等待图像采集...

扫描点 3: Z=0.400m
发送目标: [...]
等待图像采集...
检测到黄色方块！图像位置: (320, 240), 面积: 1250px, 距离: 0.856m

✅ 检测到黄色方块！位置: X=0.5234, Y=-0.0012, Z=0.4987

机械臂回到初始位置

✅ 方块检测成功！
  相对于base_link的位置:
    X = 0.5234 m
    Y = -0.0012 m
    Z = 0.4987 m
```

### 未检测到方块

```
扫描点 8: Z=0.650m
发送目标: [...]
等待图像采集...

扫描点 9: Z=0.700m
发送目标: [...]
等待图像采集...

⚠️  扫描完成，未检测到黄色方块

机械臂回到初始位置

⚠️  未检测到方块，使用默认切割点
```

## 扫描性能

### 扫描时间估算

```
总时间 = 移动次数 × (移动时间 + 停留时间)
移动次数 = (z_end - z_start) / z_step + 1
```

示例（默认参数）：
- 移动次数 = (0.7 - 0.3) / 0.05 + 1 = 9次
- 移动时间 = 1.5秒/次
- 停留时间 = 2.0秒/次
- 总时间 ≈ 9 × 3.5 = 31.5秒

### 优化建议

1. **快速扫描**（牺牲精度）：
   ```yaml
   z_step: 0.1    # 更大步进
   ```
   时间：约16秒

2. **精细扫描**（提高精度）：
   ```yaml
   z_step: 0.03   # 更小步进
   ```
   时间：约50秒

3. **缩小范围**（已知大致高度）：
   ```yaml
   z_start: 0.4
   z_end: 0.6
   z_step: 0.05
   ```
   时间：约17.5秒

## 相机参数

从URDF配置：
- **位置**：安装在link1上
  - X: 0.11m
  - Y: 0.0215m
  - Z: 0.5m（相对于link1）
  
- **视场角**：80度（1.3962634 rad）
- **分辨率**：640×480
- **帧率**：30 FPS
- **话题**：
  - 图像：`/front_camera/image_raw`
  - 相机信息：`/front_camera/camera_info`

## 故障排查

### 1. 无法检测到方块

**可能原因**：
- 方块不在相机视野内
- 光照条件不佳
- 颜色阈值不匹配

**解决方法**：
- 调整扫描范围（z_start, z_end）
- 调整机器人距离（distance_from_marker）
- 修改HSV颜色阈值（在marker_scanner.py中）

### 2. 检测位置不准确

**可能原因**：
- 距离估算误差
- TF变换延迟
- 相机标定不准

**解决方法**：
- 减小扫描步进（提高采样率）
- 增加停留时间（更多图像帧）
- 进行相机标定

### 3. 扫描时间过长

**解决方法**：
- 增大z_step
- 缩小扫描范围
- 如果已知方块高度，直接移动到该高度

### 4. TF变换失败

**检查**：
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

### 5. 相机图像无输出

**检查**：
```bash
# 查看话题列表
ros2 topic list | grep camera

# 查看图像话题
ros2 topic echo /front_camera/image_raw --once

# 使用rqt查看图像
ros2 run rqt_image_view rqt_image_view /front_camera/image_raw
```

## 高级功能

### 1. 调整颜色检测阈值

编辑 `marker_scanner.py`：

```python
# 黄色HSV范围
self.yellow_lower = np.array([20, 100, 100])  # H, S, V 下限
self.yellow_upper = np.array([30, 255, 255])  # H, S, V 上限
```

HSV颜色参考：
- 黄色：H=20-30
- 橙色：H=10-20
- 绿色：H=40-80
- 蓝色：H=100-130
- 红色：H=0-10 或 170-180

### 2. 调整检测灵敏度

```python
# 最小轮廓面积（像素）
if area > 100:  # 增大此值可减少误检
```

### 3. 根据检测结果调整切割点

在 `navigation_cutting_task.py` 中，检测到方块后可以动态调整切割点：

```python
if marker_pos is not None:
    # 根据检测到的方块位置调整切割点
    detected_x, detected_y, detected_z = marker_pos
    
    # 例如：在检测到的位置周围切割
    adjusted_points = [
        (detected_x, detected_y - 0.025, detected_z - 0.025, 0),
        (detected_x, detected_y + 0.025, detected_z - 0.025, 0),
        (detected_x, detected_y + 0.025, detected_z + 0.025, 0),
        (detected_x, detected_y - 0.025, detected_z + 0.025, 0),
    ]
```

## 下一步开发

可能的改进方向：
1. 使用深度相机获取精确距离
2. 添加ArUco标记识别
3. 实现多方块检测和选择
4. 添加自适应扫描（根据初步检测调整范围）
5. 使用机器学习提高检测鲁棒性
6. 添加实时图像显示和调试界面
