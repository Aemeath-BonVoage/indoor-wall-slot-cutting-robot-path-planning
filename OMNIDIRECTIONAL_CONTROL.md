# 全向移动控制说明

## 问题分析

从日志看到：
- 距离从1120mm减小到93mm后卡住
- "朝向目标角度误差"一直保持在17-24°
- 机器人无法继续接近目标

## 根本原因

**麦克纳姆轮机器人是全向移动平台！**

之前的控制只使用了：
```python
cmd.linear.x  # 前进/后退
cmd.angular.z # 旋转
```

这相当于把全向移动机器人当成差速驱动机器人使用了！

## 麦克纳姆轮的能力

麦克纳姆轮可以：
1. 前后移动（linear.x）
2. **左右平移（linear.y）** ← 之前没用！
3. 旋转（angular.z）
4. 任意方向移动（组合上述三个）

## 正确的控制策略

### 坐标系转换

全局误差 → 机器人坐标系误差：

```python
# 全局坐标系中的误差
error_x_global = target_x - current_x
error_y_global = target_y - current_y

# 转换到机器人坐标系
robot_yaw = current_yaw
error_x_robot = error_x_global * cos(-robot_yaw) - error_y_global * sin(-robot_yaw)
error_y_robot = error_x_global * sin(-robot_yaw) + error_y_global * cos(-robot_yaw)
```

### 全向移动控制

```python
# 在机器人坐标系中直接移动
cmd.linear.x = kp * error_x_robot  # 前后
cmd.linear.y = kp * error_y_robot  # 左右
cmd.angular.z = kp * error_yaw     # 旋转
```

## 控制流程

### 阶段1：距离 > 10cm
使用全向移动直接朝目标点移动：

```python
if distance_error > 0.1:
    # 转换到机器人坐标系
    error_x_robot = error_x * cos(-robot_yaw) - error_y * sin(-robot_yaw)
    error_y_robot = error_x * sin(-robot_yaw) + error_y * cos(-robot_yaw)
    
    # 全向移动
    cmd.linear.x = kp_linear * error_x_robot
    cmd.linear.y = kp_linear * error_y_robot
    
    # 同时调整朝向（朝向目标点）
    angle_to_target = atan2(error_y, error_x)
    angle_error = angle_to_target - robot_yaw
    cmd.angular.z = kp_angular * angle_error
```

### 阶段2：距离 < 10cm
只调整最终朝向：

```python
else:
    # 停止移动，只旋转
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.angular.z = kp_angular * error_yaw
```

## 预期效果

修复后的日志：

```
迭代0: 距离=1120mm, 机器人系误差(x=1050mm, y=350mm), 角度误差=11.5°, 收敛=0/5
  模式: 全向移动到目标

迭代20: 距离=850mm, 机器人系误差(x=800mm, y=250mm), 角度误差=8.2°, 收敛=0/5
  模式: 全向移动到目标

迭代40: 距离=620mm, 机器人系误差(x=580mm, y=180mm), 角度误差=5.5°, 收敛=0/5
  模式: 全向移动到目标

迭代60: 距离=380mm, 机器人系误差(x=350mm, y=120mm), 角度误差=3.2°, 收敛=0/5
  模式: 全向移动到目标

迭代80: 距离=150mm, 机器人系误差(x=140mm, y=45mm), 角度误差=1.8°, 收敛=0/5
  模式: 全向移动到目标

迭代100: 距离=65mm, 机器人系误差(x=60mm, y=20mm), 角度误差=0.9°, 收敛=0/5
  模式: 调整最终朝向

迭代120: 距离=18mm, 机器人系误差(x=17mm, y=5mm), 角度误差=2.1°, 收敛=3/5
  模式: 调整最终朝向

✅ 精确定位完成！
   最终位置: x=8.5000, y=-3.3000, yaw=-1.5708
   位置误差: 15.2mm, 角度误差: 1.23°
   总迭代次数: 127
```

注意：
- 距离应该持续减小，不会卡住
- 机器人坐标系误差（x和y）都应该减小
- 角度误差在移动阶段可能波动，但在最后阶段会收敛

## 为什么之前的方法失败

### 差速驱动方法（错误）

```python
# 只能前进和旋转
if angle_error > threshold:
    cmd.angular.z = ...  # 先转向目标
else:
    cmd.linear.x = ...   # 再前进
```

问题：
- 如果目标在侧面，需要先转90°，再前进，再转回来
- 路径不是直线，效率低
- 容易累积误差

### 全向移动方法（正确）

```python
# 可以直接斜向移动
cmd.linear.x = error_x_robot  # 前后分量
cmd.linear.y = error_y_robot  # 左右分量
cmd.angular.z = error_yaw     # 同时调整朝向
```

优点：
- 直线路径，最短距离
- 不需要先转向
- 充分利用麦克纳姆轮的能力

## 坐标系说明

### 全局坐标系（map/odom）
- X轴：东（右）
- Y轴：北（上）
- 原点：地图原点

### 机器人坐标系（base_link）
- X轴：机器人前方
- Y轴：机器人左侧
- 原点：机器人中心

### 转换公式

从全局到机器人：
```
[x_robot]   [cos(-θ)  -sin(-θ)] [x_global]
[y_robot] = [sin(-θ)   cos(-θ)] [y_global]
```

其中 θ 是机器人的朝向（yaw）

## 调试技巧

### 1. 检查速度命令

```bash
ros2 topic echo /cmd_vel
```

应该看到：
```yaml
linear:
  x: 0.15    # 前进速度
  y: -0.08   # 左移速度（负值表示右移）
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3     # 旋转速度
```

### 2. 检查机器人位置

```bash
ros2 topic echo /odom
```

### 3. 可视化

在RViz中添加：
- Odometry显示
- TF显示
- Path显示（如果有）

### 4. 手动测试全向移动

```bash
# 测试左右平移
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 测试斜向移动
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

机器人应该能够左右平移和斜向移动。

## 参数调整

如果还有问题，可以调整：

### 1. 增加线速度增益
```python
kp_linear = 0.8  # 从0.5增加到0.8
```

### 2. 分别调整X和Y的增益
```python
kp_linear_x = 0.8
kp_linear_y = 0.6  # Y方向可能需要不同的增益
```

### 3. 添加速度限制
```python
max_linear_vel_x = 0.2
max_linear_vel_y = 0.15  # Y方向可能需要更小的限制
```

### 4. 添加死区
```python
if abs(error_x_robot) < 0.01:  # 1cm死区
    cmd.linear.x = 0
if abs(error_y_robot) < 0.01:
    cmd.linear.y = 0
```
