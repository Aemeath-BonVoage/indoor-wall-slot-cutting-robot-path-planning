#!/usr/bin/env python3
"""
测试各关节运动方向 - 通过改变关节值观察末端位置变化
"""

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """将RPY角度转换为旋转矩阵"""
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def create_transform_matrix(xyz, rpy):
    """创建4x4齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T


def forward_kinematics(j1, j2, j3, j4):
    """
    计算正向运动学
    j1, j2, j3: 移动关节值 (米)
    j4: 旋转关节值 (弧度)
    返回末端位置 [x, y, z]
    """
    # joint1: base_link -> link1 (Z轴移动)
    # 先应用关节移动，再应用固定变换
    joint1_trans = np.eye(4)
    joint1_trans[2, 3] = j1  # Z轴移动
    T1_fixed = create_transform_matrix([0.038668, -0.0001, 0.048], [0, 0, 0])
    T1 = T1_fixed @ joint1_trans

    # joint2: link1 -> link2 (局部Z轴移动，但坐标系旋转了90度)
    joint2_trans = np.eye(4)
    joint2_trans[2, 3] = j2  # 在局部坐标系Z轴移动
    T2_fixed = create_transform_matrix([0.0665, -0.13661, 0.45879], [1.5708, 0, 0])
    T2_local = T2_fixed @ joint2_trans
    T2 = T1 @ T2_local

    # joint3: link2 -> link3 (局部-Z轴移动)
    joint3_trans = np.eye(4)
    joint3_trans[2, 3] = -j3  # 在局部坐标系-Z轴移动（注意axis是-Z）
    T3_fixed = create_transform_matrix([0.3059, 0, -0.13656], [0, 1.5708, 0])
    T3_local = T3_fixed @ joint3_trans
    T3 = T2 @ T3_local

    # joint4: link3 -> link4 (旋转)
    joint4_rot = np.eye(4)
    Rz = np.array([
        [np.cos(-j4), -np.sin(-j4), 0],
        [np.sin(-j4), np.cos(-j4), 0],
        [0, 0, 1]
    ])
    joint4_rot[:3, :3] = Rz
    T4_fixed = create_transform_matrix([0, 0, 0.1257], [0, 0, 0])
    T4_local = T4_fixed @ joint4_rot
    T4 = T3 @ T4_local

    return T4[:3, 3]


def test_joint(joint_name, joint_idx, delta=0.05):
    """测试单个关节的运动方向"""
    print(f"\n{'='*60}")
    print(f"测试 {joint_name}")
    print(f"{'='*60}")

    # 初始位置
    joints = [0.0, 0.0, 0.0, 0.0]
    pos0 = forward_kinematics(*joints)
    print(f"初始位置: X={pos0[0]:.5f}, Y={pos0[1]:.5f}, Z={pos0[2]:.5f}")

    # 正向移动
    joints[joint_idx] = delta
    pos_plus = forward_kinematics(*joints)
    print(f"关节+{delta}: X={pos_plus[0]:.5f}, Y={pos_plus[1]:.5f}, Z={pos_plus[2]:.5f}")

    # 负向移动
    joints[joint_idx] = -delta
    pos_minus = forward_kinematics(*joints)
    print(f"关节-{delta}: X={pos_minus[0]:.5f}, Y={pos_minus[1]:.5f}, Z={pos_minus[2]:.5f}")

    # 计算变化量
    delta_plus = pos_plus - pos0
    delta_minus = pos_minus - pos0

    print("\n正向移动时末端变化:")
    print(f"  ΔX = {delta_plus[0]:+.5f} m")
    print(f"  ΔY = {delta_plus[1]:+.5f} m")
    print(f"  ΔZ = {delta_plus[2]:+.5f} m")

    print("\n负向移动时末端变化:")
    print(f"  ΔX = {delta_minus[0]:+.5f} m")
    print(f"  ΔY = {delta_minus[1]:+.5f} m")
    print(f"  ΔZ = {delta_minus[2]:+.5f} m")

    # 判断主要运动方向
    abs_delta = np.abs(delta_plus)
    max_idx = np.argmax(abs_delta)
    axes = ['X', 'Y', 'Z']
    direction = '+' if delta_plus[max_idx] > 0 else '-'

    print(f"\n结论: {joint_name} 主要影响 {direction}{axes[max_idx]} 轴")
    print(f"  关节值增加 → 末端 {axes[max_idx]} {direction}{abs_delta[max_idx]:.5f}m")


def main():
    print("="*60)
    print("机械臂关节运动方向测试")
    print("="*60)
    print("\n通过改变各关节值，观察末端位置变化来确定运动方向")

    test_joint("joint1 (升降)", 0, delta=0.1)
    test_joint("joint2 (横移?)", 1, delta=0.05)
    test_joint("joint3 (伸缩?)", 2, delta=0.03)

    print("\n" + "="*60)
    print("总结")
    print("="*60)
    print("\n根据上述测试结果，可以确定各关节在base_link坐标系中的实际运动方向。")
    print("注意：base_link坐标系中，X向前，Y向左，Z向上（ROS标准）")


if __name__ == '__main__':
    main()
