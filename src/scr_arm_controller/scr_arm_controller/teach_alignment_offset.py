#!/usr/bin/env python3
"""
精对位零位标定（示教）：在真值已知的参考位姿处记录估计器的读数，求 (法向, 沿墙, 朝向) 偏置.

做法与 AGV 充电桩 / 工装定位的“示教零位”相同：把机器人放到准确的参考位姿（真机上用
工装或测量设备，仿真中用 Gazebo 传送到真值目标），静止状态下以零偏置运行估计器若干秒，
偏置 = 估计均值 − 真值（WallEKF 状态 (d, u, ψ) 的约定）。之后估计器在测量模型内部扣除该偏置，
相机外参误差、标记安装误差、激光测距偏差等静态误差在参考位姿处被一并消除。

  ros2 run scr_arm_controller teach_alignment_offset --out fine_alignment_taught.yaml [--seconds 8]

输出为 ROS 参数文件，执行任务或基准测试时用 --ros-args --params-file 加载，
或用 alignment_benchmark --calib 指定。需先启动仿真（不需要 Nav2）。
"""
import argparse
import math
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import SetEntityState

from scr_arm_controller.alignment_benchmark import (DISTANCE, TAG_X, WALL_Y, teleport,
                                                    wall_state_true)
from scr_arm_controller.navigation_cutting_task import NavigationCuttingTask

TEMPLATE = """# 精对位零位标定（teach_alignment_offset 生成，{date}）
# 参考位姿：离墙 {D:.3f} m、正对定位标记；{n} 个估计样本（{seconds:.0f} s，静止）
# 偏置 = 估计均值 − 真值；样本标准差：法向 {sd:.3f} mm、沿墙 {su:.3f} mm、朝向 {spsi:.4f}°
# 偏置的标准误（批均值法）：法向 {ed:.3f} mm、沿墙 {eu:.3f} mm、朝向 {epsi:.4f}°
# 参考位姿真值误差（传送后静置）：法向 {gd:+.3f} mm、沿墙 {gu:+.3f} mm、朝向 {gpsi:+.4f}°
navigation_cutting_task:
  ros__parameters:
    alignment:
      offset_normal: {c_d:.6f}
      offset_lateral: {c_u:.6f}
      offset_yaw_deg: {c_psi:.5f}
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', default='fine_alignment_taught.yaml')
    # 估计误差有秒级的相关时间（激光朝向单帧 σ ≈ 0.26°，EKF 随机游走 0.01°/步），8 s 的均值
    # 仍有约 0.03° 的朝向不确定度，经标记耦合成约 0.4 mm 的沿墙偏置；默认取 30 s
    ap.add_argument('--seconds', type=float, default=30.0, help='采样时长（仿真时间）')
    ap.add_argument('--settle', type=float, default=3.0, help='开始采样前的收敛时间')
    args = ap.parse_args()

    rclpy.init()
    node = NavigationCuttingTask()
    node.apply_alignment_config(offset_normal=0.0, offset_lateral=0.0, offset_yaw_deg=0.0)
    log = node.get_logger()
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    if not client.wait_for_service(timeout_sec=20.0):
        log.error('找不到 /gazebo/set_entity_state，请确认仿真已启动')
        return
    tx, ty, tyaw = TAG_X, WALL_Y + DISTANCE, -math.pi / 2
    node.stop_robot()
    node.move_arm_for_approach(joint1=node.observe_joint1)
    teleport(node, client, tx, ty, tyaw)
    node.sleep_sim(1.0)
    # 定点保持（仿真轮子摩擦为 0，零速度下底盘会缓慢滑移）；每个样本都配同一时刻的真值
    node.base_hold.enabled = True
    node.base_hold.hold()
    node.sleep_sim(1.0)

    est = node.estimator
    est.reset()
    est.use_tag = True
    a = node.align_cfg
    node.servo.detector = a['detector']
    node.servo.detectors = {a['detector']}
    period = 1.0 / float(a['control_rate'])
    samples, truth = [], []
    t0 = node.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok():
        t = node.get_clock().now().nanoseconds * 1e-9 - t0
        X, _ = est.step()
        if X is not None and est.u_known and t > args.settle:
            samples.append(X.copy())
            truth.append(wall_state_true(node.get_ground_truth_position()))
        if t > args.settle + args.seconds:
            break
        node.sleep_sim(period)
    node.base_hold.release()
    if len(samples) < 10:
        log.error(f'有效样本过少（{len(samples)}），检查定位标记是否可见')
        return
    S, T = np.array(samples), np.array(truth)
    c = S.mean(axis=0) - T.mean(axis=0)
    sd = S.std(axis=0)
    # 均值的标准误：样本自相关，按批均值法（分 6 批）估计
    E = S - T
    batches = np.array([b.mean(axis=0) for b in np.array_split(E, 6)])
    sem = batches.std(axis=0, ddof=1) / math.sqrt(len(batches))
    g = T.mean(axis=0) - np.array([DISTANCE, 0.0, 0.0])
    text = TEMPLATE.format(
        date=time.strftime('%Y-%m-%d'), D=DISTANCE, n=len(S), seconds=args.seconds,
        sd=sd[0] * 1000, su=sd[1] * 1000, spsi=math.degrees(sd[2]),
        ed=sem[0] * 1000, eu=sem[1] * 1000, epsi=math.degrees(sem[2]),
        gd=g[0] * 1000, gu=g[1] * 1000, gpsi=math.degrees(g[2]),
        c_d=c[0], c_u=c[1], c_psi=math.degrees(c[2]))
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(text)
    log.info(f'零位偏置：法向 {c[0] * 1000:+.3f} mm、沿墙 {c[1] * 1000:+.3f} mm、'
             f'朝向 {math.degrees(c[2]):+.4f}°（{len(S)} 个样本，标准误 {sem[1] * 1000:.3f} mm / '
             f'{math.degrees(sem[2]):.4f}°）→ {args.out}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
