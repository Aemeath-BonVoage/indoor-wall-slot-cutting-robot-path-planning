# -*- coding: utf-8 -*-
"""Gazebo Classic 运行时生成 / 删除实体（gazebo_ros_factory 的 /spawn_entity、/delete_entity）."""
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

# gazebo_ros_factory 在 gzserver 的根命名空间下提供这两个服务（不带 /gazebo 前缀，
# 与 gazebo_ros 的 spawn_entity.py 相同）
SPAWN_SERVICE = '/spawn_entity'
DELETE_SERVICE = '/delete_entity'


def _call(node, srv_type, name, req, timeout):
    cli = node.create_client(srv_type, name)
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'服务 {name} 不可用（gzserver 是否加载了 libgazebo_ros_factory.so？）')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError(f'调用 {name} 超时')
        return fut.result()
    finally:
        node.destroy_client(cli)


def spawn(node, name, xml, timeout=10.0, reference_frame='world'):
    """生成实体；位姿写在 SDF 的 <pose> 里。返回 (success, status_message)."""
    req = SpawnEntity.Request(name=name, xml=xml, reference_frame=reference_frame)
    res = _call(node, SpawnEntity, SPAWN_SERVICE, req, timeout)
    return bool(res.success), res.status_message


def delete(node, name, timeout=10.0):
    """删除实体。返回 (success, status_message)."""
    res = _call(node, DeleteEntity, DELETE_SERVICE, DeleteEntity.Request(name=name),
                timeout)
    return bool(res.success), res.status_message
