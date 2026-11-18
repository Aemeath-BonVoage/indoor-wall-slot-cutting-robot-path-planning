# -*- coding: utf-8 -*-
"""gazebo_io 的服务名（Gazebo Classic 的 gazebo_ros_factory）."""
import pytest

gazebo_io = pytest.importorskip('scr_viz.gazebo_io')


def test_factory_service_names():
    # gazebo_ros_factory 在根命名空间提供服务，没有 /gazebo 前缀（运行中的 gzserver 实测）
    assert gazebo_io.SPAWN_SERVICE == '/spawn_entity'
    assert gazebo_io.DELETE_SERVICE == '/delete_entity'
