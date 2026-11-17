# -*- coding: utf-8 -*-
"""测试公共夹具：测试数据目录、包内 config 目录."""
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'data')
CONFIG = os.path.join(HERE, '..', 'config')


@pytest.fixture
def task_file():
    return os.path.join(DATA, 'task_room3_south.yaml')


@pytest.fixture
def plan_file():
    return os.path.join(DATA, 'plan_room3_south.yaml')


@pytest.fixture
def demo_task_file():
    return os.path.join(CONFIG, 'wall_task_room3_south_demo.yaml')
