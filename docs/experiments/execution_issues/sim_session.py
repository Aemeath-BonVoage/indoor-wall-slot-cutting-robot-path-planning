"""执行层排查实验共用的仿真启动与清理（无界面 Gazebo + Nav2 + 任务层）.

与 plan_validation/run_validation.py 相同的隔离方式：ROS_DOMAIN_ID = 49、
GAZEBO_MASTER_URI = http://localhost:11353；结束时按进程组结束本次启动的全部进程，
再结束本域遗留进程并执行 fastdds shm clean。运行前先 source 工作空间。
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'plan_validation'))
import run_validation as rv  # noqa: E402

rv.DOMAIN = '49'
rv.MASTER = 'http://localhost:11353'

# Nav2（use_composition:=True 时在容器 nav2_container 中）应加载的组件
NAV2_NODES = ('map_server', 'amcl', 'lifecycle_manager_localization', 'controller_server',
              'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'waypoint_follower', 'velocity_smoother', 'lifecycle_manager_navigation')
READY_LINE = 'Task executor active'


def env():
    e = rv.isolated_env()
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'GAZEBO_MASTER_URI')})
    return e


def kill_stray_gazebo():
    """上一次运行遗留的 gzserver（任何域）会占住 CPU 与端口，先结束."""
    subprocess.run(['pkill', '-9', '-f', 'gzserver'], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gzclient'], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def launch_cmd(extra=()):
    return ['ros2', 'launch', 'scr_bringup', 'sim_nav.launch.py', 'gui:=false',
            'rviz:=false'] + list(extra)


class Session:
    """一次仿真启动；with 语句结束时清理全部进程."""

    def __init__(self, out, extra=(), log_name='launch.log'):
        self.out = out
        self.extra = list(extra)
        self.log_path = os.path.join(out, log_name)
        self.env = env()
        self.proc = None
        self.t0 = None

    def __enter__(self):
        kill_stray_gazebo()
        for pid in rv.leftover(self.env):
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        self.log = open(self.log_path, 'w')
        self.t0 = time.time()
        self.proc = subprocess.Popen(launch_cmd(self.extra), env=self.env, stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        return self

    def log_text(self):
        with open(self.log_path, errors='replace') as f:
            return f.read()

    def wait_ready(self, timeout):
        """等任务层激活（行为树执行器打印 READY_LINE）；返回 (是否就绪, 墙钟秒)."""
        while time.time() - self.t0 < timeout:
            text = self.log_text()
            if READY_LINE in text:
                return True, time.time() - self.t0
            if 'Dependencies did not come up' in text or self.proc.poll() is not None:
                return False, time.time() - self.t0
            time.sleep(1.0)
        return False, time.time() - self.t0

    def __exit__(self, *exc):
        rv.kill_group(self.proc, 'launch')
        self.log.close()
        time.sleep(2.0)
        for pid in rv.leftover(self.env):
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        kill_stray_gazebo()
        subprocess.run('fastdds shm clean', shell=True, env=self.env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return False


def launch_diagnosis(text):
    """从启动日志中提取 Nav2 组件加载与进程退出的情况."""
    loaded = re.findall(r"Loaded node '/?([\w/]+)' in container", text)
    died = re.findall(r'\[ERROR\] \[([\w\-.]+)\]: process has died[^\n]*', text)
    return {
        'loaded': [n.split('/')[-1] for n in loaded],
        'missing': [n for n in NAV2_NODES if n not in [x.split('/')[-1] for x in loaded]],
        'died': died,
    }
