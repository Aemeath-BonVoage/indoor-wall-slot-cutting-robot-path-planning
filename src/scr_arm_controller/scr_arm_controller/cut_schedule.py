"""
一个停靠站的切削状态时间表：由机械臂轨迹给出每个时刻的切深、进给与预计的切削力（不依赖 ROS）.

整站连续轨迹（arm_trajectory.ArmTrajectory）在开始执行时各段的起止时刻就已确定，因此刀具
何时切入、以多大的进给沿哪个方向切、何时退出都可以提前算出。由此得到两样东西：

- 切深 a(t)：发给切割力插件（话题 cutting_force/engagement）。仿真中是空切，刀尖停在墙前，
  切深不能由几何关系得到；这里按“切深处的点对应规定槽深”折算：下刀行程的最后一段槽深
  对应切深从 0 增加到规定值。空切时整个下刀行程（退刀位置到墙前间隙处，约 1 cm）短于
  槽深，这时把整个行程按比例映射到 0 – 槽深（travel_ratio < 1），下刀速度相应折算。
- 预计的底盘力旋量 (F_x, F_y, M_z)(t)：由名义的进给 / 下刀速度、切深、刀片方向和刀具位置
  经切割力模型（cutting_force.py）算出，供定点保持作前馈（hold.feedforward_topic）。它不含
  材料不均匀性，材料参数取名义值，所以只是实际切削力中可预知的部分。

另外提供接触点拆分（split_plunges）：下刀段在刀尖接触墙面处插入一个停顿点，使切入从静止
开始。不拆分时刀具以全速接触墙面，切削力是一次阶跃。
"""
import math
from dataclasses import dataclass

from scr_arm_controller import cutting_force as cf

# 机械臂零位时刀具连杆原点在 base_link 系中的位置，以及关节到笛卡尔的符号
# （与 cartesian_cutting.CartesianCuttingController.inverse_kinematics 一致）
ARM_X0, ARM_Y0, ARM_Z0 = 0.53677, -0.00015, 0.50679
TOOL_OFFSET = 0.1594        # 刀具连杆原点到刀尖（沿 base_link x）
MIN_FEED = 0.0005           # 低于该进给速度视为未进给（与切割力插件的 min_feed 相同）
BASE_HEIGHT = 0.05          # base_link 原点离地高度的近似值，只影响轮压估计，不影响水平面力旋量


def tool_position(q):
    """关节 (j1, j2, j3, j4) → 刀具连杆原点在 base_link 系中的 (x, y, z)."""
    return ARM_X0 - q[2], ARM_Y0 - q[1], ARM_Z0 + q[0]


@dataclass
class CutState:
    """某一时刻的切削状态（名义值）."""

    depth: float = 0.0          # 切深 a（米）
    feed: float = 0.0           # 沿刀片方向的进给速度（米 / 秒，带符号）
    plunge: float = 0.0         # 下刀速度（米 / 秒，朝墙为正）
    orientation: str = 'vertical'   # 刀片方向：horizontal（沿墙）/ vertical
    feed_sign: int = 1          # cutting_force.feed_axis 的约定：横槽沿墙向右为正，竖槽向上为正
    tool: tuple = (0.0, 0.0, 0.0)   # 刀具连杆原点（base_link 系）


def contact_position(x_deep, plunge_starts, slot_depth):
    """刀尖接触墙面时的伸缩轴位置：最深位置后退一个槽深，但不早于最靠前的下刀起点."""
    x = x_deep - slot_depth
    if plunge_starts:
        x = max(x, max(plunge_starts))
    return min(x, x_deep - 1e-4)


def travel_ratio(points, is_cut, slot_depth):
    """一个停靠站的点序列的 切入行程 / 槽深（≤ 1）；没有下刀段时为 1."""
    deep = [p[0] for p, c in zip(points, is_cut) if c]
    starts = [a[0] for a, b, ca, cb in zip(points[:-1], points[1:], is_cut[:-1], is_cut[1:])
              if cb and not ca]
    if not deep:
        return 1.0
    return (max(deep) - contact_position(max(deep), starts, slot_depth)) / slot_depth


def split_plunges(points, is_cut, slot_depth, margin=0.002):
    """
    在每个下刀段的接触点插入一个停顿点，返回 (新点列, 新的切深标注).

    points：[(x, y, z, rotation)]；下刀段是从非切深点到切深点的运动（只动 x）。接触点取
    最深位置后退 slot_depth + margin 处；下刀起点已经比它更靠近墙面时不插入。插入的点标为
    非切深，所以它之前的一段按空行程速度接近，之后的一段才是从静止开始的下刀。
    """
    out_p, out_c, prev = [], [], None
    for p, cut in zip(points, is_cut):
        if cut and prev is not None and not prev[1]:
            x_contact = p[0] - slot_depth - margin
            if prev[0][0] < x_contact - 1e-6:
                out_p.append((x_contact, p[1], p[2], p[3]))
                out_c.append(False)
        out_p.append(tuple(p))
        out_c.append(bool(cut))
        prev = (p, cut)
    return out_p, out_c


class CutSchedule:
    """一条整站轨迹的切削状态与预计力旋量."""

    def __init__(self, traj, blade=None, slot_depth=cf.SLOT_DEPTH, standoff=0.70,
                 platform=None):
        self.traj = traj
        self.blade = blade or cf.BladeParams()
        self.slot_depth = float(slot_depth)
        self.standoff = float(standoff)
        self.platform = platform or cf.Platform()
        # 最深位置：切割 / 下刀段终点的伸缩轴位置（x 最大 = joint3 最小）；接触位置：最深位置
        # 后退一个槽深，但不早于下刀段的起点（空切时下刀行程短于槽深）
        deep = [tool_position(s.q1)[0] for s in traj.segments if s.kind in ('cut', 'plunge')]
        self.x_deep = max(deep) if deep else None
        self.x_contact = None
        if deep:
            self.x_contact = contact_position(
                self.x_deep, [tool_position(s.q0)[0] for s in traj.segments
                              if s.kind == 'plunge'], self.slot_depth)

    @property
    def travel_ratio(self):
        """切入行程 / 槽深（≤ 1）；没有切割段时为 1."""
        if self.x_deep is None:
            return 1.0
        return (self.x_deep - self.x_contact) / self.slot_depth

    @property
    def duration(self):
        return self.traj.duration

    def state(self, t):
        """轨迹起点后 t 秒的切削状态；轨迹之外或没有切割段时切深为 0."""
        if self.x_deep is None or t < 0.0 or t > self.traj.duration:
            return CutState()
        # 空行程（站内换位、从别处回到退刀位置）不接触墙面，无论伸缩轴在什么位置
        kind = 'rapid'
        for seg in self.traj.segments:
            if seg.t0 <= t:
                kind = seg.kind
        if kind == 'rapid':
            return CutState()
        q, qd, _ = self.traj.sample(t)
        x, y, z = tool_position(q)
        scale = 1.0 / self.travel_ratio
        depth = min(self.slot_depth, max(0.0, (x - self.x_contact) * scale))
        plunge = max(-qd[2], 0.0) * scale if depth < self.slot_depth else 0.0
        theta = q[3]
        s, c = math.sin(theta), math.cos(theta)
        vy, vz = -qd[1], qd[0]
        feed = vy * s + vz * c              # 沿刀片方向 (0, sin θ, cos θ) 的速度
        horizontal = abs(s) > abs(c)
        if abs(feed) < MIN_FEED:
            # 未进给（下刀、停顿）：方向符号取默认值，即沿刀片方向 (0, sin θ, cos θ) 为正
            # （与切割力插件的约定相同）
            sign = (-1 if s >= 0.0 else 1) if horizontal else (1 if c >= 0.0 else -1)
        elif horizontal:
            sign = 1 if -vy >= 0.0 else -1
        else:
            sign = 1 if vz >= 0.0 else -1
        return CutState(depth=depth, feed=feed, plunge=plunge,
                        orientation='horizontal' if horizontal else 'vertical',
                        feed_sign=sign, tool=(x, y, z))

    def wrench(self, t, min_feed=MIN_FEED):
        """轨迹起点后 t 秒预计的底盘力旋量 cutting_force.Wrench（名义材料，无不均匀性）."""
        st = self.state(t)
        if st.depth <= 0.0:
            return cf.Wrench()
        feed = abs(st.feed) if abs(st.feed) >= min_feed else 0.0
        # 刀片中心：最深位置时在墙面内 slot_depth − R 处，随伸缩轴位置前后移动
        centre_x = self.standoff + self.slot_depth - self.blade.radius + (st.tool[0] - self.x_deep)
        pf = cf.Platform(
            mass=self.platform.mass, inertia_z=self.platform.inertia_z,
            com_x=self.platform.com_x, com_y=self.platform.com_y,
            half_length=self.platform.half_length, half_width=self.platform.half_width,
            wheel_radius=self.platform.wheel_radius,
            standoff=centre_x + self.blade.radius - st.depth,
            tool_y=st.tool[1], tool_z=st.tool[2] + BASE_HEIGHT)
        return cf.cut_wrench(self.blade, pf, st.depth, feed, st.orientation, st.feed_sign,
                             st.plunge)

    def peak(self, dt=0.02):
        """整条轨迹上预计的 (|F_x|, |F_y|, |M_z|) 最大值."""
        out = [0.0, 0.0, 0.0]
        for k in range(int(self.traj.duration / dt) + 1):
            w = self.wrench(k * dt).planar
            out = [max(a, abs(b)) for a, b in zip(out, w)]
        return tuple(out)
