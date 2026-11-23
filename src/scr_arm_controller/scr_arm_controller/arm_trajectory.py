"""
机械臂切割轨迹的时间参数化（纯 Python，不依赖 ROS）.

一个停靠站的机械臂点序列（退刀 → 下刀 → 沿线切 → 退刀 → 转刀 → …）在关节空间里是一条
折线：机械臂是直角坐标解耦结构，每一段只动一个轴（站内换位时 y、z、刀片角可以同时动）。
逐点执行时每段固定 2 s，与距离无关。这里把整条折线参数化成一条轨迹：

- 每段是“静止 → 静止”的直线运动：拐点处必须精确停住（下刀到位后才能进给，刀片在槽内
  不能转向，退刀到位后才能转刀），所以相邻两段在拐点的速度为零，不做拐角过渡；下刀
  到位后可以再等待一小段时间（dwell）让底盘的晃动衰减，然后才开始进给；
- 每段的速度曲线是加加速度受限的七段 S 曲线（jerk = 0 时退化为梯形）；
- 段的类型决定限值：切割段（cut）与下刀段（plunge）按刀尖路径速度取工艺进给，退刀
  （retract）与空行程（rapid）取各轴的速度 / 加速度限值；多轴同动时按最慢的轴同步。

输出是 S 曲线各阶段边界上的 (位置, 速度, 加速度) 结点。每个阶段内加加速度为常数，位置是
时间的三次多项式；joint_trajectory_controller 在给出位置、速度、加速度的相邻两点之间做
五次样条插值，而与两端的位置、速度、加速度都吻合的五次多项式是唯一的，所以控制器
内部按 1 kHz 采样得到的指令与这里的 S 曲线完全一致，不需要把轨迹加密成几百个点。
梯形曲线同理：只给位置和速度，控制器做三次样条，在每个匀加速阶段内同样是精确的。
"""
import math
from dataclasses import dataclass, field

JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
LINEAR = (0, 1, 2)          # joint1 = Z、joint2 = Y、joint3 = X（米）；joint4 为刀片转角（弧度）
LATERAL = 1                 # joint2：沿墙轴
EPS = 1e-9
MIN_PHASE = 1e-6            # 短于该时长的阶段并入相邻结点（控制器要求结点时间严格递增）


@dataclass(frozen=True)
class AxisLimit:
    """一个轴（或一条路径）的速度 / 加速度 / 加加速度上限；j = 0 表示不限加加速度（梯形）."""

    v: float
    a: float
    j: float = 0.0

    def scaled(self, k):
        return AxisLimit(self.v * k, self.a * k, self.j * k)


@dataclass(frozen=True)
class MotionLimits:
    """
    轨迹限值.

    feed / plunge 是工艺参数：刀尖沿槽线进给与下刀切入的路径速度（米 / 秒），与机械臂能跑
    多快无关。rapid_* 是刀具不接触墙体时各轴的限值；沿墙轴（joint2）的加速度单独给出
    （rapid_accel_lateral，None 表示与其他直线轴相同；横切时取它与 feed_accel 中较小者）：
    它的反作用力相对底盘中心有约 0.4 m 的力臂，加减速时推动底盘偏航，是机械臂运动对
    底盘的主要扰动。jerk_time 是加速度
    从 0 升到上限的时间（加加速度 = 加速度 / jerk_time），0 表示梯形速度曲线。dwell 是下刀
    到位后、开始进给前的等待时间。feed_ramp_time 是切割 / 下刀段的速度从 0 升到进给值的
    最短时间（0 = 只受 feed_accel 限制）：切削力随进给速度增大，进给缓慢建立时切削力也
    缓慢建立，定点保持才跟得上（见 docs/experiments/cutting_force）。
    """

    feed: float = 0.05
    plunge: float = 0.05
    feed_accel: float = 0.5
    rapid_speed: float = 0.25
    rapid_accel: float = 1.0
    rapid_accel_lateral: float = None
    rot_speed: float = 2.5
    rot_accel: float = 12.0
    jerk_time: float = 0.05
    dwell: float = 0.2          # 下刀到位后、开始进给前的等待（秒）
    feed_ramp_time: float = 0.0     # 进给 / 下刀速度从 0 升到设定值的最短时间（秒）

    def __post_init__(self):
        for k in ('feed', 'plunge', 'feed_accel', 'rapid_speed', 'rapid_accel', 'rot_speed',
                  'rot_accel'):
            if not getattr(self, k) > 0.0:
                raise ValueError(f'{k} must be > 0')
        if self.rapid_accel_lateral is not None and not self.rapid_accel_lateral > 0.0:
            raise ValueError('rapid_accel_lateral must be > 0')
        if self.jerk_time < 0.0 or self.dwell < 0.0 or self.feed_ramp_time < 0.0:
            raise ValueError('jerk_time, dwell and feed_ramp_time must be >= 0')

    def _jerk(self, a):
        return a / self.jerk_time if self.jerk_time > 0.0 else 0.0

    def axis(self, i):
        """关节 i 的空行程限值."""
        if i == LATERAL and self.rapid_accel_lateral is not None:
            a = self.rapid_accel_lateral
            return AxisLimit(self.rapid_speed, a, self._jerk(a))
        if i in LINEAR:
            return AxisLimit(self.rapid_speed, self.rapid_accel, self._jerk(self.rapid_accel))
        return AxisLimit(self.rot_speed, self.rot_accel, self._jerk(self.rot_accel))

    def path(self, kind):
        """切割 / 下刀段的刀尖路径限值."""
        v = self.feed if kind == 'cut' else self.plunge
        a = self.feed_accel
        if self.feed_ramp_time > 0.0:
            a = min(a, v / self.feed_ramp_time)
        return AxisLimit(v, a, self._jerk(a))


# 切割时的机械臂运动参数：技能服务器的 ROS 参数 arm_motion.*（task_skills.py），每次执行
# CutStation 时读取，可在运行中修改。取值依据与对比见 docs/experiments/arm_trajectory/README.md
ARM_MOTION_PARAMS = {
    # 'continuous'：整站机械臂点作为一条连续轨迹，按下列限值做时间参数化；
    # 'point_to_point'：逐点执行，每点固定 motion.duration 秒（对比基线）
    'mode': 'continuous',
    'feed_rate': MotionLimits.feed,             # m/s，切割进给（工艺参数：刀尖沿槽线的速度）
    'plunge_rate': MotionLimits.plunge,         # m/s，下刀切入速度（工艺参数）
    'feed_accel': MotionLimits.feed_accel,      # m/s²，切割 / 下刀段的加速度
    'rapid_speed': MotionLimits.rapid_speed,    # m/s，退刀与空行程时直线轴的速度上限
    'rapid_accel': MotionLimits.rapid_accel,    # m/s²
    # m/s²，沿墙轴（joint2）的加速度上限；0 = 与 rapid_accel 相同
    'rapid_accel_lateral': 0.0,
    'rot_speed': MotionLimits.rot_speed,        # rad/s，刀片转向
    'rot_accel': MotionLimits.rot_accel,        # rad/s²
    'jerk_time': MotionLimits.jerk_time,        # s，加速度升到上限的时间；0 = 梯形速度曲线
    'dwell': MotionLimits.dwell,                # s，下刀到位后、开始进给前的等待
    # s，进给 / 下刀速度从 0 升到设定值的最短时间；0 = 只受 feed_accel 限制
    'feed_ramp_time': MotionLimits.feed_ramp_time,
    # 下刀段在刀尖接触墙面处先停一下，切入从静止开始（配合 feed_ramp_time 使切削力缓慢建立）
    'split_plunge': False,
    # 进给的依据：'fixed' = 直接取 feed_rate / plunge_rate（空切的默认做法）；
    # 'force_limit' = 取切割力模型给出的上限（cutting_force.feed_limit：主轴功率、轮地摩擦与
    # 定点保持的力矩裕量），feed_rate / plunge_rate 仍作为上限
    'feed_basis': 'fixed',
    'material': 'brick',                        # 墙体材料（cutting_force.MATERIALS 的键）
    'slot_depth': 0.035,                        # m，槽深（切深）
    'power_fraction': 0.67,                     # 进给上限所用的主轴功率占输出功率的比例
    'friction_mu': 0.3,                         # 进给上限所用的轮地摩擦系数（有粉尘的地面）
    'friction_margin': 1.5,                    # 所需摩擦系数的安全系数
    'start_lead': 0.15,                         # s，轨迹起始时刻相对最新控制器状态的提前量
}


def limits_from_params(params):
    """arm_motion.* 参数字典 → MotionLimits."""
    return MotionLimits(
        feed=float(params['feed_rate']), plunge=float(params['plunge_rate']),
        feed_accel=float(params['feed_accel']), rapid_speed=float(params['rapid_speed']),
        rapid_accel=float(params['rapid_accel']),
        rapid_accel_lateral=float(params.get('rapid_accel_lateral') or 0.0) or None,
        rot_speed=float(params['rot_speed']),
        rot_accel=float(params['rot_accel']), jerk_time=float(params['jerk_time']),
        dwell=float(params['dwell']),
        feed_ramp_time=float(params.get('feed_ramp_time') or 0.0))


# ---------------------------------------------------------------------------
# 一维“静止 → 静止”速度曲线
# ---------------------------------------------------------------------------
def rest_to_rest(length, lim: AxisLimit):
    """
    长度 length（> 0）的静止 → 静止运动在限值 lim 下的最短时间曲线.

    返回阶段列表 [(时长, 阶段起点加速度, 加加速度)]。S 曲线为七段（加加速 / 匀加速 /
    减加速 / 匀速 / 加减速 / 匀减速 / 减减速），行程不够时依次省去匀速段、匀加速段；
    梯形为三段。时长为 0 的阶段不输出。
    """
    L, v, a, j = float(length), lim.v, lim.a, lim.j
    if L <= 0.0:
        return []
    if j <= 0.0:
        if L >= v * v / a:
            ta, tv = v / a, L / v - v / a
        else:
            ta, tv = math.sqrt(L / a), 0.0
        raw = [(ta, a, 0.0), (tv, 0.0, 0.0), (ta, -a, 0.0)]
        return [p for p in raw if p[0] > 0.0]
    # 加速到 v 所需的时间：能否达到加速度上限
    if v * j >= a * a:
        tj, ta = a / j, a / j + v / a
    else:
        tj = math.sqrt(v / j)
        ta = 2.0 * tj
    if L >= v * ta:
        tv = L / v - ta
    else:
        # 达不到 v：峰值速度 vp 满足 vp·ta(vp) = L
        tv = 0.0
        tj = a / j
        vp = 0.5 * a * (-tj + math.sqrt(tj * tj + 4.0 * L / a))
        if vp >= a * tj:
            ta = tj + vp / a
        else:
            tj = (L / (2.0 * j)) ** (1.0 / 3.0)
            ta = 2.0 * tj
    tc = max(ta - 2.0 * tj, 0.0)
    ap = j * tj
    raw = [(tj, 0.0, j), (tc, ap, 0.0), (tj, ap, -j), (tv, 0.0, 0.0),
           (tj, 0.0, -j), (tc, -ap, 0.0), (tj, -ap, j)]
    return [p for p in raw if p[0] > 0.0]


def rest_to_rest_time(length, lim: AxisLimit):
    """rest_to_rest 的总时长（秒）."""
    return sum(p[0] for p in rest_to_rest(length, lim))


def _integrate(phases):
    """阶段列表 → 各阶段起点状态 [(t, s, v, a_start, jerk)] 与终点 (t, s, v)."""
    out, t, s, v = [], 0.0, 0.0, 0.0
    for dt, a0, j in phases:
        out.append((t, s, v, a0, j))
        s += v * dt + a0 * dt * dt / 2.0 + j * dt ** 3 / 6.0
        v += a0 * dt + j * dt * dt / 2.0
        t += dt
    return out, (t, s, v)


@dataclass
class Segment:
    """折线的一段：q0 → q1 的直线运动，标量路径参数 s ∈ [0, 1] 按 phases 变化."""

    index: int                  # 目标点在点序列中的序号
    kind: str                   # cut / plunge / retract / rapid
    q0: tuple
    q1: tuple
    t0: float
    phases: list = field(default_factory=list)
    dwell: float = 0.0

    def __post_init__(self):
        self._starts, (self.motion_time, _, _) = _integrate(self.phases)

    @property
    def t1(self):
        return self.t0 + self.motion_time + self.dwell

    @property
    def path_length(self):
        """刀尖路径长度（米）：三个直线轴的欧氏距离."""
        return math.sqrt(sum((self.q1[i] - self.q0[i]) ** 2 for i in LINEAR))

    def scalar(self, tau):
        """段内时刻 tau 的 (s, ds/dt, d²s/dt²)."""
        if not self.phases or tau <= 0.0:
            return 0.0 if self.phases else 1.0, 0.0, 0.0
        if tau >= self.motion_time:
            return 1.0, 0.0, 0.0
        k = len(self._starts) - 1
        while self._starts[k][0] > tau:
            k -= 1
        t, s, v, a0, j = self._starts[k]
        d = tau - t
        return (s + v * d + a0 * d * d / 2.0 + j * d ** 3 / 6.0,
                v + a0 * d + j * d * d / 2.0, a0 + j * d)

    def state(self, tau):
        """段内时刻 tau 的关节 (位置, 速度, 加速度)."""
        s, sd, sdd = self.scalar(tau)
        dq = [b - a for a, b in zip(self.q0, self.q1)]
        return ([a + d * s for a, d in zip(self.q0, dq)], [d * sd for d in dq],
                [d * sdd for d in dq])


class ArmTrajectory:
    """一个停靠站的整条轨迹：若干首尾相接的段."""

    def __init__(self, segments, with_accel):
        self.segments = segments
        self.with_accel = with_accel        # 结点是否带加速度（S 曲线：是；梯形：否）

    @property
    def duration(self):
        return self.segments[-1].t1 if self.segments else 0.0

    def sample(self, t):
        """时刻 t（秒，自轨迹起点）的关节 (位置, 速度, 加速度)."""
        seg = self.segments[0]
        for s in self.segments:
            if s.t0 <= t:
                seg = s
        return seg.state(t - seg.t0)

    def knots(self):
        """
        发给控制器的结点 [(t, 位置, 速度, 加速度)]，时间严格递增.

        每段取各阶段的边界；相邻段共用拐点处的结点（速度、加速度为零）。梯形曲线的加速度在
        结点处不连续，结点不带加速度（返回 None），由控制器做三次样条。
        """
        out = [(0.0, list(self.segments[0].q0), [0.0] * 4, [0.0] * 4)] if self.segments else []
        for seg in self.segments:
            times = [st[0] for st in seg._starts[1:]] + [seg.motion_time]
            for tau in times:
                if seg.t0 + tau - out[-1][0] < MIN_PHASE:
                    continue
                q, qd, qdd = seg.state(tau)
                out.append((seg.t0 + tau, q, qd, qdd))
            if seg.phases:
                # 段末：位置取目标点本身，速度、加速度严格为零（消除积分的舍入误差）
                out[-1] = (out[-1][0], list(seg.q1), [0.0] * 4, [0.0] * 4)
            if seg.dwell > MIN_PHASE:
                out.append((seg.t1, list(seg.q1), [0.0] * 4, [0.0] * 4))
        if not self.with_accel:
            out = [(t, q, qd, None) for t, q, qd, _ in out]
        return out

    def summary(self):
        """按段类型汇总：{类型: (段数, 时间, 路径长度)}."""
        out = {}
        for s in self.segments:
            n, t, d = out.get(s.kind, (0, 0.0, 0.0))
            out[s.kind] = (n + 1, t + s.t1 - s.t0, d + s.path_length)
        return out


def segment_kinds(is_cut, start_at_depth=False):
    """由“是否处于切深”标注每段的类型：cut / plunge / retract / rapid."""
    out, prev = [], bool(start_at_depth)
    for cut in is_cut:
        cut = bool(cut)
        out.append('cut' if (prev and cut) else 'plunge' if cut else
                   'retract' if prev else 'rapid')
        prev = cut
    return out


def path_limit(kind, q0, q1, lim: MotionLimits):
    """
    一段直线运动的标量路径参数 s ∈ [0, 1] 的限值；q0 == q1 时返回 None.

    各轴限值 v_i 折算到 s：ds/dt ≤ v_i / |Δq_i|，取所有运动轴中最紧的一个，因此多轴同动时
    各轴同时起停、都不超限。切割 / 下刀段再加上刀尖路径速度（三个直线轴的合成）的限值。
    """
    dq = [abs(b - a) for a, b in zip(q0, q1)]
    cands = [lim.axis(i).scaled(1.0 / d) for i, d in enumerate(dq) if d > EPS]
    if not cands:
        return None
    if kind in ('cut', 'plunge'):
        length = math.sqrt(sum(dq[i] ** 2 for i in LINEAR))
        if length > EPS:
            cands.append(lim.path(kind).scaled(1.0 / length))
    jerks = [c.j for c in cands]
    return AxisLimit(min(c.v for c in cands), min(c.a for c in cands),
                     0.0 if min(jerks) <= 0.0 else min(jerks))


def plan_trajectory(start, waypoints, is_cut, lim: MotionLimits = None, start_at_depth=False):
    """
    把一个停靠站的关节点序列参数化成一条轨迹.

    start：当前关节位置 (j1, j2, j3, j4)；waypoints：依次到达的关节点；is_cut：各点是否处于
    切深（与 slot_execution.station_arm_points 的输出一致）。与上一点重合的点不产生运动，
    只保留一个零时长的段，段序号仍与点序号一一对应。
    """
    lim = lim or MotionLimits()
    if len(waypoints) != len(is_cut):
        raise ValueError('waypoints and is_cut differ in length')
    segs, t, q = [], 0.0, tuple(float(v) for v in start)
    for k, (wp, kind) in enumerate(zip(waypoints, segment_kinds(is_cut, start_at_depth))):
        wp = tuple(float(v) for v in wp)
        pl = path_limit(kind, q, wp, lim)
        phases = rest_to_rest(1.0, pl) if pl is not None else []
        seg = Segment(k, kind, q, wp, t, phases, lim.dwell if kind == 'plunge' else 0.0)
        segs.append(seg)
        t, q = seg.t1, wp
    return ArmTrajectory(segs, with_accel=lim.jerk_time > 0.0)


def segment_time(kind, q0, q1, lim: MotionLimits):
    """一段运动的时长（秒），不含下刀后的 dwell."""
    pl = path_limit(kind, q0, q1, lim)
    return rest_to_rest_time(1.0, pl) if pl is not None else 0.0
