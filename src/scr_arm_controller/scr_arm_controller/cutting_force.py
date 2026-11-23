"""
切割力模型：刀片上的切削力 → 底盘受到的力旋量 → 轮地摩擦与倾覆裕度（不依赖 ROS）.

金刚石圆盘在砌体 / 混凝土上开槽。模型分三层：

1. 刀片上的切向力 F_t 与法向（径向）力 F_n，由比切削能给出::

       Q    = b·(a·v_f + c(a)·v_p)                 材料去除率（进给 + 下刀）
       h_eq = Q / (b·v_c)                          当量切屑厚度
       u    = g·u_0·(h_eq / h_0)^(−ε)              比切削能（尺寸效应）
       F_t  = u·Q / v_c,     F_n = λ·F_t

   b 为锯缝总宽，a 为切深，v_f 为进给速度，v_p 为下刀速度，c(a) = 2√(a(2R − a)) 为刀片
   在墙面上的弦长，v_c 为刀片线速度，g 为材料不均匀性的乘性增益（均值 1）。

2. 分解到进给方向 f 与墙面外法向 n（指向机器人）。进给部分的合力作用在接触弧上
   φ_r = κ·φ_e 处（φ_e = arccos(1 − a/R) 为接触弧角，从最深点向进给前方量起），下刀部分
   作用在最深点。σ = +1 为逆铣（最深点处刀齿运动方向与进给同向，切向力阻碍进给并把刀片
   拉向墙内），σ = −1 为顺铣::

       F_f = −F_n,f·sin φ_r − σ·F_t,f·cos φ_r − σ·F_t,p     沿进给方向（负 = 阻碍进给）
       F_w =  F_n,f·cos φ_r − σ·F_t,f·sin φ_r + F_n,p       沿 n（正 = 把刀具推离墙面）

   切向力对刀片中心的力矩与 φ 无关：τ = σ·F_t·R·(n × f)。因此刀具受力等效为作用在刀片
   中心的力 F_f·f + F_w·n 加一个绕刀轴的力偶 τ。

3. 换算到底盘：base_link 系 x 指向墙面、y 向左、z 向上，原点取在地面上。墙面外法向
   n = −x；横槽的进给方向 f = ∓y（沿墙向右为 +u = −y），竖槽 f = ±z。

数值的出处与可信程度（与 scr_planner 的规则常量同一做法）：
  [核对]  已核对产品资料或论文摘要
  [文献]  文献中的模型形式或量级，数值未能对照原文核对
  [推算]  由已核对的量计算得到
  [假设]  本项目的工程假设，可调
"""
import math
import random
from dataclasses import dataclass, field, replace

G = 9.81

# ---------------------------------------------------------------------------
# 刀具（参照手持开槽机的规格）
# ---------------------------------------------------------------------------
BLADE_DIAMETER = 0.150      # [核对] Bosch GNF 35 CA：刀片直径 150 mm，槽深 0–35 mm
SPINDLE_RPM = 9300.0        # [核对] 同上：空载转速 9300 r/min → 线速度 73 m/s
SPINDLE_POWER = 820.0       # [核对] 同上：输出功率 820 W（额定输入 1400 W）
KERF_WIDTH = 0.005          # [假设] 两片 2.5 mm 刀片同时切两条锯缝，槽芯另行剔除
SLOT_DEPTH = 0.035          # [国标] 与 scr_planner 的 SLOT_DEPTH 一致（管径 20 + 保护层 15 mm）
FORCE_RATIO = 4.0           # [文献] 法向力 / 切向力；石材圆锯的报道值在个位数量级，未核对原文
RESULTANT_FRACTION = 0.6    # [假设] 合力作用点在接触弧上的相对位置 κ（文献以 α = k·β 表示，
#                             且顺铣小于逆铣；切屑厚度随 φ 增大，取略大于弧中点）
SIZE_EXPONENT = 0.25        # [文献] 单位宽度切削力 ∝ h_eq^n 的幂律形式，n = 1 − ε；ε 的取值为假设
REF_FEED = 0.005            # [假设] 给出 u_0 的参考工况：进给 5 mm/s、切深 35 mm
INHOMOGENEITY_COV = 0.25    # [假设] 比切削能的变异系数（骨料、灰缝、孔洞）
INHOMOGENEITY_LENGTH = 0.015  # [假设] 相关长度，取粗骨料粒径量级

# 比切削能 u_0（J/m³，参考工况下）。[假设]：按“开槽机输出功率 820 W 时的进给速度”反推的量级，
# 分别对应约 2.8 m/min（加气块）、0.7 m/min（烧结砖）、0.19 m/min（C30 混凝土）；
# 石材湿切圆锯的文献值为 1–10 J/mm³ 量级 [文献，未核对原文]。
MATERIALS = {
    'aerated_block': 0.10e9,
    'brick': 0.40e9,
    'concrete': 1.50e9,
}


@dataclass
class BladeParams:
    """刀片与切削过程参数."""

    diameter: float = BLADE_DIAMETER
    kerf_width: float = KERF_WIDTH
    spindle_rpm: float = SPINDLE_RPM
    spindle_power: float = SPINDLE_POWER
    specific_energy: float = MATERIALS['brick']
    force_ratio: float = FORCE_RATIO
    resultant_fraction: float = RESULTANT_FRACTION
    size_exponent: float = SIZE_EXPONENT
    ref_depth: float = SLOT_DEPTH
    ref_feed: float = REF_FEED
    up_cut: bool = True       # [假设] 逆铣（手持开槽机为避免刀具自行前窜的常用转向）

    @property
    def radius(self):
        return 0.5 * self.diameter

    @property
    def cutting_speed(self):
        """刀片线速度 v_c（m/s）."""
        return math.pi * self.diameter * self.spindle_rpm / 60.0

    @property
    def ref_chip(self):
        """参考工况的当量切屑厚度 h_0（m）."""
        return self.ref_depth * self.ref_feed / self.cutting_speed

    @classmethod
    def for_material(cls, name, **kw):
        return cls(specific_energy=MATERIALS[name], **kw)


@dataclass
class ToolForces:
    """刀片受力（进给方向 f、墙面外法向 n 所在的刀片平面内）."""

    tangential: float = 0.0   # F_t（N）
    normal: float = 0.0       # F_n（N）
    feed: float = 0.0         # F_f：沿进给方向（负 = 阻碍进给）
    wall: float = 0.0         # F_w：沿墙面外法向（正 = 推离墙面）
    couple: float = 0.0       # 绕 n × f 的力偶 σ·F_t·R（N·m）
    power: float = 0.0        # 切削功率 F_t·v_c（W）
    chip: float = 0.0         # 当量切屑厚度 h_eq（m）


def engagement_angle(depth, radius):
    """接触弧角 φ_e = arccos(1 − a/R)（rad）."""
    a = min(max(depth, 0.0), radius)
    return math.acos(1.0 - a / radius)


def tool_forces(blade, depth, feed_rate, plunge_rate=0.0, gain=1.0):
    """
    刀片受力.

    depth：切深 a（m）；feed_rate：沿槽线的进给速度大小 v_f（m/s）；plunge_rate：下刀速度
    v_p（m/s，切深增加为正，退刀时取 0）；gain：材料不均匀性的乘性增益。
    """
    r = blade.radius
    a = min(max(depth, 0.0), r)
    vf, vp = abs(feed_rate), max(plunge_rate, 0.0)
    b, vc = blade.kerf_width, blade.cutting_speed
    q_feed = b * a * vf
    q_plunge = b * 2.0 * math.sqrt(a * (2.0 * r - a)) * vp
    q = q_feed + q_plunge
    if q <= 0.0:
        return ToolForces()
    h = q / (b * vc)
    u = gain * blade.specific_energy * (h / blade.ref_chip) ** (-blade.size_exponent)
    ft_feed, ft_plunge = u * q_feed / vc, u * q_plunge / vc
    lam = blade.force_ratio
    sigma = 1.0 if blade.up_cut else -1.0
    phi = blade.resultant_fraction * engagement_angle(a, r)
    s, c = math.sin(phi), math.cos(phi)
    ft = ft_feed + ft_plunge
    return ToolForces(
        tangential=ft, normal=lam * ft,
        feed=-lam * ft_feed * s - sigma * ft_feed * c - sigma * ft_plunge,
        wall=lam * ft_feed * c - sigma * ft_feed * s + lam * ft_plunge,
        couple=sigma * ft * r, power=ft * vc, chip=h)


def max_feed_rate(blade, depth, power=None):
    """主轴功率允许的最大进给速度（m/s）：解 F_t(v_f)·v_c = P."""
    p = blade.spindle_power if power is None else power
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if tool_forces(blade, depth, mid).power > p:
            hi = mid
        else:
            lo = mid
    return lo


# ---------------------------------------------------------------------------
# 底盘：几何、质量与力旋量
# ---------------------------------------------------------------------------
@dataclass
class Platform:
    """底盘与刀具的几何 / 质量参数（base_link 系：x 指向墙面，y 向左，z 自地面向上）."""

    mass: float = 30.5              # [推算] URDF 各连杆质量之和（kg）
    inertia_z: float = 0.92         # [推算] 仿真插件加载时计算的绕竖直轴转动惯量（kg·m²）
    com_x: float = 0.010            # [推算] URDF 质心在 base_link 原点前方约 1 cm
    com_y: float = 0.0
    half_length: float = 0.29658    # 前后轮距之半 l_x（scr_base_control 的 wheel_separation_x / 2）
    half_width: float = 0.2469      # 左右轮距之半 l_y
    wheel_radius: float = 0.05
    standoff: float = 0.70          # base_link 原点到墙面的距离（精对位目标）
    tool_y: float = 0.0             # 刀片中心的横向位置（−0.06 … 0.08 m）
    tool_z: float = 0.55            # 刀片中心离地高度（0.26 … 0.81 m）

    def blade_centre(self, depth, radius):
        """刀片中心在 base_link 系中的位置（切深 a 时最深点在墙面内 a 处）."""
        return (self.standoff + depth - radius, self.tool_y, self.tool_z)

    def tool_tip(self, depth):
        """刀尖（刀片最深点）在 base_link 系水平面内的位置 (x, y)."""
        return (self.standoff + depth, self.tool_y)


@dataclass
class Wrench:
    """作用在底盘上的力旋量，对 base_link 原点在地面上的投影取矩."""

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0

    @property
    def planar(self):
        """水平面内的 (F_x, F_y, M_z)."""
        return (self.fx, self.fy, self.mz)

    def scaled(self, k):
        return Wrench(*(k * v for v in (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def feed_axis(orientation, feed_sign):
    """进给方向单位向量 f（base_link 系）：横槽沿墙向右为正（−y），竖槽向上为正（+z）."""
    s = 1.0 if feed_sign >= 0 else -1.0
    if orientation == 'horizontal':
        return (0.0, -s, 0.0)
    if orientation == 'vertical':
        return (0.0, 0.0, s)
    raise ValueError(f'unknown orientation: {orientation}')


def base_wrench(forces, platform, blade, depth, orientation, feed_sign=1):
    """刀片受力 → 底盘力旋量（刀具经机械臂刚性连接到底盘）."""
    f = feed_axis(orientation, feed_sign)
    n = (-1.0, 0.0, 0.0)
    force = tuple(forces.feed * fi + forces.wall * ni for fi, ni in zip(f, n))
    axis = _cross(n, f)
    couple = tuple(forces.couple * ai for ai in axis)
    m = _cross(platform.blade_centre(depth, blade.radius), force)
    return Wrench(force[0], force[1], force[2],
                  m[0] + couple[0], m[1] + couple[1], m[2] + couple[2])


def cut_wrench(blade, platform, depth, feed_rate, orientation, feed_sign=1,
               plunge_rate=0.0, gain=1.0):
    """给定切削状态下的底盘力旋量（tool_forces + base_wrench）."""
    return base_wrench(tool_forces(blade, depth, feed_rate, plunge_rate, gain),
                       platform, blade, depth, orientation, feed_sign)


# ---------------------------------------------------------------------------
# 静力学裕度：轮压、所需摩擦系数、倾覆
# ---------------------------------------------------------------------------
# 轮序 [lf, rf, lb, rb]；辊子轴线方向与 scr_base_control 的接触模型一致
_H = math.sqrt(0.5)
_ROLLER = ((_H, -_H), (_H, _H), (_H, _H), (_H, -_H))
_SIGN = ((1, 1), (1, -1), (-1, 1), (-1, -1))       # (前后, 左右)


def wheel_loads(wrench, platform):
    """
    四个轮的法向压力 [lf, rf, lb, rb]（N）.

    四点支承静不定，取刚性车架、对称分配（俯仰力矩由前后轮对分担，侧倾力矩由左右轮对
    分担，无对角扭曲分量）。负值表示该轮已离地（开始倾覆）。
    """
    w = platform.mass * G
    lx, ly = platform.half_length, platform.half_width
    n0 = (w - wrench.fz) / 4.0
    a = (platform.com_x * w + wrench.my) / (4.0 * lx)       # 前轮增加量
    b = (platform.com_y * w - wrench.mx) / (4.0 * ly)       # 左轮增加量
    return [n0 + sx * a + sy * b for sx, sy in _SIGN]


def roller_forces(wrench, platform, loads=None):
    """
    平衡水平力旋量所需的辊子轴向摩擦力 [lf, rf, lb, rb] 与所需摩擦系数.

    麦克纳姆轮只能沿辊子轴线传力（垂直轴线方向辊子自由滚动）。三个平衡方程、四个未知力，
    剩余的一个自由度（内力模态 (1, 1, −1, −1)）用来使 max |f_i| / N_i 最小。
    返回 (f, mu_required)；有轮离地时 mu_required = inf。
    """
    n = wheel_loads(wrench, platform) if loads is None else loads
    k = platform.half_length + platform.half_width
    fx, fy, mz = -wrench.fx, -wrench.fy, -wrench.mz       # 地面需要提供的合力
    # 最小范数解：f = Aᵀ(AAᵀ)⁻¹·b，A 的三行相互正交（模方均为 2、2、2k²）
    base = [(fx * _H + fy * ry) / 2.0 + mz * t / (2.0 * k * k)
            for (_, ry), t in zip(_ROLLER, (-_H * k, _H * k, -_H * k, _H * k))]
    null = (1.0, 1.0, -1.0, -1.0)
    if min(n) <= 0.0:
        return base, math.inf

    def worst(t):
        return max(abs(b0 + t * z) / ni for b0, z, ni in zip(base, null, n))

    # 分段线性凸函数的一维最小化：候选点为任意两条 |·|/N 直线的交点
    cands = [0.0]
    for i in range(4):
        cands.append(-base[i] / null[i])
        for j in range(i + 1, 4):
            for sgn in (1.0, -1.0):
                den = null[i] / n[i] - sgn * null[j] / n[j]
                if abs(den) > 1e-12:
                    cands.append((sgn * base[j] / n[j] - base[i] / n[i]) / den)
    t = min(cands, key=worst)
    return [b0 + t * z for b0, z in zip(base, null)], worst(t)


@dataclass
class StaticMargins:
    """静力学裕度."""

    loads: list = field(default_factory=list)   # 四个轮的法向压力（N）
    mu_mecanum: float = 0.0     # 麦克纳姆轮（辊子轴向传力）所需摩擦系数
    mu_locked: float = 0.0      # 四点抱死 / 支腿（各向同性摩擦）所需摩擦系数的下界
    min_load_ratio: float = 1.0   # 最小轮压 / 静态平均轮压（≤ 0 即倾覆）


def static_margins(wrench, platform):
    """轮压、所需摩擦系数与倾覆裕度."""
    loads = wheel_loads(wrench, platform)
    _, mu = roller_forces(wrench, platform, loads)
    total = sum(loads)
    lever = math.hypot(platform.half_length, platform.half_width)
    # 各向同性接触：合力由总正压力承担，力矩按四个接地点到中心的距离折算成等效切向力
    mu_locked = (math.hypot(wrench.fx, wrench.fy) + abs(wrench.mz) / lever) / total \
        if total > 0.0 and min(loads) > 0.0 else math.inf
    return StaticMargins(loads, mu, mu_locked, min(loads) / (platform.mass * G / 4.0))


# ---------------------------------------------------------------------------
# 随时间变化的切削过程
# ---------------------------------------------------------------------------
class Inhomogeneity:
    """
    材料不均匀性：沿切削路程的二阶高斯–马尔可夫过程，乘在比切削能上（均值 1）.

    两个相同的一阶环节串联（相关长度 length）。一阶过程的样本路径处处不可导，相当于
    切削力每前进零点几毫米就有一次小的跳变；刀片同时切削整个接触弧，力的变化应当是
    连续的，因此再经一次同样长度的平滑。输出按平稳方差归一化，变异系数为 cov。
    """

    def __init__(self, cov=INHOMOGENEITY_COV, length=INHOMOGENEITY_LENGTH, seed=1,
                 floor=0.2):
        self.cov, self.length, self.floor = cov, length, floor
        self._rng = random.Random(seed)
        self._x1 = self._rng.gauss(0.0, 1.0) if cov > 0.0 else 0.0
        self._x2 = self._x1 * math.sqrt(0.5)

    def advance(self, distance):
        """刀具沿槽线前进 distance（m）后的增益."""
        if self.cov <= 0.0:
            return 1.0
        rho = math.exp(-abs(distance) / self.length) if self.length > 0.0 else 0.0
        self._x1 = rho * self._x1 + math.sqrt(max(0.0, 1.0 - rho * rho)) * self._rng.gauss(0, 1)
        self._x2 = rho * self._x2 + (1.0 - rho) * self._x1
        # 第二级的平稳方差为 (1 + ρ²) / (1 + ρ)²
        x = self._x2 * (1.0 + rho) / math.sqrt(1.0 + rho * rho)
        return max(self.floor, 1.0 + self.cov * x)


@dataclass
class CutProfile:
    """
    一刀的时间历程.

    kind：
      'cut'   先下刀（切深以 plunge_rate 线性增加、不进给），到位后以 feed_rate 进给；
      'step'  t_start 时刻切深与进给同时到位（阶跃扰动，最不利的切入方式）；
      'ramp'  切削力在 ramp_time 内由 0 线性增加到 'step' 的数值（斜坡扰动，对应下刀与
              进给速度都缓慢增加的切入方式）。
    进给 length 后退刀（切削力回到 0）。
    """

    orientation: str = 'horizontal'
    feed_sign: int = 1
    depth: float = SLOT_DEPTH
    feed_rate: float = REF_FEED
    plunge_rate: float = 0.002
    length: float = 0.10
    t_start: float = 1.0
    kind: str = 'cut'
    ramp_time: float = 10.0

    @property
    def t_feed(self):
        """开始进给的时刻."""
        if self.kind == 'cut':
            return self.t_start + self.depth / self.plunge_rate
        return self.t_start

    @property
    def t_end(self):
        """进给结束（开始退刀）的时刻."""
        return self.t_feed + self.length / self.feed_rate

    def state(self, t):
        """时刻 t 的 (切深, 进给速度, 下刀速度, 已切长度)."""
        if t < self.t_start or t >= self.t_end:
            return 0.0, 0.0, 0.0, 0.0
        if self.kind == 'cut' and t < self.t_feed:
            return (t - self.t_start) * self.plunge_rate, 0.0, self.plunge_rate, 0.0
        return self.depth, self.feed_rate, 0.0, (t - self.t_feed) * self.feed_rate

    def force_scale(self, t):
        """'ramp' 的线性增益，其余为 1."""
        if self.kind != 'ramp':
            return 1.0
        return min(1.0, max(0.0, (t - self.t_start) / self.ramp_time))


class CuttingProcess:
    """按时间推进的切削过程：给出每个时刻的底盘力旋量（含不均匀性）."""

    def __init__(self, blade, platform, profile, seed=None, cov=INHOMOGENEITY_COV,
                 length=INHOMOGENEITY_LENGTH):
        self.blade, self.platform, self.profile = blade, platform, profile
        self.noise = Inhomogeneity(cov if seed is not None else 0.0, length,
                                   seed if seed is not None else 0)
        self._dist = 0.0
        self._gain = 1.0

    def _platform_at(self, dist):
        """刀片随进给移动：以底盘参数中的刀具位置为这一刀的中点."""
        off = dist - 0.5 * self.profile.length
        sgn = 1.0 if self.profile.feed_sign >= 0 else -1.0
        if self.profile.orientation == 'horizontal':
            return replace(self.platform, tool_y=self.platform.tool_y - sgn * off)
        return replace(self.platform, tool_z=self.platform.tool_z + sgn * off)

    def nominal(self, t):
        """不含不均匀性的力旋量（由规划可预知的部分，用作前馈）."""
        depth, vf, vp, dist = self.profile.state(t)
        return cut_wrench(self.blade, self._platform_at(dist), depth, vf,
                          self.profile.orientation, self.profile.feed_sign, vp,
                          self.profile.force_scale(t))

    def wrench(self, t):
        """时刻 t 的力旋量；须按时间递增的顺序调用（不均匀性过程随路程推进）."""
        depth, vf, vp, dist = self.profile.state(t)
        if dist > self._dist:
            self._gain = self.noise.advance(dist - self._dist)
        self._dist = dist
        return cut_wrench(self.blade, self._platform_at(dist), depth, vf,
                          self.profile.orientation, self.profile.feed_sign, vp,
                          self._gain * self.profile.force_scale(t))
