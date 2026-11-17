# -*- coding: utf-8 -*-
"""住宅墙面随机生成器：结构（承重 / 门窗 / 梁柱）+ 常见点位布置 + 1–4 项开槽任务.

单位：厘米（与 wall_rules 相同）。每面墙由一个整数种子完全确定：
    scene, meta = wall_gen.residential_wall(seed)
同一种子在任何机器上生成相同的墙（只用 random.Random，不依赖 numpy 随机数）。

点位高度（中心离地）与出处（标注方式同 wall_rules.py）：
  普通插座 30 cm            [行业] 插座底边距地不低于 0.3 m 的通行做法
  电视柜插座 55 cm、床头插座 68 cm  [行业]
  开关 130 cm、距门框 15–20 cm      [行业]（GB 50303 中有同样数值的表述，未核对原文）
  户内配电箱 底边 160–180 cm        [行业]
  家居弱电箱 底边 30–40 cm          [行业]
  吊顶 / 过路接线盒 230–250 cm      [假设]
  强弱电分槽、平行间距 ≥ 30 cm      [行业]（wall_rules.WEAK_CURRENT_SPACING）

开槽任务只取两端都在机械臂可切高度带内的回路（可切高度是任务约束，见 slot_execution
报告）；开关、配电箱、接线盒等带外点位只作为障碍出现，保持墙面布局真实。
  非承重墙：从已有插座 / 弱电口引线到同墙的新插座（同高、电视柜、床头）或新弱电口；
  承重墙：禁止横槽，只能在已有点位正上方加新点位（竖槽）。
"""
import random

import wall_rules as wr

BAND_CM = (26.929, 79.929)     # RobotModel().reach_band × 100（测试中校验一致）
BOX = 9                        # 86 型线盒外形约 86 mm
SOCKET_H, CABINET_H, BEDSIDE_H = 30, 55, 68
WEAK_SEP = 40                  # 强、弱电两端线盒中心最小距离：30 cm 间距 + 线盒半宽

# 任务类型：(名称, 回路, 新点位高度, |Δu| 范围, 权重)
TASK_TYPES_PARTITION = (
    ('extend', 'strong', SOCKET_H, (40, 180), 3),      # 同高延伸插座
    ('cabinet', 'strong', CABINET_H, (30, 150), 2),    # 电视柜插座
    ('bedside', 'strong', BEDSIDE_H, (30, 150), 2),    # 床头插座
    ('weak', 'weak', SOCKET_H, (40, 180), 1),          # 网络 / 电视弱电口
    ('weak_cab', 'weak', CABINET_H, (30, 150), 1),     # 电视柜处弱电口
)
TASK_TYPES_LOAD_BEARING = (
    ('cabinet', 'strong', CABINET_H, (0, 0), 2),
    ('bedside', 'strong', BEDSIDE_H, (0, 0), 2),
    ('weak_cab', 'weak', CABINET_H, (0, 0), 1),
)
LABELS = {'extend': '新插座', 'cabinet': '电视柜插座', 'bedside': '床头插座',
          'weak': '新弱电口', 'weak_cab': '电视柜弱电口'}


def box_at(cx, cy, label, kind='box'):
    return wr.Rect(cx - BOX // 2, cy - BOX // 2, BOX, BOX, kind, label)


def _overlap(r, others, gap=0):
    return any(r.x < o.x + o.w + gap and o.x < r.x + r.w + gap and
               r.y < o.y + o.h + gap and o.y < r.y + r.h + gap for o in others)


def _structure(rng, W, H, lb):
    """梁、柱、门、窗、承重墙窄墙垛."""
    obs = []
    if rng.random() < 0.5:
        bh = rng.randrange(40, 61, 5)
        obs.append(wr.Rect(0, H - bh, W, bh, 'beam', '梁'))
    if rng.random() < 0.3:
        cw = rng.randrange(30, 51, 5)
        obs.append(wr.Rect(0 if rng.random() < 0.5 else W - cw, 0, cw, H, 'column', '柱'))
    if rng.random() < 0.5:
        dw = rng.randrange(80, 101, 10)
        for _ in range(20):
            r = wr.Rect(rng.randrange(wr.EDGE_MARGIN, W - dw - wr.EDGE_MARGIN), 0, dw, 210,
                        'door', '门')
            if not _overlap(r, obs, 30):
                obs.append(r)
                break
    if rng.random() < 0.5:
        ww = rng.randrange(100, 181, 10)
        sill = 90 if rng.random() < 0.75 else 50          # 普通窗 / 飘窗
        top = min(H - 30, sill + 150)
        for _ in range(20):
            r = wr.Rect(rng.randrange(wr.EDGE_MARGIN, W - ww - wr.EDGE_MARGIN), sill, ww,
                        top - sill, 'window', '窗' if sill == 90 else '飘窗')
            if not _overlap(r, obs, 30):
                obs.append(r)
                break
    if lb:
        spans = sorted([(o.x, o.x + o.w) for o in obs if o.kind in ('door', 'window', 'column')]
                       + [(-1, 0), (W, W + 1)])
        for (a0, a1), (b0, b1) in zip(spans[:-1], spans[1:]):
            if 0 < b0 - a1 < wr.PIER_MIN:
                obs.append(wr.Rect(a1, 0, b0 - a1, H, 'pier', '窄墙垛'))
    return obs


def _free_box(scene, circuit, cx, cy, extra):
    """线盒（中心 cx, cy）不与任何障碍的外扩区重叠，且离墙边足够远."""
    if not (wr.EDGE_MARGIN + BOX <= cx <= scene.width - wr.EDGE_MARGIN - BOX):
        return False
    r = box_at(cx, cy, '')
    for o in scene.obstacles + extra:
        if _overlap(r, [o.grown(scene.margin_of(o, circuit))]):
            return False
    return True


def _fixtures(rng, scene, door):
    """带外点位（开关、配电箱、接线盒）与带内已有点位（插座、弱电口、弱电箱）."""
    W, H = scene.width, scene.height
    obs = scene.obstacles
    if door is not None:                                  # 门边开关
        side = rng.choice((-1, 1))
        off = rng.randrange(15, 21)
        cx = door.x - off - BOX // 2 if side < 0 else door.x + door.w + off + BOX // 2
        if wr.EDGE_MARGIN <= cx <= W - wr.EDGE_MARGIN:
            obs.append(box_at(cx, 130, '开关'))
    if rng.random() < 0.25:                               # 户内配电箱
        for _ in range(20):
            r = wr.Rect(rng.randrange(wr.EDGE_MARGIN, W - 45), rng.randrange(160, 181, 5),
                        35, 30, 'box', '配电箱')
            if not _overlap(r, obs, 10):
                obs.append(r)
                break
    for _ in range(rng.randrange(0, 3)):                  # 吊顶 / 过路接线盒
        r = box_at(rng.randrange(20, W - 20), rng.randrange(230, 251, 5), '接线盒')
        if not _overlap(r, obs, 10):
            obs.append(r)
    if rng.random() < 0.2:                                # 家居弱电箱（带内的大障碍）
        for _ in range(20):
            r = wr.Rect(rng.randrange(wr.EDGE_MARGIN, W - 40), rng.randrange(30, 41, 5),
                        30, 40, 'box', '弱电箱')
            if not _overlap(r, obs, 20):
                obs.append(r)
                break
    if rng.random() < 0.25:                               # 已有弱电竖向线槽（到吊顶）
        x = rng.randrange(wr.EDGE_MARGIN, W - wr.EDGE_MARGIN)
        r = wr.Rect(x, SOCKET_H, 2, H - SOCKET_H, 'weak_line', '弱电线槽')
        if not _overlap(r, obs, 0):
            obs.append(r)

    def place(n, circuit, label):
        out = []
        for _ in range(n):
            for _ in range(30):
                cx = rng.randrange(wr.EDGE_MARGIN + BOX, W - wr.EDGE_MARGIN - BOX)
                if _free_box(scene, circuit, cx, SOCKET_H, []) and \
                        all(abs(cx - b[0]) >= 30 for b in out):
                    out.append((cx, SOCKET_H, circuit, label))
                    break
        return out

    sockets = place(rng.randrange(1, 5), 'strong', '已有插座')
    weak = place(rng.randrange(0, 2), 'weak', '已有弱电口')
    return sockets + weak


def residential_wall(seed, n_slots=None, load_bearing=None, max_tries=200, routable=None):
    """按种子生成一面住宅墙及开槽任务 → (WallScene, meta)；放不下任务时换布局重试.

    n_slots：任务条数（None 时 1–4 均匀）；load_bearing：None 时 30 % 为承重墙。
    routable(scene, k) -> bool：可选的单槽可行性检查（例如在可切高度带内单独规划第 k
    条槽，见 task_plan.standalone_routable）；给出时只保留各自单独可行的任务。多条槽
    之间的相互阻挡不在这里排除——那是规划方法要解决的问题。
    meta：dict(seed, n_slots, load_bearing, task_types)。
    """
    rng = random.Random(seed)
    n = n_slots or rng.randint(1, 4)
    lb = (rng.random() < 0.3) if load_bearing is None else load_bearing
    for _ in range(max_tries):
        W, H = rng.randrange(300, 601, 10), 280
        structure = _structure(rng, W, H, lb)
        scene = wr.WallScene(W, H, lb, list(structure))
        door = next((o for o in structure if o.kind == 'door'), None)
        feeds = _fixtures(rng, scene, door)
        tasks = _tasks(rng, scene, feeds, n, lb)
        if tasks is None:
            continue
        used = {(t[0][0], t[0][1]) for t in tasks}
        for cx, cy, circuit, label in feeds:            # 未用作起点的已有点位是普通障碍
            if (cx, cy) not in used:
                scene.obstacles.append(box_at(cx, cy, label))
        scene.tasks = [t[:4] for t in tasks]
        scene.circuits = [t[4] for t in tasks]
        scene.task_boxes = [(box_at(*t[0], t[2]), box_at(*t[1], t[3])) for t in tasks]
        scene.start, scene.goal, scene.start_label, scene.goal_label = scene.tasks[0]
        if routable is not None and not all(routable(scene, k) for k in range(n)):
            continue
        return scene, dict(seed=seed, n_slots=n, load_bearing=lb,
                           task_types=[t[5] for t in tasks])
    raise RuntimeError(f'seed {seed}: no layout after {max_tries} tries')


def _tasks(rng, scene, feeds, n, lb):
    """n 项任务 [(start, goal, start_label, goal_label, circuit, type)]；放不下返回 None."""
    types = TASK_TYPES_LOAD_BEARING if lb else TASK_TYPES_PARTITION
    weights = [t[4] for t in types]
    out, ends = [], []                                    # ends: (cx, cy, circuit)
    free_feeds = list(feeds)

    def clear_of_others(cx, cy, circuit):
        for ex, ey, ec in ends:
            gap = WEAK_SEP if ec != circuit else 25
            if abs(cx - ex) < gap and abs(cy - ey) < gap:
                return False
        return True

    for _ in range(n):
        for _ in range(60):
            name, circuit, gy, (dmin, dmax), _ = rng.choices(types, weights)[0]
            cand = [f for f in free_feeds if f[2] == circuit]
            if not cand:                                  # 没有同类已有点位：补一个
                cx = rng.randrange(wr.EDGE_MARGIN + BOX, scene.width - wr.EDGE_MARGIN - BOX)
                if not _free_box(scene, circuit, cx, SOCKET_H, []):
                    continue
                f = (cx, SOCKET_H, circuit, '已有插座' if circuit == 'strong' else '已有弱电口')
            else:
                f = rng.choice(cand)
            du = rng.randint(dmin, dmax) * rng.choice((-1, 1))
            gx = f[0] + du
            if not (_free_box(scene, circuit, gx, gy, [])
                    and clear_of_others(f[0], f[1], circuit)
                    and clear_of_others(gx, gy, circuit)
                    and _endpoint_clear_of_feeds(gx, gy, circuit, feeds, f)):
                continue
            if abs(gy - f[1]) == 0 and abs(du) < 2 * wr.BEND_RADIUS + 10:
                continue                                  # 同高且太近：没有合规的阶梯走法
            out.append(((f[0], f[1]), (gx, gy), f[3], LABELS[name], circuit, name))
            ends.extend([(f[0], f[1], circuit), (gx, gy, circuit)])
            if f in free_feeds:
                free_feeds.remove(f)
            break
        else:
            return None
    return out


def _endpoint_clear_of_feeds(gx, gy, circuit, feeds, own):
    """新点位不能压在其他已有点位上（含外扩）."""
    for f in feeds:
        if f is own:
            continue
        gap = (WEAK_SEP if f[2] != circuit else BOX + wr.BOX_MARGIN)
        if abs(gx - f[0]) < gap and abs(gy - f[1]) < gap:
            return False
    return True


def scene_to_config(scene, left_end=(0.0, 0.0), frame_id='map'):
    """WallScene → plan_wall_slots.py 可读的任务配置（米）."""
    m = lambda v: round(v / 100.0, 3)  # noqa: E731
    geom = lambda o: dict(u=m(o.x), v=m(o.y), w=m(o.w), h=m(o.h))  # noqa: E731
    tasks = []
    for k, (s, g, sl, gl) in enumerate(scene.tasks):
        # 配置中的点坐标 = 栅格下标 / 100（与 slot_plan.scene_from_config 一致）
        t = dict(start=[m(s[0]), m(s[1])], start_label=sl, goal=[m(g[0]), m(g[1])],
                 goal_label=gl, circuit=scene.circuit_of(k))
        if k < len(scene.task_boxes):
            sb, gb = scene.task_boxes[k]
            t.update(start_box=geom(sb), goal_box=geom(gb))
        tasks.append(t)
    return dict(
        wall=dict(frame_id=frame_id, left_end=list(left_end),
                  right_end=[left_end[0] + m(scene.width), left_end[1]],
                  height=m(scene.height), load_bearing=bool(scene.load_bearing)),
        obstacles=[dict(kind=o.kind, label=o.label, **geom(o)) for o in scene.obstacles],
        tasks=tasks)
