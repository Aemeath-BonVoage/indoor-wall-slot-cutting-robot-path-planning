#!/usr/bin/env python3
"""
机械臂轨迹实验的汇总与作图（不需要 ROS / Gazebo）.

  python3 docs/experiments/arm_trajectory/make_report.py

读取 runs/<运行名>/ 与 ../plan_validation/runs/（逐点执行的此前运行），写出：
  runs_summary.csv      每次运行一行：时间分解、刀尖轨迹误差、到站误差、底盘漂移、覆盖率、
                        关节跟踪误差
  stations_all.csv      逐站数据（含生成器给出的轨迹时长与时间模型预测）
  calibration.csv       时间模型 traj_v1 的标定与验证
  summary_tables.md     报告正文所用的两张表（逐点 vs 连续；参数扫描）
  fig_profile.png       同一停靠站逐点执行与连续轨迹的速度曲线（由 extract_profile.py 导出
                        的 profile_*.csv 作图）
  fig_time.png          各站耗时之和的分解：逐点 vs 连续
  fig_tradeoff.png      进给 / 加速度 / 空行程速度 / 加加速度扫描：时间与精度
  fig_time_model.png    时间模型：预测 vs 实测
  fig_planning.png      400 面墙上联合规划的节省：逐点执行的时间模型 vs 连续轨迹的时间模型
  planning_summary.csv  上图的统计量（benchmark/data/results.csv 由
                        task_planning_benchmark.py --time-model traj_v1 生成）
所有图同时写出同名 PDF。
"""
import csv
import glob
import gzip
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'docs'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'python'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_arm_controller'))

import calibrate_time_model as cal  # noqa: E402
import slot_plan as sp  # noqa: E402
from scr_arm_controller.arm_trajectory import (ARM_MOTION_PARAMS, MotionLimits,  # noqa: E402
                                               limits_from_params, plan_trajectory)
from scr_arm_controller.slot_execution import station_arm_points  # noqa: E402

RUNS = os.path.join(HERE, 'runs')
OLD_RUNS = os.path.join(HERE, '..', 'plan_validation', 'runs')
ROBOT = sp.RobotModel()
SAMPLE_DT = 0.05            # /gazebo/link_states 的采样周期（20 Hz）
T_RESIDUAL = 1.0            # CutStation 开始时的到站残差测量（10 个周期 × 0.1 s）
# 机械臂在观测位姿（横移前回到该位姿）下的关节值：滑台 observe_joint1，伸缩轴缩回
OBSERVE = (-0.15, 0.0, 0.02, 0.0)
# 时间模型：标定用运行与验证用运行（都是默认参数的连续轨迹）
CAL_RUNS = ('cont_dp11_s1', 'cont_dp11_s2', 'cont_rule_s1', 'cont_rule_s2')
VAL_RUNS = ('cont_joint_s1', 'cont_joint_s2')
PARAM_KEYS = ('feed_rate', 'plunge_rate', 'feed_accel', 'rapid_speed', 'rapid_accel',
              'rapid_accel_lateral', 'rot_speed', 'rot_accel', 'jerk_time', 'dwell')


# ---------------------------------------------------------------------------
# 读取
# ---------------------------------------------------------------------------
def read_trace(d):
    """tool_trace → 列表 (t, station, u, v, depth, arm_x, phase, base_u, base_d)."""
    path = os.path.join(d, 'tool_trace.csv.gz')
    opener = gzip.open if os.path.exists(path) else open
    path = path if os.path.exists(path) else os.path.join(d, 'tool_trace.csv')
    out = []
    with opener(path, 'rt') as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            out.append((float(row[0]), int(row[1]), float(row[2]), float(row[3]), float(row[4]),
                        float(row[5]), row[6], float(row[7]), float(row[8])))
    return out


def run_params(d):
    """本次运行的 arm_motion 参数（默认值 + skill_params.yaml 中的覆盖）."""
    p = dict(ARM_MOTION_PARAMS)
    path = os.path.join(d, 'skill_params.yaml')
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            p.update(yaml.safe_load(f)['/**']['ros__parameters'].get('arm_motion', {}))
    else:
        p['mode'] = 'point_to_point'        # 此前的运行（plan_validation）：逐点执行
    return p


def ik(p):
    """笛卡尔点 → 关节值（与 cartesian_cutting.inverse_kinematics 相同）."""
    return (p[2] - ROBOT.z0, -(p[1] - ROBOT.y0), -(p[0] - ROBOT.x0), p[3])


def station_trajectory(station, plan_dict, limits, gap):
    """规划文件中一个停靠站的名义轨迹（从观测位姿出发，空切）."""
    pts, is_cut = station_arm_points(station, plan_dict, gap)
    return plan_trajectory(OBSERVE, [ik(p) for p in pts], is_cut, limits)


# ---------------------------------------------------------------------------
# 单次运行的指标
# ---------------------------------------------------------------------------
def dist_poly(p, pts):
    p = np.asarray(p)
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a), np.asarray(b)
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def cutting_runs(trace):
    runs, prev = [], None
    for k, r in enumerate(trace):
        if r[1] >= 0 and r[6] == 'cut':
            if prev != k - 1:
                runs.append([])
            runs[-1].append((r[2], r[3]))
            prev = k
    return runs


def path_metrics(trace, plan):
    """刀尖轨迹误差（切割阶段样本到规划槽线的距离）与覆盖率（槽线取样点到刀尖折线）."""
    cut = [(r[2], r[3]) for r in trace if r[1] >= 0 and r[6] == 'cut']
    d = np.array([min(dist_poly(q, cp) for cp in plan.cut_paths) for q in cut]) * 1000
    band = ROBOT.reach_band
    pts = [q for cp in plan.cut_paths
           for piece in sp.clip_polyline(cp, band[0], band[1], axis=1)
           for q in sp._densify(piece, 0.005)]
    lines = [r for r in cutting_runs(trace) if len(r) >= 2]
    covered = sum(min(dist_poly(q, line) for line in lines) < 0.003 for q in pts)
    return dict(path_err_mean_mm=float(d.mean()), path_err_p95_mm=float(np.percentile(d, 95)),
                path_err_max_mm=float(d.max()), coverage_path=covered / len(pts))


def cross_track(trace):
    """
    切割段内的横向偏差（毫米）：每段刀尖轨迹垂直于切割方向的坐标相对该段中位数的偏离.

    刀尖轨迹误差（path_metrics）包含底盘到站的沿墙误差：它对一个停靠站是常数，整段竖切
    都偏同一个量，与机械臂怎么运动无关。这里去掉每段的常数偏移，只留下切割过程中机械臂
    跟踪误差与底盘晃动造成的偏离，用来比较不同的运动参数。
    """
    dev = []
    for run in cutting_runs(trace):
        if len(run) < 3:
            continue
        uv = np.array(run)
        cross = uv[:, 1] if np.ptp(uv[:, 0]) > np.ptp(uv[:, 1]) else uv[:, 0]
        dev += list(np.abs(cross - np.median(cross)) * 1000)
    dev = np.array(dev)
    return dict(cross_rms_mm=float(np.sqrt(np.mean(dev ** 2))),
                cross_p95_mm=float(np.percentile(dev, 95)), cross_max_mm=float(dev.max()))


def base_drift(trace):
    """
    机械臂运动期间的底盘漂移（真值，毫米）：每站相对机械臂开始运动时刻的最大位移.

    与 stations.csv 的 base_drift_mm（相对到站时刻、含残差测量的 1 s）口径不同：这里只看
    机械臂在动的时段，用来比较机械臂运动对底盘的扰动。
    """
    out = {}
    for r in trace:
        if r[1] >= 0 and r[6] in ('move', 'cut'):
            out.setdefault(r[1], []).append((r[7], r[8]))
    return [max(math.hypot(u - b[0][0], d - b[0][1]) for u, d in b) * 1000
            for _, b in sorted(out.items())]


def tracking(d):
    """关节跟踪误差（指令 − 实际），取机械臂运动时段的样本；没有 arm_state.csv 时返回空."""
    path = os.path.join(d, 'arm_state.csv')
    if not os.path.exists(path):
        return {}
    rows = [r for r in csv.DictReader(open(path)) if int(r['station']) >= 0
            and r['phase'] in ('move', 'cut')]
    if not rows:
        return {}
    e = np.array([[float(r[f'ref_joint{i}']) - float(r[f'act_joint{i}']) for i in (1, 2, 3, 4)]
                  for r in rows])
    cut = np.array([r['phase'] == 'cut' for r in rows])
    lin = np.abs(e[:, :3]) * 1000
    out = dict(track_lin_rms_mm=float(np.sqrt(np.mean(lin ** 2))),
               track_lin_max_mm=float(lin.max()),
               track_rot_max_deg=float(np.degrees(np.abs(e[:, 3]).max())))
    if cut.any():
        out.update(track_cut_rms_mm=float(np.sqrt(np.mean(lin[cut] ** 2))),
                   track_cut_max_mm=float(lin[cut].max()))
    return out


def summarize(d):
    name = os.path.basename(d)
    with open(os.path.join(d, 'summary.yaml'), encoding='utf-8') as f:
        s = yaml.safe_load(f)
    with open(os.path.join(d, 'slot_plan.yaml'), encoding='utf-8') as f:
        plan_dict = yaml.safe_load(f)
    plan, _, _ = sp.plan_from_dict(plan_dict)
    rows = cal.load([d])
    p = run_params(d)
    trace = read_trace(d)
    n = {ph: sum(1 for r in trace if r[1] >= 0 and r[6] == ph) for ph in ('cut', 'move')}
    f = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    total, move, cut_action = f('station_s').sum(), f('move_s').sum(), f('cut_s').sum()
    t_cut, t_arm_other = n['cut'] * SAMPLE_DT, n['move'] * SAMPLE_DT
    lat = np.abs(f('lateral_mm'))
    drift = base_drift(trace)
    res = dict(run=name, mode=p['mode'], plan=plan_dict.get('routes', ''),
               points=int(f('points').sum()), stations=len(rows), all_ok=s.get('all_ok'),
               **{k: p[k] for k in PARAM_KEYS},
               stations_total_s=float(total), base_move_s=float(move),
               cut_action_s=float(cut_action), arm_cut_s=t_cut, arm_other_s=t_arm_other,
               fixed_s=float(total - move - t_cut - t_arm_other),
               sim_time_s=s['sim_time_s'],
               lateral_mm_mean=float(lat.mean()), lateral_mm_max=float(lat.max()),
               hold_drift_mm_mean=float(f('base_drift_mm').mean()),
               hold_drift_mm_max=float(f('base_drift_mm').max()),
               arm_drift_mm_mean=float(np.mean(drift)), arm_drift_mm_max=float(np.max(drift)),
               coverage=s.get('coverage'), recoveries=s.get('recoveries'))
    res.update(path_metrics(trace, plan))
    res.update(cross_track(trace))
    res.update(tracking(d))
    # 逐站：生成器给出的名义轨迹时长（连续轨迹）
    lim = limits_from_params(p)
    gap = s.get('dry_run_gap')
    for r in rows:
        st = next(x for x in plan_dict['stations'] if x['index'] == int(r['station']))
        r['traj_s'] = (station_trajectory(st, plan_dict, lim, gap).duration
                       if p['mode'] == 'continuous' else float('nan'))
        r['mode'] = p['mode']
    return res, rows


def write_csv(path, rows):
    keys = list(rows[0].keys())
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in r.items()})


# ---------------------------------------------------------------------------
# 时间模型
# ---------------------------------------------------------------------------
def analytic_t_point(lim, plunge):
    """
    由轨迹限值推出的每点耗时（秒）：一个直线小段的 4 个点 = 下刀 + 切割段的加减速开销 +
    下刀后的等待 + 退刀 + 转刀，除以 4。plunge：下刀 / 退刀行程（米）。
    """
    from scr_arm_controller.arm_trajectory import rest_to_rest_time
    ramp = lim.feed / lim.feed_accel + lim.jerk_time
    t_plunge = rest_to_rest_time(plunge, lim.path('plunge'))
    t_retract = rest_to_rest_time(plunge, lim.axis(2))
    t_rot = rest_to_rest_time(math.pi / 2, lim.axis(3))
    return dict(ramp=ramp, plunge=t_plunge, dwell=lim.dwell, retract=t_retract, rotate=t_rot,
                t_point=(ramp + t_plunge + lim.dwell + t_retract + t_rot) / 4.0)


def time_model(all_rows):
    cal_rows = [r for r in all_rows if r['run'] in CAL_RUNS]
    val_rows = [r for r in all_rows if r['run'] in VAL_RUNS]
    if not cal_rows:
        return None
    feed = ARM_MOTION_PARAMS['feed_rate']
    out = []
    for label, fit in (('fit:traj (v_feed fixed)', lambda a: cal.fit_traj(a, feed)),
                       ('fit:traj (v_feed free)', cal.fit_traj),
                       ('fit:phases-reg (no length term)', cal.fit_phases_reg)):
        p = fit(cal.arrays(cal_rows))
        st = cal.stats(p, cal_rows)
        loo = []
        for k in sorted({r['run'] for r in cal_rows}):
            q = fit(cal.arrays([r for r in cal_rows if r['run'] != k]))
            test = [r for r in cal_rows if r['run'] == k]
            loo.append(cal.stats(q, test)['rmse'] ** 2 * len(test))
        row = dict(source=label, t_settle=p[0], v_strafe=p[1], t_point=p[2], v_feed=p[3],
                   stations=len(cal_rows), rmse_s=st['rmse'], bias_s=st['bias'],
                   loo_rmse_s=float(np.sqrt(sum(loo) / len(cal_rows))),
                   max_abs_total_err_s=max(abs(t) for t in st['total_err']))
        if val_rows:
            sv = cal.stats(p, val_rows)
            row.update(val_stations=len(val_rows), val_rmse_s=sv['rmse'], val_bias_s=sv['bias'],
                       val_total_err_s=' / '.join(f'{t:+.1f}' for t in sv['total_err']))
        out.append(row)
    for name in ('traj_v1', 'align_v3'):
        if name not in sp.TIME_MODELS:
            continue
        p = cal.preset_params(sp.TimeModel.preset(name))
        st = cal.stats(p, cal_rows)
        row = dict(source=f'preset:{name}', t_settle=p[0], v_strafe=p[1], t_point=p[2],
                   v_feed=p[3], stations=len(cal_rows), rmse_s=st['rmse'], bias_s=st['bias'],
                   loo_rmse_s='', max_abs_total_err_s=max(abs(t) for t in st['total_err']))
        if val_rows:
            sv = cal.stats(p, val_rows)
            row.update(val_stations=len(val_rows), val_rmse_s=sv['rmse'], val_bias_s=sv['bias'],
                       val_total_err_s=' / '.join(f'{t:+.1f}' for t in sv['total_err']))
        out.append(row)
    return out


def generator_check(all_rows):
    """实测切割阶段耗时 vs 生成器给出的轨迹时长：cut_s = c + k·traj_s."""
    rows = [r for r in all_rows if r['mode'] == 'continuous']
    x = np.array([r['traj_s'] for r in rows])
    y = np.array([float(r['cut_s']) for r in rows])
    (c, k), *_ = np.linalg.lstsq(np.c_[np.ones_like(x), x], y, rcond=None)
    e = c + k * x - y
    return dict(stations=len(rows), intercept_s=float(c), slope=float(k),
                rmse_s=float(np.sqrt(np.mean(e ** 2))), max_abs_s=float(np.abs(e).max()),
                mean_overhead_s=float(np.mean(y - x)))


# ---------------------------------------------------------------------------
# 图
# ---------------------------------------------------------------------------
def by_run(summ):
    return {r['run']: r for r in summ}


def plot_time(summ, out):
    import figstyle as fs
    fig, ax = fs.subplots(width='single', height=2.7)
    groups = [('rule', 'point_to_point', 'Rule-based routes\npoint-to-point'),
              ('rule', 'continuous', 'Rule-based routes\ncontinuous'),
              ('joint', 'point_to_point', 'Joint planning\npoint-to-point'),
              ('joint', 'continuous', 'Joint planning\ncontinuous')]
    parts = [('base_move_s', 'Base move to station', 'neutral'),
             ('fixed_s', 'Residual measurement, overhead', 'alt2'),
             ('arm_other_s', 'Arm: reposition, plunge, retract, rotate', 'baseline'),
             ('arm_cut_s', 'Arm: cutting segments', 'proposed')]
    default = {k: ARM_MOTION_PARAMS[k] for k in PARAM_KEYS}
    for i, (plan, mode, _) in enumerate(groups):
        rs = [r for r in summ if r['plan'] == plan and r['mode'] == mode and r['all_ok']
              and (mode == 'point_to_point' or all(r[k] == default[k] for k in PARAM_KEYS))
              and r['points'] in (80, 88)]
        if not rs:
            continue
        bottom = 0.0
        for key, lab, role in parts:
            v = float(np.mean([r[key] for r in rs]))
            ax.bar(i, v, 0.62, bottom=bottom, label=lab if i == 0 else None, **fs.bar(role))
            bottom += v
        tot = [r['stations_total_s'] for r in rs]
        ax.text(i, bottom + 3, f'{np.mean(tot):.0f} s\n(n = {len(rs)})', ha='center',
                va='bottom', fontsize=7)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([g[2] for g in groups])
    ax.set_ylabel(fs.label('Sum of station times (simulation)', 's'))
    ax.set_ylim(0, 330)
    ax.legend(loc='upper right', ncol=1)
    fs.save(fig, out)


def sweep_rows(summ, key, others):
    """
    联合规划上只改 key（及随它一起改的 others）的连续轨迹运行，按 key 排序.

    进给、加速度、空行程速度、加加速度的扫描在 dwell = 0 下进行（确定下刀后等待时间之前），
    dwell 的扫描在其余参数取默认值下进行。
    """
    base = {k: ARM_MOTION_PARAMS[k] for k in PARAM_KEYS}
    if key != 'dwell':
        base['dwell'] = 0.0
    rs = [r for r in summ if r['mode'] == 'continuous' and r['plan'] == 'joint' and r['all_ok']
          and r['points'] == 80
          and all(r[k] == base[k] for k in PARAM_KEYS if k != key and k not in others)]
    if others:      # 一起改的参数与 key 保持同一比例
        rs = [r for r in rs if all(abs(r[o] / base[o] - r[key] / base[key]) < 1e-9
                                   for o in others)]
    return sorted(rs, key=lambda r: r[key]), base[key]


def plot_tradeoff(summ, out):
    import figstyle as fs
    fig, axes = fs.subplots(2, 5, width='double', height=3.7, sharex='col', sharey='row')
    sweeps = [('feed_rate', (), 'Feed rate', 'm/s'),
              ('rapid_accel', ('feed_accel', 'rot_accel'), 'Rapid accel.', 'm/s$^2$'),
              ('rapid_speed', (), 'Rapid speed', 'm/s'),
              ('jerk_time', (), 'Jerk ramp time', 's'),
              ('dwell', (), 'Dwell after plunge', 's')]
    p2p = [r for r in summ if r['mode'] == 'point_to_point' and r['plan'] == 'joint']
    for j, (key, others, name, unit) in enumerate(sweeps):
        rs, ref = sweep_rows(summ, key, others)
        xs = sorted({r[key] for r in rs})
        pos = {x: i for i, x in enumerate(xs)}          # 等间距的类别轴，刻度写参数值
        top, bot = axes[0][j], axes[1][j]

        def series(k):
            return [float(np.mean([r[k] for r in rs if r[key] == x])) for x in xs]

        for r in rs:
            top.plot(pos[r[key]], r['cut_action_s'], **fs.marker('proposed', ms=3))
        top.plot(range(len(xs)), series('cut_action_s'), label='Continuous',
                 **fs.line('proposed', lw=0.8))
        if p2p:
            top.axhline(np.mean([r['cut_action_s'] for r in p2p]),
                        label='Point-to-point', **fs.line('baseline', lw=0.8))
        for k, role, lab in (('cross_max_mm', 'proposed', 'Cross-track deviation, max'),
                             ('track_lin_max_mm', 'alt', 'Joint tracking error, max'),
                             ('arm_drift_mm_max', 'alt2', 'Base drift, max')):
            for r in rs:
                bot.plot(pos[r[key]], r[k], **fs.marker(role, ms=3))
            bot.plot(range(len(xs)), series(k), label=lab, **fs.line(role, lw=0.8))
        if p2p:
            bot.axhline(np.mean([r['cross_max_mm'] for r in p2p]),
                        label='Cross-track deviation, max,\npoint-to-point',
                        **fs.line('baseline', lw=0.8))
        for ax in (top, bot):
            if ref in pos:
                ax.axvline(pos[ref], color=fs.GREY, lw=0.6, ls=(0, (1, 2)))
            ax.set_xlim(-0.5, len(xs) - 0.5)
        bot.set_xticks(range(len(xs)))
        bot.set_xticklabels([f'{x:g}' for x in xs])
        bot.set_xlabel(fs.label(name, unit))
    axes[0][0].set_ylim(0, 200)
    axes[1][0].set_ylim(0, 1.9)
    axes[0][0].set_ylabel(fs.label('CutStation time, 11 stations', 's'))
    axes[1][0].set_ylabel(fs.label('Error', 'mm'))
    axes[0][2].legend(loc='lower left')
    axes[1][3].legend(loc='upper left')
    fs.save(fig, out)


def plot_time_model(all_rows, out):
    import figstyle as fs
    if 'traj_v1' not in sp.TIME_MODELS:
        return
    p = cal.preset_params(sp.TimeModel.preset('traj_v1'))
    fig, axes = fs.subplots(1, 2, width='double', height=3.1)
    ax = axes[0]
    for runs, role, lab, mk in ((CAL_RUNS, 'neutral', 'Calibration data', 'o'),
                                (VAL_RUNS, 'proposed', 'Validation data', '^')):
        rows = [r for r in all_rows if r['run'] in runs]
        if not rows:
            continue
        a = cal.arrays(rows)
        ax.plot(a['y'], cal.predict(p, a), ls='none', marker=mk, ms=3.2, mfc='none',
                mec=fs.color(role), mew=0.7,
                label=f'{lab} ({len({r["run"] for r in rows})} runs, {len(rows)} stations)')
    lim = [5, 30]
    ax.plot(lim, lim, label='Prediction = measurement', **fs.line('reference', lw=0.7))
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel(fs.label('Measured time per station (simulation)', 's'))
    ax.set_ylabel(fs.label('Predicted time per station', 's'))
    val = [r for r in all_rows if r['run'] in VAL_RUNS]
    if val:
        e = cal.predict(p, cal.arrays(val)) - cal.arrays(val)['y']
        bias = f'{np.mean(e):+.2f}'.replace('-', '−')
        fs.note(ax, f'Validation: RMSE {np.sqrt(np.mean(e ** 2)):.2f} s,\nmean bias {bias} s',
                x=0.97, y=0.03, va='bottom')
    ax.legend(loc='upper left')
    # (b) 生成器时长 vs 实测切割阶段耗时（全部连续轨迹运行，含参数扫描）
    ax = axes[1]
    rows = [r for r in all_rows if r['mode'] == 'continuous']
    x = np.array([r['traj_s'] for r in rows])
    y = np.array([float(r['cut_s']) for r in rows])
    ax.plot(x, y, ls='none', marker='o', ms=2.6, mfc='none', mec=fs.color('proposed'), mew=0.6,
            label=f'Stations ({len({r["run"] for r in rows})} runs, {len(rows)} stations)')
    g = generator_check(all_rows)
    xx = np.array([0.0, max(x) * 1.05])
    ax.plot(xx, g['intercept_s'] + g['slope'] * xx, **fs.line('reference', lw=0.7),
            label=f'Fit: {g["intercept_s"]:.2f} s + {g["slope"]:.3f} × duration')
    ax.set_xlabel(fs.label('Trajectory duration from the generator', 's'))
    ax.set_ylabel(fs.label('Measured CutStation time', 's'))
    ax.legend(loc='upper left')
    fs.panel_labels(axes, ['Time model traj_v1', 'Generator duration vs. measurement'])
    fs.save(fig, out)


def plot_profile(out):
    """
    逐点执行与连续轨迹在同一停靠站的关节速度（extract_profile.py 导出的数据）.

    速度由关节位置对时间求差分得到：Gazebo 给出的升降轴速度带约 +5 mm/s 的常数偏置
    （静止时也不为零），位置没有这个问题。
    """
    import figstyle as fs
    files = {m: os.path.join(HERE, f'profile_{m}.csv') for m in ('p2p', 'cont')}
    if not all(os.path.exists(f) for f in files.values()):
        return
    fig, axes = fs.subplots(2, 1, width='double', height=3.6, sharex=True, sharey=True)
    names = [('q_joint1', 'Lift (Z)', 'proposed'), ('q_joint2', 'Along wall (Y)', 'baseline'),
             ('q_joint3', 'Extend (X)', 'alt'), ('q_joint4', 'Blade rotation', 'alt2')]
    for ax, mode in zip(axes, ('p2p', 'cont')):
        rows = [r for r in csv.DictReader(open(files[mode]))
                if r['phase'] in ('move', 'cut') and float(r['t']) >= 0.0]
        t = np.array([float(r['t']) for r in rows])
        for key, lab, role in names:
            v = np.gradient(np.array([float(r[key]) for r in rows]), t)
            scale = 1.0 if key != 'q_joint4' else 0.1
            ax.plot(t, v * scale, label=lab + (' (rad/s × 0.1)' if key == 'q_joint4' else ''),
                    **fs.line(role, lw=0.8))
        cut = np.array([r['phase'] == 'cut' for r in rows])
        edges = np.flatnonzero(np.diff(np.r_[0, cut.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[a], t[min(b, len(t) - 1)], color=fs.GREY, alpha=0.15, lw=0)
        ax.set_ylabel(fs.label('Joint velocity', 'm/s'))
    axes[0].set_xlim(0, 25)
    axes[0].set_ylim(-0.17, 0.3)
    axes[1].set_xlabel(fs.label('Time since the first arm point of the station', 's'))
    axes[1].legend(loc='center right', ncol=1)
    fs.panel_labels(axes, ['Point-to-point, 2 s per point', 'Continuous trajectory'])
    fs.save(fig, out)


def is_default(r):
    return r['mode'] == 'continuous' and all(r[k] == ARM_MOTION_PARAMS[k] for k in PARAM_KEYS)


def fmt_runs(rs, key, nd=1):
    """各次运行的取值，用 “ / ” 分隔（按运行名排序）."""
    return ' / '.join(f'{r[key]:.{nd}f}' if r.get(key) is not None else '—'
                      for r in sorted(rs, key=lambda r: r['run']))


def write_tables(summ, path):
    """报告正文的表：数值直接取自 runs_summary 的各行，避免手抄."""
    lines = []
    groups = [(plan, mode, [r for r in summ if r['plan'] == plan and r['all_ok']
                            and r['points'] == pts
                            and (r['mode'] == mode == 'point_to_point' or
                                 (mode == 'continuous' and is_default(r)))])
              for plan, pts in (('rule', 88), ('joint', 80))
              for mode in ('point_to_point', 'continuous')]
    head = ['指标'] + [f'{"规则路线" if p == "rule" else "联合规划"}，'
                     f'{"逐点" if m == "point_to_point" else "连续"}（n = {len(rs)}）'
                     for p, m, rs in groups]
    lines += ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    metrics = [('各站耗时之和（s）', 'stations_total_s', 1),
               ('其中：底盘到站（s）', 'base_move_s', 1),
               ('其中：`CutStation`（s）', 'cut_action_s', 1),
               ('机械臂切割段（s）', 'arm_cut_s', 1),
               ('机械臂其余运动（s）', 'arm_other_s', 1),
               ('刀尖轨迹误差，平均（mm）', 'path_err_mean_mm', 2),
               ('刀尖轨迹误差，p95（mm）', 'path_err_p95_mm', 2),
               ('刀尖轨迹误差，最大（mm）', 'path_err_max_mm', 2),
               ('切割段内横向偏差，RMS（mm）', 'cross_rms_mm', 3),
               ('切割段内横向偏差，p95（mm）', 'cross_p95_mm', 3),
               ('切割段内横向偏差，最大（mm）', 'cross_max_mm', 2),
               ('到站沿墙误差，平均（mm）', 'lateral_mm_mean', 2),
               ('到站沿墙误差，最大（mm）', 'lateral_mm_max', 2),
               ('机械臂运动期间底盘漂移，各站平均（mm）', 'arm_drift_mm_mean', 2),
               ('机械臂运动期间底盘漂移，最大（mm）', 'arm_drift_mm_max', 2),
               ('关节跟踪误差，直线轴最大（mm）', 'track_lin_max_mm', 2),
               ('关节跟踪误差，切割段内最大（mm）', 'track_cut_max_mm', 2),
               ('覆盖率（折线距离）', 'coverage_path', 3)]
    for name, key, nd in metrics:
        lines.append('| ' + ' | '.join([name] + [fmt_runs(rs, key, nd) for _, _, rs in groups])
                     + ' |')
    lines += ['', '| 配置 | 改动 | 运行数 | `CutStation` 之和（s） | 相对默认（s） | 横向偏差 '
              'p95 / 最大（mm） | 关节跟踪误差最大（mm） | 底盘漂移最大（mm） |',
              '|---|---|---|---|---|---|---|---|']
    joint = [r for r in summ if r['plan'] == 'joint' and r['points'] == 80 and r['all_ok']]
    ref = float(np.mean([r['cut_action_s'] for r in joint if is_default(r)] or [np.nan]))
    configs = {}
    for r in joint:
        name = ('p2p' if r['mode'] == 'point_to_point' else 'cont' if is_default(r)
                else r['run'].rsplit('_joint_', 1)[0])
        configs.setdefault(name, []).append(r)
    for name, rs in sorted(configs.items()):
        diff = {k: rs[0][k] for k in PARAM_KEYS if rs[0][k] != ARM_MOTION_PARAMS[k]}
        change = ('逐点执行' if rs[0]['mode'] == 'point_to_point' else
                  ', '.join(f'{k} = {v:g}' for k, v in diff.items()) or '默认')
        m = lambda k: float(np.mean([r[k] for r in rs if r.get(k) is not None]  # noqa: E731
                                    or [np.nan]))
        lines.append(f'| `{name}` | {change} | {len(rs)} | {m("cut_action_s"):.1f} | '
                     f'{m("cut_action_s") - ref:+.1f} | {m("cross_p95_mm"):.3f} / '
                     f'{max(r["cross_max_mm"] for r in rs):.2f} | {m("track_lin_max_mm"):.2f} | '
                     f'{max(r["arm_drift_mm_max"] for r in rs):.2f} |')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def load_benchmark(path):
    """task_planning_benchmark 的 results.csv → {方法: {种子: 行}}."""
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            out.setdefault(r['method'], {})[int(r['seed'])] = r
    return out


def planning_stats(res, tm):
    """
    配对墙（逐槽基线、现有方法、联合规划都可行，与 task_planning 报告的口径相同）上的统计.

    节省与时间构成取非承重墙（承重墙只能竖向开槽，没有选线余地）；相对逐槽基线的节省与
    下界证书取全部配对墙。
    """
    cur, joint, lb = res['current'], res['joint'], res.get('lower_bound', {})
    per = res.get('per_slot', {})
    paired = [k for k in cur if all(k in m and m[k]['feasible'] == '1'
                                    for m in (cur, joint, per))]
    seeds = [k for k in paired if cur[k]['load_bearing'] == '0']
    f = lambda rows, key: np.array([float(rows[k][key]) for k in seeds])  # noqa: E731
    t_cur, t_joint = f(cur, 't_exact'), f(joint, 't_exact')
    save = t_cur - t_joint
    pct = 100 * save / t_cur
    rng = np.random.default_rng(0)
    boot = [np.mean(rng.choice(pct, len(pct))) for _ in range(5000)]
    parts = {}
    for name, rows in (('current', cur), ('joint', joint)):
        parts[name] = dict(
            settle=float(np.mean(f(rows, 'stations'))) * tm.t_settle,
            strafe=float(np.mean(f(rows, 'travel_m'))) / tm.v_strafe,
            points=float(np.mean(f(rows, 'points'))) * tm.t_point,
            cutting=float(np.mean(f(rows, 'length_m'))) * tm.t_per_metre)
    base_pct = [100 * (float(per[k]['t_exact']) - float(joint[k]['t_exact']))
                / float(per[k]['t_exact']) for k in paired]
    cert = [k for k in paired if k in lb and lb[k]['feasible'] == '1']
    opt = [float(joint[k]['t_exact']) - float(lb[k]['t_exact']) < 0.05 for k in cert]
    stats = dict(paired_walls=len(paired), walls=len(seeds), mean_current_s=float(t_cur.mean()),
                 mean_joint_s=float(t_joint.mean()), mean_saving_s=float(save.mean()),
                 mean_saving_pct=float(pct.mean()), ci_lo_pct=float(np.percentile(boot, 2.5)),
                 ci_hi_pct=float(np.percentile(boot, 97.5)),
                 better=int((save > 1e-6).sum()), equal=int((np.abs(save) <= 1e-6).sum()),
                 worse=int((save < -1e-6).sum()),
                 mean_points_saved=float(np.mean(f(cur, 'points') - f(joint, 'points'))),
                 mean_stations_saved=float(np.mean(f(cur, 'stations') - f(joint, 'stations'))),
                 mean_length_added_m=float(np.mean(f(joint, 'length_m') - f(cur, 'length_m'))),
                 baseline_to_joint_pct=float(np.mean(base_pct)) if base_pct else float('nan'),
                 certified_optimal_pct=100 * float(np.mean(opt)) if opt else float('nan'))
    for name in ('current', 'joint'):
        stats.update({f'{name}_{k}_s': v for k, v in parts[name].items()})
    return stats, pct, parts


def plot_planning(out):
    import figstyle as fs
    sets = [('align_v3', 'Point-to-point model (align_v3)', 'baseline',
             os.path.join(HERE, '..', 'task_planning', 'align_v3', 'data', 'results.csv')),
            ('traj_v1', 'Continuous model (traj_v1)', 'proposed',
             os.path.join(HERE, 'benchmark', 'data', 'results.csv'))]
    if not all(os.path.exists(p) for *_, p in sets) or 'traj_v1' not in sp.TIME_MODELS:
        return None
    fig, (ax1, ax2) = fs.subplots(1, 2, width='double', height=2.9,
                                  gridspec_kw=dict(width_ratios=[1.15, 1]))
    comp = [('settle', 'Per-station overhead', 'neutral'), ('strafe', 'Strafing', 'alt2'),
            ('points', 'Arm: per-point term', 'baseline'),
            ('cutting', 'Arm: cut length / feed', 'proposed')]
    rows, x, ticks = [], 0, []
    for name, lab, role, path in sets:
        tm = sp.TimeModel.preset(name)
        stats, pct, parts = planning_stats(load_benchmark(path), tm)
        rows.append(dict(time_model=name, **stats))
        xs = np.sort(pct)
        ax2.step(xs, np.arange(1, len(xs) + 1) / len(xs), where='post',
                 label=f'{lab}: mean {stats["mean_saving_pct"]:.2f} %', **fs.line(role))
        for method in ('current', 'joint'):
            bottom = 0.0
            for key, clab, crole in comp:
                if parts[method][key] > 0.0:
                    ax1.bar(x, parts[method][key], 0.7, bottom=bottom, **fs.bar(crole))
                bottom += parts[method][key]
            ax1.text(x, bottom + 4, f'{bottom:.0f} s', ha='center', va='bottom', fontsize=7)
            ticks.append(('Rule-based\n+ DP' if method == 'current' else 'Joint\nplanning')
                         + f'\n{name}')
            x += 1
        x += 0.5
    for key, clab, crole in comp:
        ax1.bar(0, 0, label=clab, **fs.bar(crole))
    ax1.set_xticks([0, 1, 2.5, 3.5])
    ax1.set_xticklabels(ticks)
    ax1.set_ylabel(fs.label('Mean predicted time per wall', 's'))
    ax1.set_ylim(0, ax1.get_ylim()[1] * 1.3)
    ax1.legend(loc='upper right', ncol=2)
    ax2.set_xlabel(fs.label('Saving of joint planning over rule-based routes + DP', '%'))
    ax2.set_ylabel('Fraction of walls')
    ax2.set_ylim(0, 1.02)
    ax2.legend(loc='lower right')
    fs.panel_labels([ax1, ax2], ['Where the predicted time goes', 'Per-wall saving (ECDF)'])
    fs.save(fig, out)
    write_csv(os.path.join(HERE, 'planning_summary.csv'), rows)
    return rows


def main():
    dirs = sorted(d for d in glob.glob(os.path.join(RUNS, '*'))
                  if os.path.exists(os.path.join(d, 'summary.yaml')))
    dirs += [os.path.join(OLD_RUNS, r) for r in ('rule_s1', 'rule_s2', 'joint_s1', 'joint_s2')]
    summ, all_rows = [], []
    for d in dirs:
        res, rows = summarize(d)
        summ.append(res)
        all_rows += rows
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in res.items()})
    write_csv(os.path.join(HERE, 'runs_summary.csv'), summ)
    write_csv(os.path.join(HERE, 'stations_all.csv'), all_rows)
    write_tables(summ, os.path.join(HERE, 'summary_tables.md'))
    if any(r['mode'] == 'continuous' for r in all_rows):
        print('生成器时长 vs 实测：', generator_check(all_rows))
    tm = time_model(all_rows)
    if tm:
        write_csv(os.path.join(HERE, 'calibration.csv'), tm)
        for r in tm:
            print(r)
    lim = MotionLimits()
    print('解析推导的每点耗时（空切，下刀行程 10.6 mm）：', analytic_t_point(lim, 0.0106))
    plot_time(summ, os.path.join(HERE, 'fig_time.png'))
    plot_tradeoff(summ, os.path.join(HERE, 'fig_tradeoff.png'))
    plot_time_model(all_rows, os.path.join(HERE, 'fig_time_model.png'))
    plot_profile(os.path.join(HERE, 'fig_profile.png'))
    for r in plot_planning(os.path.join(HERE, 'fig_planning.png')) or []:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
