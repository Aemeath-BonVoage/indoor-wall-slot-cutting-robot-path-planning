"""
批量离线对比实验：PBVS / IBVS / 2.5D 混合.

用法::

    python3 run_experiments.py                 # 全部实验 + 作图（约 20 分钟，2 进程）
    python3 run_experiments.py --only E1,E3    # 只跑部分实验
    python3 run_experiments.py --plot-only     # 只用已有 CSV 重新作图 / 汇总

输出（本目录）：trials_<E>.csv（逐次试验）、summary.csv（汇总）、results.md（表格）、fig_*.png。
所有随机量固定种子，同一实验内各方法使用同一组起点（配对比较）。
"""
import argparse
import csv
import math
from multiprocessing import Pool
import os

import numpy as np

import sim
from sim import Scenario

HERE = os.path.dirname(os.path.abspath(__file__))
METHODS = ['PBVS', 'IBVS', 'Hybrid']
DEG = math.pi / 180

BASE = dict(pixel_noise=0.5, range_noise=0.01)     # “基准噪声”：亚像素角点 + Gazebo 原生激光噪声


def jobs_e0():
    """E0：IBVS 雅可比深度方式消融."""
    starts = sim.random_starts(100)
    out = []
    for nl, nz in [('无噪声', {}), ('基准噪声', BASE), ('2px/2cm', dict(pixel_noise=2,
                                                                     range_noise=0.02))]:
        for dm in ['desired', 'laser', 'size', 'mean']:
            sc = Scenario(depth_mode=dm, **nz)
            for mode, hz in [('stop', None), ('steady', 30.0)]:
                for i, s in enumerate(starts):
                    out.append(('E0', f'{nl}|{dm}|{mode}', 'IBVS', sc, s, i, hz))
    return out


E1_CONDS = [('无噪声', {}), ('基准 0.5px/1cm', BASE),
            ('2px/2cm', dict(pixel_noise=2, range_noise=0.02)),
            ('5px/5cm', dict(pixel_noise=5, range_noise=0.05))]


def jobs_e1():
    """E1：主对比（Gazebo 同分布起点），停止模式与稳态模式."""
    starts = sim.random_starts(100)
    out = []
    for label, kw in E1_CONDS:
        for m in METHODS:
            for mode, hz in [('stop', None), ('steady', 30.0)]:
                for i, s in enumerate(starts):
                    out.append(('E1', f'{label}|{mode}', m, Scenario(**kw), s, i, hz))
    return out


PIX_LEVELS = [0.0, 0.25, 0.5, 1.0, 2.0, 5.0]
RANGE_LEVELS = [0.0, 0.01, 0.02, 0.05, 0.1]


def jobs_e2():
    """E2：噪声扫描（稳态模式衡量精度下限）."""
    starts = sim.random_starts(50)
    out = []
    for p in PIX_LEVELS:
        for m in METHODS:
            for i, s in enumerate(starts):
                out.append(('E2', f'pix|{p}', m, Scenario(pixel_noise=p, range_noise=0.01),
                            s, i, 30.0))
    for r in RANGE_LEVELS:
        for m in METHODS:
            for i, s in enumerate(starts):
                out.append(('E2', f'range|{r}', m, Scenario(pixel_noise=0.5, range_noise=r),
                            s, i, 30.0))
    return out


CALIB = [
    ('cam_yaw', '相机偏航 (°)', [-2, -1, -0.5, 0.5, 1, 2], DEG),
    ('cam_pitch', '相机俯仰 (°)', [-2, -1, 1, 2], DEG),
    ('cam_roll', '相机横滚 (°)', [-2, -1, 1, 2], DEG),
    ('cam_dy', '相机横向位置 (mm)', [-10, -5, 5, 10], 1e-3),
    ('cam_dx', '相机前后位置 (mm)', [-10, 10], 1e-3),
    ('lidar_yaw', '激光偏航 (°)', [-1, -0.5, 0.5, 1], DEG),
    ('lidar_dx', '激光前后位置 (mm)', [-10, 10], 1e-3),
    ('fx_scale', '焦距误差 (%)', [-3, -1, 1, 3], None),
    ('cx_err', '主点 cx 误差 (px)', [-5, -2, 2, 5], 1.0),
    ('cy_err', '主点 cy 误差 (px)', [-5, 5], 1.0),
]


def calib_kw(key, level, unit):
    if key == 'fx_scale':
        return {key: 1.0 + level / 100.0}
    return {key: level * unit}


def jobs_e3():
    """E3：标定误差敏感性（无噪声，稳态模式 → 纯偏差），模型期望 vs 示教期望."""
    starts = sim.random_starts(10)
    out = []
    for key, _, levels, unit in CALIB:
        for lv in levels:
            for ref in ['model', 'taught']:
                sc = Scenario(reference=ref, **calib_kw(key, lv, unit))
                for m in METHODS:
                    for i, s in enumerate(starts):
                        out.append(('E3', f'{key}|{lv}|{ref}', m, sc, s, i, 30.0))
    return out


def jobs_e4():
    """E4：大范围起点（视野考验）."""
    starts = sim.random_starts(100, seed=7, dist=(0.8, 2.0), lateral=0.8, yaw_deg=35.0)
    out = []
    for label, kw in [('无噪声', {}), ('基准噪声', BASE)]:
        for m in METHODS:
            for i, s in enumerate(starts):
                out.append(('E4', label, m, Scenario(**kw), s, i, None))
    return out


def jobs_e5():
    """E5：检测丢帧."""
    starts = sim.random_starts(100)
    out = []
    for p in [0.0, 0.1, 0.3, 0.5]:
        for m in METHODS:
            for i, s in enumerate(starts):
                out.append(('E5', f'{p}', m, Scenario(dropout=p, **BASE), s, i, None))
    return out


def jobs_e6():
    """E6：运行时方块高度与示教 / 标称不一致."""
    starts = sim.random_starts(10)
    out = []
    for dz in [-30, -10, 10, 30]:
        for ref in ['model', 'taught']:
            sc = Scenario(marker_dz=dz * 1e-3, reference=ref)
            for m in METHODS:
                for i, s in enumerate(starts):
                    out.append(('E6', f'{dz}|{ref}', m, sc, s, i, 30.0))
    return out


def jobs_e7():
    """E7：方块尺寸对 IBVS 的影响（弱可观方向随尺寸改善）."""
    starts = sim.random_starts(50)
    out = []
    for size in [0.05, 0.10, 0.20]:
        for nl, kw in [('基准噪声', BASE), ('2px/2cm', dict(pixel_noise=2, range_noise=0.02))]:
            sc = Scenario(marker_size=size, **kw)
            for mode, hz in [('stop', None), ('steady', 30.0)]:
                for m in METHODS:
                    for i, s in enumerate(starts):
                        out.append(('E7', f'{size}|{nl}|{mode}', m, sc, s, i, hz))
    return out


EXPERIMENTS = {'E0': jobs_e0, 'E1': jobs_e1, 'E2': jobs_e2, 'E3': jobs_e3, 'E4': jobs_e4,
               'E5': jobs_e5, 'E6': jobs_e6, 'E7': jobs_e7}
FIELDS = ['exp', 'cond', 'method', 'trial', 'status', 'success', 'time', 'settle_time',
          'pos_err', 'normal', 'lateral', 'yaw', 'tip_lateral', 'min_fov_margin',
          'fov_drop', 'min_clearance', 'path', 'fov_lost']


def _run(job):
    exp, cond, method, sc, start, i, hz = job
    r = sim.run_trial(method, sc, start, seed=1000 + i, horizon=hz)
    r.update(exp=exp, cond=cond, trial=i)
    return {k: r[k] for k in FIELDS}


def run(exp, workers):
    jobs = EXPERIMENTS[exp]()
    print(f'{exp}: {len(jobs)} 次试验', flush=True)
    with Pool(workers) as pool:
        rows = pool.map(_run, jobs, chunksize=20)
    path = os.path.join(HERE, f'trials_{exp}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.6g}' if isinstance(v, float) else v) for k, v in r.items()})
    return rows


def load(exp):
    path = os.path.join(HERE, f'trials_{exp}.csv')
    if not os.path.exists(path):
        return []
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            for k in FIELDS[4:]:
                if k == 'status':
                    continue
                if k in ('success', 'fov_lost'):
                    r[k] = r[k] == 'True'
                else:
                    r[k] = float(r[k])
            rows.append(r)
    return rows


# ------------------------------------------------------------------ 汇总
def summarize(rows):
    """按 (exp, cond, method) 聚合."""
    groups = {}
    for r in rows:
        groups.setdefault((r['exp'], r['cond'], r['method']), []).append(r)
    out = []
    for (exp, cond, method), g in groups.items():
        pos = np.array([r['pos_err'] for r in g]) * 1000
        yaw = np.abs([r['yaw'] for r in g]) / DEG
        out.append({
            'exp': exp, 'cond': cond, 'method': method, 'n': len(g),
            'success': sum(r['success'] for r in g),
            'converged': sum(r['status'] in ('converged', 'horizon') for r in g),
            'timeout': sum(r['status'] == 'timeout' for r in g),
            'lost': sum(r['status'].startswith('lost') for r in g),
            'fov_lost': sum(r['fov_lost'] for r in g),
            'pos_mean': pos.mean(), 'pos_std': pos.std(), 'pos_max': pos.max(),
            'pos_p95': np.percentile(pos, 95),
            'normal_mean': np.mean([r['normal'] for r in g]) * 1000,
            'lateral_mean': np.mean([r['lateral'] for r in g]) * 1000,
            'yaw_mean': yaw.mean(), 'yaw_max': yaw.max(),
            'yaw_signed': np.mean([r['yaw'] for r in g]) / DEG,
            'tip_mean': np.mean(np.abs([r['tip_lateral'] for r in g])) * 1000,
            'time_mean': np.mean([r['time'] for r in g]),
            'settle_mean': np.mean([r['settle_time'] for r in g]),
            'min_clear': min(r['min_clearance'] for r in g) * 1000,
            'min_margin': min(r['min_fov_margin'] for r in g),
            'fov_drop_max': max(r['fov_drop'] for r in g),
            'fov_drop_mean': np.mean([r['fov_drop'] for r in g]),
            'path_mean': np.mean([r['path'] for r in g]),
        })
    return out


def write_summary(summ):
    keys = list(summ[0].keys())
    with open(os.path.join(HERE, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for s in summ:
            w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in s.items()})


def md_tables(summ):
    """生成 results.md：各实验的 Markdown 表格."""
    idx = {(s['exp'], s['cond'], s['method']): s for s in summ}
    lines = ['# 视觉伺服方法对比：完整结果表', '',
             '由 `run_experiments.py` 生成，说明见 [README.md](README.md)。', '']

    def row(s, label):
        return (f"| {label} | {s['success']}/{s['n']} | "
                f"{s['pos_mean']:.1f} ± {s['pos_std']:.1f} ({s['pos_max']:.1f}) | "
                f"{s['yaw_mean']:.2f} ({s['yaw_max']:.2f}) | {s['tip_mean']:.1f} | "
                f"{s['time_mean']:.1f} | {s['settle_mean']:.1f} |")

    head = ('| 条件 / 方法 | 成功 | 位置误差 mean ± std (max) mm | 偏航 mean (max) ° | '
            '刀尖横向 mm | 停止时间 s | 真值进入容差 s |\n|---|---|---|---|---|---|---|')
    for exp, title, conds, methods in [
        ('E1', 'E1 主对比——停止模式', [f'{c}|stop' for c, _ in E1_CONDS], METHODS),
        ('E1', 'E1 主对比——稳态模式（30 s，最后 5 s RMS）',
         [f'{c}|steady' for c, _ in E1_CONDS], METHODS),
    ]:
        lines += [f'### {title}', '', head]
        for c in conds:
            for m in methods:
                s = idx.get((exp, c, m))
                if s:
                    lines.append(row(s, f"{c.split('|')[0]} / {m}"))
        lines.append('')

    lines += ['### E0 IBVS 深度方式消融', '', head]
    for nl in ['无噪声', '基准噪声', '2px/2cm']:
        for mode in ['stop', 'steady']:
            for dm in ['desired', 'laser', 'size', 'mean']:
                s = idx.get(('E0', f'{nl}|{dm}|{mode}', 'IBVS'))
                if s:
                    lines.append(row(s, f'{nl} / {mode} / {dm}'))
    lines.append('')

    lines += ['### E2 噪声扫描（稳态）', '',
              '| 扫描量 | 水平 | ' + ' | '.join(f'{m} 位置 mm / 偏航 °' for m in METHODS) + ' |',
              '|---|---|' + '---|' * len(METHODS)]
    for kind, levels in [('pix', PIX_LEVELS), ('range', RANGE_LEVELS)]:
        for lv in levels:
            cells = []
            for m in METHODS:
                s = idx.get(('E2', f'{kind}|{lv}', m))
                cells.append(f"{s['pos_mean']:.1f} / {s['yaw_mean']:.2f}" if s else '—')
            name = '像素 σ (px)' if kind == 'pix' else '测距 σ (m)'
            lines.append(f'| {name} | {lv} | ' + ' | '.join(cells) + ' |')
    lines.append('')

    lines += ['### E3 标定误差敏感性（无噪声，稳态偏差）', '',
              '每格：位置误差 mm / 偏航 °（10 个起点平均）。', '',
              '| 误差项 | 水平 | ' + ' | '.join(f'{m} 模型' for m in METHODS) + ' | '
              + ' | '.join(f'{m} 示教' for m in METHODS) + ' |',
              '|---|---|' + '---|' * 6]
    for key, label, levels, _ in CALIB:
        for lv in levels:
            cells = []
            for ref in ['model', 'taught']:
                for m in METHODS:
                    s = idx.get(('E3', f'{key}|{lv}|{ref}', m))
                    cells.append(f"{s['pos_mean']:.1f} / {s['yaw_mean']:.2f}" if s else '—')
            lines.append(f'| {label} | {lv} | ' + ' | '.join(cells) + ' |')
    lines.append('')

    head4 = ('| 条件 / 方法 | 成功 | 超时 | 丢失（出视野） | 位置误差 mean (max) mm | 偏航 mean ° | '
             '停止时间 s | 最小视野余量 px | 余量下降 mean / max px | 最小刀具间隙 mm |\n|---|---|---|---|---|---|---|---|---|---|')
    for exp, title, conds in [('E4', 'E4 大范围起点（离墙 0.8–2 m，横向 ±0.8 m，朝向 ±35°，起始可见）',
                               ['无噪声', '基准噪声']),
                              ('E5', 'E5 检测丢帧（基准噪声）', ['0.0', '0.1', '0.3', '0.5'])]:
        lines += [f'### {title}', '', head4]
        for c in conds:
            for m in METHODS:
                s = idx.get((exp, c, m))
                if s:
                    lines.append(
                        f"| {c} / {m} | {s['success']}/{s['n']} | {s['timeout']} | "
                        f"{s['lost']}（{s['fov_lost']}） | {s['pos_mean']:.1f} ({s['pos_max']:.1f}) | "
                        f"{s['yaw_mean']:.2f} | {s['time_mean']:.1f} | {s['min_margin']:.0f} | "
                        f"{s['fov_drop_mean']:.0f} / {s['fov_drop_max']:.0f} | "
                        f"{s['min_clear']:.1f} |")
        lines.append('')

    lines += ['### E6 定位标记高度与参考不一致（无噪声，稳态）', '',
              '每格：法向误差 / 横向误差 mm，偏航 °（10 个起点平均，带符号）。', '',
              '| Δh (mm) | 参考 | ' + ' | '.join(METHODS) + ' |', '|---|---|---|---|---|']
    for dz in [-30, -10, 10, 30]:
        for ref in ['model', 'taught']:
            cells = []
            for m in METHODS:
                s = idx.get(('E6', f'{dz}|{ref}', m))
                cells.append(f"{s['normal_mean']:+.1f} / {s['lateral_mean']:+.1f}, "
                             f"{s['yaw_signed']:+.2f}°" if s else '—')
            lines.append(f'| {dz:+d} | {ref} | ' + ' | '.join(cells) + ' |')
    lines.append('')

    lines += ['### E7 定位标记尺寸', '', head]
    for size in ['0.05', '0.1', '0.2']:
        for nl in ['基准噪声', '2px/2cm']:
            for mode in ['stop', 'steady']:
                for m in METHODS:
                    s = idx.get(('E7', f'{size}|{nl}|{mode}', m))
                    if s:
                        lines.append(row(s, f'{float(size)*100:.0f} cm / {nl} / {mode} / {m}'))
    lines.append('')
    with open(os.path.join(HERE, 'results.md'), 'w') as f:
        f.write('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default=','.join(EXPERIMENTS))
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--plot-only', action='store_true')
    a = ap.parse_args()
    if not a.plot_only:
        for exp in a.only.split(','):
            run(exp, a.jobs)
    rows = []
    for exp in EXPERIMENTS:
        rows += load(exp)
    if rows:
        summ = summarize(rows)
        write_summary(summ)
        md_tables(summ)
    import plots
    plots.make_all(rows)


if __name__ == '__main__':
    main()

