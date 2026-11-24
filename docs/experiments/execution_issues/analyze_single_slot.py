#!/usr/bin/env python3
"""
单条槽执行精度的复测汇总（第 7 节；结果写入 slot_execution 报告，不需要 Gazebo）.

  python3 analyze_single_slot.py

对比 slot_execution/runs/single_slot（早期仿真配置上的一次运行）与本节 runs/single_slot_s*
（当前默认配置，种子 1–3），指标取实验记录器 summary.yaml 中的同名字段。
输出 single_slot_summary.csv。
"""
import csv
import glob
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
OLD = os.path.join(HERE, '..', 'slot_execution', 'runs', 'single_slot')
KEYS = ['path_err_mean_mm', 'path_err_p95_mm', 'path_err_max_mm', 'coverage', 'coverage_path',
        'station_lateral_mm_mean_abs', 'station_lateral_mm_max_abs',
        'station_standoff_mm_mean_abs', 'station_standoff_mm_max_abs',
        'station_yaw_deg_mean_abs', 'station_yaw_deg_max_abs',
        'station_base_drift_mm_mean_abs', 'station_base_drift_mm_max_abs',
        'stations', 'stations_done', 'stations_total_s', 'all_ok']


def load(d):
    with open(os.path.join(d, 'summary.yaml'), encoding='utf-8') as f:
        s = yaml.safe_load(f)
    return {k: s.get(k, '') for k in KEYS}


def main():
    rows = [dict(run='slot_execution/single_slot', config='early', **load(OLD))]
    new = sorted(d for d in glob.glob(os.path.join(HERE, 'runs', 'single_slot_s*'))
                 if os.path.exists(os.path.join(d, 'summary.yaml')))
    for d in new:
        rows.append(dict(run=os.path.basename(d), config='current', **load(d)))
    cur = [r for r in rows if r['config'] == 'current']
    if cur:
        mean = dict(run='current mean', config='current')
        for k in KEYS:
            v = [r[k] for r in cur
                 if isinstance(r[k], (int, float)) and not isinstance(r[k], bool)]
            mean[k] = round(float(np.mean(v)), 4) if v else ''
        rows.append(mean)
    with open(os.path.join(HERE, 'single_slot_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    for r in rows:
        print(r)


if __name__ == '__main__':
    main()
