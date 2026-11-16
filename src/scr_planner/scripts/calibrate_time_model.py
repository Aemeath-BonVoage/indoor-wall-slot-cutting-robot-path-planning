#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 Gazebo 执行记录标定时间模型：每站耗时 = t_settle + 横移距离 / v_strafe + 点数 × t_point.

  python3 src/scr_planner/scripts/calibrate_time_model.py <run_dir> [<run_dir> ...]
"""
import csv
import os
import sys

import numpy as np


def main():
    rows = []
    for d in sys.argv[1:]:
        rows += list(csv.DictReader(open(os.path.join(d, 'stations.csv'))))
    A = np.array([[1.0, float(r['move_dist_m']), float(r['points'])] for r in rows])
    y = np.array([float(r['station_s']) for r in rows])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    rmse = float(np.sqrt(np.mean((pred - y) ** 2)))
    t_settle, k_dist, t_point = coef
    v = 1.0 / k_dist if k_dist > 1e-9 else float('inf')
    print(f'stations: {len(rows)}')
    print(f't_settle = {t_settle:.2f} s, v_strafe = {v:.4f} m/s (1/v = {k_dist:.1f} s/m), '
          f't_point = {t_point:.3f} s, RMSE = {rmse:.2f} s, '
          f'R2 = {1 - np.sum((pred - y) ** 2) / np.sum((y - y.mean()) ** 2):.3f}')


if __name__ == '__main__':
    main()
