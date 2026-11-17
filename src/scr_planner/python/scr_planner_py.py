# -*- coding: utf-8 -*-
"""ctypes wrapper of the C++ OrientedAStar (libscr_planner_core.so).

    from scr_planner_py import OrientedAStar
    planner = OrientedAStar()                     # finds the library itself
    res = planner.plan(grid_u8, (sx, sy), (gx, gy), weight_x=2.0, allow_x=True,
                       max_run_x=50, bend_radius=12)
    res['corners']  # [(x, y), ...]  start, bends, goal

grid_u8: 2-D numpy uint8 array indexed [y, x]; 0 = free, >= 253 = blocked.
"""
import ctypes
import glob
import os

import numpy as np

STATUS = {0: 'success', 1: 'start outside grid', 2: 'goal outside grid',
          3: 'goal in obstacle', 4: 'no path', 5: 'timeout'}


class _Params(ctypes.Structure):
    _fields_ = [('turn_penalty', ctypes.c_double), ('weight_x', ctypes.c_double),
                ('weight_y', ctypes.c_double), ('cost_weight', ctypes.c_double),
                ('goal_tolerance_cells', ctypes.c_double), ('timeout_s', ctypes.c_double),
                ('allow_x', ctypes.c_int32), ('allow_y', ctypes.c_int32),
                ('max_run_x', ctypes.c_int32), ('max_run_y', ctypes.c_int32),
                ('bend_radius', ctypes.c_int32), ('allow_unknown', ctypes.c_int32),
                ('obstacle_threshold', ctypes.c_int32), ('max_expansions', ctypes.c_int64)]


class _Result(ctypes.Structure):
    _fields_ = [('status', ctypes.c_int32), ('turns', ctypes.c_int32),
                ('n_corners', ctypes.c_int32), ('goal_x', ctypes.c_int32),
                ('goal_y', ctypes.c_int32), ('cost', ctypes.c_double),
                ('expansions', ctypes.c_int64), ('time_ms', ctypes.c_double)]


def _find_library():
    env = os.environ.get('SCR_PLANNER_LIB')
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    # installed: <prefix>/share/scr_planner/python -> <prefix>/lib
    candidates = [os.path.join(here, '..', '..', '..', 'lib', 'libscr_planner_core.so')]
    repo = os.path.abspath(os.path.join(here, '..', '..', '..'))
    candidates += glob.glob(os.path.join(repo, 'build', 'scr_planner', 'libscr_planner_core.so'))
    candidates += glob.glob(os.path.join(repo, 'install', 'scr_planner', 'lib',
                                         'libscr_planner_core.so'))
    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)
    raise FileNotFoundError('libscr_planner_core.so not found; build scr_planner or set '
                            'SCR_PLANNER_LIB')


class OrientedAStar:
    def __init__(self, lib_path=None):
        self._lib = ctypes.CDLL(lib_path or _find_library())
        self._lib.scr_planner_create.restype = ctypes.c_void_p
        self._lib.scr_planner_destroy.argtypes = [ctypes.c_void_p]
        self._lib.scr_default_params.argtypes = [ctypes.POINTER(_Params)]
        self._lib.scr_planner_plan.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int32, ctypes.c_int32,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32,
            ctypes.POINTER(_Params), ctypes.POINTER(_Result),
            ctypes.POINTER(ctypes.c_int32), ctypes.c_int32]
        self._lib.scr_planner_plan.restype = ctypes.c_int32
        self._handle = self._lib.scr_planner_create()

    def __del__(self):
        if getattr(self, '_handle', None):
            self._lib.scr_planner_destroy(self._handle)
            self._handle = None

    def plan(self, grid, start, goal, **params):
        grid = np.ascontiguousarray(grid, dtype=np.uint8)
        p = _Params()
        self._lib.scr_default_params(ctypes.byref(p))
        for k, v in params.items():
            if not hasattr(p, k):
                raise KeyError(f'unknown parameter {k}')
            setattr(p, k, int(v) if isinstance(getattr(p, k), int) else float(v))
        res = _Result()
        max_corners = 4096
        buf = (ctypes.c_int32 * (2 * max_corners))()
        h, w = grid.shape
        self._lib.scr_planner_plan(
            self._handle, grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), w, h,
            int(start[0]), int(start[1]), int(goal[0]), int(goal[1]),
            ctypes.byref(p), ctypes.byref(res), buf, max_corners)
        n = min(res.n_corners, max_corners)
        return dict(status=STATUS.get(res.status, str(res.status)), ok=res.status == 0,
                    corners=[(buf[2 * i], buf[2 * i + 1]) for i in range(n)],
                    turns=res.turns, cost=res.cost, expansions=res.expansions,
                    time_ms=res.time_ms, goal=(res.goal_x, res.goal_y))
