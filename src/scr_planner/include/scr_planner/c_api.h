/* Copyright 2026 SCR Project
 *
 * Plain C interface to OrientedAStar, for Python (ctypes) and other callers.
 * See python/scr_planner_py.py for the Python wrapper.
 */
#ifndef SCR_PLANNER__C_API_H_
#define SCR_PLANNER__C_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  double turn_penalty;
  double weight_x;
  double weight_y;
  double cost_weight;
  double goal_tolerance_cells;
  double timeout_s;
  int32_t allow_x;
  int32_t allow_y;
  int32_t max_run_x;
  int32_t max_run_y;
  int32_t bend_radius;
  int32_t allow_unknown;
  int32_t obstacle_threshold;
  int64_t max_expansions;
} scr_params_t;

typedef struct
{
  int32_t status;      /* 0 = success, see scr_planner::Status */
  int32_t turns;
  int32_t n_corners;   /* may exceed max_corners; only max_corners are written */
  int32_t goal_x;
  int32_t goal_y;
  double cost;
  int64_t expansions;
  double time_ms;
} scr_result_t;

void scr_default_params(scr_params_t * params);
void * scr_planner_create(void);
void scr_planner_destroy(void * planner);
/* grid: row-major uint8 costs, index = y * width + x.
 * corners_xy: output buffer of 2 * max_corners ints (x0, y0, x1, y1, ...). */
int32_t scr_planner_plan(
  void * planner, const uint8_t * grid, int32_t width, int32_t height,
  int32_t sx, int32_t sy, int32_t gx, int32_t gy,
  const scr_params_t * params, scr_result_t * result,
  int32_t * corners_xy, int32_t max_corners);

#ifdef __cplusplus
}
#endif

#endif  /* SCR_PLANNER__C_API_H_ */
