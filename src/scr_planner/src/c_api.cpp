// Copyright 2026 SCR Project

#include "scr_planner/c_api.h"

#include "scr_planner/oriented_astar.hpp"

namespace
{
scr_planner::SearchParams toParams(const scr_params_t & c)
{
  scr_planner::SearchParams p;
  p.turn_penalty = c.turn_penalty;
  p.weight_x = c.weight_x;
  p.weight_y = c.weight_y;
  p.cost_weight = c.cost_weight;
  p.goal_tolerance_cells = c.goal_tolerance_cells;
  p.timeout_s = c.timeout_s;
  p.allow_x = c.allow_x != 0;
  p.allow_y = c.allow_y != 0;
  p.max_run_x = c.max_run_x;
  p.max_run_y = c.max_run_y;
  p.bend_radius = c.bend_radius;
  p.allow_unknown = c.allow_unknown != 0;
  p.obstacle_threshold = static_cast<uint8_t>(c.obstacle_threshold);
  p.max_expansions = c.max_expansions > 0 ? static_cast<size_t>(c.max_expansions) : 0;
  return p;
}
}  // namespace

extern "C" {

void scr_default_params(scr_params_t * c)
{
  const scr_planner::SearchParams p;
  c->turn_penalty = p.turn_penalty;
  c->weight_x = p.weight_x;
  c->weight_y = p.weight_y;
  c->cost_weight = p.cost_weight;
  c->goal_tolerance_cells = p.goal_tolerance_cells;
  c->timeout_s = p.timeout_s;
  c->allow_x = p.allow_x;
  c->allow_y = p.allow_y;
  c->max_run_x = p.max_run_x;
  c->max_run_y = p.max_run_y;
  c->bend_radius = p.bend_radius;
  c->allow_unknown = p.allow_unknown;
  c->obstacle_threshold = p.obstacle_threshold;
  c->max_expansions = static_cast<int64_t>(p.max_expansions);
}

void * scr_planner_create(void) {return new scr_planner::OrientedAStar();}

void scr_planner_destroy(void * planner)
{
  delete static_cast<scr_planner::OrientedAStar *>(planner);
}

int32_t scr_planner_plan(
  void * planner, const uint8_t * grid, int32_t width, int32_t height,
  int32_t sx, int32_t sy, int32_t gx, int32_t gy,
  const scr_params_t * params, scr_result_t * result,
  int32_t * corners_xy, int32_t max_corners)
{
  auto * pl = static_cast<scr_planner::OrientedAStar *>(planner);
  scr_planner::GridView view{static_cast<unsigned int>(width),
    static_cast<unsigned int>(height), grid};
  const auto r = pl->plan(view, {sx, sy}, {gx, gy}, toParams(*params));
  result->status = static_cast<int32_t>(r.status);
  result->turns = r.turns;
  result->n_corners = static_cast<int32_t>(r.corners.size());
  result->goal_x = r.goal.x;
  result->goal_y = r.goal.y;
  result->cost = r.cost;
  result->expansions = static_cast<int64_t>(r.expansions);
  result->time_ms = r.time_ms;
  for (int32_t i = 0; i < result->n_corners && i < max_corners; ++i) {
    corners_xy[2 * i] = r.corners[i].x;
    corners_xy[2 * i + 1] = r.corners[i].y;
  }
  return result->status;
}

}  // extern "C"
