// Copyright 2026 SCR Project

#include "scr_planner/oriented_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>

namespace scr_planner
{

namespace
{

// Headings: 0 = +x, 1 = +y, 2 = -x, 3 = -y.
constexpr int kDx[4] = {1, 0, -1, 0};
constexpr int kDy[4] = {0, 1, 0, -1};
constexpr int kNumHeadings = 4;

struct QueueEntry
{
  double f;
  double g;
  int32_t state;
};

struct QueueCompare
{
  // std::priority_queue is a max-heap: return true if a has LOWER priority.
  bool operator()(const QueueEntry & a, const QueueEntry & b) const
  {
    if (a.f != b.f) {return a.f > b.f;}
    return a.g < b.g;  // tie-break towards deeper nodes
  }
};

/// Lower bound on the number of heading changes needed to cover the offset
/// (dx, dy) when currently travelling along `heading`.
inline int minTurns(int heading, int dx, int dy)
{
  const int need_x = dx > 0 ? 0 : (dx < 0 ? 2 : -1);
  const int need_y = dy > 0 ? 1 : (dy < 0 ? 3 : -1);
  const int n = (need_x >= 0) + (need_y >= 0);
  if (n == 0) {return 0;}
  const bool aligned = heading == need_x || heading == need_y;
  return aligned ? n - 1 : n;
}

inline int headingOf(const Cell & from, const Cell & to)
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (dx > 0) {return 0;}
  if (dy > 0) {return 1;}
  if (dx < 0) {return 2;}
  return 3;
}

}  // namespace

std::string toString(Status s)
{
  switch (s) {
    case Status::kSuccess: return "success";
    case Status::kStartOutOfBounds: return "start outside map";
    case Status::kGoalOutOfBounds: return "goal outside map";
    case Status::kGoalBlocked: return "goal in obstacle";
    case Status::kNoPath: return "no path";
    case Status::kTimeout: return "timeout";
  }
  return "unknown";
}

bool OrientedAStar::traversable(
  const GridView & grid, int x, int y, const SearchParams & p) const
{
  if (!grid.inBounds(x, y)) {return false;}
  const uint8_t c = grid.at(x, y);
  if (c == cost::kUnknown) {return p.allow_unknown;}
  return c < p.obstacle_threshold;
}

bool OrientedAStar::findNearestFree(
  const GridView & grid, Cell goal, const SearchParams & p, Cell & out) const
{
  const int r = static_cast<int>(std::floor(p.goal_tolerance_cells));
  const double r2 = p.goal_tolerance_cells * p.goal_tolerance_cells;
  double best = std::numeric_limits<double>::infinity();
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const double d2 = dx * dx + dy * dy;
      if (d2 > r2 || d2 >= best) {continue;}
      if (traversable(grid, goal.x + dx, goal.y + dy, p)) {
        best = d2;
        out = Cell{goal.x + dx, goal.y + dy};
      }
    }
  }
  return std::isfinite(best);
}

SearchResult OrientedAStar::plan(
  const GridView & grid, Cell start, Cell goal, const SearchParams & p)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  SearchResult res;
  res.goal = goal;
  auto finish = [&](Status s) {
      res.status = s;
      res.time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
      return res;
    };

  if (!grid.inBounds(start.x, start.y)) {return finish(Status::kStartOutOfBounds);}
  if (!grid.inBounds(goal.x, goal.y)) {return finish(Status::kGoalOutOfBounds);}

  if (start == goal) {
    res.cells = {start};
    res.corners = {start};
    return finish(Status::kSuccess);
  }

  if (!traversable(grid, goal.x, goal.y, p)) {
    Cell adjusted;
    if (p.goal_tolerance_cells <= 0.0 || !findNearestFree(grid, goal, p, adjusted)) {
      return finish(Status::kGoalBlocked);
    }
    goal = adjusted;
    res.goal = goal;
    if (start == goal) {
      res.cells = {start};
      res.corners = {start};
      return finish(Status::kSuccess);
    }
  }

  // --- buffers ---------------------------------------------------------------
  const size_t n_states = static_cast<size_t>(grid.width) * grid.height * kNumHeadings;
  if (g_.size() != n_states) {
    g_.assign(n_states, 0.0);
    parent_.assign(n_states, -1);
    visit_stamp_.assign(n_states, 0);
    closed_stamp_.assign(n_states, 0);
    stamp_ = 0;
  }
  if (++stamp_ == 0) {  // wrapped around: hard reset
    std::fill(visit_stamp_.begin(), visit_stamp_.end(), 0);
    std::fill(closed_stamp_.begin(), closed_stamp_.end(), 0);
    stamp_ = 1;
  }

  const int width = static_cast<int>(grid.width);
  const double tp = p.turn_penalty;
  const double cw = std::max(0.0, p.cost_weight);
  auto stateOf = [&](int x, int y, int h) {return (y * width + x) * kNumHeadings + h;};
  auto heuristic = [&](int x, int y, int h) {
      const int dx = goal.x - x;
      const int dy = goal.y - y;
      return static_cast<double>(std::abs(dx) + std::abs(dy)) + tp * minTurns(h, dx, dy);
    };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
  // The robot has no preferred heading at the start: seed all four.
  for (int h = 0; h < kNumHeadings; ++h) {
    const int32_t s = stateOf(start.x, start.y, h);
    g_[s] = 0.0;
    parent_[s] = -1;
    visit_stamp_[s] = stamp_;
    open.push({heuristic(start.x, start.y, h), 0.0, s});
  }

  int32_t goal_state = -1;
  size_t expansions = 0;
  while (!open.empty()) {
    const QueueEntry top = open.top();
    open.pop();
    const int32_t s = top.state;
    if (closed_stamp_[s] == stamp_) {continue;}
    if (top.g > g_[s]) {continue;}  // stale entry
    closed_stamp_[s] = stamp_;
    ++expansions;

    const int cell = s / kNumHeadings;
    const int h = s % kNumHeadings;
    const int x = cell % width;
    const int y = cell / width;
    if (x == goal.x && y == goal.y) {
      goal_state = s;
      break;
    }

    if ((expansions & 0xFFF) == 0) {
      if (p.max_expansions > 0 && expansions >= p.max_expansions) {
        res.expansions = expansions;
        return finish(Status::kTimeout);
      }
      if (p.timeout_s > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() > p.timeout_s)
      {
        res.expansions = expansions;
        return finish(Status::kTimeout);
      }
    }

    for (int nh = 0; nh < kNumHeadings; ++nh) {
      if (nh == (h + 2) % kNumHeadings) {continue;}  // never reverse onto parent
      const int nx = x + kDx[nh];
      const int ny = y + kDy[nh];
      if (!traversable(grid, nx, ny, p)) {continue;}
      const int32_t ns = stateOf(nx, ny, nh);
      if (closed_stamp_[ns] == stamp_) {continue;}

      double step = 1.0;
      if (cw > 0.0) {
        const uint8_t c = grid.at(nx, ny);
        if (c != cost::kUnknown) {step += cw * std::min<int>(c, 252) / 252.0;}
      }
      if (nh != h) {step += tp;}
      const double ng = g_[s] + step;
      if (visit_stamp_[ns] != stamp_ || ng < g_[ns]) {
        visit_stamp_[ns] = stamp_;
        g_[ns] = ng;
        parent_[ns] = s;
        open.push({ng + heuristic(nx, ny, nh), ng, ns});
      }
    }
  }
  res.expansions = expansions;
  if (goal_state < 0) {return finish(Status::kNoPath);}

  // --- reconstruct -------------------------------------------------------------
  std::vector<Cell> rev;
  for (int32_t s = goal_state; s >= 0; s = parent_[s]) {
    const int cell = s / kNumHeadings;
    rev.push_back(Cell{cell % width, cell / width});
  }
  res.cells.assign(rev.rbegin(), rev.rend());
  res.corners = extractCorners(res.cells);
  res.turns = countTurns(res.cells);
  res.cost = g_[goal_state];
  return finish(Status::kSuccess);
}

std::vector<Cell> OrientedAStar::extractCorners(const std::vector<Cell> & cells)
{
  if (cells.size() <= 2) {return cells;}
  std::vector<Cell> out{cells.front()};
  for (size_t i = 1; i + 1 < cells.size(); ++i) {
    if (headingOf(cells[i - 1], cells[i]) != headingOf(cells[i], cells[i + 1])) {
      out.push_back(cells[i]);
    }
  }
  out.push_back(cells.back());
  return out;
}

int OrientedAStar::countTurns(const std::vector<Cell> & cells)
{
  int turns = 0;
  for (size_t i = 1; i + 1 < cells.size(); ++i) {
    if (headingOf(cells[i - 1], cells[i]) != headingOf(cells[i], cells[i + 1])) {++turns;}
  }
  return turns;
}

}  // namespace scr_planner
