// Copyright 2026 SCR Project

#include "scr_planner/oriented_astar.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>

namespace scr_planner
{

namespace
{

// Headings: 0 = +x, 1 = +y, 2 = -x, 3 = -y.  Axis of heading h is h % 2.
constexpr int kDx[4] = {1, 0, -1, 0};
constexpr int kDy[4] = {0, 1, 0, -1};
constexpr int kNumHeadings = 4;
constexpr double kInf = std::numeric_limits<double>::infinity();

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

inline int headingOf(const Cell & from, const Cell & to)
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) >= std::abs(dy)) {return dx >= 0 ? 0 : 2;}
  return dy > 0 ? 1 : 3;
}

}  // namespace

std::string toString(Status s)
{
  switch (s) {
    case Status::kSuccess: return "success";
    case Status::kStartOutOfBounds: return "start outside grid";
    case Status::kGoalOutOfBounds: return "goal outside grid";
    case Status::kGoalBlocked: return "goal in obstacle";
    case Status::kNoPath: return "no path";
    case Status::kTimeout: return "timeout";
  }
  return "unknown";
}

std::vector<Cell> OrientedAStar::arcCells(int from, int to, int r)
{
  std::vector<Cell> out;
  if (r <= 0 || (from % 2) == (to % 2)) {return out;}
  // Tangent points on the incoming / outgoing legs and the arc centre.
  const double p0x = -r * kDx[from], p0y = -r * kDy[from];
  const double p1x = r * kDx[to], p1y = r * kDy[to];
  const double cx = p0x + p1x, cy = p0y + p1y;
  const double a0 = std::atan2(p0y - cy, p0x - cx);
  double da = std::atan2(p1y - cy, p1x - cx) - a0;
  while (da > M_PI) {da -= 2 * M_PI;}
  while (da < -M_PI) {da += 2 * M_PI;}
  const int n = std::max(2, static_cast<int>(std::ceil(std::abs(da) * r / 0.25)) + 1);
  for (int i = 0; i < n; ++i) {
    const double a = a0 + da * i / (n - 1);
    const Cell c{static_cast<int>(std::lround(cx + r * std::cos(a))),
      static_cast<int>(std::lround(cy + r * std::sin(a)))};
    if (std::find(out.begin(), out.end(), c) == out.end()) {out.push_back(c);}
  }
  return out;
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
  double best = kInf;
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
  if (start != goal && !traversable(grid, goal.x, goal.y, p)) {
    Cell adjusted;
    if (p.goal_tolerance_cells <= 0.0 || !findNearestFree(grid, goal, p, adjusted)) {
      return finish(Status::kGoalBlocked);
    }
    goal = adjusted;
    res.goal = goal;
  }
  if (start == goal) {
    res.corners = {start};
    res.cells = {start};
    return finish(Status::kSuccess);
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
  if (++stamp_ == 0) {
    std::fill(visit_stamp_.begin(), visit_stamp_.end(), 0);
    std::fill(closed_stamp_.begin(), closed_stamp_.end(), 0);
    stamp_ = 1;
  }

  // --- constraint tables -------------------------------------------------------
  const bool allow[2] = {p.allow_x, p.allow_y};
  const double weight[2] = {p.weight_x, p.weight_y};
  const int max_run[2] = {
    p.max_run_x > 0 ? p.max_run_x : std::numeric_limits<int>::max(),
    p.max_run_y > 0 ? p.max_run_y : std::numeric_limits<int>::max()};
  const int r = std::max(0, p.bend_radius);
  std::array<std::array<std::vector<Cell>, kNumHeadings>, kNumHeadings> arcs;
  if (r > 0) {
    for (int a = 0; a < kNumHeadings; ++a) {
      for (int b = 0; b < kNumHeadings; ++b) {arcs[a][b] = arcCells(a, b, r);}
    }
  }
  const double cw = std::max(0.0, p.cost_weight);
  const double tp = p.turn_penalty;

  const int width = static_cast<int>(grid.width);
  auto stateOf = [&](int x, int y, int h) {return (y * width + x) * kNumHeadings + h;};
  // Lower bound on the remaining cost.  Every segment after a non-start state
  // begins with a bend, and each still-uncovered axis needs its own segment.
  auto heuristic = [&](int x, int y, bool at_start) {
      const int dx = goal.x - x;
      const int dy = goal.y - y;
      if ((dx != 0 && !allow[0]) || (dy != 0 && !allow[1])) {return kInf;}
      const int n = (dx != 0) + (dy != 0);
      const int bends = n == 0 ? 0 : (at_start ? n - 1 : n);
      return weight[0] * std::abs(dx) + weight[1] * std::abs(dy) + tp * bends;
    };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
  const int32_t start_state = stateOf(start.x, start.y, 0);
  {
    const double h0 = heuristic(start.x, start.y, true);
    if (!std::isfinite(h0)) {return finish(Status::kNoPath);}
    g_[start_state] = 0.0;
    parent_[start_state] = -1;
    visit_stamp_[start_state] = stamp_;
    open.push({h0, 0.0, start_state});
  }

  int32_t goal_state = -1;
  size_t expansions = 0;
  while (!open.empty()) {
    const QueueEntry top = open.top();
    open.pop();
    const int32_t s = top.state;
    if (closed_stamp_[s] == stamp_) {continue;}
    if (top.g > g_[s]) {continue;}  // stale
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
    if ((expansions & 0xFF) == 0) {
      if ((p.max_expansions > 0 && expansions >= p.max_expansions) ||
        (p.timeout_s > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() > p.timeout_s))
      {
        res.expansions = expansions;
        return finish(Status::kTimeout);
      }
    }

    const bool at_start = (s == start_state);
    for (int d = 0; d < kNumHeadings; ++d) {
      const int axis = d % 2;
      if (!allow[axis]) {continue;}
      if (!at_start && axis == h % 2) {continue;}  // segments alternate axes
      if (!at_start && r > 0) {
        bool arc_free = true;
        for (const Cell & o : arcs[h][d]) {
          if (!traversable(grid, x + o.x, y + o.y, p)) {arc_free = false; break;}
        }
        if (!arc_free) {continue;}
      }
      const double g0 = g_[s] + (at_start ? 0.0 : tp);
      double acc = 0.0;
      for (int k = 1; k <= max_run[axis]; ++k) {
        const int nx = x + k * kDx[d];
        const int ny = y + k * kDy[d];
        if (!traversable(grid, nx, ny, p)) {break;}
        acc += weight[axis];
        if (cw > 0.0) {
          const uint8_t c = grid.at(nx, ny);
          if (c != cost::kUnknown) {acc += cw * std::min<int>(c, 252) / 252.0;}
        }
        if (r > 0) {
          // Legs touching a bend need r (one bend) or 2r (bends at both ends).
          const bool ends_at_goal = (nx == goal.x && ny == goal.y);
          const int bends_on_leg = (at_start ? 0 : 1) + (ends_at_goal ? 0 : 1);
          if (k < bends_on_leg * r) {continue;}
        }
        const int32_t ns = stateOf(nx, ny, d);
        if (closed_stamp_[ns] == stamp_) {continue;}
        const double ng = g0 + acc;
        if (visit_stamp_[ns] != stamp_ || ng < g_[ns]) {
          const double hn = heuristic(nx, ny, false);
          if (!std::isfinite(hn)) {continue;}
          visit_stamp_[ns] = stamp_;
          g_[ns] = ng;
          parent_[ns] = s;
          open.push({ng + hn, ng, ns});
        }
      }
    }
  }
  res.expansions = expansions;
  if (goal_state < 0) {return finish(Status::kNoPath);}

  std::vector<Cell> rev;
  for (int32_t s = goal_state; s >= 0; s = parent_[s]) {
    const int cell = s / kNumHeadings;
    rev.push_back(Cell{cell % width, cell / width});
  }
  res.corners.assign(rev.rbegin(), rev.rend());
  res.cells = densify(res.corners);
  res.turns = static_cast<int>(res.corners.size()) - 2;
  res.cost = g_[goal_state];
  return finish(Status::kSuccess);
}

std::vector<Cell> OrientedAStar::densify(const std::vector<Cell> & corners)
{
  std::vector<Cell> out;
  if (corners.empty()) {return out;}
  out.push_back(corners.front());
  for (size_t i = 1; i < corners.size(); ++i) {
    const Cell a = corners[i - 1];
    const Cell b = corners[i];
    const int sx = (b.x > a.x) - (b.x < a.x);
    const int sy = (b.y > a.y) - (b.y < a.y);
    const int n = std::abs(b.x - a.x) + std::abs(b.y - a.y);
    for (int k = 1; k <= n; ++k) {out.push_back(Cell{a.x + sx * k, a.y + sy * k});}
  }
  return out;
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
