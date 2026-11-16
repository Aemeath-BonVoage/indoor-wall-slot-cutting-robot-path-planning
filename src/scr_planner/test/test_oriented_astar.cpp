// Copyright 2026 SCR Project
//
// Unit tests for the ROS-free OrientedAStar core.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include "scr_planner/oriented_astar.hpp"

using scr_planner::Cell;
using scr_planner::GridView;
using scr_planner::OrientedAStar;
using scr_planner::SearchParams;
using scr_planner::SearchResult;
using scr_planner::Status;

namespace
{

/// Grid built from ASCII rows; row index == y.
///   '.' free  '#' lethal  'i' inscribed  '?' unknown  '1'..'9' cost = digit * 25
struct TestGrid
{
  unsigned int w{0}, h{0};
  std::vector<uint8_t> data;

  explicit TestGrid(const std::vector<std::string> & rows)
  {
    h = rows.size();
    w = rows.front().size();
    data.resize(w * h);
    for (unsigned int y = 0; y < h; ++y) {
      for (unsigned int x = 0; x < w; ++x) {
        const char c = rows[y][x];
        uint8_t v = 0;
        if (c == '#') {v = 254;} else if (c == 'i') {v = 253;} else if (c == '?') {
          v = 255;
        } else if (c >= '1' && c <= '9') {v = static_cast<uint8_t>((c - '0') * 25);}
        data[y * w + x] = v;
      }
    }
  }
  TestGrid(unsigned int w_, unsigned int h_, uint8_t fill = 0)
  : w(w_), h(h_), data(w_ * h_, fill) {}

  GridView view() const {return GridView{w, h, data.data()};}
  void set(int x, int y, uint8_t v) {data[y * w + x] = v;}
};

bool passable(const GridView & g, int x, int y, const SearchParams & p)
{
  if (!g.inBounds(x, y)) {return false;}
  const uint8_t c = g.at(x, y);
  if (c == 255) {return p.allow_unknown;}
  return c < p.obstacle_threshold;
}

double stepCost(const GridView & g, int x, int y, const SearchParams & p)
{
  const uint8_t c = g.at(x, y);
  if (p.cost_weight <= 0.0 || c == 255) {return 1.0;}
  return 1.0 + p.cost_weight * std::min<int>(c, 252) / 252.0;
}

/// Recompute the objective of a cell sequence independently of the planner.
double pathCost(const GridView & g, const std::vector<Cell> & cells, const SearchParams & p)
{
  double c = 0.0;
  for (size_t i = 1; i < cells.size(); ++i) {c += stepCost(g, cells[i].x, cells[i].y, p);}
  return c + p.turn_penalty * OrientedAStar::countTurns(cells);
}

/// Reference: plain Dijkstra over (x, y, heading), no heuristic, no pruning.
double dijkstraReference(const GridView & g, Cell s, Cell t, const SearchParams & p)
{
  const int dx[4] = {1, 0, -1, 0};
  const int dy[4] = {0, 1, 0, -1};
  const size_t n = static_cast<size_t>(g.width) * g.height * 4;
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  using QE = std::pair<double, size_t>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  auto id = [&](int x, int y, int hd) {return (static_cast<size_t>(y) * g.width + x) * 4 + hd;};
  for (int hd = 0; hd < 4; ++hd) {
    dist[id(s.x, s.y, hd)] = 0.0;
    pq.push({0.0, id(s.x, s.y, hd)});
  }
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) {continue;}
    const int hd = u % 4;
    const int cell = u / 4;
    const int x = cell % g.width;
    const int y = cell / g.width;
    if (x == t.x && y == t.y) {return d;}
    for (int nh = 0; nh < 4; ++nh) {
      const int nx = x + dx[nh];
      const int ny = y + dy[nh];
      if (!passable(g, nx, ny, p)) {continue;}
      const double nd = d + stepCost(g, nx, ny, p) + (nh != hd ? p.turn_penalty : 0.0);
      const size_t v = id(nx, ny, nh);
      if (nd < dist[v]) {
        dist[v] = nd;
        pq.push({nd, v});
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

void expectValidPath(
  const GridView & g, const SearchResult & r, Cell s, Cell t, const SearchParams & p)
{
  ASSERT_TRUE(r.ok());
  ASSERT_FALSE(r.cells.empty());
  EXPECT_EQ(r.cells.front(), s);
  EXPECT_EQ(r.cells.back(), t);
  for (size_t i = 1; i < r.cells.size(); ++i) {
    const int manhattan = std::abs(r.cells[i].x - r.cells[i - 1].x) +
      std::abs(r.cells[i].y - r.cells[i - 1].y);
    EXPECT_EQ(manhattan, 1) << "non 4-connected step at " << i;
    EXPECT_TRUE(passable(g, r.cells[i].x, r.cells[i].y, p)) << "blocked cell at " << i;
  }
  EXPECT_EQ(r.turns, OrientedAStar::countTurns(r.cells));
  EXPECT_EQ(static_cast<int>(r.corners.size()), r.cells.size() > 1 ? r.turns + 2 : 1);
  EXPECT_NEAR(r.cost, pathCost(g, r.cells, p), 1e-6);
}

}  // namespace

TEST(OrientedAStar, StraightLineHasNoTurns)
{
  TestGrid grid(10, 5);
  OrientedAStar planner;
  SearchParams p;
  const auto r = planner.plan(grid.view(), {1, 2}, {8, 2}, p);
  expectValidPath(grid.view(), r, {1, 2}, {8, 2}, p);
  EXPECT_EQ(r.cells.size(), 8u);
  EXPECT_EQ(r.turns, 0);
  EXPECT_DOUBLE_EQ(r.cost, 7.0);
  ASSERT_EQ(r.corners.size(), 2u);
}

TEST(OrientedAStar, OpenFieldUsesExactlyOneTurn)
{
  // Plain A* would happily staircase here; any manhattan path is shortest,
  // the turn penalty must pick an L-shape.
  TestGrid grid(20, 20);
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 0.5;
  const auto r = planner.plan(grid.view(), {2, 3}, {15, 17}, p);
  expectValidPath(grid.view(), r, {2, 3}, {15, 17}, p);
  EXPECT_EQ(r.turns, 1);
  EXPECT_EQ(r.cells.size(), 13u + 14u + 1u);
  EXPECT_DOUBLE_EQ(r.cost, 27.0 + 0.5);
}

TEST(OrientedAStar, DetoursAroundWall)
{
  const TestGrid grid({
    "...#...",
    "...#...",
    "...#...",
    "...#...",
    "...#...",
    "...#...",
    ".......",
  });
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 2.0;
  const auto r = planner.plan(grid.view(), {0, 0}, {6, 0}, p);
  expectValidPath(grid.view(), r, {0, 0}, {6, 0}, p);
  EXPECT_EQ(r.cells.size(), 19u);  // 6 down + 6 across + 6 up
  EXPECT_EQ(r.turns, 2);
  EXPECT_DOUBLE_EQ(r.cost, 18.0 + 2 * 2.0);
}

TEST(OrientedAStar, PrefersGapInLineWithGoal)
{
  // Gaps at x=2 and x=6.  Every shortest path has length 10; only the one
  // through the x=6 gap needs a single turn.
  const TestGrid grid({
    "S......",
    ".......",
    "##.###.",
    ".......",
    "......G",
  });
  const Cell s{0, 0}, t{6, 4};
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 1.0;
  const auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.cells.size(), 11u);
  EXPECT_EQ(r.turns, 1);
}

TEST(OrientedAStar, TurnPenaltyTradesLengthForFewerTurns)
{
  // Staggered gaps (x=2 in the first wall, x=5 in the second) give a short
  // route with 4 turns (length 14); the bypass at x=8 has 2 turns (length 20).
  // Break-even at turn_penalty = 3.
  const TestGrid grid({
    "S........",
    "##.######",
    ".........",
    "#####.##.",
    "G........",
  });
  // Open the bypass column.
  TestGrid g2 = grid;
  g2.set(8, 1, 0);
  const Cell s{0, 0}, t{0, 4};
  OrientedAStar planner;
  SearchParams p;

  p.turn_penalty = 1.0;
  auto r = planner.plan(g2.view(), s, t, p);
  expectValidPath(g2.view(), r, s, t, p);
  EXPECT_EQ(r.cells.size() - 1, 14u);
  EXPECT_EQ(r.turns, 4);
  EXPECT_DOUBLE_EQ(r.cost, 14.0 + 4.0);

  p.turn_penalty = 5.0;
  r = planner.plan(g2.view(), s, t, p);
  expectValidPath(g2.view(), r, s, t, p);
  EXPECT_EQ(r.cells.size() - 1, 20u);
  EXPECT_EQ(r.turns, 2);
  EXPECT_DOUBLE_EQ(r.cost, 20.0 + 10.0);
}

TEST(OrientedAStar, UnreachableGoalFails)
{
  const TestGrid grid({
    ".......",
    ".#####.",
    ".#...#.",
    ".#...#.",
    ".#####.",
  });
  OrientedAStar planner;
  SearchParams p;
  const auto r = planner.plan(grid.view(), {0, 0}, {3, 2}, p);
  EXPECT_EQ(r.status, Status::kNoPath);
  EXPECT_TRUE(r.cells.empty());
}

TEST(OrientedAStar, OutOfBoundsAndBlockedEndpoints)
{
  TestGrid grid(5, 5);
  grid.set(4, 4, 254);
  OrientedAStar planner;
  SearchParams p;
  EXPECT_EQ(planner.plan(grid.view(), {-1, 0}, {2, 2}, p).status, Status::kStartOutOfBounds);
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {5, 2}, p).status, Status::kGoalOutOfBounds);
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {4, 4}, p).status, Status::kGoalBlocked);

  p.goal_tolerance_cells = 1.5;  // nearest free cell is adjacent
  const auto r = planner.plan(grid.view(), {0, 0}, {4, 4}, p);
  ASSERT_TRUE(r.ok());
  EXPECT_NE(r.goal, (Cell{4, 4}));
  EXPECT_EQ(std::abs(r.goal.x - 4) + std::abs(r.goal.y - 4), 1);
  EXPECT_EQ(r.cells.back(), r.goal);
}

TEST(OrientedAStar, StartEqualsGoal)
{
  TestGrid grid(3, 3);
  OrientedAStar planner;
  const auto r = planner.plan(grid.view(), {1, 1}, {1, 1}, SearchParams{});
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.cells.size(), 1u);
  EXPECT_EQ(r.turns, 0);
}

TEST(OrientedAStar, InscribedBlocksButStartMayBeInside)
{
  const TestGrid grid({
    "iii..",
    "iii..",
    "iii..",
  });
  OrientedAStar planner;
  SearchParams p;
  // Start inside the inflation ring, goal outside, and the only way out is the
  // ring itself -> no path.
  EXPECT_EQ(planner.plan(grid.view(), {0, 1}, {4, 1}, p).status, Status::kNoPath);
  // Start on the ring edge next to free space: allowed to leave.
  const auto r = planner.plan(grid.view(), {2, 1}, {4, 1}, p);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.cells.size(), 3u);
}

TEST(OrientedAStar, UnknownCellsRespectAllowUnknown)
{
  const TestGrid grid({
    ".....",
    "#?###",
    ".....",
  });
  OrientedAStar planner;
  SearchParams p;
  p.allow_unknown = true;
  EXPECT_TRUE(planner.plan(grid.view(), {0, 0}, {0, 2}, p).ok());
  p.allow_unknown = false;
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {0, 2}, p).status, Status::kNoPath);
}

TEST(OrientedAStar, CostWeightAvoidsHighCostCorridor)
{
  // Two equally long, equally bent corridors; the upper one runs through
  // high-cost (near-obstacle) cells.
  const TestGrid grid({
    ".99999.",
    ".#####.",
    ".......",
  });
  const Cell s{0, 0}, t{6, 0};
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 0.5;
  p.cost_weight = 0.0;
  auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.turns, 0);  // unweighted: straight through the costly row

  p.cost_weight = 5.0;
  r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  for (const auto & c : r.cells) {
    if (c.y == 0) {EXPECT_TRUE(c.x == 0 || c.x == 6);}
  }
  EXPECT_EQ(r.turns, 2);
}

TEST(OrientedAStar, ExpansionLimitTimesOut)
{
  TestGrid grid(300, 300);
  OrientedAStar planner;
  SearchParams p;
  p.max_expansions = 1;
  p.cost_weight = 0.0;
  // Force a large search: wall with a gap at the far end.
  for (int y = 0; y < 299; ++y) {grid.set(150, y, 254);}
  const auto r = planner.plan(grid.view(), {0, 0}, {299, 0}, p);
  EXPECT_EQ(r.status, Status::kTimeout);
}

TEST(OrientedAStar, CornerExtraction)
{
  const std::vector<Cell> cells{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}, {3, 2}};
  const auto corners = OrientedAStar::extractCorners(cells);
  ASSERT_EQ(corners.size(), 4u);
  EXPECT_EQ(corners[1], (Cell{2, 0}));
  EXPECT_EQ(corners[2], (Cell{2, 2}));
  EXPECT_EQ(OrientedAStar::countTurns(cells), 2);
}

TEST(OrientedAStar, MatchesDijkstraOnRandomGrids)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  OrientedAStar planner;  // reused across calls on purpose (buffer reuse path)
  int solved = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const unsigned int w = 8 + trial % 17;
    const unsigned int h = 6 + (trial * 7) % 19;
    TestGrid grid(w, h);
    const double density = 0.15 + 0.25 * u(rng);
    for (auto & c : grid.data) {
      const double r = u(rng);
      if (r < density) {c = 254;} else if (r < density + 0.2) {
        c = static_cast<uint8_t>(u(rng) * 252);
      }
    }
    std::uniform_int_distribution<int> rx(0, w - 1), ry(0, h - 1);
    const Cell s{rx(rng), ry(rng)}, t{rx(rng), ry(rng)};
    grid.set(s.x, s.y, 0);
    grid.set(t.x, t.y, 0);

    SearchParams p;
    p.turn_penalty = std::vector<double>{0.0, 0.7, 2.5, 6.0}[trial % 4];
    p.cost_weight = (trial % 3 == 0) ? 0.0 : 3.0;

    const double ref = dijkstraReference(grid.view(), s, t, p);
    const auto r = planner.plan(grid.view(), s, t, p);
    if (std::isinf(ref)) {
      EXPECT_EQ(r.status, Status::kNoPath) << "trial " << trial;
      continue;
    }
    ++solved;
    expectValidPath(grid.view(), r, s, t, p);
    EXPECT_NEAR(r.cost, ref, 1e-6) << "trial " << trial << " tp=" << p.turn_penalty;
  }
  EXPECT_GT(solved, 150);
}

TEST(OrientedAStar, MonotoneInTurnPenalty)
{
  // For tp1 < tp2 the optimal turn count cannot increase and the optimal
  // length cannot decrease (parametric optimality argument).
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  OrientedAStar planner;
  for (int trial = 0; trial < 60; ++trial) {
    TestGrid grid(30, 25);
    for (auto & c : grid.data) {c = u(rng) < 0.25 ? 254 : 0;}
    const Cell s{0, 0}, t{29, 24};
    grid.set(s.x, s.y, 0);
    grid.set(t.x, t.y, 0);
    int prev_turns = std::numeric_limits<int>::max();
    size_t prev_len = 0;
    for (double tp : {0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0}) {
      SearchParams p;
      p.turn_penalty = tp;
      const auto r = planner.plan(grid.view(), s, t, p);
      if (!r.ok()) {break;}
      EXPECT_LE(r.turns, prev_turns) << "trial " << trial << " tp " << tp;
      EXPECT_GE(r.cells.size(), prev_len) << "trial " << trial << " tp " << tp;
      prev_turns = r.turns;
      prev_len = r.cells.size();
    }
  }
}
