// Copyright 2026 SCR Project
//
// Unit tests for the segment-based OrientedAStar.

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

constexpr int kDx[4] = {1, 0, -1, 0};
constexpr int kDy[4] = {0, 1, 0, -1};
constexpr double kInf = std::numeric_limits<double>::infinity();

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

double stepCost(const GridView & g, int x, int y, int axis, const SearchParams & p)
{
  double c = axis == 0 ? p.weight_x : p.weight_y;
  const uint8_t v = g.at(x, y);
  if (p.cost_weight > 0.0 && v != 255) {c += p.cost_weight * std::min<int>(v, 252) / 252.0;}
  return c;
}

bool arcFree(const GridView & g, int x, int y, int from, int to, const SearchParams & p)
{
  if (p.bend_radius <= 0) {return true;}
  for (const Cell & o : OrientedAStar::arcCells(from, to, p.bend_radius)) {
    if (!passable(g, x + o.x, y + o.y, p)) {return false;}
  }
  return true;
}

int headingBetween(Cell a, Cell b)
{
  if (b.x > a.x) {return 0;}
  if (b.y > a.y) {return 1;}
  if (b.x < a.x) {return 2;}
  return 3;
}

/// Independent check of every rule on a corner list, plus cost recomputation.
void expectValidPath(
  const GridView & g, const SearchResult & r, Cell s, Cell t, const SearchParams & p)
{
  ASSERT_TRUE(r.ok()) << scr_planner::toString(r.status);
  ASSERT_GE(r.corners.size(), 1u);
  EXPECT_EQ(r.corners.front(), s);
  EXPECT_EQ(r.corners.back(), t);
  EXPECT_EQ(r.turns, std::max(0, static_cast<int>(r.corners.size()) - 2));
  double cost = 0.0;
  const int n = static_cast<int>(r.corners.size());
  for (int i = 1; i < n; ++i) {
    const Cell a = r.corners[i - 1], b = r.corners[i];
    ASSERT_TRUE(a.x == b.x || a.y == b.y) << "diagonal segment " << i;
    ASSERT_NE(a, b);
    const int d = headingBetween(a, b);
    const int axis = d % 2;
    const int len = std::abs(b.x - a.x) + std::abs(b.y - a.y);
    EXPECT_TRUE(axis == 0 ? p.allow_x : p.allow_y) << "forbidden axis, segment " << i;
    const int max_run = axis == 0 ? p.max_run_x : p.max_run_y;
    if (max_run > 0) {EXPECT_LE(len, max_run) << "segment " << i;}
    if (p.bend_radius > 0) {
      const int bends = (i > 1) + (i < n - 1);
      EXPECT_GE(len, bends * p.bend_radius) << "segment " << i << " too short for bend";
    }
    if (i > 1) {
      const int prev = headingBetween(r.corners[i - 2], a);
      EXPECT_NE(prev % 2, axis) << "consecutive segments not perpendicular at " << i;
      EXPECT_TRUE(arcFree(g, a.x, a.y, prev, d, p)) << "bend arc blocked at corner " << i - 1;
      cost += p.turn_penalty;
    }
    for (int k = 1; k <= len; ++k) {
      const int x = a.x + k * kDx[d], y = a.y + k * kDy[d];
      EXPECT_TRUE(passable(g, x, y, p)) << "blocked cell (" << x << "," << y << ")";
      cost += stepCost(g, x, y, axis, p);
    }
  }
  EXPECT_NEAR(r.cost, cost, 1e-6);
}

/// Reference: Dijkstra stepping ONE cell at a time with state
/// (x, y, heading, run length, first-segment flag).  Shares nothing with the
/// segment-level search except the arc discretisation.
double dijkstraReference(const GridView & g, Cell s, Cell t, const SearchParams & p)
{
  if (s == t) {return 0.0;}
  const int r = p.bend_radius;
  const int lim[2] = {p.max_run_x, p.max_run_y};
  // Run lengths beyond every threshold behave the same: saturate there.
  int cap = std::max(2 * r, 1);
  for (int a = 0; a < 2; ++a) {if (lim[a] > 0) {cap = std::max(cap, lim[a]);}}
  const int W = g.width, H = g.height;
  auto id = [&](int x, int y, int h, int run, int first) {
      return ((((static_cast<size_t>(y) * W + x) * 4 + h) * (cap + 1) + run) * 2) + first;
    };
  std::vector<double> dist(static_cast<size_t>(W) * H * 4 * (cap + 1) * 2, kInf);
  struct QE {double d; int x, y, h, run, first;};
  auto cmp = [](const QE & a, const QE & b) {return a.d > b.d;};
  std::priority_queue<QE, std::vector<QE>, decltype(cmp)> pq(cmp);
  auto relax = [&](double d, int x, int y, int h, int run, int first) {
      const size_t k = id(x, y, h, std::min(run, cap), first);
      if (d < dist[k]) {dist[k] = d; pq.push({d, x, y, h, std::min(run, cap), first});}
    };
  const bool allow[2] = {p.allow_x, p.allow_y};
  for (int d = 0; d < 4; ++d) {
    const int nx = s.x + kDx[d], ny = s.y + kDy[d];
    if (!allow[d % 2] || !passable(g, nx, ny, p)) {continue;}
    if (lim[d % 2] > 0 && 1 > lim[d % 2]) {continue;}
    relax(stepCost(g, nx, ny, d % 2, p), nx, ny, d, 1, 1);
  }
  while (!pq.empty()) {
    const QE q = pq.top();
    pq.pop();
    if (q.d > dist[id(q.x, q.y, q.h, q.run, q.first)]) {continue;}
    if (q.x == t.x && q.y == t.y && q.run >= (q.first ? 0 : r)) {return q.d;}
    const int axis = q.h % 2;
    // straight on
    {
      const int nx = q.x + kDx[q.h], ny = q.y + kDy[q.h];
      const bool within = lim[axis] <= 0 || q.run + 1 <= lim[axis];
      if (within && passable(g, nx, ny, p)) {
        relax(q.d + stepCost(g, nx, ny, axis, p), nx, ny, q.h, q.run + 1, q.first);
      }
    }
    // bend
    if (q.run >= (q.first ? r : 2 * r)) {
      for (int d : {(q.h + 1) % 4, (q.h + 3) % 4}) {
        if (!allow[d % 2]) {continue;}
        if (!arcFree(g, q.x, q.y, q.h, d, p)) {continue;}
        const int nx = q.x + kDx[d], ny = q.y + kDy[d];
        if (!passable(g, nx, ny, p)) {continue;}
        relax(q.d + p.turn_penalty + stepCost(g, nx, ny, d % 2, p), nx, ny, d, 1, 0);
      }
    }
  }
  return kInf;
}

int segmentLength(Cell a, Cell b) {return std::abs(b.x - a.x) + std::abs(b.y - a.y);}

}  // namespace

// --- basic behaviour ----------------------------------------------------------

TEST(OrientedAStar, StraightLineHasNoTurns)
{
  TestGrid grid(10, 5);
  OrientedAStar planner;
  SearchParams p;
  const auto r = planner.plan(grid.view(), {1, 2}, {8, 2}, p);
  expectValidPath(grid.view(), r, {1, 2}, {8, 2}, p);
  EXPECT_EQ(r.turns, 0);
  EXPECT_DOUBLE_EQ(r.cost, 7.0);
  EXPECT_EQ(r.cells.size(), 8u);
}

TEST(OrientedAStar, OpenFieldUsesExactlyOneTurn)
{
  TestGrid grid(20, 20);
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 0.5;
  const auto r = planner.plan(grid.view(), {2, 3}, {15, 17}, p);
  expectValidPath(grid.view(), r, {2, 3}, {15, 17}, p);
  EXPECT_EQ(r.turns, 1);
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
  EXPECT_EQ(r.turns, 2);
  EXPECT_DOUBLE_EQ(r.cost, 18.0 + 2 * 2.0);
}

TEST(OrientedAStar, TurnPenaltyTradesLengthForFewerTurns)
{
  // Staggered gaps give a 14-cell route with 4 bends; the bypass at x=8 is 20
  // cells with 2 bends.  Break-even at turn_penalty = 3.
  const TestGrid grid({
    "S........",
    "##.#####.",
    ".........",
    "#####.##.",
    "G........",
  });
  const Cell s{0, 0}, t{0, 4};
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 1.0;
  auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.cells.size() - 1, 14u);
  EXPECT_EQ(r.turns, 4);

  p.turn_penalty = 5.0;
  r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.cells.size() - 1, 20u);
  EXPECT_EQ(r.turns, 2);
}

TEST(OrientedAStar, FailureModes)
{
  const TestGrid enclosed({
    ".......",
    ".#####.",
    ".#...#.",
    ".#...#.",
    ".#####.",
  });
  OrientedAStar planner;
  SearchParams p;
  EXPECT_EQ(planner.plan(enclosed.view(), {0, 0}, {3, 2}, p).status, Status::kNoPath);

  TestGrid grid(5, 5);
  grid.set(4, 4, 254);
  EXPECT_EQ(planner.plan(grid.view(), {-1, 0}, {2, 2}, p).status, Status::kStartOutOfBounds);
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {5, 2}, p).status, Status::kGoalOutOfBounds);
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {4, 4}, p).status, Status::kGoalBlocked);
  p.goal_tolerance_cells = 1.5;
  const auto r = planner.plan(grid.view(), {0, 0}, {4, 4}, p);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(std::abs(r.goal.x - 4) + std::abs(r.goal.y - 4), 1);
  EXPECT_EQ(r.corners.back(), r.goal);
}

TEST(OrientedAStar, StartEqualsGoal)
{
  TestGrid grid(3, 3);
  OrientedAStar planner;
  const auto r = planner.plan(grid.view(), {1, 1}, {1, 1}, SearchParams{});
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.corners.size(), 1u);
  EXPECT_EQ(r.turns, 0);
}

TEST(OrientedAStar, StartMayBeInsideObstacleButNotItsNeighbours)
{
  const TestGrid grid({
    "iii..",
    "iii..",
    "iii..",
  });
  OrientedAStar planner;
  SearchParams p;
  EXPECT_EQ(planner.plan(grid.view(), {0, 1}, {4, 1}, p).status, Status::kNoPath);
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
  EXPECT_TRUE(planner.plan(grid.view(), {0, 0}, {0, 2}, p).ok());
  p.allow_unknown = false;
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {0, 2}, p).status, Status::kNoPath);
}

TEST(OrientedAStar, CostWeightAvoidsHighCostCorridor)
{
  const TestGrid grid({
    ".99999.",
    ".#####.",
    ".......",
  });
  const Cell s{0, 0}, t{6, 0};
  OrientedAStar planner;
  SearchParams p;
  p.turn_penalty = 0.5;
  auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.turns, 0);
  p.cost_weight = 5.0;
  r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.turns, 2);
}

TEST(OrientedAStar, ExpansionLimitTimesOut)
{
  TestGrid grid(200, 200);
  for (int y = 0; y < 199; ++y) {grid.set(100, y, 254);}
  OrientedAStar planner;
  SearchParams p;
  p.max_expansions = 1;
  EXPECT_EQ(planner.plan(grid.view(), {0, 0}, {199, 0}, p).status, Status::kTimeout);
}

// --- construction rules -------------------------------------------------------

TEST(OrientedAStar, ForbiddenHorizontalAxis)
{
  // Load-bearing wall: no horizontal chase at all.
  TestGrid grid(10, 10);
  OrientedAStar planner;
  SearchParams p;
  p.allow_x = false;
  auto r = planner.plan(grid.view(), {3, 0}, {3, 9}, p);
  expectValidPath(grid.view(), r, {3, 0}, {3, 9}, p);
  EXPECT_EQ(r.turns, 0);
  EXPECT_EQ(planner.plan(grid.view(), {3, 0}, {4, 9}, p).status, Status::kNoPath);
  grid.set(3, 5, 254);  // vertical route blocked -> no legal route
  EXPECT_EQ(planner.plan(grid.view(), {3, 0}, {3, 9}, p).status, Status::kNoPath);
}

TEST(OrientedAStar, AxisWeightPrefersVerticalDetour)
{
  // Obstacle in the middle of a straight horizontal run.  Going round it costs
  // 2 extra vertical cells either way; with weight_x high the planner should
  // still go round (not possible otherwise) but keep horizontal travel minimal.
  TestGrid grid(21, 11);
  for (int y = 0; y <= 6; ++y) {grid.set(10, y, 254);}
  OrientedAStar planner;
  SearchParams p;
  p.weight_x = 3.0;
  const auto r = planner.plan(grid.view(), {0, 3}, {20, 3}, p);
  expectValidPath(grid.view(), r, {0, 3}, {20, 3}, p);
  int horiz = 0;
  for (size_t i = 1; i < r.corners.size(); ++i) {
    if (r.corners[i].y == r.corners[i - 1].y) {horiz += segmentLength(r.corners[i - 1], r.corners[i]);}
  }
  EXPECT_EQ(horiz, 20);
}

TEST(OrientedAStar, HorizontalRunLimitForcesSteps)
{
  // 30 cells to the right with horizontal chases <= 10 cells: H10 V H10 V H10.
  TestGrid grid(31, 5);
  OrientedAStar planner;
  SearchParams p;
  p.max_run_x = 10;
  const auto r = planner.plan(grid.view(), {0, 2}, {30, 2}, p);
  expectValidPath(grid.view(), r, {0, 2}, {30, 2}, p);
  EXPECT_EQ(r.turns, 4);
  EXPECT_DOUBLE_EQ(r.cost, 32.0 + 4 * p.turn_penalty);
  p.max_run_x = 9;  // needs 4 horizontal segments now
  const auto r2 = planner.plan(grid.view(), {0, 2}, {30, 2}, p);
  expectValidPath(grid.view(), r2, {0, 2}, {30, 2}, p);
  EXPECT_EQ(r2.turns, 6);
}

TEST(OrientedAStar, BendRadiusLengthensShortLegs)
{
  // Goal 3 cells to the side and 20 up: an L would have a 3-cell leg, but a
  // radius-5 bend needs >= 5.  Best legal route: 8 / 20 / 5 (or mirrored).
  TestGrid grid(30, 25);
  OrientedAStar planner;
  SearchParams p;
  p.bend_radius = 5;
  const Cell s{10, 0}, t{13, 20};
  const auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.turns, 2);
  EXPECT_EQ(r.cells.size() - 1, 33u);
  p.bend_radius = 0;
  EXPECT_EQ(planner.plan(grid.view(), s, t, p).turns, 1);
}

TEST(OrientedAStar, BendArcMustBeFree)
{
  // Block the inside of both L-shaped bends; the conduit arc would hit them
  // although the legs themselves are free.
  TestGrid grid(25, 25);
  const Cell s{0, 0}, t{20, 20};
  const auto arc_a = OrientedAStar::arcCells(0, 1, 5);  // right then up, bend at (20,0)
  const auto arc_b = OrientedAStar::arcCells(1, 0, 5);  // up then right, bend at (0,20)
  const Cell mid_a{20 + arc_a[arc_a.size() / 2].x, 0 + arc_a[arc_a.size() / 2].y};
  const Cell mid_b{0 + arc_b[arc_b.size() / 2].x, 20 + arc_b[arc_b.size() / 2].y};
  grid.set(mid_a.x, mid_a.y, 254);
  grid.set(mid_b.x, mid_b.y, 254);
  OrientedAStar planner;
  SearchParams p;
  p.bend_radius = 0;
  auto r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_EQ(r.turns, 1);  // without the bend rule the L is fine
  p.bend_radius = 5;
  r = planner.plan(grid.view(), s, t, p);
  expectValidPath(grid.view(), r, s, t, p);
  EXPECT_GE(r.turns, 2);
}

TEST(OrientedAStar, ArcCellsGeometry)
{
  for (int from = 0; from < 4; ++from) {
    for (int to : {(from + 1) % 4, (from + 3) % 4}) {
      for (int r : {1, 3, 12}) {
        const auto arc = OrientedAStar::arcCells(from, to, r);
        ASSERT_GE(arc.size(), 2u);
        // Tangent points on the incoming and outgoing legs.
        EXPECT_NE(std::find(arc.begin(), arc.end(), Cell{-r * kDx[from], -r * kDy[from]}),
          arc.end());
        EXPECT_NE(std::find(arc.begin(), arc.end(), Cell{r * kDx[to], r * kDy[to]}), arc.end());
        const double cx = -r * kDx[from] + r * kDx[to];
        const double cy = -r * kDy[from] + r * kDy[to];
        for (const Cell & c : arc) {
          EXPECT_NEAR(std::hypot(c.x - cx, c.y - cy), r, 0.75);
        }
      }
    }
  }
  EXPECT_TRUE(OrientedAStar::arcCells(0, 2, 5).empty());  // not a bend
}

TEST(OrientedAStar, CornerHelpers)
{
  const std::vector<Cell> cells{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}, {3, 2}};
  const auto corners = OrientedAStar::extractCorners(cells);
  ASSERT_EQ(corners.size(), 4u);
  EXPECT_EQ(corners[1], (Cell{2, 0}));
  EXPECT_EQ(OrientedAStar::countTurns(cells), 2);
  EXPECT_EQ(OrientedAStar::densify(corners), cells);
}

// --- optimality against an independent reference ------------------------------

TEST(OrientedAStar, MatchesStepwiseDijkstraOnRandomGrids)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  OrientedAStar planner;  // reused on purpose (buffer reuse path)
  int solved = 0, infeasible = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const unsigned int w = 8 + trial % 13;
    const unsigned int h = 6 + (trial * 7) % 15;
    TestGrid grid(w, h);
    const double density = 0.05 + 0.25 * u(rng);
    for (auto & c : grid.data) {
      const double r = u(rng);
      if (r < density) {c = 254;} else if (r < density + 0.15) {
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
    p.weight_x = (trial % 5 < 2) ? 1.0 : 2.0;
    p.allow_x = (trial % 11 != 0);
    p.max_run_x = (trial % 4 == 1) ? 2 + trial % 5 : 0;
    p.max_run_y = (trial % 7 == 3) ? 3 + trial % 4 : 0;
    p.bend_radius = std::vector<int>{0, 0, 1, 2, 3}[trial % 5];

    const double ref = dijkstraReference(grid.view(), s, t, p);
    const auto r = planner.plan(grid.view(), s, t, p);
    if (std::isinf(ref)) {
      ++infeasible;
      EXPECT_EQ(r.status, Status::kNoPath) << "trial " << trial;
      continue;
    }
    ++solved;
    expectValidPath(grid.view(), r, s, t, p);
    EXPECT_NEAR(r.cost, ref, 1e-6) << "trial " << trial;
  }
  EXPECT_GT(solved, 150);
  EXPECT_GT(infeasible, 10);
}

TEST(OrientedAStar, MonotoneInTurnPenalty)
{
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
