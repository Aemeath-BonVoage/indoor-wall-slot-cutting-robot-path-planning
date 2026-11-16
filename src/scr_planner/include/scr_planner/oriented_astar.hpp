// Copyright 2026 SCR Project
//
// Orientation-aware A* on a 4-connected grid ("enhanced A*", C++ version).
//
// The search state is (x, y, heading) with heading in {+x, +y, -x, -y}, so the
// cost of a path is exactly
//
//     J = sum(step_cost) + turn_penalty * (number of heading changes)
//
// and A* with a consistent heuristic returns a path that minimises J.  The
// Python prototype (src/path_planning/improved_astar.py) keyed its closed set on
// (x, y) only; with a turn penalty that makes the cost path-dependent and the
// result is not guaranteed optimal.  Adding the heading to the state fixes it.
//
// This header is ROS-free so it can be unit-tested and benchmarked on plain
// grids; the Nav2 plugin is a thin adapter around it.

#ifndef SCR_PLANNER__ORIENTED_ASTAR_HPP_
#define SCR_PLANNER__ORIENTED_ASTAR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace scr_planner
{

/// Cost values follow nav2_costmap_2d conventions.
namespace cost
{
constexpr uint8_t kFree = 0;
constexpr uint8_t kInscribed = 253;
constexpr uint8_t kLethal = 254;
constexpr uint8_t kUnknown = 255;
}  // namespace cost

struct Cell
{
  int x{0};
  int y{0};
  bool operator==(const Cell & o) const {return x == o.x && y == o.y;}
  bool operator!=(const Cell & o) const {return !(*this == o);}
};

/// Non-owning, row-major view of a costmap (index = y * width + x).
struct GridView
{
  unsigned int width{0};
  unsigned int height{0};
  const uint8_t * data{nullptr};

  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(width) && y < static_cast<int>(height);
  }
  uint8_t at(int x, int y) const {return data[static_cast<size_t>(y) * width + x];}
};

struct SearchParams
{
  /// Extra cost (in cells of travel) charged for every heading change.
  double turn_penalty{2.5};
  /// If > 0, each step into a cell costs 1 + cost_weight * cost / 252.
  double cost_weight{0.0};
  /// Treat NO_INFORMATION (255) cells as traversable (with zero extra cost).
  bool allow_unknown{true};
  /// Cells with cost >= this value are not traversable (253 = inscribed).
  uint8_t obstacle_threshold{cost::kInscribed};
  /// If the goal cell is blocked, search for the nearest traversable cell
  /// within this radius (in cells).  0 disables.
  double goal_tolerance_cells{0.0};
  /// Abort after this many seconds (<= 0: no limit).
  double timeout_s{0.0};
  /// Abort after this many expanded states (0: no limit).
  size_t max_expansions{0};
};

enum class Status
{
  kSuccess,
  kStartOutOfBounds,
  kGoalOutOfBounds,
  kGoalBlocked,
  kNoPath,
  kTimeout,
};

std::string toString(Status s);

struct SearchResult
{
  Status status{Status::kNoPath};
  /// Dense 4-connected cell sequence from start to (possibly adjusted) goal.
  std::vector<Cell> cells;
  /// Start, every corner, and the goal.
  std::vector<Cell> corners;
  /// The goal actually used (differs from the request if tolerance kicked in).
  Cell goal;
  int turns{0};
  double cost{0.0};
  size_t expansions{0};
  double time_ms{0.0};

  bool ok() const {return status == Status::kSuccess;}
};

class OrientedAStar
{
public:
  /// Plans from start to goal.  The start cell itself is always accepted even
  /// if its cost is above the obstacle threshold (the robot may be localised
  /// slightly inside the inflation ring); its neighbours are not.
  SearchResult plan(const GridView & grid, Cell start, Cell goal, const SearchParams & params);

  /// Remove collinear interior cells, keeping start, corners and goal.
  static std::vector<Cell> extractCorners(const std::vector<Cell> & cells);
  /// Number of heading changes along a 4-connected cell sequence.
  static int countTurns(const std::vector<Cell> & cells);

private:
  bool traversable(const GridView & grid, int x, int y, const SearchParams & p) const;
  bool findNearestFree(
    const GridView & grid, Cell goal, const SearchParams & p, Cell & out) const;

  // Buffers reused between calls (planner is called at ~1 Hz on the same map).
  std::vector<double> g_;
  std::vector<int32_t> parent_;
  std::vector<uint32_t> visit_stamp_;   // g_/parent_ valid iff == stamp_
  std::vector<uint32_t> closed_stamp_;  // closed iff == stamp_
  uint32_t stamp_{0};
};

}  // namespace scr_planner

#endif  // SCR_PLANNER__ORIENTED_ASTAR_HPP_
