// Copyright 2026 SCR Project
//
// Orientation-aware, segment-based A* for wall-chase (开槽) planning.
//
// A path is a sequence of axis-aligned straight segments; consecutive segments
// are perpendicular.  The planner minimises
//
//     J = sum over cells entered of (axis_weight + cost_weight * cell_cost / 252)
//         + turn_penalty * (number of bends)
//
// subject to construction rules that are properties of whole segments:
//   * an axis may be forbidden (no horizontal chase on load-bearing walls),
//   * a maximum straight length per axis (horizontal chase <= 500 mm),
//   * a conduit bend radius r: every leg adjacent to a bend is >= r long
//     (>= 2r between two bends) and the quarter-circle arc of radius r at each
//     bend lies in free space.
//
// Each A* edge is one complete straight segment, so the state is just
// (x, y, heading of the last segment) and the result is provably optimal for J
// under these constraints.  The Python prototype (src/path_planning/
// improved_astar.py) searched over (x, y) only, which is not optimal once a
// turn penalty is present, and could not express the segment rules at all.
//
// The library is ROS-free; a C API (scr_planner/c_api.h) exposes it to Python.

#ifndef SCR_PLANNER__ORIENTED_ASTAR_HPP_
#define SCR_PLANNER__ORIENTED_ASTAR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace scr_planner
{

/// Cell values follow nav2_costmap_2d conventions.
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

/// Non-owning, row-major view of a cost grid (index = y * width + x).
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
  /// Extra cost (in cells of travel) for every bend.
  double turn_penalty{4.0};
  /// Cost per cell travelled along x / y (> 0).  weight_x > weight_y makes
  /// vertical chases preferable to horizontal ones.
  double weight_x{1.0};
  double weight_y{1.0};
  /// Whether segments along x / y are allowed at all.
  bool allow_x{true};
  bool allow_y{true};
  /// Maximum length (cells) of one straight segment along x / y; 0 = no limit.
  int max_run_x{0};
  int max_run_y{0};
  /// Conduit bend radius in cells; 0 disables the bend constraints.
  int bend_radius{0};
  /// If > 0, each cell entered adds cost_weight * min(cost, 252) / 252.
  double cost_weight{0.0};
  /// Treat NO_INFORMATION (255) cells as traversable.
  bool allow_unknown{true};
  /// Cells with value >= this are not traversable.
  uint8_t obstacle_threshold{cost::kInscribed};
  /// If the goal cell is blocked, use the nearest free cell within this radius.
  double goal_tolerance_cells{0.0};
  /// Abort after this many seconds (<= 0: no limit).
  double timeout_s{0.0};
  /// Abort after this many expanded states (0: no limit).
  size_t max_expansions{0};
};

enum class Status
{
  kSuccess = 0,
  kStartOutOfBounds = 1,
  kGoalOutOfBounds = 2,
  kGoalBlocked = 3,
  kNoPath = 4,
  kTimeout = 5,
};

std::string toString(Status s);

struct SearchResult
{
  Status status{Status::kNoPath};
  /// Start, every bend, and the goal.
  std::vector<Cell> corners;
  /// Dense 4-connected cell sequence from start to goal.
  std::vector<Cell> cells;
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
  /// The start cell is always accepted even if its value is above the
  /// obstacle threshold (it is usually the edge of a junction box).
  SearchResult plan(const GridView & grid, Cell start, Cell goal, const SearchParams & params);

  /// Cells covered by the quarter arc of radius r at a bend located at the
  /// origin, entered along heading `from` and left along heading `to`
  /// (headings: 0 = +x, 1 = +y, 2 = -x, 3 = -y; must be perpendicular).
  /// Offsets are relative to the bend cell and include both tangent points.
  static std::vector<Cell> arcCells(int from, int to, int r);

  /// Remove collinear interior cells, keeping start, corners and goal.
  static std::vector<Cell> extractCorners(const std::vector<Cell> & cells);
  /// Dense cell sequence through axis-aligned corners.
  static std::vector<Cell> densify(const std::vector<Cell> & corners);
  /// Number of heading changes along a 4-connected cell sequence.
  static int countTurns(const std::vector<Cell> & cells);

private:
  bool traversable(const GridView & grid, int x, int y, const SearchParams & p) const;
  bool findNearestFree(
    const GridView & grid, Cell goal, const SearchParams & p, Cell & out) const;

  // Buffers reused between calls.
  std::vector<double> g_;
  std::vector<int32_t> parent_;
  std::vector<uint32_t> visit_stamp_;   // g_/parent_ valid iff == stamp_
  std::vector<uint32_t> closed_stamp_;  // closed iff == stamp_
  uint32_t stamp_{0};
};

}  // namespace scr_planner

#endif  // SCR_PLANNER__ORIENTED_ASTAR_HPP_
