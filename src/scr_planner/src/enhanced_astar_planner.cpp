// Copyright 2026 SCR Project

#include "scr_planner/enhanced_astar_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scr_planner
{

std::vector<Point2D> densifyPolyline(const std::vector<Point2D> & vertices, double step)
{
  std::vector<Point2D> out;
  if (vertices.empty()) {return out;}
  out.push_back(vertices.front());
  for (size_t i = 1; i < vertices.size(); ++i) {
    const Point2D & a = vertices[i - 1];
    const Point2D & b = vertices[i];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step - 1e-9)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      out.push_back(Point2D{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  return out;
}

std::vector<Point2D> axisAlignedPolyline(
  const std::vector<Point2D> & centres, const Point2D & start, const Point2D & goal)
{
  std::vector<Point2D> v;
  auto push = [&v](const Point2D & p) {
      if (v.empty() || std::hypot(p.x - v.back().x, p.y - v.back().y) > 1e-9) {v.push_back(p);}
    };
  push(start);
  if (centres.size() >= 2) {
    // Project the exact start onto the first grid segment: a perpendicular,
    // sub-cell hop keeps every segment parallel to an axis.
    const Point2D & c0 = centres[0];
    const Point2D & c1 = centres[1];
    const bool first_along_x = std::abs(c1.y - c0.y) < std::abs(c1.x - c0.x);
    push(first_along_x ? Point2D{start.x, c0.y} : Point2D{c0.x, start.y});
    for (size_t i = 1; i + 1 < centres.size(); ++i) {push(centres[i]);}
    const Point2D & a = centres[centres.size() - 2];
    const Point2D & b = centres.back();
    const bool last_along_x = std::abs(b.y - a.y) < std::abs(b.x - a.x);
    push(last_along_x ? Point2D{goal.x, b.y} : Point2D{b.x, goal.y});
  }
  push(goal);
  if (v.size() == 1) {v.push_back(goal);}  // start == goal: keep two poses
  return v;
}

void EnhancedAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("EnhancedAStarPlanner: parent node expired");}
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;
  using rclcpp::ParameterValue;
  declare_parameter_if_not_declared(node, name + ".turn_penalty", ParameterValue(2.5));
  declare_parameter_if_not_declared(node, name + ".use_cost_weight", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".cost_weight", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".allow_unknown", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".tolerance", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".max_planning_time", ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name + ".max_iterations", ParameterValue(0));
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", ParameterValue(false));

  params_.turn_penalty = node->get_parameter(name + ".turn_penalty").as_double();
  use_cost_weight_ = node->get_parameter(name + ".use_cost_weight").as_bool();
  cost_weight_ = node->get_parameter(name + ".cost_weight").as_double();
  params_.allow_unknown = node->get_parameter(name + ".allow_unknown").as_bool();
  tolerance_m_ = node->get_parameter(name + ".tolerance").as_double();
  params_.timeout_s = node->get_parameter(name + ".max_planning_time").as_double();
  params_.max_expansions =
    static_cast<size_t>(std::max<int64_t>(0, node->get_parameter(name + ".max_iterations").as_int()));
  use_final_approach_orientation_ =
    node->get_parameter(name + ".use_final_approach_orientation").as_bool();

  RCLCPP_INFO(
    logger_, "Configured %s (scr_planner/EnhancedAStarPlanner): turn_penalty=%.2f, "
    "cost_weight=%s%.2f, allow_unknown=%s, tolerance=%.2f m, max_planning_time=%.2f s",
    name_.c_str(), params_.turn_penalty, use_cost_weight_ ? "" : "off/", cost_weight_,
    params_.allow_unknown ? "true" : "false", tolerance_m_, params_.timeout_s);
}

void EnhancedAStarPlanner::activate()
{
  auto node = node_.lock();
  if (!node) {return;}
  param_cb_ = node->add_on_set_parameters_callback(
    std::bind(&EnhancedAStarPlanner::onParameterChange, this, std::placeholders::_1));
}

void EnhancedAStarPlanner::deactivate()
{
  auto node = node_.lock();
  if (node && param_cb_) {node->remove_on_set_parameters_callback(param_cb_.get());}
  param_cb_.reset();
}

void EnhancedAStarPlanner::cleanup()
{
  costmap_ros_.reset();
  costmap_ = nullptr;
}

rcl_interfaces::msg::SetParametersResult EnhancedAStarPlanner::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n.rfind(name_ + ".", 0) != 0) {continue;}
    const std::string key = n.substr(name_.size() + 1);
    if (key == "turn_penalty") {
      params_.turn_penalty = p.as_double();
    } else if (key == "use_cost_weight") {
      use_cost_weight_ = p.as_bool();
    } else if (key == "cost_weight") {
      cost_weight_ = p.as_double();
    } else if (key == "allow_unknown") {
      params_.allow_unknown = p.as_bool();
    } else if (key == "tolerance") {
      tolerance_m_ = p.as_double();
    } else if (key == "max_planning_time") {
      params_.timeout_s = p.as_double();
    } else if (key == "max_iterations") {
      params_.max_expansions = static_cast<size_t>(std::max<int64_t>(0, p.as_int()));
    } else if (key == "use_final_approach_orientation") {
      use_final_approach_orientation_ = p.as_bool();
    }
  }
  return result;
}

nav_msgs::msg::Path EnhancedAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> plan_lock(mutex_);
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "EnhancedAStarPlanner: start/goal must be in the global frame '" + global_frame_ +
            "' (got '" + start.header.frame_id + "' / '" + goal.header.frame_id + "')");
  }

  SearchResult result;
  double resolution = 0.0;
  std::vector<Point2D> vertices;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    resolution = costmap_->getResolution();
    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
      throw nav2_core::PlannerException("EnhancedAStarPlanner: start is outside the costmap");
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      throw nav2_core::PlannerException("EnhancedAStarPlanner: goal is outside the costmap");
    }
    GridView grid{costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
      costmap_->getCharMap()};
    SearchParams p = params_;
    p.cost_weight = use_cost_weight_ ? cost_weight_ : 0.0;
    p.goal_tolerance_cells = tolerance_m_ / resolution;
    const Cell goal_cell{static_cast<int>(gx), static_cast<int>(gy)};
    result = search_.plan(grid, Cell{static_cast<int>(sx), static_cast<int>(sy)}, goal_cell, p);

    if (!result.ok()) {
      throw nav2_core::PlannerException(
              "EnhancedAStarPlanner failed: " + toString(result.status) + " (" +
              std::to_string(result.expansions) + " expansions, " +
              std::to_string(result.time_ms) + " ms)");
    }

    std::vector<Point2D> centres;
    centres.reserve(result.corners.size());
    for (const Cell & c : result.corners) {
      double wx, wy;
      costmap_->mapToWorld(c.x, c.y, wx, wy);
      centres.push_back({wx, wy});
    }
    // If the goal had to be moved out of an obstacle (tolerance), end at the
    // adjusted cell centre instead of the requested position.
    Point2D goal_xy{goal.pose.position.x, goal.pose.position.y};
    if (result.goal != goal_cell) {goal_xy = centres.back();}
    vertices = axisAlignedPolyline(centres, {start.pose.position.x, start.pose.position.y}, goal_xy);
  }

  const std::vector<Point2D> dense = densifyPolyline(vertices, resolution);
  // Heading of each pose = direction of travel to the next pose.  The short
  // perpendicular hops joining the exact start/goal to the grid path inherit
  // the heading of the neighbouring long segment instead of pointing sideways.
  std::vector<double> yaw(dense.size(), 0.0);
  std::vector<bool> tiny(dense.size(), true);
  for (size_t i = 0; i + 1 < dense.size(); ++i) {
    const double dx = dense[i + 1].x - dense[i].x;
    const double dy = dense[i + 1].y - dense[i].y;
    yaw[i] = std::atan2(dy, dx);
    tiny[i] = std::hypot(dx, dy) < 0.5 * resolution;
  }
  for (size_t i = 0; i + 1 < dense.size(); ++i) {
    if (!tiny[i]) {continue;}
    if (i > 0) {
      yaw[i] = yaw[i - 1];
    } else if (dense.size() > 2 && !tiny[1]) {
      yaw[0] = yaw[1];
    }
  }
  if (dense.size() >= 2) {yaw.back() = yaw[dense.size() - 2];}

  path.poses.reserve(dense.size());
  for (size_t i = 0; i < dense.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = dense[i].x;
    ps.pose.position.y = dense[i].y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw[i]);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  if (!path.poses.empty() && !use_final_approach_orientation_) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  last_result_ = result;
  RCLCPP_INFO(
    logger_, "EnhancedAStar: %zu poses, %zu corners, %d turns, cost %.1f, %zu expansions, "
    "%.2f ms", path.poses.size(), result.corners.size(), result.turns, result.cost,
    result.expansions, result.time_ms);
  return path;
}

}  // namespace scr_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(scr_planner::EnhancedAStarPlanner, nav2_core::GlobalPlanner)
