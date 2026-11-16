// Copyright 2026 SCR Project
//
// Nav2 global planner plugin wrapping OrientedAStar.
// Produces axis-aligned ("横平竖直") paths with a provably minimal
// length + turn_penalty * turns cost on the Nav2 costmap.

#ifndef SCR_PLANNER__ENHANCED_ASTAR_PLANNER_HPP_
#define SCR_PLANNER__ENHANCED_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "scr_planner/oriented_astar.hpp"
#include "tf2_ros/buffer.h"

namespace scr_planner
{

struct Point2D
{
  double x;
  double y;
};

/// Densify a polyline: consecutive output points are at most `step` apart.
/// Every input vertex is kept, so corners stay exact.
std::vector<Point2D> densifyPolyline(const std::vector<Point2D> & vertices, double step);

/// Turn the corner cell centres of a grid path into a world polyline from the
/// exact start to the exact goal in which every segment is axis-aligned: the
/// start/goal are joined to the first/last grid segment by a perpendicular
/// hop of at most half a cell.
std::vector<Point2D> axisAlignedPolyline(
  const std::vector<Point2D> & centres, const Point2D & start, const Point2D & goal);

class EnhancedAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  EnhancedAStarPlanner() = default;
  ~EnhancedAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /// Statistics of the last successful plan (for tests / introspection).
  const SearchResult & lastResult() const {return last_result_;}

private:
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("EnhancedAStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mutex_;
  OrientedAStar search_;
  SearchParams params_;
  double tolerance_m_{0.0};
  bool use_cost_weight_{true};
  double cost_weight_{1.0};
  bool use_final_approach_orientation_{false};
  SearchResult last_result_;
};

}  // namespace scr_planner

#endif  // SCR_PLANNER__ENHANCED_ASTAR_PLANNER_HPP_
