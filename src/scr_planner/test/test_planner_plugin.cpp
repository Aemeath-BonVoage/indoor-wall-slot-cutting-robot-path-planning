// Copyright 2026 SCR Project
//
// Integration test of the Nav2 plugin against a real Costmap2DROS.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "scr_planner/enhanced_astar_planner.hpp"
#include "tf2/utils.h"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

const auto * const kEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw, std::string frame = "map")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

class PluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("scr_planner_test");
    node_->declare_parameter("GridBased.tolerance", 0.0);
    node_->declare_parameter("GridBased.use_cost_weight", false);
    node_->declare_parameter("GridBased.turn_penalty", 3.0);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    // 4 m x 3 m at 5 cm, origin (0, 0), all free.
    costmap_ = costmap_ros_->getCostmap();
    costmap_->resizeMap(80, 60, 0.05, 0.0, 0.0);
    costmap_->resetMap(0, 0, 80, 60);

    planner_ = std::make_unique<scr_planner::EnhancedAStarPlanner>();
    planner_->configure(node_, "GridBased", nullptr, costmap_ros_);
    planner_->activate();
  }

  void TearDown() override
  {
    planner_->deactivate();
    planner_->cleanup();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
  }

  void wallAtX(double wx, double y0, double y1)
  {
    unsigned int mx, my0, my1, tmp;
    costmap_->worldToMap(wx, y0, mx, my0);
    costmap_->worldToMap(wx, y1, tmp, my1);
    for (unsigned int y = my0; y <= my1; ++y) {
      costmap_->setCost(mx, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<scr_planner::EnhancedAStarPlanner> planner_;
};

}  // namespace

TEST(Densify, KeepsVerticesAndSpacing)
{
  const auto out = scr_planner::densifyPolyline({{0, 0}, {1.0, 0}, {1.0, 0.33}}, 0.05);
  ASSERT_GE(out.size(), 3u);
  EXPECT_DOUBLE_EQ(out.front().x, 0.0);
  EXPECT_DOUBLE_EQ(out.back().y, 0.33);
  bool has_corner = false;
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_LE(std::hypot(out[i].x - out[i - 1].x, out[i].y - out[i - 1].y), 0.05 + 1e-9);
    if (std::abs(out[i].x - 1.0) < 1e-12 && std::abs(out[i].y) < 1e-12) {has_corner = true;}
  }
  EXPECT_TRUE(has_corner);
}

TEST_F(PluginTest, DensePathAroundWall)
{
  wallAtX(2.0, 0.0, 2.2);  // wall from the bottom edge up to y = 2.2 m
  const auto start = makePose(0.5, 0.5, 0.0);
  const auto goal = makePose(3.5, 0.5, M_PI / 2);
  const auto path = planner_->createPlan(start, goal);

  ASSERT_GT(path.poses.size(), 10u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_NEAR(path.poses.front().pose.position.x, 0.5, 1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y, 0.5, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.x, 3.5, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, 0.5, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), M_PI / 2, 1e-6);

  const auto & r = planner_->lastResult();
  EXPECT_EQ(r.turns, 2);  // up, across above the wall, down

  double max_y = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), 0.05 + 1e-9) << "gap at " << i;
    // Every hop is axis-aligned, including the ones from/to the exact start/goal.
    EXPECT_TRUE(std::abs(b.x - a.x) < 1e-9 || std::abs(b.y - a.y) < 1e-9) << i;
    unsigned int mx, my;
    ASSERT_TRUE(costmap_->worldToMap(b.x, b.y, mx, my));
    EXPECT_LT(costmap_->getCost(mx, my), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    max_y = std::max(max_y, b.y);
    // Orientation of intermediate poses follows the direction of travel.
    if (i + 1 < path.poses.size() && std::hypot(b.x - a.x, b.y - a.y) > 0.025) {
      const double yaw = tf2::getYaw(path.poses[i - 1].pose.orientation);
      EXPECT_NEAR(std::cos(yaw), (b.x - a.x) / std::hypot(b.x - a.x, b.y - a.y), 1e-6);
    }
  }
  EXPECT_GT(max_y, 2.2);
}

TEST_F(PluginTest, ThrowsWhenGoalUnreachable)
{
  wallAtX(2.0, 0.0, 2.95);  // full-height wall splits the map
  EXPECT_THROW(
    planner_->createPlan(makePose(0.5, 0.5, 0), makePose(3.5, 0.5, 0)),
    nav2_core::PlannerException);
}

TEST_F(PluginTest, ThrowsOnBlockedGoalWithoutTolerance)
{
  unsigned int mx, my;
  costmap_->worldToMap(3.0, 1.0, mx, my);
  costmap_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(
    planner_->createPlan(makePose(0.5, 0.5, 0), makePose(3.0, 1.0, 0)),
    nav2_core::PlannerException);

  // With a tolerance the goal snaps to the nearest free cell.
  node_->set_parameter(rclcpp::Parameter("GridBased.tolerance", 0.2));
  const auto path = planner_->createPlan(makePose(0.5, 0.5, 0), makePose(3.0, 1.0, 0));
  ASSERT_FALSE(path.poses.empty());
  const auto & last = path.poses.back().pose.position;
  EXPECT_LT(std::hypot(last.x - 3.0, last.y - 1.0), 0.2);
}

TEST_F(PluginTest, ThrowsOnWrongFrameAndOutsideMap)
{
  EXPECT_THROW(
    planner_->createPlan(makePose(0.5, 0.5, 0, "odom"), makePose(1.0, 1.0, 0)),
    nav2_core::PlannerException);
  EXPECT_THROW(
    planner_->createPlan(makePose(0.5, 0.5, 0), makePose(10.0, 1.0, 0)),
    nav2_core::PlannerException);
}

TEST_F(PluginTest, DynamicTurnPenaltyIsApplied)
{
  // Open map, diagonal goal: any manhattan path is shortest; exactly one turn.
  const auto path = planner_->createPlan(makePose(0.5, 0.5, 0), makePose(3.0, 2.5, 0));
  EXPECT_EQ(planner_->lastResult().turns, 1);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("GridBased.turn_penalty", 0.0))
    .successful);
  planner_->createPlan(makePose(0.5, 0.5, 0), makePose(3.0, 2.5, 0));
  EXPECT_DOUBLE_EQ(
    planner_->lastResult().cost,
    static_cast<double>(planner_->lastResult().cells.size() - 1));
  (void)path;
}
