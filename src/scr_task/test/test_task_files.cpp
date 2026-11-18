// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 任务文件解析：对照实际的 11 站规划文件与单站任务文件检查（与 Python 侧
// scr_arm_controller.slot_execution 的规则一致）.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>

#include "scr_task/task_files.hpp"

namespace
{

std::string repo(const std::string & rel)
{
  const char * root = std::getenv("SCR_REPO_ROOT");
  return std::string(root ? root : "../..") + "/" + rel;
}

const char * kPlan = "docs/experiments/slot_execution/runs/dp/slot_plan.yaml";

double yawOf(const geometry_msgs::msg::PoseStamped & p)
{
  return 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
}

}  // namespace

TEST(TaskFiles, SlotPlanStationsAndPoses)
{
  const auto plan = scr_task::loadSlotPlan(repo(kPlan));
  const YAML::Node d = YAML::LoadFile(repo(kPlan));
  ASSERT_EQ(plan.stations.size(), d["stations"].size());
  ASSERT_EQ(plan.stations.size(), 11u);
  ASSERT_TRUE(plan.marker_u.has_value());
  EXPECT_NEAR(*plan.marker_u, 1.505, 1e-9);
  // 停靠位姿与规划器写出的 pose 一致
  for (size_t i = 0; i < plan.stations.size(); ++i) {
    const auto & st = plan.stations[i];
    const auto ref = d["stations"][i]["pose"];
    const auto p = plan.wallPose(st.u_c, plan.standoff);
    EXPECT_NEAR(p.pose.position.x, ref["x"].as<double>(), 2e-4);
    EXPECT_NEAR(p.pose.position.y, ref["y"].as<double>(), 2e-4);
    EXPECT_NEAR(yawOf(p), ref["yaw"].as<double>(), 2e-4);
    EXPECT_NEAR(plan.uOf(p.pose.position.x, p.pose.position.y), st.u_c, 1e-9);
    // 每站先进刀、后退刀：首末点不在切深，至少两点在切深
    int depth = 0;
    for (const auto & a : st.points) {
      depth += a.at_depth;
    }
    EXPECT_GE(depth, 2);
    EXPECT_FALSE(st.points.front().at_depth);
    EXPECT_FALSE(st.points.back().at_depth);
  }
  // 标记位置 (8.42, -3.9245) 的墙面坐标与规划文件中的 marker_u 一致
  EXPECT_NEAR(plan.uOf(8.42, -3.9245), 1.505, 1e-3);
}

TEST(TaskFiles, DryRunMovesOnlyDepthPoints)
{
  const double gap = 0.01;
  const auto plan = scr_task::loadSlotPlan(repo(kPlan));
  const auto dry = scr_task::loadSlotPlan(repo(kPlan), gap);
  for (size_t i = 0; i < plan.stations.size(); ++i) {
    for (size_t k = 0; k < plan.stations[i].points.size(); ++k) {
      const auto & a = plan.stations[i].points[k];
      const auto & b = dry.stations[i].points[k];
      EXPECT_EQ(a.at_depth, b.at_depth);
      if (a.at_depth) {
        EXPECT_NEAR(b.x + plan.tool_offset, plan.standoff - gap, 1e-12);
      } else {
        EXPECT_DOUBLE_EQ(a.x, b.x);
      }
      EXPECT_DOUBLE_EQ(a.y, b.y);
      EXPECT_DOUBLE_EQ(a.z, b.z);
    }
  }
}

TEST(TaskFiles, RejectsFailedPlan)
{
  const std::string path = "/tmp/scr_task_failed_plan.yaml";
  {
    std::ofstream f(path);
    f << "status: failure\nwall: {left_end: [0, 0], right_end: [1, 0]}\nstations: []\n";
  }
  EXPECT_THROW(scr_task::loadSlotPlan(path), std::runtime_error);
  EXPECT_THROW(
    scr_task::loadSlotPlan(repo("src/scr_arm_controller/config/task_at_target.yaml")),
    std::runtime_error);
}

TEST(TaskFiles, SingleStationTask)
{
  const auto t =
    scr_task::loadCuttingTask(repo("src/scr_arm_controller/config/task_at_target.yaml"));
  EXPECT_NEAR(t.nav_goal.pose.position.x, 8.61277, 1e-9);
  EXPECT_NEAR(yawOf(t.nav_goal), -1.64894, 1e-9);
  EXPECT_TRUE(t.align);
  EXPECT_EQ(t.mode, "visual");
  EXPECT_NEAR(t.standoff, 0.70, 1e-12);
  EXPECT_TRUE(t.scan);
  EXPECT_EQ(t.points.size(), 17u);
  EXPECT_NEAR(t.points[5].rotation, M_PI / 2.0, 1e-12);
  EXPECT_NEAR(t.point_duration, 2.0, 1e-12);
  EXPECT_TRUE(t.return_home);
  EXPECT_NEAR(t.wall_gap, 0.005, 1e-12);
}
