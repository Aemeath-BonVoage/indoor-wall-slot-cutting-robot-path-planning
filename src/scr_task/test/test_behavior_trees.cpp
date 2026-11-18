// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 行为树结构测试：用同名、同端口的模拟技能节点代替 action 客户端，检查逐站执行、
// 恢复路径（重新对位、切割重试、Nav2 段）与失败报告。不需要 ROS 通信。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include "scr_task/bt_nodes.hpp"

namespace
{

using BT::NodeStatus;
using scr_task::ArmPoints;

// 模拟节点的调用记录与故障注入
struct Script
{
  std::map<std::string, int> calls;                    // 按注册 ID 计数
  std::map<std::string, int> calls_by_name;            // 按 XML 中的节点名计数
  std::map<std::string, std::map<int, int>> failures;  // ID → 停靠站 → 剩余注入失败次数
  std::vector<double> move_offsets;
  std::vector<ArmPoints> cut_points;
  std::vector<bool> cut_compensate;
  int hold_halted{0};
};
Script g;

class MockSkill : public BT::StatefulActionNode
{
public:
  MockSkill(const std::string & name, const BT::NodeConfiguration & conf, std::string id)
  : BT::StatefulActionNode(name, conf), id_(std::move(id)) {}

  NodeStatus onStart() override
  {
    ++g.calls[id_];
    ++g.calls_by_name[name()];
    if (id_ == "HoldBase") {
      return NodeStatus::RUNNING;
    }
    int station = -1;
    getInput("station", station);
    if (id_ == "MoveToStation") {
      double u = 0.0;
      getInput("lateral_offset", u);
      g.move_offsets.push_back(u);
    }
    if (id_ == "CutStation") {
      ArmPoints pts;
      getInput("points", pts);
      g.cut_points.push_back(pts);
      bool comp = false;
      getInput("compensate", comp);
      g.cut_compensate.push_back(comp);
    }
    auto & left = g.failures[id_][station];
    if (left > 0) {
      --left;
      return NodeStatus::FAILURE;
    }
    if (id_ == "MoveToStation") {
      setOutput("converged", true);
      setOutput("arrived_at", scr_task::nowSeconds(config().blackboard));
      // 单个标记、离开标记后看不到它：到站时的无标记行程 = 到墙面原点的距离
      double u = 0.0;
      getInput("lateral_offset", u);
      setOutput("since_fix", std::abs(u));
    } else if (id_ == "CutStation") {
      setOutput("cut_time", 1.0);
      setOutput("clamped", 0);
      setOutput("sensed_lateral", 0.001);
      setOutput("sensed_standoff", 0.0);
      setOutput("sensed_yaw", 0.0);
    } else if (id_ == "FineAlign") {
      setOutput("origin_residual", 0.0002);
      setOutput("since_fix", 0.0);
    }
    return NodeStatus::SUCCESS;
  }
  NodeStatus onRunning() override {return NodeStatus::RUNNING;}
  void onHalted() override {++g.hold_halted;}

private:
  std::string id_;
};

template<class Real>
void registerMock(BT::BehaviorTreeFactory & f, const std::string & id)
{
  BT::NodeBuilder builder = [id](const std::string & name, const BT::NodeConfiguration & c) {
      return std::make_unique<MockSkill>(name, c, id);
    };
  f.registerBuilder(
    BT::TreeNodeManifest{BT::NodeType::ACTION, id, Real::providedPorts(), ""},
    builder);
}

std::string env(const char * key, const char * fallback)
{
  const char * v = std::getenv(key);
  return v ? v : fallback;
}

class BehaviorTrees : public ::testing::Test
{
protected:
  void SetUp() override
  {
    g = Script{};
    scr_task::registerFlowNodes(factory_);
    registerMock<scr_task::FineAlignAction>(factory_, "FineAlign");
    registerMock<scr_task::MoveToStationAction>(factory_, "MoveToStation");
    registerMock<scr_task::CutStationAction>(factory_, "CutStation");
    registerMock<scr_task::HoldBaseAction>(factory_, "HoldBase");
    registerMock<scr_task::ScanMarkerAction>(factory_, "ScanMarker");
    registerMock<scr_task::RetractArmAction>(factory_, "RetractArm");
    registerMock<nav2_behavior_tree::NavigateToPoseAction>(factory_, "NavigateToPose");
    bb_ = BT::Blackboard::create();
    records_ = std::make_shared<scr_task::StationRecords>();
    bb_->set<scr_task::StationRecordsPtr>(scr_task::kRecordsKey, records_);
    bb_->set<std::string>(scr_task::kLastFailureKey, "");
    bb_->set<int>("recoveries", 0);
    bb_->set<int>("station_recoveries", 0);
    bb_->set<double>("marker_x", 8.42);
    bb_->set<double>("marker_y", -3.9245);
    bb_->set<double>("nav_standoff", 1.2);
  }

  NodeStatus runPlan(bool dry_run = true, double max_strafe = 1.5, double realign = 0.0)
  {
    bb_->set<double>("realign_distance", realign);
    bb_->set<std::string>(
      "plan_file", env("SCR_REPO_ROOT", "../..") +
      std::string("/docs/experiments/slot_execution/runs/dp/slot_plan.yaml"));
    bb_->set<bool>("dry_run", dry_run);
    bb_->set<double>("dry_run_gap", 0.01);
    bb_->set<bool>("compensate", true);
    bb_->set<double>("max_strafe", max_strafe);
    return run("slot_plan.xml");
  }

  NodeStatus run(const std::string & file)
  {
    auto tree = factory_.createTreeFromFile(env("SCR_TASK_BT_DIR", ".") + "/" + file, bb_);
    NodeStatus s = NodeStatus::RUNNING;
    for (int i = 0; i < 10000 && s == NodeStatus::RUNNING; ++i) {
      s = tree.tickRoot();
    }
    return s;
  }

  int recoveries() const {return bb_->get<int>("recoveries");}

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
  scr_task::StationRecordsPtr records_;
};

TEST_F(BehaviorTrees, SlotPlanNominal)
{
  ASSERT_EQ(runPlan(), NodeStatus::SUCCESS);
  ASSERT_EQ(records_->size(), 11u);
  EXPECT_EQ(g.calls["NavigateToPose"], 1);
  EXPECT_EQ(g.calls["FineAlign"], 1);
  EXPECT_EQ(g.calls["MoveToStation"], 11);
  EXPECT_EQ(g.calls["CutStation"], 11);
  EXPECT_EQ(g.calls["HoldBase"], 11);
  EXPECT_EQ(g.hold_halted, 11);               // 每站切割结束时停止定点保持
  EXPECT_EQ(g.calls_by_name["RetractAtEnd"], 1);
  EXPECT_EQ(recoveries(), 0);
  // 横移目标 = u_c − marker_u（墙面原点为定位标记）
  EXPECT_NEAR(g.move_offsets.front(), 1.2749 - 1.505, 1e-3);
  EXPECT_NEAR(g.move_offsets.back(), 2.4100 - 1.505, 2e-3);
  for (size_t i = 0; i < records_->size(); ++i) {
    const auto & r = (*records_)[i];
    EXPECT_EQ(r.station, static_cast<int>(i));
    EXPECT_FALSE(r.nav_leg);
    EXPECT_TRUE(r.moved);
    EXPECT_EQ(r.recoveries, 0);
    EXPECT_NEAR(r.sensed_lateral_mm, 1.0, 1e-9);
    EXPECT_NEAR(r.origin_residual_mm, 0.2, 1e-9);
    EXPECT_TRUE(g.cut_compensate[i]);
  }
  // 空切：切深处的点 x = standoff − gap − tool_offset
  for (const auto & pts : g.cut_points) {
    for (const auto & p : pts) {
      if (p.at_depth) {
        EXPECT_NEAR(p.x, 0.7 - 0.01 - 0.1594, 1e-9);
      }
    }
  }
}

TEST_F(BehaviorTrees, StationFailureTriggersRealignment)
{
  g.failures["MoveToStation"][3] = 1;
  ASSERT_EQ(runPlan(), NodeStatus::SUCCESS);
  ASSERT_EQ(records_->size(), 11u);
  EXPECT_EQ(g.calls["FineAlign"], 2);           // 起始对位 + 恢复时重新对位
  EXPECT_EQ(g.calls_by_name["ReturnToMarker"], 1);
  EXPECT_EQ(g.calls["MoveToStation"], 12);
  EXPECT_EQ((*records_)[3].recoveries, 1);
  EXPECT_EQ((*records_)[4].recoveries, 0);
  EXPECT_EQ(recoveries(), 1);
}

TEST_F(BehaviorTrees, CutFailureIsRetriedOnce)
{
  g.failures["CutStation"][5] = 1;
  ASSERT_EQ(runPlan(), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls["CutStation"], 12);
  EXPECT_EQ(g.calls["FineAlign"], 1);
  EXPECT_EQ(records_->size(), 11u);
}

TEST_F(BehaviorTrees, PersistentCutFailureAbortsWithReport)
{
  g.failures["CutStation"][2] = 2;
  ASSERT_EQ(runPlan(), NodeStatus::FAILURE);
  EXPECT_EQ(records_->size(), 2u);               // S0、S1 已完成
  EXPECT_EQ(g.hold_halted, 3);                   // 失败站的定点保持也被停止
  EXPECT_EQ(g.calls_by_name["RetractAtEnd"], 0);
  const auto why = bb_->get<std::string>(scr_task::kLastFailureKey);
  EXPECT_NE(why.find("station S2 failed"), std::string::npos) << why;
}

TEST_F(BehaviorTrees, AlignmentFailureReapproachesMarker)
{
  g.failures["FineAlign"][-1] = 1;
  ASSERT_EQ(runPlan(), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls["FineAlign"], 2);
  EXPECT_EQ(g.calls_by_name["ReapproachMarker"], 1);
  EXPECT_EQ(recoveries(), 1);
}

TEST_F(BehaviorTrees, FarStationsUseNav2Leg)
{
  ASSERT_EQ(runPlan(true, 0.05), NodeStatus::SUCCESS);  // 站距 0.055–0.23 m，全部超过 0.05 m
  EXPECT_EQ(g.calls_by_name["NavigateToStation"], 11);
  for (const auto & r : *records_) {
    EXPECT_TRUE(r.nav_leg);
  }
}

TEST(RealignTrigger, ThresholdOnTravelPlusNextMove)
{
  EXPECT_FALSE(scr_task::realignDue(0.2, 0.05, 0.3));
  EXPECT_TRUE(scr_task::realignDue(0.2, 0.12, 0.3));
  EXPECT_TRUE(scr_task::realignDue(0.2, -0.12, 0.3));   // 横移方向不影响行程
  EXPECT_FALSE(scr_task::realignDue(5.0, 1.0, 0.0));    // 阈值 ≤ 0：关闭
  EXPECT_FALSE(scr_task::realignDue(5.0, 1.0, -1.0));
}

TEST_F(BehaviorTrees, RealignIsOffByDefault)
{
  ASSERT_EQ(runPlan(), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls_by_name["StrafeToMarker"], 0);
  for (const auto & r : *records_) {
    EXPECT_FALSE(r.realigned);
  }
}

TEST_F(BehaviorTrees, TravelWithoutTagFixTriggersRealignment)
{
  // 站相对标记 −0.23 … +0.905 m，相邻 0.055–0.12 m；阈值 0.3 m：
  // S0–S4 的无标记行程 + 下一段横移 ≤ 0.3 m，从 S5（0.185 + 0.12 > 0.3）起每站都先回标记
  ASSERT_EQ(runPlan(true, 1.5, 0.3), NodeStatus::SUCCESS);
  ASSERT_EQ(records_->size(), 11u);
  EXPECT_EQ(g.calls_by_name["StrafeToMarker"], 6);
  EXPECT_EQ(g.calls["FineAlign"], 7);
  EXPECT_EQ(g.calls["MoveToStation"], 11 + 6);
  EXPECT_EQ(recoveries(), 0);                   // 计划内的重新对位不计为恢复
  for (size_t i = 0; i < records_->size(); ++i) {
    const auto & r = (*records_)[i];
    EXPECT_EQ(r.realigned, i >= 5) << "S" << i;
    EXPECT_NEAR(r.since_fix_m, std::abs(r.u_c - 1.505), 2e-3);
  }
  // 重新对位后横移的目标仍是该站（墙面原点不变）
  EXPECT_NEAR(g.move_offsets.back(), 2.4100 - 1.505, 2e-3);
}

TEST_F(BehaviorTrees, InvalidPlanFailsBeforeMoving)
{
  bb_->set<std::string>("plan_file", "/nonexistent/plan.yaml");
  bb_->set<bool>("dry_run", false);
  bb_->set<double>("dry_run_gap", 0.01);
  bb_->set<bool>("compensate", true);
  bb_->set<double>("max_strafe", 1.5);
  ASSERT_EQ(run("slot_plan.xml"), NodeStatus::FAILURE);
  EXPECT_EQ(g.calls["NavigateToPose"], 0);
  EXPECT_NE(
    bb_->get<std::string>(scr_task::kLastFailureKey).find("LoadSlotPlan"), std::string::npos);
}

class CuttingTaskTree : public BehaviorTrees
{
protected:
  NodeStatus runTask(const std::string & yaml_text)
  {
    const std::string path = "/tmp/scr_task_test_task.yaml";
    FILE * f = std::fopen(path.c_str(), "w");
    std::fputs(yaml_text.c_str(), f);
    std::fclose(f);
    bb_->set<std::string>("task_file", path);
    return run("cutting_task.xml");
  }
  static std::string task(bool align, bool scan)
  {
    return std::string("navigation: {x: 8.6, y: -2.0, yaw: -1.6}\n") +
           "precise_positioning: {enabled: " + (align ? "true" : "false") +
           ", distance_from_marker: 0.7}\n" +
           "scanning: {enabled: " + (scan ? "true" : "false") + "}\n" +
           "points:\n  - {x: 0.52, y: 0.0, z: 0.5, rotation: 0}\n"
           "  - {x: 0.55, y: 0.0, z: 0.5, rotation: 90}\n"
           "motion: {duration: 2.0, return_home: true, dry_run_gap: 0.005}\n";
  }
};

TEST_F(CuttingTaskTree, Nominal)
{
  ASSERT_EQ(runTask(task(true, true)), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls["NavigateToPose"], 1);
  EXPECT_EQ(g.calls["FineAlign"], 1);
  EXPECT_EQ(g.calls["ScanMarker"], 1);
  EXPECT_EQ(g.calls["CutStation"], 1);
  EXPECT_EQ(g.hold_halted, 1);
  ASSERT_EQ(g.cut_points.size(), 1u);
  EXPECT_EQ(g.cut_points[0].size(), 2u);
}

TEST_F(CuttingTaskTree, ScanFailureDoesNotStopCutting)
{
  g.failures["ScanMarker"][-1] = 1;
  ASSERT_EQ(runTask(task(true, true)), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls["CutStation"], 1);
}

TEST_F(CuttingTaskTree, AlignmentAndScanCanBeDisabled)
{
  ASSERT_EQ(runTask(task(false, false)), NodeStatus::SUCCESS);
  EXPECT_EQ(g.calls["FineAlign"], 0);
  EXPECT_EQ(g.calls["ScanMarker"], 0);
  EXPECT_EQ(g.calls["CutStation"], 1);
}

TEST_F(CuttingTaskTree, PersistentAlignmentFailureAbortsBeforeCutting)
{
  g.failures["FineAlign"][-1] = 2;
  ASSERT_EQ(runTask(task(true, true)), NodeStatus::FAILURE);
  EXPECT_EQ(g.calls["FineAlign"], 2);
  EXPECT_EQ(g.calls["CutStation"], 0);
}

}  // namespace
