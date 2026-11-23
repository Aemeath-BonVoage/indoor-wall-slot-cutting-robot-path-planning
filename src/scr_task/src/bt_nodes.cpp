// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
#include "scr_task/bt_nodes.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace scr_task
{

using BT::InputPort;
using BT::OutputPort;
using BT::BidirectionalPort;
using BT::NodeStatus;

double nowSeconds(const BT::Blackboard::Ptr & bb)
{
  // 执行器节点的时钟（仿真时间）。BtActionServer 放在黑板上的 "node" 是另建的客户端节点，
  // 其时钟不跟随 /clock（实测为墙钟），不能用于计时。
  rclcpp::Clock::SharedPtr clock;
  if (bb && bb->get(kClockKey, clock) && clock) {
    return clock->now().seconds();
  }
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace
{
template<class T>
T inputOr(const BT::TreeNode & n, const std::string & key, T fallback)
{
  T v;
  return n.getInput<T>(key, v) ? v : fallback;
}
}  // namespace

// ---------------------------------------------------------------------------
// 技能 action 节点
// ---------------------------------------------------------------------------
FineAlignAction::FineAlignAction(const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/fine_align", conf) {}

void FineAlignAction::on_tick()
{
  goal_.standoff = inputOr(*this, "standoff", 0.0);
  goal_.mode = inputOr<std::string>(*this, "mode", "");
  goal_.settle_time = inputOr(*this, "settle_time", 0.0);
}

NodeStatus FineAlignAction::on_success()
{
  setOutput("origin_residual", result_.result->origin_residual);
  setOutput("since_fix", 0.0);   // 对位以参考标记收敛：沿墙位置刚由标记确定
  return NodeStatus::SUCCESS;
}

BT::PortsList FineAlignAction::providedPorts()
{
  return providedBasicPorts(
    {
      InputPort<double>("standoff", "Target distance to the wall (m)"),
      InputPort<std::string>("mode", "visual | visual_v1 | odom (empty: node default)"),
      InputPort<double>("settle_time", "Wait before aligning (s)"),
      OutputPort<double>("origin_residual", "Along-wall residual of the parking pose (m)"),
      OutputPort<double>("since_fix", "Along-wall odometry travel since the last tag fix (m)")});
}

MoveToStationAction::MoveToStationAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/move_to_station", conf) {}

void MoveToStationAction::on_tick()
{
  goal_.station = inputOr(*this, "station", -1);
  goal_.lateral_offset = inputOr(*this, "lateral_offset", 0.0);
  goal_.standoff = inputOr(*this, "standoff", 0.0);
  goal_.max_speed = inputOr(*this, "max_speed", 0.0);
  goal_.timeout = inputOr(*this, "timeout", 0.0);
  setOutput("converged", false);
}

NodeStatus MoveToStationAction::on_success()
{
  setOutput("converged", static_cast<bool>(result_.result->converged));
  setOutput("arrived_at", nowSeconds(config().blackboard));
  setOutput("since_fix", result_.result->odom_since_fix);
  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToStationAction::providedPorts()
{
  return providedBasicPorts(
    {
      InputPort<int>("station", "Station index (status labelling)"),
      InputPort<double>("lateral_offset", "Target u relative to the wall origin (m)"),
      InputPort<double>("standoff", "Target distance to the wall (m)"),
      InputPort<double>("max_speed", "Speed limit (m/s, 0: default)"),
      InputPort<double>("timeout", "Timeout (s, 0: default)"),
      OutputPort<bool>("converged", "Converged by the arrival criterion"),
      OutputPort<double>("arrived_at", "Arrival time (s)"),
      OutputPort<double>("since_fix", "Along-wall odometry travel since the last tag fix (m)")});
}

CutStationAction::CutStationAction(const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/cut_station", conf) {}

void CutStationAction::on_tick()
{
  goal_.station = inputOr(*this, "station", -1);
  goal_.points = inputOr(*this, "points", ArmPoints{});
  goal_.point_duration = inputOr(*this, "point_duration", 2.0);
  goal_.compensate = inputOr(*this, "compensate", false);
  goal_.lateral_offset = inputOr(*this, "lateral_offset", 0.0);
  goal_.standoff = inputOr(*this, "standoff", 0.0);
  goal_.tool_offset = inputOr(*this, "tool_offset", 0.0);
  goal_.wall_gap = inputOr(*this, "wall_gap", 0.0);
  goal_.return_home = inputOr(*this, "return_home", false);
  if (goal_.points.empty()) {
    config().blackboard->set(kLastFailureKey, name() + ": no arm points");
    should_send_goal_ = false;
  }
}

NodeStatus CutStationAction::on_success()
{
  const auto & r = *result_.result;
  setOutput("cut_time", r.duration);
  setOutput("clamped", r.clamped);
  setOutput("sensed_lateral", r.sensed_lateral);
  setOutput("sensed_standoff", r.sensed_standoff);
  setOutput("sensed_yaw", r.sensed_yaw);
  return NodeStatus::SUCCESS;
}

BT::PortsList CutStationAction::providedPorts()
{
  return providedBasicPorts(
    {
      InputPort<int>("station", "Station index (-1: single-station task)"),
      InputPort<ArmPoints>("points", "Arm points in base_link"),
      InputPort<double>(
        "point_duration", "Duration of each arm segment (s); point-to-point mode only"),
      InputPort<bool>("compensate", "Measure the arrival residual and correct the points"),
      InputPort<double>("lateral_offset", "Reference u of this station (m)"),
      InputPort<double>("standoff", "Reference distance to the wall (m)"),
      InputPort<double>("tool_offset", "Tool tip offset from link4 (m)"),
      InputPort<double>("wall_gap", "Dry run: stop the deepest tip this far from the wall (m)"),
      InputPort<bool>("return_home", "Retract before and after the points"),
      OutputPort<double>("cut_time", "Execution time (s)"),
      OutputPort<int>("clamped", "Points clamped to joint limits"),
      OutputPort<double>("sensed_lateral", "Residual used for compensation (m)"),
      OutputPort<double>("sensed_standoff", "Residual used for compensation (m)"),
      OutputPort<double>("sensed_yaw", "Residual used for compensation (rad)")});
}

HoldBaseAction::HoldBaseAction(const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/hold_base", conf) {}

ScanMarkerAction::ScanMarkerAction(const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/scan_marker", conf) {}

void ScanMarkerAction::on_tick()
{
  goal_.samples = inputOr(*this, "samples", 5);
}

BT::PortsList ScanMarkerAction::providedPorts()
{
  return providedBasicPorts({InputPort<int>("samples", "Frames to average")});
}

RetractArmAction::RetractArmAction(const std::string & name, const BT::NodeConfiguration & conf)
: SkillActionNode(name, "/task_skills/retract_arm", conf) {}

void RetractArmAction::on_tick()
{
  goal_.lift = inputOr(*this, "lift", 0.0);
}

BT::PortsList RetractArmAction::providedPorts()
{
  return providedBasicPorts({InputPort<double>("lift", "Slide height joint1 (m)")});
}

// ---------------------------------------------------------------------------
// 流程节点
// ---------------------------------------------------------------------------
BT::PortsList LoadSlotPlan::providedPorts()
{
  return {
    InputPort<std::string>("plan_file", "Plan YAML written by plan_wall_slots.py"),
    InputPort<bool>("dry_run", "Dry run (tool tip stops in front of the wall)"),
    InputPort<double>("dry_run_gap", "Dry-run gap (m)"),
    InputPort<double>("marker_x", "Fiducial marker x in map (m)"),
    InputPort<double>("marker_y", "Fiducial marker y in map (m)"),
    InputPort<double>("nav_standoff", "Nav2 goal distance to the wall (m)"),
    OutputPort<SlotPlanPtr>("plan"),
    OutputPort<geometry_msgs::msg::PoseStamped>("marker_pose", "Nav2 goal in front of marker"),
    OutputPort<double>("standoff", "Station distance to the wall (m)"),
    OutputPort<double>("marker_u", "Wall coordinate of the marker (m)"),
    OutputPort<double>("tool_offset", "Tool tip offset from link4 (m)"),
    OutputPort<int>("station_count")};
}

NodeStatus LoadSlotPlan::tick()
{
  std::string file;
  if (!getInput("plan_file", file) || file.empty()) {
    config().blackboard->set(kLastFailureKey, name() + ": no plan_file");
    return NodeStatus::FAILURE;
  }
  std::optional<double> gap;
  if (inputOr(*this, "dry_run", false)) {
    gap = inputOr(*this, "dry_run_gap", 0.01);
  }
  std::shared_ptr<SlotPlan> plan;
  try {
    plan = std::make_shared<SlotPlan>(loadSlotPlan(file, gap));
  } catch (const std::exception & e) {
    config().blackboard->set(kLastFailureKey, name() + ": " + e.what());
    return NodeStatus::FAILURE;
  }
  // 墙面原点 = 定位标记：执行器以已知的标记坐标为准，规划文件中的 marker_u 只作一致性检查
  const double mu = plan->uOf(inputOr(*this, "marker_x", 8.42), inputOr(*this, "marker_y", 0.0));
  if (plan->marker_u && std::abs(*plan->marker_u - mu) > 1e-3) {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      RCLCPP_WARN(
        node->get_logger(), "plan marker_u %.3f differs from the executor marker (%.3f)",
        *plan->marker_u, mu);
    }
  }
  setOutput("plan", SlotPlanPtr(plan));
  setOutput("marker_pose", plan->wallPose(mu, inputOr(*this, "nav_standoff", 1.2)));
  setOutput("standoff", plan->standoff);
  setOutput("marker_u", mu);
  setOutput("tool_offset", plan->tool_offset);
  setOutput("station_count", static_cast<int>(plan->stations.size()));
  return NodeStatus::SUCCESS;
}

BT::PortsList LoadCuttingTask::providedPorts()
{
  return {
    InputPort<std::string>("task_file", "Single-station task YAML"),
    OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal"),
    OutputPort<bool>("align"), OutputPort<double>("standoff"),
    OutputPort<std::string>("mode"), OutputPort<bool>("scan"),
    OutputPort<ArmPoints>("points"), OutputPort<double>("point_duration"),
    OutputPort<bool>("return_home"), OutputPort<double>("wall_gap")};
}

NodeStatus LoadCuttingTask::tick()
{
  std::string file;
  if (!getInput("task_file", file) || file.empty()) {
    config().blackboard->set(kLastFailureKey, name() + ": no task_file");
    return NodeStatus::FAILURE;
  }
  CuttingTask t;
  try {
    t = loadCuttingTask(file);
  } catch (const std::exception & e) {
    config().blackboard->set(kLastFailureKey, name() + ": " + e.what());
    return NodeStatus::FAILURE;
  }
  setOutput("nav_goal", t.nav_goal);
  setOutput("align", t.align);
  setOutput("standoff", t.standoff);
  setOutput("mode", t.mode);
  setOutput("scan", t.scan);
  setOutput("points", t.points);
  setOutput("point_duration", t.point_duration);
  setOutput("return_home", t.return_home);
  setOutput("wall_gap", t.wall_gap);
  return NodeStatus::SUCCESS;
}

BT::PortsList ForEachStation::providedPorts()
{
  return {
    InputPort<SlotPlanPtr>("plan"),
    InputPort<double>("marker_u", "Wall coordinate of the origin (m)"),
    InputPort<double>("max_strafe", "Use Nav2 when the station gap exceeds this (m)"),
    InputPort<double>("nav_standoff", "Nav2 goal distance to the wall (m)"),
    OutputPort<int>("station"),
    OutputPort<double>("u_c", "Station wall coordinate (m)"),
    OutputPort<double>("station_u", "Station u relative to the origin (m)"),
    OutputPort<ArmPoints>("points"),
    OutputPort<double>("point_duration"),
    OutputPort<double>("cut_length"),
    OutputPort<double>("move_dist"),
    OutputPort<bool>("nav_leg", "Station is reached with a Nav2 leg first"),
    OutputPort<geometry_msgs::msg::PoseStamped>("nav_pose", "Nav2 goal of the leg"),
    OutputPort<double>("station_start", "Start time of this station (s)"),
    BidirectionalPort<int>("station_recoveries", "Recovery counter, reset per station")};
}

bool ForEachStation::startStation()
{
  SlotPlanPtr plan;
  if (!getInput("plan", plan) || !plan) {
    return false;
  }
  const auto & st = plan->stations.at(index_);
  const double marker_u = inputOr(*this, "marker_u", 0.0);
  const double gap = std::abs(st.u_c - u_prev_);
  const bool nav_leg = gap > inputOr(*this, "max_strafe", 1.5);
  setOutput("station", st.index);
  setOutput("u_c", st.u_c);
  setOutput("station_u", st.u_c - marker_u);
  setOutput("points", st.points);
  setOutput("point_duration", st.point_duration);
  setOutput("cut_length", st.cut_length);
  setOutput("move_dist", gap);
  setOutput("nav_leg", nav_leg);
  setOutput("nav_pose", plan->wallPose(st.u_c, inputOr(*this, "nav_standoff", 1.2)));
  setOutput("station_start", nowSeconds(config().blackboard));
  setOutput("station_recoveries", 0);
  return true;
}

NodeStatus ForEachStation::tick()
{
  SlotPlanPtr plan;
  if (!getInput("plan", plan) || !plan) {
    config().blackboard->set(kLastFailureKey, name() + ": no plan");
    return NodeStatus::FAILURE;
  }
  if (status() == NodeStatus::IDLE) {
    index_ = 0;
    started_ = false;
    u_prev_ = inputOr(*this, "marker_u", 0.0);
  }
  setStatus(NodeStatus::RUNNING);
  if (index_ >= plan->stations.size()) {
    index_ = 0;
    return NodeStatus::SUCCESS;
  }
  if (!started_) {
    startStation();
    started_ = true;
  }
  const NodeStatus child = child_node_->executeTick();
  switch (child) {
    case NodeStatus::RUNNING:
      return NodeStatus::RUNNING;
    case NodeStatus::SUCCESS:
      haltChild();
      u_prev_ = plan->stations[index_].u_c;
      ++index_;
      started_ = false;
      if (index_ >= plan->stations.size()) {
        index_ = 0;
        return NodeStatus::SUCCESS;
      }
      return NodeStatus::RUNNING;
    case NodeStatus::FAILURE:
    default: {
        std::string why;
        config().blackboard->get(kLastFailureKey, why);
        config().blackboard->set(
          kLastFailureKey,
          "station S" + std::to_string(plan->stations[index_].index) + " failed" +
          (why.empty() ? std::string() : " (" + why + ")"));
        haltChild();
        index_ = 0;
        started_ = false;
        return NodeStatus::FAILURE;
      }
  }
}

void ForEachStation::halt()
{
  index_ = 0;
  started_ = false;
  DecoratorNode::halt();
}

BT::PortsList RecordStation::providedPorts()
{
  return {
    InputPort<int>("station"), InputPort<double>("u_c"), InputPort<ArmPoints>("points"),
    InputPort<double>("move_dist"), InputPort<bool>("nav_leg"),
    InputPort<double>("station_start"), InputPort<double>("arrived_at"),
    InputPort<double>("cut_time"), InputPort<bool>("converged"),
    InputPort<double>("sensed_lateral"), InputPort<double>("sensed_standoff"),
    InputPort<double>("sensed_yaw"), InputPort<int>("clamped"),
    InputPort<double>("cut_length"), InputPort<double>("origin_residual"),
    InputPort<int>("recoveries"), InputPort<bool>("realigned"), InputPort<double>("since_fix")};
}

NodeStatus RecordStation::tick()
{
  StationRecordsPtr records;
  if (!config().blackboard->get(kRecordsKey, records) || !records) {
    return NodeStatus::SUCCESS;   // 没有记录容器（例如单元测试中）时只跳过
  }
  scr_interfaces::msg::StationRecord r;
  const double now = nowSeconds(config().blackboard);
  const double t0 = inputOr(*this, "station_start", now);
  r.station = inputOr(*this, "station", -1);
  r.u_c = inputOr(*this, "u_c", 0.0);
  r.points = static_cast<int>(inputOr(*this, "points", ArmPoints{}).size());
  r.move_dist_m = inputOr(*this, "move_dist", 0.0);
  r.nav_leg = inputOr(*this, "nav_leg", false);
  r.station_s = now - t0;
  r.move_s = inputOr(*this, "arrived_at", t0) - t0;
  r.cut_s = inputOr(*this, "cut_time", 0.0);
  r.moved = inputOr(*this, "converged", false);
  r.sensed_lateral_mm = inputOr(*this, "sensed_lateral", 0.0) * 1000.0;
  r.sensed_standoff_mm = inputOr(*this, "sensed_standoff", 0.0) * 1000.0;
  r.sensed_yaw_deg = inputOr(*this, "sensed_yaw", 0.0) * 180.0 / M_PI;
  r.clamped = inputOr(*this, "clamped", 0);
  r.cut_cm = inputOr(*this, "cut_length", 0.0) * 100.0;
  r.cut_ok = true;
  r.origin_residual_mm = inputOr(*this, "origin_residual", 0.0) * 1000.0;
  r.recoveries = inputOr(*this, "recoveries", 0);
  r.realigned = inputOr(*this, "realigned", false);
  r.since_fix_m = inputOr(*this, "since_fix", 0.0);
  records->push_back(r);
  return NodeStatus::SUCCESS;
}

BT::PortsList CountRecovery::providedPorts()
{
  return {
    BidirectionalPort<int>("station_count", "Per-station recovery counter"),
    BidirectionalPort<int>("total_count", "Recovery counter of the whole task")};
}

NodeStatus CountRecovery::tick()
{
  for (const char * key : {"station_count", "total_count"}) {
    int n = 0;
    getInput(key, n);
    setOutput(key, n + 1);
  }
  return NodeStatus::SUCCESS;
}

BT::PortsList CheckFlag::providedPorts()
{
  return {InputPort<bool>("value")};
}

NodeStatus CheckFlag::tick()
{
  return inputOr(*this, "value", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

bool realignDue(double since_fix, double move_dist, double threshold)
{
  return threshold > 0.0 && since_fix + std::abs(move_dist) > threshold;
}

BT::PortsList RealignDue::providedPorts()
{
  return {
    InputPort<double>("since_fix", "Along-wall odometry travel since the last tag fix (m)"),
    InputPort<double>("move_dist", "Strafe to the next station (m)"),
    InputPort<double>("threshold", "Re-align after this travel without a tag fix (m, <= 0: off)"),
    OutputPort<bool>("due", "Re-alignment was triggered")};
}

NodeStatus RealignDue::tick()
{
  const bool due = realignDue(
    inputOr(*this, "since_fix", 0.0), inputOr(*this, "move_dist", 0.0),
    inputOr(*this, "threshold", 0.0));
  setOutput("due", due);
  return due ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

void registerFlowNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<RealignDue>("RealignDue");
  factory.registerNodeType<LoadSlotPlan>("LoadSlotPlan");
  factory.registerNodeType<LoadCuttingTask>("LoadCuttingTask");
  factory.registerNodeType<ForEachStation>("ForEachStation");
  factory.registerNodeType<RecordStation>("RecordStation");
  factory.registerNodeType<CountRecovery>("CountRecovery");
  factory.registerNodeType<CheckFlag>("CheckFlag");
}

}  // namespace scr_task

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scr_task::FineAlignAction>("FineAlign");
  factory.registerNodeType<scr_task::MoveToStationAction>("MoveToStation");
  factory.registerNodeType<scr_task::CutStationAction>("CutStation");
  factory.registerNodeType<scr_task::HoldBaseAction>("HoldBase");
  factory.registerNodeType<scr_task::ScanMarkerAction>("ScanMarker");
  factory.registerNodeType<scr_task::RetractArmAction>("RetractArm");
  scr_task::registerFlowNodes(factory);
}
