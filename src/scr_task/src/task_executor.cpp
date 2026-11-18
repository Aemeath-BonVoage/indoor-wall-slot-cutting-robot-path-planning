// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
#include "scr_task/task_executor.hpp"

#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace scr_task
{

using nav2_util::CallbackReturn;
using nav2_behavior_tree::BtStatus;

std::string runningLeaves(const BT::Tree & tree)
{
  std::string out;
  for (const auto & n : tree.nodes) {
    if (n->status() == BT::NodeStatus::RUNNING && n->type() == BT::NodeType::ACTION) {
      out += (out.empty() ? "" : "+") + n->name();
    }
  }
  return out;
}

TaskExecutor::TaskExecutor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("task_executor", "", options)
{
  const std::string share = ament_index_cpp::get_package_share_directory("scr_task");
  declare_parameter("slot_plan_bt", share + "/behavior_trees/slot_plan.xml");
  declare_parameter("cutting_task_bt", share + "/behavior_trees/cutting_task.xml");
  declare_parameter(
    "plugin_lib_names",
    std::vector<std::string>{"nav2_navigate_to_pose_action_bt_node", "scr_task_bt_nodes"});
  // 定位标记（墙面坐标原点）在 map 中的位置（scr_description/worlds/room.world）
  declare_parameter("marker_x", 8.42);
  declare_parameter("marker_y", -3.9245);
  declare_parameter("nav_standoff", 1.2);
  // 自上次接受标记更新以来的沿墙行程超过该值（米）时，去下一站前先回参考标记重新对位；
  // ≤ 0 关闭（见 docs/experiments/along_wall_accuracy）
  declare_parameter("realign_distance", 0.0);
}

std::vector<std::string> TaskExecutor::requiredActions()
{
  return {"/navigate_to_pose", "/task_skills/fine_align", "/task_skills/move_to_station",
    "/task_skills/cut_station", "/task_skills/hold_base", "/task_skills/scan_marker",
    "/task_skills/retract_arm"};
}

CallbackReturn TaskExecutor::on_configure(const rclcpp_lifecycle::State &)
{
  slot_plan_bt_ = get_parameter("slot_plan_bt").as_string();
  cutting_task_bt_ = get_parameter("cutting_task_bt").as_string();
  marker_x_ = get_parameter("marker_x").as_double();
  marker_y_ = get_parameter("marker_y").as_double();
  nav_standoff_ = get_parameter("nav_standoff").as_double();
  realign_distance_ = get_parameter("realign_distance").as_double();
  const auto libs = get_parameter("plugin_lib_names").as_string_array();

  plan_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ExecuteSlotPlan>>(
    shared_from_this(), "execute_slot_plan", libs, slot_plan_bt_,
    [this](ExecuteSlotPlan::Goal::ConstSharedPtr g) {return onPlanGoal(g);},
    [this]() {onPlanLoop();},
    [this](ExecuteSlotPlan::Goal::ConstSharedPtr) {
      RCLCPP_WARN(get_logger(), "Preemption is not supported; finish or cancel the plan first");
      plan_server_->terminatePendingGoal();
    },
    [this](ExecuteSlotPlan::Result::SharedPtr r, BtStatus s) {onPlanDone(r, s);});
  task_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ExecuteCuttingTask>>(
    shared_from_this(), "execute_cutting_task", libs, cutting_task_bt_,
    [this](ExecuteCuttingTask::Goal::ConstSharedPtr g) {return onTaskGoal(g);},
    [this]() {onTaskLoop();},
    [this](ExecuteCuttingTask::Goal::ConstSharedPtr) {
      RCLCPP_WARN(get_logger(), "Preemption is not supported; finish or cancel the task first");
      task_server_->terminatePendingGoal();
    },
    [this](ExecuteCuttingTask::Result::SharedPtr r, BtStatus s) {onTaskDone(r, s);});
  if (!plan_server_->on_configure() || !task_server_->on_configure()) {
    return CallbackReturn::FAILURE;
  }
  record_pub_ = create_publisher<scr_interfaces::msg::StationRecord>(
    "~/station_records", rclcpp::QoS(50).reliable());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskExecutor::on_activate(const rclcpp_lifecycle::State &)
{
  if (!plan_server_->on_activate() || !task_server_->on_activate()) {
    RCLCPP_ERROR(get_logger(), "Could not load the behaviour trees (are the skill servers up?)");
    return CallbackReturn::FAILURE;
  }
  record_pub_->on_activate();
  RCLCPP_INFO(
    get_logger(), "Task executor active: /execute_slot_plan (%s), /execute_cutting_task (%s)",
    slot_plan_bt_.c_str(), cutting_task_bt_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskExecutor::on_deactivate(const rclcpp_lifecycle::State &)
{
  plan_server_->on_deactivate();
  task_server_->on_deactivate();
  record_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskExecutor::on_cleanup(const rclcpp_lifecycle::State &)
{
  plan_logger_.reset();
  task_logger_.reset();
  plan_server_->on_cleanup();
  task_server_->on_cleanup();
  plan_server_.reset();
  task_server_.reset();
  record_pub_.reset();
  return CallbackReturn::SUCCESS;
}

void TaskExecutor::setCommonBlackboard(const BT::Blackboard::Ptr & bb)
{
  bb->set<rclcpp::Clock::SharedPtr>(kClockKey, get_clock());
  bb->set<double>("marker_x", marker_x_);
  bb->set<double>("marker_y", marker_y_);
  bb->set<double>("nav_standoff", nav_standoff_);
  bb->set<std::string>(kLastFailureKey, "");
  bb->set<int>("recoveries", 0);
  bb->set<int>("station_recoveries", 0);
  bb->set<int>("station", -1);
  bb->set<double>("since_fix", 0.0);
  bb->set<bool>("realign_due", false);
  start_ = now();
}

std::string TaskExecutor::failureText(
  const BT::Blackboard::Ptr & bb, const FailureLogger * logger) const
{
  std::string why;
  bb->get(kLastFailureKey, why);
  if (why.empty() && logger && !logger->failures.empty()) {
    why = "failed at " + logger->failures.back();
  }
  return why;
}

// ---------------------------------------------------------------------------
// 多站开槽
// ---------------------------------------------------------------------------
bool TaskExecutor::onPlanGoal(ExecuteSlotPlan::Goal::ConstSharedPtr goal)
{
  const auto bt = goal->behavior_tree.empty() ? slot_plan_bt_ : goal->behavior_tree;
  if (!plan_server_->loadBehaviorTree(bt)) {
    RCLCPP_ERROR(get_logger(), "Cannot load behaviour tree %s", bt.c_str());
    return false;
  }
  auto bb = plan_server_->getBlackboard();
  setCommonBlackboard(bb);
  bb->set<std::string>("plan_file", goal->plan_file);
  bb->set<bool>("dry_run", goal->dry_run);
  bb->set<double>("dry_run_gap", goal->dry_run_gap > 0.0 ? goal->dry_run_gap : 0.01);
  bb->set<bool>("compensate", goal->compensate);
  bb->set<double>("max_strafe", goal->max_strafe > 0.0 ? goal->max_strafe : 1.5);
  const double realign =
    goal->realign_distance == 0.0 ? realign_distance_ : goal->realign_distance;
  bb->set<double>("realign_distance", realign);
  bb->set<int>("station_count", 0);
  records_ = std::make_shared<StationRecords>();
  published_ = 0;
  bb->set<StationRecordsPtr>(kRecordsKey, records_);
  plan_logger_ = std::make_unique<FailureLogger>(plan_server_->getTree().rootNode());
  const std::string realign_text = realign > 0.0 ? std::to_string(realign) + " m" : "never";
  RCLCPP_INFO(
    get_logger(), "Executing slot plan %s (dry run %s, compensation %s, re-align after %s)",
    goal->plan_file.c_str(), goal->dry_run ? "on" : "off", goal->compensate ? "on" : "off",
    realign_text.c_str());
  return true;
}

void TaskExecutor::onPlanLoop()
{
  auto bb = plan_server_->getBlackboard();
  auto fb = std::make_shared<ExecuteSlotPlan::Feedback>();
  bb->get("station", fb->station);
  bb->get("station_count", fb->station_count);
  bb->get("recoveries", fb->recoveries);
  fb->phase = runningLeaves(plan_server_->getTree());
  plan_server_->publishFeedback(fb);
  while (records_ && published_ < records_->size()) {
    record_pub_->publish((*records_)[published_++]);
  }
}

void TaskExecutor::onPlanDone(ExecuteSlotPlan::Result::SharedPtr result, BtStatus status)
{
  onPlanLoop();
  auto bb = plan_server_->getBlackboard();
  result->success = status == BtStatus::SUCCEEDED;
  result->stations = records_ ? *records_ : StationRecords{};
  result->stations_done = static_cast<int>(result->stations.size());
  result->total_time = (now() - start_).seconds();
  if (!result->success) {
    result->error = status == BtStatus::CANCELED ? "canceled" :
      failureText(bb, plan_logger_.get());
  }
  RCLCPP_INFO(
    get_logger(), "Slot plan finished: %s, %d stations, %.1f s%s%s",
    result->success ? "success" : "failure", result->stations_done, result->total_time,
    result->error.empty() ? "" : ", ", result->error.c_str());
}

// ---------------------------------------------------------------------------
// 单站任务
// ---------------------------------------------------------------------------
bool TaskExecutor::onTaskGoal(ExecuteCuttingTask::Goal::ConstSharedPtr goal)
{
  const auto bt = goal->behavior_tree.empty() ? cutting_task_bt_ : goal->behavior_tree;
  if (!task_server_->loadBehaviorTree(bt)) {
    RCLCPP_ERROR(get_logger(), "Cannot load behaviour tree %s", bt.c_str());
    return false;
  }
  auto bb = task_server_->getBlackboard();
  setCommonBlackboard(bb);
  bb->set<std::string>("task_file", goal->task_file);
  task_logger_ = std::make_unique<FailureLogger>(task_server_->getTree().rootNode());
  RCLCPP_INFO(get_logger(), "Executing cutting task %s", goal->task_file.c_str());
  return true;
}

void TaskExecutor::onTaskLoop()
{
  auto fb = std::make_shared<ExecuteCuttingTask::Feedback>();
  task_server_->getBlackboard()->get("recoveries", fb->recoveries);
  fb->phase = runningLeaves(task_server_->getTree());
  task_server_->publishFeedback(fb);
}

void TaskExecutor::onTaskDone(ExecuteCuttingTask::Result::SharedPtr result, BtStatus status)
{
  result->success = status == BtStatus::SUCCEEDED;
  result->total_time = (now() - start_).seconds();
  if (!result->success) {
    result->error = status == BtStatus::CANCELED ? "canceled" :
      failureText(task_server_->getBlackboard(), task_logger_.get());
  }
  RCLCPP_INFO(
    get_logger(), "Cutting task finished: %s, %.1f s%s%s",
    result->success ? "success" : "failure", result->total_time,
    result->error.empty() ? "" : ", ", result->error.c_str());
}

}  // namespace scr_task
