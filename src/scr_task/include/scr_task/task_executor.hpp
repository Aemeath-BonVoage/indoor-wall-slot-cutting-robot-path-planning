// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
// 行为树任务执行器：与 Nav2 bt_navigator 相同的结构，每种任务是一个由行为树执行的 action.
//
//   /execute_slot_plan     scr_interfaces/action/ExecuteSlotPlan     多站开槽规划
//   /execute_cutting_task  scr_interfaces/action/ExecuteCuttingTask  单站任务
//   /task_executor/station_records  scr_interfaces/msg/StationRecord  每站结束时发布
#ifndef SCR_TASK__TASK_EXECUTOR_HPP_
#define SCR_TASK__TASK_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "scr_interfaces/action/execute_cutting_task.hpp"
#include "scr_interfaces/action/execute_slot_plan.hpp"
#include "scr_task/bt_nodes.hpp"

namespace scr_task
{

// 记录转为 FAILURE 的叶节点（失败报告用）
class FailureLogger : public BT::StatusChangeLogger
{
public:
  explicit FailureLogger(BT::TreeNode * root)
  : BT::StatusChangeLogger(root) {}
  void callback(
    BT::Duration, const BT::TreeNode & node, BT::NodeStatus,
    BT::NodeStatus status) override
  {
    if (status == BT::NodeStatus::FAILURE &&
      (node.type() == BT::NodeType::ACTION || node.type() == BT::NodeType::CONDITION))
    {
      failures.push_back(node.name());
    }
  }
  void flush() override {}
  std::vector<std::string> failures;
};

// 正在执行的叶节点名（多个时以 + 连接），用于 action 反馈
std::string runningLeaves(const BT::Tree & tree);

class TaskExecutor : public nav2_util::LifecycleNode
{
public:
  using ExecuteSlotPlan = scr_interfaces::action::ExecuteSlotPlan;
  using ExecuteCuttingTask = scr_interfaces::action::ExecuteCuttingTask;

  explicit TaskExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 行为树中用到的 action 服务器名（激活前应全部可用，否则建树失败）
  static std::vector<std::string> requiredActions();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void setCommonBlackboard(const BT::Blackboard::Ptr & bb);

  bool onPlanGoal(ExecuteSlotPlan::Goal::ConstSharedPtr goal);
  void onPlanLoop();
  void onPlanDone(ExecuteSlotPlan::Result::SharedPtr result, nav2_behavior_tree::BtStatus status);

  bool onTaskGoal(ExecuteCuttingTask::Goal::ConstSharedPtr goal);
  void onTaskLoop();
  void onTaskDone(
    ExecuteCuttingTask::Result::SharedPtr result, nav2_behavior_tree::BtStatus status);

  std::string failureText(const BT::Blackboard::Ptr & bb, const FailureLogger * logger) const;

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ExecuteSlotPlan>> plan_server_;
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ExecuteCuttingTask>> task_server_;
  std::unique_ptr<FailureLogger> plan_logger_, task_logger_;
  rclcpp_lifecycle::LifecyclePublisher<scr_interfaces::msg::StationRecord>::SharedPtr record_pub_;
  StationRecordsPtr records_;
  size_t published_{0};
  rclcpp::Time start_;
  std::string slot_plan_bt_, cutting_task_bt_;
  double marker_x_{0.0}, marker_y_{0.0}, nav_standoff_{1.2};
  double realign_distance_{0.0};
};

}  // namespace scr_task

#endif  // SCR_TASK__TASK_EXECUTOR_HPP_
