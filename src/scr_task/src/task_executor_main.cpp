// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
// 行为树任务执行器入口：等待 Nav2 与技能服务器就绪后自行完成 configure → activate.
#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "scr_task/task_executor.hpp"

namespace
{

// 与 nav2_simple_commander 的 waitUntilNav2Active 相同：轮询 bt_navigator 的生命周期状态
bool waitForNav2(const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>("/bt_navigator/get_state");
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(std::chrono::seconds(1))) {
      auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->current_state.label == "active")
      {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

bool waitForActions(const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (const auto & name : scr_task::TaskExecutor::requiredActions()) {
    bool found = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto names = node->get_topic_names_and_types();
      if (names.count(name + "/_action/status")) {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!found) {
      RCLCPP_ERROR(node->get_logger(), "Action server %s not available", name.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor_node = std::make_shared<scr_task::TaskExecutor>();
  auto waiter = rclcpp::Node::make_shared("task_executor_startup");
  const auto timeout = std::chrono::seconds(
    executor_node->declare_parameter("startup_timeout", 300));
  RCLCPP_INFO(waiter->get_logger(), "Waiting for Nav2 and the skill servers...");
  if (!waitForNav2(waiter, timeout) || !waitForActions(waiter, timeout)) {
    RCLCPP_FATAL(waiter->get_logger(), "Dependencies did not come up; exiting");
    rclcpp::shutdown();
    return 1;
  }
  waiter.reset();
  executor_node->configure();
  if (executor_node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(executor_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
