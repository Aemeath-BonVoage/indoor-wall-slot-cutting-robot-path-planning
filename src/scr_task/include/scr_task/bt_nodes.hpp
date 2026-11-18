// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
// 行为树节点：技能 action 客户端（基于 nav2_behavior_tree::BtActionNode）与任务流程节点.
//
// 技能节点与 scr_arm_controller/skill_server.py 的 action 一一对应；流程节点负责读取任务
// 文件、遍历停靠站、记录每站结果与统计恢复次数。全部在 scr_task_bt_nodes 库中注册，
// 由行为树执行器（task_executor）按插件库名加载。
#ifndef SCR_TASK__BT_NODES_HPP_
#define SCR_TASK__BT_NODES_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "scr_interfaces/action/cut_station.hpp"
#include "scr_interfaces/action/fine_align.hpp"
#include "scr_interfaces/action/hold_base.hpp"
#include "scr_interfaces/action/move_to_station.hpp"
#include "scr_interfaces/action/retract_arm.hpp"
#include "scr_interfaces/action/scan_marker.hpp"
#include "scr_interfaces/msg/station_record.hpp"
#include "scr_task/task_files.hpp"

namespace scr_task
{

using SlotPlanPtr = std::shared_ptr<const SlotPlan>;
using StationRecords = std::vector<scr_interfaces::msg::StationRecord>;
using StationRecordsPtr = std::shared_ptr<StationRecords>;

// 黑板上的公共键（执行器在每个目标开始时写入）
constexpr const char * kRecordsKey = "station_records";   // StationRecordsPtr
constexpr const char * kLastFailureKey = "last_failure";  // std::string
constexpr const char * kClockKey = "sim_clock";           // rclcpp::Clock::SharedPtr

// ---------------------------------------------------------------------------
// 技能 action 节点：失败时把“节点名: 服务器给出的原因”写到黑板 last_failure
// ---------------------------------------------------------------------------
template<class ActionT>
class SkillActionNode : public nav2_behavior_tree::BtActionNode<ActionT>
{
public:
  SkillActionNode(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<ActionT>(xml_tag_name, action_name, conf) {}

  BT::NodeStatus on_aborted() override
  {
    std::string why = this->name() + " aborted";
    if (this->result_.result) {
      const std::string msg = message(*this->result_.result);
      if (!msg.empty()) {
        why += ": " + msg;
      }
    }
    this->config().blackboard->set(kLastFailureKey, why);
    RCLCPP_WARN(this->node_->get_logger(), "%s", why.c_str());
    return BT::NodeStatus::FAILURE;
  }

protected:
  template<class R>
  static auto message(const R & r)->decltype(r.message, std::string()) {return r.message;}
  static std::string message(...) {return "";}
};

class FineAlignAction : public SkillActionNode<scr_interfaces::action::FineAlign>
{
public:
  FineAlignAction(const std::string & name, const BT::NodeConfiguration & conf);
  void on_tick() override;
  BT::NodeStatus on_success() override;
  static BT::PortsList providedPorts();
};

class MoveToStationAction : public SkillActionNode<scr_interfaces::action::MoveToStation>
{
public:
  MoveToStationAction(const std::string & name, const BT::NodeConfiguration & conf);
  void on_tick() override;
  BT::NodeStatus on_success() override;
  static BT::PortsList providedPorts();
};

class CutStationAction : public SkillActionNode<scr_interfaces::action::CutStation>
{
public:
  CutStationAction(const std::string & name, const BT::NodeConfiguration & conf);
  void on_tick() override;
  BT::NodeStatus on_success() override;
  static BT::PortsList providedPorts();
};

// 定点保持：一直 RUNNING，被 Parallel 停止时取消目标（服务器随即释放保持）
class HoldBaseAction : public SkillActionNode<scr_interfaces::action::HoldBase>
{
public:
  HoldBaseAction(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts() {return providedBasicPorts({});}
};

class ScanMarkerAction : public SkillActionNode<scr_interfaces::action::ScanMarker>
{
public:
  ScanMarkerAction(const std::string & name, const BT::NodeConfiguration & conf);
  void on_tick() override;
  static BT::PortsList providedPorts();
};

class RetractArmAction : public SkillActionNode<scr_interfaces::action::RetractArm>
{
public:
  RetractArmAction(const std::string & name, const BT::NodeConfiguration & conf);
  void on_tick() override;
  static BT::PortsList providedPorts();
};

// ---------------------------------------------------------------------------
// 流程节点
// ---------------------------------------------------------------------------

// 读取离线开槽规划，给出标记前的导航目标与墙面原点（定位标记）的 u
class LoadSlotPlan : public BT::SyncActionNode
{
public:
  LoadSlotPlan(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 读取单站任务文件
class LoadCuttingTask : public BT::SyncActionNode
{
public:
  LoadCuttingTask(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 逐站执行子树：每站开始前把该站的参数写到输出端口；子树失败则整体失败，
// 全部成功则 SUCCESS。每站之间让出一次 tick。
class ForEachStation : public BT::DecoratorNode
{
public:
  ForEachStation(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf) {}
  static BT::PortsList providedPorts();
  void halt() override;

private:
  BT::NodeStatus tick() override;
  bool startStation();
  size_t index_{0};
  bool started_{false};
  double u_prev_{0.0};
};

// 把本站结果追加到黑板 station_records
class RecordStation : public BT::SyncActionNode
{
public:
  RecordStation(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 恢复路径入口处计数（本站 / 全程）
class CountRecovery : public BT::SyncActionNode
{
public:
  CountRecovery(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 布尔端口为 true 时 SUCCESS
class CheckFlag : public BT::ConditionNode
{
public:
  CheckFlag(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 按行程触发的重新对位：自上次接受标记更新以来的沿墙行程 since_fix 加上去下一站的
// 横移 move_dist 超过 threshold（米，≤ 0 表示关闭）时 SUCCESS，并把判定结果写到 due。
// 沿墙误差中与横移距离成正比的部分只靠里程计外推时不断累积，按行程而不是按站数触发。
bool realignDue(double since_fix, double move_dist, double threshold);

class RealignDue : public BT::ConditionNode
{
public:
  RealignDue(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// 注册流程节点（技能节点由插件入口 BT_RegisterNodesFromPlugin 一并注册；
// 单元测试只注册流程节点，技能节点用同名的模拟节点代替）
void registerFlowNodes(BT::BehaviorTreeFactory & factory);

// 当前时间（秒）：黑板上有执行器时钟时取该时钟（仿真时间），否则取单调时钟（单元测试）
double nowSeconds(const BT::Blackboard::Ptr & bb);

}  // namespace scr_task

#endif  // SCR_TASK__BT_NODES_HPP_
