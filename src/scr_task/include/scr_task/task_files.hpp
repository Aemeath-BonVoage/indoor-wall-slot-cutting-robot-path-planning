// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 任务文件解析：离线开槽规划（plan_wall_slots.py 输出）与单站任务（task_at_target.yaml 格式）.
//
// 与 scr_arm_controller/slot_execution.py 实现同一套规则（墙面坐标系、停靠位姿、空切点），
// 两边的单元测试对照同一份规划文件检查。
#ifndef SCR_TASK__TASK_FILES_HPP_
#define SCR_TASK__TASK_FILES_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scr_interfaces/msg/arm_point.hpp"

namespace scr_task
{

using ArmPoints = std::vector<scr_interfaces::msg::ArmPoint>;

struct Station
{
  int index{0};
  double u_c{0.0};          // 停靠站沿墙坐标（米）
  ArmPoints points;
  double point_duration{2.0};
  double cut_length{0.0};   // 米
};

struct SlotPlan
{
  std::string frame_id{"map"};
  std::array<double, 2> p0{};   // 墙左端点（面向墙面）
  std::array<double, 2> t{};    // 沿墙向右的单位向量
  std::array<double, 2> n{};    // 指向房间的法向
  double standoff{0.7};
  double x_cut{0.0};
  double tool_offset{0.0};
  std::optional<double> marker_u;
  std::vector<Station> stations;

  // 正对墙面、base_link 离墙 standoff、位于墙面 u 处的位姿
  geometry_msgs::msg::PoseStamped wallPose(double u, double standoff) const;
  // map 坐标 (x, y) 在墙面坐标中的 u
  double uOf(double x, double y) const;
};

// 读取规划文件；dry_run_gap：空切，切深处的点改为刀尖停在规划墙面前 gap 处。
// 规划状态不是 success 或没有停靠站时抛出 std::runtime_error。
SlotPlan loadSlotPlan(const std::string & path, std::optional<double> dry_run_gap = {});

struct CuttingTask
{
  geometry_msgs::msg::PoseStamped nav_goal;
  bool align{true};
  double standoff{0.6};
  std::string mode{"visual"};
  bool scan{true};
  ArmPoints points;
  double point_duration{3.0};
  bool return_home{true};
  double wall_gap{0.0};     // >0：空切
};

CuttingTask loadCuttingTask(const std::string & path);

}  // namespace scr_task

#endif  // SCR_TASK__TASK_FILES_HPP_
