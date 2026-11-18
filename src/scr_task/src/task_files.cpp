// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
#include "scr_task/task_files.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace scr_task
{

namespace
{

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

scr_interfaces::msg::ArmPoint makePoint(const YAML::Node & p)
{
  scr_interfaces::msg::ArmPoint a;
  a.x = p["x"].as<double>();
  a.y = p["y"].as<double>();
  a.z = p["z"].as<double>();
  a.rotation = p["rotation"].as<double>() * M_PI / 180.0;
  return a;
}

}  // namespace

geometry_msgs::msg::PoseStamped SlotPlan::wallPose(double u, double d) const
{
  const double x = p0[0] + u * t[0] + d * n[0];
  const double y = p0[1] + u * t[1] + d * n[1];
  return makePose(frame_id, x, y, std::atan2(-n[1], -n[0]));
}

double SlotPlan::uOf(double x, double y) const
{
  return (x - p0[0]) * t[0] + (y - p0[1]) * t[1];
}

SlotPlan loadSlotPlan(const std::string & path, std::optional<double> dry_run_gap)
{
  const YAML::Node d = YAML::LoadFile(path);
  if (!d["wall"] || !d["stations"]) {
    throw std::runtime_error("not a slot plan (missing wall / stations): " + path);
  }
  const std::string status = d["status"] ? d["status"].as<std::string>() : "";
  if (status != "success" || d["stations"].size() == 0) {
    throw std::runtime_error("plan is not executable: status '" + status + "'");
  }
  SlotPlan plan;
  if (d["frame_id"]) {
    plan.frame_id = d["frame_id"].as<std::string>();
  }
  const auto l = d["wall"]["left_end"];
  const auto r = d["wall"]["right_end"];
  plan.p0 = {l[0].as<double>(), l[1].as<double>()};
  const double dx = r[0].as<double>() - plan.p0[0];
  const double dy = r[1].as<double>() - plan.p0[1];
  const double len = std::hypot(dx, dy);
  plan.t = {dx / len, dy / len};
  plan.n = {plan.t[1], -plan.t[0]};
  const auto robot = d["robot"];
  plan.standoff = robot["standoff"].as<double>();
  plan.x_cut = robot["x_cut"].as<double>();
  plan.tool_offset = robot["tool_offset"].as<double>();
  if (d["marker_u"]) {
    plan.marker_u = d["marker_u"].as<double>();
  }
  const double x_dry = dry_run_gap ? plan.standoff - *dry_run_gap - plan.tool_offset : 0.0;
  for (const auto & s : d["stations"]) {
    Station st;
    st.index = s["index"].as<int>();
    st.u_c = s["u_c"].as<double>();
    st.cut_length = s["cut_length"] ? s["cut_length"].as<double>() : 0.0;
    if (s["motion"] && s["motion"]["duration"]) {
      st.point_duration = s["motion"]["duration"].as<double>();
    }
    for (const auto & p : s["points"]) {
      auto a = makePoint(p);
      a.at_depth = std::abs(a.x - plan.x_cut) < 1e-6;
      if (a.at_depth && dry_run_gap) {
        a.x = x_dry;
      }
      st.points.push_back(a);
    }
    plan.stations.push_back(st);
  }
  return plan;
}

CuttingTask loadCuttingTask(const std::string & path)
{
  const YAML::Node d = YAML::LoadFile(path);
  if (!d["navigation"] || !d["points"] || d["points"].size() < 2) {
    throw std::runtime_error("task file needs 'navigation' and at least two 'points': " + path);
  }
  CuttingTask task;
  const auto nav = d["navigation"];
  task.nav_goal = makePose(
    "map", nav["x"].as<double>(), nav["y"].as<double>(), nav["yaw"].as<double>());
  if (const auto pp = d["precise_positioning"]) {
    task.align = pp["enabled"] ? pp["enabled"].as<bool>() : true;
    task.standoff = pp["distance_from_marker"] ? pp["distance_from_marker"].as<double>() : 0.6;
    task.mode = pp["mode"] ? pp["mode"].as<std::string>() : "visual";
  }
  if (const auto sc = d["scanning"]) {
    task.scan = sc["enabled"] ? sc["enabled"].as<bool>() : true;
  }
  for (const auto & p : d["points"]) {
    task.points.push_back(makePoint(p));
  }
  if (const auto m = d["motion"]) {
    task.point_duration = m["duration"] ? m["duration"].as<double>() : 3.0;
    task.return_home = m["return_home"] ? m["return_home"].as<bool>() : true;
    task.wall_gap = (m["dry_run_gap"] && !m["dry_run_gap"].IsNull()) ?
      m["dry_run_gap"].as<double>() : 0.0;
  }
  return task;
}

}  // namespace scr_task
