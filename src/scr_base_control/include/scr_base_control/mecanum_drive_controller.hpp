// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 麦克纳姆轮底盘控制器（ros2_control 控制器插件）.
//
// Humble 的 ros2_controllers 中没有麦克纳姆轮控制器（mecanum_drive_controller 从 Jazzy
// 开始提供），这里按 diff_drive_controller 的结构实现：
//   - 订阅 cmd_vel（base_link 系 vx, vy, wz），逆运动学 → 四个轮的 velocity 命令接口；
//     超过 cmd_vel_timeout 没有新指令则停车；
//   - 读四个轮的 velocity 状态接口，正运动学 → 积分得到轮式里程计，发布 nav_msgs/Odometry，
//     可选发布 odom → base_link TF（用 robot_localization 融合时关闭）。
// 控制器只依赖硬件接口，仿真（scr_base_control/ScrMecanumBaseSystem）与真机共用。
#ifndef SCR_BASE_CONTROL__MECANUM_DRIVE_CONTROLLER_HPP_
#define SCR_BASE_CONTROL__MECANUM_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "scr_base_control/mecanum_kinematics.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace scr_base_control
{

class MecanumDriveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct StampedCmd
  {
    Twist2D v;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  std::vector<std::string> wheels_;
  MecanumKinematics kin_;
  Odometry2D odom_;
  double cmd_timeout_{0.5};
  double publish_period_{0.02};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  std::string odom_frame_, base_frame_;
  bool publish_tf_{true};
  std::vector<double> pose_cov_, twist_cov_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<StampedCmd>> cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_raw_pub_;
};

}  // namespace scr_base_control

#endif  // SCR_BASE_CONTROL__MECANUM_DRIVE_CONTROLLER_HPP_
