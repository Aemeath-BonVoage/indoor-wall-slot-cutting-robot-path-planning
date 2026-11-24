// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
#include "scr_base_control/mecanum_drive_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace scr_base_control
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn MecanumDriveController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "wheel_names", {"joint_lf_link", "joint_rf_link", "joint_lb_link", "joint_rb_link"});
    auto_declare<double>("wheel_radius", 0.05);
    auto_declare<double>("wheel_separation_x", 0.59316);   // 前后轮距
    auto_declare<double>("wheel_separation_y", 0.4938);    // 左右轮距
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<std::string>("odom_topic", "/odom");
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_link");
    auto_declare<bool>("enable_odom_tf", true);
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("average_twist", true);
    // 协方差对角线 (x, y, z, roll, pitch, yaw)：融合时 robot_localization 据此加权
    auto_declare<std::vector<double>>(
      "pose_covariance_diagonal", {1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2});
    auto_declare<std::vector<double>>(
      "twist_covariance_diagonal", {1e-3, 4e-3, 1e6, 1e6, 1e6, 4e-3});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
{
  InterfaceConfiguration c{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & w : wheels_) {
    c.names.push_back(w + "/velocity");
  }
  return c;
}

InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
{
  InterfaceConfiguration c{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & w : wheels_) {
    c.names.push_back(w + "/velocity");
  }
  return c;
}

CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  wheels_ = node->get_parameter("wheel_names").as_string_array();
  if (wheels_.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "wheel_names must list 4 joints [lf, rf, lb, rb]");
    return CallbackReturn::ERROR;
  }
  kin_.wheel_radius = node->get_parameter("wheel_radius").as_double();
  kin_.k = 0.5 * (node->get_parameter("wheel_separation_x").as_double() +
    node->get_parameter("wheel_separation_y").as_double());
  cmd_timeout_ = node->get_parameter("cmd_vel_timeout").as_double();
  const double rate = node->get_parameter("publish_rate").as_double();
  publish_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
  odom_frame_ = node->get_parameter("odom_frame_id").as_string();
  base_frame_ = node->get_parameter("base_frame_id").as_string();
  publish_tf_ = node->get_parameter("enable_odom_tf").as_bool();
  pose_cov_ = node->get_parameter("pose_covariance_diagonal").as_double_array();
  twist_cov_ = node->get_parameter("twist_covariance_diagonal").as_double_array();
  if (pose_cov_.size() != 6 || twist_cov_.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "covariance diagonals need 6 entries");
    return CallbackReturn::ERROR;
  }

  cmd_.writeFromNonRT(std::make_shared<StampedCmd>());
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    node->get_parameter("cmd_vel_topic").as_string(), rclcpp::SystemDefaultsQoS(),
    [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      auto c = std::make_shared<StampedCmd>();
      c->v = {msg->linear.x, msg->linear.y, msg->angular.z};
      c->stamp = get_node()->now();
      cmd_.writeFromNonRT(c);
    });
  odom_raw_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    node->get_parameter("odom_topic").as_string(), rclcpp::SystemDefaultsQoS());
  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    odom_raw_pub_);
  odom_pub_->msg_.header.frame_id = odom_frame_;
  odom_pub_->msg_.child_frame_id = base_frame_;
  for (size_t i = 0; i < 6; ++i) {
    odom_pub_->msg_.pose.covariance[i * 7] = pose_cov_[i];
    odom_pub_->msg_.twist.covariance[i * 7] = twist_cov_[i];
  }
  if (publish_tf_) {
    tf_raw_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    tf_pub_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      tf_raw_pub_);
    tf_pub_->msg_.transforms.resize(1);
    tf_pub_->msg_.transforms[0].header.frame_id = odom_frame_;
    tf_pub_->msg_.transforms[0].child_frame_id = base_frame_;
  }
  RCLCPP_INFO(
    node->get_logger(), "mecanum drive: r = %.4f m, lx + ly = %.4f m, odom on %s%s",
    kin_.wheel_radius, kin_.k, odom_raw_pub_->get_topic_name(),
    publish_tf_ ? " (+ TF)" : "");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 4 || state_interfaces_.size() != 4) {
    RCLCPP_ERROR(get_node()->get_logger(), "expected 4 wheel command and state interfaces");
    return CallbackReturn::ERROR;
  }
  odom_ = Odometry2D();
  twist_avg_ = TwistAverager();
  cmd_.writeFromNonRT(std::make_shared<StampedCmd>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {
    c.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 里程计：测得轮速 → 机体速度 → 积分
  WheelSpeeds w;
  for (size_t i = 0; i < 4; ++i) {
    w[i] = state_interfaces_[i].get_value();
    if (!std::isfinite(w[i])) {
      w[i] = 0.0;
    }
  }
  const Twist2D v = kin_.forward(w);
  const double dt = period.seconds();
  if (dt > 0.0 && dt < 1.0) {
    odom_.integrate(v, dt);
    twist_avg_.add(v, dt);
  }

  // 指令：超时停车
  const auto cmd = *cmd_.readFromRT();
  Twist2D target;
  if (cmd && cmd->stamp.nanoseconds() > 0 && (time - cmd->stamp).seconds() <= cmd_timeout_) {
    target = cmd->v;
  }
  const WheelSpeeds wc = kin_.inverse(target);
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(wc[i]);
  }

  if (publish_period_ > 0.0 && (time - last_publish_).seconds() < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_ = time;
  // twist：两次发布之间的平均速度（average_twist = false 时为发布这一拍的瞬时速度；
  // 运行中可用 ros2 param set 切换，供对照实验使用）
  average_twist_ = get_node()->get_parameter("average_twist").as_bool();
  const double qz = std::sin(odom_.yaw / 2.0);
  const double qw = std::cos(odom_.yaw / 2.0);
  if (odom_pub_->trylock()) {
    // 没拿到发布锁时不取平均，下一次发布覆盖两段时间
    const Twist2D v_avg = twist_avg_.take();
    const Twist2D & v_pub = average_twist_ ? v_avg : v;
    auto & m = odom_pub_->msg_;
    m.header.stamp = time;
    m.pose.pose.position.x = odom_.x;
    m.pose.pose.position.y = odom_.y;
    m.pose.pose.orientation.z = qz;
    m.pose.pose.orientation.w = qw;
    m.twist.twist.linear.x = v_pub.vx;
    m.twist.twist.linear.y = v_pub.vy;
    m.twist.twist.angular.z = v_pub.wz;
    odom_pub_->unlockAndPublish();
  }
  if (tf_pub_ && tf_pub_->trylock()) {
    auto & t = tf_pub_->msg_.transforms[0];
    t.header.stamp = time;
    t.transform.translation.x = odom_.x;
    t.transform.translation.y = odom_.y;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    tf_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace scr_base_control

PLUGINLIB_EXPORT_CLASS(
  scr_base_control::MecanumDriveController, controller_interface::ControllerInterface)
