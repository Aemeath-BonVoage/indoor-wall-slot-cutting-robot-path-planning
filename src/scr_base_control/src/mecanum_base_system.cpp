// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
#include "scr_base_control/mecanum_base_system.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "gazebo/physics/physics.hh"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace scr_base_control
{

namespace
{
double param(const hardware_interface::HardwareInfo & info, const std::string & key, double d)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? d : std::stod(it->second);
}

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & d)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? d : it->second;
}

std::array<double, 4> parse4(const std::string & s)
{
  std::array<double, 4> out{0.0, 0.0, 0.0, 0.0};
  std::stringstream ss(s);
  std::string item;
  for (size_t i = 0; i < 4 && std::getline(ss, item, ','); ++i) {
    out[i] = std::stod(item);
  }
  return out;
}
}  // namespace

bool ScrMecanumBaseSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh, gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & info, sdf::ElementPtr)
{
  nh_ = model_nh;
  model_ = parent_model;
  const auto log = nh_->get_logger();
  base_ = model_->GetLink(param(info, "base_link", std::string("base_link")));
  if (!base_) {
    RCLCPP_ERROR(log, "ScrMecanumBaseSystem: base link not found");
    return false;
  }
  if (info.joints.size() != 4) {
    RCLCPP_ERROR(log, "ScrMecanumBaseSystem: expected 4 wheel joints [lf, rf, lb, rb]");
    return false;
  }
  for (const auto & j : info.joints) {
    wheel_names_.push_back(j.name);
    wheel_joints_.push_back(model_->GetJoint(j.name));   // 外观用，可为空
  }
  kin_.wheel_radius = param(info, "wheel_radius", 0.05);
  kin_.k = 0.5 * (param(info, "wheel_separation_x", 0.59316) +
    param(info, "wheel_separation_y", 0.4938));
  k_lin_ = param(info, "k_lin", 20.0);
  k_ang_ = param(info, "k_ang", 20.0);
  a_max_ = param(info, "max_accel", 5.0);
  alpha_max_ = param(info, "max_ang_accel", 10.0);
  wheel_kp_ = param(info, "wheel_kp", 0.05);
  pose_velocity_ = param(info, "velocity_source", std::string("pose")) == "pose";
  // 模拟陀螺：imu_rate = 0 时不发布
  const double imu_rate = param(info, "imu_rate", 100.0);
  if (imu_rate > 0.0) {
    imu_period_ = 1.0 / imu_rate;
    imu_frame_ = param(info, "imu_frame", std::string("base_link"));
    gyro_noise_ = param(info, "gyro_noise", 0.001);
    imu_rng_.seed(static_cast<unsigned int>(param(info, "seed", 1.0)) + 1000u);
    // 零偏：给定量级、随机符号（相当于启动标定后的剩余零偏）
    const double b = param(info, "gyro_bias", 5e-5);
    gyro_bias_ = imu_normal_(imu_rng_) < 0.0 ? -b : b;
    imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(
      param(info, "imu_topic", std::string("/imu")), rclcpp::SensorDataQoS());
  }
  dir_ = parse4(param(info, "wheel_directions", std::string("-1,1,-1,1")));
  wheel_plant_ = param(info, "plant", std::string("force")) == "wheels";
  wheel_max_torque_ = param(info, "wheel_max_torque", 3.0);
  contact_damping_ = param(info, "contact_damping", 3000.0);
  friction_mu_ = param(info, "friction_mu", 1.0);
  {
    // X 形布置：辊子轴线方向与逆运动学一致（lf、rb 为 (1, −1)，rf、lb 为 (1, 1)）
    const double lx = 0.5 * param(info, "wheel_separation_x", 0.59316);
    const double ly = 0.5 * param(info, "wheel_separation_y", 0.4938);
    const double h = std::sqrt(0.5);
    contact_xy_ = {{{lx, ly}, {lx, -ly}, {-lx, ly}, {-lx, -ly}}};
    roller_ = {{{h, -h}, {h, h}, {h, h}, {h, -h}}};
  }
  if (wheel_plant_) {
    for (size_t i = 0; i < 4; ++i) {
      if (!wheel_joints_[i]) {
        RCLCPP_ERROR(log, "plant=wheels needs the wheel joint %s", wheel_names_[i].c_str());
        return false;
      }
    }
  }

  EncoderErrorParams e;
  e.scale = parse4(param(info, "wheel_scale_errors", std::string("0,0,0,0")));
  e.slip_sigma = param(info, "slip_sigma", 0.0);
  e.slip_tau = param(info, "slip_tau", 0.5);
  e.lateral_slip = param(info, "lateral_slip", 0.0);
  e.yaw_slip = param(info, "yaw_slip", 0.0);
  e.seed = static_cast<unsigned int>(param(info, "seed", 1.0));
  encoder_ = std::make_unique<EncoderErrorModel>(kin_, e);
  computeMassProperties();
  RCLCPP_INFO(log, "ScrMecanumBaseSystem: plant %s", wheel_plant_ ? "wheels" : "force");
  RCLCPP_INFO(
    log, "ScrMecanumBaseSystem: mass %.2f kg, Iz %.3f kg m^2; encoder errors scale "
    "[%.3f %.3f %.3f %.3f] slip %.3f/%.2f s lateral %.3f yaw %.3f seed %u",
    mass_, inertia_z_, e.scale[0], e.scale[1], e.scale[2], e.scale[3], e.slip_sigma,
    e.slip_tau, e.lateral_slip, e.yaw_slip, e.seed);
  return true;
}

void ScrMecanumBaseSystem::computeMassProperties()
{
  // 整机总质量与绕底盘竖直轴的转动惯量（加载时计算一次，机械臂姿态的影响很小）
  mass_ = 0.0;
  inertia_z_ = 0.0;
  const auto c0 = base_->WorldPose().Pos();
  for (const auto & link : model_->GetLinks()) {
    const double m = link->GetInertial()->Mass();
    const auto r = link->WorldCoGPose().Pos() - c0;
    mass_ += m;
    inertia_z_ += link->WorldInertiaMatrix()(2, 2) + m * (r.X() * r.X() + r.Y() * r.Y());
  }
}

std::vector<hardware_interface::StateInterface> ScrMecanumBaseSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < 4; ++i) {
    out.emplace_back(wheel_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    out.emplace_back(wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
ScrMecanumBaseSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < 4; ++i) {
    out.emplace_back(wheel_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
  }
  return out;
}

hardware_interface::return_type ScrMecanumBaseSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (wheel_plant_) {
    // 编码器：轮子关节的实际转角 / 转速（转为“前进为正”）
    for (size_t i = 0; i < 4; ++i) {
      vel_[i] = dir_[i] * wheel_joints_[i]->GetVelocity(0);
      pos_[i] = dir_[i] * wheel_joints_[i]->Position(0);
    }
    return hardware_interface::return_type::OK;
  }
  // 编码器：底盘真实机体速度 → 轮速（含误差模型）→ 积分轮角
  const double yaw = base_->WorldPose().Rot().Yaw();
  const double c = std::cos(yaw), s = std::sin(yaw);
  const Twist2D vt{c * vxw_ + s * vyw_, -s * vxw_ + c * vyw_, wz_};
  const WheelSpeeds w = encoder_->measure(vt, dt);
  for (size_t i = 0; i < 4; ++i) {
    vel_[i] = w[i];
    pos_[i] += w[i] * dt;
  }
  return hardware_interface::return_type::OK;
}

void ScrMecanumBaseSystem::applyWheelContacts(const WheelSpeeds & wc)
{
  const auto pose = base_->WorldPose();
  const double yaw = pose.Rot().Yaw();
  const double c = std::cos(yaw), s = std::sin(yaw);
  const auto v = base_->WorldLinearVel();
  const double wz = base_->WorldAngularVel().Z();
  const double vx = c * v.X() + s * v.Y();     // 机体系
  const double vy = -s * v.X() + c * v.Y();
  const double n_max = friction_mu_ * mass_ * 9.81 / 4.0;
  for (size_t i = 0; i < 4; ++i) {
    // 轮子电机：ODE 关节电机，目标转速 + 最大力矩（每个物理步设置）
    auto & j = wheel_joints_[i];
    j->SetParam("fmax", 0, wheel_max_torque_);
    j->SetParam("vel", 0, dir_[i] * wc[i]);
    // 接地点速度（机体系）与轮缘速度之差在辊子轴线上的分量
    const double px = contact_xy_[i][0], py = contact_xy_[i][1];
    const double cvx = vx - wz * py, cvy = vy + wz * px;
    const double surface = kin_.wheel_radius * dir_[i] * j->GetVelocity(0);
    const double e = (cvx - surface) * roller_[i][0] + cvy * roller_[i][1];
    const double f = -std::clamp(contact_damping_ * e, -n_max, n_max);
    const double fx = f * roller_[i][0], fy = f * roller_[i][1];
    // 力以世界系表达，作用点以 base_link 系表达
    base_->AddForceAtRelativePosition(
      ignition::math::Vector3d(c * fx - s * fy, s * fx + c * fy, 0.0),
      ignition::math::Vector3d(px, py, 0.0));
    // 反作用：地面推底盘 = 轮缘被地面向后推，力矩阻碍轮子转动（电机负载）
    j->SetForce(0, -dir_[i] * kin_.wheel_radius * fx);
  }
}

void ScrMecanumBaseSystem::updateBodyVelocity()
{
  const auto pose = base_->WorldPose();
  const double t = model_->GetWorld()->SimTime().Double();
  const double x = pose.Pos().X(), y = pose.Pos().Y(), yaw = pose.Rot().Yaw();
  if (!pose_velocity_ || !have_prev_ || t <= prev_t_) {
    const auto v = base_->WorldLinearVel();
    if (!pose_velocity_ || !have_prev_) {
      vxw_ = v.X();
      vyw_ = v.Y();
      wz_ = base_->WorldAngularVel().Z();
    }
  } else {
    // 位姿差分：Gazebo 中底盘在零速度指令下仍会缓慢转动（约 0.08°/s），这部分位姿变化
    // 来自约束修正，不出现在连杆速度里；用连杆速度做速度闭环和编码器时都看不到它
    const double dt = t - prev_t_;
    const double a = dt / (vel_tau_ + dt);
    vxw_ += a * ((x - prev_x_) / dt - vxw_);
    vyw_ += a * ((y - prev_y_) / dt - vyw_);
    wz_ += a * (std::remainder(yaw - prev_yaw_, 2.0 * M_PI) / dt - wz_);
  }
  have_prev_ = true;
  prev_t_ = t;
  prev_x_ = x;
  prev_y_ = y;
  prev_yaw_ = yaw;
}

void ScrMecanumBaseSystem::publishImu()
{
  const auto t = model_->GetWorld()->SimTime();
  if (!imu_pub_ || t.Double() - last_imu_t_ < imu_period_ - 1e-9) {
    return;
  }
  last_imu_t_ = t.Double();
  sensor_msgs::msg::Imu m;
  m.header.stamp.sec = t.sec;
  m.header.stamp.nanosec = t.nsec;
  m.header.frame_id = imu_frame_;
  m.orientation_covariance[0] = -1.0;   // 不提供姿态
  m.angular_velocity.x = gyro_noise_ * imu_normal_(imu_rng_);
  m.angular_velocity.y = gyro_noise_ * imu_normal_(imu_rng_);
  m.angular_velocity.z = wz_ + gyro_bias_ + gyro_noise_ * imu_normal_(imu_rng_);
  const double var = gyro_noise_ * gyro_noise_;
  m.angular_velocity_covariance = {var, 0, 0, 0, var, 0, 0, 0, var};
  m.linear_acceleration.z = 9.81;
  m.linear_acceleration_covariance[0] = -1.0;   // 不提供加速度
  imu_pub_->publish(m);
}

hardware_interface::return_type ScrMecanumBaseSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  WheelSpeeds wc;
  for (size_t i = 0; i < 4; ++i) {
    wc[i] = std::isfinite(cmd_[i]) ? cmd_[i] : 0.0;
  }
  if (wheel_plant_) {
    updateBodyVelocity();
    publishImu();
    applyWheelContacts(wc);
    return hardware_interface::return_type::OK;
  }
  // 命令轮速 → 机体速度指令 → 力 / 力矩速度闭环（每个物理步都施加）
  updateBodyVelocity();
  publishImu();
  const Twist2D vc = kin_.forward(wc);
  const auto pose = base_->WorldPose();
  const double yaw = pose.Rot().Yaw();
  const double vx_w = vc.vx * std::cos(yaw) - vc.vy * std::sin(yaw);
  const double vy_w = vc.vx * std::sin(yaw) + vc.vy * std::cos(yaw);
  double fx = mass_ * k_lin_ * (vx_w - vxw_);
  double fy = mass_ * k_lin_ * (vy_w - vyw_);
  const double f_max = mass_ * a_max_;
  const double f_norm = std::hypot(fx, fy);
  if (f_norm > f_max) {
    fx *= f_max / f_norm;
    fy *= f_max / f_norm;
  }
  const double t_max = inertia_z_ * alpha_max_;
  const double tz = std::clamp(inertia_z_ * k_ang_ * (vc.wz - wz_), -t_max, t_max);
  base_->AddForce(ignition::math::Vector3d(fx, fy, 0.0));
  base_->AddTorque(ignition::math::Vector3d(0.0, 0.0, tz));
  // 轮子外观：小增益速度 P 控制（摩擦为 0，对底盘运动无影响）
  for (size_t i = 0; i < 4; ++i) {
    if (wheel_joints_[i]) {
      wheel_joints_[i]->SetForce(
        0, wheel_kp_ * (dir_[i] * wc[i] - wheel_joints_[i]->GetVelocity(0)));
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace scr_base_control

PLUGINLIB_EXPORT_CLASS(
  scr_base_control::ScrMecanumBaseSystem, gazebo_ros2_control::GazeboSystemInterface)
