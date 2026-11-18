// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
// 麦克纳姆轮底盘的仿真硬件接口（gazebo_ros2_control 的 GazeboSystemInterface 插件）.
//
// 对上与真机硬件接口相同：四个轮的 velocity 命令接口与 position / velocity 状态接口。
// 对下有两种被控对象（硬件参数 plant）：
//   force   命令轮速 → 麦克纳姆正运动学 → 机体速度指令 → 在底盘上施加力 / 力矩做速度闭环
//           （与 scr_planar_move 插件相同）；轮子摩擦为 0、只按命令转动作为外观。
//           编码器读数由底盘真实运动经逆运动学得到，再叠加 EncoderErrorModel 的误差。
//   wheels  轮子关节由电机（ODE 关节电机：目标转速 + 最大力矩）驱动，底盘只受轮–地接触力：
//           每个轮在辊子轴线方向上的滑移速度 e = (v_接触点 − r·ω·x̂)·r̂ 产生摩擦力
//           F = −sat(c·e, μN)·r̂，作用于底盘的接地点，反作用力矩加在轮子上（电机负载）；
//           垂直于辊子轴线的方向辊子自由滚动、无力。编码器读数取轮子关节的实际转速，打滑由
//           接触力饱和与电机力矩限制产生。
//           （Gazebo Classic 的 ODE 摩擦方向 fdir1 只能在碰撞体自身坐标系中表达，轮子转动时
//           会跟着转，无法直接表达麦克纳姆辊子的固定方向，所以接触力在插件中计算。）
// 两种情况下控制器积分出的里程计都会随运动漂移，而不是 Gazebo 的真值位姿。
#ifndef SCR_BASE_CONTROL__MECANUM_BASE_SYSTEM_HPP_
#define SCR_BASE_CONTROL__MECANUM_BASE_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <random>

#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "scr_base_control/encoder_error_model.hpp"
#include "scr_base_control/mecanum_kinematics.hpp"

namespace scr_base_control
{

class ScrMecanumBaseSystem : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info, sdf::ElementPtr sdf) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void computeMassProperties();
  // 底盘实际速度（世界系 vx, vy 与 wz）：pose 模式下由相邻物理步的位姿差分并低通得到
  void updateBodyVelocity();
  void publishImu();
  void applyWheelContacts(const WheelSpeeds & wc);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  std::vector<std::string> wheel_names_;
  std::vector<gazebo::physics::JointPtr> wheel_joints_;
  MecanumKinematics kin_;
  std::unique_ptr<EncoderErrorModel> encoder_;
  std::array<double, 4> pos_{}, vel_{}, cmd_{};
  std::array<double, 4> dir_{-1.0, 1.0, -1.0, 1.0};   // 关节转向 → “前进为正”的轮速
  bool wheel_plant_{false};
  double wheel_max_torque_{3.0};
  double contact_damping_{3000.0};   // N/(m/s)，辊子轴向滑移 → 摩擦力的线性段斜率
  double friction_mu_{1.0};
  std::array<std::array<double, 2>, 4> contact_xy_{};   // 接地点（base_link 系）
  std::array<std::array<double, 2>, 4> roller_{};       // 辊子轴线单位向量（base_link 系）
  double mass_{1.0}, inertia_z_{1.0};
  double k_lin_{20.0}, k_ang_{20.0}, a_max_{5.0}, alpha_max_{10.0};
  double wheel_kp_{0.05};
  // 速度来源：true = 位姿差分（含 ODE 约束修正造成、但不出现在连杆速度中的位姿变化），
  // false = Gazebo 连杆速度（早期插件的做法）
  bool pose_velocity_{true};
  double vel_tau_{0.005};
  bool have_prev_{false};
  double prev_t_{0.0}, prev_x_{0.0}, prev_y_{0.0}, prev_yaw_{0.0};
  double vxw_{0.0}, vyw_{0.0}, wz_{0.0};

  // 陀螺：由同一个位姿差分角速度加噪声与零偏生成（Gazebo 的 IMU 传感器读连杆角速度，
  // 与位姿变化不一致，见 updateBodyVelocity）
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::string imu_frame_{"base_link"};
  double imu_period_{0.01}, last_imu_t_{-1.0};
  double gyro_noise_{0.001}, gyro_bias_{0.0};
  std::mt19937 imu_rng_{2};
  std::normal_distribution<double> imu_normal_{0.0, 1.0};
};

}  // namespace scr_base_control

#endif  // SCR_BASE_CONTROL__MECANUM_BASE_SYSTEM_HPP_
