// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 轮式编码器的误差模型：由底盘真实运动生成“测得的”轮速.
//
// 仿真中底盘由力直接驱动（轮子摩擦为 0、只作外观），轮子不与地面真实接触。编码器读数
// 按“轮子随底盘纯滚动”的假设由真实机体速度经麦克纳姆逆运动学得到，再叠加：
//   - 轮径 / 标定误差：每个轮固定的比例误差 scale_i；
//   - 打滑：每个轮一个一阶高斯–马尔可夫过程 g_i（标准差 slip_sigma、相关时间 slip_tau）；
//   - 辊子横向打滑：横向运动时轮子多转 lateral_slip（编码器高估横向位移）；
//   - 旋转打滑：原地转向时轮子多转 yaw_slip。
//   ω_meas,i = (1 + scale_i + g_i) · IK(vx, (1 + lateral_slip)·vy, (1 + yaw_slip)·wz)_i
// 全部参数为 0 时测量值等于真值（理想里程计）。
#ifndef SCR_BASE_CONTROL__ENCODER_ERROR_MODEL_HPP_
#define SCR_BASE_CONTROL__ENCODER_ERROR_MODEL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <random>

#include "scr_base_control/mecanum_kinematics.hpp"

namespace scr_base_control
{

struct EncoderErrorParams
{
  std::array<double, 4> scale{0.0, 0.0, 0.0, 0.0};
  double slip_sigma{0.0};
  double slip_tau{0.5};       // 秒
  double lateral_slip{0.0};
  double yaw_slip{0.0};
  unsigned int seed{1};
};

class EncoderErrorModel
{
public:
  explicit EncoderErrorModel(
    const MecanumKinematics & kin = MecanumKinematics(),
    const EncoderErrorParams & p = EncoderErrorParams())
  : kin_(kin), p_(p), rng_(p.seed) {}

  // 真实机体速度 → 测得的轮速；dt 为距上次调用的时间（推进打滑过程）
  WheelSpeeds measure(const Twist2D & v_true, double dt)
  {
    const Twist2D v{v_true.vx, (1.0 + p_.lateral_slip) * v_true.vy,
      (1.0 + p_.yaw_slip) * v_true.wz};
    WheelSpeeds w = kin_.inverse(v);
    const double a = p_.slip_tau > 0.0 ? std::exp(-dt / p_.slip_tau) : 0.0;
    const double b = p_.slip_sigma * std::sqrt(std::max(0.0, 1.0 - a * a));
    for (size_t i = 0; i < 4; ++i) {
      if (p_.slip_sigma > 0.0) {
        slip_[i] = a * slip_[i] + b * normal_(rng_);
      }
      w[i] *= 1.0 + p_.scale[i] + slip_[i];
    }
    return w;
  }

  const std::array<double, 4> & slip() const {return slip_;}

private:
  MecanumKinematics kin_;
  EncoderErrorParams p_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::array<double, 4> slip_{0.0, 0.0, 0.0, 0.0};
};

}  // namespace scr_base_control

#endif  // SCR_BASE_CONTROL__ENCODER_ERROR_MODEL_HPP_
