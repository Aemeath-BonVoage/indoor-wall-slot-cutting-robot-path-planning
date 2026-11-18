// Copyright 2026 SCR Team
// SPDX-License-Identifier: Apache-2.0
//
// 麦克纳姆轮底盘运动学与里程计积分（纯计算，不依赖 ROS）.
//
// 轮序 [lf, rf, lb, rb]；底盘速度 (vx, vy, wz) 在 base_link 系；k = lx + ly
// （前后轮距之半 + 左右轮距之半）。与 scr_description/scripts/mecanum_controller.py 的
// 约定一致：
//   ω_lf = (vx − vy − k·wz) / r    ω_rf = (vx + vy + k·wz) / r
//   ω_lb = (vx + vy − k·wz) / r    ω_rb = (vx − vy + k·wz) / r
#ifndef SCR_BASE_CONTROL__MECANUM_KINEMATICS_HPP_
#define SCR_BASE_CONTROL__MECANUM_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <utility>

namespace scr_base_control
{

struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

using WheelSpeeds = std::array<double, 4>;   // rad/s，[lf, rf, lb, rb]

struct MecanumKinematics
{
  double wheel_radius{0.05};
  double k{0.29658 + 0.2469};

  WheelSpeeds inverse(const Twist2D & v) const
  {
    const double r = wheel_radius;
    return {(v.vx - v.vy - k * v.wz) / r, (v.vx + v.vy + k * v.wz) / r,
      (v.vx + v.vy - k * v.wz) / r, (v.vx - v.vy + k * v.wz) / r};
  }

  // 最小二乘正解（四个轮速、三个自由度）
  Twist2D forward(const WheelSpeeds & w) const
  {
    const double r = wheel_radius;
    return {r / 4.0 * (w[0] + w[1] + w[2] + w[3]),
      r / 4.0 * (-w[0] + w[1] + w[2] - w[3]),
      r / (4.0 * k) * (-w[0] + w[1] - w[2] + w[3])};
  }
};

// 平面里程计：以 dt 内的平均朝向把机体速度转到 odom 系（二阶 Runge–Kutta）
struct Odometry2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  void integrate(const Twist2D & v, double dt)
  {
    const double mid = yaw + 0.5 * v.wz * dt;
    const double c = std::cos(mid);
    const double s = std::sin(mid);
    x += (c * v.vx - s * v.vy) * dt;
    y += (s * v.vx + c * v.vy) * dt;
    yaw = std::remainder(yaw + v.wz * dt, 2.0 * M_PI);
  }
};

}  // namespace scr_base_control

#endif  // SCR_BASE_CONTROL__MECANUM_KINEMATICS_HPP_
