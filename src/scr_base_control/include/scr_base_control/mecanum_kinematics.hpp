// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
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

// 两次发布之间机体速度的时间平均（里程计消息的 twist）.
//
// 控制器每个周期（1 kHz）积分位姿，但每 20 ms 才发布一次里程计。若 twist 只取发布那一拍
// 的瞬时速度，速度中与发布同相位的周期分量（例如由同一里程计消息触发的定点保持指令引起的
// 速度波动）在采样后不再平均为零，twist 的积分就与位姿的变化不一致：融合 twist 的 EKF 在
// mm/s 级运动下因此偏差约 0.5 mm/s（docs/experiments/execution_issues 第 5 节）。取平均后
// twist × 发布周期 等于这段时间内（机体系下）的位移，与位姿积分一致。
class TwistAverager
{
public:
  void add(const Twist2D & v, double dt)
  {
    if (!(dt > 0.0)) {
      return;
    }
    sum_.vx += v.vx * dt;
    sum_.vy += v.vy * dt;
    sum_.wz += v.wz * dt;
    t_ += dt;
    last_ = v;
  }

  // 自上次 take() 以来的平均速度；这段时间内没有积分时返回最近一次的瞬时速度
  Twist2D take()
  {
    Twist2D out = last_;
    if (t_ > 0.0) {
      out = {sum_.vx / t_, sum_.vy / t_, sum_.wz / t_};
    }
    sum_ = Twist2D();
    t_ = 0.0;
    return out;
  }

private:
  Twist2D sum_;
  Twist2D last_;
  double t_{0.0};
};

}  // namespace scr_base_control

#endif  // SCR_BASE_CONTROL__MECANUM_KINEMATICS_HPP_
