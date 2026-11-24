// Copyright 2026 Aemeath-BonVoage
// SPDX-License-Identifier: MIT
//
// 麦克纳姆运动学、里程计积分与编码器误差模型.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "scr_base_control/encoder_error_model.hpp"
#include "scr_base_control/mecanum_kinematics.hpp"

using scr_base_control::EncoderErrorModel;
using scr_base_control::EncoderErrorParams;
using scr_base_control::MecanumKinematics;
using scr_base_control::Odometry2D;
using scr_base_control::Twist2D;

TEST(MecanumKinematics, InverseThenForwardIsIdentity)
{
  const MecanumKinematics k;
  for (const Twist2D v : {Twist2D{0.3, 0.0, 0.0}, Twist2D{0.0, -0.2, 0.0},
      Twist2D{0.0, 0.0, 0.7}, Twist2D{0.12, -0.31, 0.45}})
  {
    const auto w = k.inverse(v);
    const auto b = k.forward(w);
    EXPECT_NEAR(b.vx, v.vx, 1e-12);
    EXPECT_NEAR(b.vy, v.vy, 1e-12);
    EXPECT_NEAR(b.wz, v.wz, 1e-12);
  }
}

TEST(MecanumKinematics, MatchesVisualWheelConvention)
{
  // 与 scr_description/scripts/mecanum_controller.py 的轮速公式一致
  const MecanumKinematics k;
  const Twist2D v{0.2, 0.1, 0.3};
  const double kk = 0.29658 + 0.2469;
  const auto w = k.inverse(v);
  EXPECT_NEAR(w[0], (0.2 - 0.1 - kk * 0.3) / 0.05, 1e-12);
  EXPECT_NEAR(w[1], (0.2 + 0.1 + kk * 0.3) / 0.05, 1e-12);
  EXPECT_NEAR(w[2], (0.2 + 0.1 - kk * 0.3) / 0.05, 1e-12);
  EXPECT_NEAR(w[3], (0.2 - 0.1 + kk * 0.3) / 0.05, 1e-12);
}

TEST(Odometry2D, CircleClosesAndStrafeIsLateral)
{
  Odometry2D o;
  const double dt = 0.001, w = 0.5, v = 0.25;
  for (int i = 0; i < static_cast<int>(2 * M_PI / w / dt); ++i) {
    o.integrate({v, 0.0, w}, dt);
  }
  EXPECT_NEAR(o.x, 0.0, 2e-3);
  EXPECT_NEAR(o.y, 0.0, 2e-3);
  Odometry2D s;
  s.yaw = M_PI / 2;
  for (int i = 0; i < 1000; ++i) {
    s.integrate({0.0, 0.1, 0.0}, dt);   // 朝 +y 时向左横移 → −x
  }
  EXPECT_NEAR(s.x, -0.1, 1e-9);
  EXPECT_NEAR(s.y, 0.0, 1e-9);
}

TEST(EncoderErrorModel, ZeroErrorIsExact)
{
  EncoderErrorModel m;
  const MecanumKinematics k;
  const Twist2D v{0.1, -0.05, 0.2};
  const auto w = m.measure(v, 0.001);
  const auto ref = k.inverse(v);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(w[i], ref[i]);
  }
}

TEST(EncoderErrorModel, ScaleAndSlipBiasTheOdometry)
{
  const MecanumKinematics k;
  EncoderErrorParams p;
  p.scale = {0.01, 0.01, 0.01, 0.01};   // 轮径一致偏大 1 %：直行距离高估 1 %
  p.lateral_slip = 0.03;
  p.yaw_slip = 0.02;
  EncoderErrorModel m(k, p);
  const auto fwd = k.forward(m.measure({0.2, 0.0, 0.0}, 0.001));
  EXPECT_NEAR(fwd.vx, 0.202, 1e-12);
  const auto lat = k.forward(m.measure({0.0, 0.1, 0.0}, 0.001));
  EXPECT_NEAR(lat.vy, 0.1 * 1.03 * 1.01, 1e-12);
  const auto rot = k.forward(m.measure({0.0, 0.0, 0.5}, 0.001));
  EXPECT_NEAR(rot.wz, 0.5 * 1.02 * 1.01, 1e-12);
}

TEST(EncoderErrorModel, SlipProcessIsStationaryWithGivenSigma)
{
  EncoderErrorParams p;
  p.slip_sigma = 0.02;
  p.slip_tau = 0.5;
  p.seed = 7;
  EncoderErrorModel m(MecanumKinematics(), p);
  double sum = 0.0, sum2 = 0.0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    m.measure({0.1, 0.0, 0.0}, 0.01);
    sum += m.slip()[0];
    sum2 += m.slip()[0] * m.slip()[0];
  }
  const double mean = sum / n;
  const double sd = std::sqrt(sum2 / n - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.003);
  EXPECT_NEAR(sd, 0.02, 0.002);
}

// 发布的 twist 取两次发布之间的平均：速度中与发布同相位的周期分量不再造成偏差，
// twist × 周期 等于位移（瞬时采样会把这一分量当作常值速度）
TEST(TwistAverager, MeanMatchesDisplacementUnderPhaseLockedRipple)
{
  scr_base_control::TwistAverager avg;
  const double dt = 0.001;
  const int per = 20;                       // 1 kHz 积分，50 Hz 发布
  double disp = 0.0, sampled = 0.0, averaged = 0.0;
  for (int k = 0; k < 50 * per; ++k) {
    // 0.2 mm/s 的平均运动 + 与发布同相位的 ±1 mm/s 波动（周期 20 ms）
    const double v = 0.0002 + 0.001 * std::cos(2.0 * M_PI * (k % per) / per);
    disp += v * dt;
    avg.add({v, 0.0, 0.0}, dt);
    if (k % per == per - 1) {
      sampled += v * per * dt;              // 只取发布这一拍
      averaged += avg.take().vx * per * dt;
    }
  }
  EXPECT_NEAR(averaged, disp, 1e-12);
  EXPECT_GT(std::abs(sampled - disp), 10.0 * std::abs(averaged - disp) + 1e-5);
}

TEST(TwistAverager, EmptyIntervalReturnsLastSample)
{
  scr_base_control::TwistAverager avg;
  avg.add({0.1, -0.2, 0.3}, 0.01);
  avg.take();
  const auto v = avg.take();
  EXPECT_DOUBLE_EQ(v.vx, 0.1);
  EXPECT_DOUBLE_EQ(v.vy, -0.2);
  EXPECT_DOUBLE_EQ(v.wz, 0.3);
}
