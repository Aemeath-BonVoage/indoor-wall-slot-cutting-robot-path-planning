// 切割力模型：参考数值（与 scr_arm_controller/test/test_cutting_force.py 相同）、
// 力的方向与不均匀性过程的统计性质.
#include <gtest/gtest.h>

#include <cmath>

#include "scr_description/cutting_force_model.hpp"

using scr_description::BladeParams;
using scr_description::Inhomogeneity;
using scr_description::ToolForces;
using scr_description::Vec3;
using scr_description::toolForces;
using scr_description::toolWrench;

namespace
{
void expectForces(
  const ToolForces & f, double ft, double fn, double feed, double wall, double couple,
  double power)
{
  EXPECT_NEAR(f.tangential, ft, 1e-5);
  EXPECT_NEAR(f.normal, fn, 1e-5);
  EXPECT_NEAR(f.feed, feed, 1e-5);
  EXPECT_NEAR(f.wall, wall, 1e-5);
  EXPECT_NEAR(f.couple, couple, 1e-5);
  EXPECT_NEAR(f.power, power, 1e-3);
}
}  // namespace

TEST(CuttingForceModel, MatchesReferenceValues)
{
  const BladeParams b;
  EXPECT_NEAR(b.cuttingSpeed(), 73.042029, 1e-5);
  // 参考工况：切削功率 = u_0·b·a·v_f = 0.4e9 × 0.005 × 0.035 × 0.005 = 350 W
  expectForces(
    toolForces(b, 0.035, 0.005), 4.791762, 19.167047, -14.84215, 13.040216, 0.359382, 350.0);
  expectForces(
    toolForces(b, 0.035, 0.002, 0.0, 1.3), 3.133167, 12.532668, -9.704768, 8.526546,
    0.234988, 228.852884);
  expectForces(
    toolForces(b, 0.020, 0.010, 0.001), 7.214409, 28.857634, -15.029967, 24.894929,
    0.541081, 526.955042);
  // 只下刀：合力沿墙面法向，切向力沿刀片平面
  expectForces(
    toolForces(b, 0.010, 0.0, 0.002), 4.261472, 17.04589, -4.261472, 17.04589, 0.31961,
    311.266595);
  BladeParams down;
  down.up_cut = false;
  expectForces(
    toolForces(down, 0.035, 0.005), 4.791762, 19.167047, -6.959443, 18.490615, -0.359382,
    350.0);
}

TEST(CuttingForceModel, NoEngagementNoForce)
{
  const BladeParams b;
  EXPECT_EQ(toolForces(b, 0.0, 0.005).normal, 0.0);
  EXPECT_EQ(toolForces(b, 0.035, 0.0).normal, 0.0);
  EXPECT_EQ(toolForces(b, 0.035, 0.0, -0.01).normal, 0.0);   // 退刀不产生切削力
}

TEST(CuttingForceModel, ForceGrowsSublinearlyWithFeed)
{
  const BladeParams b;
  const double f1 = toolForces(b, 0.035, 0.004).tangential;
  const double f2 = toolForces(b, 0.035, 0.008).tangential;
  EXPECT_NEAR(f2 / f1, std::pow(2.0, 1.0 - b.size_exponent), 1e-9);
}

TEST(CuttingForceModel, WrenchDirections)
{
  const BladeParams b;
  const ToolForces t = toolForces(b, 0.035, 0.005);
  // 横槽、沿墙向右进给：base_link 系 f = −y，n = −x
  const auto h = toolWrench(t, Vec3{0.0, -1.0, 0.0}, Vec3{-1.0, 0.0, 0.0});
  EXPECT_LT(h.force[0], 0.0);              // 把机器人推离墙面
  EXPECT_GT(h.force[1], 0.0);              // 反力与进给方向相反（向左）
  EXPECT_NEAR(h.torque[2], t.couple, 1e-12);   // 力偶绕竖直轴
  // 竖槽、向上进给：反力向下，力偶绕 y
  const auto v = toolWrench(t, Vec3{0.0, 0.0, 1.0}, Vec3{-1.0, 0.0, 0.0});
  EXPECT_LT(v.force[2], 0.0);
  EXPECT_NEAR(v.force[1], 0.0, 1e-12);
  EXPECT_NEAR(v.torque[1], t.couple, 1e-12);
}

TEST(CuttingForceModel, InhomogeneityIsStationary)
{
  Inhomogeneity g(0.25, 0.015, 7);
  double sum = 0.0, sum2 = 0.0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const double x = g.advance(0.003);
    sum += x;
    sum2 += x * x;
  }
  const double mean = sum / n;
  EXPECT_NEAR(mean, 1.0, 0.01);
  EXPECT_NEAR(std::sqrt(sum2 / n - mean * mean), 0.25, 0.01);
  Inhomogeneity off(0.0, 0.015, 7);
  EXPECT_EQ(off.advance(0.01), 1.0);
}
