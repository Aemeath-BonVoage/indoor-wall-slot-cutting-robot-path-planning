// 切割力模型（纯计算，不依赖 ROS / Gazebo）.
//
// 与 scr_arm_controller/cutting_force.py 是同一个模型，两边的单元测试对照同一组参考数值。
//
// 刀片上的切向力 F_t 与法向力 F_n 由比切削能给出：
//     Q    = b·(a·v_f + c(a)·v_p)          材料去除率（进给 + 下刀），c(a) = 2·sqrt(a·(2R − a))
//     h_eq = Q / (b·v_c)                   当量切屑厚度
//     u    = g·u_0·(h_eq / h_0)^(−ε)       比切削能（尺寸效应），g 为材料不均匀性增益
//     F_t  = u·Q / v_c,   F_n = λ·F_t
// 分解到进给方向 f 与墙面外法向 n（σ = +1 逆铣，−1 顺铣；φ_r = κ·arccos(1 − a/R)）：
//     F_f = −F_n,f·sin φ_r − σ·F_t,f·cos φ_r − σ·F_t,p
//     F_w =  F_n,f·cos φ_r − σ·F_t,f·sin φ_r + F_n,p
// 切向力对刀片中心的力矩 τ = σ·F_t·R，绕 n × f。刀具受力等效为作用在刀片中心的
// F_f·f + F_w·n 加力偶 τ·(n × f)。
#ifndef SCR_DESCRIPTION__CUTTING_FORCE_MODEL_HPP_
#define SCR_DESCRIPTION__CUTTING_FORCE_MODEL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace scr_description
{

struct BladeParams
{
  double diameter{0.150};           // m
  double kerf_width{0.005};         // m，锯缝总宽
  double spindle_rpm{9300.0};
  double specific_energy{0.40e9};   // J/m³，参考工况下（烧结砖）
  double force_ratio{4.0};          // F_n / F_t
  double resultant_fraction{0.6};   // 合力作用点在接触弧上的相对位置 κ
  double size_exponent{0.25};       // ε
  double ref_depth{0.035};          // m
  double ref_feed{0.005};           // m/s
  bool up_cut{true};

  double radius() const {return 0.5 * diameter;}
  double cuttingSpeed() const {return M_PI * diameter * spindle_rpm / 60.0;}
  double refChip() const {return ref_depth * ref_feed / cuttingSpeed();}
};

struct ToolForces
{
  double tangential{0.0};   // F_t
  double normal{0.0};       // F_n
  double feed{0.0};         // F_f：沿进给方向（负 = 阻碍进给）
  double wall{0.0};         // F_w：沿墙面外法向（正 = 推离墙面）
  double couple{0.0};       // 绕 n × f 的力偶
  double power{0.0};        // W
};

inline ToolForces toolForces(
  const BladeParams & b, double depth, double feed_rate, double plunge_rate = 0.0,
  double gain = 1.0)
{
  const double r = b.radius();
  const double a = std::clamp(depth, 0.0, r);
  const double vf = std::abs(feed_rate);
  const double vp = std::max(plunge_rate, 0.0);
  const double vc = b.cuttingSpeed();
  const double q_feed = b.kerf_width * a * vf;
  const double q_plunge = b.kerf_width * 2.0 * std::sqrt(a * (2.0 * r - a)) * vp;
  const double q = q_feed + q_plunge;
  ToolForces out;
  if (q <= 0.0) {
    return out;
  }
  const double h = q / (b.kerf_width * vc);
  const double u = gain * b.specific_energy * std::pow(h / b.refChip(), -b.size_exponent);
  const double ft_feed = u * q_feed / vc;
  const double ft_plunge = u * q_plunge / vc;
  const double sigma = b.up_cut ? 1.0 : -1.0;
  const double phi = b.resultant_fraction * std::acos(1.0 - a / r);
  const double s = std::sin(phi), c = std::cos(phi);
  const double lam = b.force_ratio;
  out.tangential = ft_feed + ft_plunge;
  out.normal = lam * out.tangential;
  out.feed = -lam * ft_feed * s - sigma * ft_feed * c - sigma * ft_plunge;
  out.wall = lam * ft_feed * c - sigma * ft_feed * s + lam * ft_plunge;
  out.couple = sigma * out.tangential * r;
  out.power = out.tangential * vc;
  return out;
}

using Vec3 = std::array<double, 3>;

inline Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

struct ToolWrench
{
  Vec3 force{0.0, 0.0, 0.0};    // 作用在刀片中心
  Vec3 torque{0.0, 0.0, 0.0};   // 力偶
};

// 刀片受力 → 三维力与力偶；f 为进给方向单位向量，n 为墙面外法向单位向量（同一坐标系）
inline ToolWrench toolWrench(const ToolForces & t, const Vec3 & f, const Vec3 & n)
{
  ToolWrench w;
  const Vec3 axis = cross(n, f);
  for (size_t i = 0; i < 3; ++i) {
    w.force[i] = t.feed * f[i] + t.wall * n[i];
    w.torque[i] = t.couple * axis[i];
  }
  return w;
}

// 材料不均匀性：沿切削路程的二阶高斯–马尔可夫过程（两个相同的一阶环节串联，相关长度
// length），乘在比切削能上（均值 1、变异系数 cov）。一阶过程的样本路径处处不可导；刀片同时
// 切削整个接触弧，力的变化应当连续，因此再经一次同样长度的平滑并按平稳方差归一化。
class Inhomogeneity
{
public:
  explicit Inhomogeneity(double cov = 0.25, double length = 0.015, unsigned int seed = 1)
  : cov_(cov), length_(length), rng_(seed)
  {
    x1_ = cov_ > 0.0 ? normal_(rng_) : 0.0;
    x2_ = x1_ * std::sqrt(0.5);
  }

  // 刀具沿槽线前进 distance（m）后的增益
  double advance(double distance)
  {
    if (cov_ <= 0.0) {
      return 1.0;
    }
    const double rho = length_ > 0.0 ? std::exp(-std::abs(distance) / length_) : 0.0;
    x1_ = rho * x1_ + std::sqrt(std::max(0.0, 1.0 - rho * rho)) * normal_(rng_);
    x2_ = rho * x2_ + (1.0 - rho) * x1_;
    // 第二级的平稳方差为 (1 + ρ²) / (1 + ρ)²
    const double x = x2_ * (1.0 + rho) / std::sqrt(1.0 + rho * rho);
    return std::max(floor_, 1.0 + cov_ * x);
  }

private:
  double cov_, length_;
  double floor_{0.2};
  double x1_{0.0}, x2_{0.0};
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace scr_description

#endif  // SCR_DESCRIPTION__CUTTING_FORCE_MODEL_HPP_
