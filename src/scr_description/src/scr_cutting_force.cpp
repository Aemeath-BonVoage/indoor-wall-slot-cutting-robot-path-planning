// 切割力插件：按切割力模型在刀具（或底盘）上施加力旋量
//
// 问题：Gazebo 的墙是刚体，仿真中只能空切（刀尖停在墙前），机器人不受任何切削反力。
// 本插件不让刀具与墙真实接触，而是按 include/scr_description/cutting_force_model.hpp 的
// 模型计算切削力，直接加在刀具连杆上；底盘经机械臂关节感受到这个力。
//
// 默认不启用（<enabled>false</enabled>）：不启用时不订阅话题、不连接物理更新回调，
// 对仿真没有任何影响。
//
// 切削状态的来源：
//   进给方向与速度  刀片只能沿自身平面切削：刀片方向取刀片关节（blade_joint，默认 joint4）
//                   的转角 θ，在墙面内为 (sin θ, cos θ)（θ = 0 沿 z 竖切，90° 沿 y 横切）；
//                   进给速度取刀具相对底盘的速度（位置差分 + 低通 velocity_tau）在该方向上
//                   的分量；
//   切深            <engagement>topic</engagement>：话题 cutting_force/engagement
//                   （std_msgs/Float64，米，0 = 未切入；超过 engagement_timeout 没有新消息
//                   则视为 0，timeout = 0 表示一直保持）；
//                   <engagement>gap</engagement>：由刀尖到墙面（wall_point、wall_normal，
//                   世界系）的间隙 g 推算，a = depth·clamp((dry_run_gap + plunge_ramp − g)
//                   / plunge_ramp, 0, 1)，用于空切：刀尖到达墙前 dry_run_gap 处即视为切到
//                   规定深度；
//   下刀速度        切深由话题给出时取切深的变化率（差分 + 低通，只取增加的方向）：空切时
//                   刀具的实际行程短于槽深，下刀速度不能由刀具速度得到；切深由间隙推算时
//                   取刀具相对底盘朝墙面方向（+x）的速度。
// 另有话题 cutting_force/test_wrench（geometry_msgs/Wrench，base_link 系，作用在底盘
// 原点）：直接给定外力，用于阶跃 / 斜坡扰动试验，与模型给出的力叠加。
//
// 发布 cutting_force/wrench（geometry_msgs/WrenchStamped，base_link 系，对 base_link 原点
// 取矩）：实际施加的合力旋量，供记录与对照。
//
// 几何约定：刀尖（刀片最深点）= 刀具连杆原点沿底盘 x 轴前移 tip_offset；刀片中心在刀尖
// 后方一个半径处；墙面外法向取 −x（底盘正对墙面）。

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "scr_description/cutting_force_model.hpp"

namespace scr_description
{

using ignition::math::Vector3d;

class ScrCuttingForce : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    const auto log = ros_node_->get_logger();
    if (!sdf->Get<bool>("enabled", false).first) {
      RCLCPP_INFO(log, "scr_cutting_force: disabled");
      return;
    }
    base_ = model_->GetLink(sdf->Get<std::string>("base_link", "base_link").first);
    tool_ = model_->GetLink(sdf->Get<std::string>("tool_link", "link4").first);
    if (!base_ || !tool_) {
      RCLCPP_ERROR(log, "scr_cutting_force: base or tool link not found, plugin inactive");
      return;
    }
    blade_joint_ = model_->GetJoint(sdf->Get<std::string>("blade_joint", "joint4").first);
    if (!blade_joint_) {
      RCLCPP_ERROR(log, "scr_cutting_force: blade joint not found, plugin inactive");
      return;
    }
    apply_to_base_ = sdf->Get<std::string>("apply_to", "tool").first == "base";
    gap_mode_ = sdf->Get<std::string>("engagement", "topic").first == "gap";
    tip_offset_ = sdf->Get<double>("tip_offset", 0.1594).first;
    depth_ = sdf->Get<double>("depth", 0.035).first;
    dry_run_gap_ = sdf->Get<double>("dry_run_gap", 0.010).first;
    plunge_ramp_ = std::max(1e-4, sdf->Get<double>("plunge_ramp", 0.008).first);
    wall_point_ = sdf->Get<Vector3d>("wall_point", Vector3d::Zero).first;
    wall_normal_ = sdf->Get<Vector3d>("wall_normal", Vector3d(1, 0, 0)).first.Normalized();
    engagement_timeout_ = sdf->Get<double>("engagement_timeout", 0.0).first;
    velocity_tau_ = std::max(1e-3, sdf->Get<double>("velocity_tau", 0.1).first);
    min_feed_ = sdf->Get<double>("min_feed", 0.0005).first;
    scale_ = sdf->Get<double>("scale", 1.0).first;
    max_force_ = sdf->Get<double>("max_force", 300.0).first;
    const double rate = sdf->Get<double>("publish_rate", 50.0).first;
    publish_period_ = rate > 0.0 ? 1.0 / rate : 0.0;

    blade_.diameter = sdf->Get<double>("blade_diameter", blade_.diameter).first;
    blade_.kerf_width = sdf->Get<double>("kerf_width", blade_.kerf_width).first;
    blade_.spindle_rpm = sdf->Get<double>("spindle_rpm", blade_.spindle_rpm).first;
    blade_.specific_energy = sdf->Get<double>("specific_energy", blade_.specific_energy).first;
    blade_.force_ratio = sdf->Get<double>("force_ratio", blade_.force_ratio).first;
    blade_.resultant_fraction =
      sdf->Get<double>("resultant_fraction", blade_.resultant_fraction).first;
    blade_.size_exponent = sdf->Get<double>("size_exponent", blade_.size_exponent).first;
    blade_.up_cut = sdf->Get<bool>("up_cut", true).first;
    noise_ = std::make_unique<Inhomogeneity>(
      sdf->Get<double>("inhomogeneity_cov", 0.25).first,
      sdf->Get<double>("inhomogeneity_length", 0.015).first,
      static_cast<unsigned int>(sdf->Get<int>("seed", 1).first));

    engagement_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
      "cutting_force/engagement", rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        commanded_depth_ = std::max(0.0, msg->data);
        engagement_time_ = model_->GetWorld()->SimTime().Double();
      });
    test_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
      "cutting_force/test_wrench", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Wrench::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        test_wrench_ = *msg;
      });
    wrench_pub_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "cutting_force/wrench", rclcpp::QoS(10));
    base_frame_ = base_->GetName();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScrCuttingForce::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(
      log, "scr_cutting_force: enabled, engagement by %s, force applied to the %s, "
      "specific energy %.2f J/mm^3, kerf %.1f mm",
      gap_mode_ ? "tip-to-wall gap" : "topic", apply_to_base_ ? "base" : "tool",
      blade_.specific_energy * 1e-9, blade_.kerf_width * 1e3);
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    const double dt = now - prev_time_;
    const auto base_pose = base_->WorldPose();
    const auto tool_pose = tool_->WorldPose();
    // 刀具连杆原点在 base_link 系中的位置
    const Vector3d p = base_pose.Rot().RotateVectorReverse(tool_pose.Pos() - base_pose.Pos());
    const Vector3d tip = p + Vector3d(tip_offset_, 0.0, 0.0);

    double commanded, depth;
    geometry_msgs::msg::Wrench test;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commanded = commanded_depth_;
      if (engagement_timeout_ > 0.0 && now - engagement_time_ > engagement_timeout_) {
        commanded = 0.0;
      }
      test = test_wrench_;
    }
    if (gap_mode_) {
      const Vector3d tip_world = base_pose.Pos() + base_pose.Rot().RotateVector(tip);
      const double gap = (tip_world - wall_point_).Dot(wall_normal_);
      depth = depth_ * std::clamp((dry_run_gap_ + plunge_ramp_ - gap) / plunge_ramp_, 0.0, 1.0);
    } else {
      depth = commanded;
    }

    if (!have_prev_ || dt <= 0.0) {
      have_prev_ = true;
      prev_time_ = now;
      prev_p_ = p;
      return;
    }
    // 刀具相对底盘的速度（位置差分 + 低通）；朝墙面方向的分量为下刀速度
    const double a = dt / (velocity_tau_ + dt);
    vel_ += a * ((p - prev_p_) / dt - vel_);
    depth_rate_ += a * ((depth - prev_depth_) / dt - depth_rate_);
    prev_depth_ = depth;
    const double plunge = std::max(gap_mode_ ? vel_.X() : depth_rate_, 0.0);
    const double theta = blade_joint_->Position(0);
    const Vec3 blade_dir{0.0, std::sin(theta), std::cos(theta)};
    const double feed_signed = vel_.Y() * blade_dir[1] + vel_.Z() * blade_dir[2];
    const double feed = std::abs(feed_signed);
    prev_time_ = now;
    prev_p_ = p;

    Vector3d force(0, 0, 0), torque(0, 0, 0);      // base_link 系；force 作用在刀片中心
    const Vector3d centre = tip - Vector3d(blade_.radius(), 0.0, 0.0);
    if (depth > 0.0) {
      const bool feeding = feed >= min_feed_;
      const double gain = feeding ? noise_->advance(feed * dt) : last_gain_;
      last_gain_ = gain;
      const ToolForces t = toolForces(
        blade_, depth, feeding ? feed : 0.0, plunge, scale_ * gain);
      // 未进给（下刀、停顿）时沿用上一次的进给方向符号
      if (feeding) {
        feed_sign_ = feed_signed >= 0.0 ? 1.0 : -1.0;
      }
      const Vec3 f{0.0, feed_sign_ * blade_dir[1], feed_sign_ * blade_dir[2]};
      const ToolWrench w = toolWrench(t, f, Vec3{-1.0, 0.0, 0.0});
      force.Set(w.force[0], w.force[1], w.force[2]);
      torque.Set(w.torque[0], w.torque[1], w.torque[2]);
      const double norm = force.Length();
      if (norm > max_force_) {
        force *= max_force_ / norm;
        torque *= max_force_ / norm;
      }
    }

    const auto & rot = base_pose.Rot();
    auto target = apply_to_base_ ? base_ : tool_;
    if (force.SquaredLength() > 0.0 || torque.SquaredLength() > 0.0) {
      target->AddForceAtWorldPosition(
        rot.RotateVector(force), base_pose.Pos() + rot.RotateVector(centre));
      target->AddTorque(rot.RotateVector(torque));
    }
    const Vector3d test_force(test.force.x, test.force.y, test.force.z);
    const Vector3d test_torque(test.torque.x, test.torque.y, test.torque.z);
    if (test_force.SquaredLength() > 0.0 || test_torque.SquaredLength() > 0.0) {
      base_->AddForceAtWorldPosition(rot.RotateVector(test_force), base_pose.Pos());
      base_->AddTorque(rot.RotateVector(test_torque));
    }

    if (publish_period_ > 0.0 && now - last_publish_ >= publish_period_) {
      last_publish_ = now;
      const Vector3d total_torque = torque + centre.Cross(force) + test_torque;
      geometry_msgs::msg::WrenchStamped msg;
      msg.header.stamp.sec = info.simTime.sec;
      msg.header.stamp.nanosec = info.simTime.nsec;
      msg.header.frame_id = base_frame_;
      msg.wrench.force.x = force.X() + test_force.X();
      msg.wrench.force.y = force.Y() + test_force.Y();
      msg.wrench.force.z = force.Z() + test_force.Z();
      msg.wrench.torque.x = total_torque.X();
      msg.wrench.torque.y = total_torque.Y();
      msg.wrench.torque.z = total_torque.Z();
      wrench_pub_->publish(msg);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_, tool_;
  gazebo::physics::JointPtr blade_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr engagement_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr test_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  gazebo::event::ConnectionPtr update_conn_;

  BladeParams blade_;
  std::unique_ptr<Inhomogeneity> noise_;
  bool apply_to_base_{false}, gap_mode_{false};
  double tip_offset_{0.1594}, depth_{0.035}, dry_run_gap_{0.010}, plunge_ramp_{0.008};
  Vector3d wall_point_, wall_normal_{1, 0, 0};
  double engagement_timeout_{0.0}, velocity_tau_{0.1}, min_feed_{0.0005};
  double scale_{1.0}, max_force_{300.0};
  double publish_period_{0.02}, last_publish_{-1e9};
  std::string base_frame_{"base_link"};

  std::mutex mutex_;
  double commanded_depth_{0.0}, engagement_time_{-1e9};
  geometry_msgs::msg::Wrench test_wrench_;

  bool have_prev_{false};
  double prev_time_{0.0};
  Vector3d prev_p_, vel_;
  double prev_depth_{0.0}, depth_rate_{0.0};
  double last_gain_{1.0}, feed_sign_{1.0};
};

GZ_REGISTER_MODEL_PLUGIN(ScrCuttingForce)

}  // namespace scr_description
