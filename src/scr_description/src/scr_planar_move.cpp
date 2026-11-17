// 基于力的平面运动插件（替代 gazebo_ros_planar_move）
//
// 问题：gazebo_ros_planar_move 调用 Model::SetLinearVel / SetAngularVel，给所有连杆设置
// 同一个线速度和角速度。机器人旋转时偏心连杆（本机机械臂）本应有额外的线速度 ω × r，
// 关节刚度越高这一矛盾越严重。实测（空旷处）：机械臂 PID 力控时实际转速只有指令的 33–37 %。
//
// 做法：只在底盘上施加力与力矩，对速度做 P 控制（类似 ROS 1 的 force_based_move）：
//     F  = M_total · k_lin · (v_cmd − v_origin)      （水平面，限幅 M_total · a_max）
//     τz = I_z    · k_ang · (ω_cmd − ω_z)           （限幅 I_z · α_max）
// 整个铰接体在关节约束下一起运动；碰撞时接触力可以抵消驱动力，不会穿墙。
// 轮子摩擦为 0，底盘在水平面内自由滑动，速度能快速收敛到指令值。
// 注意：需配合机械臂力控（gazebo_ros2_control position_pid），position 接口会每周期
// “瞬移”关节、丢掉旋转速度，使本插件同样失效。
//
// 接口与 gazebo_ros_planar_move 保持一致：订阅 cmd_vel（base_link 系 vx, vy, wz），
// 发布 odom 与 odom→base_link TF（位姿取模型世界位姿，与原插件相同）。

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Vector3.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scr_description
{

class ScrPlanarMove : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    odom_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_link").first;
    publish_tf_ = sdf->Get<bool>("publish_odom_tf", true).first;
    const double publish_rate = sdf->Get<double>("publish_rate", 50.0).first;
    publish_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;
    cmd_timeout_ = sdf->Get<double>("cmd_timeout", 0.5).first;
    k_lin_ = sdf->Get<double>("k_lin", 20.0).first;            // 1/s，速度环带宽
    k_ang_ = sdf->Get<double>("k_ang", 20.0).first;
    a_max_ = sdf->Get<double>("max_accel", 5.0).first;          // m/s²，限制驱动力
    alpha_max_ = sdf->Get<double>("max_ang_accel", 10.0).first;  // rad/s²

    base_ = model_->GetLink(base_frame_);
    if (!base_) {
      base_ = model_->GetLinks().front();
      RCLCPP_WARN(ros_node_->get_logger(), "找不到连杆 %s，使用 %s", base_frame_.c_str(),
        base_->GetName().c_str());
    }
    ComputeMassProperties();

    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = *msg;
        last_cmd_time_ = model_->GetWorld()->SimTime().Double();
      });
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    if (publish_tf_) {
      tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    }

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScrPlanarMove::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(ros_node_->get_logger(),
      "scr_planar_move 已加载（力控速度环）：总质量 %.2f kg，绕 z 转动惯量 %.3f kg·m²",
      mass_, inertia_z_);
  }

private:
  // 整机总质量与绕底盘竖直轴的转动惯量（机械臂姿态变化影响很小，加载时计算一次）
  void ComputeMassProperties()
  {
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

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool fresh = (now - last_cmd_time_) <= cmd_timeout_;
      vx = fresh ? cmd_.linear.x : 0.0;
      vy = fresh ? cmd_.linear.y : 0.0;
      wz = fresh ? cmd_.angular.z : 0.0;
    }

    const auto pose = base_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    // 指令：base_link 原点在 base_link 系下的 (vx, vy) → 世界系
    const double vx_w = vx * std::cos(yaw) - vy * std::sin(yaw);
    const double vy_w = vx * std::sin(yaw) + vy * std::cos(yaw);
    const auto v = base_->WorldLinearVel();        // base_link 原点速度（世界系）
    const auto w = base_->WorldAngularVel();

    const double f_max = mass_ * a_max_;
    double fx = mass_ * k_lin_ * (vx_w - v.X());
    double fy = mass_ * k_lin_ * (vy_w - v.Y());
    const double f_norm = std::hypot(fx, fy);
    if (f_norm > f_max) {
      fx *= f_max / f_norm;
      fy *= f_max / f_norm;
    }
    const double t_max = inertia_z_ * alpha_max_;
    const double tz = std::clamp(inertia_z_ * k_ang_ * (wz - w.Z()), -t_max, t_max);

    base_->AddForce(ignition::math::Vector3d(fx, fy, 0.0));
    base_->AddTorque(ignition::math::Vector3d(0.0, 0.0, tz));

    if (publish_period_ > 0.0 && now - last_publish_time_ >= publish_period_) {
      last_publish_time_ = now;
      Publish(info.simTime, pose, v, w);
    }
  }

  void Publish(
    const gazebo::common::Time & t, const ignition::math::Pose3d & pose,
    const ignition::math::Vector3d & v_world, const ignition::math::Vector3d & w_world)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = t.sec;
    odom.header.stamp.nanosec = t.nsec;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    // 速度：实际速度，转到 base_link 系
    const double yaw = pose.Rot().Yaw();
    odom.twist.twist.linear.x = v_world.X() * std::cos(yaw) + v_world.Y() * std::sin(yaw);
    odom.twist.twist.linear.y = -v_world.X() * std::sin(yaw) + v_world.Y() * std::cos(yaw);
    odom.twist.twist.angular.z = w_world.Z();
    odom_pub_->publish(odom);

    if (tf_pub_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose.Pos().X();
      tf.transform.translation.y = pose.Pos().Y();
      tf.transform.translation.z = pose.Pos().Z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_pub_->sendTransform(tf);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  gazebo::event::ConnectionPtr update_conn_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist cmd_;
  double last_cmd_time_{-1e9};
  double last_publish_time_{-1e9};
  double publish_period_{0.02};
  double cmd_timeout_{0.5};
  double k_lin_{20.0}, k_ang_{20.0}, a_max_{5.0}, alpha_max_{10.0};
  double mass_{1.0}, inertia_z_{1.0};
  bool publish_tf_{true};
  std::string odom_frame_, base_frame_;
};

GZ_REGISTER_MODEL_PLUGIN(ScrPlanarMove)

}  // namespace scr_description
