#include "cbf_safety_driver/driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace cbf_safety_driver {

CbfSafetyDriverNode::CbfSafetyDriverNode(const rclcpp::NodeOptions& options)
    : Node("cbf_safety_driver", options) {
  const std::string mode_str = this->declare_parameter<std::string>("safety_mode", "cbf_force");
  params_.v_limit = this->declare_parameter<double>("v_limit", 0.25);
  params_.alpha = this->declare_parameter<double>("alpha", 4.0);
  params_.d_active = this->declare_parameter<double>("d_active", 0.25);
  params_.d_safe = this->declare_parameter<double>("d_safe", 0.02);
  params_.r_reach = this->declare_parameter<double>("r_reach", 0.01);
  params_.eff_mass = this->declare_parameter<double>("eff_mass", 3.5);
  params_.qdot_max = this->declare_parameter<double>("qdot_max", 2.0);
  const double rate_hz = this->declare_parameter<double>("rate_hz", 500.0);

  filter_ = make_safety_filter(parse_safety_mode(mode_str), params_);
  // Базовая заглушка Якобиана 7-DOF до подключения модели FK/MuJoCo.
  input_.jac_tcp.setZero();
  input_.jac_tcp(0, 0) = 1.0;
  input_.jac_tcp(1, 1) = 1.0;
  input_.jac_tcp(2, 2) = 1.0;

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "desired_tcp_twist", 10,
      std::bind(&CbfSafetyDriverNode::on_twist, this, std::placeholders::_1));
  zone_sub_ = this->create_subscription<cbf_safety_interfaces::msg::ZoneEstimate>(
      "zone_estimate", 10,
      std::bind(&CbfSafetyDriverNode::on_zone, this, std::placeholders::_1));
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&CbfSafetyDriverNode::on_joints, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<cbf_safety_interfaces::msg::SafetyStatus>("safety_status", 10);
  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("commands", 10);
  mode_srv_ = this->create_service<cbf_safety_interfaces::srv::SetSafetyMode>(
      "set_safety_mode",
      std::bind(&CbfSafetyDriverNode::on_set_mode, this, std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0));
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CbfSafetyDriverNode::on_control_tick, this));
  RCLCPP_INFO(this->get_logger(), "Started cbf_safety_driver in mode '%s'", mode_str.c_str());
}

void CbfSafetyDriverNode::on_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  input_.v_desired = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void CbfSafetyDriverNode::on_zone(const cbf_safety_interfaces::msg::ZoneEstimate::SharedPtr msg) {
  input_.zone_visible = msg->visible;
  input_.zone_name = msg->zone_name.empty() ? "head" : msg->zone_name;
  input_.zone_center = Eigen::Vector3d(msg->center.x, msg->center.y, msg->center.z);
  input_.zone_vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  input_.zone_radius = msg->radius;
}

void CbfSafetyDriverNode::on_joints(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Упрощённая оценка Якобиана по углам суставов до подключения FK.
  if (msg->position.size() >= 3) {
    const double q0 = msg->position[0];
    input_.jac_tcp(0, 0) = std::cos(q0);
    input_.jac_tcp(1, 0) = std::sin(q0);
  }
}

void CbfSafetyDriverNode::on_set_mode(
    const std::shared_ptr<cbf_safety_interfaces::srv::SetSafetyMode::Request> req,
    std::shared_ptr<cbf_safety_interfaces::srv::SetSafetyMode::Response> res) {
  try {
    filter_ = make_safety_filter(parse_safety_mode(req->mode), params_);
    res->accepted = true;
    res->message = "Switched safety mode to " + req->mode;
  } catch (const std::invalid_argument& err) {
    res->accepted = false;
    res->message = err.what();
  }
}

void CbfSafetyDriverNode::on_control_tick() {
  const FilterOutput out = filter_->apply(input_);

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.assign(out.qdot_ref.data(), out.qdot_ref.data() + out.qdot_ref.size());
  cmd_pub_->publish(cmd);

  cbf_safety_interfaces::msg::SafetyStatus status;
  status.header.stamp = this->now();
  status.mode = to_string(filter_->mode());
  status.intervened = out.intervened;
  status.emergency_stop = out.emergency_stop;
  status.min_barrier_residual = out.min_barrier_residual;
  status.slack = out.slack;
  status.qp_ms = out.qp_ms;
  status_pub_->publish(status);
}

}  // namespace cbf_safety_driver
