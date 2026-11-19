#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "cbf_safety_driver/safety_filter.hpp"
#include "cbf_safety_interfaces/msg/safety_status.hpp"
#include "cbf_safety_interfaces/msg/zone_estimate.hpp"
#include "cbf_safety_interfaces/srv/set_safety_mode.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace cbf_safety_driver {

class CbfSafetyDriverNode final : public rclcpp::Node {
public:
  explicit CbfSafetyDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void on_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_zone(const cbf_safety_interfaces::msg::ZoneEstimate::SharedPtr msg);
  void on_joints(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_set_mode(
      const std::shared_ptr<cbf_safety_interfaces::srv::SetSafetyMode::Request> req,
      std::shared_ptr<cbf_safety_interfaces::srv::SetSafetyMode::Response> res);
  void on_control_tick();

  SafetyParams params_;
  std::unique_ptr<SafetyFilter> filter_;
  FilterInput input_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<cbf_safety_interfaces::msg::ZoneEstimate>::SharedPtr zone_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<cbf_safety_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Service<cbf_safety_interfaces::srv::SetSafetyMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cbf_safety_driver
