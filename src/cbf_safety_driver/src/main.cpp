#include <memory>

#include "cbf_safety_driver/driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cbf_safety_driver::CbfSafetyDriverNode>());
  rclcpp::shutdown();
  return 0;
}
